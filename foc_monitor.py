#!/usr/bin/env python3
"""
FOC 实时串口诊断工具
- 解析 telemetry 数据，解决终端换行造成的断帧问题
- 包含自动急停保护（电流>1.5A 或 vbus<10V 自动发 d），单次保护避免循环
- 实时判断 FOC 状态及解析 FAULT 标志位
- 支持发送命令 (a/e/d/+/-/s)
- 用法: python foc_monitor.py [PORT] [BAUD]
"""
import serial
import re
import sys
import threading
import time
import math

PORT = "COM11"
BAUD = 115200

if len(sys.argv) > 1:
    PORT = sys.argv[1]
if len(sys.argv) > 2:
    BAUD = int(sys.argv[2])

# ---------- 状态码 ----------
STATE_NAMES = {0: "INIT", 1: "CALIBRATE", 2: "ALIGN", 3: "READY", 4: "RUN", 5: "FAULT"}

# 故障码位定义（与 app_control.h FaultCode_t 保持一致）
FAULT_BITS = {
    0x01: "VBUS_LOW",
    0x02: "VBUS_HIGH",
    0x04: "TEMP",
    0x08: "ENCODER",
    0x10: "ADC_SAT"
}
FAULT_ENCODER = 0x08  # 用于 diagnose() 的常量
FAULT_TEMP    = 0x04

# ---------- 滑动窗口 ----------
WINDOW = 20  # 帧
history = []

# telemetry 格式补充了 FAULT,%u:
PAT = re.compile(
    r"T,(\d+),IU,([-\d.]+),IV,([-\d.]+),IW,([-\d.]+),"
    r"VBUS,([-\d.]+),M_ANG,([-\d.]+),E_ANG,([-\d.]+),ST,(\d+),FAULT,(\d+),"
    r"IWR,([-\d.]+),KVL,([-\d.]+),"
    r"VEC,(\d+),"
    r"ENC_AGE,(\d+),ENC_VAL,(\d+),"
    r"SPD_EST,([-\d.]+),SPD_MODE,(\d+),SPD_REF,([-\d.]+),IQ_REF,([-\d.]+),"
    r"ID_MEAS,([-\d.]+),IQ_MEAS,([-\d.]+),"
    r"CTRL,(\d+),POS_TGT,([-\d.]+),POS_ACT,([-\d.]+),POS_ERR,([-\d.]+),"
    r"IMU,([-\d.]+),IMU_RDY,(\d+)"  # STAB_LIM/IMU_HOME optional, search() ignores trailing
)
DQ_STAT_PAT = re.compile(
    r"DQ_STAT,ID_AVG,([-\d.]+),IQ_AVG,([-\d.]+),"
    r"ID_RMS,([-\d.]+),IQ_RMS,([-\d.]+),"
    r"IQREF_AVG,([-\d.]+),IQERR_AVG,([-\d.]+)"
)
# 故障码位定义（与 app_control.h FaultCode_t 保持一致）
FAULT_ENCODER    = 0x08
FAULT_TEMP       = 0x04
CMD_PAT = re.compile(r"\[CMD\].*")

ser = None

def parse_line(line):
    # 先尝试匹配统计帧
    m_stat = DQ_STAT_PAT.search(line)
    if m_stat:
        return {
            "type": "DQ_STAT",
            "id_avg":     float(m_stat.group(1)),
            "iq_avg":     float(m_stat.group(2)),
            "id_rms":     float(m_stat.group(3)),
            "iq_rms":     float(m_stat.group(4)),
            "iqref_avg":  float(m_stat.group(5)),
            "iqerr_avg":  float(m_stat.group(6)),
        }
    
    # 尝试匹配普通遥测帧
    m = PAT.search(line)
    if not m:
        return None
    return {
        "type":     "T_FRAME",
        "t_ms":     int(m.group(1)),
        "iu":       float(m.group(2)),
        "iv":       float(m.group(3)),
        "iw":       float(m.group(4)),
        "vbus":     float(m.group(5)),
        "m_ang":    float(m.group(6)),
        "e_ang":    float(m.group(7)),
        "st":       int(m.group(8)),
        "fault":    int(m.group(9)),
        "iw_raw":   float(m.group(10)),
        "kvl":      float(m.group(11)),
        "vec":      int(m.group(12)),   # 0=U+ 1=V+ 2=W+ 255=Normal
        "enc_age":   int(m.group(13)),
        "enc_valid": int(m.group(14)),
        "spd_est":   float(m.group(15)),
        "spd_mode":  int(m.group(16)),
        "spd_ref":   float(m.group(17)),
        "iq_ref":    float(m.group(18)),
        "id_meas":   float(m.group(19)),  # Park变换后d轴实测 [A]
        "iq_meas":   float(m.group(20)),  # Park变换后q轴实测 [A]
        # 位置控制字段（group 21-26）
        "ctrl_mode": int(m.group(21)),    # 0=SPEED 1=POSITION 2=STABILIZE
        "pos_tgt":   float(m.group(22)),  # 目标角 [deg]
        "pos_act":   float(m.group(23)),  # 实际角 [deg]
        "pos_err":   float(m.group(24)),  # 误差 [deg]
        "imu":       float(m.group(25)),  # IMU倾斜角 [deg]
        "imu_rdy":   int(m.group(26)),    # 0=IMU未就绪 1=就绪
    }

def print_fault(f_code):
    if f_code == 0: return "NONE"
    errs = [name for bit, name in FAULT_BITS.items() if (f_code & bit)]
    return "|".join(errs) if errs else str(f_code)

def diagnose(hist, auto_stopped_flag, latest_dq_stat=None):
    """基于滑动窗口输出诊断，并执行自动保护"""
    if len(hist) < 3:
        return "", auto_stopped_flag

    msgs = []
    latest = hist[-1]
    st = latest["st"]
    f_code = latest["fault"]

    # 只用 ST=RUN 帧计算电流和电压，避免 ALIGN/READY 混入
    run_hist = [h for h in hist if h.get("st") == 4] or [latest]
    iu_rms = math.sqrt(sum(h["iu"]**2 for h in run_hist) / len(run_hist))
    iv_rms = math.sqrt(sum(h["iv"]**2 for h in run_hist) / len(run_hist))
    iw_rms = math.sqrt(sum(h["iw"]**2 for h in run_hist) / len(run_hist))
    i_total = math.sqrt(iu_rms**2 + iv_rms**2 + iw_rms**2)

    vbus_min = min(h["vbus"] for h in run_hist)
    vbus_max = max(h["vbus"] for h in run_hist)

    # 自动急停保护
    if st == 4:
        if (i_total > 1.5 or vbus_min < 10.0) and not auto_stopped_flag:
            if ser is not None:
                ser.write(b'd')
            print("\n🚨 [AUTO-PROTECT] 检测到恶劣工况 (I>1.5A 或 VBUS<10V)，自动发送 'd' 停机！🚨\n")
            auto_stopped_flag = True
    else:
        # 非 RUN 状态时清除标志位
        auto_stopped_flag = False

    if len(hist) >= 5:
        dt_ms = hist[-1]["t_ms"] - hist[-5]["t_ms"]
        if dt_ms > 0:
            d_ang = hist[-1]["m_ang"] - hist[-5]["m_ang"]
            if d_ang > 180: d_ang -= 360
            if d_ang < -180: d_ang += 360
            speed_dps = d_ang / (dt_ms / 1000.0)
        else:
            speed_dps = 0
    else:
        speed_dps = 0

    state_name = STATE_NAMES.get(st, f"?{st}")
    msgs.append(f"ST={state_name}")

    if st == 4:  # RUN
        msgs.append(f"I={i_total:.2f}A")
        msgs.append(f"VBUS={vbus_min:.1f}V")
        # 速度均值：只统计 ST=RUN && SPD_MODE=1 的帧，避免被力矩/ALIGN 样本拉低
        spd_hist = [h for h in hist if h.get("st") == 4 and h.get("spd_mode", 0) == 1]
        if len(spd_hist) < 3:
            spd_hist = [latest]   # 样本不足时降级到最新帧
        spd_vals  = [h.get("spd_est", 0.0) * 57.2958 for h in spd_hist]
        spd_avg   = sum(spd_vals) / len(spd_vals)
        spd_inst  = latest.get("spd_est", 0.0) * 57.2958
        spd_mode  = latest.get("spd_mode", 0)
        spd_ref_dps = latest.get("spd_ref", 0.0) * 57.2958
        iq_ref_val  = latest.get("iq_ref", 0.0)
        id_meas_val = latest.get("id_meas", 0.0)
        iq_meas_val = latest.get("iq_meas", 0.0)
        mode_str  = "🎯速度" if spd_mode else "🔧力矩"
        msgs.append(f"Ref={spd_ref_dps:.0f}°/s|均值={spd_avg:.0f}°/s|瞬时={spd_inst:.0f}°/s|IqRef={iq_ref_val:.3f}|Iq={iq_meas_val:.3f}|Id={id_meas_val:.3f}A({mode_str})")
        if i_total < 0.40: msgs.append("✅ 零电流正常")
        elif i_total < 0.80: msgs.append("⚠️ 残余电流偏大")

        if len(hist) >= 10:
            i_recent = sum(abs(h["iu"]) + abs(h["iv"]) for h in hist[-5:]) / 5
            i_early  = sum(abs(h["iu"]) + abs(h["iv"]) for h in hist[-10:-5]) / 5
            if i_recent > i_early + 0.15:
                msgs.append("📈 发散中")
            elif i_recent < i_early - 0.15:
                msgs.append("📉 收敛中")
        if latest_dq_stat:
            msgs.append(f"[50ms统计] Id均={latest_dq_stat['id_avg']:.3f}(rms={latest_dq_stat['id_rms']:.3f}) | Iq均={latest_dq_stat['iq_avg']:.3f}(rms={latest_dq_stat['iq_rms']:.3f}) | IqRef均={latest_dq_stat['iqref_avg']:.3f} | 误差={latest_dq_stat['iqerr_avg']:.3f}")

        # 位置控制显示
        ctrl_mode = latest.get("ctrl_mode", 0)
        CTRL_NAMES = {0: "速度", 1: "🎯位置", 2: "🔮自稳"}
        if ctrl_mode in (1, 2):
            pos_tgt = latest.get("pos_tgt", 0.0)
            pos_act = latest.get("pos_act", 0.0)
            pos_err = latest.get("pos_err", 0.0)
            imu_rdy = latest.get("imu_rdy", 0)
            imu_val = latest.get("imu", 0.0)
            mode_label = CTRL_NAMES.get(ctrl_mode, "?")
            imu_str = f" | IMU={imu_val:.1f}°✅" if imu_rdy else " | ⚠️IMU离线"
            msgs.append(f"{mode_label}|TGT={pos_tgt:.1f}°|ACT={pos_act:.1f}°|ERR={pos_err:+.1f}°{imu_str}")
    
    if st in (6, 5): # READY / ALIGN
        msgs.append(f"KVL_REC={latest['kvl']:.3f}A{'✓' if abs(latest['kvl'])<0.1 else '❌'} IW={latest['iw']:.2f}")

    if st == 5:
        fstr = f"FAULT_CODE: {print_fault(f_code)}"
        # FAULT_ENCODER = 0x08（见 app_control.h）
        if f_code & FAULT_ENCODER:
            enc_age  = latest.get("enc_age", -1)
            enc_val  = latest.get("enc_valid", -1)
            fstr += f" | ENC_AGE={enc_age}ms ENC_VALID={enc_val}"
        msgs.append(fstr)
    if st != 5 and f_code != 0:
        msgs.append(f"WARN(Fault={print_fault(f_code)})")

    # KVL_REC（重构后三相电流之和，理想=0，低边重构正确时应<0.1A）
    kvl_latest = abs(hist[-1].get("kvl", 0.0))
    if kvl_latest > 0.10:
        msgs.append(f"⚡KVL_REC={kvl_latest:.3f}A(重构误差大)")
    else:
        msgs.append(f"KVL_REC={kvl_latest:.3f}A✓")

    # 固定矢量模式诊断（vec != 255 时激活）
    vec = latest.get("vec", 255)
    VEC_NAMES = {0: "U+", 1: "V+", 2: "W+"}
    if vec in VEC_NAMES:
        # 取本矢量阶段同 VEC 标签的最近 5 帧均值
        same = [h for h in hist if h.get("vec") == vec][-5:]
        if same:
            avg_iu  = sum(h["iu"]     for h in same) / len(same)
            avg_iv  = sum(h["iv"]     for h in same) / len(same)
            avg_iw  = sum(h["iw"]     for h in same) / len(same)  # meas重构W相
            avg_iwr = sum(h["iw_raw"] for h in same) / len(same)  # 原始shunt
            msgs.append(
                f"[{VEC_NAMES[vec]}] IU={avg_iu:+.3f} IV={avg_iv:+.3f} IW={avg_iw:+.3f}A"
                f"(IWR_raw={avg_iwr:+.3f})"
            )
            # 符号检查：全部用 meas 口径（重构后）
            # 驱动相应为正；另外两相应为负
            expected_pos = {0: "iu", 1: "iv", 2: "iw"}   # W+ 用重构 iw，不用 iw_raw
            pos_key  = expected_pos[vec]
            neg_keys = [k for k in ["iu", "iv", "iw"] if k != pos_key]
            # 用均值判断，比最后一帧更稳
            pos_avg  = sum(h[pos_key]  for h in same) / len(same)
            neg_avgs = [sum(h[k] for h in same) / len(same) for k in neg_keys]
            neg_ok   = all(v < -0.02 for v in neg_avgs)
            if pos_avg > 0.05 and neg_ok:
                msgs.append("✅ 重构符号正确")
            elif pos_avg < -0.05:
                msgs.append(f"❌ {VEC_NAMES[vec]} 驱动相均值为负 → 重构符号错误")
            else:
                msgs.append(f"⚠️ 驱动相均值({pos_avg:+.3f}A)<0.05 → 通电/幅值/CTRL_SD?")


    # ── IMU 一律显示，不依赖 ctrl_mode（READY 状态也要看到）──────────────────
    imu_rdy_g = latest.get("imu_rdy", 0)
    imu_val_g = latest.get("imu", 0.0)
    if imu_rdy_g:
        msgs.append(f"IMU={imu_val_g:+.2f}°✅")
    else:
        msgs.append("IMU_RDY=0⚠️")

    return " | ".join(msgs), auto_stopped_flag


def keyboard_thread():
    """键盘输入线程"""
    print("\n--- 命令: a=ALIGN e=ENABLE d=DISABLE +=Iq+0.05 -=Iq-0.05 ]=Spd+30°/s [=Spd-30°/s 0=停转 s=STATUS ---")
    print("---          P0/P30/P-30=位置  H=回零(同步IMU零点)  G=自稳  GLIM10/GLIM30=自稳限幅 ---")
    print("---          KP1.5/KD0.03/PLIM45/DZ0.5=调参  STEP20/HTEST=测试  q=退出 ---\n")
    while True:
        try:
            c = input()
            if c == 'q':
                print("[EXIT]")
                if ser: ser.close()
                sys.exit(0)
            if ser:
                # 单字符即时命令（不加\r\n，固件端立即处理）
                if c in ('a', 'e', 'd', '+', '-', 's', ']', '[', '0') and len(c) == 1:
                    ser.write(c.encode())
                    print(f"  >>> 手动发送 '{c}'")
                else:
                    # 多字符行命令加\r\n（固件端的行缓冲解析需要）
                    ser.write((c + '\r\n').encode())
                    print(f"  >>> 手动发送 '{c}'")
        except (EOFError, KeyboardInterrupt):
            if ser: ser.close()
            sys.exit(0)


def main():
    global ser
    print(f"[INFO] 尝试连接 {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口: {e}")
        sys.exit(1)

    print(f"[INFO] 连接成功. 等待数据...\n")
    t = threading.Thread(target=keyboard_thread, daemon=True)
    t.start()

    frame_count = 0
    auto_stopped = False
    
    # 帧边界锚点：用 "T,<时间戳>,IU," 而不是简单 "T,"
    # 避免 "ST,3" 和 "FAULT,0" 中的 T, 被误判为新帧头
    FRAME_HEAD = re.compile(r'(T,\d+,IU,|DQ_STAT,ID_AVG,)')
    # 按照 头 组装帧
    remnant = ""
    latest_dq_stat = None  # 记录最新的一帧 DQ统计

    while True:
        try:
            raw = ser.read(1024)
            if not raw:
                continue
            
            # 过滤换行符，全部变成单行文本
            text = raw.decode("ascii", errors="ignore").replace("\r", "").replace("\n", "")
            remnant += text
            
            # 把 [CMD] 这种插入的命令回显剥离出来
            while "[CMD]" in remnant:
                idx = remnant.find("[CMD]")
                # 找下一个真正的帧头
                m_next = FRAME_HEAD.search(remnant, idx + 1)
                end_idx = m_next.start() if m_next else len(remnant)
                cmd_line = remnant[idx:end_idx]
                print(f"  <<< {cmd_line}")
                remnant = remnant[:idx] + remnant[end_idx:]

            # 找所有帧头位置，两个相邻帧头之间就是一个完整帧
            # 注意：每轮切掉 remnant 前缀后必须重新查找，不能复用旧 matches
            while True:
                m1 = FRAME_HEAD.search(remnant)
                if not m1:
                    break
                m2 = FRAME_HEAD.search(remnant, m1.end())
                if not m2:
                    break   # 第二个帧头还没到，等待更多数据
                frame_str = remnant[m1.start():m2.start()]
                remnant   = remnant[m2.start():]   # 消耗掉第一帧，从第二帧头开始

                d = parse_line(frame_str)
                if d is None:
                    # 正则不匹配时打印原始帧（截断到 160 字符便于观察）
                    print(f"  [RAW] {frame_str[:160]}")
                    continue

                if d["type"] == "DQ_STAT":
                    latest_dq_stat = d
                    continue  # 不进 history
                
                # 下面是 T_FRAME
                history.append(d)
                if len(history) > WINDOW:
                    history.pop(0)

                frame_count += 1
                if frame_count % 5 == 0:
                    diag, auto_stopped = diagnose(history, auto_stopped, latest_dq_stat)
                    if diag:
                        ts = d["t_ms"] / 1000.0
                        print(f"[{ts:8.1f}s] {diag}")

        except (KeyboardInterrupt, serial.SerialException):
            break

    if ser:
        ser.close()
    print("\n[INFO] 已断开")


if __name__ == "__main__":
    main()
