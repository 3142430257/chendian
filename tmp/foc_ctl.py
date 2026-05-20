"""FOC 串口测试工具：抓帧 + 解析 + 命令脚本 + watchdog 自动急停

用法：
    python foc_ctl.py snapshot 5
    python foc_ctl.py run "d:0.5;a:2.8;e:0.5;S10:4;S0:3;d:0.5"
    python foc_ctl.py stop          # 强制发 d 急停
"""
import serial
import re
import sys
import time
import argparse
import json

PORT = "COM14"
BAUD = 115200

# Watchdog 阈值（一旦命中立刻发 d 并中止后续脚本）
WD_POS_DEG       = 200.0     # |POS| > 200° 急停
WD_SPD_RAD       = 15.0      # |SPD_EST| > 15 rad/s（负载大惯量大）
WD_IQ_REF_A      = 0.55      # |IQ_REF| > 0.55A（饱和警戒）
WD_FAULT_NONZERO = True

PAT = re.compile(
    r"T,(\d+),IU,([-\d.]+),IV,([-\d.]+),IW,([-\d.]+),"
    r"VBUS,([-\d.]+),M_ANG,([-\d.]+),E_ANG,([-\d.]+),ST,(\d+),FAULT,(\d+),"
    r"IWR,([-\d.]+),KVL,([-\d.]+),"
    r"VEC,(\d+),"
    r"ENC_AGE,(\d+),ENC_VAL,(\d+),"
    r".*?"
    r"SPD_EST,([-\d.]+),SPD_MODE,(\d+),SPD_REF,([-\d.]+),IQ_REF,([-\d.]+),"
    r"ID_MEAS,([-\d.]+),IQ_MEAS,([-\d.]+),"
    r"CTRL,(\d+),POS_TGT,([-\d.]+),POS_ACT,([-\d.]+),POS_ERR,([-\d.]+),"
    r"IMU,([-\d.]+),IMU_RDY,(\d+)"
)
ENC_DROP_PAT = re.compile(r"ENC_DROP,(\d+)")
ENC_BAD_PAT  = re.compile(r"ENC_BAD,(\d+)")
THETA_E_PAT  = re.compile(r"THETA_E,([-\d.]+)")
OFFSET_PAT   = re.compile(r"OFFSET,([-\d.]+)")
FINE_PAT     = re.compile(r"FINE,([-\d.]+)")
STATE_NAMES = {0:"INIT",1:"CALIB",2:"ALIGN",3:"READY",4:"RUN",5:"FAULT"}


def parse_frame(line):
    m = PAT.search(line)
    if not m:
        return None
    drop_m = ENC_DROP_PAT.search(line)
    bad_m  = ENC_BAD_PAT.search(line)
    theta_m  = THETA_E_PAT.search(line)
    offset_m = OFFSET_PAT.search(line)
    fine_m   = FINE_PAT.search(line)
    return {
        "t_ms": int(m.group(1)),
        "iu": float(m.group(2)), "iv": float(m.group(3)), "iw": float(m.group(4)),
        "vbus": float(m.group(5)),
        "m_ang": float(m.group(6)), "e_ang": float(m.group(7)),
        "st": int(m.group(8)), "fault": int(m.group(9)),
        "iwr": float(m.group(10)), "kvl": float(m.group(11)),
        "vec": int(m.group(12)),
        "enc_age": int(m.group(13)), "enc_val": int(m.group(14)),
        "spd_est": float(m.group(15)), "spd_mode": int(m.group(16)),
        "spd_ref": float(m.group(17)), "iq_ref": float(m.group(18)),
        "id_meas": float(m.group(19)), "iq_meas": float(m.group(20)),
        "ctrl": int(m.group(21)),
        "pos_tgt": float(m.group(22)), "pos_act": float(m.group(23)), "pos_err": float(m.group(24)),
        "imu": float(m.group(25)), "imu_rdy": int(m.group(26)),
        "enc_drop": int(drop_m.group(1)) if drop_m else -1,
        "enc_bad":  int(bad_m.group(1))  if bad_m  else -1,
        "theta_e": float(theta_m.group(1))  if theta_m  else 0.0,
        "offset":  float(offset_m.group(1)) if offset_m else 0.0,
        "fine":    float(fine_m.group(1))   if fine_m   else 0.0,
    }


def watchdog_check(f):
    """返回触发原因字符串，未触发返回 None"""
    if WD_FAULT_NONZERO and f["fault"] != 0:
        return f"FAULT={f['fault']}"
    if abs(f["pos_act"]) > WD_POS_DEG:
        return f"POS_ACT={f['pos_act']:+.1f}deg (limit {WD_POS_DEG})"
    if abs(f["spd_est"]) > WD_SPD_RAD:
        return f"SPD_EST={f['spd_est']:+.2f}rad/s (limit {WD_SPD_RAD})"
    if abs(f["iq_ref"]) > WD_IQ_REF_A:
        return f"IQ_REF={f['iq_ref']:+.3f}A (limit {WD_IQ_REF_A})"
    return None


HEAD = re.compile(r"T,\d+,IU,")


def collect_frames(ser, duration_s, watchdog=False):
    """抓帧 duration_s 秒；watchdog=True 时一旦命中阈值立即发 d 并返回触发原因
    返回 (frames, trip_reason or None)
    """
    t_end = time.monotonic() + duration_s
    rem = ""
    frames = []
    trip = None
    while time.monotonic() < t_end and trip is None:
        chunk = ser.read(512)
        if not chunk:
            continue
        rem += chunk.decode("ascii", errors="ignore").replace("\r","").replace("\n","")
        while True:
            m1 = HEAD.search(rem)
            if not m1: break
            m2 = HEAD.search(rem, m1.end())
            if not m2: break
            seg = rem[m1.start():m2.start()]
            rem = rem[m2.start():]
            f = parse_frame(seg)
            if f:
                frames.append(f)
                if watchdog:
                    reason = watchdog_check(f)
                    if reason:
                        # 立即急停（连发 3 次保险）
                        for _ in range(3):
                            ser.write(b'd')
                            time.sleep(0.005)
                        trip = reason
                        print(f"  [WATCHDOG TRIPPED] {reason} @ t={f['t_ms']}ms POS={f['pos_act']:+.2f}deg SPD={f['spd_est']*57.3:+.1f}dps IQ={f['iq_ref']:+.3f}A")
                        # 抓 0.5s 后帧观察衰减
                        post_end = time.monotonic() + 0.5
                        while time.monotonic() < post_end:
                            chunk2 = ser.read(512)
                            if chunk2:
                                rem += chunk2.decode("ascii", errors="ignore").replace("\r","").replace("\n","")
                                while True:
                                    n1 = HEAD.search(rem)
                                    if not n1: break
                                    n2 = HEAD.search(rem, n1.end())
                                    if not n2: break
                                    seg2 = rem[n1.start():n2.start()]
                                    rem = rem[n2.start():]
                                    f2 = parse_frame(seg2)
                                    if f2: frames.append(f2)
                        return frames, trip
    return frames, trip


def summarize(frames, label=""):
    if not frames:
        print(f"[{label}] NO FRAMES")
        return None
    f0, fl = frames[0], frames[-1]
    pos_min = min(f["pos_act"] for f in frames)
    pos_max = max(f["pos_act"] for f in frames)
    spd_dps_min = min(f["spd_est"] for f in frames) * 57.2958
    spd_dps_max = max(f["spd_est"] for f in frames) * 57.2958
    spd_abs_max = max(abs(f["spd_est"]) for f in frames) * 57.2958
    iq_max = max(abs(f["iq_ref"]) for f in frames)
    iq_meas_max = max(abs(f["iq_meas"]) for f in frames)
    enc_age_max = max(f["enc_age"] for f in frames)
    enc_drop_delta = fl["enc_drop"] - f0["enc_drop"] if f0["enc_drop"] >= 0 else -1
    enc_bad_delta  = fl["enc_bad"]  - f0["enc_bad"]  if f0["enc_bad"]  >= 0 else -1
    fault_set = sorted({f["fault"] for f in frames if f["fault"] != 0})
    st_seq = []
    for f in frames:
        if not st_seq or st_seq[-1] != f["st"]:
            st_seq.append(f["st"])
    print(f"[{label}] frames={len(frames)} dur={fl['t_ms']-f0['t_ms']}ms")
    print(f"  ST seq        : {[STATE_NAMES.get(s,s) for s in st_seq]}")
    print(f"  FAULT bits    : {fault_set if fault_set else 'none'}")
    print(f"  VBUS          : {f0['vbus']:.2f} -> {fl['vbus']:.2f}V")
    print(f"  POS_ACT       : {pos_min:+7.2f}° .. {pos_max:+7.2f}° (final={fl['pos_act']:+.2f})")
    print(f"  SPD_EST       : {spd_dps_min:+7.1f}°/s .. {spd_dps_max:+7.1f}°/s (|max|={spd_abs_max:.1f})")
    print(f"  SPD_REF final : {fl['spd_ref']*57.2958:+.1f}°/s  IQ_REF max |{iq_max:.3f}|A")
    print(f"  IQ_MEAS max   : |{iq_meas_max:.3f}|A")
    print(f"  ENC_AGE max   : {enc_age_max}ms  drop+={enc_drop_delta}  bad+={enc_bad_delta}")
    print(f"  CTRL_MODE     : {fl['ctrl']}  SPD_MODE={fl['spd_mode']}")


# 固件 cmd_feed_byte 的 IMMEDIATE 白名单（不需要回车，固件立即处理）
FW_IMMEDIATE_CHARS = set("edas+-][0EOXL")


def write_cmd(ser, cmd, immediate=None):
    """immediate 自动判定：单字符且在固件白名单 → 不加回车
    其他一律加 \\r\\n
    """
    if immediate is None:
        immediate = (len(cmd) == 1 and cmd in FW_IMMEDIATE_CHARS)
    if immediate:
        ser.write(cmd.encode())
    else:
        ser.write((cmd + "\r\n").encode())
    print(f"  >>> SENT: {cmd!r}{'(imm)' if immediate else '(line)'}")


def open_serial():
    return serial.Serial(PORT, BAUD, timeout=0.05)


def cmd_snapshot(args):
    dur = float(args.duration)
    with open_serial() as ser:
        ser.reset_input_buffer()
        time.sleep(0.1)
        frames, trip = collect_frames(ser, dur, watchdog=False)  # snapshot 不联动 d
        summarize(frames, "snapshot")


def cmd_stop(args):
    with open_serial() as ser:
        ser.write(b'd')
        print("[stop] sent d")


def cmd_run_script(args):
    sequence = [s.strip() for s in args.script.split(";") if s.strip()]
    print(f"[script] sequence: {sequence}")
    print(f"[script] watchdog: POS>{WD_POS_DEG}° |SPD|>{WD_SPD_RAD}rps |IQ|>{WD_IQ_REF_A}A FAULT!=0")
    all_frames = []
    aborted = False
    with open_serial() as ser:
        ser.reset_input_buffer()
        time.sleep(0.1)
        for step in sequence:
            if aborted:
                print(f"[skip] {step}")
                continue
            if ":" in step:
                cmd_str, dur = step.split(":")
                dur = float(dur)
            else:
                cmd_str = step
                dur = 0.5
            cmd_str = cmd_str.strip()
            write_cmd(ser, cmd_str)
            # 'd' 命令不开 watchdog（停机后位置可能仍超限）
            # 'a' 命令期间 ALIGN 内部会自动测试方向（注入Iq让电机短暂移动），
            # 此时不能让外部 watchdog 中断，否则 ALIGN 序列被打断
            wd = (cmd_str != 'd' and cmd_str != 'a')
            fs, trip = collect_frames(ser, dur, watchdog=wd)
            summarize(fs, f"AFTER_{cmd_str}_{dur}s")
            all_frames.extend([(cmd_str, f) for f in fs])
            if trip:
                print(f"  [ABORT] {trip}")
                aborted = True
        # 收尾 d 兜底（即便已 abort 也再发一次）
        if aborted:
            ser.write(b'd')
            time.sleep(0.3)
            print("[abort] final 'd' sent")
    if args.log:
        with open(args.log, "w") as f:
            for tag, fr in all_frames:
                f.write(json.dumps({"tag": tag, **fr}) + "\n")
        print(f"[log] wrote {args.log}")


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    p_snap = sub.add_parser("snapshot")
    p_snap.add_argument("duration", type=float, default=5.0, nargs="?")
    p_snap.set_defaults(func=cmd_snapshot)
    p_run = sub.add_parser("run")
    p_run.add_argument("script")
    p_run.add_argument("--log", default=None)
    p_run.set_defaults(func=cmd_run_script)
    p_stop = sub.add_parser("stop")
    p_stop.set_defaults(func=cmd_stop)
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
