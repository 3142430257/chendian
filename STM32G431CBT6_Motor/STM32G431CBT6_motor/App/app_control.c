/**
 * @file    app_control.c
 * @brief   系统状态机实现
 *
 * 状态转换：
 *   INIT → CALIBRATE（外设初始化完成）
 *   CALIBRATE → READY（零偏采集完成，编码器有效）
 *   READY → RUN（app_control_enable() 被调用）
 *   RUN/READY → FAULT（软件故障检测触发）
 *   FAULT → READY（手动复位，需先清除故障源）
 *
 * 软件保护（主循环 ~1kHz 检测）：
 *   VBUS < 8V 或 > 26V → FAULT_VBUS_LOW / FAULT_VBUS_HIGH
 *   温度超限             → FAULT_TEMP
 *   编码器信号丢失       → FAULT_ENCODER
 *   任一 ADC 通道饱和    → FAULT_ADC_SATURATE
 */

#include "app_control.h"
#include "board_config.h"
#include "foc_interface.h"
#include "encoder_if.h"
#include "mpu6050.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>   /* logf(), fmaxf(), fminf() */

/* ============================================================
 * 速度环参数（调参修改此处）
 * ============================================================ */
#define KP_SPD          (0.02f)     /* A/(rad/s)：Kp小防振荡，靠Ki提供启动力矩 */
#define KI_SPD          (0.40f)     /* A/rad：@90°/s误差，约0.5s积分到0.3A启动 */
#define IQ_MAX_A        (0.45f)    /* A，速度环安全上限（短时验证，热测后定版）*/
#define OMEGA_SLEW_RATE (2.0f)     /* rad/s²，目标速度斜坡速率 */
#define SPEED_EST_ALPHA (0.15f)    /* IIR alpha：fc≈10Hz，更强滤波减少噪声 */
#define SPEED_EST_DT_MS (20U)      /* 速度估算最小间隔20ms(50Hz)，减少差分噪声 */
#define TWO_PI          (6.28318530f)

/* 外部 GPIO（CubeMX 在 gpio.c 生成）*/
extern void MX_GPIO_Init(void);  /* 仅供参考，实际已由 main 调用 */

/* ============================================================
 * 内部状态
 * ============================================================ */
static AppState_t s_state = STATE_INIT;
static uint8_t    s_fault = FAULT_NONE;
static volatile float s_iq_ref_a = 0.0f;  /* q轴目标电流 [A]，主循环写、ISR 读 */

/* --- 速度环状态 --- */
static float    s_theta_prev_rad = 0.0f;   /* 上次机械角 [rad] */
static uint32_t s_est_prev_ms    = 0;      /* 上次估算时刻 [ms] */
static float    s_omega_est_rps  = 0.0f;   /* 滤波后速度 [rad/s] */
static float    s_omega_ref_rps  = 0.0f;   /* 经斜坡后的速度目标 [rad/s] */
static float    s_omega_ref_cmd  = 0.0f;   /* 用户目标（未经斜坡）[rad/s] */
static float    s_speed_integ    = 0.0f;   /* PI 积分项 */
static bool     s_speed_mode     = false;  /* true=速度模式, false=力矩模式 */

/* --- 位置环状态 --- */
static CtrlMode_t s_ctrl_mode           = CTRL_SPEED;
static float      s_pos_home_offset_deg = 0.0f;   /* 回零时的编码器角 [deg] */
static float      s_pos_target_deg      = 0.0f;   /* 位置目标 [deg]，已clamp到±90° */
static float      s_vel_filt_dps        = 0.0f;   /* D项速度滤波 [°/s]，MOTOR_SIGN坐标系 */
static float      s_kp_pos              = 1.0f;   /* 位置P增益 [°/s 每 °误差]（实测稳定值）*/
static float      s_kd_pos              = 0.02f;  /* 位置D增益（用滤波速度，实测稳定值）*/
static float      s_pos_spd_lim_dps     = 45.0f;  /* 位置环输出速度限幅 [°/s]（实测稳定值）*/
static float      s_pos_deadband_deg    = 0.5f;   /* 位置死区 [deg] */
static float      s_stab_lim_deg        = 10.0f;  /* 自稳目标软限幅 [°]，默认10°，第G命令前可用 GLIM<val> 调大 */
static float      s_imu_home_offset_deg  = 0.0f;  /* IMU pitch 安装偏置，H 命令时刷新 */

/* 零偏采样窗口 */
#define CALIB_SAMPLES   (256U)
static uint32_t s_calib_sum_iu = 0;
static uint32_t s_calib_sum_iv = 0;
static uint32_t s_calib_sum_iw = 0;
static uint16_t s_calib_cnt    = 0;
static uint32_t last_frame_cnt = 0;

/* 前向声明（定义在文件尾部）*/
static void align_update(const FocMeasurement_t *m);

/* ── 位置环抖助函数 ── */

/* 角度迹化到 -180∼+180° */
static float wrap180(float x)
{
    while (x >  180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

/* 获取相对 home 的实际角（带 MOTOR_SIGN）[deg] */
static float get_pos_actual_deg(void)
{
    float raw_deg = encoder_get_angle_rad() * (180.0f / 3.14159265f);
    return MOTOR_SIGN * wrap180(raw_deg - s_pos_home_offset_deg);
}

/* ---------------------------------------------------------- */
static void ctrl_sd_set(bool enable)
{
    GPIO_PinState pin = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, pin);
}

/* ============================================================
 * 软件故障检测
 * ============================================================ */
static uint8_t check_faults(const FocMeasurement_t *m, const uint16_t *adc_raw)
{
    uint8_t f = FAULT_NONE;

    /* VBUS */
    if (m->vbus_v < VBUS_MIN_V)  f |= FAULT_VBUS_LOW;
    if (m->vbus_v > VBUS_MAX_V)  f |= FAULT_VBUS_HIGH;

    /* 编码器 */
    if (!encoder_is_valid())      f |= FAULT_ENCODER;

    /* ADC 饱和（任意通道）*/
    for (uint8_t i = 0; i < FOC_ADC_CH_COUNT; i++) {
        if (adc_raw[i] == 0U || adc_raw[i] == 4095U) {
            f |= FAULT_ADC_SATURATE;
            break;
        }
    }

    /* 温度：NTC B 参数法换算（NCP18XH103，10K@25C，B=3380K）
     * 分压电路：VCC3.3 → NTC(Rt) → VTEMP → 4.7k → GND
     * 正确公式：Rt = 4.7 * (Vcc - Vadc) / Vadc
     * 1/T = 1/T0 + ln(Rt/R0) / B  (T0=298.15K, R0=10kOhm) */
    {
        float vadc_t = ADC_TO_VOLT(adc_raw[FOC_ADC_IDX_VTEMP]);
        if (vadc_t > 0.01f) {   /* 防止除零（NTC 短路时 Vadc ≈ 0）*/
            float rt_k   = VTEMP_PULLUP_K * (ADC_REF_V - vadc_t) / vadc_t;
            float ln_r   = logf(rt_k / VTEMP_NTC_25C_K);
            float temp_k = 1.0f / (1.0f / 298.15f + ln_r / VTEMP_NTC_B);
            float temp_c = temp_k - 273.15f;
            if (temp_c > VTEMP_TEMP_LIMIT_C) {
                f |= FAULT_TEMP;
            }
        } else {
            /* Vadc ≈ 0：NTC 短路（波在上极高温）——视为温度异常 */
            f |= FAULT_TEMP;
        }
    }

    return f;
}

/* ============================================================
 * 公共接口
 * ============================================================ */
void app_control_init(void)
{
    s_state      = STATE_INIT;
    s_fault      = FAULT_NONE;
    s_iq_ref_a   = 0.0f;      /* 确保上电时 Iq 指令为零 */
    s_calib_cnt  = 0;
    s_calib_sum_iu = s_calib_sum_iv = s_calib_sum_iw = 0;
    last_frame_cnt = foc_interface_get_frame_count();

    ctrl_sd_set(false);   /* 初始禁止驱动板 */
    s_state = STATE_CALIBRATE;
}

void app_control_update(void)
{
    FocMeasurement_t meas;
    foc_interface_get_measurement(&meas);

    switch (s_state)
    {
    /* ---- 零偏校准 ---- */
    case STATE_CALIBRATE:
    {
        uint32_t current_frame = foc_interface_get_frame_count();
        if (current_frame != last_frame_cnt) {
            last_frame_cnt = current_frame;

            /* 驱动板未使能，累加真实 ADC 原始值求均值作为零偏 */
            uint16_t raw_now[FOC_ADC_CH_COUNT];
            foc_interface_get_adc_raw(raw_now);
            s_calib_sum_iu += raw_now[FOC_ADC_IDX_IU];
            s_calib_sum_iv += raw_now[FOC_ADC_IDX_IV];
            s_calib_sum_iw += raw_now[FOC_ADC_IDX_IW];
            s_calib_cnt++;

            if (s_calib_cnt >= CALIB_SAMPLES)
            {
                /* 四舍五入平均值写入各相零偏 */
                iu_offset_raw = (uint16_t)((s_calib_sum_iu + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);
                iv_offset_raw = (uint16_t)((s_calib_sum_iv + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);
                iw_offset_raw = (uint16_t)((s_calib_sum_iw + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);
                s_state = STATE_READY;
            }
        }
        break;
    }

    /* ---- 编码器对齐（串口 'a' 命令触发）---- */
    case STATE_ALIGN:
        align_update(&meas);
        break;

    /* ---- 就绪 ---- */
    case STATE_READY:
        if (meas.vbus_v < VBUS_MIN_V || !encoder_is_valid()) {
            /* 仅等待，不进故障（可能刚上电，VBUS 还未稳定）*/
        }
        break;

    /* ---- 运行中 ---- */
    case STATE_RUN:
    {
        /* 获取真实 ADC 原始快照，用于 IU/IV/IW 饱和检测 */
        uint16_t adc_raw[FOC_ADC_CH_COUNT];
        foc_interface_get_adc_raw(adc_raw);
        uint8_t f_now = check_faults(&meas, adc_raw);

        /* 去抖：同一种故障连续 FAULT_DEBOUNCE_CNT 帧才触发
         * 不同类型交替出现时重置计数器，防止误累加 */
        #define FAULT_DEBOUNCE_CNT  (500U)
        static uint16_t fault_consec = 0;
        static uint8_t  last_fault   = FAULT_NONE;
        if (f_now != FAULT_NONE) {
            if (f_now == last_fault) {
                fault_consec++;
            } else {
                /* 故障类型变化，重新开始计数 */
                last_fault   = f_now;
                fault_consec = 1;
            }
            if (fault_consec >= FAULT_DEBOUNCE_CNT) {
                s_fault = f_now;
                s_iq_ref_a = 0.0f;
                ctrl_sd_set(false);
                s_state = STATE_FAULT;
                fault_consec = 0;
                last_fault   = FAULT_NONE;
            }
        } else {
            fault_consec = 0;
            last_fault   = FAULT_NONE;
        }
        /* ---- 速度估算 + 位置环 + 速度 PI ---- */
        {
            float   theta_now = encoder_get_angle_rad();
            uint32_t now_ms   = HAL_GetTick();
            uint32_t dt_ms    = now_ms - s_est_prev_ms;

            if (dt_ms >= SPEED_EST_DT_MS) {
                float dt_s    = dt_ms * 1e-3f;
                float d_theta = theta_now - s_theta_prev_rad;
                /* 角度展开（防止 0→2π 跳变） */
                if (d_theta >  3.14159265f) d_theta -= TWO_PI;
                if (d_theta < -3.14159265f) d_theta += TWO_PI;

                float omega_raw = d_theta / dt_s;
                if (omega_raw >  30.0f) omega_raw =  30.0f;
                if (omega_raw < -30.0f) omega_raw = -30.0f;
                s_omega_est_rps = SPEED_EST_ALPHA * omega_raw
                                + (1.0f - SPEED_EST_ALPHA) * s_omega_est_rps;

                s_theta_prev_rad = theta_now;
                s_est_prev_ms    = now_ms;

                /* ---- D 项速度滤波（°/s，与 pos_actual 坐标一致）---- */
                float omega_dps = s_omega_est_rps * (180.0f / 3.14159265f);
                s_vel_filt_dps  = 0.85f * s_vel_filt_dps + 0.15f * omega_dps;
                float vel_pos   = MOTOR_SIGN * s_vel_filt_dps;

                /* ---- 位置环（CTRL_POSITION）---- */
                if (s_ctrl_mode == CTRL_POSITION) {
                    float pos_actual = get_pos_actual_deg();

                    /* ── wrap穿越保护（最高优先级）──────────────────────────────
                     * wrap180 在 ±180° 处突变，pos_err 会跳 360°，驱动电机
                     * 再转一圈。保护条件 >170° 对正常 ±90° 云台绝不触发。
                     * 触发时：omega_ref=0 + 清积分，不 break，让速度PI本拍
                     * 以 omega_ref=0 输出制动 Iq，s_iq_ref_a 立即更新，无残余扭矩 */
                    if (fabsf(pos_actual) > 170.0f) {
                        s_omega_ref_rps = 0.0f;
                        s_speed_integ   = 0.0f;
                        /* 不写 s_iq_ref_a，由下方速度PI（omega_ref=0）主动制动 */
                    } else {
                        /* ── 正常位置环 ──────────────────────────────────────*/
                        float target      = s_pos_target_deg;
                        float spd_lim_now = s_pos_spd_lim_dps;
                        if (fabsf(pos_actual) > POS_WARN_LIM_DEG) {
                            spd_lim_now = s_pos_spd_lim_dps * 0.3f;
                        }
                        float pos_err = target - pos_actual;

                        /* pos_err 过零清速度积分 ─────────────────────────────
                         * 电机越过目标点（pos_err 翻号）时，位置环已在反向制动。
                         * 此时速度PI里还保留着趋近阶段积累的顺向积分，会产生
                         * "二次冲击"：越过目标点后反向更猛。在翻号那一拍清零积
                         * 分，让速度PI从零开始重建，大幅抑制双超调振荡。 */
                        static float s_prev_pos_err = 0.0f;
                        if (s_prev_pos_err * pos_err < 0.0f && fabsf(pos_err) > 0.5f) {
                            s_speed_integ = 0.0f;
                        }
                        s_prev_pos_err = pos_err;

                        /* 接近目标时限制积分上限（防 stiction 积分过量）──────────
                         * 仅在最后 3° 内才限幅：更远处需要完整积分克服摩擦。
                         * 3° 内限幅 0.15A，让冲击可控，过零清积分再收回目标点。*/
                        if (fabsf(pos_err) < 3.0f) {
                            if (s_speed_integ >  0.15f) s_speed_integ =  0.15f;
                            if (s_speed_integ < -0.15f) s_speed_integ = -0.15f;
                        }

                        float spd_cmd;
                        if (fabsf(pos_err) < s_pos_deadband_deg) {
                            spd_cmd = 0.0f;
                        } else {
                            spd_cmd = s_kp_pos * pos_err - s_kd_pos * vel_pos;
                        }
                        if (pos_actual >  POS_HARD_LIM_DEG && spd_cmd > 0.0f) spd_cmd = 0.0f;
                        if (pos_actual < -POS_HARD_LIM_DEG && spd_cmd < 0.0f) spd_cmd = 0.0f;
                        if (spd_cmd >  spd_lim_now) spd_cmd =  spd_lim_now;
                        if (spd_cmd < -spd_lim_now) spd_cmd = -spd_lim_now;
                        s_omega_ref_rps = spd_cmd * (3.14159265f / 180.0f);
                    }
                } else if (s_ctrl_mode == CTRL_STABILIZE) {
                    /* 自稳模式：目标角 = -IMU_SIGN × pitch
                     * 复用位置 PD 框架；pos_actual 在本分支各自取，不跨作用域 */
                    float pos_actual = get_pos_actual_deg();   /* ← 修复：本分支独立声明 */

                    if (mpu6050_is_ready()) {
                        float tgt = -IMU_SIGN * (mpu6050_get_pitch_deg() - s_imu_home_offset_deg);
                        float lim = s_stab_lim_deg;
                        if (tgt >  lim) tgt =  lim;
                        if (tgt < -lim) tgt = -lim;
                        s_pos_target_deg = tgt;
                    }
                    /* IMU 未就绪时保持上一拍目标不变，位置环仍正常工作 */
                    {
                        float target      = s_pos_target_deg;
                        float spd_lim_now = s_pos_spd_lim_dps;
                        float pos_err     = target - pos_actual;
                        float spd_cmd;
                        if (fabsf(pos_err) < s_pos_deadband_deg) {
                            spd_cmd = 0.0f;
                        } else {
                            spd_cmd = s_kp_pos * pos_err - s_kd_pos * vel_pos;
                        }
                        if (pos_actual >  POS_HARD_LIM_DEG && spd_cmd > 0.0f) spd_cmd = 0.0f;
                        if (pos_actual < -POS_HARD_LIM_DEG && spd_cmd < 0.0f) spd_cmd = 0.0f;
                        if (spd_cmd >  spd_lim_now) spd_cmd =  spd_lim_now;
                        if (spd_cmd < -spd_lim_now) spd_cmd = -spd_lim_now;
                        s_omega_ref_rps = spd_cmd * (3.14159265f / 180.0f);
                    }
                }


                /* ---- 速度 PI ---- */
                bool run_spd_pi = (s_ctrl_mode == CTRL_POSITION)   ||
                                  (s_ctrl_mode == CTRL_STABILIZE)   ||  /* 主动刺使归零 */
                                  (s_ctrl_mode == CTRL_SPEED && s_speed_mode);
                if (run_spd_pi) {
                    if (s_ctrl_mode == CTRL_SPEED) {
                        /* 斜坡仅在速度模式运行，位置模式由位置环直接设 omega_ref */
                        float delta = s_omega_ref_cmd - s_omega_ref_rps;
                        float step  = OMEGA_SLEW_RATE * dt_s;
                        if      (delta >  step) s_omega_ref_rps += step;
                        else if (delta < -step) s_omega_ref_rps -= step;
                        else                    s_omega_ref_rps  = s_omega_ref_cmd;
                    }
                    float err    = s_omega_ref_rps - s_omega_est_rps;
                    float iq_cmd = KP_SPD * err + s_speed_integ;
                    float iq_clamped = fmaxf(-IQ_MAX_A, fminf(IQ_MAX_A, iq_cmd));
                    if (iq_clamped == iq_cmd) {
                        s_speed_integ += KI_SPD * err * dt_s;
                    }
                    s_iq_ref_a = iq_clamped;
                }
            }
        }
        break;
    }

    /* ---- 故障 ---- */
    case STATE_FAULT:
        ctrl_sd_set(false);
        /* 等待手动复位（app_control_disable + app_control_enable）*/
        break;

    default:
        break;
    }
}

/**
 * @brief  请求使能驱动板并进入 RUN 态
 * @return true  = 使能成功
 *         false = 前提条件不满足（VBUS/编码器异常），拒绝使能
 */
bool app_control_enable(void)
{
    if (s_state != STATE_READY) {
        return false;
    }

    /* --- 使能前提条件检查 ---
     * 前提不满足时保持 STATE_READY 返回 false，不进故障态。
     * 这不是系统故障，只是操作时机过早，调用方重试即可。*/
    FocMeasurement_t meas;
    foc_interface_get_measurement(&meas);

    if (meas.vbus_v < VBUS_MIN_V || meas.vbus_v > VBUS_MAX_V) {
        return false;   /* VBUS 未就绪，保持 READY */
    }
    if (!encoder_is_valid()) {
        return false;   /* 编码器未就绪，保持 READY */
    }

    /* 前提满足，使能 */
    s_fault = FAULT_NONE;
    /* 速度估算器基准重置：防止旧角度/旧时间产生一帧错误速度 */
    s_theta_prev_rad = encoder_get_angle_rad();
    s_est_prev_ms    = HAL_GetTick();
    s_omega_est_rps  = 0.0f;
    /* 位置环滤波器初始化 */
    s_vel_filt_dps   = 0.0f;
    ctrl_sd_set(true);
    s_state = STATE_RUN;
    return true;
}

void app_control_disable(void)
{
    s_iq_ref_a = 0.0f;
    /* 速度环清零 */
    s_speed_mode    = false;
    s_omega_ref_rps = 0.0f;
    s_omega_ref_cmd = 0.0f;
    s_speed_integ   = 0.0f;
    s_omega_est_rps = 0.0f;
    /* 速度估算基准重置 */
    s_theta_prev_rad = 0.0f;
    s_est_prev_ms    = 0;
    /* 位置环清零（保留 home_offset，方便重新使能后直接用 P0）*/
    s_ctrl_mode    = CTRL_SPEED;
    s_vel_filt_dps = 0.0f;
    ctrl_sd_set(false);
    if (s_state == STATE_FAULT) {
        s_fault = FAULT_NONE;
        s_state = STATE_READY;
    } else {
        s_state = STATE_READY;
    }
}

uint8_t app_control_get_state(void) { return (uint8_t)s_state; }
uint8_t app_control_get_fault(void) { return s_fault; }

/* ============================================================
 * Iq 指令接口（V3 Simulink 前置锁定）
 * ============================================================ */

/**
 * @brief  获取当前 q 轴目标电流 [A]
 * @note   仅 STATE_RUN 时返回有效值；其余状态强制返回 0.0f，
 *         防止状态机未就绪时 ISR 仍然输出非零指令。
 */
float app_control_get_iq_ref(void)
{
    if (s_state == STATE_RUN) {
        return s_iq_ref_a;
    }
    return 0.0f;
}

/**
 * @brief  设置 q 轴目标电流 [A]（速度环输出或外部调试指令）
 * @note   写操作在主循环；ISR 读 s_iq_ref_a 为 volatile float，
 *         Cortex-M4 单精度浮点写为原子操作，无需额外临界区。
 */
void app_control_set_iq_ref(float iq_ref_a)
{
    s_iq_ref_a = iq_ref_a;
}

/* ============================================================
 * ALIGN 接口（串口 'a' 命令触发，不自动上电执行）
 * ============================================================ */
#define ALIGN_HOLD_MS   (2000U)  /* 加长到2s，确保转子充分稳定后再记录 offset */
#define ALIGN_ID_V      (1.5f)   /* d 轴对齐电压 [V]，Rs=2.7Ω → I≈0.56A */
static uint32_t s_align_start_ms = 0;
static float    s_theta_offset_e = 0.0f;  /* 电角度偏移 [rad] */

bool app_control_start_align(void)
{
    if (s_state != STATE_READY) return false;
    FocMeasurement_t m;
    foc_interface_get_measurement(&m);
    if (m.vbus_v < VBUS_MIN_V || !encoder_is_valid()) return false;

    s_align_start_ms = HAL_GetTick();
    ctrl_sd_set(true);
    s_state = STATE_ALIGN;
    return true;
}

static void align_update(const FocMeasurement_t *m)
{
    uint32_t now = HAL_GetTick();

    /* 超时 / 过流 / 编码器失效 → 立即停止
     * 注意：CTRL_SD 上电瞬间 EMI 可能让 encoder 短暂无效，
     * 前 80ms 为启动保护期，不校验编码器避免误退出。*/
    bool startup  = ((now - s_align_start_ms) < 80U);
    bool overtime = ((now - s_align_start_ms) > (ALIGN_HOLD_MS + 500U));
    bool overcurr = (m->iu_a > 1.5f || m->iu_a < -1.5f ||
                     m->iv_a > 1.5f || m->iv_a < -1.5f ||
                     m->iw_a > 1.5f || m->iw_a < -1.5f);

    if (overtime || overcurr || (!startup && !encoder_is_valid())) {
        ctrl_sd_set(false);
        s_state = STATE_READY;
        return;
    }

    if ((now - s_align_start_ms) >= ALIGN_HOLD_MS) {
        /* 转子已稳定在 theta_e=0，记录电角度偏移
         * 编码器方向反转：offset = 2π - (pp * theta_m) mod 2π
         * 保证与 RUN 分支 elec_angle = (2π - raw_e) - offset 一致 */
        float theta_m_now = encoder_get_angle_rad();
        float raw_e = theta_m_now * (float)MOTOR_POLE_PAIRS;
        while (raw_e >= 2.0f * 3.14159265f) raw_e -= 2.0f * 3.14159265f;
        while (raw_e <  0.0f)               raw_e += 2.0f * 3.14159265f;
        s_theta_offset_e = raw_e;  /* 测试版：直接记录raw_e作offset，配合RUN里的 raw_e - offset */
        /* 原来是(2π-raw_e)，若恢复则需同步改回 RUN 里的公式 */
        while (s_theta_offset_e >= 2.0f * 3.14159265f) s_theta_offset_e -= 2.0f * 3.14159265f;
        while (s_theta_offset_e <  0.0f)               s_theta_offset_e += 2.0f * 3.14159265f;
        ctrl_sd_set(false);
        s_state = STATE_READY;
    }
}

float app_control_get_align_id_ref(void)
{
    return (s_state == STATE_ALIGN) ? ALIGN_ID_V : 0.0f;
}

float app_control_get_theta_offset(void)
{
    return s_theta_offset_e;
}

/* ============================================================
 * 速度环公共接口
 * ============================================================ */

void app_control_set_speed_mode(bool enable)
{
    if (enable && !s_speed_mode) {
        /* 进入速度模式：清积分+基准重置+斜坡从当前速度开始 */
        s_speed_mode     = true;
        s_speed_integ    = 0.0f;
        s_theta_prev_rad = encoder_get_angle_rad();
        s_est_prev_ms    = HAL_GetTick();
        s_omega_ref_rps  = s_omega_est_rps;
        s_omega_ref_cmd  = s_omega_ref_rps;
        /* 进入速度模式时退出位置模式 */
        if (s_ctrl_mode != CTRL_SPEED) {
            s_ctrl_mode    = CTRL_SPEED;
            s_vel_filt_dps = 0.0f;
        }
    } else if (!enable && s_speed_mode) {
        s_speed_mode = false;
    }
}

void app_control_set_omega_ref(float omega_rad_s)
{
    s_omega_ref_cmd = omega_rad_s;
}

float app_control_get_omega_est(void)  { return s_omega_est_rps; }
float app_control_get_omega_ref(void)  { return s_omega_ref_rps; }

bool app_control_get_speed_mode(void)
{
    return s_speed_mode;
}

/* ============================================================
 * 位置控制公共接口
 * ============================================================ */

void app_control_set_ctrl_mode(CtrlMode_t mode)
{
    if (mode == s_ctrl_mode) return;
    /* 切换模式时清位置环状态 */
    s_speed_integ  = 0.0f;
    s_vel_filt_dps = 0.0f;
    s_ctrl_mode    = mode;
    if (mode == CTRL_SPEED) {
        /* 切回速度模式：斜坡基准从当前速度开始，防止阶跃 */
        s_omega_ref_rps = s_omega_est_rps;
        s_omega_ref_cmd = s_omega_est_rps;
    } else {
        /* 进入位置/自稳模式：必须清 s_speed_mode。
         * 否则 set_speed_mode(true) 看到 mode==true 会跳过重置逻辑，
         * 导致和 S<dps>/]命令切不回速度模式。 */
        s_speed_mode = false;
    }
}

CtrlMode_t app_control_get_ctrl_mode(void) { return s_ctrl_mode; }

void app_control_set_pos_target(float deg)
{
    /* 限制到软限位范围 */
    if (deg >  POS_SOFT_LIM_DEG) deg =  POS_SOFT_LIM_DEG;
    if (deg < -POS_SOFT_LIM_DEG) deg = -POS_SOFT_LIM_DEG;
    s_pos_target_deg = deg;
}

void app_control_home(void)
{
    /* 以当前编码器角为 0° 基准，必须在载荷水平时执行！ */
    s_pos_home_offset_deg = encoder_get_angle_rad() * (180.0f / 3.14159265f);
    s_pos_target_deg      = 0.0f;
    /* 同步记录 IMU 安装偏置：如果 MPU6050 静止读数不是 0°，这里吸收该偏置 */
    s_imu_home_offset_deg = mpu6050_get_pitch_deg();
    app_control_set_ctrl_mode(CTRL_POSITION);
}

float app_control_get_pos_actual_deg(void) { return get_pos_actual_deg(); }
float app_control_get_pos_target_deg(void) { return s_pos_target_deg; }

void app_control_set_kp_pos(float kp)
{
    if (kp < 0.0f)  kp = 0.0f;
    if (kp > 20.0f) kp = 20.0f;
    s_kp_pos = kp;
}
void app_control_set_kd_pos(float kd)
{
    if (kd < 0.0f) kd = 0.0f;
    if (kd > 2.0f) kd = 2.0f;
    s_kd_pos = kd;
}
void app_control_set_pos_spd_lim(float spd)
{
    if (spd <  10.0f) spd =  10.0f;   /* 最小 10°/s */
    if (spd > 300.0f) spd = 300.0f;   /* 最大 300°/s */
    s_pos_spd_lim_dps = spd;
}
void app_control_set_pos_deadband(float dz)
{
    if (dz < 0.0f) dz = 0.0f;
    if (dz > 5.0f) dz = 5.0f;
    s_pos_deadband_deg = dz;
}

/* 调参参数读取接口（主循环回显实际生效值） */
float app_control_get_kp_pos(void)       { return s_kp_pos; }
float app_control_get_kd_pos(void)       { return s_kd_pos; }
float app_control_get_pos_spd_lim(void)  { return s_pos_spd_lim_dps; }
float app_control_get_pos_deadband(void) { return s_pos_deadband_deg; }

void app_control_set_stab_lim(float deg)
{
    if (deg < 2.0f)  deg = 2.0f;
    if (deg > 90.0f) deg = 90.0f;
    s_stab_lim_deg = deg;
}
float app_control_get_stab_lim(void)  { return s_stab_lim_deg; }
float app_control_get_imu_home(void)  { return s_imu_home_offset_deg; }
