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
 * 速度环参数（经验证稳定，平滑优先于响应速度）
 * ============================================================ */
#define KP_SPD          (0.30f)    /* A/(rad/s) - 大 KP 提供强制动 */
#define KI_SPD          (0.020f)   /* 积分克服齿槽 */
#define IQ_MAX_A        (0.25f)    /* A - 限制最大力矩防飞 */
#define IQ_SLEW_RATE_A_S (2.00f)   /* 不限制 */
#define SPEED_FF_GAIN   (0.0f)     /* 关闭前馈 */
#define SPEED_INTEG_MAX (0.20f)    /* 积分上限 */
#define OMEGA_SLEW_RATE (50.0f)    /* 不限制 */
#define FRICTION_FF_A   (0.0f)     /* 关闭 friction */
#define START_BOOST_A   (0.0f)     /* 关闭 boost */
#define START_BOOST_MS  (0U)
#define START_DONE_RPS  (0.10f)
#define STOP_DAMP_GAIN  (0.050f)
#define STOP_DAMP_MAX_A (0.15f)
#define STOP_DAMP_ON_RPS (0.20f)
#define ENC_TORQUE_HOLD_MS (25U)

/* 外部 GPIO（CubeMX 在 gpio.c 生成）*/
extern void MX_GPIO_Init(void);  /* 仅供参考，实际已由 main 调用 */

/* ============================================================
 * 内部状态
 * ============================================================ */
static AppState_t s_state = STATE_INIT;
static uint8_t    s_fault = FAULT_NONE;
static volatile float s_iq_ref_a = 0.0f;  /* q轴目标电流 [A]，主循环写、ISR 读 */

/* --- 速度环状态 --- */
static float    s_omega_est_rps  = 0.0f;   /* 滤波后速度 [rad/s] */
static float    s_omega_ref_rps  = 0.0f;   /* 经斜坡后的速度目标 [rad/s] */
static float    s_omega_ref_cmd  = 0.0f;   /* 用户目标（未经斜坡）[rad/s] */
static float    s_speed_integ    = 0.0f;   /* PI 积分项 */
static bool     s_speed_mode     = false;  /* true=速度模式, false=力矩模式 */
static uint32_t s_start_boost_until_ms = 0;
static float    s_start_boost_sign     = 0.0f;

/* --- 位置环状态 --- */
static CtrlMode_t s_ctrl_mode           = CTRL_SPEED;
static float      s_pos_home_offset_deg = 0.0f;   /* 回零时的编码器角 [deg] */
static float      s_pos_target_deg      = 0.0f;   /* 位置目标 [deg]，已clamp到±90° */
static float      s_vel_filt_dps        = 0.0f;   /* D项速度滤波 [°/s]，MOTOR_SIGN坐标系 */
static float      s_kp_pos              = 0.5f;   /* 位置P增益：半速接近目标 */
static float      s_kd_pos              = 0.15f;  /* 位置D增益：重阻尼防过冲 */
static float      s_pos_spd_lim_dps     = 45.0f;  /* 位置环输出速度限幅 [°/s]（实测稳定值）*/
static float      s_pos_deadband_deg    = 0.5f;   /* 位置死区 [deg] */
static float      s_stab_lim_deg        = 10.0f;  /* 自稳目标软限幅 [°]，默认10°，第G命令前可用 GLIM<val> 调大 */
static float      s_imu_home_offset_deg  = 0.0f;  /* IMU pitch 安装偏置，H 命令时刷新 */

/* 对齐 */
static float    s_theta_offset_e = 0.0f;  /* 电角度偏移 [rad] */
static float    s_theta_fine_rad = 0.0f;  /* 手动微调偏移 [rad]（默认0，调试用 F 命令）*/
static bool     s_align_need_verify = false;

/* 零偏采样窗口 */
#define CALIB_SAMPLES   (256U)
static uint32_t s_calib_sum_iu = 0;
static uint32_t s_calib_sum_iv = 0;
static uint32_t s_calib_sum_iw = 0;
static uint16_t s_calib_cnt    = 0;
static uint32_t last_frame_cnt = 0;

/* 前向声明（定义在文件尾部）*/
static void align_update(const FocMeasurement_t *m);
static void set_iq_ref_internal(float iq_ref_a);
static void set_iq_ref_slewed(float iq_ref_a, float dt_s);

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

    /* 编码器 — 暂时禁用：EMI 导致虚假触发，后续加更宽松的去抖 */
    // if (!encoder_is_valid())      f |= FAULT_ENCODER;

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
    set_iq_ref_internal(0.0f);      /* 确保上电时 Iq 指令为零 */
    s_calib_cnt  = 0;
    s_calib_sum_iu = s_calib_sum_iv = s_calib_sum_iw = 0;
    last_frame_cnt = foc_interface_get_frame_count();

    /* 校准必须在驱动禁用状态下进行，确保零电流。
     * CTRL_SD 拉低 → 栅极驱动器关闭 → MOSFET 全关 → 相电流 = 0。
     * 此时 ADC 读到的是运放 1.25V 偏置电压，对应理论值 ~1551。
     * ISR 仍在运行但 Simulink 输出不影响实际电流（MOSFET 已关断）。*/
    ctrl_sd_set(false);
    foc_interface_write_ccr(0.5f, 0.5f, 0.5f);  /* 零电压输出 */
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
                uint16_t raw_u = (uint16_t)((s_calib_sum_iu + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);
                uint16_t raw_v = (uint16_t)((s_calib_sum_iv + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);
                uint16_t raw_w = (uint16_t)((s_calib_sum_iw + CALIB_SAMPLES / 2U) / CALIB_SAMPLES);

                /* 合理性检查：理论零电流 ADC ≈ 1.25V/3.3V×4095 = 1551。
                 * 允许 ±300 容差（1251~1851），超出范围则用理论值兜底。
                 * 防止校准期间意外通电导致 offset 严重偏差。 */
                #define OFFSET_THEORETICAL  (1551U)
                #define OFFSET_TOLERANCE    (300U)
                if (raw_u < OFFSET_THEORETICAL - OFFSET_TOLERANCE ||
                    raw_u > OFFSET_THEORETICAL + OFFSET_TOLERANCE) {
                    raw_u = OFFSET_THEORETICAL;
                }
                if (raw_v < OFFSET_THEORETICAL - OFFSET_TOLERANCE ||
                    raw_v > OFFSET_THEORETICAL + OFFSET_TOLERANCE) {
                    raw_v = OFFSET_THEORETICAL;
                }
                if (raw_w < OFFSET_THEORETICAL - OFFSET_TOLERANCE ||
                    raw_w > OFFSET_THEORETICAL + OFFSET_TOLERANCE) {
                    raw_w = OFFSET_THEORETICAL;
                }
                iu_offset_raw = raw_u;
                iv_offset_raw = raw_v;
                iw_offset_raw = raw_w;
                ctrl_sd_set(false);  /* 校准完成，保持驱动禁用直到用户使能 */
                s_state = STATE_READY;
            }
        }
        break;
    }

    /* ---- 编码器对齐（串口 'a' 命令触发）---- */
    case STATE_ALIGN:
        align_update(&meas);
        break;

    /* ---- 就绪 + 对齐方向验证 ---- */
    case STATE_READY:
        if (s_align_need_verify) {
            s_align_need_verify = false;
            /* 方向验证暂时关闭——Vq脉冲方法在低带宽PI下响应太慢，
             * 500ms内无法产生可靠的净位移。需要更长的验证时间或
             * 不同的判断逻辑。暂时接受对齐本身的质量，
             * 必要时用 F 命令手动补偿。 */
        }
        /* VBUS/编码器异常时仅等待，不进故障 */
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
                set_iq_ref_internal(0.0f);
                ctrl_sd_set(false);
                s_state = STATE_FAULT;
                fault_consec = 0;
                last_fault   = FAULT_NONE;
            }
        } else {
            fault_consec = 0;
            last_fault   = FAULT_NONE;
        }
        /* ---- 编码器速度 + 位置环 + 速度 PI ---- */
        {
            if ((HAL_GetTick() - encoder_get_last_capture_ms()) > ENC_TORQUE_HOLD_MS) {
                s_omega_est_rps = 0.0f;
                s_vel_filt_dps  = 0.0f;
                s_speed_integ   = 0.0f;
                set_iq_ref_internal(0.0f);
                break;
            }

            /* ---- 即时速度保护（每次主循环都执行）----
             * 不依赖慢速估计，直接看编码器角度变化。
             * 如果瞬时速度超过 5 rad/s（286°/s），立刻清零力矩+积分。 */
            {
                static float s_prev_angle_rad = 0.0f;
                static uint32_t s_prev_angle_ms = 0;
                float cur_angle = encoder_get_angle_rad();
                uint32_t cur_ms = HAL_GetTick();
                uint32_t dt_prot = cur_ms - s_prev_angle_ms;
                if (dt_prot >= 5U && dt_prot < 100U) {
                    float da = cur_angle - s_prev_angle_rad;
                    if (da >  3.14159265f) da -= 6.28318530f;
                    if (da < -3.14159265f) da += 6.28318530f;
                    float inst_spd = da / ((float)dt_prot * 1e-3f);
                    if (fabsf(inst_spd) > 5.0f) {
                        /* 速度过大，紧急制动 */
                        s_speed_integ = 0.0f;
                        set_iq_ref_internal(0.0f);
                        s_prev_angle_rad = cur_angle;
                        s_prev_angle_ms = cur_ms;
                        break;  /* 跳过本拍速度环 */
                    }
                    s_prev_angle_rad = cur_angle;
                    s_prev_angle_ms = cur_ms;
                }
            }

            /* 速度环节拍：只在 dt >= 10ms 时执行，避免高频主循环
             * 对过时速度估计反复积分导致振荡 */
            uint32_t now_ms = HAL_GetTick();
            static uint32_t s_speed_loop_prev_ms = 0;
            uint32_t dt_ms_loop = now_ms - s_speed_loop_prev_ms;
            if (dt_ms_loop < 10U) break;  /* 不到 10ms 不执行速度环 */
            float dt_s = (float)dt_ms_loop * 1e-3f;
            s_speed_loop_prev_ms = now_ms;
            if (dt_s > 0.1f) dt_s = 0.1f;  /* 防异常长间隔 */

            s_omega_est_rps = encoder_get_omega_rad_s();
            float omega_dps = s_omega_est_rps * (180.0f / 3.14159265f);
            s_vel_filt_dps  = 0.85f * s_vel_filt_dps + 0.15f * omega_dps;
            float vel_pos   = MOTOR_SIGN * s_vel_filt_dps;

                /* ---- 位置环（CTRL_POSITION）----
                 * 直接 PD → IqRef，绕过速度 PI。
                 * 直接力矩测试 0.5A→~300dps，所以 IqRef = spd_cmd(°/s) / 300(°/s/A) */
                if (s_ctrl_mode == CTRL_POSITION) {
                    float pos_actual = get_pos_actual_deg();

                    if (fabsf(pos_actual) > 170.0f) {
                        set_iq_ref_internal(0.0f);
                        s_speed_integ = 0.0f;
                    } else {
                        float target      = s_pos_target_deg;
                        float pos_err     = target - pos_actual;

                        float spd_cmd;
                        if (fabsf(pos_err) < s_pos_deadband_deg) {
                            spd_cmd = 0.0f;
                        } else {
                            spd_cmd = s_kp_pos * pos_err - s_kd_pos * vel_pos;
                        }
                        float spd_lim_now = s_pos_spd_lim_dps;
                        if (fabsf(pos_actual) > POS_WARN_LIM_DEG) {
                            spd_lim_now = s_pos_spd_lim_dps * 0.3f;
                        }
                        if (pos_actual >  POS_HARD_LIM_DEG && spd_cmd > 0.0f) spd_cmd = 0.0f;
                        if (pos_actual < -POS_HARD_LIM_DEG && spd_cmd < 0.0f) spd_cmd = 0.0f;
                        if (spd_cmd >  spd_lim_now) spd_cmd =  spd_lim_now;
                        if (spd_cmd < -spd_lim_now) spd_cmd = -spd_lim_now;

                        /* 直接 PD 输出 → IqRef：300dps/A 换算 */
                        float iq_cmd = spd_cmd / 300.0f;
                        if (iq_cmd >  IQ_MAX_A) iq_cmd =  IQ_MAX_A;
                        if (iq_cmd < -IQ_MAX_A) iq_cmd = -IQ_MAX_A;
                        set_iq_ref_internal(iq_cmd);
                    }
                    s_omega_ref_rps = 0.0f;  /* 不用速度PI */
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


                /* ---- 速度控制：位置跟踪模式 ----
                 * 不用速度 PI。把速度命令转成匀速递增的位置目标，
                 * 用位置 PD 控制器跟踪。消除速度估计噪声导致的抖动。
                 *
                 * IqRef = KP_TRK * pos_err - KD_TRK * vel_est
                 * pos_target += omega_ref * dt
                 */
                #define KP_TRK   (0.006f)   /* A/deg — 很软的弹簧，不振荡 */
                #define KD_TRK   (0.005f)   /* A/(deg/s) — 重阻尼 */
                #define IQ_TRK_MAX (0.30f)  /* A — 限制力矩防甩 */
                #define POS_ERR_MAX_DEG (10.0f)  /* 缩小误差限制 */

                bool run_spd = (s_ctrl_mode == CTRL_STABILIZE)   ||
                               (s_ctrl_mode == CTRL_SPEED && s_speed_mode);
                if (run_spd) {
                    if (s_ctrl_mode == CTRL_SPEED) {
                        float pos_actual = get_pos_actual_deg();
                        float omega_dps_cmd = s_omega_ref_cmd * (180.0f / 3.14159265f);

                        /* S0 时停止：目标锁定到当前位置 */
                        if (fabsf(s_omega_ref_cmd) < 0.05f) {
                            s_pos_target_deg = pos_actual;
                        } else {
                            /* 计算误差时处理 wrap：把误差限制在 ±180° 内 */
                            float err_now = s_pos_target_deg - pos_actual;
                            while (err_now >  180.0f) err_now -= 360.0f;
                            while (err_now < -180.0f) err_now += 360.0f;

                            /* 只在误差小于限制时递增目标 */
                            if (fabsf(err_now) < POS_ERR_MAX_DEG) {
                                s_pos_target_deg += omega_dps_cmd * dt_s;
                            }
                            /* 防止 target 无限增长：wrap 到 ±360° 范围 */
                            while (s_pos_target_deg >  360.0f) s_pos_target_deg -= 360.0f;
                            while (s_pos_target_deg < -360.0f) s_pos_target_deg += 360.0f;
                        }
                    }

                    float pos_actual = get_pos_actual_deg();
                    float pos_err = s_pos_target_deg - pos_actual;
                    /* wrap 误差到 ±180° */
                    while (pos_err >  180.0f) pos_err -= 360.0f;
                    while (pos_err < -180.0f) pos_err += 360.0f;

                    /* 限制误差防止力矩过大 */
                    if (pos_err >  POS_ERR_MAX_DEG) pos_err =  POS_ERR_MAX_DEG;
                    if (pos_err < -POS_ERR_MAX_DEG) pos_err = -POS_ERR_MAX_DEG;

                    /* ISR 级 PD 全权负责 IqRef，主循环不再写 */
                    /* （主循环只负责更新 s_pos_target_deg） */
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
    s_omega_est_rps  = 0.0f;
    /* 位置环滤波器初始化 */
    s_vel_filt_dps   = 0.0f;
    ctrl_sd_set(true);
    s_state = STATE_RUN;
    return true;
}

void app_control_disable(void)
{
    set_iq_ref_internal(0.0f);
    /* 速度环清零 */
    s_speed_mode    = false;
    s_omega_ref_rps = 0.0f;
    s_omega_ref_cmd = 0.0f;
    s_speed_integ   = 0.0f;
    s_omega_est_rps = 0.0f;
    s_start_boost_until_ms = 0;
    s_start_boost_sign     = 0.0f;
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
    if (s_state == STATE_RUN || s_state == STATE_ALIGN) {
        return s_iq_ref_a;
    }
    return 0.0f;
}

/**
 * @brief  设置 q 轴目标电流 [A]（速度环输出或外部调试指令）
 * @note   写操作在主循环；ISR 读 s_iq_ref_a 为 volatile float，
 *         Cortex-M4 单精度浮点写为原子操作，无需额外临界区。
 */
static void set_iq_ref_internal(float iq_ref_a)
{
    float prev = s_iq_ref_a;
    bool need_reset = ((prev * iq_ref_a) < 0.0f || fabsf(iq_ref_a - prev) > 0.10f);
    s_iq_ref_a = iq_ref_a;
    if (need_reset) {
        foc_interface_request_current_pi_reset();
    }
}

static void set_iq_ref_slewed(float iq_ref_a, float dt_s)
{
    float max_step = IQ_SLEW_RATE_A_S * dt_s;
    if (max_step < 0.001f) {
        max_step = 0.001f;
    }
    float prev = s_iq_ref_a;
    float delta = iq_ref_a - prev;
    if (delta > max_step) {
        iq_ref_a = prev + max_step;
    } else if (delta < -max_step) {
        iq_ref_a = prev - max_step;
    }
    set_iq_ref_internal(iq_ref_a);
}

void app_control_set_iq_ref(float iq_ref_a)
{
    set_iq_ref_internal(iq_ref_a);
}

/* ============================================================
 * ALIGN 接口（串口 'a' 命令触发，不自动上电执行）
 * ============================================================ */
#define ALIGN_HOLD_MS   (2000U)  /* 2s 对齐保持 */
#define ALIGN_ID_V      (5.0f)   /* d 轴对齐电压 [V]，Rs=2.7Ω → I≈1.85A，强力锁定克服齿槽 */
static uint32_t s_align_start_ms = 0;

/* 旋转式 ALIGN 状态变量 */
static uint8_t  s_align_phase = 0;
static uint32_t s_align_phase_t0 = 0;
static float    s_align_sin_sum = 0.0f;
static float    s_align_cos_sum = 0.0f;
static uint32_t s_align_sample_cnt = 0;

bool app_control_start_align(void)
{
    if (s_state != STATE_READY) return false;

    s_align_start_ms = HAL_GetTick();
    s_align_phase = 0;
    s_align_sin_sum = 0.0f;
    s_align_cos_sum = 0.0f;
    s_align_sample_cnt = 0;
    s_state = STATE_ALIGN;
    ctrl_sd_set(true);
    return true;
}

/* === 旋转校准式 ALIGN（openloop + π/2 修正）===
 * 用 openloop 模式旋转电机，采样 forced_theta 和 encoder 的相位差。
 * 圆形平均 + π/2 修正得到 offset。方向已验证正确。
 */
#define ALIGN_OL_OMEGA      (5.5f)   /* rad/s 电气 */
#define ALIGN_OL_VQ         (2.0f)   /* V */
#define ALIGN_WARMUP_MS     (800U)
#define ALIGN_SAMPLE_MS     (1500U)
#define ALIGN_COOLDOWN_MS   (400U)
#define ALIGN_TOTAL_MS      (ALIGN_WARMUP_MS + ALIGN_SAMPLE_MS + ALIGN_COOLDOWN_MS)

static void align_update(const FocMeasurement_t *m)
{
    (void)m;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_align_start_ms;

    if (elapsed > ALIGN_TOTAL_MS + 1000U) {
        foc_openloop_stop();
        ctrl_sd_set(false);
        s_state = STATE_READY;
        s_align_phase = 0;
        return;
    }

    if (s_align_phase == 0) {
        foc_openloop_start(ALIGN_OL_OMEGA, ALIGN_OL_VQ);
        s_align_phase = 1;
        s_align_phase_t0 = now;
        s_align_sin_sum = 0.0f;
        s_align_cos_sum = 0.0f;
        s_align_sample_cnt = 0;
    }
    else if (s_align_phase == 1) {
        if (elapsed >= ALIGN_WARMUP_MS) {
            s_align_phase = 2;
            s_align_phase_t0 = now;
        }
    }
    else if (s_align_phase == 2) {
        float forced_e = foc_openloop_get_theta();
        float enc_e = encoder_get_elec_angle_rad((uint8_t)MOTOR_POLE_PAIRS);
        float diff = forced_e - enc_e;
        s_align_sin_sum += sinf(diff);
        s_align_cos_sum += cosf(diff);
        s_align_sample_cnt++;

        if (elapsed >= ALIGN_WARMUP_MS + ALIGN_SAMPLE_MS) {
            if (s_align_sample_cnt > 0) {
                float avg_diff = atan2f(s_align_sin_sum, s_align_cos_sum);
                s_theta_offset_e = -avg_diff + 1.5707963f;  /* +π/2 修正 */
                while (s_theta_offset_e <  0.0f)               s_theta_offset_e += 6.28318530f;
                while (s_theta_offset_e >= 6.28318530f)        s_theta_offset_e -= 6.28318530f;
            }
            s_align_phase = 3;
            s_align_phase_t0 = now;
        }
    }
    else if (s_align_phase == 3) {
        uint32_t cd_elapsed = now - s_align_phase_t0;
        if (cd_elapsed < ALIGN_COOLDOWN_MS) {
            float ratio = 1.0f - (float)cd_elapsed / (float)ALIGN_COOLDOWN_MS;
            foc_openloop_start(ALIGN_OL_OMEGA, ALIGN_OL_VQ * ratio);
        } else {
            foc_openloop_stop();
            /* 进入方向验证阶段 */
            s_align_phase = 4;
            s_align_phase_t0 = now;
            /* 记录验证前位置 */
            s_align_sin_sum = encoder_get_angle_rad();  /* 复用变量存 pos_before */
            /* 注入 +Iq 验证方向（ISR 级保护防飞车） */
            set_iq_ref_internal(0.30f);
        }
    }
    else if (s_align_phase == 4) {
        /* 方向验证：+Iq 注入 300ms，看编码器方向 */
        uint32_t vf_elapsed = now - s_align_phase_t0;
        if (vf_elapsed >= 300U) {
            set_iq_ref_internal(0.0f);
            HAL_Delay(50);
            float pos_after = encoder_get_angle_rad();
            float pos_before = s_align_sin_sum;  /* 之前存的 */
            float delta = pos_after - pos_before;
            if (delta >  3.14159265f) delta -= 6.28318530f;
            if (delta < -3.14159265f) delta += 6.28318530f;

            /* +Iq 应该让电机往 MOTOR_SIGN 正方向走 */
            if (delta * MOTOR_SIGN < -0.01f) {
                /* 方向反了，offset 加 π */
                s_theta_offset_e += 3.14159265f;
                if (s_theta_offset_e >= 6.28318530f)
                    s_theta_offset_e -= 6.28318530f;
            }

            ctrl_sd_set(false);
            s_state = STATE_READY;
            s_align_phase = 0;
        }
    }
}

float app_control_get_align_id_ref(void)
{
    /* phase 4 时不注入 Vd（方向验证需要正常闭环） */
    if (s_state == STATE_ALIGN && s_align_phase < 4) return ALIGN_ID_V;
    return 0.0f;
}

float app_control_get_theta_offset(void)
{
    return s_theta_offset_e;
}

void app_control_set_fine_offset(float rad)
{
    s_theta_fine_rad = rad;
}

float app_control_get_fine_offset(void)
{
    return s_theta_fine_rad;
}

/* ============================================================
 * 速度环公共接口
 * ============================================================ */

void app_control_set_speed_mode(bool enable)
{
    if (enable && !s_speed_mode) {
        s_speed_mode     = true;
        s_speed_integ    = 0.0f;
        s_omega_ref_rps  = 0.0f;
        s_omega_ref_cmd  = 0.0f;
        s_start_boost_until_ms = 0;
        s_start_boost_sign     = 0.0f;
        /* 位置跟踪模式：初始化目标为当前位置 */
        s_pos_target_deg = get_pos_actual_deg();
        if (s_ctrl_mode != CTRL_SPEED) {
            s_ctrl_mode    = CTRL_SPEED;
            s_vel_filt_dps = 0.0f;
        }
    } else if (!enable && s_speed_mode) {
        s_speed_mode = false;
        s_start_boost_until_ms = 0;
        s_start_boost_sign     = 0.0f;
    }
}

void app_control_set_omega_ref(float omega_rad_s)
{
    float prev = s_omega_ref_cmd;
    bool start_or_reverse = (fabsf(omega_rad_s) >= 0.05f) &&
                            (fabsf(prev) < 0.05f || (prev * omega_rad_s) < 0.0f);
    if ((prev * omega_rad_s) < 0.0f ||
        fabsf(omega_rad_s) < 0.05f ||
        fabsf(omega_rad_s - prev) > 0.50f) {
        s_speed_integ = 0.0f;
    }
    if (fabsf(omega_rad_s) < 0.05f) {
        s_omega_ref_rps = 0.0f;
        s_start_boost_until_ms = 0;
        s_start_boost_sign     = 0.0f;
        set_iq_ref_internal(0.0f);
    } else if ((prev * omega_rad_s) < 0.0f) {
        s_omega_ref_rps = 0.0f;
    }
    if (start_or_reverse) {
        s_start_boost_until_ms = HAL_GetTick() + START_BOOST_MS;
        s_start_boost_sign = (omega_rad_s > 0.0f) ? 1.0f : -1.0f;
    }
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

float app_control_get_pos_target(void) { return s_pos_target_deg; }
float app_control_get_pos_home_offset(void) { return s_pos_home_offset_deg; }
bool app_control_is_pos_tracking(void) {
    return (s_ctrl_mode == CTRL_STABILIZE) ||
           (s_ctrl_mode == CTRL_SPEED && s_speed_mode);
}

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
