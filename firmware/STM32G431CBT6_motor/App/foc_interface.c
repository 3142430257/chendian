/**
 * @file    foc_interface.c
 * @brief   FOC 算法输入/输出接口实现
 *
 * V2：接入 Simulink Embedded Coder 生成的 foc_controller_step()
 *     控制器输入：I_a, I_b, theta_e, iq_ref, V_bus, reset
 *     控制器输出：duty_a, duty_b, duty_c
 *
 * 调用顺序（ISR 内，< 20~25 μs）：
 *   1. ADC 原始值 → 物理量（IU/IV/IW/VBUS）
 *   2. 取编码器角度
 *   3. foc_model_step()（V1 跳过，V2 接入）
 *   4. 限幅后写 TIM1 CCR1/2/3
 */

#include "foc_interface.h"
#include "foc_controller.h"
#include "board_config.h"
#include "encoder_spi.h"
#include "app_control.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>     /* cosf, sinf */

/* ==========================================================
 * 调试开关：编译前先定义以下 FOC_DBG_xx 其中之一，验证完成后删除定义
 *
 * FOC_DBG_DUTY_TEST    -- 全三相 50% duty，验 PWM/驱动 基础是否正常
 *                         预期: IU/IV/IWR 接近 0A
 *
 * FOC_DBG_FIXED_VECTOR -- U+/V+/W+ 固定矢量循环（每 DBG_VEC_HOLD_MS 切一次）
 *                         验证: PWM 相序、ADC 通道映射、采样符号
 *                         预期（Y接星形）:
 *                           U+: IU>0, IV≈-IU/2, IWR≈-IU/2, KVL≈0
 *                           V+: IV>0, IU≈-IV/2, IWR≈-IV/2, KVL≈0
 *                           W+: IWR>0, IU≈-IWR/2, IV≈-IWR/2, KVL≈0
 *                         如果符号错误 → 检查对应相 ADC 增益/接线
 *                         如果相序错误 → 检查驱动板 U/V/W 与电机接线
 * ========================================================== */

// #define FOC_DBG_DUTY_TEST
// #define FOC_DBG_FIXED_VECTOR   /* 第二轮验证：重构后电流符号是否正确 */
// #define FOC_DBG_SINGLE_CH      /* 第三轮：单通道 PA8/PA9/PA10 各自独立驱动诊断 */

/* 固定矢量参数（首次先用 0.3V，无响应再小步加到 0.5V，别超 1A） */
#define DBG_VEC_V       (0.3f)    /* 施加相电压 [V] */
#define DBG_VEC_HOLD_MS (400U)    /* 每个矢量保持 ms */

extern TIM_HandleTypeDef htim1;  /* CubeMX 在 tim.c 生成 */

/* 每相零偏（主程序初始化时测量）*/
uint16_t iu_offset_raw = 2048U;
uint16_t iv_offset_raw = 2048U;
uint16_t iw_offset_raw = 2048U;

/* 最新 ADC 原始值快照（ISR 内更新，主循环读取）*/
static volatile uint16_t s_adc_raw_snap[FOC_ADC_CH_COUNT] = {0};
static volatile uint32_t s_adc_frame_cnt = 0;

/* 测量量快照（ISR 写，主循环读）*/
static FocMeasurement_t s_meas_buf = {0};
static volatile float   s_target_rpm = 0.0f;

/* 固定矢量调试：当前矢量阶段（0=U+ 1=V+ 2=W+），供 telemetry 读取 */
volatile uint8_t s_dbg_vec_phase = 0xFF;   /* 0xFF = 非调试模式 */

/* dq 窗口统计累加器（ISR 专属，无需 volatile）*/
static uint32_t  s_stat_cnt    = 0;
static float     s_stat_sum_id  = 0.0f;
static float     s_stat_sum_iq  = 0.0f;
static float     s_stat_sum_iqr = 0.0f;
static float     s_stat_sum_id2 = 0.0f;
static float     s_stat_sum_iq2 = 0.0f;
static float     s_stat_sum_err = 0.0f;

/* dq 统计快照（ISR 写，主循环读，临界区保护）*/
static FocDqStats_t s_dq_snap   = {0};
static volatile bool s_dq_ready = false;

/* ---- 开环强制换向 ---- */
static volatile bool  s_openloop_active = false;
static volatile float s_openloop_theta  = 0.0f;
static volatile float s_openloop_omega  = 0.0f;
static volatile float s_openloop_vq     = 0.0f;
static volatile bool  s_pi_bypass       = false;
static volatile bool  s_current_pi_reset_req = false;
#define OPENLOOP_DT  (1.0f / 20000.0f)
#define IQ_ZERO_DEADBAND_A (0.005f)

/* ---- 开环期间编码器诊断 ---- */
static volatile float s_calib_sum_plus  = 0.0f;  /* sum(theta_ol + raw_e) */
static volatile float s_calib_sum_minus = 0.0f;  /* sum(theta_ol - raw_e) */
static volatile uint32_t s_calib_cnt    = 0;
static volatile float s_calib_enc_last  = 0.0f;  /* 最后一次编码器读数 */
#define TWO_PI       (6.28318530f)

/* ---- 强制闭环旋转（绕过编码器）---- */
static volatile bool  s_forced_active = false;
static volatile float s_forced_theta  = 0.0f;   /* 当前强制电角度 [rad] */
static volatile float s_forced_omega  = 0.0f;   /* 电角速度 [rad/s] */

void foc_interface_set_pi_bypass(bool on) { s_pi_bypass = on; }
static volatile float s_bypass_vq = 0.0f;
void foc_interface_set_bypass_vq(float vq) { s_bypass_vq = vq; }
void foc_interface_request_current_pi_reset(void) { s_current_pi_reset_req = true; }

/* ---- 5ms 滑窗速度估算（ISR 内，替代旧预测器）---- */
void foc_forced_start(float elec_omega_rps) {
    s_forced_theta = 0.0f;
    s_forced_omega = elec_omega_rps;
    s_forced_active = true;
}
void foc_forced_stop(void) {
    s_forced_active = false;
}

float foc_forced_get_theta(void) {
    return s_forced_theta;
}

float foc_openloop_get_theta(void) {
    return s_openloop_theta;
}

/* ---------------------------------------------------------- */
static inline uint32_t clamp_ccr(float duty_norm)
{
    /* duty_norm: [0.0, 1.0] 归一化占空比 */
    if (duty_norm < 0.0f) duty_norm = 0.0f;
    if (duty_norm > 1.0f) duty_norm = 1.0f;
    uint32_t ccr = (uint32_t)(duty_norm * (float)TIM1_ARR);
    if (ccr < TIM1_CCR_MIN) ccr = TIM1_CCR_MIN;
    if (ccr > TIM1_CCR_MAX) ccr = TIM1_CCR_MAX;
    return ccr;
}

/* ---------------------------------------------------------- */
void foc_interface_write_ccr(float duty_a, float duty_b, float duty_c)
{
    /* 归一化占空比 → 限幅 CCR → 写 TIM1（V3 Simulink 输出接口）
     * 限幅范围 [TIM1_CCR_MIN, TIM1_CCR_MAX]，绝不打到 0%/100% */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clamp_ccr(duty_a));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, clamp_ccr(duty_b));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, clamp_ccr(duty_c));
}

/* ---------------------------------------------------------- */
void foc_interface_init(void)
{
    memset(&s_meas_buf, 0, sizeof(s_meas_buf));
    s_target_rpm = 0.0f;

    /* 初始化 Simulink 生成的控制器 */
    foc_controller_initialize();
}

/* ---------------------------------------------------------- */
/* ISR 频率诊断 */
static volatile uint32_t s_isr_count = 0;
static volatile uint32_t s_isr_freq_hz = 0;
static uint32_t s_isr_last_sec = 0;

uint32_t foc_interface_get_isr_freq(void) { return s_isr_freq_hz; }

void foc_interface_step(const uint16_t *adc_raw)
{
    /* ISR 频率测量 */
    s_isr_count++;
    uint32_t now_ms = HAL_GetTick();
    if ((now_ms - s_isr_last_sec) >= 1000U) {
        s_isr_freq_hz = s_isr_count;
        s_isr_count = 0;
        s_isr_last_sec = now_ms;
    }

    /* 保存 ADC 原始快照（用于校准和故障检测）*/
    for (uint8_t i = 0; i < FOC_ADC_CH_COUNT; i++) {
        s_adc_raw_snap[i] = adc_raw[i];
    }

    /* --- Step 1: ADC 原始值 → 低边采样 → 相电流重构 ---
     *
     * 低边采样特性：
     *   驱动相（高占空比，低边FET断开）→ 采样值 ≈ 0，无效
     *   返回相（低占空比，低边FET导通）→ 采样值 = 返回电流量级（物理为正）
     *
     * FOC Clarke 约定：正值 = 电流流入电机
     *   → 返回相真实电流 = -(采样值)
     *   → 驱动相真实电流 = -(其余两相真实电流之和)  [KVL重构]
     *
     * 判断驱动相：比较当前 CCR，最大者对应本 ISR 采样周期内的驱动相
     */
    float iu_shunt = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IU], iu_offset_raw);
    float iv_shunt = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IV], iv_offset_raw);
    float iw_shunt = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IW], iw_offset_raw);

    uint32_t ccr_a = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t ccr_b = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t ccr_c = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);

    /* ---- 低边电流重构（带滞回）----
     *
     * 低调制比（占空比≈50%）时三个 CCR 几乎相等，SVPWM 中性点注入
     * 的微小波动会导致"最大相"在 U/V/W 间随机切换。每切换一次，
     * 重构所用的物理采样组合就变一次，三相 shunt 的零点偏差被放大
     * 为电流跳变 → Id/Iq 振荡 → 转矩脉动 → 噪声。
     *
     * 滞回：CCR 最大与次大差距 < ARR×2% 时，保持上一拍的选相。*/
    #define CCR_HYST_THRESH  ((uint32_t)(TIM1_ARR * 0.02f))  /* ARR 的 2% */
    static uint8_t s_last_phase = 0;  /* 0=U, 1=V, 2=W */

    /* 三元素冒泡排序（ccr_a/b/c 降序 → idx[0]=max, idx[1]=mid, idx[2]=min）*/
    uint32_t vals[3] = {ccr_a, ccr_b, ccr_c};
    uint8_t  idx[3]  = {0, 1, 2};  /* 0=U, 1=V, 2=W */
    if (vals[0] < vals[1]) { uint32_t tv = vals[0]; vals[0] = vals[1]; vals[1] = tv;
                              uint8_t ti = idx[0];   idx[0] = idx[1];   idx[1] = ti; }
    if (vals[1] < vals[2]) { uint32_t tv = vals[1]; vals[1] = vals[2]; vals[2] = tv;
                              uint8_t ti = idx[1];   idx[1] = idx[2];   idx[2] = ti; }
    if (vals[0] < vals[1]) { uint32_t tv = vals[0]; vals[0] = vals[1]; vals[1] = tv;
                              uint8_t ti = idx[0];   idx[0] = idx[1];   idx[1] = ti; }

    uint8_t driven_phase;
    if ((vals[0] - vals[1]) < CCR_HYST_THRESH) {
        driven_phase = s_last_phase;  /* 差距太小，保持上一拍 */
    } else {
        driven_phase = idx[0];        /* 正常选最大 CCR 相 */
        s_last_phase = idx[0];
    }

    float iu, iv, iw;
    if (driven_phase == 0) {
        /* U 相驱动（CCR 最大），IU 采样不可靠，从 IV/IW 重构 */
        iv = -iv_shunt;
        iw = -iw_shunt;
        iu = -(iv + iw);
    } else if (driven_phase == 1) {
        /* V 相驱动，IV 采样不可靠，从 IU/IW 重构 */
        iu = -iu_shunt;
        iw = -iw_shunt;
        iv = -(iu + iw);
    } else {
        /* W 相驱动，IW 采样不可靠，从 IU/IV 重构 */
        iu = -iu_shunt;
        iv = -iv_shunt;
        iw = -(iu + iv);
    }

    float vbus_adc = ADC_TO_VBUS(adc_raw[FOC_ADC_IDX_VBUS]);
    float vbus = 12.0f;  /* 实际电源电压 */
    (void)vbus_adc;


    /* 更新测量快照（所有模式都需要，保证遥测和保护看到实时值）*/
    s_meas_buf.iu_a      = iu;
    s_meas_buf.iv_a      = iv;
    s_meas_buf.iw_a      = iw;
    s_meas_buf.vbus_v    = vbus_adc;  /* 遥测显示 ADC 读数，便于排查分压比 */

#ifdef FOC_DBG_DUTY_TEST
    /* ==== 50% duty 旁路，验证 PWM/驱动 基础 ====
     * 预期: IU/IV/IWR 接近 0A。若仍有几安说明驱动/ADC采样自身有故障。*/
    s_dbg_vec_phase = 0xFF;
    foc_interface_write_ccr(0.5f, 0.5f, 0.5f);
    foc_controller_U.In_reset = true;
    foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
    foc_controller_U.In_theta_e = 0.0f; foc_controller_U.In_iq_ref = 0.0f;
    foc_controller_U.In_V_bus = vbus;
    foc_controller_step();

#elif defined(FOC_DBG_FIXED_VECTOR)
    /* ==== U+/V+/W+ 固定矢量循环，验证采样链 ====
     * 矢量切换由 HAL_GetTick() 驱动（主循环精度），ISR 内只读 s_vp。
     * 注意：本模式会持续通电，使能 CTRL_SD 后再开始。*/
    {
        static uint8_t  s_vp  = 0;
        static uint32_t s_vt0 = 0;
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - s_vt0) >= DBG_VEC_HOLD_MS) {
            s_vp  = (uint8_t)((s_vp + 1U) % 3U);
            s_vt0 = now_ms;
        }
        s_dbg_vec_phase = s_vp;
        float d = (vbus > 1.0f) ? (0.75f * DBG_VEC_V / vbus) : 0.0f;
        switch (s_vp) {
        case 0:  foc_interface_write_ccr(0.5f+d, 0.5f-d, 0.5f-d); break; /* U+ */
        case 1:  foc_interface_write_ccr(0.5f-d, 0.5f+d, 0.5f-d); break; /* V+ */
        default: foc_interface_write_ccr(0.5f-d, 0.5f-d, 0.5f+d); break; /* W+ */
        }
        /* 重置 Simulink 积分器（矢量测试期间不需要闭环）*/
        foc_controller_U.In_reset = true;
        foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
        foc_controller_U.In_theta_e = 0.0f; foc_controller_U.In_iq_ref = 0.0f;
        foc_controller_U.In_V_bus = vbus;
        foc_controller_step();
    }

#elif defined(FOC_DBG_SINGLE_CH)
    /* ==== 单通道 CH1/CH2/CH3 激活诊断 ====
     * 每 500ms 切换：CH1+/CH2+/CH3+/全50%（静默对照）
     * 一次只激活一个 PWM 通道高出 50%，其他两相保持 50%。
     * 预期 Y 接电机：只有激活通道对应物理相驱动电流为正，其余两相返回 -1/2 各。
     * 用于诊断：PA8/PA9/PA10 各自实际控制驱动板的哪相 H 桥。
     * 看遥测 VEC=0/1/2/3 + IU/IV/IW 联合判断。 */
    {
        static uint8_t  s_sp  = 0;
        static uint32_t s_st0 = 0;
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - s_st0) >= 500U) {
            s_sp  = (uint8_t)((s_sp + 1U) % 4U);
            s_st0 = now_ms;
        }
        s_dbg_vec_phase = s_sp;  /* 0/1/2 = CH1+/CH2+/CH3+; 3 = idle */
        float d = (vbus > 1.0f) ? (0.5f * 1.0f / vbus) : 0.0f;  /* Vq=1V 单相，电流 ~0.4A */
        switch (s_sp) {
        case 0:  foc_interface_write_ccr(0.5f+d, 0.5f, 0.5f); break;  /* CH1+ */
        case 1:  foc_interface_write_ccr(0.5f, 0.5f+d, 0.5f); break;  /* CH2+ */
        case 2:  foc_interface_write_ccr(0.5f, 0.5f, 0.5f+d); break;  /* CH3+ */
        default: foc_interface_write_ccr(0.5f, 0.5f, 0.5f); break;     /* idle */
        }
        foc_controller_U.In_reset = true;
        foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
        foc_controller_U.In_theta_e = 0.0f; foc_controller_U.In_iq_ref = 0.0f;
        foc_controller_U.In_V_bus = vbus;
        foc_controller_step();
    }

#else
    /* --- Step 2: 编码器电角度 --- */
    uint8_t ctrl_state = app_control_get_state();
    float elec_angle;
    s_dbg_vec_phase = 0xFF;  /* 非调试模式标记 */

    if (ctrl_state == STATE_ALIGN && !s_forced_active && !s_openloop_active) {
        float vd  = app_control_get_align_id_ref();
        if (vd > 0.1f) {
            /* Vd 锁定模式（phase 0-3） */
            float d   = (vbus > 1.0f) ? (0.75f * vd / vbus) : 0.0f;
            foc_interface_write_ccr(0.5f + d, 0.5f - d, 0.5f - d);
            foc_controller_U.In_reset = true;
            foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
            foc_controller_U.In_theta_e = 0.0f; foc_controller_U.In_iq_ref = 0.0f;
            foc_controller_U.In_V_bus = vbus;
            foc_controller_step();
        } else {
            /* phase 4 方向验证：走正常闭环（用 offset + Iq） */
            goto normal_closed_loop;
        }
    } else if (s_openloop_active) {
        /* ---- 开环强制换向：手动扫描电角度 ---- */
        s_openloop_theta += s_openloop_omega * OPENLOOP_DT;
        if (s_openloop_theta >= TWO_PI)  s_openloop_theta -= TWO_PI;
        if (s_openloop_theta < 0.0f)     s_openloop_theta += TWO_PI;
        elec_angle = s_openloop_theta;
        s_meas_buf.angle_rad = elec_angle;
        /* 直接用电压注入（SVPWM），不走电流PI */
        float vq = s_openloop_vq;
        float vd = 0.0f;
        float cos_t = cosf(elec_angle);
        float sin_t = sinf(elec_angle);
        /* 反 Park: Valpha = Vd*cos - Vq*sin, Vbeta = Vd*sin + Vq*cos */
        float va = vd * cos_t - vq * sin_t;
        float vb = vd * sin_t + vq * cos_t;
        /* 反 Clarke → 三相电压 */
        float v_u = va;
        float v_v = -0.5f * va + 0.866025f * vb;
        float v_w = -0.5f * va - 0.866025f * vb;
        /* SVPWM 中性点注入 */
        float vn = -(fmaxf(fmaxf(v_u,v_v),v_w) + fminf(fminf(v_u,v_v),v_w)) / 2.0f;
        float da = (v_u + vn) / vbus + 0.5f;
        float db = (v_v + vn) / vbus + 0.5f;
        float dc = (v_w + vn) / vbus + 0.5f;
        foc_interface_write_ccr(da, db, dc);
        /* 同步记录编码器（用于自动校准）*/
        float raw_e_ol = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
        s_calib_enc_last = raw_e_ol;
        /* 用sin/cos平均避免角度wrapping问题 */
        float diff = elec_angle - raw_e_ol;
        float summ = elec_angle + raw_e_ol;
        s_calib_sum_plus  += summ;
        s_calib_sum_minus += diff;
        s_calib_cnt++;
        /* 重置 Simulink 积分器 */
        foc_controller_U.In_reset = true;
        foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
        foc_controller_U.In_theta_e = elec_angle;
        foc_controller_U.In_iq_ref = 0.0f;
        foc_controller_U.In_V_bus = vbus;
        foc_controller_step();
    } else if (s_forced_active) {
        /* ---- 强制闭环旋转：角度固定递增，PI控电流 ---- */
        s_forced_theta += s_forced_omega * OPENLOOP_DT;
        if (s_forced_theta >= TWO_PI)  s_forced_theta -= TWO_PI;
        if (s_forced_theta < 0.0f)     s_forced_theta += TWO_PI;
        elec_angle = s_forced_theta;
        s_meas_buf.angle_rad = elec_angle;

        foc_controller_U.In_I_a     = iu;
        foc_controller_U.In_I_b     = iv;
        foc_controller_U.In_theta_e = elec_angle;
        foc_controller_U.In_iq_ref  = app_control_get_iq_ref();
        foc_controller_U.In_V_bus   = vbus;
        /* 响应 PI 复位请求（forced 启动时调一次防 PI 残留） */
        bool reset_now = s_current_pi_reset_req;
        s_current_pi_reset_req = false;
        foc_controller_U.In_reset   = reset_now;
        foc_controller_step();
        foc_interface_write_ccr(
            foc_controller_Y.Out_duty_a,
            foc_controller_Y.Out_duty_b,
            foc_controller_Y.Out_duty_c
        );
    } else {
        /* ---- 正常闭环模式（SPI 编码器 + 5ms 滑窗速度）---- */
        normal_closed_loop: ;

        /* 角度直接读缓存（主循环中 SPI 更新） */
        float raw_e  = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
        float offset = app_control_get_theta_offset();
        float fine   = app_control_get_fine_offset();

        /* 电角度（无预测，SPI 与电流采样严格同步） */
        elec_angle = raw_e - offset + fine;
        while (elec_angle <  0.0f)               elec_angle += TWO_PI;
        while (elec_angle >= TWO_PI)              elec_angle -= TWO_PI;
        s_meas_buf.angle_rad = elec_angle;

        /* 5ms 滑窗速度估算（100×50μs） */
        /* 低通滤波电流测量值 */
        static float s_iu_filt = 0.0f, s_iv_filt = 0.0f;
        static bool  s_filt_init = false;
        if (!s_filt_init) { s_iu_filt = iu; s_iv_filt = iv; s_filt_init = true; }
        s_iu_filt = 0.2f * iu + 0.8f * s_iu_filt;
        s_iv_filt = 0.2f * iv + 0.8f * s_iv_filt;

        float iq_ref_cmd = app_control_get_iq_ref();

        /* ---- ISR 级位置 PD 控制器（1kHz，每次编码器更新时执行）----
         * 直接用编码器角度做 PD，不依赖主循环的慢速度估计。
         * 只在 STABILIZE 或 SPEED+speed_mode 时激活。 */
        {
            static float s_prev_mech_rad = 0.0f;
            static float s_vel_isr_dps = 0.0f;
            static bool  s_mech_init = false;
            float cur_mech = encoder_get_angle_rad();
            if (!s_mech_init) {
                s_prev_mech_rad = cur_mech;
                s_mech_init = true;
            }
            float dm = cur_mech - s_prev_mech_rad;
            if (dm >  3.14159265f) dm -= 6.28318530f;
            if (dm < -3.14159265f) dm += 6.28318530f;
            s_prev_mech_rad = cur_mech;

            /* dm 只在编码器更新时非零（每 1ms）。
             * 速度 = dm / 0.001s，IIR 滤波 */
            if (fabsf(dm) > 0.0001f) {
                float spd_raw_dps = dm * (180.0f / 3.14159265f) / 0.001f;
                /* 限幅防 EMI 毛刺 */
                if (spd_raw_dps >  1000.0f) spd_raw_dps =  1000.0f;
                if (spd_raw_dps < -1000.0f) spd_raw_dps = -1000.0f;
                s_vel_isr_dps = 0.5f * spd_raw_dps + 0.5f * s_vel_isr_dps;
            }

            /* 速度保护：> 400°/s 切断 */
            if (fabsf(s_vel_isr_dps) > 400.0f) {
                iq_ref_cmd = 0.0f;
            }
            /* ISR 级 PD：在 STABILIZE、POSITION 或 SPEED+speed_mode 时激活 */
            else if (ctrl_state == STATE_RUN &&
                     (app_control_get_ctrl_mode() == CTRL_STABILIZE ||
                      app_control_get_ctrl_mode() == CTRL_POSITION ||
                      app_control_is_pos_tracking())) {
                /* 直接用主循环算好的 pos_actual 和 target，避免 wrap 不一致 */
                float pos_actual = app_control_get_pos_actual_deg();
                float target = app_control_get_pos_target();
                float pos_err = target - pos_actual;
                /* wrap 误差到 ±180° */
                if (pos_err >  180.0f) pos_err -= 360.0f;
                if (pos_err < -180.0f) pos_err += 360.0f;
                if (pos_err >  25.0f) pos_err =  25.0f;
                if (pos_err < -25.0f) pos_err = -25.0f;

                #define ISR_KP_NEAR (0.015f)  /* A/deg — 误差<10°时，防抖 */
                #define ISR_KP_FAR  (0.030f)  /* A/deg — 误差>10°时，快速回位 */
                #define ISR_KD  (0.010f)   /* A/(deg/s) */
                #define ISR_KI  (0.0005f)  /* A/(deg·s) */
                #define ISR_KI_MAX (0.25f)
                #define ISR_IQ_MAX (0.45f)

                static float s_isr_integ = 0.0f;
                static float s_prev_pos_err = 0.0f;

                /* 非线性 KP：远处大力回位，近处轻柔防抖 */
                float kp_now = (fabsf(pos_err) > 10.0f) ? ISR_KP_FAR : ISR_KP_NEAR;

                /* 积分 */
                if (fabsf(pos_err) < 20.0f) {
                    s_isr_integ += ISR_KI * pos_err;
                    if (s_isr_integ >  ISR_KI_MAX) s_isr_integ =  ISR_KI_MAX;
                    if (s_isr_integ < -ISR_KI_MAX) s_isr_integ = -ISR_KI_MAX;
                }
                if (s_prev_pos_err * pos_err < 0.0f && fabsf(pos_err) < 3.0f) {
                    s_isr_integ *= 0.5f;
                }
                s_prev_pos_err = pos_err;

                iq_ref_cmd = kp_now * pos_err + s_isr_integ - ISR_KD * s_vel_isr_dps;
                if (iq_ref_cmd >  ISR_IQ_MAX) iq_ref_cmd =  ISR_IQ_MAX;
                if (iq_ref_cmd < -ISR_IQ_MAX) iq_ref_cmd = -ISR_IQ_MAX;
            }
        }

        bool zero_torque = (fabsf(iq_ref_cmd) < IQ_ZERO_DEADBAND_A);

        foc_controller_U.In_I_a     = s_iu_filt;
        foc_controller_U.In_I_b     = s_iv_filt;
        foc_controller_U.In_theta_e = elec_angle;
        foc_controller_U.In_iq_ref  = zero_torque ? 0.0f : iq_ref_cmd;
        foc_controller_U.In_V_bus   = vbus;
        bool reset_pi = (ctrl_state != STATE_RUN) || s_current_pi_reset_req || zero_torque;
        s_current_pi_reset_req = false;
        foc_controller_U.In_reset   = reset_pi;
        foc_controller_step();
        if (zero_torque) {
            foc_interface_write_ccr(0.5f, 0.5f, 0.5f);
        } else if (!s_pi_bypass) {
            foc_interface_write_ccr(
                foc_controller_Y.Out_duty_a,
                foc_controller_Y.Out_duty_b,
                foc_controller_Y.Out_duty_c
            );
        } else {
            /* PI 旁路：用编码器角度 + 直接电压注入（完全绕过 Simulink）*/
            float vq_bp = s_bypass_vq;
            float cos_t = cosf(elec_angle);
            float sin_t = sinf(elec_angle);
            float va = -vq_bp * sin_t;
            float vb =  vq_bp * cos_t;
            float v_u = va;
            float v_v = -0.5f * va + 0.866025f * vb;
            float v_w = -0.5f * va - 0.866025f * vb;
            float vn = -(fmaxf(fmaxf(v_u,v_v),v_w) + fminf(fminf(v_u,v_v),v_w)) / 2.0f;
            float da = (v_u + vn) / vbus + 0.5f;
            float db = (v_v + vn) / vbus + 0.5f;
            float dc = (v_w + vn) / vbus + 0.5f;
            foc_interface_write_ccr(da, db, dc);
        }
        /* --- Clarke + Park → Id_meas / Iq_meas ---
         * Clarke: Ialpha = Ia,  Ibeta = (Ia + 2*Ib) / sqrt(3)
         * Park:   Id = Ialpha*cos(θ) + Ibeta*sin(θ)
         *         Iq = -Ialpha*sin(θ) + Ibeta*cos(θ)
         * 此处 Ia=iu, Ib=iv（与 Simulink 输入一致）*/
        {
            float ia = iu;
            float ib = iv;
            float ialpha = ia;
            float ibeta  = (ia + 2.0f * ib) * 0.57735027f;  /* 1/sqrt(3) */
            float cos_t  = cosf(elec_angle);
            float sin_t  = sinf(elec_angle);
            float id = ialpha * cos_t + ibeta * sin_t;
            float iq = -ialpha * sin_t + ibeta * cos_t;
            s_meas_buf.id_meas = id;
            s_meas_buf.iq_meas = iq;

            /* --- dq 窗口统计累加（仅 STATE_RUN 且未 reset）--- */
            if (ctrl_state == STATE_RUN) {
                float iq_ref_now = foc_controller_U.In_iq_ref;
                s_stat_sum_id  += id;
                s_stat_sum_iq  += iq;
                s_stat_sum_iqr += iq_ref_now;
                s_stat_sum_id2 += id * id;
                s_stat_sum_iq2 += iq * iq;
                s_stat_sum_err += (iq_ref_now - iq);
                s_stat_cnt++;

                if (s_stat_cnt >= FOC_DQ_STAT_N) {
                    float n_inv = 1.0f / (float)s_stat_cnt;
                    FocDqStats_t tmp;
                    tmp.id_avg     = s_stat_sum_id  * n_inv;
                    tmp.iq_avg     = s_stat_sum_iq  * n_inv;
                    tmp.iq_ref_avg = s_stat_sum_iqr * n_inv;
                    tmp.iq_err_avg = s_stat_sum_err * n_inv;
                    float id2_avg  = s_stat_sum_id2 * n_inv;
                    float iq2_avg  = s_stat_sum_iq2 * n_inv;
                    tmp.id_rms = (id2_avg > 0.0f) ? sqrtf(id2_avg) : 0.0f;
                    tmp.iq_rms = (iq2_avg > 0.0f) ? sqrtf(iq2_avg) : 0.0f;
                    /* 关中断写 snapshot */
                    uint32_t prim = __get_PRIMASK();
                    __disable_irq();
                    s_dq_snap  = tmp;
                    s_dq_ready = true;
                    __set_PRIMASK(prim);
                    /* 清零累加器 */
                    s_stat_cnt = 0;
                    s_stat_sum_id = s_stat_sum_iq = s_stat_sum_iqr = 0.0f;
                    s_stat_sum_id2 = s_stat_sum_iq2 = s_stat_sum_err = 0.0f;
                }
            } else {
                /* 非 RUN 状态：清零累加器，避免状态切换后窗口混入无效样本 */
                s_stat_cnt = 0;
                s_stat_sum_id = s_stat_sum_iq = s_stat_sum_iqr = 0.0f;
                s_stat_sum_id2 = s_stat_sum_iq2 = s_stat_sum_err = 0.0f;
            }
        }
    }
#endif

    /* --- 帧计数 --- */
    s_adc_frame_cnt++;
}

/* ---------------------------------------------------------- */
void foc_interface_get_measurement(FocMeasurement_t *out)
{
    /* 临界区保护：禁止中断后整块拷贝，防止主循环读到 ISR 半更新的数据。
     * FocMeasurement_t = 5 * float = 20 bytes，关中断时间 < 1 us，可接受。*/
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_meas_buf;
    __set_PRIMASK(primask);
}

void foc_interface_set_target_rpm(float rpm)
{
    s_target_rpm = rpm;
}

void foc_interface_get_adc_raw(uint16_t *out)
{
    /* 不关中断直接拷贝。
     * uint16_t 读写在 Cortex-M4 上为原子操作，单元素不会撕裂。
     * 数组整体不保证原子，但校准（取均值）和故障检测（判饱和）
     * 对单次拷贝误差不敏感，此处不加临界区。 */
    for (uint8_t i = 0; i < FOC_ADC_CH_COUNT; i++) {
        out[i] = s_adc_raw_snap[i];
    }
}

uint32_t foc_interface_get_frame_count(void)
{
    return s_adc_frame_cnt;
}

bool foc_interface_get_dq_stats(FocDqStats_t *out)
{
    if (!s_dq_ready) return false;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    *out = s_dq_snap;
    s_dq_ready = false;
    __set_PRIMASK(prim);
    return true;
}

/* ---- 开环测试接口 ---- */
void foc_openloop_start(float elec_omega_rps, float vq_volt)
{
    s_openloop_theta = 0.0f;
    s_openloop_omega = elec_omega_rps;
    s_openloop_vq    = vq_volt;
    s_calib_sum_plus = 0.0f;
    s_calib_sum_minus = 0.0f;
    s_calib_cnt = 0;
    s_openloop_active = true;
}

void foc_openloop_stop(void)
{
    s_openloop_active = false;
    s_openloop_omega  = 0.0f;
    s_openloop_vq     = 0.0f;
}

bool foc_openloop_is_active(void)
{
    return s_openloop_active;
}

void foc_calib_get(float *avg_plus, float *avg_minus, uint32_t *cnt, float *enc_last)
{
    *cnt = s_calib_cnt;
    if (s_calib_cnt > 0) {
        *avg_plus  = s_calib_sum_plus  / (float)s_calib_cnt;
        *avg_minus = s_calib_sum_minus / (float)s_calib_cnt;
    } else {
        *avg_plus = 0.0f;
        *avg_minus = 0.0f;
    }
    *enc_last = s_calib_enc_last;
}
