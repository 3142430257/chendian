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
#include "encoder_if.h"
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
void foc_interface_step(const uint16_t *adc_raw)
{
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

    float iu, iv, iw;
    if (ccr_a >= ccr_b && ccr_a >= ccr_c) {
        /* U 相驱动（CCR最大），IU 采样无效，从 IV/IW 重构 */
        iv = -iv_shunt;
        iw = -iw_shunt;
        iu = -(iv + iw);
    } else if (ccr_b >= ccr_a && ccr_b >= ccr_c) {
        /* V 相驱动，IV 采样无效，从 IU/IW 重构 */
        iu = -iu_shunt;
        iw = -iw_shunt;
        iv = -(iu + iw);
    } else {
        /* W 相驱动，IW 采样无效，从 IU/IV 重构 */
        iu = -iu_shunt;
        iv = -iv_shunt;
        iw = -(iu + iv);
    }

    float vbus = ADC_TO_VBUS(adc_raw[FOC_ADC_IDX_VBUS]);


    /* 更新测量快照（所有模式都需要，保证遥测和保护看到实时值）*/
    s_meas_buf.iu_a      = iu;
    s_meas_buf.iv_a      = iv;
    s_meas_buf.iw_a      = iw;
    s_meas_buf.vbus_v    = vbus;

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

#else
    /* --- Step 2: 编码器电角度 --- */
    uint8_t ctrl_state = app_control_get_state();
    float elec_angle;
    s_dbg_vec_phase = 0xFF;  /* 非调试模式标记 */

    if (ctrl_state == STATE_ALIGN) {
        /* 对齐模式：强制 theta_e = 0 ，居中转子到电气零点
         * 直接输出固定 SVPWM（不经 PI），Vd=0.3V，Vq=0。
         * SVPWM @theta=0: duty_a=0.5+0.75*Vd/Vbus, duty_b=duty_c=0.5-0.75*Vd/Vbus */
        float vd  = app_control_get_align_id_ref();   /* 在这里含义是 Vd [V] */
        float d   = (vbus > 1.0f) ? (0.75f * vd / vbus) : 0.0f;
        foc_interface_write_ccr(0.5f + d, 0.5f - d, 0.5f - d);
        /* 重置积分器 */
        foc_controller_U.In_reset = true;
        foc_controller_U.In_I_a = iu; foc_controller_U.In_I_b = iv;
        foc_controller_U.In_theta_e = 0.0f; foc_controller_U.In_iq_ref = 0.0f;
        foc_controller_U.In_V_bus = vbus;
        foc_controller_step();
    } else {
        /* 编码器方向测试版本: 直接使用 raw_e（不反转）
         * 如果电机方向仍不对，再恢复 (2π - raw_e) 版本 */
        float raw_e  = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
        float offset = app_control_get_theta_offset();
        elec_angle   = raw_e - offset;
        while (elec_angle <  0.0f)               elec_angle += 2.0f * 3.14159265f;
        while (elec_angle >= 2.0f * 3.14159265f) elec_angle -= 2.0f * 3.14159265f;

        s_meas_buf.angle_rad = elec_angle;

        foc_controller_U.In_I_a     = iu;
        foc_controller_U.In_I_b     = iv;
        foc_controller_U.In_theta_e = elec_angle;
        foc_controller_U.In_iq_ref  = app_control_get_iq_ref();
        foc_controller_U.In_V_bus   = vbus;
        foc_controller_U.In_reset   = (ctrl_state != STATE_RUN);
        foc_controller_step();
        foc_interface_write_ccr(
            foc_controller_Y.Out_duty_a,
            foc_controller_Y.Out_duty_b,
            foc_controller_Y.Out_duty_c
        );
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
