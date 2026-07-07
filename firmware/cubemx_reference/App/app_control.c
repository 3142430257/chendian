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
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>   /* logf() for NTC temperature calculation */

/* 外部 GPIO（CubeMX 在 gpio.c 生成）*/
extern void MX_GPIO_Init(void);  /* 仅供参考，实际已由 main 调用 */

/* ============================================================
 * 内部状态
 * ============================================================ */
static AppState_t s_state = STATE_INIT;
static uint8_t    s_fault = FAULT_NONE;

/* 零偏采样窗口 */
#define CALIB_SAMPLES   (256U)
static uint32_t s_calib_sum_iu = 0;
static uint32_t s_calib_sum_iv = 0;
static uint32_t s_calib_sum_iw = 0;
static uint16_t s_calib_cnt    = 0;

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
    s_calib_cnt  = 0;
    s_calib_sum_iu = s_calib_sum_iv = s_calib_sum_iw = 0;

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
        break;
    }

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
        s_fault = check_faults(&meas, adc_raw);

        if (s_fault != FAULT_NONE)
        {
            ctrl_sd_set(false);
            s_state = STATE_FAULT;
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
    ctrl_sd_set(true);
    s_state = STATE_RUN;
    return true;
}

void app_control_disable(void)
{
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
