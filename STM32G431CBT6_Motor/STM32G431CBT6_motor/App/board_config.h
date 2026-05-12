/**
 * @file    board_config.h
 * @brief   板级硬件参数定义
 *          所有数值均已对照 ATK-PD6010B V1.0 驱动板原理图确认
 *          主控：X Pulse STM32G431CBT6 v2.5，HSE = 12 MHz
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================
 * 时钟（CubeMX 配置参考）
 * HSE = 12 MHz → PLL → SYSCLK = 170 MHz
 * ============================================================ */
#define BOARD_HSE_MHZ       (12U)
#define BOARD_SYSCLK_MHZ    (170U)

/* ============================================================
 * PWM / TIM1
 * 中心对齐，20 kHz
 * ARR = 170 000 000 / (2 × 20 000) - 1 = 4249
 * ============================================================ */
#define TIM1_ARR            (4249U)
#define TIM1_PWM_FREQ_HZ    (20000U)
#define TIM1_PERIOD_US      (50U)       /* 控制周期 50 μs */

/* 死区 @170MHz，1 cnt ≈ 5.88 ns
 * DT=51 → ~300 ns（起步保守值）
 * 示波器确认无直通后可收紧至 DT=34(~200ns) */
#define TIM1_DEADTIME_CNT   (51U)

/* CCR 限幅：写 CCR 前必须钳制在 [CCR_MIN, CCR_MAX]
 * 留 ~2% 安全边距，禁止打到 0% / 100%
 * MIN = ARR * 0.02 ≈ 85;  MAX = ARR * 0.98 ≈ 4164 */
#define TIM1_CCR_MARGIN_PCT (0.02f)
#define TIM1_CCR_MIN        ((uint32_t)(TIM1_ARR * TIM1_CCR_MARGIN_PCT))        /* ~85  */
#define TIM1_CCR_MAX        ((uint32_t)(TIM1_ARR * (1.0f - TIM1_CCR_MARGIN_PCT))) /* ~4164 */

/* ============================================================
 * ADC 基础
 * ============================================================ */
#define ADC_REF_V           (3.3f)
#define ADC_RESOLUTION      (4095.0f)
#define ADC_TO_VOLT(raw)    ((float)(raw) * ADC_REF_V / ADC_RESOLUTION)

/* ============================================================
 * 相电流采样
 * 原理图标注：Diff Gain = 6, 采样电阻 R17/R57/R87 = 20 mΩ
 * Iout = 6 × 0.02 × I + 1.25  →  I = (Vadc - 1.25) / 0.12  [A]
 * ============================================================ */
#define AMP_SHUNT_OHM       (0.020f)                       /* 采样电阻 */
#define AMP_DIFF_GAIN       (6.0f)                         /* 差分增益 */
#define AMP_GAIN_V_PER_A    (AMP_DIFF_GAIN * AMP_SHUNT_OHM) /* 0.12 V/A */
#define AMP_OFFSET_V        (1.25f)                        /* VCC1.25 偏置 */

/* 每相独立零偏（上电静止时校准，主程序初始化阶段写入）*/
extern uint16_t iu_offset_raw;
extern uint16_t iv_offset_raw;
extern uint16_t iw_offset_raw;

/**
 * @brief  ADC 原始值 → 相电流 [A]
 * @param  raw        ADC 原始采样值 (0~4095)
 * @param  offset_raw 该相零偏原始值（静止时标定）
 */
#define ADC_TO_CURRENT(raw, offset_raw) \
    (((float)(raw) - (float)(offset_raw)) * ADC_REF_V / ADC_RESOLUTION / AMP_GAIN_V_PER_A)

/* ============================================================
 * 母线电压
 * 分压电路：POWER → R112(12K) → R113(12K) → ADC_pin → R115(1K) → GND
 * 分压比 = (12+12+1) / 1 = 25
 * ============================================================ */
#define VBUS_DIVIDER        (25.0f)
#define ADC_TO_VBUS(raw)    (ADC_TO_VOLT(raw) * VBUS_DIVIDER)

/* VBUS 软件保护阈值 */
#define VBUS_MIN_V          (4.5f)   /* 放宽到4.5V：适配非标电源和分压比误差 */
#define VBUS_MAX_V          (26.0f)

/* ============================================================
 * 温度采样（NTC）
 * 分压电路：VCC3.3 → NTC(Rt) → VTEMP_pin → 4.7kΩ → GND
 * 原理图注释：VTEMP = 3.3 × (Rt + 4.7) / 4.7
 * 正确推导：Vadc = 3.3×4.7/(Rt+4.7)  →  Rt = 4.7×(Vcc-Vadc)/Vadc  [kΩ]
 * NTC：NCP18XH103F03RB，10K@25°C，B≈3380K（需查手册确认）
 * ============================================================ */
#define VTEMP_PULLUP_K      (4.7f)
#define VTEMP_NTC_B         (3380.0f)   /* B 值，需核对 NCP18XH103 datasheet */
#define VTEMP_NTC_25C_K     (10.0f)
#define VTEMP_TEMP_LIMIT_C  (80.0f)     /* 温度保护阈值 */

/* ============================================================
 * 电机参数
 * 注意：MOTOR_POLE_PAIRS 必须与 Simulink foc_params.m 中 p 保持一致
 *       改动时两处同步修改，否则 Park 变换角度错误
 * ============================================================ */
#define MOTOR_POLE_PAIRS    (11U)   /* GM3506: 24N/22P → 22极/2 = 11极对数 */

/* ============================================================
 * 编码器（AS5048A PWM 模式）
 * PWM 频率 ~1kHz，±10% 容差 → 必须用比值法换算
 * angle_rad = (high_count / total_count) × 2π
 * ============================================================ */
#define ENCODER_TIMEOUT_MS  (200U)  /* 200ms：编码器 1kHz，EMI 严重时放宽，由跳变过滤器兜底 */

/* ============================================================
 * 引脚逻辑
 * ============================================================ */
#define CTRL_SD_ENABLE()    /* 由 gpio.c MX_GPIO_Init 后调用 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET) */
#define CTRL_SD_DISABLE()   /* HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET) */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
