/**
 * @file    foc_interface.c
 * @brief   FOC 算法输入/输出接口实现
 *
 * V1：骨架占位，完成 ADC → 物理量换算 + 写 CCR（固定50%占空比测试）
 * V2：接入 Simulink 生成的 foc_model_step()
 *
 * 调用顺序（ISR 内，< 20~25 μs）：
 *   1. ADC 原始值 → 物理量（IU/IV/IW/VBUS）
 *   2. 取编码器角度
 *   3. foc_model_step()（V1 跳过，V2 接入）
 *   4. 限幅后写 TIM1 CCR1/2/3
 */

#include "foc_interface.h"
#include "board_config.h"
#include "encoder_if.h"
#include "stm32g4xx_hal.h"
#include <string.h>

extern TIM_HandleTypeDef htim1;  /* CubeMX 在 tim.c 生成 */

/* 每相零偏（主程序初始化时测量）*/
uint16_t iu_offset_raw = 2048U;
uint16_t iv_offset_raw = 2048U;
uint16_t iw_offset_raw = 2048U;

/* 最新 ADC 原始值快照（ISR 内更新，主循环读取）*/
static volatile uint16_t s_adc_raw_snap[FOC_ADC_CH_COUNT] = {0};

/* 测量量快照（ISR 写，主循环读）*/
static FocMeasurement_t s_meas_buf = {0};
static volatile float   s_target_rpm = 0.0f;

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
void foc_interface_init(void)
{
    memset(&s_meas_buf, 0, sizeof(s_meas_buf));
    s_target_rpm = 0.0f;

    /* V1：上电先测零偏（确保电机静止、驱动板已上电但 CTRL_SD 未使能）
     * 实际项目中在 app_control 的 STATE_CALIBRATE 阶段完成
     * 这里先用默认中值 2048 */
}

/* ---------------------------------------------------------- */
void foc_interface_step(const uint16_t *adc_raw)
{
    /* 保存 ADC 原始快照（用于校准和故障检测）*/
    for (uint8_t i = 0; i < FOC_ADC_CH_COUNT; i++) {
        s_adc_raw_snap[i] = adc_raw[i];
    }

    /* --- Step 1: ADC → 物理量 --- */
    float iu = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IU], iu_offset_raw);
    float iv = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IV], iv_offset_raw);
    float iw = -(iu + iv);   /* 基尔霍夫：IW = -(IU+IV) */
    float vbus = ADC_TO_VBUS(adc_raw[FOC_ADC_IDX_VBUS]);

    /* --- Step 2: 编码器角度 --- */
    float angle = encoder_get_angle_rad();

    /* 更新测量快照（ISR 内直接写，主循环读时不关中断，V1 精度足够）*/
    s_meas_buf.iu_a     = iu;
    s_meas_buf.iv_a     = iv;
    s_meas_buf.iw_a     = iw;
    s_meas_buf.vbus_v   = vbus;
    s_meas_buf.angle_rad= angle;

    /* --- Step 3: FOC 算法（V1 跳过，V2 接入 foc_model_step()）--- */
    /*
     * TODO V2:
     *   foc_model_U.iu_a  = iu;
     *   foc_model_U.iv_a  = iv;
     *   foc_model_U.theta = angle;
     *   foc_model_U.vbus  = vbus;
     *   foc_model_step();
     *   float duty_u = foc_model_Y.duty_u;
     *   float duty_v = foc_model_Y.duty_v;
     *   float duty_w = foc_model_Y.duty_w;
     */

    /* --- Step 4: V1 固定 50% 占空比（安全测试用）--- */
    uint32_t ccr = clamp_ccr(0.5f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
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
