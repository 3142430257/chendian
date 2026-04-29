/**
 * @file    encoder_pwm.h
 * @brief   AS5048A PWM 输入捕获驱动（TIM2_CH1 / PA15）
 *          角度换算：angle = high_count / total_count × 2π
 *          （避免 PWM 频率 ±10% 容差影响）
 */

#ifndef ENCODER_PWM_H
#define ENCODER_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化编码器模块（在 TIM2 HAL_Init 完成后调用）
 */
void encoder_pwm_init(void);

/**
 * @brief  TIM2 输入捕获回调（在 HAL_TIM_IC_CaptureCallback 中调用）
 *         内部维护 high_count / total_count，计算最新角度
 * @param  htim  HAL 定时器句柄指针
 */
void encoder_pwm_capture_callback(void *htim);

/* --- 以下为 encoder_if.h 接口的实现 --- */
float encoder_get_angle_rad(void);
float encoder_get_elec_angle_rad(uint8_t pole_pairs);
bool  encoder_is_valid(void);
uint32_t encoder_get_last_capture_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_PWM_H */
