/**
 * @file    encoder_if.h
 * @brief   编码器抽象接口
 *          上层代码只调用此接口，不感知底层是 PWM 还是 SPI
 */

#ifndef ENCODER_IF_H
#define ENCODER_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  获取最新机械角（弧度，[0, 2π)）
 *         由 TIM2 PWM 输入捕获回调更新，主控制链读取此值
 */
float encoder_get_angle_rad(void);

/**
 * @brief  获取最新电角度（弧度，[0, 2π)）
 *         = 机械角 × 极对数
 * @param  pole_pairs  电机极对数
 */
float encoder_get_elec_angle_rad(uint8_t pole_pairs);

/**
 * @brief  编码器信号是否有效（未超时）
 * @return true = 信号正常，false = 信号丢失/超时
 */
bool encoder_is_valid(void);

/**
 * @brief  获取距上次有效捕获的时间 [ms]（用于超时检测）
 */
uint32_t encoder_get_last_capture_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_IF_H */
