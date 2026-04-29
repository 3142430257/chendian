/**
 * @file    telemetry.h
 * @brief   低速串口遥测（主循环调用，禁止在 ISR 中调用）
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 初始化遥测模块（USART3 初始化后调用）*/
void telemetry_init(void);

/**
 * @brief  主循环调用（约 1 kHz，串口带宽允许降低频率）
 *         内部按配置的输出周期决定是否发送
 */
void telemetry_update(void);

/** @brief 设置输出间隔 [ms]，默认 100 ms */
void telemetry_set_interval_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
