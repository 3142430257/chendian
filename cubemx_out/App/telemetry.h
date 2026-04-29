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
    /**
     * @brief  注册一个遥测数据项
     * @param  id     数据项ID（0-255）
     * @param  data   数据指针
     * @param  size   数据大小（字节）
     * @param  period 发送周期（0=禁用，1=每帧，N=每N帧）
     * @return 0=成功，-1=失败（ID重复或参数无效）
     */
    int telemetry_register(uint8_t id, const void* data, uint8_t size, uint8_t period);

    /**
     * @brief  手动触发一次遥测发送（立即发送所有已注册数据）
     * @note   建议仅在调试时使用，避免影响正常周期发送
     */
    void telemetry_trigger_now(void);

    /**
     * @brief  获取遥测模块状态
     * @param  tx_busy  输出：发送忙标志
     * @param  reg_count 输出：已注册数据项数量
     * @param  error_cnt 输出：发送错误计数
     */
    void telemetry_get_status(uint8_t* tx_busy, uint8_t* reg_count, uint16_t* error_cnt);
#endif /* TELEMETRY_H */
