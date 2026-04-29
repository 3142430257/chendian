/**
 * @file    app_control.h
 * @brief   系统状态机：上电 → 校准 → 就绪 → 运行 → 故障
 *          软件故障保护（VBUS/温度/编码器/ADC 越界），主循环调用
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** @brief 系统状态枚举 */
typedef enum {
    STATE_INIT      = 0,  /* 上电初始化 */
    STATE_CALIBRATE = 1,  /* 零偏校准（电机静止）*/
    STATE_READY     = 2,  /* 就绪，等待使能命令 */
    STATE_RUN       = 3,  /* 正常运行 */
    STATE_FAULT     = 4,  /* 故障，CTRL_SD 已拉低 */
} AppState_t;

/** @brief 故障码（可叠加，按位）*/
typedef enum {
    FAULT_NONE          = 0x00,
    FAULT_VBUS_LOW      = 0x01,
    FAULT_VBUS_HIGH     = 0x02,
    FAULT_TEMP          = 0x04,
    FAULT_ENCODER       = 0x08,
    FAULT_ADC_SATURATE  = 0x10,
} FaultCode_t;

/**
 * @brief  初始化控制模块（在所有外设 Init 完成后调用）
 */
void app_control_init(void);

/**
 * @brief  主循环状态机更新（约 1 kHz，非 ISR）
 */
void app_control_update(void);

/** @brief 获取当前状态 */
uint8_t app_control_get_state(void);

/** @brief 获取当前故障码 */
uint8_t app_control_get_fault(void);

/**
 * @brief  请求使能电机（仅 STATE_READY 有效）
 * @return true  = 使能成功
 *         false = 前提条件不满足（VBUS/编码器异常）
 */
bool app_control_enable(void);

/** @brief 外部触发停机 */
void app_control_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
