/**
 * @file    openloop_stab.h
 * @brief   开环自稳模块（临时方案，编码器修好后切回闭环 FOC）
 *
 * 工作原理：IMU 角度 → PD 控制器 → 开环电角速度 → SVPWM
 * 不依赖编码器，不修改原有 app_control / foc_interface 闭环路径。
 *
 * 使用方式：
 *   发送 'L' → 启动开环自稳
 *   发送 'X' → 停止
 *   原闭环路径（a/e/S/T/G 等命令）完全不受影响
 */

#ifndef OPENLOOP_STAB_H
#define OPENLOOP_STAB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief  启动开环自稳模式
 *         自动使能驱动板、以当前 IMU 角度为零点
 */
void openloop_stab_start(void);

/**
 * @brief  停止开环自稳，关闭驱动
 */
void openloop_stab_stop(void);

/**
 * @brief  主循环调用（1kHz），执行 PD 控制
 *         未激活时立即返回，零开销
 */
void openloop_stab_update(void);

/** @brief  是否处于开环自稳模式 */
bool openloop_stab_is_active(void);

/** @brief  获取当前控制输出的机械角速度 [°/s]，供遥测显示 */
float openloop_stab_get_omega_dps(void);

/* ---- 运行时调参 ---- */
void  openloop_stab_set_kp(float kp);
void  openloop_stab_set_kd(float kd);
void  openloop_stab_set_vq(float vq);
float openloop_stab_get_kp(void);
float openloop_stab_get_kd(void);
float openloop_stab_get_vq(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENLOOP_STAB_H */
