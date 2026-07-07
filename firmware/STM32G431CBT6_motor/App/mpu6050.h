/**
 * @file    mpu6050.h
 * @brief   MPU6050 6轴 IMU 驱动（软件 I2C bit-bang + 互补滤波器）
 *
 * 硬件连接：
 *   STM32  PB5 → MPU6050 SCL（软件 I2C，开漏模式）
 *   STM32  PB4 → MPU6050 SDA
 *   AD0 悬空或接 GND → 从机地址 0x68
 *   MPU6050 模块已自带上拉电阻，无需外加
 *
 * 使用流程：
 *   1. 在 main.c USER CODE BEGIN 2 调用 mpu6050_init()（无参数）
 *   2. 主循环每 10ms 调用 mpu6050_update()
 *   3. 读取 mpu6050_get_pitch_deg() 得到俯仰角 [°]
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- 软件 I2C 引脚（修改此处即可换引脚）---- */
#define SW_I2C_SCL_PORT   GPIOB
#define SW_I2C_SCL_PIN    GPIO_PIN_5   /* PB5 = SCL */
#define SW_I2C_SDA_PORT   GPIOB
#define SW_I2C_SDA_PIN    GPIO_PIN_4   /* PB4 = SDA */

/* ---- I2C 从机地址 ---- */
#define MPU6050_ADDR      (0x68 << 1)   /* AD0=GND/悬空 */

/* ---- 量程配置 ---- */
#define MPU6050_GYRO_FS   0   /* ±250 °/s  → 131.0 LSB/°/s */
#define MPU6050_ACCEL_FS  0   /* ±2 g       → 16384 LSB/g   */

/* ---- 互补滤波器参数 ---- */
#define COMP_ALPHA        0.98f   /* 陀螺仪权重 */
#define MPU6050_DT_S      0.010f  /* 更新周期 10 ms */

/**
 * @brief 初始化 MPU6050（配置 GPIO + 寄存器握手），无需传 I2C 句柄
 * @return true = 成功，false = 通信失败
 */
bool mpu6050_init(void);

/**
 * @brief 读取原始数据并更新互补滤波器（每 10 ms 调用一次）
 */
bool mpu6050_update(void);

/** @brief 获取 IMU 就绪标志 */
bool mpu6050_is_ready(void);

/** @brief 获取俯仰角 [°]（互补滤波输出） */
float mpu6050_get_pitch_deg(void);

/** @brief 获取陀螺仪角速度（pitch 轴）[°/s] */
float mpu6050_get_gyro_x_dps(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
