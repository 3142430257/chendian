/**
 * @file    openloop_stab.c
 * @brief   开环自稳模块实现（临时方案）
 *
 * 控制链路：
 *   IMU pitch → PD → 目标机械角速度 → ×极对数 → 开环电角速度 → foc_openloop
 *
 * 与原闭环路径完全隔离：
 *   - 不调用 app_control_enable/start_align
 *   - 不修改 STATE 状态机
 *   - 仅通过 foc_openloop_start/stop 控制电机
 *   - 编码器修好后删掉本文件即可
 */

#include "openloop_stab.h"
#include "foc_interface.h"
#include "mpu6050.h"
#include "board_config.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ---- 控制参数（可运行时调节）---- */
static float s_kp  = 0.8f;    /* 位置 P [（°/s) / °]  （降低，避免过冲） */
static float s_kd  = 0.3f;    /* 阻尼 D [（°/s) / (°/s)]  （直接用陀螺仪做阻尼） */
static float s_vq  = 0.5f;    /* 施加电压 Vq [V]         */

/* ---- 内部状态 ---- */
static bool  s_active       = false;
static float s_imu_home_deg = 0.0f;   /* 启动时记录的 IMU 角度 */
static float s_omega_out    = 0.0f;   /* 当前输出机械角速度 [°/s] */

/* 输出限幅 */
#define OMEGA_MAX_DPS  (30.0f)   /* 最大机械速度 [°/s]（降低，减小振荡幅度）*/
#define DEG2RAD        (0.01745329f)
#define DEADBAND_DEG   (0.5f)    /* 角度死区 [°]，误差小于此值不输出 */

/* ---- API 实现 ---- */

void openloop_stab_start(void)
{
    if (s_active) return;

    /* 以当前 IMU 角度为零点 */
    s_imu_home_deg = mpu6050_get_pitch_deg();
    s_omega_out    = 0.0f;

    /* 使能驱动板 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    /* 启动开环（初始速度 0，PD 会逐步修正）*/
    foc_openloop_start(0.0f, s_vq);

    s_active = true;
}

void openloop_stab_stop(void)
{
    if (!s_active) return;
    s_active = false;

    foc_openloop_stop();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    s_omega_out = 0.0f;
}

void openloop_stab_update(void)
{
    if (!s_active) return;

    /* IMU 离线保护：停止电机，避免失控 */
    if (!mpu6050_is_ready()) {
        foc_openloop_start(0.0f, 0.0f);  /* 停转但保持模式 */
        s_omega_out = 0.0f;
        return;
    }

    /* 读 IMU 当前角度和角速度 */
    float pitch_now = mpu6050_get_pitch_deg();
    float gyro_dps  = mpu6050_get_gyro_x_dps();  /* 直接用陀螺仪做阻尼 */

    float err_deg = 0.0f - (pitch_now - s_imu_home_deg);  /* 目标=0°（保持启动时姿态）*/

    /* 角度死区：误差太小不输出，避免抖动 */
    if (fabsf(err_deg) < DEADBAND_DEG && fabsf(gyro_dps) < 2.0f) {
        foc_openloop_start(0.0f, 0.0f);
        s_omega_out = 0.0f;
        return;
    }

    /* PD 控制：P=角度误差，D=陀螺仪角速度（直接用硬件测量，不微分）*/
    float omega_dps = s_kp * err_deg - s_kd * gyro_dps;

    /* 限幅 */
    if (omega_dps >  OMEGA_MAX_DPS) omega_dps =  OMEGA_MAX_DPS;
    if (omega_dps < -OMEGA_MAX_DPS) omega_dps = -OMEGA_MAX_DPS;

    s_omega_out = omega_dps;

    /* 机械角速度 [°/s] → [rad/s] → 电角速度 [rad/s] */
    float omega_mech_rps = omega_dps * DEG2RAD;
    float omega_elec_rps = omega_mech_rps * (float)MOTOR_POLE_PAIRS;

    /* 更新开环速度（Vq 方向跟随速度符号）*/
    float vq_signed = (omega_elec_rps >= 0.0f) ? s_vq : -s_vq;
    float omega_abs = fabsf(omega_elec_rps);

    /* 非常小的速度请求 → 停住（避免抖动）*/
    if (omega_abs < 0.5f) {
        omega_abs  = 0.0f;
        vq_signed  = 0.0f;
    }

    foc_openloop_start(omega_abs, vq_signed);
}

bool openloop_stab_is_active(void)
{
    return s_active;
}

float openloop_stab_get_omega_dps(void)
{
    return s_omega_out;
}

/* ---- 调参接口 ---- */
void  openloop_stab_set_kp(float kp) { if (kp >= 0.0f && kp <= 50.0f) s_kp = kp; }
void  openloop_stab_set_kd(float kd) { if (kd >= 0.0f && kd <= 5.0f)  s_kd = kd; }
void  openloop_stab_set_vq(float vq) { if (vq > 0.0f  && vq <= 2.0f)  s_vq = vq; }
float openloop_stab_get_kp(void) { return s_kp; }
float openloop_stab_get_kd(void) { return s_kd; }
float openloop_stab_get_vq(void) { return s_vq; }
