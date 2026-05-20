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
    STATE_ALIGN     = 2,  /* 编码器对齐（d 轴锁转子）*/
    STATE_READY     = 3,  /* 就绪，等待使能命令 */
    STATE_RUN       = 4,  /* 正常运行 */
    STATE_FAULT     = 5,  /* 故障，CTRL_SD 已拉低 */
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
 * @brief  获取当前 q 轴目标电流指令 [A]
 *         供 ADC DMA ISR 中的 foc_interface_step 调用
 *         STATE_RUN 时返回 s_iq_ref_a，其他状态返回 0.0f
 */
float app_control_get_iq_ref(void);

/**
 * @brief  设置 q 轴目标电流 [A]（速度环输出或外部指令）
 * @param  iq_ref_a  目标 Iq [A]，建议在主循环或速度环中写入
 */
void app_control_set_iq_ref(float iq_ref_a);

/**
 * @brief  请求使能电机（仅 STATE_READY 有效）
 * @return true  = 使能成功
 *         false = 前提条件不满足（VBUS/编码器异常）
 */
bool app_control_enable(void);

/** @brief 外部触发停机 */
void app_control_disable(void);

/**
 * @brief  介入对齐（仅 STATE_READY 有效，内含安全检查）
 * @return true = 对齐已开始，false = 驱动板/编码器未就绪
 */
bool app_control_start_align(void);

/** @brief 获取对齐电压 Vd [V]，STATE_ALIGN 时返回有效值，其他状态返回 0 */
float app_control_get_align_id_ref(void);

/** @brief 获取对齐后的电角度偏移 [rad]，定义：theta_e = pp*theta_m - offset_e */
float app_control_get_theta_offset(void);
void  app_control_set_fine_offset(float rad);
float app_control_get_fine_offset(void);

/* ============================================================
 * 速度环接口（V1：纯比例起步）
 * ============================================================ */

/** @brief  进入/退出速度模式。进入时自动清积分+斜坡从当前速度开始 */
void app_control_set_speed_mode(bool enable);

/** @brief  设置速度目标 [rad/s]（速度模式下有效） */
void app_control_set_omega_ref(float omega_rad_s);

/** @brief  获取固件内速度估算值 [rad/s] */
float app_control_get_omega_est(void);

/** @brief  获取当前速度参考（斜坡输出）[rad/s] */
float app_control_get_omega_ref(void);


/** @brief  当前是否处于速度模式 */
bool app_control_get_speed_mode(void);

/* ============================================================
 * 位置控制接口（Step 1：单轴位置保持）
 * ============================================================ */

/** @brief 控制模式枚举 */
typedef enum {
    CTRL_SPEED = 0,   /**< 速度模式（原有） */
    CTRL_POSITION,    /**< 位置保持：目标来自串口 P<deg> */
    CTRL_STABILIZE    /**< 自稳模式：目标来自 -IMU_pitch（步骤④启用）*/
} CtrlMode_t;

/** 方向符号 — 首次调试确认，防止补偿反向 */
#define MOTOR_SIGN      (+1.0f)   /**< 编码器正向 */
#define IMU_SIGN        (+1.0f)   /**< IMU俯仰正向：同上 */

/** 位置软/硬限位角度（单位：度） */
#define POS_SOFT_LIM_DEG   (90.0f)   /**< 目标 clamp 上限 */
#define POS_WARN_LIM_DEG  (100.0f)   /**< 实际角预警 + 限速 30% */
#define POS_HARD_LIM_DEG  (360.0f)   /**< 临时关闭硬限位，专心调速度环 */

/** @brief 切换控制模式（自动清积分和速度滤波器）*/
void app_control_set_ctrl_mode(CtrlMode_t mode);

/** @brief 获取当前控制模式 */
CtrlMode_t app_control_get_ctrl_mode(void);

/**
 * @brief 设置位置模式目标角 [deg]
 * @note  自动 clamp 到 ±POS_SOFT_LIM_DEG，不切换模式
 */
void app_control_set_pos_target(float deg);

/**
 * @brief 回零：以当前位置为 0°，并立即进入 CTRL_POSITION
 * @note  必须在载荷处于水平位置时调用！这是自稳模式成立的前提。
 */
void app_control_home(void);

/** @brief 获取当前实际角（相对 home，带 MOTOR_SIGN）[deg] */
float app_control_get_pos_actual_deg(void);

/** @brief 获取当前位置目标角 [deg] */
float app_control_get_pos_target_deg(void);

/** @brief ISR 用：获取位置目标 [deg] */
float app_control_get_pos_target(void);
/** @brief ISR 用：获取 home 偏移 [deg] */
float app_control_get_pos_home_offset(void);
/** @brief ISR 用：是否处于位置跟踪模式 */
bool app_control_is_pos_tracking(void);

/* ---- 运行时调参接口 ---- */
void app_control_set_kp_pos(float kp);           /**< 位置 P 增益 */
void app_control_set_kd_pos(float kd);           /**< 位置 D 增益（用于滤波速度）*/
void app_control_set_pos_spd_lim(float spd_dps); /**< 位置环输出速度限幅 [°/s] */
void app_control_set_pos_deadband(float dz_deg); /**< 位置死区 [deg] */

/* 调参读取（返回实际生效值，已经过 clamp） */
float app_control_get_kp_pos(void);
float app_control_get_kd_pos(void);
float app_control_get_pos_spd_lim(void);
float app_control_get_pos_deadband(void);

/**
 * @brief 自稳目标软限幅 [°]：默认 10°，初次发 G 命令前先用 GLIM<val> 设好再测
 * 下限 2°，上限 90°，可运行时逐步调外。
 */
void  app_control_set_stab_lim(float deg);
float app_control_get_stab_lim(void);
/** @brief 获取 H 命令时记录的 IMU pitch 偏置 [°]，供遊测确认 */
float app_control_get_imu_home(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
