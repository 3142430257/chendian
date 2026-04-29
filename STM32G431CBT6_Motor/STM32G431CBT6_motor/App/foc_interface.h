/**
 * @file    foc_interface.h
 * @brief   FOC 算法输入/输出接口
 *          负责 ADC 原始值 → 物理量转换，以及 Simulink 模型 I/O 对接
 *          在 ADC DMA Complete ISR 中调用 foc_interface_step()
 */

#ifndef FOC_INTERFACE_H
#define FOC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** @brief  测量量快照（ISR 更新，主循环读取）*/
typedef struct {
    float iu_a;     /* 相电流 U [A] */
    float iv_a;     /* 相电流 V [A] */
    float iw_a;     /* 相电流 W [A]（= -(IU+IV)，V1 可选实测）*/
    float vbus_v;   /* 母线电压 [V] */
    float angle_rad;/* 电角度 [rad]，由 encoder_get_elec_angle_rad() 更新 */
    float id_meas;  /* d轴实测电流 [A]（Clarke+Park变换后）*/
    float iq_meas;  /* q轴实测电流 [A]（Clarke+Park变换后）*/
} FocMeasurement_t;

/**
 * @brief  初始化 FOC 接口模块
 *         在所有外设 Init 完成后、ADC DMA 启动前调用
 */
void foc_interface_init(void);

/**
 * @brief  ADC DMA Complete 回调中调用（最高优先级 ISR）
 *         执行：ADC原始值 → 物理量 → foc_model_step() → 写 TIM1 CCR
 * @param  adc_raw  ADC DMA 缓冲区指针（5路：IU/IV/IW/VBUS/VTEMP）
 */
void foc_interface_step(const uint16_t *adc_raw);

/**
 * @brief  获取最新测量量快照（主循环/telemetry 调用）
 *         内部使用原子拷贝，避免数据撕裂
 */
void foc_interface_get_measurement(FocMeasurement_t *out);

/**
 * @brief  设置目标转速 [rpm]（外部命令输入）
 */
void foc_interface_set_target_rpm(float rpm);

/**
 * @brief  获取最新 ADC 原始快照（ISR 内更新，主循环/校准阶段读取）
 * @param  out  大小至少 FOC_ADC_CH_COUNT 个 uint16_t 的缓冲区
 * @note   不关中断，局部撤裂风险对于校准和故障检测可接受
 */
void foc_interface_get_adc_raw(uint16_t *out);

/**
 * @brief  获取 ADC/FOC 快环中断的累计帧计数，用于应用层时基同步
 */
uint32_t foc_interface_get_frame_count(void);

/**
 * @brief  将归一化占空比写入 TIM1 CCR1/2/3（V3 Simulink 输出接口）
 *         内部执行限幅 [TIM1_CCR_MIN, TIM1_CCR_MAX]，绝不打满
 * @param  duty_a/b/c  归一化占空比 [0.0, 1.0]
 */
void foc_interface_write_ccr(float duty_a, float duty_b, float duty_c);

/**
 * @brief  dq 窗口统计快照（ISR 每 N 帧计算一次，主循环读取）
 * @note   N = FOC_DQ_STAT_N = 1000 帧，约 50ms @20kHz
 */
#define FOC_DQ_STAT_N  (1000U)

typedef struct {
    float id_avg;     /* Id 均值 [A] */
    float iq_avg;     /* Iq 均值 [A] */
    float id_rms;     /* Id RMS [A] */
    float iq_rms;     /* Iq RMS [A] */
    float iq_ref_avg; /* IqRef 均值 [A] */
    float iq_err_avg; /* IqRef - IqMeas 均值 [A]（正=电流环欠驱动）*/
} FocDqStats_t;

/**
 * @brief  读取最新 dq 统计快照
 * @param  out  输出缓冲区（调用者提供）
 * @return true  = 有新窗口数据，out 已更新，ready 标志已清除
 *         false = 当前窗口未完成，out 不变
 */
bool foc_interface_get_dq_stats(FocDqStats_t *out);

/** @brief ADC 通道索引（与 CubeMX ADC 序列顺序一致）*/
#define FOC_ADC_IDX_IU    (0U)
#define FOC_ADC_IDX_IV    (1U)
#define FOC_ADC_IDX_IW    (2U)
#define FOC_ADC_IDX_VBUS  (3U)
#define FOC_ADC_IDX_VTEMP (4U)
#define FOC_ADC_CH_COUNT  (5U)

#ifdef __cplusplus
}
#endif

#endif /* FOC_INTERFACE_H */
