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

/** @brief  测量量快照（ISR 更新，主循环读取）*/
typedef struct {
    float iu_a;     /* 相电流 U [A] */
    float iv_a;     /* 相电流 V [A] */
    float iw_a;     /* 相电流 W [A]（= -(IU+IV)，V1 可选实测）*/
    float vbus_v;   /* 母线电压 [V] */
    float angle_rad;/* 机械角 [rad] */
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
 * @brief  ADC 通道索引（与 CubeMX ADC 序列顺序一致）*/
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
