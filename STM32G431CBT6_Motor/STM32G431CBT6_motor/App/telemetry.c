/**
 * @file    telemetry.c
 * @brief   低速串口遥测实现
 *
 * 输出格式（CSV，便于串口助手/上位机解析）：
 *   T,<tick>,IU,<iu_A>,IV,<iv_A>,IW,<iw_A>,VBUS,<vbus_V>,ANG,<ang_deg>,STATE,<state>
 *
 * 注意：
 *   - 本模块只在主循环调用，禁止在任何 ISR 中调用
 *   - 使用 HAL_UART_Transmit（阻塞）或 DMA 非阻塞均可，V1 先用阻塞
 */

#include "telemetry.h"
#include "board_config.h"
#include "foc_interface.h"
#include "encoder_if.h"
#include "app_control.h"
#include "mpu6050.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;   /* CubeMX 在 usart.c 生成 */
extern volatile uint8_t s_dbg_vec_phase;  /* foc_interface.c 暴露：固定矢量阶段 */

static uint32_t s_interval_ms  = 100U;
static uint32_t s_last_send_ms = 0U;

void telemetry_init(void)
{
    s_last_send_ms = HAL_GetTick();
}

void telemetry_set_interval_ms(uint32_t ms)
{
    s_interval_ms = ms;
}

void telemetry_update(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_send_ms) < s_interval_ms) return;
    s_last_send_ms = now;

    /* 读取最新测量值（由 foc_interface 在 ISR 中更新的快照）*/
    FocMeasurement_t meas;
    foc_interface_get_measurement(&meas);

    /* 读取 ADC 原始值用于诊断 */
    uint16_t adc_raw[FOC_ADC_CH_COUNT];
    foc_interface_get_adc_raw(adc_raw);

    float m_ang_deg = encoder_get_angle_rad() * (180.0f / 3.14159265f);
    float e_ang_deg = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS) * (180.0f / 3.14159265f);
    uint8_t state   = app_control_get_state();
    uint8_t fault   = app_control_get_fault();

    /* IWR: ADC Rank3 原始采样（低边 W 相 shunt），作诊断用
     * KVL_REC: 重构后三相电流之和，低边重构正确则应 ≈ 0
     * ENC_AGE: 自上次捕获到现在的 ms 数（应 < 2ms @ 1kHz）
     * ENC_VALID: encoder_is_valid() 实时返回值 */
    float iw_raw   = ADC_TO_CURRENT(adc_raw[FOC_ADC_IDX_IW], iw_offset_raw);
    float kvl_rec  = meas.iu_a + meas.iv_a + meas.iw_a;  /* 重构口径，理想=0 */
    uint32_t enc_age_ms = HAL_GetTick() - encoder_get_last_capture_ms();
    uint8_t  enc_valid  = encoder_is_valid() ? 1u : 0u;

    float pos_actual = app_control_get_pos_actual_deg();
    float pos_target = app_control_get_pos_target_deg();
    float pos_err    = pos_target - pos_actual;
    uint8_t ctrl_mode = (uint8_t)app_control_get_ctrl_mode();
    /* IMU_RDY=0 直到步骤③接入MPU6050；监控脚本据此决定是否画IMU曲线 */
    float imu_pitch  = mpu6050_get_pitch_deg();
    uint8_t imu_rdy  = mpu6050_is_ready() ? 1u : 0u;

    float spd_est    = app_control_get_omega_est();
    float spd_ref    = app_control_get_omega_ref();
    float iq_ref     = app_control_get_iq_ref();
    uint8_t spd_mode = app_control_get_speed_mode() ? 1u : 0u;
    /* dq 实测（Clarke+Park in ISR）*/
    float id_meas    = meas.id_meas;
    float iq_meas    = meas.iq_meas;

    char buf[540];
    int  len = snprintf(buf, sizeof(buf),
        "T,%lu,IU,%.3f,IV,%.3f,IW,%.3f,VBUS,%.2f,"
        "M_ANG,%.1f,E_ANG,%.1f,ST,%u,FAULT,%u,"
        "IWR,%.3f,KVL,%.3f,"
        "VEC,%u,"
        "ENC_AGE,%lu,ENC_VAL,%u,"
        "SPD_EST,%.2f,SPD_MODE,%u,SPD_REF,%.2f,IQ_REF,%.3f,"
        "ID_MEAS,%.3f,IQ_MEAS,%.3f,"
        "CTRL,%u,POS_TGT,%.2f,POS_ACT,%.2f,POS_ERR,%.2f,IMU,%.2f,IMU_RDY,%u,STAB_LIM,%.1f,IMU_HOME,%.2f,"
        "RAW,%u,%u,%u,%u,OFF,%u,%u,%u\r\n",
        (unsigned long)now,
        meas.iu_a, meas.iv_a, meas.iw_a,
        meas.vbus_v, m_ang_deg, e_ang_deg, state, fault,
        iw_raw, kvl_rec,
        (unsigned)s_dbg_vec_phase,
        (unsigned long)enc_age_ms, (unsigned)enc_valid,
        spd_est, (unsigned)spd_mode, spd_ref, iq_ref,
        id_meas, iq_meas,
        (unsigned)ctrl_mode, pos_target, pos_actual, pos_err, imu_pitch, (unsigned)imu_rdy,
        app_control_get_stab_lim(), app_control_get_imu_home(),
        adc_raw[0], adc_raw[1], adc_raw[2], adc_raw[3],
        iu_offset_raw, iv_offset_raw, iw_offset_raw);


    if (len > 0 && len < (int)sizeof(buf))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 50U);
    }

    /* 条件追加 dq 统计帧（每 50ms 固件端产生一次）*/
    FocDqStats_t dq;
    if (foc_interface_get_dq_stats(&dq)) {
        char buf2[180];
        int len2 = snprintf(buf2, sizeof(buf2),
            "DQ_STAT,ID_AVG,%.4f,IQ_AVG,%.4f,"
            "ID_RMS,%.4f,IQ_RMS,%.4f,"
            "IQREF_AVG,%.4f,IQERR_AVG,%.4f\r\n",
            dq.id_avg, dq.iq_avg,
            dq.id_rms, dq.iq_rms,
            dq.iq_ref_avg, dq.iq_err_avg);
        if (len2 > 0 && len2 < (int)sizeof(buf2)) {
            HAL_UART_Transmit(&huart3, (uint8_t *)buf2, (uint16_t)len2, 20U);
        }
    }
}
