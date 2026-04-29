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
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;   /* CubeMX 在 usart.c 生成 */

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

    float angle_deg = encoder_get_angle_rad() * (180.0f / 3.14159265f);
    uint8_t state   = app_control_get_state();

    char buf[128];
    int  len = snprintf(buf, sizeof(buf),
        "T,%lu,IU,%.3f,IV,%.3f,IW,%.3f,VBUS,%.2f,ANG,%.1f,STATE,%u\r\n",
        (unsigned long)now,
        meas.iu_a, meas.iv_a, meas.iw_a,
        meas.vbus_v, angle_deg, state);

    if (len > 0 && len < (int)sizeof(buf))
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 10U);
    }
}
