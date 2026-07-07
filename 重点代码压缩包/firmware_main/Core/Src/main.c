/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "app_control.h"
#include "encoder_spi.h"
#include "foc_interface.h"
#include "telemetry.h"
#include "mpu6050.h"       /* IMU 驱动（I2C1，CubeMX 使能后生效）*/
#include "openloop_stab.h" /* 开环自稳（临时方案，编码器修好后删除）*/
#include "board_config.h"  /* MOTOR_POLE_PAIRS */
#include "encoder_if.h"    /* encoder_get_angle_rad */
#include "spi.h"
/* 注意：CubeMX 生成 i2c.h 后下一行自动插入 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint16_t adc_raw_buffer[FOC_ADC_CH_COUNT]; // 5路采样
static uint8_t  rx_byte = 0;                       // 串口接收缓冲（单字节轮询）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── USART3 RX 环形缓冲区（ISR 写头，主循环读尾，SPSC 无锁）── */
#define UART_RX_RING_MASK  (63U)
static volatile uint8_t s_rx_ring[64U];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;

void serial_rx_push(uint8_t c)
{
    uint8_t next = (uint8_t)((s_rx_head + 1U) & UART_RX_RING_MASK);
    if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = c;
        s_rx_head = next;
    }
}

/* ── 行级命令解析 ── */

static bool parse_float(const char *s, float *out);  /* 前向声明 */

static void cmd_process_line(char *line)
{
    extern UART_HandleTypeDef huart3;
    static float cmd_omega_ref = 0.0f;
    static float cmd_iq_ref    = 0.0f;
    char resp[256];
    int  rlen = 0;

    static const char KNOWN_SINGLES[] = "edas+-][0EOXLQ";
    if (line[0] != '\0' && line[1] == '\0' &&
        strchr(KNOWN_SINGLES, line[0]) != NULL) {
        char c = line[0];
        switch (c) {
        case 'e':
            cmd_iq_ref = 0.0f; cmd_omega_ref = 0.0f;
            app_control_set_iq_ref(0.0f);
            app_control_set_speed_mode(false);
            if (app_control_enable())
                rlen = snprintf(resp, sizeof(resp), "[CMD] ENABLED, ST=%u\r\n", app_control_get_state());
            else {
                uint8_t st = app_control_get_state();
                FocMeasurement_t dbg_m; foc_interface_get_measurement(&dbg_m);
                rlen = snprintf(resp, sizeof(resp),
                    "[CMD] REJECT: ST=%u VBUS=%.1f ENC=%u\r\n",
                    st, dbg_m.vbus_v, encoder_is_valid() ? 1u : 0u);
            }
            break;
        case 'd':
            cmd_iq_ref = 0.0f; cmd_omega_ref = 0.0f;
            app_control_set_iq_ref(0.0f);
            app_control_disable();
            rlen = snprintf(resp, sizeof(resp), "[CMD] DISABLED, ST=%u\r\n", app_control_get_state());
            break;
        case 'a':
            cmd_omega_ref = 0.0f;
            app_control_set_speed_mode(false);
            if (app_control_start_align())
                rlen = snprintf(resp, sizeof(resp), "[CMD] ALIGN START, ST=%u\r\n", app_control_get_state());
            else {
                FocMeasurement_t dbg; foc_interface_get_measurement(&dbg);
                rlen = snprintf(resp, sizeof(resp), "[CMD] ALIGN REJECTED ST=%u VBUS=%.1f ENC=%u\r\n",
                    app_control_get_state(), dbg.vbus_v, encoder_is_valid() ? 1u : 0u);
            }
            break;
        case 's':
            rlen = snprintf(resp, sizeof(resp), "[CMD] ST=%u MODE=%u POS=%.2f TGT=%.2f\r\n",
                app_control_get_state(), (uint8_t)app_control_get_ctrl_mode(),
                app_control_get_pos_actual_deg(), app_control_get_pos_target_deg());
            break;
        case '+':
            app_control_set_speed_mode(false); cmd_omega_ref = 0.0f;
            cmd_iq_ref += 0.05f; if (cmd_iq_ref > 1.0f) cmd_iq_ref = 1.0f;
            app_control_set_iq_ref(cmd_iq_ref);
            rlen = snprintf(resp, sizeof(resp), "[CMD] IQ_REF=%.3f\r\n", cmd_iq_ref);
            break;
        case '-':
            app_control_set_speed_mode(false); cmd_omega_ref = 0.0f;
            cmd_iq_ref -= 0.05f; if (cmd_iq_ref < -1.0f) cmd_iq_ref = -1.0f;
            app_control_set_iq_ref(cmd_iq_ref);
            rlen = snprintf(resp, sizeof(resp), "[CMD] IQ_REF=%.3f\r\n", cmd_iq_ref);
            break;
        case ']':
            app_control_set_speed_mode(true);
            cmd_omega_ref += 0.5236f;
            app_control_set_omega_ref(cmd_omega_ref);
            rlen = snprintf(resp, sizeof(resp), "[CMD] SPD_REF=%.1f deg/s\r\n", cmd_omega_ref * 57.2958f);
            break;
        case '[':
            app_control_set_speed_mode(true);
            cmd_omega_ref -= 0.5236f;
            app_control_set_omega_ref(cmd_omega_ref);
            rlen = snprintf(resp, sizeof(resp), "[CMD] SPD_REF=%.1f deg/s\r\n", cmd_omega_ref * 57.2958f);
            break;
        case '0':
            app_control_set_speed_mode(true);
            cmd_omega_ref = 0.0f;
            app_control_set_omega_ref(0.0f);
            rlen = snprintf(resp, sizeof(resp), "[CMD] SPD_REF=0 (STOP)\r\n");
            break;
        case 'E': {
            uint16_t adc_snap[5];
            foc_interface_get_adc_raw(adc_snap);
            float offset = app_control_get_theta_offset();
            float mech = encoder_get_angle_rad();
            float mech_deg = mech * (180.0f / 3.14159265f);
            float elec_raw = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
            uint32_t isr_hz = foc_interface_get_isr_freq();
            float ea1 = elec_raw - offset;
            while (ea1 < 0) ea1 += 6.2832f;
            float ea2 = offset - elec_raw;
            while (ea2 < 0) ea2 += 6.2832f;
            int n = snprintf(resp, sizeof(resp),
                "[CMD] M=%.1f E=%.3f OFF=%.3f fwd=%.3f rev=%.3f ISR=%lu ADC=[%u,%u,%u]\r\n",
                mech_deg, elec_raw, offset, ea1, ea2,
                (unsigned long)isr_hz, adc_snap[0], adc_snap[1], adc_snap[2]);
            HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)n, 200);
            rlen = 0;
            break;
        }
        case 'O': {
            /* 开环测试：Vq=2V 克服齿槽，ω=5.5 rad/s 电角速 ≈ 0.5 rad/s 机械 ≈ 29°/s */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); /* CTRL_SD 使能 */
            foc_openloop_start(5.5f, 2.0f);
            rlen = snprintf(resp, sizeof(resp), "[CMD] OPENLOOP ON w=5.5 Vq=2.0\r\n");
            break;
        }
        case 'X':
            openloop_stab_stop();   /* 先停自稳（内部会 stop openloop）*/
            foc_openloop_stop();    /* 确保纯开环模式也停 */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            rlen = snprintf(resp, sizeof(resp), "[CMD] OPENLOOP OFF\r\n");
            break;
        case 'L':
            if (mpu6050_is_ready()) {
                openloop_stab_start();
                rlen = snprintf(resp, sizeof(resp), "[CMD] OL_STAB ON\r\n");
            } else {
                rlen = snprintf(resp, sizeof(resp), "[CMD] OL_STAB FAIL: IMU not ready\r\n");
            }
            break;
        case 'Q': {
            float mech = encoder_get_angle_rad();
            float elec_raw = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
            float off = app_control_get_theta_offset();
            float ea1 = elec_raw - off;
            while (ea1 < 0) ea1 += 6.2832f;
            float ea2 = off - elec_raw;
            while (ea2 < 0) ea2 += 6.2832f;
            int n = snprintf(resp, sizeof(resp),
                "[ENC] M=%.3f E=%.3f off=%.3f fwd=%.3f rev=%.3f\r\n",
                mech, elec_raw, off, ea1, ea2);
            HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)n, 200);
            rlen = 0;
            break;
        }
        default: break;
        }
        goto send_resp;
    }

    /* B 切换 PI 旁路模式（RUN 时强制零电压输出，测 ADC 读数是否正确）*/
    if (line[0] == 'B') {
        static bool bp = false;
        bp = !bp;
        foc_interface_set_pi_bypass(bp);
        rlen = snprintf(resp, sizeof(resp),
            "[CMD] PI_BYPASS=%s\r\n", bp ? "ON(零电压)" : "OFF(正常)");
    }

    /* F<deg> 手动微调电角度偏移（运行中可改）*/
    if (line[0] == 'F') {
        float deg;
        if (parse_float(line + 1, &deg)) {
            float rad = deg * (3.14159265f / 180.0f);
            app_control_set_fine_offset(rad);
            rlen = snprintf(resp, sizeof(resp),
                "[CMD] FINE_OFF=%.0f deg (%.3f rad)\r\n", deg, rad);
        } else {
            rlen = snprintf(resp, sizeof(resp),
                "[CMD] FINE_OFF=%.1f deg\r\n",
                app_control_get_fine_offset() * (180.0f / 3.14159265f));
        }
    }

    /* I<amps> 直接设 IqRef 绕过速度环（运行中可改，用于测试电流环+角度）*/
    else if (line[0] == 'I' && (line[1] == '-' || line[1] == '+' || (line[1] >= '0' && line[1] <= '9'))) {
        float val;
        if (parse_float(line + 1, &val)) {
            /* 关闭速度模式，直接写 IqRef */
            app_control_set_speed_mode(false);
            app_control_set_iq_ref(val);
            rlen = snprintf(resp, sizeof(resp),
                "[CMD] IQ_DIRECT=%.3f A (速度环已关)\r\n", val);
        }
    }

    /* D: 编码器诊断——打印所有关键角度值 */
    if (line[0] == 'D' && line[1] == '\0') {
        float mech = encoder_get_angle_rad();
        float elec_raw = encoder_get_elec_angle_rad(MOTOR_POLE_PAIRS);
        float offset = app_control_get_theta_offset();
        float fine = app_control_get_fine_offset();
        float ea_fwd = elec_raw - offset + fine;
        while (ea_fwd < 0) ea_fwd += 6.2832f;
        while (ea_fwd >= 6.2832f) ea_fwd -= 6.2832f;
        float ea_rev = offset - elec_raw + fine;
        while (ea_rev < 0) ea_rev += 6.2832f;
        while (ea_rev >= 6.2832f) ea_rev -= 6.2832f;
        rlen = snprintf(resp, sizeof(resp),
            "[ENC] M=%.3f E=%.3f off=%.3f fwd=%.3f rev=%.3f\r\n",
            mech, elec_raw, offset, ea_fwd, ea_rev);
    }

    /* V<vq>: 编码器角度 + 直接电压（完全绕过 PI，决定性测试）
     * V2.0 = 启用，Vq=2V    V0 = 关闭 */
    if (line[0] == 'V' && (line[1] == '-' || line[1] == '+' || (line[1] >= '0' && line[1] <= '9'))) {
        float vq = 0.0f;
        parse_float(line + 1, &vq);
        if (fabsf(vq) > 0.01f) {
            foc_interface_set_bypass_vq(vq);
            foc_interface_set_pi_bypass(true);
            rlen = snprintf(resp, sizeof(resp),
                "[CMD] VDIRECT ON Vq=%.1f (PI bypass, encoder angle)\r\n", vq);
        } else {
            foc_interface_set_bypass_vq(0.0f);
            foc_interface_set_pi_bypass(false);
            rlen = snprintf(resp, sizeof(resp), "[CMD] VDIRECT OFF\r\n");
        }
    }

    /* C: 自动校准——开环旋转3秒，测量编码器关系 */
    if (line[0] == 'C' && line[1] == '\0') {
        foc_openloop_start(5.5f, 2.0f);
        HAL_Delay(3000);
        foc_openloop_stop();
        float avg_p, avg_m, enc_last;
        uint32_t cnt;
        foc_calib_get(&avg_p, &avg_m, &cnt, &enc_last);
        int n = snprintf(resp, sizeof(resp),
            "[CMD] CALIB N=%lu avg(OL+enc)=%.2f avg(OL-enc)=%.2f enc=%.2f\r\n",
            (unsigned long)cnt, avg_p, avg_m, enc_last);
        HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)n, 200);
        rlen = 0;
    }

    /* R<iq>: 强制闭环旋转（绕过编码器，纯PI电流环 + 固定角速度）
     * 用于验证除编码器外的完整FOC链路
     * R0.3 = 启动，Iq=0.3A
     * R0 或 R = 停止 */
    if (line[0] == 'R') {
        float iq = 0.0f;
        if (line[1] != '\0') parse_float(line + 1, &iq);
        if (fabsf(iq) > 0.01f) {
            app_control_set_speed_mode(false);
            app_control_set_iq_ref(iq);
            foc_forced_start(5.5f);  /* 5.5 rad/s 电角速度 */
            rlen = snprintf(resp, sizeof(resp),
                "[CMD] FORCED ON w=5.5 Iq=%.2f\r\n", iq);
        } else {
            foc_forced_stop();
            app_control_set_iq_ref(0.0f);
            rlen = snprintf(resp, sizeof(resp), "[CMD] FORCED OFF\r\n");
        }
    }

    /* T: 自动角度扫描——找到正确的电角度偏移 */
    if (line[0] == 'T' && line[1] == '\0') {
        if (app_control_get_state() != STATE_RUN) {
            rlen = snprintf(resp, sizeof(resp), "[CMD] T: need RUN state first\r\n");
        } else {
            float results[12];
            for (int step = 0; step < 12; step++) {
                float angle_rad = (float)step * 30.0f * 3.14159265f / 180.0f;
                app_control_set_fine_offset(angle_rad);
                app_control_set_iq_ref(0.0f);
                HAL_Delay(300);
                float m_before = encoder_get_angle_rad();
                app_control_set_iq_ref(0.5f);
                HAL_Delay(800);
                app_control_set_iq_ref(0.0f);
                float m_after = encoder_get_angle_rad();
                results[step] = (m_after - m_before) * (180.0f / 3.14159265f);
                HAL_Delay(400);
            }
            app_control_set_fine_offset(0.0f);
            /* 直接发送，用足够长的超时 */
            int n = snprintf(resp, sizeof(resp),
                "[CMD] SWEEP 0:%+.0f|30:%+.0f|60:%+.0f|90:%+.0f|120:%+.0f|150:%+.0f|"
                "180:%+.0f|210:%+.0f|240:%+.0f|270:%+.0f|300:%+.0f|330:%+.0f\r\n",
                results[0], results[1], results[2], results[3],
                results[4], results[5], results[6], results[7],
                results[8], results[9], results[10], results[11]);
            HAL_UART_Transmit(&huart3, (uint8_t*)resp, (uint16_t)n, 200);
            rlen = 0; /* 已发送，不走 send_resp */
        }
    }

    if (strcmp(line, "HTEST") == 0) {
        rlen = snprintf(resp, sizeof(resp), "[CMD] HTEST: hold=%.2f deg\r\n",
            app_control_get_pos_target_deg());
    }
    else if (strcmp(line, "H") == 0) {
        app_control_home();
        rlen = snprintf(resp, sizeof(resp), "[CMD] HOME OK, pos=%.2f->0, CTRL=POS\r\n",
            app_control_get_pos_actual_deg());
    }
    else if (strncmp(line, "PLIM", 4) == 0) {
        float val;
        if (parse_float(line + 4, &val)) {
            app_control_set_pos_spd_lim(val);
            rlen = snprintf(resp, sizeof(resp), "[CMD] POS_SPD_LIM=%.1f dps\r\n", app_control_get_pos_spd_lim());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] PLIM: need 10-300\r\n"); }
    }
    else if (strncmp(line, "GLIM", 4) == 0) {
        /* GLIM<val>：设置自稳目标软限幅 [°]，默认10°，逐步放大到所需范围后再发G */
        float val;
        if (parse_float(line + 4, &val)) {
            app_control_set_stab_lim(val);
            rlen = snprintf(resp, sizeof(resp), "[CMD] STAB_LIM=%.1f deg\r\n", app_control_get_stab_lim());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] GLIM: need 2-90\r\n"); }
    }
    else if (strncmp(line, "KP", 2) == 0) {
        float val;
        if (parse_float(line + 2, &val)) {
            app_control_set_kp_pos(val);
            rlen = snprintf(resp, sizeof(resp), "[CMD] KP_POS=%.3f\r\n", app_control_get_kp_pos());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] KP: need 0-20\r\n"); }
    }
    else if (strncmp(line, "KD", 2) == 0) {
        float val;
        if (parse_float(line + 2, &val)) {
            app_control_set_kd_pos(val);
            rlen = snprintf(resp, sizeof(resp), "[CMD] KD_POS=%.4f\r\n", app_control_get_kd_pos());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] KD: need 0-2\r\n"); }
    }
    else if (strncmp(line, "DZ", 2) == 0) {
        float val;
        if (parse_float(line + 2, &val)) {
            app_control_set_pos_deadband(val);
            rlen = snprintf(resp, sizeof(resp), "[CMD] DEADBAND=%.2f\r\n", app_control_get_pos_deadband());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] DZ: need 0-5\r\n"); }
    }
    else if (strncmp(line, "STEP", 4) == 0) {
        float deg;
        if (parse_float(line + 4, &deg)) {
            app_control_set_pos_target(app_control_get_pos_target_deg() + deg);
            app_control_set_ctrl_mode(CTRL_POSITION);
            rlen = snprintf(resp, sizeof(resp), "[CMD] STEP+%.1f TGT=%.2f\r\n",
                deg, app_control_get_pos_target_deg());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] STEP: need numeric\r\n"); }
    }
    else if (strcmp(line, "G") == 0) {
        /* G：切换到 CTRL_STABILIZE 自稳模式（需 IMU 就绪）*/
        if (mpu6050_is_ready()) {
            app_control_set_ctrl_mode(CTRL_STABILIZE);
            rlen = snprintf(resp, sizeof(resp), "[CMD] STABILIZE ON\r\n");
        } else {
            rlen = snprintf(resp, sizeof(resp), "[CMD] IMU not ready\r\n");
        }
    }
    else if (line[0] == 'S' && line[1] != '\0') {
        float dps;
        if (parse_float(line + 1, &dps)) {
            cmd_omega_ref = dps * (3.14159265f / 180.0f);
            app_control_set_speed_mode(true);
            app_control_set_omega_ref(cmd_omega_ref);
            rlen = snprintf(resp, sizeof(resp), "[CMD] SPD_MODE REF=%.1f dps\r\n", dps);
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] S: need numeric dps\r\n"); }
    }
    else if (line[0] == 'T' && line[1] != '\0') {
        /* 开环力矩：直接设 IqRef，绕过速度PI，用于隔离验证 */
        float iq;
        if (parse_float(line + 1, &iq)) {
            if (iq > 0.45f) iq = 0.45f;
            if (iq < -0.45f) iq = -0.45f;
            app_control_set_speed_mode(false);   /* 关闭速度PI */
            cmd_iq_ref = iq;
            app_control_set_iq_ref(iq);
            rlen = snprintf(resp, sizeof(resp), "[CMD] TORQUE IQ=%.3f A\r\n", iq);
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] T: need current [A]\r\n"); }
    }
    else if (line[0] == 'P') {
        float deg;
        if (parse_float(line + 1, &deg)) {
            app_control_set_pos_target(deg);
            app_control_set_ctrl_mode(CTRL_POSITION);
            rlen = snprintf(resp, sizeof(resp), "[CMD] POS_TGT=%.2f\r\n",
                app_control_get_pos_target_deg());
        } else { rlen = snprintf(resp, sizeof(resp), "[CMD] P: need angle\r\n"); }
    }

send_resp:
    if (rlen > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)resp, (uint16_t)rlen, 5U);
    }
}

static uint8_t rx_buf[40];
static uint8_t rx_len = 0;

static bool parse_float(const char *s, float *out)
{
    char *ep;
    *out = strtof(s, &ep);
    if (ep == s) return false;
    while (*ep == ' ' || *ep == '\t') ep++;
    if (*ep != '\0') return false;
    if (!isfinite(*out)) return false;
    return true;
}

static void cmd_feed_byte(uint8_t c)
{
    static const char IMMEDIATE[] = "edas+-][0EOXL";
    if (rx_len == 0 && strchr(IMMEDIATE, (char)c)) {
        char line[2] = {(char)c, '\0'};
        cmd_process_line(line);
        return;
    }
    if (c == '\r' || c == '\n') {
        if (rx_len > 0) {
            rx_buf[rx_len] = '\0';
            cmd_process_line((char *)rx_buf);
            rx_len = 0;
        }
        return;
    }
    if (rx_len < sizeof(rx_buf) - 1) {
        rx_buf[rx_len++] = c;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  __HAL_RCC_SPI1_CLK_ENABLE();  /* 确保 SPI1 时钟在 Init 前已开 */
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 1. 初始化各应用模块
  foc_interface_init();
  encoder_spi_init();
  app_control_init();
  telemetry_init();
  /* IMU 初始化（CubeMX 生成 MX_I2C1_Init 并在此之前调用后，此处有效）
   * 若 I2C 尚未配置则函数内部直接返回 false，不影响启动 */
  mpu6050_init();

  // 2. ADC 自校准（G4 必须在 Start 前执行，消除内部偏移误差）
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // 3. 挂好数据接收方
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, FOC_ADC_CH_COUNT);

  // 3. 起振全电源桥臂 (底层内含 MOE 置位)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // 4. 最后拉开枪栓，放出内部触发源

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // 5. 使能 USART3 RXNE 中断（绕过主循环 TX 阻塞，零丢字节）
  //    优先级 3：低于 FOC ADC ISR(0-1)，高于 SysTick(15)
  HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 主循环消费 RX 环形缓冲区（字节已由 USART3_IRQHandler ISR 填入）*/
    while (s_rx_tail != s_rx_head) {
        uint8_t c = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1U) & UART_RX_RING_MASK);
        cmd_feed_byte(c);
    }
    app_control_update();

    /* IMU 10ms 定时更新（轮询，不占 ISR）*/
    static uint32_t s_imu_last_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - s_imu_last_ms >= 10U) {
        s_imu_last_ms = now_ms;
        mpu6050_update();
    }

    openloop_stab_update();  /* 开环自稳 PD 控制（未激活时零开销）*/
    telemetry_update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == &hadc1) {
        /* 验证2: Pulse 首 */
        HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

        static uint16_t enc_div = 0;
        if (++enc_div >= 20U) {
            enc_div = 0;
            encoder_spi_read_isr();
        }

        foc_interface_step(adc_raw_buffer);

        /* 验证2: Pulse 尾 */
        HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_RESET);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
