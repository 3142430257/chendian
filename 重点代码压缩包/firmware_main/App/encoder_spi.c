/**
 * @file    encoder_spi.c
 * @brief   AS5048A SPI 编码器驱动（替代 encoder_pwm.c）
 *
 * SPI 协议：AS5048A 寄存器读写，两帧 16-bit
 *   帧1: [0, R/W=1, ADDR(13:0)] → 0x4000 | addr
 *   帧2: [0, 0, DATA(13:0)]     → dummy TX, RX 含角度 + 标志位
 *
 * 在 ISR 内调用 encoder_spi_read_isr()，CS 拉低期间连发 4 字节
 * (两帧 32-bit)，总耗时 ~6μs @10MHz SPI。
 *
 * 角度提取：rx[2] bit[5:0] + rx[3] bit[7:0] = 14-bit 原始值
 */

#include "encoder_spi.h"
#include "as5048a_protocol.h"
#include "main.h"
#include "spi.h"
#include <math.h>

extern SPI_HandleTypeDef hspi1;

/* ============================================================
 * 内部状态
 * ============================================================ */
static volatile float    s_angle_rad   = 0.0f;
static volatile float    s_omega_rad_s = 0.0f;
static volatile uint32_t s_last_cap_ms = 0;
static volatile bool     s_valid       = false;
static volatile uint16_t s_last_resp_cmd = 0;
static volatile uint16_t s_last_resp_angle = 0;
static volatile uint16_t s_last_raw = 0;
static volatile uint16_t s_last_rx_raw = 0;
static volatile uint32_t s_bad_frame_count = 0;
static volatile uint16_t s_drop_count = 0;
static bool              s_speed_init  = false;
static uint16_t          s_prev_raw    = 0;
static uint32_t          s_prev_ms     = 0;
static uint16_t          s_win_raw     = 0;
static uint32_t          s_win_ms      = 0;
static bool              s_boot_candidate_valid = false;
static uint16_t          s_boot_candidate_raw = 0;
static uint8_t           s_boot_candidate_count = 0;
static bool              s_recover_candidate_valid = false;
static uint16_t          s_recover_candidate_raw = 0;
static uint8_t           s_recover_candidate_count = 0;

#define AS5048A_CPR          (16384)
#define AS5048A_HALF_CPR     (AS5048A_CPR / 2)
#define SPEED_EST_ALPHA      (0.25f)
#define SPEED_WINDOW_MS      (20U)
#define SPEED_MAX_RAD_S      (30.0f)
#define SAMPLE_MAX_RAD_S     (60.0f)
#define SPEED_RESET_DT_MS    (100U)
#define RAW_TO_RAD           (6.28318530f / 16384.0f)
#define BOOT_STABLE_RAW      (8)
#define BOOT_STABLE_COUNT    (2U)
#define RECOVER_STABLE_RAW   (12)
#define RECOVER_STABLE_COUNT (4U)
#define RECOVER_MIN_AGE_MS   (30U)
#define PLAUSIBLE_DT_MAX_MS  (25U)
#define ZERO_GLITCH_RAW      (16)

#define CS_LOW()   HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET)

static int32_t raw_delta_wrap(uint16_t now, uint16_t prev)
{
    int32_t delta = (int32_t)now - (int32_t)prev;
    if (delta > AS5048A_HALF_CPR) {
        delta -= AS5048A_CPR;
    } else if (delta < -AS5048A_HALF_CPR) {
        delta += AS5048A_CPR;
    }
    return delta;
}

static bool raw_near(uint16_t a, uint16_t b, int32_t limit)
{
    int32_t delta = raw_delta_wrap(a, b);
    return (delta <= limit && delta >= -limit);
}

static int32_t plausible_delta_limit(uint32_t dt_ms)
{
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    if (dt_ms > PLAUSIBLE_DT_MAX_MS) {
        dt_ms = PLAUSIBLE_DT_MAX_MS;
    }

    int32_t max_delta = (int32_t)((SAMPLE_MAX_RAD_S * (float)dt_ms * 1e-3f) / RAW_TO_RAD) + 4;
    if (max_delta < 8) {
        max_delta = 8;
    }
    return max_delta;
}

static void mark_bad_frame(void)
{
    if (s_bad_frame_count != 0xFFFFFFFFu) {
        s_bad_frame_count++;
    }
    if (s_drop_count != 0xFFFFu) {
        s_drop_count++;
    }
    s_omega_rad_s *= 0.80f;
    if (fabsf(s_omega_rad_s) < 0.05f) {
        s_omega_rad_s = 0.0f;
    }
}

static bool bootstrap_accept(uint16_t raw)
{
    if (!s_boot_candidate_valid || !raw_near(raw, s_boot_candidate_raw, BOOT_STABLE_RAW)) {
        s_boot_candidate_raw = raw;
        s_boot_candidate_count = 1U;
        s_boot_candidate_valid = true;
        return false;
    }

    if (s_boot_candidate_count < 0xFFu) {
        s_boot_candidate_count++;
    }
    s_boot_candidate_raw = raw;
    return (s_boot_candidate_count >= BOOT_STABLE_COUNT);
}

static bool recovery_accept(uint16_t raw, uint32_t age_ms)
{
    if (age_ms < RECOVER_MIN_AGE_MS) {
        s_recover_candidate_valid = false;
        s_recover_candidate_count = 0U;
        return false;
    }
    if (raw <= ZERO_GLITCH_RAW &&
        s_prev_raw > ZERO_GLITCH_RAW &&
        !raw_near(raw, s_prev_raw, plausible_delta_limit(PLAUSIBLE_DT_MAX_MS))) {
        s_recover_candidate_valid = false;
        s_recover_candidate_count = 0U;
        return false;
    }

    if (!s_recover_candidate_valid || !raw_near(raw, s_recover_candidate_raw, RECOVER_STABLE_RAW)) {
        s_recover_candidate_raw = raw;
        s_recover_candidate_count = 1U;
        s_recover_candidate_valid = true;
        return false;
    }

    if (s_recover_candidate_count < 0xFFu) {
        s_recover_candidate_count++;
    }
    s_recover_candidate_raw = raw;
    return (s_recover_candidate_count >= RECOVER_STABLE_COUNT);
}

static void accept_sample(uint16_t resp_cmd, uint16_t resp_angle, uint16_t raw, uint32_t now_ms)
{
    if (s_speed_init) {
        uint32_t dt_ms = now_ms - s_prev_ms;
        if (dt_ms > 0U && dt_ms <= SPEED_RESET_DT_MS) {
            uint32_t win_dt_ms = now_ms - s_win_ms;
            if (win_dt_ms >= SPEED_WINDOW_MS) {
                int32_t win_delta = raw_delta_wrap(raw, s_win_raw);
                float omega_raw = ((float)win_delta * RAW_TO_RAD) / ((float)win_dt_ms * 1e-3f);
                if (omega_raw >  SPEED_MAX_RAD_S) omega_raw =  SPEED_MAX_RAD_S;
                if (omega_raw < -SPEED_MAX_RAD_S) omega_raw = -SPEED_MAX_RAD_S;
                s_omega_rad_s = SPEED_EST_ALPHA * omega_raw
                              + (1.0f - SPEED_EST_ALPHA) * s_omega_rad_s;
                s_win_raw = raw;
                s_win_ms = now_ms;
            }
        } else if (dt_ms > SPEED_RESET_DT_MS) {
            s_omega_rad_s = 0.0f;
            s_win_raw = raw;
            s_win_ms = now_ms;
        }
    } else {
        s_speed_init = true;
        s_omega_rad_s = 0.0f;
        s_win_raw = raw;
        s_win_ms = now_ms;
    }

    s_prev_raw = raw;
    s_prev_ms = now_ms;
    s_recover_candidate_valid = false;
    s_recover_candidate_count = 0U;
    s_drop_count = 0;

    s_last_resp_cmd = resp_cmd;
    s_last_resp_angle = resp_angle;
    s_last_raw = raw;
    s_angle_rad = as5048a_raw_to_rad(raw);
    s_last_cap_ms = now_ms;
    s_valid = true;
}

static bool spi_transfer16(uint16_t tx, uint16_t *rx)
{
    SPI_TypeDef *spi = SPI1;
    volatile uint8_t *dr8 = (volatile uint8_t *)&spi->DR;
    uint8_t hi = (uint8_t)(tx >> 8);
    uint8_t lo = (uint8_t)(tx & 0xFFu);
    uint8_t rhi = 0;
    uint8_t rlo = 0;
    int t;

    if (!(spi->CR1 & SPI_CR1_SPE)) {
        spi->CR1 |= SPI_CR1_SPE;
        for (volatile int d = 0; d < 100; d++) { }
    }

    t = 100000;
    while (!(spi->SR & SPI_SR_TXE) && --t) { }
    if (t == 0) return false;
    *dr8 = hi;

    t = 100000;
    while (!(spi->SR & SPI_SR_RXNE) && --t) { }
    if (t == 0) return false;
    rhi = *dr8;

    t = 100000;
    while (!(spi->SR & SPI_SR_TXE) && --t) { }
    if (t == 0) return false;
    *dr8 = lo;

    t = 100000;
    while (!(spi->SR & SPI_SR_RXNE) && --t) { }
    if (t == 0) return false;
    rlo = *dr8;

    t = 100000;
    while ((spi->SR & SPI_SR_BSY) && --t) { }
    if (t == 0) return false;

    if (rx != 0) {
        *rx = (uint16_t)(((uint16_t)rhi << 8) | rlo);
    }
    return true;
}

static bool as5048a_read_angle(uint16_t *resp_cmd, uint16_t *resp_angle, uint16_t *raw)
{
    uint16_t cmd = as5048a_make_read_command(AS5048A_REG_ANGLE);
    uint16_t nop = as5048a_make_nop_command();
    uint16_t r1 = 0;
    uint16_t r2 = 0;

    CS_LOW();
    bool ok = spi_transfer16(cmd, &r1);
    CS_HIGH();
    if (!ok) return false;

    CS_LOW();
    ok = spi_transfer16(nop, &r2);
    CS_HIGH();
    if (!ok) return false;

    uint16_t angle_raw = as5048a_extract_angle_raw(r2);
    if (resp_cmd != 0) *resp_cmd = r1;
    if (resp_angle != 0) *resp_angle = r2;
    if (raw != 0) *raw = angle_raw;
    return true;
}

void encoder_spi_init(void)
{
    s_angle_rad   = 0.0f;
    s_omega_rad_s = 0.0f;
    s_last_cap_ms = 0;
    s_valid       = false;
    s_last_resp_cmd = 0;
    s_last_resp_angle = 0;
    s_last_raw = 0;
    s_last_rx_raw = 0;
    s_bad_frame_count = 0;
    s_drop_count = 0;
    s_speed_init = false;
    s_prev_raw = 0;
    s_prev_ms = 0;
    s_win_raw = 0;
    s_win_ms = 0;
    s_boot_candidate_valid = false;
    s_boot_candidate_raw = 0;
    s_boot_candidate_count = 0;
    s_recover_candidate_valid = false;
    s_recover_candidate_raw = 0;
    s_recover_candidate_count = 0;
    CS_HIGH();

    /* 降 SPI 速度到 ~2.7MHz（170MHz / 64），抗 PWM EMI */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    HAL_SPI_Init(&hspi1);

    encoder_spi_read_isr();
}

/**
 * @brief  ISR 内调用，SPI 读 AS5048A 角度寄存器 (0x3FFF)
 * @note   裸寄存器操作，~5μs @10MHz，避免 HAL 在 ISR 中的开销和风险
 */
void encoder_spi_read_isr(void)
{
    uint16_t resp_cmd = 0;
    uint16_t resp_angle = 0;
    uint16_t raw = 0;
    if (!as5048a_read_angle(&resp_cmd, &resp_angle, &raw)) {
        mark_bad_frame();
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    s_last_rx_raw = raw;

    if (!s_speed_init) {
        if (!bootstrap_accept(raw)) {
            mark_bad_frame();
            return;
        }
        accept_sample(resp_cmd, resp_angle, raw, now_ms);
        return;
    }

    uint32_t age_ms = now_ms - s_last_cap_ms;
    uint32_t dt_ms = now_ms - s_prev_ms;
    bool plausible = true;

    if (dt_ms <= SPEED_RESET_DT_MS) {
        int32_t delta = raw_delta_wrap(raw, s_prev_raw);
        int32_t max_delta = plausible_delta_limit(dt_ms);
        if (delta > max_delta || delta < -max_delta) {
            plausible = false;
        }
        if (raw <= ZERO_GLITCH_RAW && s_prev_raw > ZERO_GLITCH_RAW && !raw_near(raw, s_prev_raw, max_delta)) {
            plausible = false;
        }
    }

    if (!plausible) {
        uint16_t retry_resp_cmd = 0;
        uint16_t retry_resp_angle = 0;
        uint16_t retry_raw = 0;
        if (as5048a_read_angle(&retry_resp_cmd, &retry_resp_angle, &retry_raw)) {
            s_last_rx_raw = retry_raw;
            int32_t retry_delta = raw_delta_wrap(retry_raw, s_prev_raw);
            int32_t max_delta = plausible_delta_limit(dt_ms);
            if (retry_delta <= max_delta && retry_delta >= -max_delta &&
                !(retry_raw <= ZERO_GLITCH_RAW && s_prev_raw > ZERO_GLITCH_RAW &&
                  !raw_near(retry_raw, s_prev_raw, max_delta))) {
                accept_sample(retry_resp_cmd, retry_resp_angle, retry_raw, now_ms);
                return;
            }

            if (recovery_accept(retry_raw, age_ms)) {
                s_speed_init = false;
                accept_sample(retry_resp_cmd, retry_resp_angle, retry_raw, now_ms);
                return;
            }
        }

        mark_bad_frame();
        return;
    }

    accept_sample(resp_cmd, resp_angle, raw, now_ms);
}

/* ============================================================
 * encoder_if.h 接口实现
 * ============================================================ */

float encoder_get_angle_rad(void)
{
    return s_angle_rad;
}

float encoder_get_elec_angle_rad(uint8_t pole_pairs)
{
    float elec = s_angle_rad * (float)pole_pairs;
    while (elec >= 6.28318530f) elec -= 6.28318530f;
    return elec;
}

bool encoder_is_valid(void)
{
    if (!s_valid) return false;
    /* 超时 >200ms → 无效（ALIGN 期间 PWM EMI 可能短暂丢帧） */
    if ((HAL_GetTick() - s_last_cap_ms) > 200U) {
        return false;
    }
    return true;
}

uint32_t encoder_get_last_capture_ms(void)
{
    return s_last_cap_ms;
}

float encoder_get_omega_rad_s(void)
{
    if (!encoder_is_valid()) {
        return 0.0f;
    }
    return s_omega_rad_s;
}

uint32_t encoder_get_bad_frame_count(void)
{
    return s_bad_frame_count;
}

uint16_t encoder_get_drop_count(void)
{
    return s_drop_count;
}

uint16_t encoder_get_last_raw(void)
{
    return s_last_raw;
}

uint16_t encoder_get_last_rx_raw(void)
{
    return s_last_rx_raw;
}
