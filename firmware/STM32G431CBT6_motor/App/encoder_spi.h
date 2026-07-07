/**
 * @file    encoder_spi.h
 * @brief   AS5048A SPI 编码器驱动头文件
 */

#ifndef ENCODER_SPI_H
#define ENCODER_SPI_H

#include <stdint.h>
#include <stdbool.h>

/* encoder_if.h 接口 */
void     encoder_spi_init(void);
void     encoder_spi_read_isr(void);       /* ISR 内调用，~6μs */
float    encoder_get_angle_rad(void);
float    encoder_get_elec_angle_rad(uint8_t pole_pairs);
bool     encoder_is_valid(void);
uint32_t encoder_get_last_capture_ms(void);
float    encoder_get_omega_rad_s(void);
uint32_t encoder_get_bad_frame_count(void);
uint16_t encoder_get_drop_count(void);
uint16_t encoder_get_last_raw(void);
uint16_t encoder_get_last_rx_raw(void);

#endif /* ENCODER_SPI_H */
