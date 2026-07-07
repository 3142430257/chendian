/**
 * @file    as5048a_protocol.h
 * @brief   AS5048A 16-bit SPI frame helpers.
 */

#ifndef AS5048A_PROTOCOL_H
#define AS5048A_PROTOCOL_H

#include <stdint.h>

#define AS5048A_REG_ANGLE  (0x3FFFu)
#define AS5048A_DATA_MASK  (0x3FFFu)
#define AS5048A_READ_BIT   (0x4000u)
#define AS5048A_PARITY_BIT (0x8000u)

static inline uint16_t as5048a_even_parity_bit(uint16_t frame_without_parity)
{
    uint16_t v = (uint16_t)(frame_without_parity & 0x7FFFu);
    v ^= (uint16_t)(v >> 8);
    v ^= (uint16_t)(v >> 4);
    v ^= (uint16_t)(v >> 2);
    v ^= (uint16_t)(v >> 1);
    return (uint16_t)(v & 1u);
}

static inline uint16_t as5048a_make_read_command(uint16_t reg)
{
    uint16_t frame = (uint16_t)(AS5048A_READ_BIT | (reg & AS5048A_DATA_MASK));
    if (as5048a_even_parity_bit(frame) != 0u) {
        frame |= AS5048A_PARITY_BIT;
    }
    return frame;
}

static inline uint16_t as5048a_make_nop_command(void)
{
    return 0x0000u;
}

static inline uint16_t as5048a_extract_angle_raw(uint16_t response)
{
    return (uint16_t)(response & AS5048A_DATA_MASK);
}

static inline float as5048a_raw_to_rad(uint16_t raw)
{
    return (float)(raw & AS5048A_DATA_MASK) * 6.28318530f / 16384.0f;
}

#endif /* AS5048A_PROTOCOL_H */
