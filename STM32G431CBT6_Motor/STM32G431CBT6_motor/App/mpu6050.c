/**
 * @file    mpu6050.c
 * @brief   MPU6050 驱动：软件 I2C bit-bang（PB5=SCL, PB4=SDA）+ 互补滤波器
 *
 * 说明：
 *   使用开漏 GPIO 模拟 I2C，约 ~80kHz 时钟。
 *   无需 CubeMX 使能 I2C 外设，避免了 PB8/PB9（BOOT0）的硬件冲突。
 */

#include "mpu6050.h"
#include "stm32g4xx_hal.h"
#include <math.h>

/* ---- 寄存器地址 ---- */
#define REG_WHO_AM_I    0x75U
#define REG_PWR_MGMT_1  0x6BU
#define REG_GYRO_CFG    0x1BU
#define REG_ACCEL_CFG   0x1CU
#define REG_ACCEL_XOUT  0x3BU   /* 连续14字节 */

/* ---- 量程换算因子 ---- */
#define ACCEL_SCALE     (1.0f / 16384.0f)
#define GYRO_SCALE      (1.0f / 131.0f)

/* ---- 模块私有状态 ---- */
static bool    s_ready      = false;
static float   s_pitch_deg  = 0.0f;
static float   s_gyro_x_dps = 0.0f;
static uint8_t s_fail_cnt   = 0U;

/* ============================================================
 *  软件 I2C 底层（开漏 GPIO bit-bang）
 * ============================================================ */

/* 简单 NOP 延时：~170 周期 ≈ 1μs @170MHz，每半周期 3μs → ~80kHz */
static inline void sw_dly(void)
{
    volatile uint32_t i = 80U;
    while (i--) { __NOP(); }
}

#define SCL_H()  do { HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_SET);   sw_dly(); } while(0)
#define SCL_L()  do { HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_RESET); sw_dly(); } while(0)
#define SDA_H()  do { HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_SET);   } while(0)
#define SDA_L()  do { HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_RESET); } while(0)
#define SDA_R()  (HAL_GPIO_ReadPin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN) == GPIO_PIN_SET)

static void sw_start(void)
{
    SDA_H(); SCL_H();
    SDA_L(); sw_dly();
    SCL_L(); sw_dly();
}

static void sw_stop(void)
{
    SDA_L(); sw_dly();
    SCL_H(); sw_dly();
    SDA_H(); sw_dly();
}

/* 发送一字节，返回 0=ACK，1=NACK */
static uint8_t sw_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if (b & (1U << i)) SDA_H(); else SDA_L();
        sw_dly(); SCL_H(); SCL_L();
    }
    SDA_H(); sw_dly();           /* 释放 SDA 等待 ACK */
    SCL_H(); sw_dly();
    uint8_t ack = SDA_R() ? 1U : 0U;  /* 0=ACK（从机拉低）*/
    SCL_L(); sw_dly();
    return ack;
}

/* 读一字节，send_ack=true 则发 ACK，false 发 NACK（最后一字节）*/
static uint8_t sw_read_byte(bool send_ack)
{
    uint8_t b = 0;
    SDA_H();
    for (int i = 7; i >= 0; i--) {
        sw_dly(); SCL_H(); sw_dly();
        if (SDA_R()) b |= (1U << i);
        SCL_L();
    }
    if (send_ack) SDA_L(); else SDA_H();
    sw_dly(); SCL_H(); SCL_L(); sw_dly();
    SDA_H();
    return b;
}

/* ---- 封装：写单字节寄存器 ---- */
static bool reg_write(uint8_t reg, uint8_t val)
{
    sw_start();
    if (sw_write_byte(MPU6050_ADDR)) { sw_stop(); return false; }   /* addr+W */
    if (sw_write_byte(reg))          { sw_stop(); return false; }
    if (sw_write_byte(val))          { sw_stop(); return false; }
    sw_stop();
    return true;
}

/* ---- 封装：读多字节寄存器 ---- */
static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    sw_start();
    if (sw_write_byte(MPU6050_ADDR))        { sw_stop(); return false; }  /* addr+W */
    if (sw_write_byte(reg))                 { sw_stop(); return false; }
    sw_start();                                                             /* 重复起始 */
    if (sw_write_byte(MPU6050_ADDR | 0x01)) { sw_stop(); return false; }  /* addr+R */
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = sw_read_byte(i < (len - 1));  /* 最后一字节发 NACK */
    }
    sw_stop();
    return true;
}

static inline int16_t to_int16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/* ============================================================
 *  公开接口
 * ============================================================ */

bool mpu6050_init(void)
{
    /* 1. 配置 PB5（SCL）和 PB4（SDA）为开漏输出 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = SW_I2C_SCL_PIN | SW_I2C_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏：写1=高阻，写0=拉低 */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, SW_I2C_SCL_PIN | SW_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(10);   /* 等待从机上电稳定 */

    s_ready   = false;
    s_fail_cnt = 0;

    /* 2. WHO_AM_I 握手，期望 0x68 */
    uint8_t who = 0;
    if (!reg_read(REG_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68U) return false;

    /* 3. 退出睡眠，PLL 时钟 */
    if (!reg_write(REG_PWR_MGMT_1, 0x01U)) return false;
    HAL_Delay(10);

    /* 4. 陀螺仪 ±250°/s */
    if (!reg_write(REG_GYRO_CFG,  (uint8_t)(MPU6050_GYRO_FS  << 3))) return false;

    /* 5. 加速度计 ±2g */
    if (!reg_write(REG_ACCEL_CFG, (uint8_t)(MPU6050_ACCEL_FS << 3))) return false;

    s_ready = true;
    return true;
}

bool mpu6050_is_ready(void) { return s_ready; }

bool mpu6050_update(void)
{
    if (!s_ready) {
        /* 周期重试，每 100 次（约 1s）尝试一次重初始化 */
        static uint16_t s_retry = 0;
        if (++s_retry >= 100U) { s_retry = 0; mpu6050_init(); }
        return false;
    }

    uint8_t buf[14];
    if (!reg_read(REG_ACCEL_XOUT, buf, 14)) {
        if (++s_fail_cnt >= 5U) { s_fail_cnt = 0; s_ready = false; }
        return false;
    }
    s_fail_cnt = 0;

    int16_t ax = to_int16(buf[0], buf[1]);
    int16_t ay = to_int16(buf[2], buf[3]);
    int16_t az = to_int16(buf[4], buf[5]);
    int16_t gx = to_int16(buf[8], buf[9]);

    float ax_g  = ax * ACCEL_SCALE;
    float ay_g  = ay * ACCEL_SCALE;
    float az_g  = az * ACCEL_SCALE;
    float gx_dps = gx * GYRO_SCALE;
    s_gyro_x_dps = gx_dps;

    float pitch_acc = atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g))
                      * (180.0f / 3.14159265f);

    s_pitch_deg = COMP_ALPHA * (s_pitch_deg + gx_dps * MPU6050_DT_S)
                + (1.0f - COMP_ALPHA) * pitch_acc;

    return true;
}

float mpu6050_get_pitch_deg(void)   { return s_pitch_deg;  }
float mpu6050_get_gyro_x_dps(void) { return s_gyro_x_dps; }
