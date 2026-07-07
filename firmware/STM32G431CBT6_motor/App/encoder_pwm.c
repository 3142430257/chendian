/**
 * @file    encoder_pwm.c
 * @brief   AS5048A PWM 输入捕获驱动（TIM2_CH1 / PA15）
 *
 * TIM2 配置为 PWM 输入模式：
 *   - TIM2_CH1 测量周期（total_count = CCR1）
 *   - TIM2_CH2 测量高电平（high_count  = CCR2）
 * 角度 = high_count / total_count × 2π
 *
 * AS5048A 规格：
 *   - 有效占空比范围 0.3% ~ 99.7%（对应 0° ~ 360°）
 *   - 精度 12-bit（4096 步/圈）
 */

#include "encoder_pwm.h"
#include "board_config.h"
#include <math.h>
#include "stm32g4xx_hal.h"   /* 由 CubeMX 生成后自动可见 */

/* 外部 TIM2 句柄（CubeMX 在 tim.c 中生成）*/
extern TIM_HandleTypeDef htim2;

/* ============================================================
 * 内部状态
 * ============================================================ */
static volatile uint32_t s_high_count   = 0;
static volatile uint32_t s_total_count  = 0;
static volatile float    s_angle_rad    = 0.0f;
static volatile uint32_t s_last_cap_ms  = 0;
static volatile bool     s_valid        = false;

/* 跳变过滤器：相邻两次有效捕获间角度差超过此阈值则丢弃。
 * 阈值 0.5 rad ≈ 29° 机械角。正常最大转速 ~10 rad/s，1ms 间隔
 * 内最大变化 ~0.01 rad，0.5 rad 裕量 > 50x，EMI 跳变（>2 rad）会被拦截。 */
#define ENC_GLITCH_THRESH_RAD  (0.5f)
static volatile float   s_last_valid_angle_rad = 0.0f;
static volatile bool    s_had_valid_reading    = false;

/* AS5048A 有效占空比范围（去除无效区间）*/
#define AS5048_DUTY_MIN  (0.003f)   /* 0.3% */
#define AS5048_DUTY_MAX  (0.997f)   /* 99.7% */

void encoder_pwm_init(void)
{
    s_high_count  = 0;
    s_total_count = 0;
    s_angle_rad   = 0.0f;
    s_last_cap_ms = 0;
    s_valid       = false;

    /* 启动 TIM2 PWM 输入捕获
     * CubeMX 配置为 PWM Input mode：
     *   Channel1 → 测量周期（从上升沿到下一上升沿）
     *   Channel2 → 测量高电平（从上升沿到下降沿）
     */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

/**
 * @brief  在 stm32g4xx_it.c 的 HAL_TIM_IC_CaptureCallback 中调用
 */
void encoder_pwm_capture_callback(void *htim_ptr)
{
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)htim_ptr;

    if (htim->Instance != TIM2) return;

    /* PWM 输入模式：CH1 捕获周期，CH2 捕获高电平 */
    if (HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
    {
        s_total_count = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        s_high_count  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        if (s_total_count == 0U) return;  /* 防除零 */

        float duty = (float)s_high_count / (float)s_total_count;

        /* 有效性检查 */
        if (duty >= AS5048_DUTY_MIN && duty <= AS5048_DUTY_MAX)
        {
            /* 归一化到 [0, 1)，再映射到 [0, 2π) */
            float norm = (duty - AS5048_DUTY_MIN) / (AS5048_DUTY_MAX - AS5048_DUTY_MIN);

            /* 跳变过滤：EMI 会导致偶发性巨大角度跳变。
             * 但超时后（如 ALIGN 期间 EMI 持续干扰 → 编码器超时 → 电机
             * 确实转动了），第一个新读数无条件接受，不比较。*/
            if (s_had_valid_reading && s_valid) {
                float diff = norm - s_last_valid_angle_rad;
                /* 处理 0↔2π 环绕 */
                if (diff >  1.0f) diff -= 2.0f;
                if (diff < -1.0f) diff += 2.0f;
                if (fabsf(diff) > ENC_GLITCH_THRESH_RAD) {
                    return;  /* 丢弃 EMI 跳变，保留上次有效读数 */
                }
            }

            s_angle_rad   = norm * 2.0f * (float)M_PI;
            s_last_valid_angle_rad = norm;
            s_had_valid_reading    = true;
            s_last_cap_ms = HAL_GetTick();
            s_valid       = true;
        }
    }
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
    /* 归一化到 [0, 2π) */
    while (elec >= 2.0f * (float)M_PI) elec -= 2.0f * (float)M_PI;
    return elec;
}

bool encoder_is_valid(void)
{
    if (!s_valid) return false;
    /* 超时检测：连续 ENCODER_TIMEOUT_MS 未更新视为丢失 */
    if ((HAL_GetTick() - s_last_cap_ms) > ENCODER_TIMEOUT_MS)
    {
        s_valid = false;
    }
    return s_valid;
}

uint32_t encoder_get_last_capture_ms(void)
{
    return s_last_cap_ms;
}
