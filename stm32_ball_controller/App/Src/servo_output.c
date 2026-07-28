#include "servo_output.h"
#include "app_config.h"

static TIM_HandleTypeDef *s_timer = 0;
static uint32_t s_channel = 0U;
/*
 * 20 ms控制任务写目标值，1 ms舵机节拍读取目标值并更新当前值。
 * 这些跨不同执行上下文共享的标量使用volatile。
 */
static volatile bool s_started = false;
static volatile uint16_t s_current_pulse_us = SERVO_PWM_NEUTRAL_US;
static volatile uint16_t s_target_pulse_us = SERVO_PWM_NEUTRAL_US;
static volatile uint16_t s_tick_count_ms = 0U;

static uint16_t clamp_test_pulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PWM_TEST_MIN_US)
    {
        return SERVO_PWM_TEST_MIN_US;
    }
    if (pulse_us > SERVO_PWM_TEST_MAX_US)
    {
        return SERVO_PWM_TEST_MAX_US;
    }
    return pulse_us;
}

bool ServoOutput_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == 0)
    {
        return false;
    }

    s_timer = htim;
    s_channel = channel;
    s_current_pulse_us = SERVO_PWM_NEUTRAL_US;
    s_target_pulse_us = SERVO_PWM_NEUTRAL_US;
    s_tick_count_ms = 0U;

    /*
     * 必须先写中位比较值，再启动PWM。
     * 这样输出启动的第一帧就是安全中位，不会先出现零脉宽或随机脉宽。
     */
    __HAL_TIM_SET_COMPARE(s_timer, s_channel, s_current_pulse_us);
    if (HAL_TIM_PWM_Start(s_timer, s_channel) != HAL_OK)
    {
        s_timer = 0;
        s_channel = 0U;
        s_started = false;
        return false;
    }

    s_started = true;
    return true;
}

void ServoOutput_SetTargetPulseUs(uint16_t pulse_us)
{
    /* 即使上层误传 0 或 2200，也只能落在当前台架安全范围内。 */
    s_target_pulse_us = clamp_test_pulse(pulse_us);
}

void ServoOutput_SetNeutral(void)
{
    s_target_pulse_us = SERVO_PWM_NEUTRAL_US;
}

void ServoOutput_On1msTick(void)
{
    uint16_t difference;

    if (!s_started || (s_timer == 0))
    {
        return;
    }

    /*
     * SysTick固定每1 ms调用一次；累计到软件更新周期后才修改一次CCR。
     * 这样实际脉宽的渐变节拍不再依赖主循环速度。
     */
    s_tick_count_ms++;
    if (s_tick_count_ms < SERVO_COMMAND_UPDATE_MS)
    {
        return;
    }
    s_tick_count_ms = 0U;

    if (s_current_pulse_us < s_target_pulse_us)
    {
        difference = (uint16_t)(s_target_pulse_us - s_current_pulse_us);
        if (difference > SERVO_MAX_STEP_US)
        {
            difference = SERVO_MAX_STEP_US;
        }
        s_current_pulse_us = (uint16_t)(s_current_pulse_us + difference);
    }
    else if (s_current_pulse_us > s_target_pulse_us)
    {
        difference = (uint16_t)(s_current_pulse_us - s_target_pulse_us);
        if (difference > SERVO_MAX_STEP_US)
        {
            difference = SERVO_MAX_STEP_US;
        }
        s_current_pulse_us = (uint16_t)(s_current_pulse_us - difference);
    }

    __HAL_TIM_SET_COMPARE(s_timer, s_channel, s_current_pulse_us);
}

void ServoOutput_Stop(void)
{
    if (s_started && (s_timer != 0))
    {
        (void)HAL_TIM_PWM_Stop(s_timer, s_channel);
    }
    s_started = false;
    s_tick_count_ms = 0U;
}

bool ServoOutput_IsStarted(void)
{
    return s_started;
}

uint16_t ServoOutput_GetCurrentPulseUs(void)
{
    return s_current_pulse_us;
}

uint16_t ServoOutput_GetTargetPulseUs(void)
{
    return s_target_pulse_us;
}
