#include "servo_output.h"
#include "app_config.h"

static TIM_HandleTypeDef *s_timer = 0;
static uint32_t s_channel = 0U;
static bool s_started = false;
static uint16_t s_current_pulse_us = SERVO_PWM_NEUTRAL_US;
static uint16_t s_target_pulse_us = SERVO_PWM_NEUTRAL_US;
static uint32_t s_last_update_ms = 0U;

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
    s_last_update_ms = HAL_GetTick();

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

void ServoOutput_Process(uint32_t now_ms)
{
    uint16_t difference;

    if (!s_started || (s_timer == 0))
    {
        return;
    }

    /* 使用无符号减法，可正确处理 HAL_GetTick() 的自然回绕。 */
    if ((uint32_t)(now_ms - s_last_update_ms) < SERVO_COMMAND_UPDATE_MS)
    {
        return;
    }
    s_last_update_ms = now_ms;

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

