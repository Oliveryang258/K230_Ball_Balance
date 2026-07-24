#include "ball_controller.h"
#include "app_config.h"

static uint16_t clamp_control_pulse(int32_t pulse_us, bool *saturated)
{
    if (pulse_us < (int32_t)BALL_CONTROL_PWM_MIN_US)
    {
        *saturated = true;
        return BALL_CONTROL_PWM_MIN_US;
    }
    if (pulse_us > (int32_t)BALL_CONTROL_PWM_MAX_US)
    {
        *saturated = true;
        return BALL_CONTROL_PWM_MAX_US;
    }

    *saturated = false;
    return (uint16_t)pulse_us;
}

/* 不依赖 libm，避免为了一个四舍五入函数额外链接数学库。 */
static int32_t round_float_to_i32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

void BallController_Reset(BallController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->has_history = false;
    controller->updated = false;
    controller->saturated = false;
    controller->last_frame_id = 0U;
    controller->last_ball_x = 0;
    controller->last_tick_ms = 0U;
    controller->error_px = 0;
    controller->velocity_px_s = 0.0f;
    controller->p_term_us = 0.0f;
    controller->d_term_us = 0.0f;
    controller->control_offset_us = 0.0f;
    controller->target_pulse_us = SERVO_PWM_NEUTRAL_US;
}

void BallController_Init(BallController *controller)
{
    BallController_Reset(controller);
}

bool BallController_Update(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t now_ms,
    float kp_us_per_px,
    float kv_us_per_px_s,
    int8_t direction
)
{
    uint32_t dt_ms;
    float raw_velocity_px_s;
    int16_t effective_error_px;
    int32_t requested_pulse_us;
    int8_t safe_direction;

    if ((controller == 0) || (measurement == 0))
    {
        return false;
    }

    controller->updated = false;

    /*
     * 即使 main.c 已经经过 ControlGuard，这里仍做一次防御性检查。
     * 无效数据绝不能被当作“误差为零”的有效测量。
     */
    if ((!measurement->ball_valid) || (!measurement->ball_safe))
    {
        return false;
    }

    /* 主循环比相机帧率快得多，同一个 frame_id 只允许计算一次。 */
    if (controller->has_history &&
        (measurement->frame_id == controller->last_frame_id))
    {
        return false;
    }

    controller->error_px = measurement->error_px;
    effective_error_px = measurement->error_px;
    if ((effective_error_px >= -BALL_CONTROL_DEADBAND_PX) &&
        (effective_error_px <= BALL_CONTROL_DEADBAND_PX))
    {
        effective_error_px = 0;
    }

    /*
     * 第一帧没有前一位置，速度定义为 0。
     * 若两帧相隔过久，也放弃这一次差分，避免通信恢复时出现巨大速度尖峰。
     */
    if (!controller->has_history)
    {
        controller->velocity_px_s = 0.0f;
    }
    else
    {
        dt_ms = (uint32_t)(now_ms - controller->last_tick_ms);
        if ((dt_ms == 0U) || (dt_ms > BALL_CONTROL_MAX_SAMPLE_MS))
        {
            controller->velocity_px_s = 0.0f;
        }
        else
        {
            raw_velocity_px_s =
                ((float)(measurement->ball_x - controller->last_ball_x) * 1000.0f) /
                (float)dt_ms;

            controller->velocity_px_s =
                BALL_CONTROL_VELOCITY_ALPHA * raw_velocity_px_s +
                (1.0f - BALL_CONTROL_VELOCITY_ALPHA) *
                controller->velocity_px_s;
        }
    }

    /*
     * 速度项使用负号形成阻尼：
     * 球正在朝误差增大的方向运动时，D 项会提前反向制动。
     */
    controller->p_term_us = kp_us_per_px * (float)effective_error_px;
    controller->d_term_us = -kv_us_per_px_s * controller->velocity_px_s;

    safe_direction = (direction < 0) ? -1 : 1;
    controller->control_offset_us =
        (float)safe_direction *
        (controller->p_term_us + controller->d_term_us);

    requested_pulse_us =
        (int32_t)SERVO_PWM_NEUTRAL_US +
        round_float_to_i32(controller->control_offset_us);
    controller->target_pulse_us =
        clamp_control_pulse(requested_pulse_us, &controller->saturated);

    controller->last_frame_id = measurement->frame_id;
    controller->last_ball_x = measurement->ball_x;
    controller->last_tick_ms = now_ms;
    controller->has_history = true;
    controller->updated = true;
    return true;
}

uint16_t BallController_GetTargetPulseUs(const BallController *controller)
{
    if (controller == 0)
    {
        return SERVO_PWM_NEUTRAL_US;
    }
    return controller->target_pulse_us;
}
