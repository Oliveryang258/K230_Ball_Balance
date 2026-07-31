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
    controller->last_dt_ms = 0U;
    controller->velocity_history_head = 0U;
    controller->velocity_history_count = 0U;
    controller->target_x = BALL_CONTROL_DEFAULT_TARGET_X;
    controller->equilibrium_pulse_us =
        BALL_CONTROL_DEFAULT_HOLD_PWM_US;
    controller->error_px = 0;
    controller->velocity_px_s = 0.0f;
    controller->p_term_us = 0.0f;
    controller->i_term_us = 0.0f;
    controller->d_term_us = 0.0f;
    controller->control_offset_us = 0.0f;
    controller->target_pulse_us =
        controller->equilibrium_pulse_us;
}

void BallController_Init(BallController *controller)
{
    BallController_Reset(controller);
}

static int16_t set_target_x(
    BallController *controller,
    int16_t target_x,
    bool reset_slow_state
)
{
    if (target_x < VISION_SAFE_X_MIN)
    {
        target_x = VISION_SAFE_X_MIN;
    }
    else if (target_x > VISION_SAFE_X_MAX)
    {
        target_x = VISION_SAFE_X_MAX;
    }

    if (controller != 0)
    {
        /*
         * 目标点发生变化时，旧目标下积累的 I 项不能继续作用于新目标。
         * 这里只清除与旧目标绑定的慢状态，不清除视觉位置和速度历史，
         * 因此下一个 50 Hz 控制周期仍可立即使用当前速度。
         */
        if (reset_slow_state &&
            (target_x != controller->target_x))
        {
            BallController_ResetTargetSlowState(controller);
        }

        controller->target_x = target_x;
        if (controller->has_history)
        {
            controller->error_px =
                (int16_t)(target_x - controller->last_ball_x);
        }
    }
    return target_x;
}

int16_t BallController_SetTargetX(
    BallController *controller,
    int16_t target_x
)
{
    return set_target_x(controller, target_x, true);
}

int16_t BallController_TrackTargetX(
    BallController *controller,
    int16_t target_x
)
{
    return set_target_x(controller, target_x, false);
}

void BallController_ResetTargetSlowState(BallController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->i_term_us = 0.0f;
}

void BallController_ClearVelocityHistory(BallController *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->has_history = false;
    controller->velocity_history_head = 0U;
    controller->velocity_history_count = 0U;
}

uint16_t BallController_SetEquilibriumPulseUs(
    BallController *controller,
    uint16_t equilibrium_pulse_us
)
{
    if (equilibrium_pulse_us < BALL_CONTROL_PWM_MIN_US)
    {
        equilibrium_pulse_us = BALL_CONTROL_PWM_MIN_US;
    }
    else if (equilibrium_pulse_us > BALL_CONTROL_PWM_MAX_US)
    {
        equilibrium_pulse_us = BALL_CONTROL_PWM_MAX_US;
    }

    if (controller != 0)
    {
        controller->equilibrium_pulse_us =
            equilibrium_pulse_us;
    }
    return equilibrium_pulse_us;
}

BallMeasurementResult BallController_AcceptMeasurementEx(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t measurement_tick_ms
)
{
    uint32_t dt_ms;
    uint32_t velocity_dt_ms;
    float raw_velocity_px_s;
    uint8_t history_offset;
    uint8_t history_index;
    bool velocity_baseline_found;
    uint8_t velocity_sample_count;
    float sample_time_ms;
    float sample_position_px;
    float sample_count_float;
    float sum_time_ms;
    float sum_position_px;
    float sum_time_squared;
    float sum_time_position;
    float regression_denominator;

    if ((controller == 0) || (measurement == 0))
    {
        return BALL_MEAS_INVALID;
    }

    controller->updated = false;

    /*
     * 即使 main.c 已经经过 ControlGuard，这里仍做一次防御性检查。
     * 无效数据绝不能被当作“误差为零”的有效测量。
     */
    if ((!measurement->ball_valid) || (!measurement->ball_safe))
    {
        return BALL_MEAS_INVALID;
    }

    /* 控制中断比相机更新快，同一个frame_id只允许做一次位置差分。 */
    if (controller->has_history &&
        (measurement->frame_id == controller->last_frame_id))
    {
        return BALL_MEAS_DUPLICATE;
    }

    /*
     * 最后一层跳变防御：位置相对上一有效帧相差过大时丢弃本帧。
     * K230已做速度外推过滤，正常帧位置连续，这里只拦截通信错误或
     * K230端遗漏的异常帧。必须在写入error_px/环形历史之前返回，
     * 这样不更新last_ball_x，下一帧仍可与上一有效帧正常差分。
     */
    if (controller->has_history &&
        ((measurement->ball_x >
          controller->last_ball_x +
          (int16_t)BALL_CONTROL_REJECT_JUMP_PX) ||
         (measurement->ball_x <
          controller->last_ball_x -
          (int16_t)BALL_CONTROL_REJECT_JUMP_PX)))
    {
        return BALL_MEAS_REJECTED;
    }

    controller->error_px =
        (int16_t)(controller->target_x - measurement->ball_x);

    /*
     * 第一帧没有前一位置，速度定义为 0。
     * 若两帧相隔过久，也放弃这一次差分，避免通信恢复时出现巨大速度尖峰。
     */
    if (!controller->has_history)
    {
        controller->last_dt_ms = 0U;
        controller->velocity_px_s = 0.0f;
    }
    else
    {
        dt_ms = (uint32_t)(measurement_tick_ms - controller->last_tick_ms);
        controller->last_dt_ms = dt_ms;
        if ((dt_ms < BALL_CONTROL_MIN_SAMPLE_MS) ||
            (dt_ms > BALL_CONTROL_MAX_SAMPLE_MS))
        {
            /*
             * 异常dt时不接受本帧，也不把它作为下一次差分基准。
             * 清除历史后，下一帧会以速度0重新建立基线，避免速度尖峰。
             */
            BallController_Reset(controller);
            controller->last_dt_ms = dt_ms;
            return BALL_MEAS_INVALID_DT;
        }
    }

    /*
     * 把本次位置及其UART完整帧接收时刻写入固定长度环形历史。
     * 不保存图像，也不做动态内存分配。
     */
    controller->velocity_x_history[controller->velocity_history_head] =
        measurement->ball_x;
    controller->velocity_tick_history[controller->velocity_history_head] =
        measurement_tick_ms;
    controller->velocity_history_head =
        (uint8_t)((controller->velocity_history_head + 1U) %
                  BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY);
    if (controller->velocity_history_count <
        BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY)
    {
        controller->velocity_history_count++;
    }

    /*
     * 对窗口内所有“时间—位置”样本做一维最小二乘直线拟合，斜率就是
     * 钢球速度。相比只看窗口首尾：
     * - 每一帧都会参与估计，单个端点跳1 px的影响更小；
     * - 使用每帧真实UART接收时间，不假定视觉周期固定；
     * - 不累加控制误差，也不累加绝对位移。
     *
     * 时间以当前帧为0，过去样本为负毫秒，减小浮点数数值范围。
     * 从当前帧向过去扫描，包含第一个年龄达到100 ms的样本后停止，
     * 因而窗口总是尽量接近但不短于目标时间。
     */
    velocity_baseline_found = false;
    velocity_dt_ms = 0U;
    velocity_sample_count = 0U;
    sum_time_ms = 0.0f;
    sum_position_px = 0.0f;
    sum_time_squared = 0.0f;
    sum_time_position = 0.0f;

    for (history_offset = 1U;
         history_offset <= controller->velocity_history_count;
         history_offset++)
    {
        history_index =
            (uint8_t)((controller->velocity_history_head +
                       BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY -
                       history_offset) %
                      BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY);
        velocity_dt_ms =
            (uint32_t)(measurement_tick_ms -
                       controller->velocity_tick_history[history_index]);

        sample_time_ms = -(float)velocity_dt_ms;
        sample_position_px =
            (float)controller->velocity_x_history[history_index];
        sum_time_ms += sample_time_ms;
        sum_position_px += sample_position_px;
        sum_time_squared += sample_time_ms * sample_time_ms;
        sum_time_position += sample_time_ms * sample_position_px;
        velocity_sample_count++;

        if (velocity_dt_ms >= BALL_CONTROL_VELOCITY_WINDOW_MS)
        {
            velocity_baseline_found = true;
            break;
        }
    }

    if (velocity_baseline_found && (velocity_sample_count >= 2U))
    {
        sample_count_float = (float)velocity_sample_count;
        regression_denominator =
            sample_count_float * sum_time_squared -
            sum_time_ms * sum_time_ms;

        if (regression_denominator > 0.0f)
        {
            /* 拟合斜率单位为px/ms，乘1000转换为px/s。 */
            raw_velocity_px_s =
                (sample_count_float * sum_time_position -
                 sum_time_ms * sum_position_px) /
                regression_denominator *
                1000.0f;

            controller->velocity_px_s =
                BALL_CONTROL_VELOCITY_ALPHA * raw_velocity_px_s +
                (1.0f - BALL_CONTROL_VELOCITY_ALPHA) *
                controller->velocity_px_s;
        }
    }

    controller->last_frame_id = measurement->frame_id;
    controller->last_ball_x = measurement->ball_x;
    controller->last_tick_ms = measurement_tick_ms;
    controller->has_history = true;
    controller->updated = true;
    return BALL_MEAS_ACCEPTED;
}

bool BallController_AcceptMeasurement(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t measurement_tick_ms
)
{
    return BallController_AcceptMeasurementEx(
        controller,
        measurement,
        measurement_tick_ms
    ) == BALL_MEAS_ACCEPTED;
}

bool BallController_StepPid(
    BallController *controller,
    float kp_us_per_px,
    float ki_us_per_px_s,
    float kv_us_per_px_s,
    int8_t direction
)
{
    int16_t effective_error_px;
    int32_t requested_pulse_us;
    int8_t safe_direction;
    float previous_i_term_us;
    float integral_delta_us = 0.0f;
    float directed_integral_delta_us;
    bool may_integrate = false;

    if ((controller == 0) || (!controller->has_history))
    {
        return false;
    }

    effective_error_px = controller->error_px;
    if ((effective_error_px >= -BALL_CONTROL_DEADBAND_PX) &&
        (effective_error_px <= BALL_CONTROL_DEADBAND_PX))
    {
        effective_error_px = 0;
    }

    /*
     * 速度项使用负号形成阻尼：
     * 球正在朝误差增大的方向运动时，D 项会提前反向制动。
     */
    controller->p_term_us = kp_us_per_px * (float)effective_error_px;
    controller->d_term_us = -kv_us_per_px_s * controller->velocity_px_s;

    /*
     * 位置I只消费”新视觉测量”事件，不按50 Hz对同一帧反复累计。
     *
     * 积分分离采用“暂停但保留”：
     * - 中心死区内不继续积分，但保留维持静态平衡所需的I；
     * - 超出积分作用区时暂停积分，避免大误差期间继续积累；
     * - 不能因为单帧视觉抖动进入死区或越过积分区就突然把I清零。
     *
     * 只有Ki被明确设为0，或上层guard调用BallController_Reset()时才清零。
     */
    previous_i_term_us = controller->i_term_us;

    if (ki_us_per_px_s <= 0.0f)
    {
        controller->i_term_us = 0.0f;
    }
    else if ((effective_error_px != 0) &&
             (effective_error_px >=
              -BALL_CONTROL_LEGACY_INTEGRAL_ZONE_PX) &&
             (effective_error_px <=
              BALL_CONTROL_LEGACY_INTEGRAL_ZONE_PX) &&
             controller->updated &&
             (controller->last_dt_ms >= BALL_CONTROL_MIN_SAMPLE_MS) &&
             (controller->last_dt_ms <= BALL_CONTROL_MAX_SAMPLE_MS))
    {
        integral_delta_us =
            ki_us_per_px_s *
            (float)effective_error_px *
            ((float)controller->last_dt_ms / 1000.0f);
        controller->i_term_us += integral_delta_us;

        if (controller->i_term_us > BALL_CONTROL_INTEGRAL_MAX_US)
        {
            controller->i_term_us = BALL_CONTROL_INTEGRAL_MAX_US;
        }
        else if (controller->i_term_us < -BALL_CONTROL_INTEGRAL_MAX_US)
        {
            controller->i_term_us = -BALL_CONTROL_INTEGRAL_MAX_US;
        }
        may_integrate = true;
    }
    controller->updated = false;

    safe_direction = (direction < 0) ? -1 : 1;
    controller->control_offset_us =
        (float)safe_direction *
        (controller->p_term_us +
         controller->i_term_us +
         controller->d_term_us);

    requested_pulse_us =
        (int32_t)controller->equilibrium_pulse_us +
        round_float_to_i32(controller->control_offset_us);
    controller->target_pulse_us =
        clamp_control_pulse(requested_pulse_us, &controller->saturated);

    /*
     * 抗积分饱和：若本次新增长的I正把输出继续推向已经触发的PWM限幅，
     * 就撤销这一次积分，再重新计算输出。解除饱和方向的积分仍然允许。
     */
    directed_integral_delta_us =
        (float)safe_direction *
        (controller->i_term_us - previous_i_term_us);
    if (controller->saturated && may_integrate &&
        (((requested_pulse_us > (int32_t)BALL_CONTROL_PWM_MAX_US) &&
          (directed_integral_delta_us > 0.0f)) ||
         ((requested_pulse_us < (int32_t)BALL_CONTROL_PWM_MIN_US) &&
          (directed_integral_delta_us < 0.0f))))
    {
        controller->i_term_us = previous_i_term_us;
        controller->control_offset_us =
            (float)safe_direction *
            (controller->p_term_us +
             controller->i_term_us +
             controller->d_term_us);
        requested_pulse_us =
            (int32_t)controller->equilibrium_pulse_us +
            round_float_to_i32(controller->control_offset_us);
        controller->target_pulse_us =
            clamp_control_pulse(requested_pulse_us, &controller->saturated);
    }

    return true;
}

bool BallController_Step(
    BallController *controller,
    float kp_us_per_px,
    float kv_us_per_px_s,
    int8_t direction
)
{
    return BallController_StepPid(
        controller,
        kp_us_per_px,
        0.0f,
        kv_us_per_px_s,
        direction
    );
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
    if (!BallController_AcceptMeasurement(controller, measurement, now_ms))
    {
        return false;
    }

    return BallController_Step(
        controller,
        kp_us_per_px,
        kv_us_per_px_s,
        direction
    );
}

uint16_t BallController_GetTargetPulseUs(const BallController *controller)
{
    if (controller == 0)
    {
        return SERVO_PWM_NEUTRAL_US;
    }
    return controller->target_pulse_us;
}
