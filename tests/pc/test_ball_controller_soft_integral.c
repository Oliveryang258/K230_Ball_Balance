#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "ball_controller.h"

static void assert_near(float actual, float expected, float tolerance)
{
    float difference = actual - expected;

    if (difference < 0.0f)
    {
        difference = -difference;
    }
    assert(difference <= tolerance);
}

static void accept_sample(
    BallController *controller,
    uint16_t frame_id,
    uint32_t tick_ms,
    int16_t error_px
)
{
    VisionMeasurement measurement = {0};

    measurement.ball_valid = true;
    measurement.ball_safe = true;
    measurement.frame_id = frame_id;
    measurement.ball_x =
        (int16_t)(BALL_CONTROL_DEFAULT_TARGET_X - error_px);

    assert(BallController_AcceptMeasurementEx(
        controller,
        &measurement,
        tick_ms
    ) == BALL_MEAS_ACCEPTED);
}

static void step_pid(BallController *controller)
{
    assert(BallController_StepPid(
        controller,
        0.0f,
        1.0f,
        0.0f,
        -1
    ));
}

static void test_active_unwind_is_one_us_per_control_period(void)
{
    BallController controller;
    uint8_t cycle;

    BallController_Init(&controller);
    accept_sample(&controller, 1U, 100U, 20);
    step_pid(&controller);
    controller.i_term_us = 105.0f;

    BallController_SetIntegralSoftLimit(&controller, 60.0f, 1.0f);
    step_pid(&controller);
    assert_near(controller.i_term_us, 104.0f, 0.001f);
    assert(controller.integral_soft_unwind_active);

    for (cycle = 0U; cycle < 44U; cycle++)
    {
        BallController_SetIntegralSoftLimit(&controller, 60.0f, 1.0f);
        step_pid(&controller);
    }
    assert_near(controller.i_term_us, 60.0f, 0.001f);

    BallController_SetIntegralSoftLimit(&controller, 60.0f, 1.0f);
    step_pid(&controller);
    assert_near(controller.i_term_us, 60.0f, 0.001f);
    assert(!controller.integral_soft_unwind_active);
}

static void test_helpful_error_uses_real_dt_instead_of_fixed_unwind(void)
{
    BallController controller;

    BallController_Init(&controller);
    accept_sample(&controller, 1U, 100U, 0);
    step_pid(&controller);
    controller.i_term_us = 100.0f;

    BallController_SetIntegralSoftLimit(&controller, 60.0f, 1.0f);
    accept_sample(&controller, 2U, 120U, -20);
    step_pid(&controller);

    /* Ki=1、error=-20、dt=20 ms，自然退积分0.4 us。 */
    assert_near(controller.i_term_us, 99.6f, 0.001f);
    assert(!controller.integral_soft_unwind_active);
}

static void test_harmful_error_is_blocked_and_actively_unwound(void)
{
    BallController controller;

    BallController_Init(&controller);
    accept_sample(&controller, 1U, 100U, 0);
    step_pid(&controller);
    controller.i_term_us = 100.0f;

    BallController_SetIntegralSoftLimit(&controller, 60.0f, 1.0f);
    accept_sample(&controller, 2U, 120U, 20);
    step_pid(&controller);

    /* 正误差本会继续增加正I，因此阻止增长并主动释放1 us。 */
    assert_near(controller.i_term_us, 99.0f, 0.001f);
    assert(controller.integral_soft_unwind_active);
}

static void test_legacy_limit_still_clamps_immediately(void)
{
    BallController controller;

    BallController_Init(&controller);
    accept_sample(&controller, 1U, 100U, 0);
    step_pid(&controller);
    controller.i_term_us = 105.0f;

    BallController_SetIntegralLimit(&controller, 60.0f);
    step_pid(&controller);
    assert_near(controller.i_term_us, 60.0f, 0.001f);
}

int main(void)
{
    test_active_unwind_is_one_us_per_control_period();
    test_helpful_error_uses_real_dt_instead_of_fixed_unwind();
    test_harmful_error_is_blocked_and_actively_unwound();
    test_legacy_limit_still_clamps_immediately();
    puts("ball-controller soft-integral tests passed");
    return 0;
}
