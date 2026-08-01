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
    if (difference > tolerance)
    {
        fprintf(
            stderr,
            "assert_near failed: actual=%.6f expected=%.6f tolerance=%.6f\n",
            (double)actual,
            (double)expected,
            (double)tolerance
        );
    }
    assert(difference <= tolerance);
}

static void run_sample(
    BallController *controller,
    uint16_t frame_id,
    uint32_t tick_ms,
    int16_t error_px,
    float velocity_px_s,
    float kp,
    float kv
)
{
    VisionMeasurement measurement = {0};

    measurement.ball_valid = true;
    measurement.ball_safe = true;
    measurement.frame_id = frame_id;
    measurement.ball_x = (int16_t)(322 - error_px);
    measurement.error_px = error_px;

    assert(BallController_AcceptMeasurementEx(
        controller,
        &measurement,
        tick_ms
    ) == BALL_MEAS_ACCEPTED);
    controller->velocity_px_s = velocity_px_s;
    assert(BallController_StepPid(
        controller,
        kp,
        0.0f,
        kv,
        -1
    ));
}

static void confirm_stuck(
    BallController *controller,
    int16_t error_px,
    float velocity_px_s
)
{
    run_sample(
        controller,
        1U,
        100U,
        error_px,
        velocity_px_s,
        1.2f,
        0.5f
    );
    assert(!controller->stiction_active);
    assert_near(controller->stiction_compensation_us, 0.0f, 0.001f);

    run_sample(
        controller,
        2U,
        120U,
        error_px,
        velocity_px_s,
        1.2f,
        0.5f
    );
    assert(controller->stiction_active);
}

static void test_two_new_samples_are_required(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 25, 0.0f, 1.2f, 0.5f);

    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 1U);
    assert_near(controller.stiction_compensation_us, 0.0f, 0.001f);

    assert(BallController_StepPid(
        &controller,
        1.2f,
        0.0f,
        0.5f,
        -1
    ));
    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 1U);
}

static void test_interrupted_confirmation_restarts(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 25, 0.0f, 1.2f, 0.5f);
    assert(controller.stiction_confirm_count == 1U);

    run_sample(&controller, 2U, 120U, 25, 7.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 0U);

    run_sample(&controller, 3U, 140U, 25, 0.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 1U);
}

static void test_speed_hysteresis_prevents_chatter(void)
{
    BallController controller;

    BallController_Init(&controller);
    confirm_stuck(&controller, 25, 0.0f);
    assert_near(controller.stiction_compensation_us, -80.0f, 0.001f);

    run_sample(&controller, 3U, 140U, 25, 8.0f, 1.2f, 0.5f);
    assert(controller.stiction_active);
    assert_near(controller.stiction_compensation_us, -80.0f, 0.001f);

    run_sample(&controller, 4U, 160U, 25, 11.9f, 1.2f, 0.5f);
    assert(controller.stiction_active);
    assert_near(controller.stiction_compensation_us, -80.0f, 0.001f);

    run_sample(&controller, 5U, 180U, 25, 12.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    assert_near(controller.stiction_compensation_us, 0.0f, 0.001f);
}

static void test_error_returning_to_center_exits(void)
{
    BallController controller;

    BallController_Init(&controller);
    confirm_stuck(&controller, 25, 0.0f);

    run_sample(&controller, 3U, 140U, 8, 0.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 0U);
    assert_near(controller.stiction_compensation_us, 0.0f, 0.001f);
}

static void test_error_direction_change_requires_reconfirmation(void)
{
    BallController controller;

    BallController_Init(&controller);
    confirm_stuck(&controller, 25, 0.0f);

    run_sample(&controller, 3U, 140U, -25, 0.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    assert_near(controller.stiction_compensation_us, 0.0f, 0.001f);

    run_sample(&controller, 4U, 160U, -25, 0.0f, 1.2f, 0.5f);
    assert(!controller.stiction_active);
    run_sample(&controller, 5U, 180U, -25, 0.0f, 1.2f, 0.5f);
    assert(controller.stiction_active);
    assert_near(controller.stiction_compensation_us, 130.0f, 0.001f);
}

static void test_addition_and_distance_schedule(void)
{
    BallController near_positive;
    BallController far_positive;
    BallController negative;

    BallController_Init(&near_positive);
    confirm_stuck(&near_positive, 9, 0.0f);
    assert_near(
        near_positive.stiction_compensation_us,
        -98.8235f,
        0.001f
    );
    assert_near(near_positive.control_offset_us, -109.6235f, 0.001f);
    assert(
        near_positive.target_pulse_us ==
        (uint16_t)(SERVO_PWM_NEUTRAL_US - 110U)
    );

    BallController_Init(&far_positive);
    confirm_stuck(&far_positive, 100, 0.0f);
    assert_near(
        far_positive.stiction_compensation_us,
        -80.0f,
        0.001f
    );
    assert_near(far_positive.control_offset_us, -200.0f, 0.001f);
    assert(
        far_positive.target_pulse_us ==
        (uint16_t)(SERVO_PWM_NEUTRAL_US - 200U)
    );

    BallController_Init(&negative);
    confirm_stuck(&negative, -25, 0.0f);
    assert_near(negative.stiction_compensation_us, 130.0f, 0.001f);
    assert_near(negative.control_offset_us, 160.0f, 0.001f);
    assert(
        negative.target_pulse_us ==
        (uint16_t)(SERVO_PWM_NEUTRAL_US + 160U)
    );
}

static void test_reset_clears_latch(void)
{
    BallController controller;

    BallController_Init(&controller);
    confirm_stuck(&controller, -25, 0.0f);

    BallController_Reset(&controller);
    assert(!controller.stiction_active);
    assert(controller.stiction_confirm_count == 0U);
    assert_near(controller.stiction_compensation_us, 0.0f, 0.001f);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);
}

int main(void)
{
    test_two_new_samples_are_required();
    test_interrupted_confirmation_restarts();
    test_speed_hysteresis_prevents_chatter();
    test_error_returning_to_center_exits();
    test_error_direction_change_requires_reconfirmation();
    test_addition_and_distance_schedule();
    test_reset_clears_latch();
    puts("ball-controller stiction-hysteresis tests passed");
    return 0;
}
