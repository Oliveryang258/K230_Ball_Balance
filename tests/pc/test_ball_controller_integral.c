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
    float ki
)
{
    VisionMeasurement measurement = {0};

    measurement.ball_valid = true;
    measurement.ball_safe = true;
    measurement.frame_id = frame_id;
    measurement.ball_x =
        (int16_t)(BALL_CONTROL_DEFAULT_TARGET_X - error_px);
    measurement.error_px = error_px;

    assert(BallController_AcceptMeasurementEx(
        controller,
        &measurement,
        tick_ms
    ) == BALL_MEAS_ACCEPTED);
    controller->velocity_px_s = velocity_px_s;
    assert(BallController_StepPid(
        controller,
        1.2f,
        ki,
        0.5f,
        -1
    ));
}

static void test_integral_waits_for_steady_confirmation(void)
{
    BallController controller;
    uint16_t frame_id;

    BallController_Init(&controller);
    for (frame_id = 1U; frame_id <= 8U; frame_id++)
    {
        run_sample(
            &controller,
            frame_id,
            (uint32_t)(80U + 20U * frame_id),
            20,
            0.0f,
            0.10f
        );
    }

    assert(controller.integral_candidate_ms == 140U);
    assert_near(
        controller.i_term_us,
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f *
        7.0f,
        0.001f
    );

    run_sample(&controller, 9U, 260U, 20, 0.0f, 0.10f);
    assert(controller.integral_candidate_ms == 160U);
    assert_near(
        controller.i_term_us,
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f *
        7.0f +
        0.040f,
        0.001f
    );

    run_sample(&controller, 10U, 280U, 20, 0.0f, 0.10f);
    assert_near(
        controller.i_term_us,
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f *
        7.0f +
        0.080f,
        0.001f
    );
}

static void test_reused_frame_does_not_advance_confirmation(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    assert(controller.integral_candidate_ms == 0U);

    assert(BallController_StepPid(
        &controller,
        1.2f,
        0.10f,
        0.5f,
        -1
    ));
    assert(controller.integral_candidate_ms == 0U);
    assert_near(controller.i_term_us, 0.0f, 0.001f);
}

static void test_motion_uses_direction_scheduled_integral(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    controller.i_term_us = 5.0f;

    /* 中速朝目标运动：使用35%的Ki。 */
    run_sample(&controller, 2U, 120U, 20, 10.0f, 0.10f);
    assert(controller.integral_candidate_ms == 0U);
    assert_near(
        controller.i_term_us,
        5.0f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f,
        0.001f
    );

    /* 高速朝目标运动：使用15%的Ki。 */
    run_sample(&controller, 3U, 140U, 20, 40.0f, 0.10f);
    assert(controller.integral_candidate_ms == 0U);
    assert_near(
        controller.i_term_us,
        5.0f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_FAST_SCALE *
        20.0f *
        0.020f,
        0.001f
    );

    /* 远离目标运动：恢复完整Ki。 */
    run_sample(&controller, 4U, 160U, 20, -10.0f, 0.10f);
    assert(controller.integral_candidate_ms == 0U);
    assert_near(
        controller.i_term_us,
        5.0f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_FAST_SCALE *
        20.0f *
        0.020f +
        0.040f,
        0.001f
    );
}

static void test_low_speed_preintegrates_before_confirmation(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.i_term_us = 10.0f;

    run_sample(&controller, 2U, 120U, 20, 0.0f, 0.10f);
    assert(controller.integral_candidate_ms == 20U);
    assert_near(
        controller.i_term_us,
        10.0f +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f,
        0.001f
    );
}

static void test_error_crossing_rapidly_unwinds_integral(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.i_term_us = 30.0f;

    run_sample(&controller, 2U, 120U, -10, 0.0f, 0.10f);
    assert(controller.integral_candidate_ms == 0U);
    assert_near(
        controller.i_term_us,
        30.0f -
        0.10f *
        BALL_CONTROL_INTEGRAL_UNWIND_SCALE *
        10.0f *
        0.020f,
        0.001f
    );
}

#if BALL_CONTROL_I_BREAKAWAY_ENABLED != 0U
static void test_breakaway_releases_starting_integral_once(void)
{
    BallController controller;
    float expected_before_release;
    float expected_after_release;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);

    /*
     * 模拟钢球已经低速卡住足够长时间，并积累了足以克服静摩擦的I。
     */
    controller.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    controller.i_term_us = 50.0f;
    run_sample(&controller, 2U, 120U, 20, 0.0f, 0.10f);
    assert(controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_release_count == 0U);

    /*
     * 第一帧正确方向运动只计数，不释放，避免单帧速度尖峰误触发。
     */
    run_sample(&controller, 3U, 140U, 20, 20.0f, 0.10f);
    assert(controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_moving_count == 1U);

    expected_before_release =
        controller.i_term_us +
        0.10f *
        BALL_CONTROL_INTEGRAL_APPROACH_MEDIUM_SCALE *
        20.0f *
        0.020f;

    /*
     * 第二帧仍向目标运动，确认已经脱离静摩擦，只保留设定比例的I。
     */
    run_sample(&controller, 4U, 160U, 20, 20.0f, 0.10f);
    expected_after_release =
        expected_before_release *
        BALL_CONTROL_I_BREAKAWAY_RETAIN_SCALE;
    assert(!controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_moving_count == 0U);
    assert(controller.integral_breakaway_release_count == 1U);
    assert_near(
        controller.i_term_us,
        expected_after_release,
        0.001f
    );

    /*
     * 继续运动不会再次释放；必须再次经历低速积累才能重新武装。
     */
    run_sample(&controller, 5U, 180U, 20, 20.0f, 0.10f);
    assert(controller.integral_breakaway_release_count == 1U);
    assert(controller.i_term_us > expected_after_release);
}

static void test_breakaway_wrong_direction_does_not_release(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    controller.i_term_us = 50.0f;
    run_sample(&controller, 2U, 120U, 20, 0.0f, 0.10f);
    assert(controller.integral_breakaway_armed);

    /* error为正而velocity为负，钢球正在远离目标，不能释放启动I。 */
    run_sample(&controller, 3U, 140U, 20, -20.0f, 0.10f);
    run_sample(&controller, 4U, 160U, 20, -20.0f, 0.10f);
    assert(controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_release_count == 0U);
    assert(controller.i_term_us > 50.0f);
}
#else
static void test_breakaway_disabled_keeps_integral(void)
{
    BallController controller;
    float integral_before_motion;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    controller.i_term_us = 50.0f;

    run_sample(&controller, 2U, 120U, 20, 0.0f, 0.10f);
    assert(!controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_release_count == 0U);
    integral_before_motion = controller.i_term_us;

    run_sample(&controller, 3U, 140U, 20, 20.0f, 0.10f);
    run_sample(&controller, 4U, 160U, 20, 20.0f, 0.10f);
    assert(!controller.integral_breakaway_armed);
    assert(controller.integral_breakaway_release_count == 0U);
    assert(controller.i_term_us > integral_before_motion);
}
#endif

static void test_integral_limit_and_negative_direction(void)
{
    BallController positive;
    BallController negative;

    BallController_Init(&positive);
    run_sample(&positive, 1U, 100U, 50, 0.0f, 0.10f);
    positive.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    positive.i_term_us = BALL_CONTROL_INTEGRAL_MAX_US - 0.01f;
    run_sample(&positive, 2U, 120U, 50, 0.0f, 0.10f);
    assert_near(
        positive.i_term_us,
        BALL_CONTROL_INTEGRAL_MAX_US,
        0.001f
    );

    BallController_Init(&negative);
    run_sample(&negative, 1U, 100U, -20, 0.0f, 0.10f);
    negative.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    run_sample(&negative, 2U, 120U, -20, 0.0f, 0.10f);
    assert_near(negative.i_term_us, -0.04f, 0.001f);
}

static void test_large_steady_error_is_not_eligible(void)
{
    BallController controller;
    uint16_t frame_id;

    BallController_Init(&controller);
    for (frame_id = 1U; frame_id <= 9U; frame_id++)
    {
        run_sample(
            &controller,
            frame_id,
            (uint32_t)(80U + 20U * frame_id),
            120,
            0.0f,
            0.10f
        );
    }

    assert(controller.integral_candidate_ms == 0U);
    assert_near(controller.i_term_us, 0.0f, 0.001f);
}

static void test_zero_ki_and_reset_clear_integral(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 0.10f);
    controller.integral_candidate_ms =
        BALL_CONTROL_INTEGRAL_CONFIRM_MS;
    controller.i_term_us = 10.0f;

    assert(BallController_StepPid(
        &controller,
        1.2f,
        0.0f,
        0.5f,
        -1
    ));
    assert(controller.integral_candidate_ms == 0U);
    assert_near(controller.i_term_us, 0.0f, 0.001f);

    controller.integral_candidate_ms = 200U;
    controller.i_term_us = 5.0f;
    BallController_Reset(&controller);
    assert(controller.integral_candidate_ms == 0U);
    assert_near(controller.i_term_us, 0.0f, 0.001f);
}

int main(void)
{
    test_integral_waits_for_steady_confirmation();
    test_reused_frame_does_not_advance_confirmation();
    test_motion_uses_direction_scheduled_integral();
    test_low_speed_preintegrates_before_confirmation();
    test_error_crossing_rapidly_unwinds_integral();
#if BALL_CONTROL_I_BREAKAWAY_ENABLED != 0U
    test_breakaway_releases_starting_integral_once();
    test_breakaway_wrong_direction_does_not_release();
#else
    test_breakaway_disabled_keeps_integral();
#endif
    test_integral_limit_and_negative_direction();
    test_large_steady_error_is_not_eligible();
    test_zero_ki_and_reset_clear_integral();
    puts("ball-controller conditional-integral tests passed");
    return 0;
}
