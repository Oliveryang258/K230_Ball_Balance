#include <assert.h>
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

static void run_sample(BallController *controller,
                       uint16_t frame_id,
                       uint32_t tick_ms,
                       int16_t error_px,
                       float velocity_px_s,
                       float ki)
{
    VisionMeasurement measurement = {0};

    measurement.ball_valid = true;
    measurement.ball_safe = true;
    measurement.frame_id = frame_id;
    measurement.ball_x =
        (int16_t)(BALL_CONTROL_TRACK_CENTER_X_PX - error_px);

    assert(BallController_AcceptMeasurementEx(
        controller,
        &measurement,
        tick_ms
    ) == BALL_MEAS_ACCEPTED);

    /*
     * 速度由测试直接注入，用来确认旧版I不会根据速度改变积分倍率。
     */
    controller->velocity_px_s = velocity_px_s;
    assert(BallController_StepPid(
        controller,
        0.0f,
        ki,
        0.0f,
        -1
    ));
}

static void test_full_ki_is_used_even_while_ball_is_moving(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 200.0f, 2.0f);
    assert_near(controller.i_term_us, 0.0f, 0.001f);

    run_sample(&controller, 2U, 120U, 20, 200.0f, 2.0f);
    assert_near(controller.i_term_us, 0.8f, 0.001f);

    /* 没有新视觉帧时，50 Hz控制周期不得重复积分。 */
    assert(BallController_StepPid(
        &controller,
        0.0f,
        2.0f,
        0.0f,
        -1
    ));
    assert_near(controller.i_term_us, 0.8f, 0.001f);
}

static void test_integral_zone_and_limit(void)
{
    BallController controller;

    BallController_Init(&controller);
    run_sample(&controller, 1U, 100U, 20, 0.0f, 2.0f);
    run_sample(&controller, 2U, 120U, 20, 0.0f, 2.0f);
    assert_near(controller.i_term_us, 0.8f, 0.001f);

    /* 超出旧版积分区时冻结已有I，不继续累计，也不清零。 */
    run_sample(
        &controller,
        3U,
        140U,
        BALL_CONTROL_LEGACY_INTEGRAL_ZONE_PX + 1,
        0.0f,
        2.0f
    );
    assert_near(controller.i_term_us, 0.8f, 0.001f);

    controller.i_term_us = BALL_CONTROL_INTEGRAL_MAX_US - 0.1f;
    run_sample(
        &controller,
        4U,
        160U,
        BALL_CONTROL_LEGACY_INTEGRAL_ZONE_PX,
        0.0f,
        2.0f
    );
    assert_near(
        controller.i_term_us,
        BALL_CONTROL_INTEGRAL_MAX_US,
        0.001f
    );
}

int main(void)
{
    assert(BALL_CONTROL_USE_LEGACY_INTEGRAL != 0U);
    test_full_ki_is_used_even_while_ball_is_moving();
    test_integral_zone_and_limit();
    puts("ball-controller legacy-integral tests passed");
    return 0;
}
