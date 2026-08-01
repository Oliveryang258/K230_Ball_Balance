#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "ball_controller.h"

static VisionMeasurement make_measurement(
    uint16_t frame_id,
    int16_t ball_x
)
{
    VisionMeasurement measurement = {0};

    measurement.ball_valid = true;
    measurement.ball_safe = true;
    measurement.frame_id = frame_id;
    measurement.ball_x = ball_x;
    measurement.error_px = 1234;
    return measurement;
}

static void test_target_and_hold_pwm_are_clamped(void)
{
    BallController controller;

    BallController_Init(&controller);
    assert(controller.target_x == BALL_CONTROL_DEFAULT_TARGET_X);
    assert(
        controller.equilibrium_pulse_us ==
        BALL_CONTROL_DEFAULT_HOLD_PWM_US
    );

    assert(
        BallController_SetTargetX(&controller, -100) ==
        VISION_SAFE_X_MIN
    );
    assert(
        BallController_SetTargetX(&controller, 700) ==
        VISION_SAFE_X_MAX
    );
    assert(
        BallController_SetEquilibriumPulseUs(&controller, 0U) ==
        BALL_CONTROL_PWM_MIN_US
    );
    assert(
        BallController_SetEquilibriumPulseUs(&controller, 65535U) ==
        BALL_CONTROL_PWM_MAX_US
    );
}

static void test_competition_target_presets(void)
{
    assert(BALL_CONTROL_TRACK_CENTER_X_PX == 314);
    assert(BALL_CONTROL_TARGET_NEGATIVE_5CM_X == 190);
    assert(BALL_CONTROL_TARGET_POSITIVE_5CM_X == 440);
    assert(BALL_CONTROL_ERROR_1CM_PX == 25);
}

static void test_stm32_computes_error_from_local_target(void)
{
    BallController controller;
    VisionMeasurement measurement;

    BallController_Init(&controller);
    assert(BallController_SetTargetX(&controller, 500) == 500);
    assert(
        BallController_SetEquilibriumPulseUs(&controller, 1700U) ==
        1700U
    );

    measurement = make_measurement(1U, 450);
    assert(
        BallController_AcceptMeasurementEx(
            &controller,
            &measurement,
            100U
        ) == BALL_MEAS_ACCEPTED
    );
    assert(controller.error_px == 50);
    assert(BallController_StepPid(
        &controller,
        1.0f,
        0.0f,
        0.0f,
        -1
    ));
    assert(controller.target_pulse_us == 1650U);

    /*
     * 自动切换目标时，旧目标下积累的I项必须清零；速度历史则保留，
     * 这样既避免旧积分造成反向冲击，又不会丢掉当前运动趋势。
     */
    controller.i_term_us = 25.0f;
    controller.integral_candidate_ms = 80U;
    assert(controller.has_history);
    assert(BallController_SetTargetX(&controller, 400) == 400);
    assert(controller.error_px == -50);
    assert(controller.i_term_us == 0.0f);
    assert(controller.integral_candidate_ms == 0U);
    assert(controller.has_history);

    /* 连续参考轨迹接口必须保留I，仅更新参考位置和当前位置误差。 */
    controller.i_term_us = 12.0f;
    controller.integral_candidate_ms = 60U;
    assert(BallController_TrackTargetX(&controller, 408) == 408);
    assert(controller.error_px == -42);
    assert(controller.i_term_us == 12.0f);
    assert(controller.integral_candidate_ms == 60U);
}

int main(void)
{
    test_target_and_hold_pwm_are_clamped();
    test_competition_target_presets();
    test_stm32_computes_error_from_local_target();
    puts("ball-controller arbitrary-target tests passed");
    return 0;
}
