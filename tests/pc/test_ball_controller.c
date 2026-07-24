#include <assert.h>
#include <stdio.h>

#include "app_config.h"
#include "ball_controller.h"

static VisionMeasurement make_measurement(
    uint16_t frame_id,
    int16_t error_px,
    int16_t ball_x
)
{
    VisionMeasurement measurement;

    measurement.frame_id = frame_id;
    measurement.error_px = error_px;
    measurement.ball_x = ball_x;
    measurement.ball_valid = true;
    measurement.ball_safe = true;
    return measurement;
}

int main(void)
{
    BallController controller;
    VisionMeasurement measurement;
    bool updated;

    BallController_Init(&controller);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);

    /* 第一帧只有 P 项，没有伪造的速度。 */
    measurement = make_measurement(1U, 10, 351);
    updated = BallController_Update(&controller, &measurement, 1000U, 1.0f, 0.1f, 1);
    assert(updated);
    assert(controller.velocity_px_s == 0.0f);
    assert(controller.target_pulse_us == 1430U);

    /* 同一个 frame_id 不能重复积分或重复计算速度。 */
    updated = BallController_Update(&controller, &measurement, 1010U, 1.0f, 0.1f, 1);
    assert(!updated);

    /*
     * 50 ms 内 x 增加 10 px，原始速度为 +200 px/s；
     * alpha=0.5 后得到 +100 px/s，D 项为 -10 us。
     */
    measurement = make_measurement(2U, 0, 361);
    updated = BallController_Update(&controller, &measurement, 1050U, 1.0f, 0.1f, 1);
    assert(updated);
    assert((controller.velocity_px_s > 99.9f) &&
           (controller.velocity_px_s < 100.1f));
    assert(controller.target_pulse_us == 1410U);

    /* 方向取反后，完全相同的控制量应作用到中位另一侧。 */
    BallController_Reset(&controller);
    measurement = make_measurement(3U, 10, 351);
    updated = BallController_Update(&controller, &measurement, 2000U, 1.0f, 0.0f, -1);
    assert(updated);
    assert(controller.target_pulse_us == 1410U);

    /* 中心死区内 P 项为零。 */
    BallController_Reset(&controller);
    measurement = make_measurement(4U, BALL_CONTROL_DEADBAND_PX, 357);
    updated = BallController_Update(&controller, &measurement, 3000U, 5.0f, 0.0f, 1);
    assert(updated);
    assert(controller.p_term_us == 0.0f);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);

    /* 极大增益也必须被自动控制专用范围截住。 */
    BallController_Reset(&controller);
    measurement = make_measurement(5U, 300, 61);
    updated = BallController_Update(&controller, &measurement, 4000U, 10.0f, 0.0f, 1);
    assert(updated);
    assert(controller.saturated);
    assert(controller.target_pulse_us == BALL_CONTROL_PWM_MAX_US);

    measurement.ball_valid = false;
    assert(!BallController_Update(&controller, &measurement, 4050U, 1.0f, 0.0f, 1));

    puts("ball_controller tests passed");
    return 0;
}
