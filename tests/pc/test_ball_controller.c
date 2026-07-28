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
    BallMeasurementResult measurement_result;
    int sample_index;

    BallController_Init(&controller);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);
    assert(BALL_CONTROL_PWM_MIN_US ==
           (SERVO_PWM_NEUTRAL_US - BALL_CONTROL_PWM_SOFT_RANGE_US));
    assert(BALL_CONTROL_PWM_MAX_US ==
           (SERVO_PWM_NEUTRAL_US + BALL_CONTROL_PWM_SOFT_RANGE_US));

    /* 第一帧只有 P 项，没有伪造的速度。 */
    measurement = make_measurement(1U, 10, 351);
    updated = BallController_Update(&controller, &measurement, 1000U, 1.0f, 0.1f, 1);
    assert(updated);
    assert(controller.velocity_px_s == 0.0f);
    assert(controller.target_pulse_us == (SERVO_PWM_NEUTRAL_US + 10U));

    /* 同一个 frame_id 不能重复积分或重复计算速度。 */
    updated = BallController_Update(&controller, &measurement, 1010U, 1.0f, 0.1f, 1);
    assert(!updated);

    /* 第二帧累计时间不足100 ms，速度窗口仍在预热，不能产生伪速度。 */
    measurement = make_measurement(2U, 0, 361);
    updated = BallController_Update(&controller, &measurement, 1050U, 1.0f, 0.1f, 1);
    assert(updated);
    assert(controller.velocity_px_s == 0.0f);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);

    /*
     * 第三帧与第一帧相差20 px/100 ms，原始速度为+200 px/s；
     * alpha=0.20后得到+40 px/s，D项为-4 us。
     */
    measurement = make_measurement(3U, 0, 371);
    updated = BallController_Update(&controller, &measurement, 1100U, 1.0f, 0.1f, 1);
    assert(updated);
    assert((controller.velocity_px_s > 39.9f) &&
           (controller.velocity_px_s < 40.1f));
    assert(controller.target_pulse_us == (SERVO_PWM_NEUTRAL_US - 4U));

    /*
     * 静止圆心在相邻帧交替跳动±2 px时，100 ms窗口的起点和终点相同，
     * 不能再像相邻帧差分那样产生约±40 px/s的假速度。
     */
    BallController_Reset(&controller);
    measurement = make_measurement(10U, 1, 360);
    assert(BallController_AcceptMeasurement(
        &controller, &measurement, 1200U
    ));
    measurement = make_measurement(11U, -1, 362);
    assert(BallController_AcceptMeasurement(
        &controller, &measurement, 1250U
    ));
    measurement = make_measurement(12U, 1, 360);
    assert(BallController_AcceptMeasurement(
        &controller, &measurement, 1300U
    ));
    assert(controller.velocity_px_s == 0.0f);
    measurement = make_measurement(13U, -1, 362);
    assert(BallController_AcceptMeasurement(
        &controller, &measurement, 1350U
    ));
    assert(controller.velocity_px_s == 0.0f);

    /*
     * 模拟STM32的50 Hz控制取样：前5帧x=360，第6帧只量化跳动到361。
     * 真实窗口为100 ms。因为拟合使用窗口内全部6个点，而不是只看
     * 最后跳变的两个端点，alpha=0.20后应约为1.43 px/s；
     * 若使用相邻帧差分则会达到50 px/s。
     */
    BallController_Reset(&controller);
    for (sample_index = 0; sample_index < 5; sample_index++)
    {
        measurement = make_measurement(
            (uint16_t)(100U + sample_index),
            1,
            360
        );
        assert(BallController_AcceptMeasurement(
            &controller,
            &measurement,
            (uint32_t)(2000U + (uint32_t)sample_index * 20U)
        ));
    }
    measurement = make_measurement(105U, 0, 361);
    assert(BallController_AcceptMeasurement(
        &controller, &measurement, 2100U
    ));
    assert((controller.velocity_px_s > 1.3f) &&
           (controller.velocity_px_s < 1.5f));

    /* 方向取反后，完全相同的控制量应作用到中位另一侧。 */
    BallController_Reset(&controller);
    measurement = make_measurement(4U, 10, 351);
    updated = BallController_Update(&controller, &measurement, 2000U, 1.0f, 0.0f, -1);
    assert(updated);
    assert(controller.target_pulse_us == (SERVO_PWM_NEUTRAL_US - 10U));

    /* 中心死区内 P 项为零。 */
    BallController_Reset(&controller);
    measurement = make_measurement(5U, BALL_CONTROL_DEADBAND_PX, 357);
    updated = BallController_Update(&controller, &measurement, 3000U, 5.0f, 0.0f, 1);
    assert(updated);
    assert(controller.p_term_us == 0.0f);
    assert(controller.target_pulse_us == SERVO_PWM_NEUTRAL_US);

    /* 极大增益也必须被自动控制专用范围截住。 */
    BallController_Reset(&controller);
    measurement = make_measurement(6U, 300, 61);
    updated = BallController_Update(&controller, &measurement, 4000U, 10.0f, 0.0f, 1);
    assert(updated);
    assert(controller.saturated);
    assert(controller.target_pulse_us == BALL_CONTROL_PWM_MAX_US);

    measurement.ball_valid = false;
    assert(!BallController_Update(&controller, &measurement, 4050U, 1.0f, 0.0f, 1));

    /*
     * 新实时路径把“新测量估计”和“固定周期控制”分开：
     * 重复frame_id不能再次差分，但控制任务仍可使用最近状态再次执行。
     */
    BallController_Reset(&controller);
    measurement = make_measurement(7U, 20, 341);
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        5000U
    ));
    assert(BallController_Step(&controller, 1.0f, 0.0f, 1));
    assert(controller.target_pulse_us ==
           (SERVO_PWM_NEUTRAL_US + 20U));

    assert(!BallController_AcceptMeasurement(
        &controller,
        &measurement,
        5020U
    ));
    assert(BallController_Step(&controller, 1.0f, 0.0f, 1));
    assert(controller.target_pulse_us ==
           (SERVO_PWM_NEUTRAL_US + 20U));

    /* 异常dt丢弃当前速度更新并清除差分历史。 */
    measurement.frame_id = 8U;
    measurement.ball_x = 350;
    measurement_result = BallController_AcceptMeasurementEx(
        &controller,
        &measurement,
        5004U
    );
    assert(measurement_result == BALL_MEAS_INVALID_DT);
    assert(!controller.has_history);
    assert(controller.velocity_px_s == 0.0f);

    /* 异常后下一帧只重新建立基线，速度仍为0。 */
    measurement.frame_id = 9U;
    measurement_result = BallController_AcceptMeasurementEx(
        &controller,
        &measurement,
        5075U
    );
    assert(measurement_result == BALL_MEAS_ACCEPTED);
    assert(controller.has_history);
    assert(controller.velocity_px_s == 0.0f);

    /*
     * 位置I只在第二个及后续新帧上按真实dt累计。
     * 50 Hz控制任务对同一帧重复Step时，不得再次积分。
     */
    BallController_Reset(&controller);
    measurement = make_measurement(20U, 40, 321);
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        6000U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert(controller.i_term_us == 0.0f);

    measurement.frame_id = 21U;
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        6050U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert((controller.i_term_us > 0.19f) &&
           (controller.i_term_us < 0.21f));

    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert((controller.i_term_us > 0.19f) &&
           (controller.i_term_us < 0.21f));

    /* 积分项具有独立限幅，复位时必须清零。 */
    measurement.frame_id = 22U;
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        6100U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        100.0f,
        0.0f,
        1
    ));
    assert(controller.i_term_us == BALL_CONTROL_INTEGRAL_MAX_US);
    BallController_Reset(&controller);
    assert(controller.i_term_us == 0.0f);

    /*
     * PWM已饱和且积分继续推向同一方向时，必须撤销本次积分，
     * 不能让隐藏的积分能量继续累积。
     */
    measurement = make_measurement(30U, 40, 321);
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        7000U
    ));
    assert(BallController_StepPid(
        &controller,
        100.0f,
        1.0f,
        0.0f,
        1
    ));
    measurement.frame_id = 31U;
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        7050U
    ));
    assert(BallController_StepPid(
        &controller,
        100.0f,
        1.0f,
        0.0f,
        1
    ));
    assert(controller.saturated);
    assert(controller.i_term_us == 0.0f);

    /*
     * 单帧进入中心死区或跑出积分作用区只能暂停积分，不能把已经建立的
     * 静态补偿突然清零。
     */
    BallController_Reset(&controller);
    measurement = make_measurement(40U, 20, 341);
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        8000U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    measurement.frame_id = 41U;
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        8050U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert((controller.i_term_us > 0.09f) &&
           (controller.i_term_us < 0.11f));

    measurement = make_measurement(42U, 0, 361);
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        8100U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert((controller.i_term_us > 0.09f) &&
           (controller.i_term_us < 0.11f));

    measurement = make_measurement(
        43U,
        BALL_CONTROL_INTEGRAL_ZONE_PX + 1,
        280
    );
    assert(BallController_AcceptMeasurement(
        &controller,
        &measurement,
        8150U
    ));
    assert(BallController_StepPid(
        &controller,
        0.0f,
        0.1f,
        0.0f,
        1
    ));
    assert((controller.i_term_us > 0.09f) &&
           (controller.i_term_us < 0.11f));

    puts("ball_controller tests passed");
    return 0;
}
