#include <assert.h>
#include <stdio.h>

#include "app_config.h"
#include "task3_sequence.h"

/*
 * 模拟钢球能够跟随经过限速的参考目标，然后在当前步骤最终目标处稳定。
 * 返回完成本步骤时的毫秒时刻。
 */
static uint32_t follow_ramp_and_finish_step(Task3Sequence *sequence,
                                            uint32_t now_ms)
{
    int16_t command_x;
    int16_t final_x;

    final_x = Task3Sequence_GetFinalTargetX(sequence);

    while (Task3Sequence_GetTargetX(sequence) != final_x)
    {
        command_x = Task3Sequence_GetTargetX(sequence);
        now_ms += 20U;
        Task3Sequence_Update(
            sequence,
            now_ms,
            true,
            command_x,
            0.0f
        );
    }

    /*
     * 参考目标到达终点后，让钢球在终点连续稳定满配置时间。
     */
    Task3Sequence_Update(sequence, now_ms, true, final_x, 0.0f);
    now_ms += TASK3_STABLE_HOLD_MS;
    Task3Sequence_Update(sequence, now_ms, true, final_x, 0.0f);
    return now_ms;
}

static void test_full_sequence_completes(void)
{
    Task3Sequence sequence;
    uint32_t now_ms = 100U;

    Task3Sequence_Init(&sequence);
    assert(Task3Sequence_GetState(&sequence) == TASK3_SEQUENCE_IDLE);
    assert(Task3Sequence_GetTargetX(&sequence) ==
           BALL_CONTROL_TRACK_CENTER_X_PX);

    Task3Sequence_Start(&sequence, now_ms);
    assert(Task3Sequence_GetState(&sequence) == TASK3_SEQUENCE_RUNNING);
    assert(Task3Sequence_GetStep(&sequence) == 0U);

    now_ms = follow_ramp_and_finish_step(&sequence, now_ms);
    assert(Task3Sequence_GetStep(&sequence) == 1U);
    assert(Task3Sequence_GetFinalTargetX(&sequence) ==
           TASK3_TARGET_POINT_1_X);
    /*
     * 步骤刚切换时，PID参考仍停在上一个点，不允许直接跳到新终点。
     */
    assert(Task3Sequence_GetTargetX(&sequence) ==
           TASK3_TARGET_POINT_0_X);

    now_ms = follow_ramp_and_finish_step(&sequence, now_ms);
    assert(Task3Sequence_GetStep(&sequence) == 2U);

    now_ms = follow_ramp_and_finish_step(&sequence, now_ms);
    assert(Task3Sequence_GetState(&sequence) ==
           TASK3_SEQUENCE_COMPLETE);
    assert(Task3Sequence_GetTargetX(&sequence) ==
           TASK3_TARGET_POINT_2_X);
    assert(Task3Sequence_GetFinalTargetX(&sequence) ==
           TASK3_TARGET_POINT_2_X);
    assert(now_ms < TASK3_TOTAL_TIMEOUT_MS);
}

static void test_plus_five_to_minus_five_is_rate_limited(void)
{
    Task3Sequence sequence;
    uint32_t now_ms = 0U;
    int16_t previous_command_x;
    int16_t new_command_x;

    Task3Sequence_Init(&sequence);
    Task3Sequence_Start(&sequence, now_ms);

    /* 中心稳定后进入第1步，再跟随到第1个5 cm目标。 */
    now_ms = follow_ramp_and_finish_step(&sequence, now_ms);
    now_ms = follow_ramp_and_finish_step(&sequence, now_ms);

    assert(Task3Sequence_GetStep(&sequence) == 2U);
    assert(Task3Sequence_GetFinalTargetX(&sequence) ==
           TASK3_TARGET_POINT_2_X);

    previous_command_x = Task3Sequence_GetTargetX(&sequence);
    now_ms += 20U;
    Task3Sequence_Update(
        &sequence,
        now_ms,
        true,
        previous_command_x,
        0.0f
    );
    new_command_x = Task3Sequence_GetTargetX(&sequence);

    assert(new_command_x != TASK3_TARGET_POINT_2_X);
    assert(
        (new_command_x - previous_command_x) ==
        -TASK3_TARGET_SLEW_PX_PER_CONTROL
    );
}

static void test_stability_must_be_continuous(void)
{
    Task3Sequence sequence;

    Task3Sequence_Init(&sequence);
    Task3Sequence_Start(&sequence, 0U);

    Task3Sequence_Update(
        &sequence,
        0U,
        true,
        TASK3_TARGET_POINT_0_X,
        0.0f
    );

    /* 中途速度超限，连续稳定计时必须清零。 */
    Task3Sequence_Update(
        &sequence,
        150U,
        true,
        TASK3_TARGET_POINT_0_X,
        TASK3_STABLE_SPEED_MAX_PX_S + 1.0f
    );

    Task3Sequence_Update(
        &sequence,
        200U,
        true,
        TASK3_TARGET_POINT_0_X,
        0.0f
    );
    Task3Sequence_Update(
        &sequence,
        399U,
        true,
        TASK3_TARGET_POINT_0_X,
        0.0f
    );
    assert(Task3Sequence_GetStep(&sequence) == 0U);

    Task3Sequence_Update(
        &sequence,
        400U,
        true,
        TASK3_TARGET_POINT_0_X,
        0.0f
    );
    assert(Task3Sequence_GetStep(&sequence) == 1U);
}

static void test_timeout_holds_current_target(void)
{
    Task3Sequence sequence;
    int16_t target_before_timeout;

    Task3Sequence_Init(&sequence);
    Task3Sequence_Start(&sequence, 1000U);
    Task3Sequence_Update(
        &sequence,
        1020U,
        true,
        TASK3_TARGET_POINT_0_X,
        0.0f
    );
    target_before_timeout = Task3Sequence_GetTargetX(&sequence);

    Task3Sequence_Update(
        &sequence,
        1000U + TASK3_TOTAL_TIMEOUT_MS,
        false,
        0,
        0.0f
    );

    assert(Task3Sequence_GetState(&sequence) ==
           TASK3_SEQUENCE_TIMEOUT);
    assert(Task3Sequence_GetTargetX(&sequence) ==
           target_before_timeout);
    assert(Task3Sequence_GetElapsedMs(&sequence) ==
           TASK3_TOTAL_TIMEOUT_MS);
}

int main(void)
{
    test_full_sequence_completes();
    test_plus_five_to_minus_five_is_rate_limited();
    test_stability_must_be_continuous();
    test_timeout_holds_current_target();

    printf("task3_sequence tests passed\n");
    return 0;
}
