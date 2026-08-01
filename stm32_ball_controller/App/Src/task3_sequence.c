#include "task3_sequence.h"

#include "app_config.h"

#define TASK3_POINT_COUNT 3U

/*
 * 第三题目标序列集中放在这里。
 * 每个目标点的具体坐标仍由 app_config.h 统一配置。
 */
static const int16_t s_task3_targets[TASK3_POINT_COUNT] =
{
    TASK3_TARGET_POINT_0_X,
    TASK3_TARGET_POINT_1_X,
    TASK3_TARGET_POINT_2_X
};

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*
 * 把“最终目标”变成位置PID能够平稳跟随的参考轨迹。
 * 返回true表示本周期结束后参考目标已经到达最终目标。
 */
static bool SlewCommandTarget(Task3Sequence *sequence)
{
    int32_t difference;
    int32_t step;

    difference =
        (int32_t)sequence->target_x -
        (int32_t)sequence->command_target_x;
    step = (int32_t)TASK3_TARGET_SLEW_PX_PER_CONTROL;

    if (step <= 0)
    {
        sequence->command_target_x = sequence->target_x;
        return true;
    }

    if (difference > step)
    {
        sequence->command_target_x =
            (int16_t)((int32_t)sequence->command_target_x + step);
        return false;
    }

    if (difference < -step)
    {
        sequence->command_target_x =
            (int16_t)((int32_t)sequence->command_target_x - step);
        return false;
    }

    sequence->command_target_x = sequence->target_x;
    return true;
}

void Task3Sequence_Init(Task3Sequence *sequence)
{
    if (sequence == 0)
    {
        return;
    }

    sequence->state = TASK3_SEQUENCE_IDLE;
    sequence->step = 0U;
    sequence->target_x = TASK3_TARGET_POINT_0_X;
    sequence->command_target_x = TASK3_TARGET_POINT_0_X;
    sequence->start_ms = 0U;
    sequence->elapsed_ms = 0U;
    sequence->stable_since_ms = 0U;
    sequence->stable_timing = false;
}

void Task3Sequence_Start(Task3Sequence *sequence, uint32_t now_ms)
{
    if (sequence == 0)
    {
        return;
    }

    /*
     * 每次启动都重新开始，因此同一套演示可以反复运行。
     * 起始点（第0步，中心）由人工放置钢球，不再参与到位判定：
     * 直接跳过中心点的稳定等待，从中心向第一个5cm目标移动，节省演示时间。
     * command_target 仍从中心开始，由斜坡限速平滑移动到第一个目标；
     * 到位检测只在5cm目标点上执行。
     */
    sequence->state = TASK3_SEQUENCE_RUNNING;
    sequence->step = 1U;
    sequence->target_x = s_task3_targets[1];
    sequence->command_target_x = s_task3_targets[0];
    sequence->start_ms = now_ms;
    sequence->elapsed_ms = 0U;
    sequence->stable_since_ms = 0U;
    sequence->stable_timing = false;
}

void Task3Sequence_Update(Task3Sequence *sequence,
                          uint32_t now_ms,
                          bool measurement_valid,
                          int16_t ball_x,
                          float velocity_px_s)
{
    int32_t error_px;
    bool position_ok;
    bool speed_ok;

    if ((sequence == 0) || (sequence->state != TASK3_SEQUENCE_RUNNING))
    {
        return;
    }

    /*
     * 无符号减法天然兼容毫秒计数器回绕，只要单次任务时长远小于
     * 2^31 ms（本任务只有 4.8 s）即可。
     */
    sequence->elapsed_ms = now_ms - sequence->start_ms;

    if (sequence->elapsed_ms >= TASK3_TOTAL_TIMEOUT_MS)
    {
        sequence->state = TASK3_SEQUENCE_TIMEOUT;
        sequence->stable_timing = false;
        return;
    }

    /*
     * UART 超时、球丢失或坐标越界时，不能把这段时间算作“稳定”。
     * 恢复有效测量后必须重新连续稳定满 TASK3_STABLE_HOLD_MS。
     */
    if (!measurement_valid)
    {
        sequence->stable_timing = false;
        return;
    }

    /*
     * 只有视觉和通信有效时才推进参考轨迹。否则控制器本身会进入保护，
     * 如果状态机仍在后台推进，视觉恢复时反而会再次出现大目标阶跃。
     */
    if (!SlewCommandTarget(sequence))
    {
        sequence->stable_timing = false;
        return;
    }

    error_px = (int32_t)sequence->target_x - (int32_t)ball_x;
    position_ok = (AbsInt32(error_px) <= TASK3_STABLE_ERROR_MAX_PX);
    speed_ok = (AbsFloat(velocity_px_s) <= TASK3_STABLE_SPEED_MAX_PX_S);

    if (!(position_ok && speed_ok))
    {
        sequence->stable_timing = false;
        return;
    }

    if (!sequence->stable_timing)
    {
        sequence->stable_since_ms = now_ms;
        sequence->stable_timing = true;
        return;
    }

    if ((uint32_t)(now_ms - sequence->stable_since_ms) < TASK3_STABLE_HOLD_MS)
    {
        return;
    }

    /*
     * 当前点连续稳定时间已满足：切换到下一目标。
     * 切换后重新计时，防止把上一个目标的稳定时间带到下一个目标。
     */
    sequence->stable_timing = false;

    if ((uint8_t)(sequence->step + 1U) < TASK3_POINT_COUNT)
    {
        sequence->step++;
        sequence->target_x = s_task3_targets[sequence->step];
    }
    else
    {
        /*
         * 最后一个目标就是题目要求的-5 cm。完成后保持当前最终目标，
         * 不能像旧版本一样自动把参考位置改回中心。
         */
        sequence->state = TASK3_SEQUENCE_COMPLETE;
    }
}

Task3SequenceState Task3Sequence_GetState(const Task3Sequence *sequence)
{
    return (sequence == 0) ? TASK3_SEQUENCE_IDLE : sequence->state;
}

uint8_t Task3Sequence_GetStep(const Task3Sequence *sequence)
{
    return (sequence == 0) ? 0U : sequence->step;
}

int16_t Task3Sequence_GetTargetX(const Task3Sequence *sequence)
{
    return (sequence == 0) ?
        BALL_CONTROL_TRACK_CENTER_X_PX :
        sequence->command_target_x;
}

int16_t Task3Sequence_GetFinalTargetX(const Task3Sequence *sequence)
{
    return (sequence == 0) ?
        BALL_CONTROL_TRACK_CENTER_X_PX :
        sequence->target_x;
}

uint32_t Task3Sequence_GetElapsedMs(const Task3Sequence *sequence)
{
    return (sequence == 0) ? 0U : sequence->elapsed_ms;
}
