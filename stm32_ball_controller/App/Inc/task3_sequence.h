#ifndef TASK3_SEQUENCE_H
#define TASK3_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 第三题自动目标切换状态。
 *
 * 数值刻意固定，便于在 Keil Watch 中直接观察：
 *   0 = IDLE     尚未启动
 *   1 = RUNNING  正在执行
 *   2 = COMPLETE 全部目标均已稳定到达
 *   3 = TIMEOUT  4.8 s 内未完成
 */
typedef enum
{
    TASK3_SEQUENCE_IDLE = 0,
    TASK3_SEQUENCE_RUNNING = 1,
    TASK3_SEQUENCE_COMPLETE = 2,
    TASK3_SEQUENCE_TIMEOUT = 3
} Task3SequenceState;

typedef struct
{
    Task3SequenceState state;
    uint8_t step;
    /* 当前步骤最终必须到达并稳定的目标。 */
    int16_t target_x;
    /* 经过限速后，当前实际送给位置PID的参考目标。 */
    int16_t command_target_x;

    uint32_t start_ms;
    uint32_t elapsed_ms;

    uint32_t stable_since_ms;
    bool stable_timing;
} Task3Sequence;

void Task3Sequence_Init(Task3Sequence *sequence);
void Task3Sequence_Start(Task3Sequence *sequence, uint32_t now_ms);

/*
 * 每个 20 ms 控制周期调用一次。
 *
 * measurement_valid:
 *   只有视觉数据有效、UART 未超时、位置未越界时才传 true。
 * ball_x:
 *   当前钢球的图像横坐标。
 * velocity_px_s:
 *   STM32 根据真实视觉帧时间差估计出的钢球速度。
 */
void Task3Sequence_Update(Task3Sequence *sequence,
                          uint32_t now_ms,
                          bool measurement_valid,
                          int16_t ball_x,
                          float velocity_px_s);

Task3SequenceState Task3Sequence_GetState(const Task3Sequence *sequence);
uint8_t Task3Sequence_GetStep(const Task3Sequence *sequence);
int16_t Task3Sequence_GetTargetX(const Task3Sequence *sequence);
int16_t Task3Sequence_GetFinalTargetX(const Task3Sequence *sequence);
uint32_t Task3Sequence_GetElapsedMs(const Task3Sequence *sequence);

#endif
