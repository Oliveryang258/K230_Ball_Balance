#ifndef CONTROL_GUARD_H
#define CONTROL_GUARD_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"

/*
 * 数值会直接显示在g_dbg.guard中，调试时按此表解释。
 * 保护判断由20 ms控制任务执行，因此最迟在下一个控制周期处理，
 * 不能描述成硬件级“零延迟立即”保护。
 */
typedef enum
{
    CONTROL_GUARD_UART_TIMEOUT = 0,
    CONTROL_GUARD_BALL_LOST = 1,
    CONTROL_GUARD_POSITION_OUT_OF_RANGE = 2,
    CONTROL_GUARD_INVALID_DT = 3,
    CONTROL_GUARD_PROTOCOL_ERROR = 4,
    CONTROL_GUARD_READY = 5
} ControlGuardState;

ControlGuardState ControlGuard_Evaluate(
    const VisionMeasurement *measurement,
    bool has_packet,
    uint32_t last_packet_tick,
    uint32_t now_tick,
    bool protocol_error_event,
    bool invalid_dt_event
);

#endif
