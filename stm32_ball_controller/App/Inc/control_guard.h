#ifndef CONTROL_GUARD_H
#define CONTROL_GUARD_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"

typedef enum
{
    CONTROL_GUARD_LINK_TIMEOUT = 0,
    CONTROL_GUARD_VISION_INVALID,
    CONTROL_GUARD_BALL_UNSAFE,
    CONTROL_GUARD_READY
} ControlGuardState;

/*
 * 只做“是否允许进入控制计算”的判定，不计算 PID，也不输出 PWM。
 */
ControlGuardState ControlGuard_Evaluate(
    const VisionMeasurement *measurement,
    bool has_packet,
    uint32_t last_packet_tick,
    uint32_t now_tick
);

#endif

