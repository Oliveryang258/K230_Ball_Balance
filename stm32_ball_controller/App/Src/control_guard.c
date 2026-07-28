#include "control_guard.h"
#include "app_config.h"

ControlGuardState ControlGuard_Evaluate(
    const VisionMeasurement *measurement,
    bool has_packet,
    uint32_t last_packet_tick,
    uint32_t now_tick,
    bool protocol_error_event,
    bool invalid_dt_event)
{
    if ((!has_packet) ||
        ((uint32_t)(now_tick - last_packet_tick) > VISION_LINK_TIMEOUT_MS))
    {
        return CONTROL_GUARD_UART_TIMEOUT;
    }

    if (protocol_error_event)
    {
        return CONTROL_GUARD_PROTOCOL_ERROR;
    }

    if ((measurement == 0) || (!measurement->ball_valid))
    {
        return CONTROL_GUARD_BALL_LOST;
    }

    /*
     * K230的safe标志是第一层保护，STM32像素范围检查是第二层。
     * 任意一层报告越界都不允许闭环输出。
     */
    if ((!measurement->ball_safe) ||
        (measurement->ball_x < VISION_SAFE_X_MIN) ||
        (measurement->ball_x > VISION_SAFE_X_MAX))
    {
        return CONTROL_GUARD_POSITION_OUT_OF_RANGE;
    }

    if (invalid_dt_event)
    {
        return CONTROL_GUARD_INVALID_DT;
    }

    return CONTROL_GUARD_READY;
}
