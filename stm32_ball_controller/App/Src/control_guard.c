#include "control_guard.h"
#include "app_config.h"

ControlGuardState ControlGuard_Evaluate(
    const VisionMeasurement *measurement,
    bool has_packet,
    uint32_t last_packet_tick,
    uint32_t now_tick)
{
    if ((!has_packet)
        || ((uint32_t)(now_tick - last_packet_tick) > VISION_LINK_TIMEOUT_MS))
    {
        return CONTROL_GUARD_LINK_TIMEOUT;
    }

    if ((measurement == 0) || (!measurement->ball_valid))
    {
        return CONTROL_GUARD_VISION_INVALID;
    }

    /*
     * K230 的 safe 标志是第一层保护；像素范围检查是 STM32 的第二层保护。
     * 即使通信帧中的标志位意外错误，明显越界的坐标也不会进入控制器。
     */
    if ((!measurement->ball_safe)
        || (measurement->ball_x < VISION_SAFE_X_MIN)
        || (measurement->ball_x > VISION_SAFE_X_MAX))
    {
        return CONTROL_GUARD_BALL_UNSAFE;
    }

    return CONTROL_GUARD_READY;
}

