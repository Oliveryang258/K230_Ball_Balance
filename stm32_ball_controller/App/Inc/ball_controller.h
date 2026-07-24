#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"

/*
 * 钢球位置 PD 控制器。
 *
 * 该模块只完成数学计算，不直接调用 HAL，也不直接操作 PWM。
 * 因此可以先在 PC 上做纯逻辑测试，再由 main.c 决定是否把计算结果送给舵机。
 *
 * 控制律：
 *   offset_us = direction * (Kp * error_px - Kv * velocity_px_s)
 *
 * Kp 单位：us / px
 * Kv 单位：us / (px/s)
 * direction 只能取 +1 或 -1，用于匹配实际舵机安装方向。
 */
typedef struct
{
    bool has_history;
    bool updated;
    bool saturated;

    uint16_t last_frame_id;
    int16_t last_ball_x;
    uint32_t last_tick_ms;

    int16_t error_px;
    float velocity_px_s;
    float p_term_us;
    float d_term_us;
    float control_offset_us;
    uint16_t target_pulse_us;
} BallController;

/* 初始化或彻底清除控制器历史，输出回到暂定中位。 */
void BallController_Init(BallController *controller);
void BallController_Reset(BallController *controller);

/*
 * 仅当 frame_id 与上一帧不同的时候更新一次。
 *
 * 返回 true：本次确实使用了一帧新数据；
 * 返回 false：参数无效、视觉无效，或仍是上一帧数据。
 */
bool BallController_Update(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t now_ms,
    float kp_us_per_px,
    float kv_us_per_px_s,
    int8_t direction
);

uint16_t BallController_GetTargetPulseUs(const BallController *controller);

#endif
