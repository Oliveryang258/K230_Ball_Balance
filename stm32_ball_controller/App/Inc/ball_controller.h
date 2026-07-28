#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_protocol.h"

/*
 * K230约90 FPS发送，但当前STM32控制任务每20 ms取一次最新完整帧，
 * 因而100 ms窗口通常包含约5~6个有效样本。保留16帧可留出充分余量。
 * 每个历史样本只占一个int16位置和一个uint32时间戳，总内存约96字节。
 */
#define BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY 16U

/*
 * 钢球位置 PID 控制器（当前唯一的闭环）。
 *
 * 该模块只完成数学计算，不直接调用 HAL，也不直接操作 PWM。
 * 因此可以先在 PC 上做纯逻辑测试，再由 main.c 决定是否把计算结果送给舵机。
 *
 * 当前没有轨道角度传感器，所以不存在可闭环的“角度内环”：
 * - 本模块负责根据球位置误差和球速度计算轨道动作命令；
 * - ServoOutput负责把脉宽命令限幅、缓变并写入TIM2；
 * - 未来完成轨道角度—PWM标定后，可在两者之间加入静态映射层，
 *   但静态映射层本身仍不是角度闭环。
 *
 * 控制律：
 *   offset_us = direction * (P + I + D)
 *
 * Kp 单位：us / px
 * Ki 单位：us / (px*s)
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
    uint32_t last_dt_ms;

    /* 位置/时间环形历史，用于约100 ms窗口的最小二乘速度拟合。 */
    int16_t velocity_x_history[
        BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY
    ];
    uint32_t velocity_tick_history[
        BALL_CONTROLLER_VELOCITY_HISTORY_CAPACITY
    ];
    uint8_t velocity_history_head;
    uint8_t velocity_history_count;

    int16_t error_px;
    float velocity_px_s;
    float p_term_us;
    float i_term_us;
    float d_term_us;
    float control_offset_us;
    uint16_t target_pulse_us;
} BallController;

typedef enum
{
    BALL_MEAS_ACCEPTED = 0,
    BALL_MEAS_DUPLICATE,
    BALL_MEAS_INVALID,
    BALL_MEAS_INVALID_DT
} BallMeasurementResult;

/* 初始化或彻底清除控制器历史，输出回到暂定中位。 */
void BallController_Init(BallController *controller);
void BallController_Reset(BallController *controller);

/*
 * 接收一帧新的视觉测量，并更新位置、速度估计。
 *
 * 仅当frame_id变化时才做位置差分；重复帧不会重复计算速度。
 * 本函数不计算新的PWM目标，供视觉测量到达时调用。
 */
bool BallController_AcceptMeasurement(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t measurement_tick_ms
);

/*
 * 带原因返回值的新接口。measurement_tick_ms必须是完整有效UART帧在
 * USART接收中断中完成解析时记录的HAL_GetTick()，不能使用固定50 ms。
 */
BallMeasurementResult BallController_AcceptMeasurementEx(
    BallController *controller,
    const VisionMeasurement *measurement,
    uint32_t measurement_tick_ms
);

/*
 * 使用最近一次有效状态执行一次固定周期PD计算。
 *
 * 本函数应由固定周期控制任务调用；当前工程在SysTick中每20 ms调用。
 * 即使20 ms内没有新视觉帧，也会按照最近一次有效状态执行控制。
 */
bool BallController_Step(
    BallController *controller,
    float kp_us_per_px,
    float kv_us_per_px_s,
    int8_t direction
);

/*
 * 位置PID接口。积分只在收到新视觉帧后，按该帧真实dt更新一次：
 * - 重复frame_id或控制任务空跑时不重复积分；
 * - 超出积分作用区或进入中心死区时暂停并保留I；
 * - Ki设为0或guard复位时清零；
 * - 积分项具有独立限幅，PWM饱和且积分继续推向饱和时执行抗饱和回退。
 */
bool BallController_StepPid(
    BallController *controller,
    float kp_us_per_px,
    float ki_us_per_px_s,
    float kv_us_per_px_s,
    int8_t direction
);

/*
 * 兼容原有PC测试和旧调用方式：先接收新测量，再立即执行一次PD。
 * 新的STM32实时路径不再使用这个组合接口。
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
