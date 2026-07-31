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

    int16_t target_x;
    uint16_t equilibrium_pulse_us;
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
    BALL_MEAS_INVALID_DT,
    /* 位置相对上一有效帧跳变超限，丢弃本帧但不破坏差分历史。 */
    BALL_MEAS_REJECTED
} BallMeasurementResult;

/* 初始化或彻底清除控制器历史，输出回到暂定中位。 */
void BallController_Init(BallController *controller);
void BallController_Reset(BallController *controller);

/*
 * 设置任意位置目标和该位置对应的静态平衡PWM，并返回限幅后的真实值。
 * target_x限制在视觉闭环安全区；equilibrium_pulse_us限制在闭环软限幅。
 */
int16_t BallController_SetTargetX(
    BallController *controller,
    int16_t target_x
);

/*
 * 第三题参考轨迹专用接口：更新PID参考位置，但保留当前积分状态。
 * 只能用于已经经过限速的连续参考，不能拿它处理人工的大幅目标阶跃。
 */
int16_t BallController_TrackTargetX(
    BallController *controller,
    int16_t target_x
);

/*
 * 切换第三题步骤时调用一次：清除只属于旧目标的I和静摩擦慢状态，
 * 但保留视觉位置、速度和时间历史。
 */
void BallController_ResetTargetSlowState(BallController *controller);

/*
 * 短暂丢球恢复后调用：清除位置差分历史，避免基于长时间间隔的旧位置
 * 计算速度尖峰。不清除积分、目标和平衡PWM。
 */
void BallController_ClearVelocityHistory(BallController *controller);

uint16_t BallController_SetEquilibriumPulseUs(
    BallController *controller,
    uint16_t equilibrium_pulse_us
);

/*
 * 接收一帧新的视觉测量，并更新位置、速度估计。
 * 位置误差由本控制器使用target_x - ball_x本地计算；UART中的error_px
 * 仅保留协议兼容性，不再决定STM32控制目标。
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
 * - 误差位于积分区内时按完整Ki积分；
 * - 超出积分误差区时暂停积分并保留当前I；
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
