#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * K230 正常约 20 Hz 输出一帧视觉结果。
 * 150 ms 相当于连续丢失约 3 帧后进入通信超时。
 * 该值属于第一版保守参数，后续根据实测丢包情况调整。
 */
#define VISION_LINK_TIMEOUT_MS  150U

/* 当前视觉程序的安全像素范围，仅用于接收端二次检查。 */
#define VISION_SAFE_X_MIN       60
#define VISION_SAFE_X_MAX       598

/*
 * DS215MG V8.0 第一轮台架测试参数。
 *
 * 舵机规格允许的范围比这里更大，但机械连杆尚未完成安全标定。
 * 当前根据空载实测，把 1420 us 暂定为中位，只开放 1340~1500 us。
 * 该范围仍需在取下钢球后完成带连杆验证，禁止直接使用舵机规格极限。
 */
#define SERVO_PWM_NEUTRAL_US       1420U
#define SERVO_PWM_TEST_MIN_US      1340U
#define SERVO_PWM_TEST_MAX_US      1500U

/*
 * 舵机目标值每 20 ms 更新一次，每次最多变化 5 us。
 * 这与硬件持续输出的 333 Hz PWM 是两个不同概念：
 * TIM2 自动输出 333 Hz，而软件只缓慢改变比较值。
 */
#define SERVO_COMMAND_UPDATE_MS    20U
#define SERVO_MAX_STEP_US          5U

/*
 * 闭环控制阶段使用的脉宽范围要比台架测试范围更保守。
 *
 * 1340~1500 us 是“允许手动试验”的外层硬限幅；根据带负载测试，
 * 1360 us 附近曾出现持续嗡鸣。当前实测 1420 us 约为水平，
 * 1400 us 约为 +4 度、1440 us 约为 -4.8 度，因此首次自动闭环只开放
 * 1400~1440 us。先用小倾角跑通控制流程，再根据实验结果逐步扩展。
 * 后续必须根据新轨道的机械标定结果重新确认，不能把这里当成舵机规格极限。
 */
#define BALL_CONTROL_PWM_MIN_US          1400U
#define BALL_CONTROL_PWM_MAX_US          1440U

/*
 * 球位置 PD 的第一版参数。
 *
 * - 死区避免钢球已经靠近中心时，检测噪声仍让舵机来回动作；
 * - 速度低通系数越大，越相信当前一帧计算出的速度；
 * - 超过最大采样间隔后，不再用旧位置计算速度，避免产生速度尖峰。
 */
#define BALL_CONTROL_DEADBAND_PX         4
#define BALL_CONTROL_VELOCITY_ALPHA      0.50f
#define BALL_CONTROL_MAX_SAMPLE_MS       250U

#endif
