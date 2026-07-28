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
 * DS215MG V8.0 当前重新校准参数。
 *
 * 1700 us、1300~2100 us是本轮人工修改的待测值，不是已经验证的机械安全值。
 * 重新校准必须从1570 us附近逐步向两侧扩展；出现持续嗡鸣、电流上升、
 * 发热或机构顶死时立即断电。完成带连杆标定后应重新收紧上下限。
 */
#define SERVO_PWM_NEUTRAL_US       1590U
#define SERVO_PWM_TEST_MIN_US      1190U
#define SERVO_PWM_TEST_MAX_US      1990U

/*
 * 注意：970～2170 us只用于人工标定，不是默认闭环安全范围。
 * 第一阶段闭环计算值限制为中位1570 us附近±40 us，即1530～1610 us。
 */

/*
 * 舵机目标值每 20 ms 更新一次，每次最多变化 5 us。
 * 这与硬件持续输出的 333 Hz PWM 是两个不同概念：
 * TIM2 自动输出 333 Hz，而软件只缓慢改变比较值。
 */
#define SERVO_COMMAND_UPDATE_MS    20U
#define SERVO_MAX_STEP_US          5U

/*
 * 自动闭环专用软限幅。
 *
 * 初始只开放中位附近±40 us。后续完成PWM—轨道角标定后，只能根据实机
 * 安全结果逐步扩大；不得直接使用人工标定的970～2170 us范围。
 */
#define BALL_CONTROL_PWM_SOFT_RANGE_US    400U
#define BALL_CONTROL_PWM_MIN_US          \
    (SERVO_PWM_NEUTRAL_US - BALL_CONTROL_PWM_SOFT_RANGE_US)
#define BALL_CONTROL_PWM_MAX_US          \
    (SERVO_PWM_NEUTRAL_US + BALL_CONTROL_PWM_SOFT_RANGE_US)

/*
 * 球位置 PD 的第一版参数。
 *
 * - 死区避免钢球已经靠近中心时，检测噪声仍让舵机来回动作；
 * - 速度不再按固定帧数或窗口首尾直接差分，而是用约100 ms窗口内的
 *   全部位置和真实时间戳拟合直线，以其斜率作为速度；
 * - 90 FPS时1 px相邻帧跳动会被放大成约90 px/s，而100 ms窗口会把
 *   同样的1 px量化变化限制到约10 px/s；
 * - 速度低通系数越大，越相信当前一次窗口差分结果；
 * - 超过最大采样间隔后，不再用旧位置计算速度，避免产生速度尖峰。
 */
#define BALL_CONTROL_DEADBAND_PX         4
#define BALL_CONTROL_VELOCITY_ALPHA      0.20f
#define BALL_CONTROL_VELOCITY_WINDOW_MS  100U
#define BALL_CONTROL_MIN_SAMPLE_MS       5U
#define BALL_CONTROL_MAX_SAMPLE_MS       250U

/*
 * 位置积分只在中心附近启用，用来缓慢克服零位偏差和静摩擦残余误差。
 * 第一版把积分项本身限制在±12 us；这不是总PWM限幅。
 */
#define BALL_CONTROL_INTEGRAL_ZONE_PX    80
#define BALL_CONTROL_INTEGRAL_MAX_US     80.0f

/*
 * STM32固定周期控制任务。
 * SysTick每1 ms进入一次中断，累计20次后执行一次状态更新、保护判断和PD。
 * 该周期与K230约20 Hz的视觉更新、TIM2的333 Hz硬件PWM是三个独立时基。
 */
#define BALL_CONTROL_PERIOD_MS            20U

/*
 * 上电自动闭环默认值。
 *
 * 当前重新校准期间默认只计算、不输出：
 * - 收到有效且安全的新视觉帧时，固定20 ms计算一次；
 * - UART超时、视觉无效或钢球越界时，ControlGuard仍会强制回到中位；
 * - 机械安全范围确认后，才把APPLY_OUTPUT改为1。
 *
 * 如果实机方向相反，只把DIRECTION从1改为-1，其他逻辑不需要改。
 */
#define BALL_CONTROL_DEFAULT_APPLY_OUTPUT  1U
#define BALL_CONTROL_DEFAULT_KP_US_PER_PX  1.2f
#define BALL_CONTROL_DEFAULT_KI_US_PER_PX_S 0.00f
#define BALL_CONTROL_DEFAULT_KV_US_PER_PX_S 0.5f
#define BALL_CONTROL_DEFAULT_DIRECTION     (-1)

#endif
