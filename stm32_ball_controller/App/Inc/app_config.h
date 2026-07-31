#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * K230 正常约 20 Hz 输出一帧视觉结果。
 * 150 ms 相当于连续丢失约 3 帧后进入通信超时。
 * 该值属于第一版保守参数，后续根据实测丢包情况调整。
 */
#define VISION_LINK_TIMEOUT_MS  150U

/*
 * 短暂视觉丢球容错。
 *
 * K230偶尔会单帧丢失钢球（20 ms），现有逻辑会立即清零积分和速度历史，
 * 造成舵机目标跳变。增加宽限期后，连续丢球未超过GRACE_MS时保持最后一次
 * 有效舵机目标、冻结积分、不更新位置/速度估计。
 *
 * 超过宽限期后才执行原有严格保护（Reset + SetNeutral）。UART超时和
 * 位置越界始终使用原严格保护，不经过宽限期。
 */
#define BALL_LOST_GRACE_MS            100U

/*
 * STM32 -> K230 black-box telemetry remains disabled until the independent
 * bidirectional UART and K230 SD-card write test has passed on the hardware.
 */
#define APP_TELEMETRY_ENABLED    1U

/*
 * CH32V307 车辆控制器 -> STM32F103 运动遥测：
 * - USART2，PA3 接收；
 * - 115200, 8N1；
 * - 正常 50 Hz，一帧 28 字节；
 * - 超过 100 ms 没有收到通过 CRC 的完整帧，就只在遥测中标记链路无效。
 *
 * 第一阶段这些数据只送往 VOFA+ 观察，不直接修改舵机 PWM。
 */
#define APP_MOTION_RX_ENABLED        1U
#define MOTION_LINK_TIMEOUT_MS       100U

/*
 * STM32F103 -> HC-06 -> PC/VOFA+：
 * - USART3，PB10 发送；
 * - 115200, 8N1；
 * - JustFloat 固定 24 通道；
 * - 跟随 50 Hz 控制周期发布，真正发送在 main while(1) 中完成。
 */
#define APP_JUSTFLOAT_ENABLED        0U

/*
 * 当前视觉程序实测可见的钢球圆心范围，仅用于接收端二次检查。
 * 必须与K230 main.py中的TUNE_BALL_SAFE_LEFT/RIGHT_X保持一致。
 */
#define VISION_SAFE_X_MIN       22
#define VISION_SAFE_X_MAX       622

/*
 * DS215MG V8.0 当前重新校准参数。
 *
 * 1510 us、1110~1910 us是本轮人工修改的待测值，不是已经验证的机械安全值。
 * 重新校准必须从1510 us附近逐步向两侧扩展；出现持续嗡鸣、电流上升、
 * 发热或机构顶死时立即断电。完成带连杆标定后应重新收紧上下限。
 */
#define SERVO_PWM_NEUTRAL_US       1410U
#define SERVO_PWM_TEST_MIN_US      810U
#define SERVO_PWM_TEST_MAX_US      2010U

/*
 * 注意：1110～1910 us仍然需要装车后重新确认机械安全范围。
 */

/*
 * 舵机目标值每 20 ms 更新一次，每次最多变化 20 us。
 * 这与硬件持续输出的 333 Hz PWM 是两个不同概念：
 * TIM2 自动输出 333 Hz，而软件只缓慢改变比较值。
 */
#define SERVO_COMMAND_UPDATE_MS    20U
#define SERVO_MAX_STEP_US          50U

/*
 * 自动闭环专用软限幅。
 *
 * 当前实验软限幅为中位附近±400 us，即1110～1910 us。装车后仍需根据
 * 连杆、舵机发热和轨道角度的实测结果重新收紧，不得视为永久安全值。
 */
#define BALL_CONTROL_PWM_SOFT_RANGE_US    600U
#define BALL_CONTROL_PWM_MIN_US          \
    (SERVO_PWM_NEUTRAL_US - BALL_CONTROL_PWM_SOFT_RANGE_US)
#define BALL_CONTROL_PWM_MAX_US          \
    (SERVO_PWM_NEUTRAL_US + BALL_CONTROL_PWM_SOFT_RANGE_US)

/*
 * 静止任意位置保持的现场调试默认值。
 *
 * TARGET_X由STM32使用UART中的ball_x本地计算误差，不再依赖K230固定中心
 * 生成的error_px。HOLD_PWM_US是目标位置处的静态平衡脉宽；第一轮仍以
 * 当前水平中位开始，逐个目标位置实测后再生成位置—保持PWM插值表。
 */
/*
 * 当前相机标定：
 * - 可见控制范围x=22～622，对应-11.6～+11.8 cm；
 * - 物理中心为x=314；
 * - 画面左侧是舵机驱动端，画面右侧是固定端。
 *
 * 运行时可在Keil Watch直接修改g_target_x；下列宏提供第三问常用预设。
 * 这些像素值来自当前支架/相机位置，移动摄像头后必须重新标定。
 */
#define BALL_CONTROL_TRACK_CENTER_X_PX          314
#define BALL_CONTROL_TRACK_SERVO_END_X_PX        22
#define BALL_CONTROL_TRACK_FIXED_END_X_PX       622

/*
 * 2026-07-30实车重新放置钢球标定：
 * - -5 cm位于画面左侧（舵机端），实测x=190；
 * - +5 cm位于画面右侧（固定端），实测x=440。
 *
 * 第三题优先使用实测坐标，不再用端点比例推算值188/445。
 * 1 cm判定使用25 px的保守整数值，避免把略大于1 cm误判为合格。
 */
#define BALL_CONTROL_TARGET_NEGATIVE_5CM_X      190
#define BALL_CONTROL_TARGET_POSITIVE_5CM_X      445
#define BALL_CONTROL_ERROR_1CM_PX                25

/*
 * 第三题自动目标序列
 *
 * 默认流程：
 *   中心 -> +5 cm（固定端） -> -5 cm（舵机端）
 * 最后保持在-5 cm，不再返回中心；后续目标由人工修改。
 *
 * 这里使用已经标定好的“图像横坐标”作为目标，不把相机画面中心误当成
 * 轨道物理中心。以后题目要求改变顺序时，只需要修改下面四个目标点。
 */
#define TASK3_SEQUENCE_ENABLED                    1U
#define TASK3_TARGET_POINT_0_X                  BALL_CONTROL_TRACK_CENTER_X_PX
#define TASK3_TARGET_POINT_1_X                  BALL_CONTROL_TARGET_POSITIVE_5CM_X
#define TASK3_TARGET_POINT_2_X                  BALL_CONTROL_TARGET_NEGATIVE_5CM_X

/*
 * 第三题专用位置PID默认值。
 *
 * 运行时在Keil Watch修改g_task3_kp、g_task3_ki、g_task3_kv即可单独调题；
 * RUNNING和COMPLETE期间使用本组参数，普通闭环继续使用g_kp/g_ki/g_kv。
 * 初值先与普通闭环一致，保证新增接口不会改变当前已验证行为。
 */
#define TASK3_DEFAULT_KP_US_PER_PX              BALL_CONTROL_DEFAULT_KP_US_PER_PX
#define TASK3_DEFAULT_KI_US_PER_PX_S            BALL_CONTROL_DEFAULT_KI_US_PER_PX_S
#define TASK3_DEFAULT_KV_US_PER_PX_S            BALL_CONTROL_DEFAULT_KV_US_PER_PX_S

/* 到位判据：位置误差不超过约 1 cm，且小球速度足够低。 */
#define TASK3_STABLE_ERROR_MAX_PX               BALL_CONTROL_ERROR_1CM_PX
#define TASK3_STABLE_SPEED_MAX_PX_S             40.0f

/* 连续满足到位判据 200 ms 后，才切换到下一个目标点。 */
#define TASK3_STABLE_HOLD_MS                    200U

/*
 * 第三题专用目标轨迹限速：每个20 ms控制周期，送给位置PID的目标最多
 * 移动5 px。50 Hz下相当于约250 px/s，因此+5 cm到-5 cm的250 px变化
 * 会被展开为约1.00 s，而不是在一个周期内完成。
 *
 * 该参数只作用于第三题状态机；Watch手动修改g_target_x以及普通闭环不受影响。
 */
#define TASK3_TARGET_SLEW_PX_PER_CONTROL        5

/*
 * 总演示时限略小于 5 s，给显示、按键和调度保留余量。
 * 超时不是“成功”：状态机会进入TIMEOUT并保持当时的目标，不自动回中心；
 * 后续目标由Watch人工修改g_target_x。
 */
#define TASK3_TOTAL_TIMEOUT_MS                  4800U

#define BALL_CONTROL_DEFAULT_TARGET_X            \
    BALL_CONTROL_TRACK_CENTER_X_PX
#define BALL_CONTROL_DEFAULT_HOLD_PWM_US     SERVO_PWM_NEUTRAL_US

/*
 * 车辆直线运动题专用的固定PWM前馈。
 *
 * 当前实测钢球在固定行驶方向上稳定偏到error约-12 px。按照
 * direction=-1、Kp=2.7 us/px估算，原P项提供的纠偏约为+32 us，
 * 因此先用+32 us作为巡航前馈初值。
 *
 * 该补偿不改变物理目标坐标，也不参与第三题。运行时通过
 * g_cruise_ff_enabled开启；第三题状态机一旦启动，main.c会自动旁路它。
 */
#define BALL_CONTROL_DEFAULT_CRUISE_FF_ENABLED   0U
#define BALL_CONTROL_DEFAULT_CRUISE_FF_US        0

/*
 * 轨道方向加速度前馈。
 *
 * CH32V307已经对acc_track_mg完成零偏、低通和死区处理，STM32不再增加
 * 慢速滤波，避免前馈比钢球偏移还晚。前馈直接叠加到平衡PWM基准：
 *
 *     af_us = KAF * acc_track_mg
 *
 * Keil Watch调参：
 *   g_kaf    增益，单位us/mg；改成0即可关闭前馈
 *   g_af_lim 软限幅，单位us
 *   g_af     本周期实际采用的前馈，单位us，只读
 */
#define BALL_CONTROL_DEFAULT_KAF_US_PER_MG       2.5f
#define BALL_CONTROL_DEFAULT_AF_LIMIT_US         70.0f
#define BALL_CONTROL_AF_HARD_LIMIT_US            80.0f

/*
 * 加速度前馈的第二级EMA低通（CH32已做第一级α=0.25）。
 * alpha_acc控制新旧权重，越大越相信新值、延迟越小但越不平滑。
 * 初值0.30等效延迟约2～3拍（40～60 ms），前馈可以接受。
 */
#define BALL_CONTROL_ACC_EMA_ALPHA               0.30f

/*
 * 前馈分量独立变化率限制，避免高频抖动直接进入PWM。
 * 总舵机slew仍为50 us/20ms（SERVO_MAX_STEP_US），这两个是更紧的分量约束。
 */
#define BALL_CONTROL_AF_MAX_STEP_US              20.0f
#define BALL_CONTROL_YF_MAX_STEP_US              5.0f

/*
 * 顺时针赛道的右转偏航角速度前馈：
 *
 *   yf_us = KY * yaw_rate_dps * clamp(wheel_speed / VREF, 0, 1)
 *
 * 只在TURN_RIGHT且运动链路、IMU均有效时生效。车速比例避免减速弯仍使用
 * 恒速弯的完整补偿。实测右转yaw约-26.7 deg/s，需要约+8 us，因此KY
 * 初值取-0.30 us/(deg/s)。把g_ky改成0即可单独关闭该前馈。
 */
#define BALL_CONTROL_DEFAULT_KY_US_PER_DPS       (-0.45f)
#define BALL_CONTROL_DEFAULT_YF_LIMIT_US         25.0f
#define BALL_CONTROL_DEFAULT_VREF                200.0f
#define BALL_CONTROL_YF_HARD_LIMIT_US            40.0f

/*
 * 右转预瞄（turn preview）——填补 turn_command 出现到 yaw_rate 建立之间的空档。
 *
 * 当前顺时针赛道 turn_command 在右转时为负值（第一段恒速右弯稳态约 -85），
 * RIGHT_TURN_SIGN = -1 将 turn_command 归一化为正幅值。
 *
 * 预瞄不依赖 motion_state==4；当 turn_magnitude 超过 START 阈值且 IMU/运动链路
 * 有效即开始建立。yaw_rate 建立后通过 yaw_handover 逐渐退出，由现有 YF 接管。
 *
 * CSV 实测依据（tick≈19342～19582）：
 *   turn_command 约 200 ms 前已出现方向，|tc|≥25 约 140 ms 前；
 *   稳态右弯 |tc|≈61～89；yaw 从 0 建立到 10 °/s 约 200 ms。
 */
#define TURN_PREVIEW_ENABLED                        1U
#define TURN_PREVIEW_RIGHT_SIGN                    (-1)
#define TURN_PREVIEW_MAX_US                        10.0f
#define TURN_PREVIEW_START                         20.0f
#define TURN_PREVIEW_FULL                          55.0f
#define TURN_PREVIEW_SIGN                          (-1)
#define TURN_PREVIEW_RISE_STEP_US                  8.0f
#define TURN_PREVIEW_FALL_STEP_US                  10.0f
#define TURN_PREVIEW_YAW_HANDOVER_DPS              25.0f

/*
 * 转弯积分交接（turn integral handover）。
 *
 * 弯道期间积分容易累积过量（实测约+23~27 us），出弯后误差反向时积分
 * 仍保持原方向，造成出弯阶段error超标。本模块：
 * 1. 入弯时保存积分基准 i_turn_entry_us；
 * 2. 弯中限制积分相对基准的变化范围（±delta）；
 * 3. 出弯后冻结正常积分，以固定步长平滑拉回入弯基准。
 *
 * Keil Watch 可调参数：
 *   g_turn_i_delta_pos_us    弯中允许的最大正向增量，us
 *   g_turn_i_delta_neg_us    弯中允许的最大负向增量，us
 *   g_turn_exit_i_step_us    出弯后每20ms拉回步长，us
 */
#define TURN_INTEGRAL_HANDOVER_ENABLED              1U
#define TURN_I_DELTA_POS_US                         15.0f
#define TURN_I_DELTA_NEG_US                         15.0f
#define TURN_EXIT_I_STEP_US                         3.0f
#define TURN_EXIT_I_CONFIRM_MS                      60U

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
 * STM32 最后一层视觉跳变防御。
 *
 * K230 已做速度外推过滤（predict_gate）和重新捕获多帧确认，正常收到的
 * 相邻有效帧位置连续。这里仍拒绝与上一有效位置相差超过该值的测量帧，
 * 作为通信错误或K230端遗漏的最后防线。拒绝帧不写入差分历史，下一帧
 * 仍可与上一有效帧正常差分。阈值比K230的predict_gate(20px)留出余量，
 * 避免与K230外推位置互相误伤。
 */
#define BALL_CONTROL_REJECT_JUMP_PX      40

/*
 * 位置积分。
 *
 * 只要新视觉帧的误差位于积分区内，就使用真实dt按完整Ki积分。
 * 积分项具有独立限幅（±MAX_US），PWM饱和且积分继续推向饱和时执行抗饱和回退。
 * Guard复位或Ki设为0时清零。转弯期间由TURN_INTEGRAL_HANDOVER接管限幅。
 */
#define BALL_CONTROL_LEGACY_INTEGRAL_ZONE_PX     80
#define BALL_CONTROL_INTEGRAL_MAX_US              90.0f

/*
 * STM32固定周期控制任务。
 * SysTick每1 ms进入一次中断，累计20次后执行一次状态更新、保护判断和PD。
 * 该周期与K230约20 Hz的视觉更新、TIM2的333 Hz硬件PWM是三个独立时基。
 */
#define BALL_CONTROL_PERIOD_MS            20U

/*
 * 控制量分解调试遥测。
 *
 * 设为1时，遥测协议从V2(64字节)切换为V3(96字节)，额外包含P/I/D/AF/YF的
 * 完整分解链路和最终舵机输出链路。仅用于诊断，正常运行时保持为0。
 *
 * 同时需要在K230端更新telemetry_logger.py以支持96字节V3帧。
 */
#define BALL_TELEMETRY_CONTROL_DECOMPOSITION 1U

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
#define BALL_CONTROL_DEFAULT_KP_US_PER_PX  3.5f
#define BALL_CONTROL_DEFAULT_KI_US_PER_PX_S 1.5f
#define BALL_CONTROL_DEFAULT_KV_US_PER_PX_S 0.9f
#define BALL_CONTROL_DEFAULT_DIRECTION     (-1)

#endif
