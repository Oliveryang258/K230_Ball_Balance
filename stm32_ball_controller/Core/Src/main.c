/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_uart_rx.h"
#include "app_telemetry_tx.h"
#include "app_motion_rx.h"
#include "app_justfloat_tx.h"
#include "app_config.h"
#include "ball_controller.h"
#include "control_guard.h"
#include "icm20602.h"
#include "servo_output.h"
#include "task3_sequence.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*
 * 保存最近一帧通过版本和校验检查的视觉数据。
 * 不能放在 while (1) 内部，否则每轮循环都会重新清零。
 */
VisionMeasurement g_measurement = {0};

/*
 * 以下变量带 volatile，方便在 Keil Watch 窗口实时观察。
 * volatile 也避免编译器因为程序暂时没有使用这些值而将它们优化掉。
 */
volatile ControlGuardState g_guard_state = CONTROL_GUARD_UART_TIMEOUT;
volatile uint8_t g_uart_rx_started = 0U;
volatile uint32_t g_uart_valid_packet_count = 0U;
volatile uint32_t g_uart_error_count = 0U;
volatile uint32_t g_protocol_error_count = 0U;
volatile uint8_t g_telemetry_tx_started = 0U;
volatile uint32_t g_telemetry_sent_count = 0U;
volatile uint32_t g_telemetry_overwrite_count = 0U;
volatile uint32_t g_telemetry_error_count = 0U;
volatile uint8_t g_control_runtime_started = 0U;
volatile uint32_t g_control_tick_count = 0U;

/*
 * 车辆运动遥测和 HC-06 调试链路的 Watch 变量。
 *
 * 车辆数据第一阶段只用于记录，不参与闭环控制。只有完成跑圈采样并确认
 * acc_track/yaw_rate 的物理轴向、正负号、噪声和时延以后，才允许把它接入前馈。
 */
volatile uint8_t g_motion_rx_started = 0U;
volatile uint8_t g_motion_link_valid = 0U;
volatile uint32_t g_motion_packet_age_ms = 0U;
volatile uint32_t g_motion_valid_packet_count = 0U;
volatile uint32_t g_motion_uart_error_count = 0U;
volatile uint32_t g_motion_protocol_error_count = 0U;
Ch32MotionMeasurement g_motion_measurement = {0};

volatile uint8_t g_justfloat_tx_started = 0U;
volatile uint32_t g_justfloat_sent_count = 0U;
volatile uint32_t g_justfloat_overwrite_count = 0U;
volatile uint32_t g_justfloat_error_count = 0U;

/*
 * ICM20602 第一阶段调试变量。
 *
 * 这些变量只用于确认“供电、软件I2C、初始化和数据读取”是否正常，
 * 当前没有接入位置环、速度环或舵机PWM计算。
 *
 * Keil Watch 建议先观察：
 *   g_imu_init_ok       初始化是否成功，成功应为1
 *   g_imu_status        驱动状态，1表示ICM20602_STATUS_OK
 *   g_imu_who_am_i      芯片身份，正确应为0x12
 *   g_imu_address       AD0低/高分别对应0x68/0x69
 *   g_imu_sample_count  成功读取计数，应持续增加
 *   g_imu_error_count   运行时读取失败计数，正常应保持0
 *   g_imu_sample        展开后查看加速度、角速度和温度
 */
volatile uint8_t g_imu_init_ok = 0U;
volatile uint8_t g_imu_valid = 0U;
volatile uint8_t g_imu_status = (uint8_t)ICM20602_STATUS_NOT_INITIALIZED;
volatile uint8_t g_imu_who_am_i = 0U;
volatile uint8_t g_imu_address = 0U;
volatile uint32_t g_imu_sample_count = 0U;
volatile uint32_t g_imu_error_count = 0U;
volatile uint32_t g_imu_last_read_tick = 0U;
ICM20602Sample g_imu_sample = {0};

/*
 * 舵机台架调试变量：默认关闭手动测试，默认目标取自配置文件中的暂定中位。
 * 使用 Watch 手动测试前必须取下钢球，并确认连杆没有顶死或明显预紧。
 */
volatile uint8_t g_servo_pwm_started = 0U;
volatile uint8_t g_manual = 0U;
volatile uint16_t g_manual_us = SERVO_PWM_NEUTRAL_US;

/*
 * 位置PID在guard为READY时固定以50 Hz计算；g_apply只负责是否应用输出。
 * 无论g_apply状态如何，ControlGuard失败都会强制回中位。
 */
BallController g_ball_controller;

/*
 * Keil Watch可直接修改的短变量。
 * 普通闭环调参时只需要添加g_kp、g_ki、g_kv、g_dir和g_apply。
 * 第三问改变停球点时直接修改g_target_x：
 *   314=中心，190=-5 cm（舵机端），440=+5 cm（固定端）。
 */
volatile uint8_t g_apply = BALL_CONTROL_DEFAULT_APPLY_OUTPUT;
volatile float g_kp = BALL_CONTROL_DEFAULT_KP_US_PER_PX;
volatile float g_ki = BALL_CONTROL_DEFAULT_KI_US_PER_PX_S;
volatile float g_kv = BALL_CONTROL_DEFAULT_KV_US_PER_PX_S;
volatile int8_t g_dir = BALL_CONTROL_DEFAULT_DIRECTION;
volatile int16_t g_target_x = BALL_CONTROL_DEFAULT_TARGET_X;
volatile uint16_t g_hold_pwm_us = BALL_CONTROL_DEFAULT_HOLD_PWM_US;

/*
 * 第三题专用PID调参接口。
 * 自动序列RUNNING和COMPLETE期间使用本组参数；普通闭环仍使用g_kp/g_ki/g_kv。
 */
volatile float g_task3_kp = TASK3_DEFAULT_KP_US_PER_PX;
volatile float g_task3_ki = TASK3_DEFAULT_KI_US_PER_PX_S;
volatile float g_task3_kv = TASK3_DEFAULT_KV_US_PER_PX_S;
/*
 * 车辆运动题专用固定前馈：
 * - 普通调参/车辆运动时可在Watch把enabled改为1；
 * - 第三题状态机不处于IDLE时自动旁路，不改变第三题任何目标点；
 * - effective_hold只读，表示本周期实际交给控制器的平衡PWM基准。
 */
volatile uint8_t g_cruise_ff_enabled =
    BALL_CONTROL_DEFAULT_CRUISE_FF_ENABLED;
volatile int16_t g_cruise_ff_us =
    BALL_CONTROL_DEFAULT_CRUISE_FF_US;
volatile uint16_t g_effective_hold_pwm_us =
    BALL_CONTROL_DEFAULT_HOLD_PWM_US;

/*
 * 加速度前馈使用短变量名，方便放入Keil Watch：
 *   g_kaf    前馈增益，单位us/mg；改成0即可关闭
 *   g_af_lim 前馈软限幅，单位us
 *   g_af     本周期实际采用的前馈，单位us，只读
 *
 * 不再设置单独的enable变量，现场关闭前馈时只需要把g_kaf改成0。
 */
volatile float g_kaf = BALL_CONTROL_DEFAULT_KAF_US_PER_MG;
volatile float g_af_lim = BALL_CONTROL_DEFAULT_AF_LIMIT_US;
volatile float g_af = 0.0f;

/*
 * 右转偏航角速度前馈的短变量名，方便Keil Watch现场调参：
 *   g_ky      偏航增益，单位us/(deg/s)；改成0即可关闭
 *   g_yf_lim  偏航前馈软限幅，单位us
 *   g_vref    满比例补偿对应的参考轮速，单位与CH32上报轮速相同
 *   g_yf      本周期实际采用的偏航前馈，单位us，只读
 */
volatile float g_ky = BALL_CONTROL_DEFAULT_KY_US_PER_DPS;
volatile float g_yf_lim = BALL_CONTROL_DEFAULT_YF_LIMIT_US;
volatile float g_vref = BALL_CONTROL_DEFAULT_VREF;
volatile float g_yf = 0.0f;

/*
 * 右转预瞄 Keil Watch 可调参数：
 *   g_turn_preview_max_us          预瞄最大幅值，us；改成0即可关闭
 *   g_turn_preview_start           turn_magnitude 起始阈值
 *   g_turn_preview_full            turn_magnitude 满幅阈值
 *   g_turn_preview_sign            补偿方向，+1 或 -1
 *   g_turn_preview_rise_step_us    每20ms最大建立量
 *   g_turn_preview_fall_step_us    每20ms最大撤除量
 *   g_turn_preview_yaw_handover_dps  yaw 达此值时预瞄完全退出
 */
volatile float g_turn_preview_max_us = TURN_PREVIEW_MAX_US;
volatile float g_turn_preview_start = TURN_PREVIEW_START;
volatile float g_turn_preview_full = TURN_PREVIEW_FULL;
volatile float g_turn_preview_sign = TURN_PREVIEW_SIGN;
volatile float g_turn_preview_rise_step_us = TURN_PREVIEW_RISE_STEP_US;
volatile float g_turn_preview_fall_step_us = TURN_PREVIEW_FALL_STEP_US;
volatile float g_turn_preview_yaw_handover_dps = TURN_PREVIEW_YAW_HANDOVER_DPS;

/*
 * 转弯积分交接 Keil Watch 可调参数：
 *   g_turn_i_delta_pos_us    弯中允许的最大正向增量，us
 *   g_turn_i_delta_neg_us    弯中允许的最大负向增量，us
 *   g_turn_exit_i_step_us    出弯后每20ms拉回步长，us
 *
 * Watch 只读：
 *   g_turn_i_entry_us        入弯时保存的积分基准，us
 *   g_turn_i_state           0=IDLE, 1=弯中限幅, 2=出弯交接
 */
volatile float g_turn_i_delta_pos_us = TURN_I_DELTA_POS_US;
volatile float g_turn_i_delta_neg_us = TURN_I_DELTA_NEG_US;
volatile float g_turn_exit_i_step_us = TURN_EXIT_I_STEP_US;
volatile float g_turn_i_entry_us = 0.0f;
volatile uint8_t g_turn_i_state = 0U;

/*
 * Watch只读：EMA滤波后的轨道加速度，单位mg。
 * 由AppControl_On1msTick在每个控制周期更新。
 */
volatile float g_acc_filt_mg = 0.0f;

/*
 * 短暂丢球容错Watch变量：
 *   g_lost_age_ms        当前连续丢球时长，ms；0表示未丢球
 *   g_lost_grace_active  1表示处于宽限期内（冻结伺服、保留积分）
 *   g_lost_recovery_count 宽限期内恢复的累计次数
 */
volatile uint32_t g_lost_age_ms = 0U;
volatile uint8_t g_lost_grace_active = 0U;
volatile uint32_t g_lost_recovery_count = 0U;

/*
 * 第三题自动演示的Watch接口：
 * 把g_task3_start写成1，程序会在下一个20 ms控制周期自动开始并清零它。
 * 实体按键接好后，按键回调只需调用AppTask3_RequestStart()。
 */
static Task3Sequence s_task3_sequence;
volatile uint8_t g_task3_start = 0U;
volatile uint8_t g_task3_state = (uint8_t)TASK3_SEQUENCE_IDLE;
volatile uint8_t g_task3_step = 0U;
volatile uint32_t g_task3_elapsed_ms = 0U;
volatile int16_t g_task3_final_x = BALL_CONTROL_DEFAULT_TARGET_X;

/*
 * Keil Watch只读调试结构体。
 * Watch中只添加g_dbg并展开，即可一次看到测量、控制和PWM状态。
 */
typedef struct
{
    uint32_t tick;          /* 20 ms控制任务累计执行次数 */
    uint8_t guard;          /* 当前保护状态，0～5 */
    uint8_t meas_status;    /* 最近测量结果：0接受、1重复、2无效、3 dt异常、255尚未处理过测量 */
    int16_t x;              /* 最近钢球横坐标 */
    int16_t err;            /* 位置误差，单位px */
    float vel;              /* 滤波速度，单位px/s */
    float p;                /* P项，单位us */
    float i;                /* 位置I项，单位us */
    float d;                /* D项，单位us */
    float out;              /* 方向处理后的控制偏移，单位us */
    uint8_t i_reset_reason; /* 最近一次I清零原因，粘滞保存 */
    uint32_t i_reset_count; /* I从非零变为零的累计次数 */
    uint16_t pwm_target;    /* 舵机模块当前目标脉宽 */
    uint16_t pwm_now;       /* 当前实际写入CCR的脉宽 */
    int16_t acc;            /* CH32轨道方向加速度，单位mg */
    float af;               /* 本周期实际加速度前馈，单位us */
    float yaw;              /* 偏航角速度，单位deg/s */
    float yf;               /* 本周期实际右转偏航前馈，单位us */
    uint32_t lost_age_ms;   /* 当前连续丢球时长，ms */
    uint8_t lost_grace;     /* 1表示处于丢球宽限期内 */
    uint32_t lost_recovery; /* 宽限期内恢复的累计次数 */
} ControlDebug;

volatile ControlDebug g_dbg = {0};

/*
 * 控制量分解 Watch 变量（只读）。
 *
 * PID 反馈链路
 */
volatile float g_error_px = 0.0f;
volatile float g_p_term_raw_us = 0.0f;
volatile float g_i_term_raw_us = 0.0f;
volatile float g_d_term_raw_us = 0.0f;
volatile float g_pid_sum_raw_us = 0.0f;
volatile float g_pid_sum_directed_us = 0.0f;
volatile float g_control_offset_us = 0.0f;

/* 加速度前馈链路 */
volatile float g_acc_raw_mg = 0.0f;
volatile float g_af_raw_us = 0.0f;
volatile float g_af_clamped_us = 0.0f;
volatile float g_af_slewed_us = 0.0f;

/* 偏航前馈链路 */
volatile float g_yaw_raw_dps = 0.0f;
volatile float g_speed_average = 0.0f;
volatile float g_speed_scale = 0.0f;
volatile float g_yf_raw_us = 0.0f;
volatile float g_yf_clamped_us = 0.0f;
volatile float g_yf_slewed_us = 0.0f;

/* 最终输出链路 */
volatile float g_hold_pwm_effective_us = 0.0f;
volatile float g_feedforward_total_us = 0.0f;
volatile float g_servo_prelimit_us = 0.0f;
volatile float g_servo_target_debug_us = 0.0f;
volatile float g_servo_current_debug_us = 0.0f;
volatile float g_servo_tracking_error_us = 0.0f;

volatile uint8_t g_servo_target_saturated = 0U;
volatile uint8_t g_servo_slew_active = 0U;
volatile uint8_t g_af_slew_active = 0U;
volatile uint8_t g_yf_slew_active = 0U;

/* 右转预瞄链路 */
volatile float g_turn_magnitude = 0.0f;
volatile float g_turn_scale = 0.0f;
volatile float g_yaw_handover = 0.0f;
volatile float g_turn_preview_raw_us = 0.0f;
volatile float g_turn_preview_target_us = 0.0f;
volatile float g_turn_preview_slewed_us = 0.0f;
volatile uint8_t g_turn_preview_slew_active = 0U;

/* 分解一致性检查 */
volatile float g_control_decomposition_error_us = 0.0f;
volatile uint32_t g_decomposition_error_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void AppTask3_RequestStart(void)
{
    /*
     * 按键中断只置一个请求标志，不在按键中断中运行状态机或PID。
     * 状态机统一由SysTick中的50 Hz控制任务更新。
     */
#if TASK3_SEQUENCE_ENABLED != 0U
    g_task3_start = 1U;
#else
    /*
     * 当前处于纯调参模式，第三题接口暂时停用。
     * 保留函数是为了以后恢复状态机时不必改动按键调用代码。
     */
    g_task3_start = 0U;
#endif
}

static uint16_t AppClampControlPwm(int32_t pulse_us)
{
    if (pulse_us < (int32_t)BALL_CONTROL_PWM_MIN_US)
    {
        return BALL_CONTROL_PWM_MIN_US;
    }
    if (pulse_us > (int32_t)BALL_CONTROL_PWM_MAX_US)
    {
        return BALL_CONTROL_PWM_MAX_US;
    }
    return (uint16_t)pulse_us;
}

/*
 * 对称限制前馈，并保留独立的硬安全上限。
 * 如果Watch误把g_af_lim写成负数，程序按其绝对值处理。
 */
static float AppClampAf(float value, float limit)
{
    if (limit < 0.0f)
    {
        limit = -limit;
    }
    if (limit > BALL_CONTROL_AF_HARD_LIMIT_US)
    {
        limit = BALL_CONTROL_AF_HARD_LIMIT_US;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/*
 * 偏航前馈独立限幅。即使Watch中误写了过大的g_yf_lim，也不能超过硬上限。
 */
static float AppClampYf(float value, float limit)
{
    if (limit < 0.0f)
    {
        limit = -limit;
    }
    if (limit > BALL_CONTROL_YF_HARD_LIMIT_US)
    {
        limit = BALL_CONTROL_YF_HARD_LIMIT_US;
    }
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static int32_t AppRoundFloatToI32(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }
    return (int32_t)(value - 0.5f);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    AppUartRx_OnRxComplete(huart);
    AppMotionRx_OnRxComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    AppTelemetryTx_OnTxComplete(huart);
    AppJustFloatTx_OnTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    AppTelemetryTx_OnError(huart);
    AppUartRx_OnError(huart);
    AppMotionRx_OnError(huart);
    AppJustFloatTx_OnError(huart);
}

static int16_t Telemetry_RoundFloatToI16(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return -32768;
    }
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

/*
 * I清零原因采用粘滞记录，避免Keil Watch漏掉仅持续一个20 ms周期的guard。
 * 1 UART超时，2丢球，3位置越界，4 dt异常，5协议/UART错误，6 Ki被设为0。
 */
static void Debug_RecordIntegralReset(uint8_t reason)
{
    g_dbg.i_reset_reason = reason;
    g_dbg.i_reset_count++;
}

/*
 * 在 main 普通上下文中以100 Hz读取IMU。
 *
 * 软件I2C逐位产生时序，属于阻塞式操作，因此绝不能把本函数放进
 * SysTick或UART中断。中断仍然可以短暂抢占本函数，不会把50 Hz球控
 * 任务的计算主体变成阻塞式I2C。
 */
static void AppImu_Poll(void)
{
    static uint32_t previous_read_tick = 0U;
    uint32_t now_tick;
    ICM20602Sample new_sample;

    if (g_imu_init_ok == 0U)
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((uint32_t)(now_tick - previous_read_tick) < 10U)
    {
        return;
    }

    /*
     * 使用当前真实毫秒tick调度，不在循环中做HAL_Delay(10)。
     * 如果主循环偶尔被其他任务推迟，直接以当前时刻重新定基准，
     * 避免随后连续补读多帧旧数据。
     */
    previous_read_tick = now_tick;
    g_imu_last_read_tick = now_tick;

    if (ICM20602_ReadSample(&new_sample))
    {
        g_imu_sample = new_sample;
        g_imu_valid = 1U;
        g_imu_sample_count++;
    }
    else
    {
        g_imu_valid = 0U;
        g_imu_error_count++;
    }

    g_imu_status = (uint8_t)ICM20602_GetStatus();
}

/*
 * 固定1 ms入口、20 ms执行一次的控制任务。
 * USART ISR只发布数据；本函数先做一致性快照，再在中断保持开启的情况下
 * 完成guard、速度估计、位置PID和舵机slew目标更新。
 */
void AppControl_On1msTick(void)
{
    static uint16_t control_divider_ms = 0U;
    static uint32_t previous_protocol_error_count = 0U;
    static uint32_t previous_uart_error_count = 0U;
    static float s_acc_filt_mg = 0.0f;
    static bool  s_acc_filt_init = false;
    static float s_af_prev = 0.0f;
    static float s_yf_prev = 0.0f;
#if TURN_INTEGRAL_HANDOVER_ENABLED != 0U
    static float    s_turn_i_entry_us = 0.0f;
    static uint8_t  s_turn_i_state = 0U;
    static uint32_t s_turn_exit_confirm_tick = 0U;
    static uint8_t  s_turn_exit_confirm_count = 0U;
    static uint8_t  s_prev_motion_state = 0U;
    static float    s_prev_yaw_magnitude = 0.0f;
    static bool     s_turn_active_last = false;
    static uint8_t  s_turn_idle_timeout = 0U;
#endif
    static uint32_t s_ball_lost_tick = 0U;
    static bool     s_ball_lost_tick_valid = false;
    static bool     s_lost_grace_active = false;
    static uint16_t s_last_valid_target_us = SERVO_PWM_NEUTRAL_US;
    static bool     s_lost_recovery_pending = false;
    AppUartRxSnapshot rx_snapshot;
    AppMotionRxSnapshot motion_snapshot;
    BallMeasurementResult measurement_result = BALL_MEAS_INVALID;
    bool protocol_error_event;
    bool invalid_dt_event = false;
    bool control_ran = false;
    float integral_before_measurement;
    float integral_before_control;
    float effective_kp;
    float effective_ki;
    float effective_kv;
    float yaw_dps;
    float wheel_speed;
    float speed_scale;
    int32_t hold_pwm;
    uint32_t now_ms;
    TelemetrySample telemetry_sample;
    JustFloatSample justfloat_sample;
#if TASK3_SEQUENCE_ENABLED != 0U
    uint8_t task3_step_before;
#endif

    if (g_control_runtime_started == 0U)
    {
        control_divider_ms = 0U;
        return;
    }

    control_divider_ms++;
    if (control_divider_ms < BALL_CONTROL_PERIOD_MS)
    {
        return;
    }
    control_divider_ms = 0U;
    g_control_tick_count++;
    now_ms = HAL_GetTick();

    /*
     * Watch中可直接修改g_target_x和g_hold_pwm_us。
     * 每个控制周期都写回限幅后的真实值，避免调试时输入越界参数。
     */
    g_hold_pwm_us = AppClampControlPwm((int32_t)g_hold_pwm_us);

    /*
     * TakeSnapshot仅在复制共享字段期间短暂关闭中断。
     * 返回后多字段测量和它的接收tick属于同一发布版本。
     */
    (void)AppUartRx_TakeSnapshot(&rx_snapshot);
    g_measurement = rx_snapshot.measurement;

    /*
     * 车辆帧也采用一次性快照，保证 sequence、时间戳、IMU 和轮速来自同一帧。
     * 此处只做记录；g_motion_link_valid 绝不等于“允许参与控制”。
     */
    (void)AppMotionRx_TakeSnapshot(&motion_snapshot);
    g_motion_measurement = motion_snapshot.measurement;
    if (motion_snapshot.has_packet)
    {
        g_motion_packet_age_ms =
            (uint32_t)(now_ms - motion_snapshot.last_packet_tick);
        g_motion_link_valid =
            (g_motion_packet_age_ms <= MOTION_LINK_TIMEOUT_MS) ? 1U : 0U;
    }
    else
    {
        g_motion_packet_age_ms = MOTION_LINK_TIMEOUT_MS + 1U;
        g_motion_link_valid = 0U;
    }

    protocol_error_event =
        (rx_snapshot.protocol_error_count != previous_protocol_error_count) ||
        (rx_snapshot.uart_error_count != previous_uart_error_count);
    previous_protocol_error_count = rx_snapshot.protocol_error_count;
    previous_uart_error_count = rx_snapshot.uart_error_count;

    g_guard_state = ControlGuard_Evaluate(
        &g_measurement,
        rx_snapshot.has_packet,
        rx_snapshot.last_packet_tick,
        now_ms,
        protocol_error_event,
        false
    );

    /*
     * 短暂丢球容错：K230偶尔单帧丢球时，不立即清空积分和速度历史，
     * 而是在宽限期内保持最后一次有效舵机目标并冻结积分。
     *
     * UART超时和位置越界等硬错误不经过宽限期，仍使用原严格保护。
     */
    if (g_guard_state == CONTROL_GUARD_BALL_LOST)
    {
        if (!s_ball_lost_tick_valid)
        {
            s_ball_lost_tick = now_ms;
            s_ball_lost_tick_valid = true;
            s_lost_grace_active = true;
            s_last_valid_target_us =
                ServoOutput_GetTargetPulseUs();
            s_lost_recovery_pending = false;
        }

        g_lost_age_ms =
            (uint32_t)(now_ms - s_ball_lost_tick);

        if (g_lost_age_ms < BALL_LOST_GRACE_MS)
        {
            g_lost_grace_active = 1U;
        }
        else
        {
            g_lost_grace_active = 0U;
            s_lost_grace_active = false;
        }
    }
    else
    {
        if (s_ball_lost_tick_valid)
        {
            s_ball_lost_tick_valid = false;

            if (s_lost_grace_active)
            {
                s_lost_recovery_pending = true;
                g_lost_recovery_count++;
            }

            s_lost_grace_active = false;
            g_lost_grace_active = 0U;
        }
        g_lost_age_ms = 0U;
    }

    if ((g_guard_state == CONTROL_GUARD_READY) &&
        rx_snapshot.new_packet)
    {
        /*
         * 宽限期恢复后第一帧：清除位置差分历史，避免基于长时间间隔的
         * 旧位置产生速度尖峰。积分和平衡PWM保持不变。
         */
        if (s_lost_recovery_pending)
        {
            BallController_ClearVelocityHistory(&g_ball_controller);
            s_lost_recovery_pending = false;
        }

        /*
         * dt来自50 Hz控制任务先后消费的两个最新有效测量在STM32上的
         * 真实接收tick。K230发送快于控制周期时允许跳过中间frame_id，
         * 但绝不使用固定20 ms伪造测量dt；重复frame_id也不会更新速度。
         * 异常dt会清空差分历史并丢弃本帧。
         */
        integral_before_measurement = g_ball_controller.i_term_us;
        measurement_result = BallController_AcceptMeasurementEx(
            &g_ball_controller,
            &g_measurement,
            rx_snapshot.last_packet_tick
        );
        g_dbg.meas_status = (uint8_t)measurement_result;
        invalid_dt_event =
            (measurement_result == BALL_MEAS_INVALID_DT);

        if (invalid_dt_event)
        {
            if (integral_before_measurement != 0.0f)
            {
                Debug_RecordIntegralReset(4U);
            }
            g_guard_state = CONTROL_GUARD_INVALID_DT;
        }
    }

#if TASK3_SEQUENCE_ENABLED != 0U
    /*
     * 第三题启动请求既可以由Watch写入，也可以由未来的实体按键调用
     * AppTask3_RequestStart()产生。请求只在50 Hz控制任务里消费。
     */
    if (g_task3_start != 0U)
    {
        g_task3_start = 0U;
        /*
         * RUNNING期间忽略重复请求，避免机械按键抖动反复把计时器清零。
         * COMPLETE、TIMEOUT或IDLE状态下再次按键都可以重新演示。
         */
        if (Task3Sequence_GetState(&s_task3_sequence) !=
            TASK3_SEQUENCE_RUNNING)
        {
            Task3Sequence_Start(&s_task3_sequence, now_ms);
            /*
             * 第三题开始时会自动撤销巡航前馈；同时清除车辆运动阶段积累的I，
             * 避免旧巡航补偿残留到静止第三题。
             */
            BallController_ResetTargetSlowState(&g_ball_controller);
        }
    }

    task3_step_before = Task3Sequence_GetStep(&s_task3_sequence);
    {
        /*
         * 被跳变防御拒绝的帧不能用来判断第三题到位误差。
         * 用上一有效帧位置（last_ball_x）替代，避免异常位置误判到位。
         */
        int16_t task3_ball_x = g_measurement.ball_x;
        if (measurement_result == BALL_MEAS_REJECTED)
        {
            task3_ball_x = g_ball_controller.last_ball_x;
        }
        Task3Sequence_Update(
            &s_task3_sequence,
            now_ms,
            (g_guard_state == CONTROL_GUARD_READY),
            task3_ball_x,
            g_ball_controller.velocity_px_s
        );
    }

    /*
     * RUNNING期间状态机拥有目标位置，人工修改g_target_x会被覆盖。
     * COMPLETE后保持题目要求的最后一个-5 cm目标；TIMEOUT也保持当时目标。
     * 演示结束后不再持续覆盖g_target_x，方便继续人工调试。
     */
    if (Task3Sequence_GetState(&s_task3_sequence) ==
        TASK3_SEQUENCE_RUNNING)
    {
        g_target_x = Task3Sequence_GetTargetX(&s_task3_sequence);

        /*
         * 新步骤开始时只清一次旧目标的积分；之后参考目标每20 ms平滑移动，
         * 使用TrackTargetX保留运动阶段的积分，避免斜坡期间I始终为零。
         */
        if (Task3Sequence_GetStep(&s_task3_sequence) !=
            task3_step_before)
        {
            BallController_ResetTargetSlowState(&g_ball_controller);
        }
    }
    /*
     * 写入本周期最终目标。目标变化时SetTargetX会清除旧目标下的I项，
     * 防止切点瞬间被旧积分推向错误方向。
     */
    if (Task3Sequence_GetState(&s_task3_sequence) ==
        TASK3_SEQUENCE_RUNNING)
    {
        g_target_x = BallController_TrackTargetX(
            &g_ball_controller,
            g_target_x
        );
    }
    else
    {
        g_target_x = BallController_SetTargetX(
            &g_ball_controller,
            g_target_x
        );
    }

    g_task3_state =
        (uint8_t)Task3Sequence_GetState(&s_task3_sequence);
    g_task3_step = Task3Sequence_GetStep(&s_task3_sequence);
    g_task3_elapsed_ms =
        Task3Sequence_GetElapsedMs(&s_task3_sequence);
    g_task3_final_x =
        Task3Sequence_GetFinalTargetX(&s_task3_sequence);
#else
    /*
     * 纯调参模式：
     * - 第三题状态机不再接管目标；
     * - g_task3_start即使在Watch中被写成1，也会被清回0；
     * - g_target_x是唯一的位置目标，可在Watch中随时修改；
     * - SetTargetX只负责安全范围限幅和目标变化时清理旧目标慢状态。
     */
    g_task3_start = 0U;
    g_target_x = BallController_SetTargetX(
        &g_ball_controller,
        g_target_x
    );
    g_task3_state = (uint8_t)TASK3_SEQUENCE_IDLE;
    g_task3_step = 0U;
    g_task3_elapsed_ms = 0U;
    g_task3_final_x = g_target_x;
#endif

    /*
     * 固定巡航前馈只改变PWM平衡基准，不篡改314/190/440等物理目标坐标。
     * 第三题从RUNNING直到COMPLETE/TIMEOUT都不允许叠加车辆运动补偿。
     *
     * 当前尚未接入车辆启动命令，因此运动题开始前可在Watch中把
     * g_cruise_ff_enabled设为1；停车调参或做第三题时设为0。
     */
#if TASK3_SEQUENCE_ENABLED != 0U
    if ((Task3Sequence_GetState(&s_task3_sequence) ==
         TASK3_SEQUENCE_IDLE) &&
        (g_cruise_ff_enabled != 0U))
#else
    if (g_cruise_ff_enabled != 0U)
#endif
    {
        hold_pwm =
            (int32_t)g_hold_pwm_us + (int32_t)g_cruise_ff_us;
    }
    else
    {
        hold_pwm = (int32_t)g_hold_pwm_us;
    }

    /*
     * 这里只使用本周期开始时取得的一致性CH32快照。前馈在起步、直行、
     * 转弯和停车过程中生效，车辆完全停止时自动归零。
     *
     * 赛题固定顺时针发车，不设置左转/右转两套系数。acc_track_mg已经表示
     * 沿钢球轨道方向的加速度，同一信号即可处理起步、直线振动、右转和停车。
     */
    g_af = 0.0f;
    g_acc_raw_mg = 0.0f;
    g_af_raw_us = 0.0f;
    g_af_clamped_us = 0.0f;
    if ((g_motion_link_valid != 0U) &&
        ((g_motion_measurement.flags &
          CH32_MOTION_FLAG_IMU_VALID) != 0U) &&
        ((g_motion_measurement.flags &
          CH32_MOTION_FLAG_FAULT) == 0U) &&
        (g_motion_measurement.motion_state >=
         CH32_MOTION_STATE_STARTING) &&
        (g_motion_measurement.motion_state <=
         CH32_MOTION_STATE_STOPPING))
    {
        float acc_raw_mg;

        acc_raw_mg = (float)g_motion_measurement.acc_track_mg;
        g_acc_raw_mg = acc_raw_mg;

        if (!s_acc_filt_init)
        {
            s_acc_filt_mg = acc_raw_mg;
            s_acc_filt_init = true;
        }
        s_acc_filt_mg +=
            BALL_CONTROL_ACC_EMA_ALPHA * (acc_raw_mg - s_acc_filt_mg);
        g_acc_filt_mg = s_acc_filt_mg;

        g_af_raw_us = g_kaf * s_acc_filt_mg;
        g_af = AppClampAf(g_af_raw_us, g_af_lim);
        g_af_clamped_us = g_af;
    }

    /* AF变化率限制，抑制高频抖动直接进入PWM */
    {
        float af_step = g_af - s_af_prev;
        if (af_step > BALL_CONTROL_AF_MAX_STEP_US)
        {
            g_af = s_af_prev + BALL_CONTROL_AF_MAX_STEP_US;
            g_af_slew_active = 1U;
        }
        else if (af_step < -BALL_CONTROL_AF_MAX_STEP_US)
        {
            g_af = s_af_prev - BALL_CONTROL_AF_MAX_STEP_US;
            g_af_slew_active = 1U;
        }
        else
        {
            g_af_slew_active = 0U;
        }
        s_af_prev = g_af;
    }
    g_af_slewed_us = g_af;

    hold_pwm += AppRoundFloatToI32(g_af);

    /*
     * 右转偏航角速度前馈：
     * 1. 当前赛题固定顺时针，只在TURN_RIGHT状态启用；
     * 2. yaw_rate_dps10除以10得到deg/s；
     * 3. 左右轮速取平均，并按g_vref归一化到0～1；
     * 4. 补偿叠加到平衡PWM基准，不绕过PID直接控制舵机。
     *
     * 当前右转yaw为负，因此g_ky初值也为负，两者相乘得到正PWM补偿。
     */
    g_yf = 0.0f;
    g_yaw_raw_dps = 0.0f;
    g_speed_average = 0.0f;
    g_speed_scale = 0.0f;
    g_yf_raw_us = 0.0f;
    g_yf_clamped_us = 0.0f;
    if ((g_motion_link_valid != 0U) &&
        ((g_motion_measurement.flags &
          CH32_MOTION_FLAG_IMU_VALID) != 0U) &&
        ((g_motion_measurement.flags &
          CH32_MOTION_FLAG_FAULT) == 0U) &&
        (g_motion_measurement.motion_state ==
         CH32_MOTION_STATE_TURN_RIGHT) &&
        (g_vref > 1.0f))
    {
        yaw_dps =
            (float)g_motion_measurement.yaw_rate_dps10 * 0.1f;
        g_yaw_raw_dps = yaw_dps;
        wheel_speed =
            ((float)g_motion_measurement.left_speed +
             (float)g_motion_measurement.right_speed) * 0.5f;

        if (wheel_speed < 0.0f)
        {
            wheel_speed = -wheel_speed;
        }
        g_speed_average = wheel_speed;

        speed_scale = wheel_speed / g_vref;
        if (speed_scale > 1.0f)
        {
            speed_scale = 1.0f;
        }
        g_speed_scale = speed_scale;

        g_yf_raw_us = g_ky * yaw_dps * speed_scale;
        g_yf = AppClampYf(g_yf_raw_us, g_yf_lim);
        g_yf_clamped_us = g_yf;
    }

    /* YF变化率限制，偏航补偿缓慢建立和撤销 */
    {
        float yf_step = g_yf - s_yf_prev;
        if (yf_step > BALL_CONTROL_YF_MAX_STEP_US)
        {
            g_yf = s_yf_prev + BALL_CONTROL_YF_MAX_STEP_US;
            g_yf_slew_active = 1U;
        }
        else if (yf_step < -BALL_CONTROL_YF_MAX_STEP_US)
        {
            g_yf = s_yf_prev - BALL_CONTROL_YF_MAX_STEP_US;
            g_yf_slew_active = 1U;
        }
        else
        {
            g_yf_slew_active = 0U;
        }
        s_yf_prev = g_yf;
    }
    g_yf_slewed_us = g_yf;

    hold_pwm += AppRoundFloatToI32(g_yf);

#if TURN_PREVIEW_ENABLED != 0U
    /*
     * 右转预瞄：填补 turn_command 出现到 yaw_rate 建立之间的空档。
     *
     * 不依赖 motion_state==4；只要运动链路有效、IMU 正常、turn_command
     * 超过阈值且车速足够即可建立。yaw_rate 建立后通过 handover 逐渐退出，
     * 避免与现有 YF 在稳态弯中重复补偿。
     */
    {
        static float s_turn_preview_prev = 0.0f;
        float speed_scale_preview;
        float wheel_speed_preview;
        float turn_mag;
        float turn_scale;
        float yaw_handover;
        float preview_target;
        float preview_step;

        g_turn_preview_raw_us = 0.0f;
        g_turn_preview_target_us = 0.0f;
        g_turn_magnitude = 0.0f;
        g_turn_scale = 0.0f;
        g_yaw_handover = 0.0f;

        if ((g_motion_link_valid != 0U) &&
            ((g_motion_measurement.flags &
              CH32_MOTION_FLAG_IMU_VALID) != 0U) &&
            ((g_motion_measurement.flags &
              CH32_MOTION_FLAG_FAULT) == 0U) &&
            (g_vref > 1.0f))
        {
            turn_mag =
                (float)TURN_PREVIEW_RIGHT_SIGN *
                (float)g_motion_measurement.turn_command;
            g_turn_magnitude = turn_mag;

            if (turn_mag > g_turn_preview_start)
            {
                turn_scale = (turn_mag - g_turn_preview_start) /
                             (g_turn_preview_full - g_turn_preview_start);
                if (turn_scale > 1.0f)
                {
                    turn_scale = 1.0f;
                }
                else if (turn_scale < 0.0f)
                {
                    turn_scale = 0.0f;
                }
                g_turn_scale = turn_scale;

                wheel_speed_preview =
                    ((float)g_motion_measurement.left_speed +
                     (float)g_motion_measurement.right_speed) * 0.5f;
                if (wheel_speed_preview < 0.0f)
                {
                    wheel_speed_preview = -wheel_speed_preview;
                }
                speed_scale_preview = wheel_speed_preview / g_vref;
                if (speed_scale_preview > 1.0f)
                {
                    speed_scale_preview = 1.0f;
                }

                yaw_handover =
                    ((float)g_motion_measurement.yaw_rate_dps10 * 0.1f);
                if (yaw_handover < 0.0f)
                {
                    yaw_handover = -yaw_handover;
                }
                yaw_handover /= g_turn_preview_yaw_handover_dps;
                if (yaw_handover > 1.0f)
                {
                    yaw_handover = 1.0f;
                }
                else if (yaw_handover < 0.0f)
                {
                    yaw_handover = 0.0f;
                }
                g_yaw_handover = yaw_handover;

                preview_target =
                    g_turn_preview_sign *
                    g_turn_preview_max_us *
                    turn_scale *
                    speed_scale_preview *
                    (1.0f - yaw_handover);
                g_turn_preview_raw_us = preview_target;
                g_turn_preview_target_us = preview_target;
            }
        }

        /* 预瞄独立变化率限制 */
        preview_step =
            g_turn_preview_target_us - s_turn_preview_prev;
        if (preview_step > g_turn_preview_rise_step_us)
        {
            g_turn_preview_slewed_us =
                s_turn_preview_prev + g_turn_preview_rise_step_us;
            g_turn_preview_slew_active = 1U;
        }
        else if (preview_step < -g_turn_preview_fall_step_us)
        {
            g_turn_preview_slewed_us =
                s_turn_preview_prev - g_turn_preview_fall_step_us;
            g_turn_preview_slew_active = 1U;
        }
        else
        {
            g_turn_preview_slewed_us = g_turn_preview_target_us;
            g_turn_preview_slew_active = 0U;
        }
        s_turn_preview_prev = g_turn_preview_slewed_us;

        hold_pwm += AppRoundFloatToI32(g_turn_preview_slewed_us);
    }
#endif

#if TURN_INTEGRAL_HANDOVER_ENABLED != 0U
    /*
     * 转弯积分交接：入弯/出弯检测。
     *
     * 入弯（IDLE→ACTIVE）：预瞄激活时保存当前积分作为基准。
     * 出弯（ACTIVE→EXITING）：turn 结束后，组合条件连续确认 60 ms。
     * EXITING 期间不回到 ACTIVE，只在积分回到基准后切回 IDLE。
     */
    {
        bool turn_active = false;
        float yaw_magnitude;

#if TURN_PREVIEW_ENABLED != 0U
        turn_active = (g_turn_preview_target_us != 0.0f);
#else
        {
            float turn_mag =
                (float)TURN_PREVIEW_RIGHT_SIGN *
                (float)g_motion_measurement.turn_command;
            turn_active = (turn_mag > g_turn_preview_start);
        }
#endif

        yaw_magnitude =
            (float)g_motion_measurement.yaw_rate_dps10 * 0.1f;
        if (yaw_magnitude < 0.0f)
        {
            yaw_magnitude = -yaw_magnitude;
        }

        if ((s_turn_i_state == 0U) && turn_active)
        {
            s_turn_i_entry_us = g_ball_controller.i_term_us;
            s_turn_i_state = 1U;
            s_turn_exit_confirm_count = 0U;
        }

        if (s_turn_i_state == 1U)
        {
            if (turn_active)
            {
                s_turn_exit_confirm_count = 0U;
                s_turn_idle_timeout = 0U;
            }
            else
            {
                bool exit_cond = false;

                if ((s_prev_motion_state == 4U) &&
                    ((g_motion_measurement.motion_state == 2U) ||
                     (g_motion_measurement.motion_state == 5U)))
                {
                    exit_cond = true;
                }

                if (yaw_magnitude < s_prev_yaw_magnitude)
                {
                    exit_cond = true;
                }

                if (exit_cond)
                {
                    s_turn_exit_confirm_count++;
                    if (s_turn_exit_confirm_count >= 3U)
                    {
                        s_turn_i_state = 2U;
                    }
                }
                else if (s_turn_exit_confirm_count > 0U)
                {
                    s_turn_exit_confirm_count--;
                }

                /* 500 ms 超时回 IDLE，避免短弯/误触发后卡在 ACTIVE */
                s_turn_idle_timeout++;
                if (s_turn_idle_timeout > 25U)
                {
                    s_turn_i_state = 0U;
                    g_turn_i_state = 0U;
                }
            }
        }

        s_prev_motion_state = g_motion_measurement.motion_state;
        s_prev_yaw_magnitude = yaw_magnitude;
        s_turn_active_last = turn_active;

        g_turn_i_entry_us = s_turn_i_entry_us;
        g_turn_i_state = s_turn_i_state;
    }
#endif

    g_effective_hold_pwm_us = AppClampControlPwm(hold_pwm);

    g_effective_hold_pwm_us = BallController_SetEquilibriumPulseUs(
        &g_ball_controller,
        g_effective_hold_pwm_us
    );

    effective_kp = g_kp;
    effective_ki = g_ki;
    effective_kv = g_kv;
#if TASK3_SEQUENCE_ENABLED != 0U
    /*
     * 第三题运行和完成后的最终保持阶段使用专用参数。
     * IDLE及TIMEOUT人工接管阶段继续使用普通闭环参数。
     */
    if ((Task3Sequence_GetState(&s_task3_sequence) ==
         TASK3_SEQUENCE_RUNNING) ||
        (Task3Sequence_GetState(&s_task3_sequence) ==
         TASK3_SEQUENCE_COMPLETE))
    {
        effective_kp = g_task3_kp;
        effective_ki = g_task3_ki;
        effective_kv = g_task3_kv;
    }
#endif

    /*
     * 没有新帧但尚未超时：仍以50 Hz使用最近有效状态计算P、D并维持输出。
     * I项只在AcceptMeasurement接受新帧后按真实dt更新，不会对旧帧重复积分。
     */
    if (g_guard_state == CONTROL_GUARD_READY)
    {
        integral_before_control = g_ball_controller.i_term_us;
        control_ran = BallController_StepPid(
            &g_ball_controller,
            effective_kp,
            effective_ki,
            effective_kv,
            g_dir
        ) ? 1U : 0U;

        if ((integral_before_control != 0.0f) &&
            (g_ball_controller.i_term_us == 0.0f) &&
            (effective_ki <= 0.0f))
        {
            Debug_RecordIntegralReset(6U);
        }

        /* PID 反馈链路 Watch 变量（StepPid 刚返回，值最新） */
        g_error_px = (float)g_ball_controller.error_px;
        g_p_term_raw_us = g_ball_controller.p_term_us;
        g_i_term_raw_us = g_ball_controller.i_term_us;
        g_d_term_raw_us = g_ball_controller.d_term_us;
        g_pid_sum_raw_us =
            g_ball_controller.p_term_us +
            g_ball_controller.i_term_us +
            g_ball_controller.d_term_us;
        g_pid_sum_directed_us =
            (float)g_dir * g_pid_sum_raw_us;
        g_control_offset_us = g_ball_controller.control_offset_us;

#if TURN_INTEGRAL_HANDOVER_ENABLED != 0U
        /*
         * 转弯积分交接：
         * ACTIVE(1) — 限制积分相对于入弯基准的变化范围。
         * EXITING(2) — 冻结正常积分，以固定步长拉回入弯基准。
         */
        if (s_turn_i_state == 1U)
        {
            float i_min = s_turn_i_entry_us - g_turn_i_delta_neg_us;
            float i_max = s_turn_i_entry_us + g_turn_i_delta_pos_us;
            if (g_ball_controller.i_term_us > i_max)
            {
                g_ball_controller.i_term_us = i_max;
            }
            if (g_ball_controller.i_term_us < i_min)
            {
                g_ball_controller.i_term_us = i_min;
            }
        }
        else if (s_turn_i_state == 2U)
        {
            float i = integral_before_control;
            float target = s_turn_i_entry_us;
            float step = g_turn_exit_i_step_us;

            if (i > target)
            {
                i -= step;
                if (i < target) { i = target; }
            }
            else if (i < target)
            {
                i += step;
                if (i > target) { i = target; }
            }
            g_ball_controller.i_term_us = i;

            if (i == target)
            {
                s_turn_i_state = 0U;
                g_turn_i_state = 0U;
            }
        }

        /* 更新 Watch/遥测所用值，反映交接后的真实积分 */
        g_i_term_raw_us = g_ball_controller.i_term_us;
#endif
    }
    else if (s_lost_grace_active)
    {
        /*
         * 丢球宽限期内：不运行PID步进，不清零积分和速度历史。
         * 舵机目标由下方的宽限期分支保持。
         */
        control_ran = false;
    }
    else
    {
        if (g_ball_controller.i_term_us != 0.0f)
        {
            /*
             * guard枚举0～4分别加1，映射为诊断原因1～5。
             * invalid dt已在测量处理处记录，内部Reset后I已经为0，不会重复计数。
             */
            Debug_RecordIntegralReset((uint8_t)g_guard_state + 1U);
        }
        BallController_Reset(&g_ball_controller);
    }

    if (g_manual != 0U)
    {
        ServoOutput_SetTargetPulseUs(g_manual_us);
    }
    else if ((g_guard_state == CONTROL_GUARD_READY) &&
             (g_apply != 0U) &&
             control_ran)
    {
        ServoOutput_SetTargetPulseUs(
            BallController_GetTargetPulseUs(&g_ball_controller)
        );
    }
    else if (s_lost_grace_active)
    {
        /*
         * 丢球宽限期内保持最后一次有效舵机目标，不跳回中位。
         * 避免单帧丢球造成最高约172 us的目标跳变。
         */
        ServoOutput_SetTargetPulseUs(s_last_valid_target_us);
    }
    else
    {
        ServoOutput_SetNeutral();
    }

    /* 最终输出链路 Watch 变量 */
    {
        uint16_t servo_target;
        uint16_t servo_current;

        g_hold_pwm_effective_us = (float)g_effective_hold_pwm_us;
        g_feedforward_total_us = g_af_slewed_us + g_yf_slewed_us;

        servo_target = ServoOutput_GetTargetPulseUs();
        servo_current = ServoOutput_GetCurrentPulseUs();
        g_servo_target_debug_us = (float)servo_target;
        g_servo_current_debug_us = (float)servo_current;
        g_servo_tracking_error_us =
            (float)servo_target - (float)servo_current;

        g_servo_target_saturated = g_ball_controller.saturated ? 1U : 0U;

        if (servo_target != servo_current)
        {
            g_servo_slew_active = 1U;
        }
        else
        {
            g_servo_slew_active = 0U;
        }

        /*
         * servo_prelimit_us = hold_pwm (含 AF+YF, 已限幅到equilibrium)
         *                     + control_offset_us
         * 使用 StepPid 内部计算：equilibrium + round(control_offset)
         * 重构公式：prelimit = effective_hold_pwm + round(control_offset)
         */
        {
            int32_t prelimit_raw;
            prelimit_raw =
                (int32_t)g_effective_hold_pwm_us +
                AppRoundFloatToI32(g_ball_controller.control_offset_us);
            g_servo_prelimit_us = (float)prelimit_raw;

            /* 一致性检查 */
            g_control_decomposition_error_us =
                g_servo_prelimit_us - g_servo_target_debug_us;
        }

        if ((g_control_decomposition_error_us > 2.0f) ||
            (g_control_decomposition_error_us < -2.0f))
        {
            g_decomposition_error_count++;
        }
    }

    g_dbg.tick = g_control_tick_count;
    g_dbg.guard = (uint8_t)g_guard_state;
    g_dbg.x = g_measurement.ball_x;
    g_dbg.err = g_ball_controller.error_px;
    g_dbg.vel = g_ball_controller.velocity_px_s;
    g_dbg.p = g_ball_controller.p_term_us;
    g_dbg.i = g_ball_controller.i_term_us;
    g_dbg.d = g_ball_controller.d_term_us;
    g_dbg.out = g_ball_controller.control_offset_us;
    g_dbg.acc = g_motion_measurement.acc_track_mg;
    g_dbg.af = g_af;
    g_dbg.yaw =
        (float)g_motion_measurement.yaw_rate_dps10 * 0.1f;
    g_dbg.yf = g_yf;
    g_dbg.lost_age_ms = g_lost_age_ms;
    g_dbg.lost_grace = g_lost_grace_active;
    g_dbg.lost_recovery = g_lost_recovery_count;
    g_dbg.pwm_target = ServoOutput_GetTargetPulseUs();
    g_dbg.pwm_now = ServoOutput_GetCurrentPulseUs();

    if (g_justfloat_tx_started != 0U)
    {
        /*
         * JustFloat 通道顺序在 app_justfloat_tx.h 中固定。
         * 这里处于 50 Hz 控制节拍，只发布一份最新快照；不会在 SysTick 中发送
         * 100 字节，也不会因为蓝牙暂时断连而阻塞 PID。
         */
        justfloat_sample.time_s = (float)now_ms * 0.001f;
        justfloat_sample.ball_x = (float)g_measurement.ball_x;
        justfloat_sample.target_x = (float)g_target_x;
        justfloat_sample.error_px = (float)g_ball_controller.error_px;
        justfloat_sample.velocity_px_s = g_ball_controller.velocity_px_s;
        justfloat_sample.p_term_us = g_ball_controller.p_term_us;
        justfloat_sample.i_term_us = g_ball_controller.i_term_us;
        justfloat_sample.d_term_us = g_ball_controller.d_term_us;
        justfloat_sample.control_out_us =
            g_ball_controller.control_offset_us;
        justfloat_sample.pwm_target_us = (float)g_dbg.pwm_target;
        justfloat_sample.pwm_now_us = (float)g_dbg.pwm_now;
        justfloat_sample.guard = (float)g_guard_state;
        justfloat_sample.motion_link_valid =
            (float)g_motion_link_valid;
        justfloat_sample.motion_state =
            (float)g_motion_measurement.motion_state;
        justfloat_sample.motion_flags =
            (float)g_motion_measurement.flags;
        justfloat_sample.acc_track_mg =
            (float)g_motion_measurement.acc_track_mg;
        justfloat_sample.yaw_rate_dps =
            (float)g_motion_measurement.yaw_rate_dps10 * 0.1f;
        justfloat_sample.vibration_mg =
            (float)g_motion_measurement.vibration_level_mg;
        justfloat_sample.line_error =
            (float)g_motion_measurement.line_error;
        justfloat_sample.left_speed =
            (float)g_motion_measurement.left_speed;
        justfloat_sample.right_speed =
            (float)g_motion_measurement.right_speed;
        justfloat_sample.turn_command =
            (float)g_motion_measurement.turn_command;
        justfloat_sample.motion_sequence =
            (float)g_motion_measurement.sequence;
        justfloat_sample.motion_age_ms =
            (float)g_motion_packet_age_ms;
        AppJustFloatTx_Publish(&justfloat_sample);
    }

    if (g_telemetry_tx_started != 0U)
    {
        /*
         * The 50 Hz control ISR only publishes a snapshot. The main loop
         * starts the non-blocking UART transfer, so telemetry can never wait
         * inside the control path.
         */
        telemetry_sample.tick_ms = now_ms;
        telemetry_sample.vision_frame_id = g_measurement.frame_id;
        telemetry_sample.ball_x = g_measurement.ball_x;
        telemetry_sample.error_px = g_ball_controller.error_px;
        telemetry_sample.velocity_px_s =
            Telemetry_RoundFloatToI16(g_ball_controller.velocity_px_s);
        telemetry_sample.p_term_us =
            Telemetry_RoundFloatToI16(g_ball_controller.p_term_us);
        telemetry_sample.i_term_us =
            Telemetry_RoundFloatToI16(g_ball_controller.i_term_us);
        telemetry_sample.d_term_us =
            Telemetry_RoundFloatToI16(g_ball_controller.d_term_us);
        telemetry_sample.control_offset_us =
            Telemetry_RoundFloatToI16(g_ball_controller.control_offset_us);
        telemetry_sample.servo_target_us = g_dbg.pwm_target;
        telemetry_sample.servo_current_us = g_dbg.pwm_now;
        telemetry_sample.guard_state = (uint8_t)g_guard_state;
        telemetry_sample.measurement_status = g_dbg.meas_status;
        telemetry_sample.flags = 0U;
        if (g_apply != 0U)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_APPLY_OUTPUT;
        }
        if (g_manual != 0U)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_MANUAL_MODE;
        }
        if (g_measurement.ball_valid)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_BALL_VALID;
        }
        if (g_measurement.ball_safe)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_BALL_SAFE;
        }
        if (control_ran)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_CONTROL_RAN;
        }
        if (g_ball_controller.saturated)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_OUTPUT_SATURATED;
        }
        if (g_servo_pwm_started != 0U)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_SERVO_STARTED;
        }
        if (rx_snapshot.new_packet)
        {
            telemetry_sample.flags |= TELEMETRY_FLAG_NEW_VISION_PACKET;
        }
        telemetry_sample.motion_state = g_motion_measurement.motion_state;
        telemetry_sample.motion_flags = g_motion_measurement.flags;
        telemetry_sample.motion_link_valid = g_motion_link_valid;
        telemetry_sample.acc_track_mg = g_motion_measurement.acc_track_mg;
        telemetry_sample.yaw_rate_dps10 = g_motion_measurement.yaw_rate_dps10;
        telemetry_sample.vibration_level_mg =
            g_motion_measurement.vibration_level_mg;
        telemetry_sample.line_error = g_motion_measurement.line_error;
        telemetry_sample.left_speed = g_motion_measurement.left_speed;
        telemetry_sample.right_speed = g_motion_measurement.right_speed;
        telemetry_sample.turn_command = g_motion_measurement.turn_command;
        telemetry_sample.motion_sequence = g_motion_measurement.sequence;
        telemetry_sample.motion_age_ms =
            (g_motion_packet_age_ms > 65535U)
                ? 65535U
                : (uint16_t)g_motion_packet_age_ms;
        telemetry_sample.lost_grace = g_lost_grace_active;
        telemetry_sample.lost_age_ms =
            (g_lost_age_ms > 65535U) ? 65535U : (uint16_t)g_lost_age_ms;
        telemetry_sample.lost_recovery =
            (g_lost_recovery_count > 65535U)
                ? 65535U
                : (uint16_t)g_lost_recovery_count;
#if BALL_TELEMETRY_CONTROL_DECOMPOSITION != 0U
        /* V3 控制量分解扩展字段 */
        telemetry_sample.pid_sum_raw_us =
            Telemetry_RoundFloatToI16(g_pid_sum_raw_us);
        telemetry_sample.pid_sum_directed_us =
            Telemetry_RoundFloatToI16(g_pid_sum_directed_us);
        telemetry_sample.acc_filtered_mg =
            Telemetry_RoundFloatToI16(g_acc_filt_mg);
        telemetry_sample.af_raw_us =
            Telemetry_RoundFloatToI16(g_af_raw_us);
        telemetry_sample.af_clamped_us =
            Telemetry_RoundFloatToI16(g_af_clamped_us);
        telemetry_sample.af_slewed_us =
            Telemetry_RoundFloatToI16(g_af_slewed_us);
        telemetry_sample.yaw_raw_dps10 =
            Telemetry_RoundFloatToI16(g_yaw_raw_dps * 10.0f);
        telemetry_sample.speed_average =
            Telemetry_RoundFloatToI16(g_speed_average);
        telemetry_sample.speed_scale_x1000 =
            (uint16_t)(g_speed_scale * 1000.0f + 0.5f);
        telemetry_sample.yf_raw_us =
            Telemetry_RoundFloatToI16(g_yf_raw_us);
        telemetry_sample.yf_clamped_us =
            Telemetry_RoundFloatToI16(g_yf_clamped_us);
        telemetry_sample.yf_slewed_us =
            Telemetry_RoundFloatToI16(g_yf_slewed_us);
        telemetry_sample.hold_pwm_effective_us = g_effective_hold_pwm_us;
        telemetry_sample.feedforward_total_us =
            Telemetry_RoundFloatToI16(g_feedforward_total_us);
        telemetry_sample.servo_prelimit_us =
            Telemetry_RoundFloatToI16(g_servo_prelimit_us);
        telemetry_sample.servo_flags = 0U;
        if (g_servo_target_saturated != 0U)
        {
            telemetry_sample.servo_flags |=
                TELEMETRY_SERVO_FLAG_TARGET_SATURATED;
        }
        if (g_servo_slew_active != 0U)
        {
            telemetry_sample.servo_flags |=
                TELEMETRY_SERVO_FLAG_SLEW_ACTIVE;
        }
        if (g_af_slew_active != 0U)
        {
            telemetry_sample.servo_flags |=
                TELEMETRY_SERVO_FLAG_AF_SLEW_ACTIVE;
        }
        if (g_yf_slew_active != 0U)
        {
            telemetry_sample.servo_flags |=
                TELEMETRY_SERVO_FLAG_YF_SLEW_ACTIVE;
        }
        telemetry_sample.turn_scale_x1000 =
            (uint16_t)(g_turn_scale * 1000.0f + 0.5f);
        telemetry_sample.yaw_handover_x1000 =
            (uint16_t)(g_yaw_handover * 1000.0f + 0.5f);
        telemetry_sample.turn_preview_raw_us =
            Telemetry_RoundFloatToI16(g_turn_preview_raw_us);
        telemetry_sample.turn_preview_slewed_us =
            Telemetry_RoundFloatToI16(g_turn_preview_slewed_us);
#endif
        AppTelemetryTx_Publish(&telemetry_sample);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(){

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	/*
	 * 启动 USART1 单字节中断接收。
	 * 当前即使没有连接 K230，也应当能够成功启动接收并保持 UART_TIMEOUT。
	 */
	if (!AppUartRx_Init(&huart1))
	{
		/* 初始化失败时进入 CubeMX 的统一错误处理，绝不继续假装通信正常。 */
		Error_Handler();
	}
	g_uart_rx_started = 1U;
	if ((APP_TELEMETRY_ENABLED != 0U) &&
		AppTelemetryTx_Init(&huart1))
	{
		g_telemetry_tx_started = 1U;
	}

	/*
	 * USART2 只接收 CH32V307 的运动帧。
	 * 即使暂时没接车辆线，中断接收也能正常启动，链路状态保持 invalid。
	 */
	if ((APP_MOTION_RX_ENABLED != 0U) &&
		AppMotionRx_Init(&huart2))
	{
		g_motion_rx_started = 1U;
	}

	/*
	 * USART3 只向 HC-06 输出 VOFA+ JustFloat。
	 * HC-06 必须已经用 AT+BAUD8 配置为 115200，否则电脑端看不到有效曲线。
	 */
	if ((APP_JUSTFLOAT_ENABLED != 0U) &&
		AppJustFloatTx_Init(&huart3))
	{
		g_justfloat_tx_started = 1U;
	}

	/*
	 * 先写入配置文件中的暂定中位，再启动 TIM2_CH1 硬件PWM。
	 * 当前没有连接舵机时，只会在 PA0 输出约 333 Hz 的中位测试波形。
	 */
	if (!ServoOutput_Init(&htim2, TIM_CHANNEL_1))
	{
		Error_Handler();
	}
	g_servo_pwm_started = 1U;
	BallController_Init(&g_ball_controller);
	Task3Sequence_Init(&s_task3_sequence);
	g_dbg.meas_status = 255U;
	g_control_runtime_started = 1U;

	/*
	 * IMU初始化失败不会进入Error_Handler，也不会阻止原有视觉闭环运行。
	 * 这样即使暂时没接ICM20602，也能在Watch中根据状态码排查接线。
	 * 修正接线后需要复位STM32，让初始化流程重新执行一次。
	 */
	g_imu_init_ok = ICM20602_Init() ? 1U : 0U;
	g_imu_status = (uint8_t)ICM20602_GetStatus();
	g_imu_who_am_i = ICM20602_GetWhoAmI();
	g_imu_address = ICM20602_GetAddress();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*
		 * 将接收统计量镜像到全局变量，便于不接串口线时在 Watch 中观察。
		 * 没有 K230 连线时，两个计数都应保持为 0。
		 */
		g_uart_valid_packet_count = AppUartRx_GetValidPacketCount();
		g_uart_error_count = AppUartRx_GetUartErrorCount();
		g_protocol_error_count = AppUartRx_GetProtocolErrorCount();
		g_motion_valid_packet_count =
			AppMotionRx_GetValidPacketCount();
		g_motion_uart_error_count =
			AppMotionRx_GetUartErrorCount();
		g_motion_protocol_error_count =
			AppMotionRx_GetProtocolErrorCount();
		AppImu_Poll();
		if (g_telemetry_tx_started != 0U)
		{
			AppTelemetryTx_Poll();
			g_telemetry_sent_count = AppTelemetryTx_GetSentCount();
			g_telemetry_overwrite_count =
				AppTelemetryTx_GetOverwriteCount();
			g_telemetry_error_count = AppTelemetryTx_GetErrorCount();
		}
		if (g_justfloat_tx_started != 0U)
		{
			AppJustFloatTx_Poll();
			g_justfloat_sent_count = AppJustFloatTx_GetSentCount();
			g_justfloat_overwrite_count =
				AppJustFloatTx_GetOverwriteCount();
			g_justfloat_error_count = AppJustFloatTx_GetErrorCount();
		}

		/*
		 * 现在只在 Keil Watch 窗口观察 g_guard_state 和 g_measurement。
		 * 不要在这里添加舵机 PWM。
		 */
			}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3002;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1520;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @note  PA2/PA3, 115200 8N1；PA3 接收 CH32V307 的车辆运动帧。
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @note  PB10/PB11, 115200 8N1；PB10 向 HC-06 输出 JustFloat。
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
