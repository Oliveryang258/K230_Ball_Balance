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
#include "app_config.h"
#include "ball_controller.h"
#include "control_guard.h"
#include "servo_output.h"
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
 * 调参时只需要添加g_kp、g_ki、g_kv、g_dir和g_apply。
 */
volatile uint8_t g_apply = BALL_CONTROL_DEFAULT_APPLY_OUTPUT;
volatile float g_kp = BALL_CONTROL_DEFAULT_KP_US_PER_PX;
volatile float g_ki = BALL_CONTROL_DEFAULT_KI_US_PER_PX_S;
volatile float g_kv = BALL_CONTROL_DEFAULT_KV_US_PER_PX_S;
volatile int8_t g_dir = BALL_CONTROL_DEFAULT_DIRECTION;

/*
 * Keil Watch只读调试结构体。
 * Watch中只添加g_dbg并展开，即可一次看到测量、控制和PWM状态。
 */
typedef struct
{
    uint32_t tick;          /* 20 ms控制任务累计执行次数 */
    uint8_t guard;          /* 当前保护状态，0～5 */
    uint8_t meas_status;    /* 0接受、1重复、2无效、3 dt异常、255无新帧 */
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
} ControlDebug;

volatile ControlDebug g_dbg = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    AppUartRx_OnRxComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    AppTelemetryTx_OnTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    AppTelemetryTx_OnError(huart);
    AppUartRx_OnError(huart);
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
 * 固定1 ms入口、20 ms执行一次的控制任务。
 * USART ISR只发布数据；本函数先做一致性快照，再在中断保持开启的情况下
 * 完成guard、速度估计、位置PID和舵机slew目标更新。
 */
void AppControl_On1msTick(void)
{
    static uint16_t control_divider_ms = 0U;
    static uint32_t previous_protocol_error_count = 0U;
    static uint32_t previous_uart_error_count = 0U;
    AppUartRxSnapshot rx_snapshot;
    BallMeasurementResult measurement_result = BALL_MEAS_INVALID;
    bool protocol_error_event;
    bool invalid_dt_event = false;
    bool control_ran = false;
    float integral_before_measurement;
    float integral_before_control;
    uint32_t now_ms;
    TelemetrySample telemetry_sample;

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
     * TakeSnapshot仅在复制共享字段期间短暂关闭中断。
     * 返回后多字段测量和它的接收tick属于同一发布版本。
     */
    (void)AppUartRx_TakeSnapshot(&rx_snapshot);
    g_measurement = rx_snapshot.measurement;

    protocol_error_event =
        (rx_snapshot.protocol_error_count != previous_protocol_error_count) ||
        (rx_snapshot.uart_error_count != previous_uart_error_count);
    previous_protocol_error_count = rx_snapshot.protocol_error_count;
    previous_uart_error_count = rx_snapshot.uart_error_count;

    g_dbg.meas_status = 255U;

    g_guard_state = ControlGuard_Evaluate(
        &g_measurement,
        rx_snapshot.has_packet,
        rx_snapshot.last_packet_tick,
        now_ms,
        protocol_error_event,
        false
    );

    if ((g_guard_state == CONTROL_GUARD_READY) &&
        rx_snapshot.new_packet)
    {
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

    /*
     * 没有新帧但尚未超时：仍以50 Hz使用最近有效状态计算P、D并维持输出。
     * I项只在AcceptMeasurement接受新帧后按真实dt更新，不会对旧帧重复积分。
     */
    if (g_guard_state == CONTROL_GUARD_READY)
    {
        integral_before_control = g_ball_controller.i_term_us;
        control_ran = BallController_StepPid(
            &g_ball_controller,
            g_kp,
            g_ki,
            g_kv,
            g_dir
        ) ? 1U : 0U;

        if ((integral_before_control != 0.0f) &&
            (g_ball_controller.i_term_us == 0.0f) &&
            (g_ki <= 0.0f))
        {
            Debug_RecordIntegralReset(6U);
        }
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
    else
    {
        ServoOutput_SetNeutral();
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
    g_dbg.pwm_target = ServoOutput_GetTargetPulseUs();
    g_dbg.pwm_now = ServoOutput_GetCurrentPulseUs();

    /*
     * The 50 Hz control ISR only publishes a snapshot. The main loop starts
     * the non-blocking UART transfer, so telemetry can never wait inside the
     * control path.
     */
    telemetry_sample.tick_ms = now_ms;
    telemetry_sample.vision_frame_id = g_measurement.frame_id;
    telemetry_sample.ball_x = g_measurement.ball_x;
    telemetry_sample.error_px = g_measurement.error_px;
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
    AppTelemetryTx_Publish(&telemetry_sample);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
	if (AppTelemetryTx_Init(&huart1))
	{
		g_telemetry_tx_started = 1U;
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
	g_control_runtime_started = 1U;
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
		AppTelemetryTx_Poll();
		g_telemetry_sent_count = AppTelemetryTx_GetSentCount();
		g_telemetry_overwrite_count =
			AppTelemetryTx_GetOverwriteCount();
		g_telemetry_error_count = AppTelemetryTx_GetErrorCount();

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
