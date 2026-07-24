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
volatile ControlGuardState g_guard_state = CONTROL_GUARD_LINK_TIMEOUT;
volatile uint8_t g_uart_rx_started = 0U;
volatile uint32_t g_uart_valid_packet_count = 0U;
volatile uint32_t g_uart_error_count = 0U;

/*
 * 舵机台架调试变量：默认关闭手动测试，默认目标取自配置文件中的暂定中位。
 * 使用 Watch 手动测试前必须取下钢球，并确认连杆没有顶死或明显预紧。
 */
volatile uint8_t g_servo_pwm_started = 0U;
volatile uint8_t g_servo_manual_test_enabled = 0U;
volatile uint16_t g_servo_manual_target_us = SERVO_PWM_NEUTRAL_US;
volatile uint16_t g_servo_current_pulse_us = SERVO_PWM_NEUTRAL_US;
volatile uint16_t g_servo_target_pulse_us = SERVO_PWM_NEUTRAL_US;

/*
 * 自动控制采用“两级开关”，上电时两级都为 0：
 * enabled 只允许计算，apply_output 才允许实际驱动舵机。
 * 这样可以先在 Watch 中检查方向、增益和限幅，再接通闭环输出。
 */
BallController g_ball_controller;
volatile uint8_t g_ball_control_enabled = 0U;
volatile uint8_t g_ball_control_apply_output = 0U;
volatile float g_ball_control_kp_us_per_px = 0.0f;
volatile float g_ball_control_kv_us_per_px_s = 0.0f;
volatile int8_t g_ball_control_direction = 1;

/* 以下是 Watch 只读观察量，不应手动改写。 */
volatile uint8_t g_ball_control_updated = 0U;
volatile uint8_t g_ball_control_saturated = 0U;
volatile int16_t g_ball_control_error_px = 0;
volatile float g_ball_control_velocity_px_s = 0.0f;
volatile float g_ball_control_p_term_us = 0.0f;
volatile float g_ball_control_d_term_us = 0.0f;
volatile float g_ball_control_offset_us = 0.0f;
volatile uint16_t g_ball_control_computed_target_us = SERVO_PWM_NEUTRAL_US;
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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    AppUartRx_OnError(huart);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  bool new_measurement;
  uint32_t now_ms;
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
	 * 当前即使没有连接 K230，也应当能够成功启动接收并保持 LINK_TIMEOUT。
	 */
	if (!AppUartRx_Init(&huart1))
	{
		/* 初始化失败时进入 CubeMX 的统一错误处理，绝不继续假装通信正常。 */
		Error_Handler();
	}
	g_uart_rx_started = 1U;

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		now_ms = HAL_GetTick();
		new_measurement = AppUartRx_GetLatest(&g_measurement);
		if (new_measurement)
		{
				/* 收到一帧格式、版本、校验都正确的新数据。 */
		}

		g_guard_state = ControlGuard_Evaluate(
				&g_measurement,
				AppUartRx_HasPacket(),
				AppUartRx_GetLastPacketTick(),
				now_ms
		);

		/*
		 * 将接收统计量镜像到全局变量，便于不接串口线时在 Watch 中观察。
		 * 没有 K230 连线时，两个计数都应保持为 0。
		 */
		g_uart_valid_packet_count = AppUartRx_GetValidPacketCount();
		g_uart_error_count = AppUartRx_GetUartErrorCount();

		/*
		 * 只有 Guard READY 且计算开关打开时才处理新帧。
		 * Guard 一旦失败就清除速度历史，避免恢复通信后产生速度尖峰。
		 */
		g_ball_control_updated = 0U;
		if ((g_guard_state == CONTROL_GUARD_READY) &&
		    (g_ball_control_enabled != 0U))
		{
			if (new_measurement)
			{
				g_ball_control_updated = BallController_Update(
					&g_ball_controller,
					&g_measurement,
					now_ms,
					g_ball_control_kp_us_per_px,
					g_ball_control_kv_us_per_px_s,
					g_ball_control_direction
				) ? 1U : 0U;
			}
		}
		else
		{
			BallController_Reset(&g_ball_controller);
		}

		/*
		 * 默认永远回到中位。只有显式打开台架测试开关后，才接受 Watch 中
		 * 的手动目标；servo_output 内部还会按照配置文件执行强制限幅。
		 */
		if (g_servo_manual_test_enabled != 0U)
		{
			ServoOutput_SetTargetPulseUs(g_servo_manual_target_us);
		}
		else if ((g_guard_state == CONTROL_GUARD_READY) &&
		         (g_ball_control_enabled != 0U) &&
		         (g_ball_control_apply_output != 0U))
		{
			ServoOutput_SetTargetPulseUs(
				BallController_GetTargetPulseUs(&g_ball_controller)
			);
		}
		else
		{
			ServoOutput_SetNeutral();
		}
		ServoOutput_Process(now_ms);

		/* 将模块内部状态镜像到全局变量，方便在 Keil Watch 中观察。 */
		g_servo_current_pulse_us = ServoOutput_GetCurrentPulseUs();
		g_servo_target_pulse_us = ServoOutput_GetTargetPulseUs();

		/* 把 PD 内部量镜像到全局变量，方便在 Keil Watch 中连续观察。 */
		g_ball_control_saturated = g_ball_controller.saturated ? 1U : 0U;
		g_ball_control_error_px = g_ball_controller.error_px;
		g_ball_control_velocity_px_s = g_ball_controller.velocity_px_s;
		g_ball_control_p_term_us = g_ball_controller.p_term_us;
		g_ball_control_d_term_us = g_ball_controller.d_term_us;
		g_ball_control_offset_us = g_ball_controller.control_offset_us;
		g_ball_control_computed_target_us =
			BallController_GetTargetPulseUs(&g_ball_controller);

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
