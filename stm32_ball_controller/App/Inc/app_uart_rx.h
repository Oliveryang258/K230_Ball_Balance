#ifndef APP_UART_RX_H
#define APP_UART_RX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "vision_protocol.h"

/* 在 MX_USART1_UART_Init() 之后调用一次。 */
bool AppUartRx_Init(UART_HandleTypeDef *huart);

/* 从 CubeMX 的 HAL_UART_RxCpltCallback() 中调用。 */
void AppUartRx_OnRxComplete(UART_HandleTypeDef *huart);

/* 从 HAL_UART_ErrorCallback() 中调用，用于错误后重新启动接收。 */
void AppUartRx_OnError(UART_HandleTypeDef *huart);

/*
 * 主循环调用。只有出现尚未读取的新测量帧时返回 true。
 * 它不会把上一次数据伪装成新数据。
 */
bool AppUartRx_GetLatest(VisionMeasurement *output);

bool AppUartRx_HasPacket(void);
uint32_t AppUartRx_GetLastPacketTick(void);
uint32_t AppUartRx_GetValidPacketCount(void);
uint32_t AppUartRx_GetUartErrorCount(void);

#endif

