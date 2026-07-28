#ifndef APP_TELEMETRY_TX_H
#define APP_TELEMETRY_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "telemetry_protocol.h"

/*
 * Initialize the STM32 -> K230 telemetry path after MX_USART1_UART_Init().
 * RX and TX share USART1 but use the HAL's independent RxState/gState paths.
 */
bool AppTelemetryTx_Init(UART_HandleTypeDef *huart);

/*
 * Publish the newest 50 Hz control snapshot. This is safe to call from SysTick:
 * it only copies a small structure and never starts or waits for a UART transfer.
 */
void AppTelemetryTx_Publish(const TelemetrySample *sample);

/*
 * Call continuously from the main loop. It takes the latest pending snapshot,
 * encodes it, and starts HAL_UART_Transmit_IT(). It never blocks the control ISR.
 */
void AppTelemetryTx_Poll(void);

void AppTelemetryTx_OnTxComplete(UART_HandleTypeDef *huart);
void AppTelemetryTx_OnError(UART_HandleTypeDef *huart);

uint32_t AppTelemetryTx_GetSentCount(void);
uint32_t AppTelemetryTx_GetOverwriteCount(void);
uint32_t AppTelemetryTx_GetErrorCount(void);

#endif
