#ifndef APP_UART_RX_H
#define APP_UART_RX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "vision_protocol.h"

/* 在MX_USART1_UART_Init()之后调用一次。 */
bool AppUartRx_Init(UART_HandleTypeDef *huart);

/* 从CubeMX的HAL_UART_RxCpltCallback()中调用。 */
void AppUartRx_OnRxComplete(UART_HandleTypeDef *huart);

/* 从HAL_UART_ErrorCallback()中调用，用于错误后重新启动接收。 */
void AppUartRx_OnError(UART_HandleTypeDef *huart);

/*
 * USART1中断发布给50 Hz控制任务的一致性快照。
 *
 * measurement、接收时间戳、new_packet和统计计数必须来自同一个瞬间，
 * 否则控制任务可能组合出“旧测量 + 新时间戳”。TakeSnapshot内部只在复制
 * 这些字段和清除new_packet期间短暂关中断，返回后PD运算全程保持中断开启。
 */
typedef struct
{
    VisionMeasurement measurement;
    bool has_packet;
    bool new_packet;
    uint32_t last_packet_tick;
    uint32_t valid_packet_count;
    uint32_t uart_error_count;
    uint32_t protocol_error_count;
} AppUartRxSnapshot;

bool AppUartRx_TakeSnapshot(AppUartRxSnapshot *snapshot);

/*
 * 旧接口仅为兼容已有调用和测试而保留。
 * 新的实时控制路径必须使用AppUartRx_TakeSnapshot()，避免分开读取字段。
 */
bool AppUartRx_GetLatest(VisionMeasurement *output);
bool AppUartRx_HasPacket(void);
uint32_t AppUartRx_GetLastPacketTick(void);
uint32_t AppUartRx_GetValidPacketCount(void);
uint32_t AppUartRx_GetUartErrorCount(void);
uint32_t AppUartRx_GetProtocolErrorCount(void);

#endif
