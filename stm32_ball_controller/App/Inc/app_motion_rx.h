#ifndef APP_MOTION_RX_H
#define APP_MOTION_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "ch32_command_protocol.h"
#include "ch32_motion_protocol.h"
#include "stm32f1xx_hal.h"

typedef struct
{
    Ch32MotionMeasurement measurement;
    bool has_packet;
    bool new_packet;
    uint32_t last_packet_tick;
    uint32_t valid_packet_count;
    uint32_t uart_error_count;
    uint32_t protocol_error_count;

    /*
     * TP命令帧（目标位置/拨码模式号）与运动帧共用USART2，
     * 按帧头各自重同步，互不影响。
     */
    Ch32CommandFrame command;
    bool command_has_packet;
    bool command_new;
    uint32_t command_last_tick;
    uint32_t command_valid_count;
    uint32_t command_error_count;

    /*
     * START_EVENT（BUT2发车，运动帧flags bit4）上升沿在接收中断里
     * 锁存，控制任务快照后消费并清零，短脉冲不会丢失。
     */
    bool start_event_pending;
} AppMotionRxSnapshot;

/* 在 MX_USART2_UART_Init() 之后启动 PA3 的单字节中断接收。 */
bool AppMotionRx_Init(UART_HandleTypeDef *huart);

void AppMotionRx_OnRxComplete(UART_HandleTypeDef *huart);
void AppMotionRx_OnError(UART_HandleTypeDef *huart);

/*
 * USART2 ISR 会更新多个字段，所以读取端必须使用一致性快照。
 * 临界区只覆盖结构体复制，不在关中断期间做 CRC、控制或浮点运算。
 */
bool AppMotionRx_TakeSnapshot(AppMotionRxSnapshot *snapshot);

uint32_t AppMotionRx_GetValidPacketCount(void);
uint32_t AppMotionRx_GetUartErrorCount(void);
uint32_t AppMotionRx_GetProtocolErrorCount(void);

#endif
