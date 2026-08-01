#ifndef CH32_COMMAND_PROTOCOL_H
#define CH32_COMMAND_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * CH32V304 -> STM32F103 目标位置命令帧（TP帧）V1。
 *
 * 固定 12 字节，小端序：
 *   0..1   帧头 0x54 0x50（ASCII "TP"）
 *   2      协议版本 1
 *   3      帧长度 12
 *   4..5   参数帧序号
 *   6      题号（拨码题目选择，2~6）
 *   7      有效标志
 *   8..9   指定位置，单位 0.1 cm（int16，小端；非慢速指定题号为0）
 *   10..11 CRC16-CCITT-FALSE，低字节在前，不含CRC本身
 *
 * CH32 每 20 ms 重发一次。与运动帧共用 USART2，靠帧头区分，
 * STM32 只需按 0x54 0x50 识别，再按版本/长度/CRC校验。
 */
#define CH32_COMMAND_HEADER_0       0x54U
#define CH32_COMMAND_HEADER_1       0x50U
#define CH32_COMMAND_VERSION        0x01U
#define CH32_COMMAND_PACKET_SIZE    12U

typedef enum
{
    CH32_COMMAND_PARSE_INCOMPLETE = 0,
    CH32_COMMAND_PARSE_VALID,
    CH32_COMMAND_PARSE_BAD_VERSION,
    CH32_COMMAND_PARSE_BAD_LENGTH,
    CH32_COMMAND_PARSE_BAD_CRC
} Ch32CommandParseResult;

typedef struct
{
    uint16_t sequence;
    uint8_t mode;
    uint8_t valid;
    int16_t position_01cm;
} Ch32CommandFrame;

typedef struct
{
    uint8_t buffer[CH32_COMMAND_PACKET_SIZE];
    uint8_t index;
} Ch32CommandParser;

void Ch32CommandParser_Init(Ch32CommandParser *parser);

Ch32CommandParseResult Ch32CommandParser_PushByte(
    Ch32CommandParser *parser,
    uint8_t byte,
    Ch32CommandFrame *output);

#endif
