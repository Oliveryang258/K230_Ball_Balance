#ifndef CH32_MOTION_PROTOCOL_H
#define CH32_MOTION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * CH32V307 -> STM32F103 车辆运动遥测 V1。
 *
 * 固定 28 字节，小端序：
 *   0..1   帧头 0x43 0x48（ASCII "CH"）
 *   2      协议版本 1
 *   3      帧长度 28
 *   4..5   sequence
 *   6..9   CH32 本地毫秒时间戳
 *   10     motion_state
 *   11     flags
 *   12..25 7 个车辆/IMU int16 或 uint16 字段
 *   26..27 CRC16-CCITT-FALSE，低字节在前
 */
#define CH32_MOTION_HEADER_0        0x43U
#define CH32_MOTION_HEADER_1        0x48U
#define CH32_MOTION_VERSION         0x01U
#define CH32_MOTION_PACKET_SIZE     28U

/*
 * motion_state和flags的数值由CH32V307发送端协议固定。
 * 在STM32接收端也给它们命名，避免控制代码里直接出现0、1、255等魔法数字。
 */
#define CH32_MOTION_STATE_STOPPED       0U
#define CH32_MOTION_STATE_STARTING      1U
#define CH32_MOTION_STATE_STRAIGHT      2U
#define CH32_MOTION_STATE_TURN_LEFT     3U
#define CH32_MOTION_STATE_TURN_RIGHT    4U
#define CH32_MOTION_STATE_STOPPING      5U
#define CH32_MOTION_STATE_FAULT         255U

#define CH32_MOTION_FLAG_IMU_VALID      0x01U
#define CH32_MOTION_FLAG_START_EVENT    0x10U
#define CH32_MOTION_FLAG_FAULT          0x80U

typedef enum
{
    CH32_MOTION_PARSE_INCOMPLETE = 0,
    CH32_MOTION_PARSE_VALID,
    CH32_MOTION_PARSE_BAD_VERSION,
    CH32_MOTION_PARSE_BAD_LENGTH,
    CH32_MOTION_PARSE_BAD_CRC
} Ch32MotionParseResult;

typedef struct
{
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint8_t motion_state;
    uint8_t flags;
    int16_t acc_track_mg;
    int16_t yaw_rate_dps10;
    uint16_t vibration_level_mg;
    int16_t line_error;
    int16_t left_speed;
    int16_t right_speed;
    int16_t turn_command;
} Ch32MotionMeasurement;

typedef struct
{
    uint8_t buffer[CH32_MOTION_PACKET_SIZE];
    uint8_t index;
} Ch32MotionParser;

void Ch32MotionParser_Init(Ch32MotionParser *parser);

Ch32MotionParseResult Ch32MotionParser_PushByte(
    Ch32MotionParser *parser,
    uint8_t byte,
    Ch32MotionMeasurement *output
);

uint16_t Ch32MotionProtocol_Crc16CcittFalse(
    const uint8_t *data,
    uint16_t length
);

#endif
