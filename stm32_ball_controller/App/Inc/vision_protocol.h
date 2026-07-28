#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_PACKET_HEADER_0  0xAAU
#define VISION_PACKET_HEADER_1  0x55U
#define VISION_PACKET_VERSION   1U
#define VISION_PACKET_SIZE      11U

#define VISION_FLAG_VALID       0x01U
#define VISION_FLAG_SAFE        0x02U

typedef struct
{
    uint16_t frame_id;
    int16_t error_px;
    int16_t ball_x;
    bool ball_valid;
    bool ball_safe;
} VisionMeasurement;

typedef struct
{
    uint8_t buffer[VISION_PACKET_SIZE];
    uint8_t index;
} VisionParser;

typedef enum
{
    VISION_PARSE_INCOMPLETE = 0,
    VISION_PARSE_VALID,
    VISION_PARSE_BAD_VERSION,
    VISION_PARSE_BAD_CHECKSUM
} VisionParseResult;

/* 初始化状态机。 */
void VisionParser_Init(VisionParser *parser);

/*
 * 扩展解析接口：区分未收满、有效帧、版本错误和校验错误。
 * 帧头搜索阶段的普通噪声不计作完整协议帧错误。
 */
VisionParseResult VisionParser_PushByteEx(
    VisionParser *parser,
    uint8_t byte,
    VisionMeasurement *output
);

/*
 * 每收到一个串口字节调用一次。
 * 只有凑齐完整帧，且版本与校验正确时才返回 true 并更新 output。
 * 这是兼容旧测试代码的布尔包装接口。
 */
bool VisionParser_PushByte(
    VisionParser *parser,
    uint8_t byte,
    VisionMeasurement *output
);

/* 供 K230 端和 PC 测试复现的简单异或校验。 */
uint8_t VisionProtocol_Checksum(const uint8_t *data, uint8_t length);

#endif
