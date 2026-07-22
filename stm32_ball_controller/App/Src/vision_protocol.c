#include "vision_protocol.h"

static int16_t read_i16_le(const uint8_t *data)
{
    uint16_t raw = (uint16_t)data[0]
                 | ((uint16_t)data[1] << 8);
    return (int16_t)raw;
}

uint8_t VisionProtocol_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t i;

    for (i = 0U; i < length; ++i)
    {
        checksum ^= data[i];
    }
    return checksum;
}

void VisionParser_Init(VisionParser *parser)
{
    if (parser == 0)
    {
        return;
    }
    parser->index = 0U;
}

bool VisionParser_PushByte(
    VisionParser *parser,
    uint8_t byte,
    VisionMeasurement *output)
{
    uint8_t expected_checksum;
    uint8_t flags;

    if ((parser == 0) || (output == 0))
    {
        return false;
    }

    /*
     * 状态 0：只等待帧头第一个字节 0xAA。
     * 噪声字节会直接丢弃，不会污染后续数据。
     */
    if (parser->index == 0U)
    {
        if (byte == VISION_PACKET_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        return false;
    }

    /*
     * 状态 1：检查第二个帧头字节。
     * 如果又收到 0xAA，就把它当成新的帧头起点；否则重新搜索。
     */
    if (parser->index == 1U)
    {
        if (byte == VISION_PACKET_HEADER_1)
        {
            parser->buffer[1] = byte;
            parser->index = 2U;
        }
        else if (byte == VISION_PACKET_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        else
        {
            parser->index = 0U;
        }
        return false;
    }

    parser->buffer[parser->index] = byte;
    parser->index++;

    if (parser->index < VISION_PACKET_SIZE)
    {
        return false;
    }

    /* 本帧已经收满；无论成功失败，下一字节都重新搜索帧头。 */
    parser->index = 0U;

    if (parser->buffer[2] != VISION_PACKET_VERSION)
    {
        return false;
    }

    expected_checksum = VisionProtocol_Checksum(&parser->buffer[2], 8U);
    if (expected_checksum != parser->buffer[10])
    {
        return false;
    }

    flags = parser->buffer[3];
    output->ball_valid = (flags & VISION_FLAG_VALID) != 0U;
    output->ball_safe = (flags & VISION_FLAG_SAFE) != 0U;
    output->frame_id = (uint16_t)parser->buffer[4]
                     | ((uint16_t)parser->buffer[5] << 8);
    output->error_px = read_i16_le(&parser->buffer[6]);
    output->ball_x = read_i16_le(&parser->buffer[8]);

    return true;
}

