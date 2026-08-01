#include "ch32_command_protocol.h"

#include "ch32_motion_protocol.h"

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

void Ch32CommandParser_Init(Ch32CommandParser *parser)
{
    if (parser != 0)
    {
        parser->index = 0U;
    }
}

Ch32CommandParseResult Ch32CommandParser_PushByte(
    Ch32CommandParser *parser,
    uint8_t byte,
    Ch32CommandFrame *output)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((parser == 0) || (output == 0))
    {
        return CH32_COMMAND_PARSE_INCOMPLETE;
    }

    /* 帧头搜索与运动帧相同的可重同步状态机，线路噪声不会破坏字节边界。 */
    if (parser->index == 0U)
    {
        if (byte == CH32_COMMAND_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        return CH32_COMMAND_PARSE_INCOMPLETE;
    }

    if (parser->index == 1U)
    {
        if (byte == CH32_COMMAND_HEADER_1)
        {
            parser->buffer[1] = byte;
            parser->index = 2U;
        }
        else if (byte == CH32_COMMAND_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        else
        {
            parser->index = 0U;
        }
        return CH32_COMMAND_PARSE_INCOMPLETE;
    }

    parser->buffer[parser->index] = byte;
    parser->index++;
    if (parser->index < CH32_COMMAND_PACKET_SIZE)
    {
        return CH32_COMMAND_PARSE_INCOMPLETE;
    }
    parser->index = 0U;

    if (parser->buffer[2] != CH32_COMMAND_VERSION)
    {
        return CH32_COMMAND_PARSE_BAD_VERSION;
    }
    if (parser->buffer[3] != CH32_COMMAND_PACKET_SIZE)
    {
        return CH32_COMMAND_PARSE_BAD_LENGTH;
    }

    received_crc = read_u16_le(&parser->buffer[10]);
    calculated_crc = Ch32MotionProtocol_Crc16CcittFalse(
        parser->buffer,
        10U
    );
    if (received_crc != calculated_crc)
    {
        return CH32_COMMAND_PARSE_BAD_CRC;
    }

    output->sequence = read_u16_le(&parser->buffer[4]);
    output->mode = parser->buffer[6];
    output->valid = parser->buffer[7];
    output->position_01cm = read_i16_le(&parser->buffer[8]);
    return CH32_COMMAND_PARSE_VALID;
}
