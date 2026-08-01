#include "ch32_motion_protocol.h"

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

uint16_t Ch32MotionProtocol_Crc16CcittFalse(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    if (data == 0)
    {
        return crc;
    }

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

void Ch32MotionParser_Init(Ch32MotionParser *parser)
{
    if (parser != 0)
    {
        parser->index = 0U;
    }
}

Ch32MotionParseResult Ch32MotionParser_PushByte(
    Ch32MotionParser *parser,
    uint8_t byte,
    Ch32MotionMeasurement *output)
{
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((parser == 0) || (output == 0))
    {
        return CH32_MOTION_PARSE_INCOMPLETE;
    }

    /* 帧头搜索采用可重同步状态机，线路噪声不会永久打乱字节边界。 */
    if (parser->index == 0U)
    {
        if (byte == CH32_MOTION_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        return CH32_MOTION_PARSE_INCOMPLETE;
    }

    if (parser->index == 1U)
    {
        if (byte == CH32_MOTION_HEADER_1)
        {
            parser->buffer[1] = byte;
            parser->index = 2U;
        }
        else if (byte == CH32_MOTION_HEADER_0)
        {
            parser->buffer[0] = byte;
            parser->index = 1U;
        }
        else
        {
            parser->index = 0U;
        }
        return CH32_MOTION_PARSE_INCOMPLETE;
    }

    parser->buffer[parser->index] = byte;
    parser->index++;
    if (parser->index < CH32_MOTION_PACKET_SIZE)
    {
        return CH32_MOTION_PARSE_INCOMPLETE;
    }
    parser->index = 0U;

    if (parser->buffer[2] != CH32_MOTION_VERSION)
    {
        return CH32_MOTION_PARSE_BAD_VERSION;
    }
    if (parser->buffer[3] != CH32_MOTION_PACKET_SIZE)
    {
        return CH32_MOTION_PARSE_BAD_LENGTH;
    }

    received_crc = read_u16_le(&parser->buffer[26]);
    calculated_crc = Ch32MotionProtocol_Crc16CcittFalse(
        parser->buffer,
        26U
    );
    if (received_crc != calculated_crc)
    {
        return CH32_MOTION_PARSE_BAD_CRC;
    }

    output->sequence = read_u16_le(&parser->buffer[4]);
    output->timestamp_ms = read_u32_le(&parser->buffer[6]);
    output->motion_state = parser->buffer[10];
    output->flags = parser->buffer[11];
    output->acc_track_mg = read_i16_le(&parser->buffer[12]);
    output->yaw_rate_dps10 = read_i16_le(&parser->buffer[14]);
    output->vibration_level_mg = read_u16_le(&parser->buffer[16]);
    output->line_error = read_i16_le(&parser->buffer[18]);
    output->left_speed = read_i16_le(&parser->buffer[20]);
    output->right_speed = read_i16_le(&parser->buffer[22]);
    output->turn_command = read_i16_le(&parser->buffer[24]);
    return CH32_MOTION_PARSE_VALID;
}
