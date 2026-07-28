#include "telemetry_protocol.h"

static void write_u16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint16_t TelemetryProtocol_Crc16Ccitt(
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if (data == 0)
    {
        return crc;
    }

    for (index = 0U; index < length; ++index)
    {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void TelemetryProtocol_Encode(
    const TelemetrySample *sample,
    uint16_t sequence,
    uint8_t packet[TELEMETRY_PACKET_SIZE]
)
{
    uint16_t crc;

    if ((sample == 0) || (packet == 0))
    {
        return;
    }

    packet[0] = TELEMETRY_PACKET_HEADER_0;
    packet[1] = TELEMETRY_PACKET_HEADER_1;
    packet[2] = TELEMETRY_PACKET_VERSION;
    packet[3] = TELEMETRY_PACKET_SIZE;
    write_u16_le(&packet[4], sequence);
    write_u32_le(&packet[6], sample->tick_ms);
    write_u16_le(&packet[10], sample->vision_frame_id);
    write_u16_le(&packet[12], (uint16_t)sample->ball_x);
    write_u16_le(&packet[14], (uint16_t)sample->error_px);
    write_u16_le(&packet[16], (uint16_t)sample->velocity_px_s);
    write_u16_le(&packet[18], (uint16_t)sample->p_term_us);
    write_u16_le(&packet[20], (uint16_t)sample->i_term_us);
    write_u16_le(&packet[22], (uint16_t)sample->d_term_us);
    write_u16_le(&packet[24], (uint16_t)sample->control_offset_us);
    write_u16_le(&packet[26], sample->servo_target_us);
    write_u16_le(&packet[28], sample->servo_current_us);
    packet[30] = sample->guard_state;
    packet[31] = sample->measurement_status;
    packet[32] = sample->flags;
    packet[33] = 0U;

    crc = TelemetryProtocol_Crc16Ccitt(
        &packet[2],
        (uint16_t)(TELEMETRY_PACKET_CRC_OFFSET - 2U)
    );
    write_u16_le(&packet[TELEMETRY_PACKET_CRC_OFFSET], crc);
}
