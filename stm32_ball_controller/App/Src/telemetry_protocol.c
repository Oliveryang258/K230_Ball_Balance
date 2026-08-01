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
    packet[33] = sample->motion_state;
    packet[34] = sample->motion_flags;
    packet[35] = sample->motion_link_valid;
    write_u16_le(&packet[36], (uint16_t)sample->acc_track_mg);
    write_u16_le(&packet[38], (uint16_t)sample->yaw_rate_dps10);
    write_u16_le(&packet[40], sample->vibration_level_mg);
    write_u16_le(&packet[42], (uint16_t)sample->line_error);
    write_u16_le(&packet[44], (uint16_t)sample->left_speed);
    write_u16_le(&packet[46], (uint16_t)sample->right_speed);
    write_u16_le(&packet[48], (uint16_t)sample->turn_command);
    write_u16_le(&packet[50], sample->motion_sequence);
    write_u16_le(&packet[52], sample->motion_age_ms);
    packet[54] = sample->lost_grace;
    write_u16_le(&packet[55], sample->lost_age_ms);
    write_u16_le(&packet[57], sample->lost_recovery);
    packet[59] = 0U;
    packet[60] = 0U;
    packet[61] = 0U;

    crc = TelemetryProtocol_Crc16Ccitt(
        &packet[2],
        (uint16_t)(TELEMETRY_PACKET_CRC_OFFSET - 2U)
    );
    write_u16_le(&packet[TELEMETRY_PACKET_CRC_OFFSET], crc);
}

#if BALL_TELEMETRY_CONTROL_DECOMPOSITION != 0U
void TelemetryProtocol_EncodeV3(
    const TelemetrySample *sample,
    uint16_t sequence,
    uint8_t packet[TELEMETRY_PACKET_V3_SIZE]
)
{
    uint16_t crc;

    if ((sample == 0) || (packet == 0))
    {
        return;
    }

    /* Bytes 0..53: identical to V2 Encode */
    packet[0] = TELEMETRY_PACKET_HEADER_0;
    packet[1] = TELEMETRY_PACKET_HEADER_1;
    packet[2] = TELEMETRY_PACKET_V3_VERSION;
    packet[3] = TELEMETRY_PACKET_V3_SIZE;
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
    packet[33] = sample->motion_state;
    packet[34] = sample->motion_flags;
    packet[35] = sample->motion_link_valid;
    write_u16_le(&packet[36], (uint16_t)sample->acc_track_mg);
    write_u16_le(&packet[38], (uint16_t)sample->yaw_rate_dps10);
    write_u16_le(&packet[40], sample->vibration_level_mg);
    write_u16_le(&packet[42], (uint16_t)sample->line_error);
    write_u16_le(&packet[44], (uint16_t)sample->left_speed);
    write_u16_le(&packet[46], (uint16_t)sample->right_speed);
    write_u16_le(&packet[48], (uint16_t)sample->turn_command);
    write_u16_le(&packet[50], sample->motion_sequence);
    write_u16_le(&packet[52], sample->motion_age_ms);
    packet[54] = sample->lost_grace;
    write_u16_le(&packet[55], sample->lost_age_ms);
    write_u16_le(&packet[57], sample->lost_recovery);
    packet[59] = 0U;
    packet[60] = 0U;
    packet[61] = 0U;

    /* V2 CRC over bytes 2..61 for backward compatibility */
    crc = TelemetryProtocol_Crc16Ccitt(
        &packet[2],
        (uint16_t)(TELEMETRY_PACKET_CRC_OFFSET - 2U)
    );
    write_u16_le(&packet[TELEMETRY_PACKET_CRC_OFFSET], crc);

    /* Bytes 64..95: V3 decomposition extension */
    write_u16_le(&packet[64], (uint16_t)sample->pid_sum_raw_us);
    write_u16_le(&packet[66], (uint16_t)sample->pid_sum_directed_us);
    write_u16_le(&packet[68], (uint16_t)sample->acc_filtered_mg);
    write_u16_le(&packet[70], (uint16_t)sample->af_raw_us);
    write_u16_le(&packet[72], (uint16_t)sample->af_clamped_us);
    write_u16_le(&packet[74], (uint16_t)sample->af_slewed_us);
    write_u16_le(&packet[76], (uint16_t)sample->yaw_raw_dps10);
    write_u16_le(&packet[78], (uint16_t)sample->speed_average);
    write_u16_le(&packet[80], sample->speed_scale_x1000);
    write_u16_le(&packet[82], (uint16_t)sample->yf_raw_us);
    write_u16_le(&packet[84], (uint16_t)sample->yf_clamped_us);
    write_u16_le(&packet[86], (uint16_t)sample->yf_slewed_us);
    write_u16_le(&packet[88], sample->hold_pwm_effective_us);
    write_u16_le(&packet[90], (uint16_t)sample->feedforward_total_us);
    write_u16_le(&packet[92], (uint16_t)sample->servo_prelimit_us);
    packet[94] = sample->servo_flags;
    packet[95] = (uint8_t)sample->motion_bias_active_us;

    /* Bytes 96..103: turn preview extension */
    write_u16_le(&packet[96], sample->turn_scale_x1000);
    write_u16_le(&packet[98], sample->yaw_handover_x1000);
    write_u16_le(&packet[100], (uint16_t)sample->turn_preview_raw_us);
    write_u16_le(&packet[102], (uint16_t)sample->turn_preview_slewed_us);

    crc = TelemetryProtocol_Crc16Ccitt(
        &packet[2],
        (uint16_t)(TELEMETRY_PACKET_V3_CRC_OFFSET - 2U)
    );
    write_u16_le(&packet[TELEMETRY_PACKET_V3_CRC_OFFSET], crc);
}
#endif
