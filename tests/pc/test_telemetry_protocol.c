#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "telemetry_protocol.h"

int main(void)
{
    static const uint8_t expected[TELEMETRY_PACKET_SIZE] = {
        0x54, 0x4D, 0x02, 0x40, 0x11, 0x00, 0x40, 0xE2,
        0x01, 0x00, 0x41, 0x01, 0x3A, 0x01, 0xE9, 0xFF,
        0x2D, 0x00, 0xC2, 0xFF, 0x1F, 0x00, 0xFC, 0xFF,
        0xDD, 0xFF, 0xFF, 0x05, 0xF0, 0x05, 0x00, 0x01,
        0x9D, 0x02, 0x03, 0x01, 0x88, 0xFF, 0x59, 0x01,
        0x58, 0x00, 0xF7, 0xFF, 0xC8, 0x00, 0xDC, 0x00,
        0x14, 0x00, 0x4D, 0x00, 0x0C, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x8D
    };
    TelemetrySample sample = {0};
    uint8_t packet[TELEMETRY_PACKET_SIZE] = {0};

    sample.tick_ms = 123456U;
    sample.vision_frame_id = 321U;
    sample.ball_x = 314;
    sample.error_px = -23;
    sample.velocity_px_s = 45;
    sample.p_term_us = -62;
    sample.i_term_us = 31;
    sample.d_term_us = -4;
    sample.control_offset_us = -35;
    sample.servo_target_us = 1535U;
    sample.servo_current_us = 1520U;
    sample.guard_state = 0U;
    sample.measurement_status = 1U;
    sample.flags = 0x9DU;
    sample.motion_state = 2U;
    sample.motion_flags = 3U;
    sample.motion_link_valid = 1U;
    sample.acc_track_mg = -120;
    sample.yaw_rate_dps10 = 345;
    sample.vibration_level_mg = 88U;
    sample.line_error = -9;
    sample.left_speed = 200;
    sample.right_speed = 220;
    sample.turn_command = 20;
    sample.motion_sequence = 77U;
    sample.motion_age_ms = 12U;

    TelemetryProtocol_Encode(&sample, 17U, packet);
    assert(memcmp(packet, expected, TELEMETRY_PACKET_SIZE) == 0);
    return 0;
}
