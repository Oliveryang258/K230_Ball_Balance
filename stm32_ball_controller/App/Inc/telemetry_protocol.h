#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdint.h>

/*
 * STM32 -> K230 telemetry packet, little-endian, fixed at 36 bytes.
 *
 *  0..1   header 0x54 0x4D ("TM")
 *  2      protocol version
 *  3      packet size
 *  4..5   telemetry sequence
 *  6..9   STM32 tick in ms
 * 10..11  latest K230 vision frame id
 * 12..13  latest ball x
 * 14..15  latest vision error in px
 * 16..17  filtered ball velocity in px/s
 * 18..19  P term in us
 * 20..21  I term in us
 * 22..23  D term in us
 * 24..25  direction-processed control offset in us
 * 26..27  servo target pulse in us
 * 28..29  servo pulse currently written to CCR in us
 * 30      control guard state
 * 31      measurement acceptance status
 * 32      flags
 * 33      reserved, zero
 * 34..35  CRC-16/CCITT-FALSE over bytes 2..33
 */
#define TELEMETRY_PACKET_HEADER_0     0x54U
#define TELEMETRY_PACKET_HEADER_1     0x4DU
#define TELEMETRY_PACKET_VERSION      1U
#define TELEMETRY_PACKET_SIZE         36U
#define TELEMETRY_PACKET_CRC_OFFSET   34U

#define TELEMETRY_FLAG_APPLY_OUTPUT       0x01U
#define TELEMETRY_FLAG_MANUAL_MODE        0x02U
#define TELEMETRY_FLAG_BALL_VALID         0x04U
#define TELEMETRY_FLAG_BALL_SAFE          0x08U
#define TELEMETRY_FLAG_CONTROL_RAN        0x10U
#define TELEMETRY_FLAG_OUTPUT_SATURATED   0x20U
#define TELEMETRY_FLAG_SERVO_STARTED      0x40U
#define TELEMETRY_FLAG_NEW_VISION_PACKET  0x80U

typedef struct
{
    uint32_t tick_ms;
    uint16_t vision_frame_id;
    int16_t ball_x;
    int16_t error_px;
    int16_t velocity_px_s;
    int16_t p_term_us;
    int16_t i_term_us;
    int16_t d_term_us;
    int16_t control_offset_us;
    uint16_t servo_target_us;
    uint16_t servo_current_us;
    uint8_t guard_state;
    uint8_t measurement_status;
    uint8_t flags;
} TelemetrySample;

uint16_t TelemetryProtocol_Crc16Ccitt(
    const uint8_t *data,
    uint16_t length
);

void TelemetryProtocol_Encode(
    const TelemetrySample *sample,
    uint16_t sequence,
    uint8_t packet[TELEMETRY_PACKET_SIZE]
);

#endif
