#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdint.h>
#include "app_config.h"

/*
 * STM32 -> K230 black-box telemetry V2, little-endian, fixed at 64 bytes.
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
 * 32      control flags
 * 33      CH32 motion state
 * 34      CH32 motion flags
 * 35      CH32 motion link valid (0/1)
 * 36..37  track-direction acceleration in mg
 * 38..39  yaw rate in 0.1 deg/s
 * 40..41  vibration level in mg
 * 42..43  line error
 * 44..45  left wheel speed
 * 46..47  right wheel speed
 * 48..49  turn command (right target - left target)
 * 50..51  latest CH32 motion frame sequence
 * 52..53  latest CH32 motion frame age in ms (saturated to uint16)
 * 54      ball lost grace active (0/1)
 * 55..56  ball lost age in ms
 * 57..58  ball lost grace recovery count
 * 59..61  reserved, zero
 * 62..63  CRC-16/CCITT-FALSE over bytes 2..61
 *
 * 64-byte fixed records let the K230 buffer 16 records into an exact 1024-byte
 * SD-card write block without converting data to CSV in the real-time loop.
 */
#define TELEMETRY_PACKET_HEADER_0     0x54U
#define TELEMETRY_PACKET_HEADER_1     0x4DU
#define TELEMETRY_PACKET_VERSION      2U
#define TELEMETRY_PACKET_SIZE         64U
#define TELEMETRY_PACKET_CRC_OFFSET   62U

#define TELEMETRY_FLAG_APPLY_OUTPUT       0x01U
#define TELEMETRY_FLAG_MANUAL_MODE        0x02U
#define TELEMETRY_FLAG_BALL_VALID         0x04U
#define TELEMETRY_FLAG_BALL_SAFE          0x08U
#define TELEMETRY_FLAG_CONTROL_RAN        0x10U
#define TELEMETRY_FLAG_OUTPUT_SATURATED   0x20U
#define TELEMETRY_FLAG_SERVO_STARTED      0x40U
#define TELEMETRY_FLAG_NEW_VISION_PACKET  0x80U

/*
 * V3 控制量分解调试遥测（96 字节）。
 *
 * 64..65  pid_sum_raw_us (int16)
 * 66..67  pid_sum_directed_us (int16)
 * 68..69  acc_filtered_mg (int16)
 * 70..71  af_raw_us (int16)
 * 72..73  af_clamped_us (int16)
 * 74..75  af_slewed_us (int16)
 * 76..77  yaw_raw_dps10 (int16)
 * 78..79  speed_average (int16)
 * 80..81  speed_scale_x1000 (uint16)
 * 82..83  yf_raw_us (int16)
 * 84..85  yf_clamped_us (int16)
 * 86..87  yf_slewed_us (int16)
 * 88..89  hold_pwm_effective_us (uint16)
 * 90..91  feedforward_total_us (int16)
 * 92..93  servo_prelimit_us (int16)
 * 94     servo_flags (uint8)
 * 95     motion_bias_active_us (int8)
 * 96..97  turn_scale_x1000 (uint16)
 * 98..99  yaw_handover_x1000 (uint16)
 * 100..101 turn_preview_raw_us (int16)
 * 102..103 turn_preview_slewed_us (int16)
 * 104..105 CRC-16/CCITT-FALSE over bytes 2..103
 */
#define TELEMETRY_PACKET_V3_VERSION      3U
#define TELEMETRY_PACKET_V3_SIZE         106U
#define TELEMETRY_PACKET_V3_CRC_OFFSET   104U

#define TELEMETRY_SERVO_FLAG_TARGET_SATURATED  0x01U
#define TELEMETRY_SERVO_FLAG_SLEW_ACTIVE       0x02U
#define TELEMETRY_SERVO_FLAG_AF_SLEW_ACTIVE    0x04U
#define TELEMETRY_SERVO_FLAG_YF_SLEW_ACTIVE    0x08U

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
    uint8_t motion_state;
    uint8_t motion_flags;
    uint8_t motion_link_valid;
    int16_t acc_track_mg;
    int16_t yaw_rate_dps10;
    uint16_t vibration_level_mg;
    int16_t line_error;
    int16_t left_speed;
    int16_t right_speed;
    int16_t turn_command;
    uint16_t motion_sequence;
    uint16_t motion_age_ms;
    uint8_t lost_grace;
    uint16_t lost_age_ms;
    uint16_t lost_recovery;
#if BALL_TELEMETRY_CONTROL_DECOMPOSITION != 0U
    /* V3 控制量分解扩展（由 EncodeV3 写入，不从 TelemetrySample 直接编码） */
    int16_t pid_sum_raw_us;
    int16_t pid_sum_directed_us;
    int16_t acc_filtered_mg;
    int16_t af_raw_us;
    int16_t af_clamped_us;
    int16_t af_slewed_us;
    int16_t yaw_raw_dps10;
    int16_t speed_average;
    uint16_t speed_scale_x1000;
    int16_t yf_raw_us;
    int16_t yf_clamped_us;
    int16_t yf_slewed_us;
    uint16_t hold_pwm_effective_us;
    int16_t feedforward_total_us;
    int16_t servo_prelimit_us;
    uint8_t servo_flags;
    int8_t motion_bias_active_us;
    uint16_t turn_scale_x1000;
    uint16_t yaw_handover_x1000;
    int16_t turn_preview_raw_us;
    int16_t turn_preview_slewed_us;
#endif
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

#if BALL_TELEMETRY_CONTROL_DECOMPOSITION != 0U
void TelemetryProtocol_EncodeV3(
    const TelemetrySample *sample,
    uint16_t sequence,
    uint8_t packet[TELEMETRY_PACKET_V3_SIZE]
);
#endif

#endif
