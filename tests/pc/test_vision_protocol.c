#include <assert.h>
#include <stdio.h>
#include "vision_protocol.h"
#include "control_guard.h"

static void make_packet(
    uint8_t packet[VISION_PACKET_SIZE],
    uint8_t flags,
    uint16_t frame_id,
    int16_t error_px,
    int16_t ball_x)
{
    packet[0] = VISION_PACKET_HEADER_0;
    packet[1] = VISION_PACKET_HEADER_1;
    packet[2] = VISION_PACKET_VERSION;
    packet[3] = flags;
    packet[4] = (uint8_t)(frame_id & 0xFFU);
    packet[5] = (uint8_t)((frame_id >> 8) & 0xFFU);
    packet[6] = (uint8_t)((uint16_t)error_px & 0xFFU);
    packet[7] = (uint8_t)(((uint16_t)error_px >> 8) & 0xFFU);
    packet[8] = (uint8_t)((uint16_t)ball_x & 0xFFU);
    packet[9] = (uint8_t)(((uint16_t)ball_x >> 8) & 0xFFU);
    packet[10] = VisionProtocol_Checksum(&packet[2], 8U);
}

int main(void)
{
    VisionParser parser;
    VisionMeasurement measurement = {0};
    uint8_t packet[VISION_PACKET_SIZE];
    bool complete = false;
    uint8_t i;

    VisionParser_Init(&parser);
    make_packet(
        packet,
        VISION_FLAG_VALID | VISION_FLAG_SAFE,
        513U,
        -123,
        484
    );

    /* 前导噪声不应产生有效结果。 */
    assert(!VisionParser_PushByte(&parser, 0x10U, &measurement));
    assert(!VisionParser_PushByte(&parser, 0xAAU, &measurement));
    assert(!VisionParser_PushByte(&parser, 0x20U, &measurement));

    for (i = 0U; i < VISION_PACKET_SIZE; ++i)
    {
        complete = VisionParser_PushByte(&parser, packet[i], &measurement);
    }

    assert(complete);
    assert(measurement.frame_id == 513U);
    assert(measurement.error_px == -123);
    assert(measurement.ball_x == 484);
    assert(measurement.ball_valid);
    assert(measurement.ball_safe);

    assert(ControlGuard_Evaluate(&measurement, false, 0U, 0U)
           == CONTROL_GUARD_LINK_TIMEOUT);
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 251U)
           == CONTROL_GUARD_LINK_TIMEOUT);
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U)
           == CONTROL_GUARD_READY);

    measurement.ball_valid = false;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U)
           == CONTROL_GUARD_VISION_INVALID);

    measurement.ball_valid = true;
    measurement.ball_safe = false;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U)
           == CONTROL_GUARD_BALL_UNSAFE);

    measurement.ball_safe = true;
    measurement.ball_x = 59;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U)
           == CONTROL_GUARD_BALL_UNSAFE);

    /* 校验和损坏后不能输出新测量。 */
    VisionParser_Init(&parser);
    packet[10] ^= 0x01U;
    complete = false;
    for (i = 0U; i < VISION_PACKET_SIZE; ++i)
    {
        complete = VisionParser_PushByte(&parser, packet[i], &measurement);
    }
    assert(!complete);

    puts("vision_protocol_tests: PASS");
    return 0;
}

