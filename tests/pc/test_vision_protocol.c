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

    assert(ControlGuard_Evaluate(&measurement, false, 0U, 0U, false, false)
           == CONTROL_GUARD_UART_TIMEOUT);
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 251U, false, false)
           == CONTROL_GUARD_UART_TIMEOUT);
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, false, false)
           == CONTROL_GUARD_READY);

    measurement.ball_valid = false;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, false, false)
           == CONTROL_GUARD_BALL_LOST);

    measurement.ball_valid = true;
    measurement.ball_safe = false;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, false, false)
           == CONTROL_GUARD_POSITION_OUT_OF_RANGE);

    measurement.ball_safe = true;
    measurement.ball_x = 59;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, false, false)
           == CONTROL_GUARD_POSITION_OUT_OF_RANGE);

    measurement.ball_x = 484;
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, true, false)
           == CONTROL_GUARD_PROTOCOL_ERROR);
    assert(ControlGuard_Evaluate(&measurement, true, 100U, 200U, false, true)
           == CONTROL_GUARD_INVALID_DT);

    /* 校验和损坏后不能输出新测量。 */
    VisionParser_Init(&parser);
    packet[10] ^= 0x01U;
    complete = false;
    for (i = 0U; i < VISION_PACKET_SIZE; ++i)
    {
        complete = VisionParser_PushByte(&parser, packet[i], &measurement);
    }
    assert(!complete);

    VisionParser_Init(&parser);
    make_packet(
        packet,
        VISION_FLAG_VALID | VISION_FLAG_SAFE,
        514U,
        0,
        351
    );
    packet[10] ^= 0x01U;
    for (i = 0U; i < (VISION_PACKET_SIZE - 1U); ++i)
    {
        assert(VisionParser_PushByteEx(&parser, packet[i], &measurement)
               == VISION_PARSE_INCOMPLETE);
    }
    assert(VisionParser_PushByteEx(
               &parser,
               packet[VISION_PACKET_SIZE - 1U],
               &measurement)
           == VISION_PARSE_BAD_CHECKSUM);

    puts("vision_protocol_tests: PASS");
    return 0;
}
