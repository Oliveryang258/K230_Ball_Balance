/* 独立验证 Ch32CommandParser 对TP帧（0x54 0x50）的解析与CRC校验。 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ch32_command_protocol.h"

/* 与 ch32_motion_protocol.c 相同的 CRC16-CCITT-FALSE，供主机端自校验。 */
uint16_t Ch32MotionProtocol_Crc16CcittFalse(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void build_tp_frame(uint8_t *frame,
                           uint16_t seq,
                           uint8_t mode,
                           uint8_t valid,
                           int16_t position_01cm)
{
    frame[0] = 0x54;
    frame[1] = 0x50;
    frame[2] = 0x01;
    frame[3] = 0x0C;
    frame[4] = (uint8_t)(seq & 0xFFU);
    frame[5] = (uint8_t)((seq >> 8) & 0xFFU);
    frame[6] = mode;
    frame[7] = valid;
    frame[8] = (uint8_t)(position_01cm & 0xFF);
    frame[9] = (uint8_t)((position_01cm >> 8) & 0xFF);
    {
        uint16_t crc = Ch32MotionProtocol_Crc16CcittFalse(frame, 10U);
        frame[10] = (uint8_t)(crc & 0xFFU);
        frame[11] = (uint8_t)((crc >> 8) & 0xFFU);
    }
}

static void feed(Ch32CommandParser *parser,
                 Ch32CommandFrame *out,
                 const uint8_t *bytes,
                 uint32_t len)
{
    uint32_t i;
    for (i = 0U; i < len; ++i)
    {
        Ch32CommandParser_PushByte(parser, bytes[i], out);
    }
}

int main(void)
{
    int fails = 0;
    Ch32CommandParser parser;
    Ch32CommandFrame out;
    uint8_t frame[12];

    /* 用例1：模式号=1（题目二"0下"）、位置+5.0cm、有效 */
    build_tp_frame(frame, 0U, 1U, 1U, 50);
    Ch32CommandParser_Init(&parser);
    feed(&parser, &out, frame, sizeof(frame));
    if (out.mode != 1U || out.valid != 1U || out.position_01cm != 50 ||
        out.sequence != 0U)
    {
        printf("CASE1 FAIL: mode=%u valid=%u pos=%d seq=%u\n",
               out.mode, out.valid, out.position_01cm, out.sequence);
        fails++;
    }
    else
    {
        printf("CASE1 OK:  mode=1 valid=1 pos=+50\n");
    }

    /* 用例2：负位置 -5.0cm */
    build_tp_frame(frame, 7U, 2U, 1U, -50);
    Ch32CommandParser_Init(&parser);
    feed(&parser, &out, frame, sizeof(frame));
    if (out.mode != 2U || out.position_01cm != -50 || out.sequence != 7U)
    {
        printf("CASE2 FAIL: mode=%u pos=%d seq=%u\n",
               out.mode, out.position_01cm, out.sequence);
        fails++;
    }
    else
    {
        printf("CASE2 OK:  mode=2 pos=-50 seq=7\n");
    }

    /* 用例3：CRC错误（翻转CRC低字节） */
    build_tp_frame(frame, 0U, 1U, 1U, 50);
    frame[10] ^= 0xFFU;
    Ch32CommandParser_Init(&parser);
    {
        Ch32CommandParseResult r = CH32_COMMAND_PARSE_INCOMPLETE;
        uint32_t i;
        for (i = 0U; i < sizeof(frame); ++i)
        {
            r = Ch32CommandParser_PushByte(&parser, frame[i], &out);
        }
        if (r != CH32_COMMAND_PARSE_BAD_CRC)
        {
            printf("CASE3 FAIL: expected BAD_CRC, got %d\n", (int)r);
            fails++;
        }
        else
        {
            printf("CASE3 OK:  bad CRC rejected\n");
        }
    }

    /* 用例4：版本错误 */
    build_tp_frame(frame, 0U, 1U, 1U, 50);
    frame[2] = 0x02U;
    Ch32CommandParser_Init(&parser);
    {
        Ch32CommandParseResult r = CH32_COMMAND_PARSE_INCOMPLETE;
        uint32_t i;
        for (i = 0U; i < sizeof(frame); ++i)
        {
            r = Ch32CommandParser_PushByte(&parser, frame[i], &out);
        }
        if (r != CH32_COMMAND_PARSE_BAD_VERSION)
        {
            printf("CASE4 FAIL: expected BAD_VERSION, got %d\n", (int)r);
            fails++;
        }
        else
        {
            printf("CASE4 OK:  bad version rejected\n");
        }
    }

    /* 用例5：长度错误 */
    build_tp_frame(frame, 0U, 1U, 1U, 50);
    frame[3] = 0x0DU;
    Ch32CommandParser_Init(&parser);
    {
        Ch32CommandParseResult r = CH32_COMMAND_PARSE_INCOMPLETE;
        uint32_t i;
        for (i = 0U; i < sizeof(frame); ++i)
        {
            r = Ch32CommandParser_PushByte(&parser, frame[i], &out);
        }
        if (r != CH32_COMMAND_PARSE_BAD_LENGTH)
        {
            printf("CASE5 FAIL: expected BAD_LENGTH, got %d\n", (int)r);
            fails++;
        }
        else
        {
            printf("CASE5 OK:  bad length rejected\n");
        }
    }

    /* 用例6：前置噪声字节后仍能重同步并解析出合法帧 */
    {
        const uint8_t noise[] = {0x00, 0x43, 0x48, 0xFF, 0xAA, 0x12};
        Ch32CommandParser_Init(&parser);
        feed(&parser, &out, noise, sizeof(noise));
        build_tp_frame(frame, 0U, 1U, 1U, 50);
        feed(&parser, &out, frame, sizeof(frame));
        if (out.mode != 1U || out.position_01cm != 50)
        {
            printf("CASE6 FAIL: mode=%u pos=%d\n", out.mode, out.position_01cm);
            fails++;
        }
        else
        {
            printf("CASE6 OK:  resync after noise\n");
        }
    }

    /* 用例7：CH运动帧字节流不会产生合法TP帧 */
    {
        const uint8_t ch_like[] =
        {
            0x43, 0x48, 0x01, 0x1C, 0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        uint32_t i;
        uint32_t valid_count = 0U;
        Ch32CommandParser_Init(&parser);
        memset(&out, 0, sizeof(out));
        for (i = 0U; i < sizeof(ch_like); ++i)
        {
            if (Ch32CommandParser_PushByte(&parser, ch_like[i], &out) ==
                CH32_COMMAND_PARSE_VALID)
            {
                valid_count++;
            }
        }
        if (valid_count != 0U)
        {
            printf("CASE7 FAIL: CH bytes decoded %u TP frame(s)\n",
                   valid_count);
            fails++;
        }
        else
        {
            printf("CASE7 OK:  CH-like bytes never decode as TP frame\n");
        }
    }

    printf("\n%s (%d failures)\n", fails ? "TEST FAILED" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
