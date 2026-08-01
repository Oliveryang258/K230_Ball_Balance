#include "app_motion_rx.h"

static UART_HandleTypeDef *s_uart = 0;
static uint8_t s_rx_byte = 0U;
static Ch32MotionParser s_parser;
static Ch32MotionMeasurement s_latest;
static Ch32CommandParser s_command_parser;
static Ch32CommandFrame s_latest_command;

static volatile bool s_has_packet = false;
static volatile bool s_new_packet = false;
static volatile uint32_t s_last_packet_tick = 0U;
static volatile uint32_t s_valid_packet_count = 0U;
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_protocol_error_count = 0U;
static volatile bool s_command_has_packet = false;
static volatile bool s_command_new = false;
static volatile uint32_t s_command_last_tick = 0U;
static volatile uint32_t s_command_valid_count = 0U;
static volatile uint32_t s_command_error_count = 0U;
static volatile bool s_start_event_pending = false;
static bool s_last_start_event = false;

static void rearm_one_byte_receive(void)
{
    if (s_uart != 0)
    {
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

bool AppMotionRx_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        return false;
    }

    s_uart = huart;
    s_has_packet = false;
    s_new_packet = false;
    s_last_packet_tick = 0U;
    s_valid_packet_count = 0U;
    s_uart_error_count = 0U;
    s_protocol_error_count = 0U;
    s_command_has_packet = false;
    s_command_new = false;
    s_command_last_tick = 0U;
    s_command_valid_count = 0U;
    s_command_error_count = 0U;
    s_start_event_pending = false;
    s_last_start_event = false;
    Ch32MotionParser_Init(&s_parser);
    Ch32CommandParser_Init(&s_command_parser);
    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) == HAL_OK;
}

void AppMotionRx_OnRxComplete(UART_HandleTypeDef *huart)
{
    Ch32MotionMeasurement decoded;
    Ch32MotionParseResult result;
    Ch32CommandFrame command;
    Ch32CommandParseResult command_result;

    if ((s_uart == 0) || (huart != s_uart))
    {
        return;
    }

    result = Ch32MotionParser_PushByte(&s_parser, s_rx_byte, &decoded);
    if (result == CH32_MOTION_PARSE_VALID)
    {
        s_latest = decoded;
        s_last_packet_tick = HAL_GetTick();
        s_valid_packet_count++;
        s_has_packet = true;
        s_new_packet = true;

        /*
         * 起步事件（BUT2发车）上升沿锁存。起步事件可能只持续
         * 一两个运动帧，锁存到控制任务快照消费为止，保证不丢。
         */
        {
            bool start_now =
                (decoded.flags & CH32_MOTION_FLAG_START_EVENT) != 0U;
            if (start_now && !s_last_start_event)
            {
                s_start_event_pending = true;
            }
            s_last_start_event = start_now;
        }
    }
    else if (result != CH32_MOTION_PARSE_INCOMPLETE)
    {
        s_protocol_error_count++;
    }

    /*
     * TP命令帧与运动帧共用USART2。每个字节同时喂给两个自同步解析器，
     * CH帧字节不会进入运动解析器之外的路径，TP帧也不影响运动链路。
     */
    command_result = Ch32CommandParser_PushByte(
        &s_command_parser,
        s_rx_byte,
        &command
    );
    if (command_result == CH32_COMMAND_PARSE_VALID)
    {
        s_latest_command = command;
        s_command_last_tick = HAL_GetTick();
        s_command_valid_count++;
        s_command_has_packet = true;
        s_command_new = true;
    }
    else if (command_result != CH32_COMMAND_PARSE_INCOMPLETE)
    {
        s_command_error_count++;
    }

    rearm_one_byte_receive();
}

void AppMotionRx_OnError(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart != s_uart))
    {
        return;
    }

    s_uart_error_count++;
    Ch32MotionParser_Init(&s_parser);
    rearm_one_byte_receive();
}

bool AppMotionRx_TakeSnapshot(AppMotionRxSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == 0)
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->measurement = s_latest;
    snapshot->has_packet = s_has_packet;
    snapshot->new_packet = s_new_packet;
    snapshot->last_packet_tick = s_last_packet_tick;
    snapshot->valid_packet_count = s_valid_packet_count;
    snapshot->uart_error_count = s_uart_error_count;
    snapshot->protocol_error_count = s_protocol_error_count;
    snapshot->command = s_latest_command;
    snapshot->command_has_packet = s_command_has_packet;
    snapshot->command_new = s_command_new;
    snapshot->command_last_tick = s_command_last_tick;
    snapshot->command_valid_count = s_command_valid_count;
    snapshot->command_error_count = s_command_error_count;
    snapshot->start_event_pending = s_start_event_pending;
    s_start_event_pending = false;
    s_new_packet = false;
    s_command_new = false;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

uint32_t AppMotionRx_GetValidPacketCount(void)
{
    return s_valid_packet_count;
}

uint32_t AppMotionRx_GetUartErrorCount(void)
{
    return s_uart_error_count;
}

uint32_t AppMotionRx_GetProtocolErrorCount(void)
{
    return s_protocol_error_count;
}
