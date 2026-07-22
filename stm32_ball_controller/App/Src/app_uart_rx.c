#include "app_uart_rx.h"

static UART_HandleTypeDef *s_uart = 0;
static uint8_t s_rx_byte = 0U;
static VisionParser s_parser;
static VisionMeasurement s_latest;

/* 这些变量会在中断和主循环之间共享，因此使用 volatile。 */
static volatile bool s_has_packet = false;
static volatile bool s_new_packet = false;
static volatile uint32_t s_last_packet_tick = 0U;
static volatile uint32_t s_valid_packet_count = 0U;
static volatile uint32_t s_uart_error_count = 0U;

static void rearm_one_byte_receive(void)
{
    if (s_uart != 0)
    {
        /* 一次只接收一个字节，便于直接喂给协议状态机。 */
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

bool AppUartRx_Init(UART_HandleTypeDef *huart)
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
    VisionParser_Init(&s_parser);

    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) == HAL_OK;
}

void AppUartRx_OnRxComplete(UART_HandleTypeDef *huart)
{
    VisionMeasurement decoded;

    if ((s_uart == 0) || (huart != s_uart))
    {
        return;
    }

    /*
     * 解析成功之前不更新时间戳。
     * 因此线路噪声或校验错误不能伪装成通信正常。
     */
    if (VisionParser_PushByte(&s_parser, s_rx_byte, &decoded))
    {
        s_latest = decoded;
        s_last_packet_tick = HAL_GetTick();
        s_valid_packet_count++;
        s_has_packet = true;
        s_new_packet = true;
    }

    rearm_one_byte_receive();
}

void AppUartRx_OnError(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart != s_uart))
    {
        return;
    }

    s_uart_error_count++;
    VisionParser_Init(&s_parser);
    rearm_one_byte_receive();
}

bool AppUartRx_GetLatest(VisionMeasurement *output)
{
    uint32_t primask;

    if (output == 0)
    {
        return false;
    }

    /*
     * 用极短临界区保证复制结构体时不会正好被串口中断改写。
     * 保存 PRIMASK 是为了不错误地打开调用者原本已经关闭的中断。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    if (!s_new_packet)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }

    *output = s_latest;
    s_new_packet = false;

    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool AppUartRx_HasPacket(void)
{
    return s_has_packet;
}

uint32_t AppUartRx_GetLastPacketTick(void)
{
    return s_last_packet_tick;
}

uint32_t AppUartRx_GetValidPacketCount(void)
{
    return s_valid_packet_count;
}

uint32_t AppUartRx_GetUartErrorCount(void)
{
    return s_uart_error_count;
}

