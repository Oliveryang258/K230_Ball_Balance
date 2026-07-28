#include "app_uart_rx.h"

static UART_HandleTypeDef *s_uart = 0;
static uint8_t s_rx_byte = 0U;
static VisionParser s_parser;

/*
 * s_latest是多字段结构体，不能依赖volatile获得一致性。
 * USART ISR负责写入；控制任务必须通过短临界区快照读取。
 */
static VisionMeasurement s_latest;

/* 这些标量在ISR与非ISR代码之间共享，因此使用volatile。 */
static volatile bool s_has_packet = false;
static volatile bool s_new_packet = false;
static volatile uint32_t s_last_packet_tick = 0U;
static volatile uint32_t s_valid_packet_count = 0U;
static volatile uint32_t s_uart_error_count = 0U;
static volatile uint32_t s_protocol_error_count = 0U;

static void rearm_one_byte_receive(void)
{
    if (s_uart != 0)
    {
        /* 一次接收一个字节，直接送入轻量协议状态机。 */
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
    s_protocol_error_count = 0U;
    VisionParser_Init(&s_parser);

    return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) == HAL_OK;
}

void AppUartRx_OnRxComplete(UART_HandleTypeDef *huart)
{
    VisionMeasurement decoded;
    VisionParseResult parse_result;

    if ((s_uart == 0) || (huart != s_uart))
    {
        return;
    }

    /*
     * 只有版本和校验都通过的完整帧才发布时间戳。
     * 该时间戳就是后续速度估计所用的“真实本地接收时刻”。
     */
    parse_result = VisionParser_PushByteEx(&s_parser, s_rx_byte, &decoded);
    if (parse_result == VISION_PARSE_VALID)
    {
        s_latest = decoded;
        s_last_packet_tick = HAL_GetTick();
        s_valid_packet_count++;
        s_has_packet = true;
        s_new_packet = true;
    }
    else if ((parse_result == VISION_PARSE_BAD_VERSION) ||
             (parse_result == VISION_PARSE_BAD_CHECKSUM))
    {
        s_protocol_error_count++;
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

bool AppUartRx_TakeSnapshot(AppUartRxSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == 0)
    {
        return false;
    }

    /*
     * 临界区只复制发布状态，不解析协议、不估计速度、不运行PD。
     * 保存PRIMASK，避免错误打开调用者原本已经关闭的中断。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    snapshot->measurement = s_latest;
    snapshot->has_packet = s_has_packet;
    snapshot->new_packet = s_new_packet;
    snapshot->last_packet_tick = s_last_packet_tick;
    snapshot->valid_packet_count = s_valid_packet_count;
    snapshot->uart_error_count = s_uart_error_count;
    snapshot->protocol_error_count = s_protocol_error_count;
    s_new_packet = false;

    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool AppUartRx_GetLatest(VisionMeasurement *output)
{
    AppUartRxSnapshot snapshot;

    if ((output == 0) || (!AppUartRx_TakeSnapshot(&snapshot)))
    {
        return false;
    }

    *output = snapshot.measurement;
    return snapshot.new_packet;
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

uint32_t AppUartRx_GetProtocolErrorCount(void)
{
    return s_protocol_error_count;
}
