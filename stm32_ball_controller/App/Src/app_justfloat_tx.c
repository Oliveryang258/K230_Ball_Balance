#include "app_justfloat_tx.h"

static UART_HandleTypeDef *s_uart = 0;
static JustFloatSample s_latest_sample;
static volatile uint8_t s_pending = 0U;
static volatile uint8_t s_busy = 0U;
static uint8_t s_tx_packet[JUSTFLOAT_PACKET_SIZE];
static volatile uint32_t s_sent_count = 0U;
static volatile uint32_t s_overwrite_count = 0U;
static volatile uint32_t s_error_count = 0U;

static void restore_irq_state(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static void encode_float_le(uint8_t *output, float value)
{
    union
    {
        float f;
        uint32_t u;
    } bits;

    bits.f = value;
    output[0] = (uint8_t)bits.u;
    output[1] = (uint8_t)(bits.u >> 8);
    output[2] = (uint8_t)(bits.u >> 16);
    output[3] = (uint8_t)(bits.u >> 24);
}

static void encode_sample(const JustFloatSample *sample)
{
    const float *channels = &sample->time_s;
    uint8_t i;

    for (i = 0U; i < JUSTFLOAT_CHANNEL_COUNT; ++i)
    {
        encode_float_le(&s_tx_packet[(uint16_t)i * 4U], channels[i]);
    }

    /* VOFA+ JustFloat 的固定帧尾。 */
    s_tx_packet[JUSTFLOAT_PACKET_SIZE - 4U] = 0x00U;
    s_tx_packet[JUSTFLOAT_PACKET_SIZE - 3U] = 0x00U;
    s_tx_packet[JUSTFLOAT_PACKET_SIZE - 2U] = 0x80U;
    s_tx_packet[JUSTFLOAT_PACKET_SIZE - 1U] = 0x7FU;
}

bool AppJustFloatTx_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        return false;
    }

    s_uart = huart;
    s_pending = 0U;
    s_busy = 0U;
    s_sent_count = 0U;
    s_overwrite_count = 0U;
    s_error_count = 0U;
    return true;
}

void AppJustFloatTx_Publish(const JustFloatSample *sample)
{
    if ((s_uart == 0) || (sample == 0))
    {
        return;
    }

    if (s_pending != 0U)
    {
        s_overwrite_count++;
    }
    s_latest_sample = *sample;
    __DMB();
    s_pending = 1U;
}

void AppJustFloatTx_Poll(void)
{
    JustFloatSample sample;
    uint32_t primask;
    HAL_StatusTypeDef status;

    if ((s_uart == 0) || (s_busy != 0U))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_pending == 0U)
    {
        restore_irq_state(primask);
        return;
    }
    sample = s_latest_sample;
    s_pending = 0U;
    restore_irq_state(primask);

    encode_sample(&sample);
    s_busy = 1U;
    status = HAL_UART_Transmit_IT(
        s_uart,
        s_tx_packet,
        JUSTFLOAT_PACKET_SIZE
    );
    if (status != HAL_OK)
    {
        s_busy = 0U;
        s_error_count++;
    }
}

void AppJustFloatTx_OnTxComplete(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart == s_uart))
    {
        s_busy = 0U;
        s_sent_count++;
    }
}

void AppJustFloatTx_OnError(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart == s_uart))
    {
        s_busy = 0U;
        s_error_count++;
    }
}

uint32_t AppJustFloatTx_GetSentCount(void)
{
    return s_sent_count;
}

uint32_t AppJustFloatTx_GetOverwriteCount(void)
{
    return s_overwrite_count;
}

uint32_t AppJustFloatTx_GetErrorCount(void)
{
    return s_error_count;
}
