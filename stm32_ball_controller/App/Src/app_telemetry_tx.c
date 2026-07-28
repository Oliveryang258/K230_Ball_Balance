#include "app_telemetry_tx.h"

static UART_HandleTypeDef *s_uart = 0;
static TelemetrySample s_latest_sample;
static volatile uint8_t s_pending = 0U;
static volatile uint8_t s_busy = 0U;
static uint8_t s_tx_packet[TELEMETRY_PACKET_SIZE];
static uint16_t s_sequence = 0U;
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

bool AppTelemetryTx_Init(UART_HandleTypeDef *huart)
{
    if (huart == 0)
    {
        return false;
    }

    s_uart = huart;
    s_pending = 0U;
    s_busy = 0U;
    s_sequence = 0U;
    s_sent_count = 0U;
    s_overwrite_count = 0U;
    s_error_count = 0U;
    return true;
}

void AppTelemetryTx_Publish(const TelemetrySample *sample)
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

void AppTelemetryTx_Poll(void)
{
    TelemetrySample sample;
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

    TelemetryProtocol_Encode(&sample, s_sequence, s_tx_packet);
    s_sequence++;
    s_busy = 1U;
    status = HAL_UART_Transmit_IT(
        s_uart,
        s_tx_packet,
        TELEMETRY_PACKET_SIZE
    );
    if (status != HAL_OK)
    {
        s_busy = 0U;
        s_error_count++;
    }
}

void AppTelemetryTx_OnTxComplete(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart == s_uart))
    {
        s_busy = 0U;
        s_sent_count++;
    }
}

void AppTelemetryTx_OnError(UART_HandleTypeDef *huart)
{
    if ((s_uart != 0) && (huart == s_uart) && (s_busy != 0U))
    {
        s_busy = 0U;
        s_error_count++;
    }
}

uint32_t AppTelemetryTx_GetSentCount(void)
{
    return s_sent_count;
}

uint32_t AppTelemetryTx_GetOverwriteCount(void)
{
    return s_overwrite_count;
}

uint32_t AppTelemetryTx_GetErrorCount(void)
{
    return s_error_count;
}
