#include "soft_i2c.h"

#include "main.h"

/*
 * 第一版固定使用 PB6/PB7。
 *
 * PB6/PB7 同时也是硬件 I2C1 的默认引脚，但本工程没有启用 HAL I2C，
 * 所以这里把它们配置成普通 GPIO 开漏输出，由软件手动产生时序。
 */
#define SOFT_I2C_GPIO_PORT          GPIOB
#define SOFT_I2C_SCL_PIN            GPIO_PIN_6
#define SOFT_I2C_SDA_PIN            GPIO_PIN_7

/*
 * 每个半周期至少等待 5 us。
 * 理想波形约为 100 kHz；由于 HAL GPIO 调用也有开销，实测频率可能更低，
 * 但远低于 ICM20602 支持的 400 kHz 上限，适合首轮连线验证。
 */
#define SOFT_I2C_HALF_PERIOD_US     5U
#define SOFT_I2C_STRETCH_TIMEOUT_US 1000U

static void SoftI2C_DelayUs(uint32_t microseconds)
{
    uint32_t start_cycles;
    uint32_t wait_cycles;
    uint32_t cycles_per_microsecond;

    /*
     * STM32F103 的 Cortex-M3 带 DWT 周期计数器。
     * 用它做微秒延时比空循环更不依赖编译优化等级。
     */
    cycles_per_microsecond = SystemCoreClock / 1000000U;
    if (cycles_per_microsecond == 0U)
    {
        cycles_per_microsecond = 1U;
    }

    wait_cycles = cycles_per_microsecond * microseconds;
    start_cycles = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start_cycles) < wait_cycles)
    {
        /* 仅等待几个微秒，不在这里调用 HAL_Delay。 */
    }
}

static void SoftI2C_SdaLow(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT,
                      SOFT_I2C_SDA_PIN,
                      GPIO_PIN_RESET);
}

static void SoftI2C_SdaRelease(void)
{
    /*
     * 开漏输出写 1 的含义是“释放总线”。
     * 真正的高电平由外部上拉电阻产生。
     */
    HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT,
                      SOFT_I2C_SDA_PIN,
                      GPIO_PIN_SET);
}

static void SoftI2C_SclLow(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT,
                      SOFT_I2C_SCL_PIN,
                      GPIO_PIN_RESET);
}

static void SoftI2C_SclRelease(void)
{
    HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT,
                      SOFT_I2C_SCL_PIN,
                      GPIO_PIN_SET);
}

static bool SoftI2C_ReadSda(void)
{
    return HAL_GPIO_ReadPin(SOFT_I2C_GPIO_PORT,
                            SOFT_I2C_SDA_PIN) == GPIO_PIN_SET;
}

static bool SoftI2C_ReadScl(void)
{
    return HAL_GPIO_ReadPin(SOFT_I2C_GPIO_PORT,
                            SOFT_I2C_SCL_PIN) == GPIO_PIN_SET;
}

static bool SoftI2C_WaitSclHigh(void)
{
    uint32_t waited_us = 0U;

    /*
     * 主机释放 SCL 后，从机理论上可以通过拉低 SCL 延长时钟。
     * 这里设置有限超时，防止接线错误时程序永久卡死。
     */
    while (!SoftI2C_ReadScl())
    {
        if (waited_us >= SOFT_I2C_STRETCH_TIMEOUT_US)
        {
            return false;
        }
        SoftI2C_DelayUs(1U);
        waited_us++;
    }
    return true;
}

static bool SoftI2C_Start(void)
{
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();
    if (!SoftI2C_WaitSclHigh())
    {
        return false;
    }
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);

    /*
     * 总线空闲时 SDA 应为高。
     * 如果仍为低，通常表示没有上拉、SDA 接错或从机卡住。
     */
    if (!SoftI2C_ReadSda())
    {
        return false;
    }

    /* SCL 为高时让 SDA 从高变低，产生 START 条件。 */
    SoftI2C_SdaLow();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SclLow();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    return true;
}

static bool SoftI2C_Stop(void)
{
    bool scl_ok;

    /* SCL 为高时让 SDA 从低变高，产生 STOP 条件。 */
    SoftI2C_SdaLow();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SclRelease();
    scl_ok = SoftI2C_WaitSclHigh();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SdaRelease();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    return scl_ok;
}

static bool SoftI2C_WriteByte(uint8_t value)
{
    uint8_t bit_index;
    bool acknowledged;

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        if ((value & 0x80U) != 0U)
        {
            SoftI2C_SdaRelease();
        }
        else
        {
            SoftI2C_SdaLow();
        }

        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
        SoftI2C_SclRelease();
        if (!SoftI2C_WaitSclHigh())
        {
            SoftI2C_SclLow();
            SoftI2C_SdaRelease();
            return false;
        }
        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
        SoftI2C_SclLow();
        value <<= 1;
    }

    /*
     * 第九个时钟用于 ACK。
     * 主机释放 SDA，从机应把 SDA 拉低表示已经收到该字节。
     */
    SoftI2C_SdaRelease();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SclRelease();
    if (!SoftI2C_WaitSclHigh())
    {
        SoftI2C_SclLow();
        return false;
    }
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    acknowledged = !SoftI2C_ReadSda();
    SoftI2C_SclLow();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    return acknowledged;
}

static bool SoftI2C_ReadByte(uint8_t *value, bool send_ack)
{
    uint8_t bit_index;
    uint8_t received = 0U;

    if (value == NULL)
    {
        return false;
    }

    SoftI2C_SdaRelease();
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        received <<= 1;
        SoftI2C_SclRelease();
        if (!SoftI2C_WaitSclHigh())
        {
            SoftI2C_SclLow();
            return false;
        }
        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
        if (SoftI2C_ReadSda())
        {
            received |= 0x01U;
        }
        SoftI2C_SclLow();
        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    }

    /*
     * 还有后续字节时发送 ACK（SDA=0）；
     * 读取最后一个字节后发送 NACK（SDA=1），再产生 STOP。
     */
    if (send_ack)
    {
        SoftI2C_SdaLow();
    }
    else
    {
        SoftI2C_SdaRelease();
    }

    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SclRelease();
    if (!SoftI2C_WaitSclHigh())
    {
        SoftI2C_SclLow();
        SoftI2C_SdaRelease();
        return false;
    }
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    SoftI2C_SclLow();
    SoftI2C_SdaRelease();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);

    *value = received;
    return true;
}

bool SoftI2C_RecoverBus(void)
{
    uint8_t pulse;

    /*
     * 如果从机在一次未完成的读操作后一直拉低 SDA，
     * 主机额外发送最多 9 个 SCL 脉冲，让从机结束当前字节。
     */
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();
    SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);

    for (pulse = 0U; (pulse < 9U) && !SoftI2C_ReadSda(); pulse++)
    {
        SoftI2C_SclLow();
        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
        SoftI2C_SclRelease();
        if (!SoftI2C_WaitSclHigh())
        {
            return false;
        }
        SoftI2C_DelayUs(SOFT_I2C_HALF_PERIOD_US);
    }

    (void)SoftI2C_Stop();
    return SoftI2C_ReadScl() && SoftI2C_ReadSda();
}

bool SoftI2C_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * 开漏输出不能主动输出高电平，符合 I2C 的“线与”规则。
     * GPIO_PULLUP 仅作为弱上拉辅助；实物仍建议使用模块自带或外接
     * 4.7 kΩ 左右的 SCL/SDA 上拉电阻到 3.3 V。
     */
    gpio_init.Pin = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFT_I2C_GPIO_PORT, &gpio_init);

    SoftI2C_SclRelease();
    SoftI2C_SdaRelease();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    SoftI2C_DelayUs(10U);
    return SoftI2C_RecoverBus();
}

bool SoftI2C_WriteRegister(uint8_t address_7bit,
                           uint8_t register_address,
                           uint8_t value)
{
    bool success = false;

    if (!SoftI2C_Start())
    {
        (void)SoftI2C_RecoverBus();
        return false;
    }

    if (SoftI2C_WriteByte((uint8_t)(address_7bit << 1)) &&
        SoftI2C_WriteByte(register_address) &&
        SoftI2C_WriteByte(value))
    {
        success = true;
    }

    if (!SoftI2C_Stop())
    {
        success = false;
    }
    return success;
}

bool SoftI2C_ReadRegister(uint8_t address_7bit,
                          uint8_t register_address,
                          uint8_t *value)
{
    return SoftI2C_ReadRegisters(address_7bit,
                                 register_address,
                                 value,
                                 1U);
}

bool SoftI2C_ReadRegisters(uint8_t address_7bit,
                           uint8_t start_register,
                           uint8_t *data,
                           uint16_t length)
{
    uint16_t index;
    bool success = false;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    /*
     * 标准寄存器连续读：
     * START -> 地址+写 -> 起始寄存器 -> RESTART -> 地址+读
     *       -> 连续数据 -> NACK -> STOP
     */
    if (!SoftI2C_Start())
    {
        (void)SoftI2C_RecoverBus();
        return false;
    }

    if (!SoftI2C_WriteByte((uint8_t)(address_7bit << 1)) ||
        !SoftI2C_WriteByte(start_register) ||
        !SoftI2C_Start() ||
        !SoftI2C_WriteByte((uint8_t)((address_7bit << 1) | 0x01U)))
    {
        (void)SoftI2C_Stop();
        return false;
    }

    for (index = 0U; index < length; index++)
    {
        bool acknowledge_more_bytes = (index + 1U) < length;
        if (!SoftI2C_ReadByte(&data[index], acknowledge_more_bytes))
        {
            (void)SoftI2C_Stop();
            return false;
        }
    }

    success = SoftI2C_Stop();
    return success;
}
