#include "icm20602.h"

#include "main.h"
#include "soft_i2c.h"

/* ICM-20602 官方寄存器地址。 */
#define ICM20602_REG_SMPLRT_DIV        0x19U
#define ICM20602_REG_CONFIG            0x1AU
#define ICM20602_REG_GYRO_CONFIG       0x1BU
#define ICM20602_REG_ACCEL_CONFIG      0x1CU
#define ICM20602_REG_ACCEL_CONFIG2     0x1DU
#define ICM20602_REG_ACCEL_XOUT_H      0x3BU
#define ICM20602_REG_ACCEL_INTEL_CTRL  0x69U
#define ICM20602_REG_USER_CTRL         0x6AU
#define ICM20602_REG_PWR_MGMT_1        0x6BU
#define ICM20602_REG_PWR_MGMT_2        0x6CU
#define ICM20602_REG_I2C_IF            0x70U
#define ICM20602_REG_WHO_AM_I          0x75U

#define ICM20602_WHO_AM_I_VALUE        0x12U
#define ICM20602_ADDRESS_AD0_LOW       0x68U
#define ICM20602_ADDRESS_AD0_HIGH      0x69U

/*
 * 第一版量程：
 * GYRO_CONFIG.FS_SEL  = 01 -> ±500 degree/s -> 65.5 LSB/(degree/s)
 * ACCEL_CONFIG.FS_SEL = 01 -> ±4 g            -> 8192 LSB/g
 */
#define ICM20602_GYRO_CONFIG_500_DPS   0x08U
#define ICM20602_ACCEL_CONFIG_4_G      0x08U
#define ICM20602_GYRO_LSB_PER_DPS      65.5f
#define ICM20602_ACCEL_LSB_PER_G       8192.0f

/*
 * DLPF_CFG=4：
 *   陀螺仪 3 dB 带宽约 20 Hz；
 * A_DLPF_CFG=4：
 *   加速度计 3 dB 带宽约 21.2 Hz。
 *
 * 这是为小车振动环境准备的保守首版，不代表最终控制参数。
 */
#define ICM20602_GYRO_DLPF_20_HZ       0x04U
#define ICM20602_ACCEL_DLPF_21_HZ      0x04U

/*
 * DLPF 开启后内部采样率为 1 kHz：
 * 1000 / (1 + 9) = 100 Hz。
 */
#define ICM20602_SAMPLE_RATE_DIVIDER   9U

#define ICM20602_BURST_LENGTH          14U

static ICM20602Status s_status = ICM20602_STATUS_NOT_INITIALIZED;
static uint8_t s_device_address = 0U;
static uint8_t s_who_am_i = 0U;

static int16_t ICM20602_MakeInt16(uint8_t high_byte, uint8_t low_byte)
{
    uint16_t unsigned_value;

    /*
     * ICM20602 的测量寄存器按“大端”排列：先高字节，再低字节。
     * 组合为 uint16_t 后再转换成 int16_t，保留二进制补码符号。
     */
    unsigned_value = ((uint16_t)high_byte << 8) | (uint16_t)low_byte;
    return (int16_t)unsigned_value;
}

static bool ICM20602_WriteAndVerify(uint8_t register_address,
                                    uint8_t value)
{
    uint8_t read_back = 0U;

    if (!SoftI2C_WriteRegister(s_device_address,
                               register_address,
                               value))
    {
        return false;
    }

    if (!SoftI2C_ReadRegister(s_device_address,
                              register_address,
                              &read_back))
    {
        return false;
    }

    return read_back == value;
}

static bool ICM20602_FindDevice(void)
{
    const uint8_t candidates[2] =
    {
        ICM20602_ADDRESS_AD0_LOW,
        ICM20602_ADDRESS_AD0_HIGH
    };
    uint8_t index;
    uint8_t who_am_i;

    /*
     * AD0 接地时地址为 0x68，接 3.3 V 时地址为 0x69。
     * 自动尝试两个地址，减少第一次接线时的配置负担。
     */
    for (index = 0U; index < 2U; index++)
    {
        who_am_i = 0U;
        if (SoftI2C_ReadRegister(candidates[index],
                                 ICM20602_REG_WHO_AM_I,
                                 &who_am_i))
        {
            s_device_address = candidates[index];
            s_who_am_i = who_am_i;
            if (who_am_i == ICM20602_WHO_AM_I_VALUE)
            {
                return true;
            }
        }
    }

    return false;
}

bool ICM20602_Init(void)
{
    uint8_t who_am_i_after_reset = 0U;

    s_status = ICM20602_STATUS_NOT_INITIALIZED;
    s_device_address = 0U;
    s_who_am_i = 0U;

    if (!SoftI2C_Init())
    {
        s_status = ICM20602_STATUS_I2C_BUS_ERROR;
        return false;
    }

    if (!ICM20602_FindDevice())
    {
        if ((s_who_am_i != 0U) &&
            (s_who_am_i != ICM20602_WHO_AM_I_VALUE))
        {
            s_status = ICM20602_STATUS_BAD_WHO_AM_I;
        }
        else
        {
            s_status = ICM20602_STATUS_DEVICE_NOT_FOUND;
        }
        return false;
    }

    /*
     * DEVICE_RESET=1：恢复芯片默认值。
     * 复位会自动清零该位；等待 100 ms 覆盖陀螺仪最慢启动时间。
     */
    if (!SoftI2C_WriteRegister(s_device_address,
                               ICM20602_REG_PWR_MGMT_1,
                               0x80U))
    {
        s_status = ICM20602_STATUS_CONFIG_ERROR;
        return false;
    }
    HAL_Delay(100U);

    if (!SoftI2C_ReadRegister(s_device_address,
                              ICM20602_REG_WHO_AM_I,
                              &who_am_i_after_reset))
    {
        s_status = ICM20602_STATUS_DEVICE_NOT_FOUND;
        return false;
    }
    s_who_am_i = who_am_i_after_reset;
    if (s_who_am_i != ICM20602_WHO_AM_I_VALUE)
    {
        s_status = ICM20602_STATUS_BAD_WHO_AM_I;
        return false;
    }

    /*
     * PWR_MGMT_1=0x01：
     *   清除 SLEEP，并按手册要求选择自动 PLL 时钟源。
     * PWR_MGMT_2=0x00：
     *   三轴加速度计和三轴陀螺仪全部启用。
     */
    if (!ICM20602_WriteAndVerify(ICM20602_REG_PWR_MGMT_1, 0x01U) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_PWR_MGMT_2, 0x00U))
    {
        s_status = ICM20602_STATUS_CONFIG_ERROR;
        return false;
    }
    HAL_Delay(20U);

    /*
     * USER_CTRL=0：不使用 FIFO。
     * I2C_IF=0：确保没有设置 I2C_IF_DIS，保持 I2C 接口有效。
     * ACCEL_INTEL_CTRL bit1=1：按手册建议上电后避免传感器输出被限幅。
     */
    if (!ICM20602_WriteAndVerify(ICM20602_REG_USER_CTRL, 0x00U) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_I2C_IF, 0x00U) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_ACCEL_INTEL_CTRL, 0x02U) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_CONFIG,
                                 ICM20602_GYRO_DLPF_20_HZ) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_GYRO_CONFIG,
                                 ICM20602_GYRO_CONFIG_500_DPS) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_ACCEL_CONFIG,
                                 ICM20602_ACCEL_CONFIG_4_G) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_ACCEL_CONFIG2,
                                 ICM20602_ACCEL_DLPF_21_HZ) ||
        !ICM20602_WriteAndVerify(ICM20602_REG_SMPLRT_DIV,
                                 ICM20602_SAMPLE_RATE_DIVIDER))
    {
        s_status = ICM20602_STATUS_CONFIG_ERROR;
        return false;
    }

    /*
     * 让数字低通滤波器积累若干样本。
     * 首版初始化在 main 中执行，所以这里允许使用 HAL_Delay；
     * 运行时读取函数本身不使用毫秒级阻塞。
     */
    HAL_Delay(50U);
    s_status = ICM20602_STATUS_OK;
    return true;
}

bool ICM20602_ReadSample(ICM20602Sample *sample)
{
    uint8_t data[ICM20602_BURST_LENGTH];

    if (sample == NULL)
    {
        s_status = ICM20602_STATUS_READ_ERROR;
        return false;
    }

    if ((s_device_address == 0U) ||
        (s_status == ICM20602_STATUS_NOT_INITIALIZED))
    {
        s_status = ICM20602_STATUS_NOT_INITIALIZED;
        return false;
    }

    /*
     * 从 ACCEL_XOUT_H(0x3B) 连续读取 14 字节：
     * accel XYZ(6) + temperature(2) + gyro XYZ(6)。
     *
     * 连续读取能保证一组数据来自同一个寄存器快照，
     * 也比逐个寄存器读取减少大量软件 I2C 开销。
     */
    if (!SoftI2C_ReadRegisters(s_device_address,
                               ICM20602_REG_ACCEL_XOUT_H,
                               data,
                               ICM20602_BURST_LENGTH))
    {
        s_status = ICM20602_STATUS_READ_ERROR;
        return false;
    }

    sample->accel_x_raw = ICM20602_MakeInt16(data[0], data[1]);
    sample->accel_y_raw = ICM20602_MakeInt16(data[2], data[3]);
    sample->accel_z_raw = ICM20602_MakeInt16(data[4], data[5]);
    sample->temperature_raw = ICM20602_MakeInt16(data[6], data[7]);
    sample->gyro_x_raw = ICM20602_MakeInt16(data[8], data[9]);
    sample->gyro_y_raw = ICM20602_MakeInt16(data[10], data[11]);
    sample->gyro_z_raw = ICM20602_MakeInt16(data[12], data[13]);

    sample->accel_x_g =
        (float)sample->accel_x_raw / ICM20602_ACCEL_LSB_PER_G;
    sample->accel_y_g =
        (float)sample->accel_y_raw / ICM20602_ACCEL_LSB_PER_G;
    sample->accel_z_g =
        (float)sample->accel_z_raw / ICM20602_ACCEL_LSB_PER_G;

    sample->gyro_x_dps =
        (float)sample->gyro_x_raw / ICM20602_GYRO_LSB_PER_DPS;
    sample->gyro_y_dps =
        (float)sample->gyro_y_raw / ICM20602_GYRO_LSB_PER_DPS;
    sample->gyro_z_dps =
        (float)sample->gyro_z_raw / ICM20602_GYRO_LSB_PER_DPS;

    /*
     * 官方换算式：
     * temperature(°C) = raw / 326.8 + 25。
     * 温度只用于通信是否正常的辅助检查，首版不参与控制。
     */
    sample->temperature_c =
        ((float)sample->temperature_raw / 326.8f) + 25.0f;

    s_status = ICM20602_STATUS_OK;
    return true;
}

ICM20602Status ICM20602_GetStatus(void)
{
    return s_status;
}

uint8_t ICM20602_GetWhoAmI(void)
{
    return s_who_am_i;
}

uint8_t ICM20602_GetAddress(void)
{
    return s_device_address;
}
