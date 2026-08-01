#ifndef ICM20602_H
#define ICM20602_H

/*
 * ICM-20602 六轴 IMU 驱动（软件 I2C 版本）
 *
 * 第一版目标：
 * 1. 识别芯片；
 * 2. 完成基础量程、采样率和低通滤波配置；
 * 3. 连续读取三轴加速度、温度和三轴角速度；
 * 4. 同时提供原始 ADC 值和已换算物理量，方便 Keil Watch 验证。
 *
 * 本模块当前只负责“测量”，不直接参与钢球控制和舵机输出。
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ICM20602_STATUS_NOT_INITIALIZED = 0,
    ICM20602_STATUS_OK,
    ICM20602_STATUS_I2C_BUS_ERROR,
    ICM20602_STATUS_DEVICE_NOT_FOUND,
    ICM20602_STATUS_BAD_WHO_AM_I,
    ICM20602_STATUS_CONFIG_ERROR,
    ICM20602_STATUS_READ_ERROR
} ICM20602Status;

typedef struct
{
    /* 寄存器直接读出的 16 位有符号原始数据。 */
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t temperature_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    /*
     * 按第一版量程换算后的物理量：
     * 加速度：g
     * 角速度：degree per second（度/秒）
     * 温度：摄氏度
     */
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} ICM20602Sample;

bool ICM20602_Init(void);
bool ICM20602_ReadSample(ICM20602Sample *sample);

ICM20602Status ICM20602_GetStatus(void);
uint8_t ICM20602_GetWhoAmI(void);
uint8_t ICM20602_GetAddress(void);

#endif
