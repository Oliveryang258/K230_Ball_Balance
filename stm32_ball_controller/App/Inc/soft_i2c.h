#ifndef SOFT_I2C_H
#define SOFT_I2C_H

/*
 * STM32F103 软件 I2C 主机驱动
 *
 * 目标硬件：
 *   SCL -> PB6
 *   SDA -> PB7
 *
 * 运行环境：
 *   STM32F103C8T6 + STM32Cube HAL
 *
 * 重要说明：
 * 1. I2C 的 SCL、SDA 必须是开漏输出，并通过上拉电阻拉到 3.3 V。
 * 2. 本驱动使用 7 位从机地址，例如 ICM20602 使用 0x68 或 0x69。
 * 3. 软件 I2C 是阻塞式代码，只能在 main 的普通上下文中调用；
 *    不要在 SysTick、UART 等中断服务函数中调用。
 */

#include <stdbool.h>
#include <stdint.h>

bool SoftI2C_Init(void);
bool SoftI2C_RecoverBus(void);

bool SoftI2C_WriteRegister(uint8_t address_7bit,
                           uint8_t register_address,
                           uint8_t value);

bool SoftI2C_ReadRegister(uint8_t address_7bit,
                          uint8_t register_address,
                          uint8_t *value);

bool SoftI2C_ReadRegisters(uint8_t address_7bit,
                           uint8_t start_register,
                           uint8_t *data,
                           uint16_t length);

#endif
