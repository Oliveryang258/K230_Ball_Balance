#ifndef SERVO_OUTPUT_H
#define SERVO_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * DS215MG V8.0 舵机PWM输出模块。
 *
 * 硬件前提：
 * - TIM2_CH1 已由 CubeMX 配置到 PA0；
 * - 定时器计数单位为 1 us；
 * - PWM 周期约 3003 us（约 333 Hz）；
 * - 舵机必须由独立 6.0~8.4 V 电源供电，并与 STM32 共地。
 *
 * 当前模块只负责中位、限幅和缓慢变化，不包含 PID。
 */

/* 在 MX_TIM2_Init() 之后调用；成功后开始输出中位PWM。 */
bool ServoOutput_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/* 设置目标脉宽；输入会被强制限制在第一轮台架测试范围内。 */
void ServoOutput_SetTargetPulseUs(uint16_t pulse_us);

/* 将目标值恢复为中位，但仍通过缓变机制逐步回中。 */
void ServoOutput_SetNeutral(void);

/* 主循环重复调用；内部每到一个软件更新周期才改变一次比较值。 */
void ServoOutput_Process(uint32_t now_ms);

/* 停止硬件PWM输出。 */
void ServoOutput_Stop(void);

/* 以下接口用于 Keil Watch 和后续调试。 */
bool ServoOutput_IsStarted(void);
uint16_t ServoOutput_GetCurrentPulseUs(void);
uint16_t ServoOutput_GetTargetPulseUs(void);

#endif

