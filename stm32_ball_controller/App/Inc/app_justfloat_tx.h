#ifndef APP_JUSTFLOAT_TX_H
#define APP_JUSTFLOAT_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

/*
 * VOFA+ JustFloat 固定 24 通道。
 * 每个通道是小端 IEEE-754 float32，结尾固定 00 00 80 7F。
 */
#define JUSTFLOAT_CHANNEL_COUNT  24U
#define JUSTFLOAT_PACKET_SIZE    ((JUSTFLOAT_CHANNEL_COUNT * 4U) + 4U)

typedef struct
{
    float time_s;             /* 0  STM32 本地时间，s */
    float ball_x;             /* 1  钢球 x，px */
    float target_x;           /* 2  目标 x，px */
    float error_px;           /* 3  位置误差，px */
    float velocity_px_s;      /* 4  钢球速度，px/s */
    float p_term_us;          /* 5  P 输出，us */
    float i_term_us;          /* 6  I 输出，us */
    float d_term_us;          /* 7  D/速度输出，us */
    float control_out_us;     /* 8  总控制偏移，us */
    float pwm_target_us;      /* 9  PWM 目标，us */
    float pwm_now_us;         /* 10 当前 PWM，us */
    float guard;              /* 11 保护状态 */
    float motion_link_valid;  /* 12 CH32 链路在超时内为 1 */
    float motion_state;       /* 13 CH32 运动状态 */
    float motion_flags;       /* 14 CH32 标志位 */
    float acc_track_mg;       /* 15 轨道方向加速度，mg */
    float yaw_rate_dps;       /* 16 横摆角速度，deg/s */
    float vibration_mg;       /* 17 高频振动指标，mg */
    float line_error;         /* 18 循迹误差 */
    float left_speed;         /* 19 左轮速度 */
    float right_speed;        /* 20 右轮速度 */
    float turn_command;       /* 21 转向命令 = right-left */
    float motion_sequence;    /* 22 CH32 帧序号 */
    float motion_age_ms;      /* 23 最新车辆帧年龄，ms */
} JustFloatSample;

bool AppJustFloatTx_Init(UART_HandleTypeDef *huart);

/* 可从 50 Hz SysTick 控制任务调用，只复制最新快照，不启动串口。 */
void AppJustFloatTx_Publish(const JustFloatSample *sample);

/* 必须在主循环连续调用，使用 HAL_UART_Transmit_IT 非阻塞发送。 */
void AppJustFloatTx_Poll(void);

void AppJustFloatTx_OnTxComplete(UART_HandleTypeDef *huart);
void AppJustFloatTx_OnError(UART_HandleTypeDef *huart);

uint32_t AppJustFloatTx_GetSentCount(void);
uint32_t AppJustFloatTx_GetOverwriteCount(void);
uint32_t AppJustFloatTx_GetErrorCount(void);

#endif
