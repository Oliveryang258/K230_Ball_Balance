#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * K230 正常约 20 Hz 输出一帧视觉结果。
 * 150 ms 相当于连续丢失约 3 帧后进入通信超时。
 * 该值属于第一版保守参数，后续根据实测丢包情况调整。
 */
#define VISION_LINK_TIMEOUT_MS  150U

/* 当前视觉程序的安全像素范围，仅用于接收端二次检查。 */
#define VISION_SAFE_X_MIN       60
#define VISION_SAFE_X_MAX       598

#endif

