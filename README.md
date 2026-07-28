# K230 Ball Balance Vision Control

Yahboom K230 12Pin + CanMV v1.8.0 MicroPython + STM32F103C8T6 钢球平衡视觉控制项目。

K230 采集灰度图像，通过 `cv_lite.grayscale_find_circles()` 检测钢球位置，经 UART 发送给 STM32；STM32 运行 PD 控制器，通过舵机调整轨道角度使钢球保持在物理中心。

## 项目结构

```
K230_Ball_Balance/
├── src/                          # K230 运行代码
│   ├── main.py                   # 入口：视觉循环 + UART 发送
│   ├── config.py                 # 所有可调参数（摄像头、UART、检测、跟踪、滤波）
│   ├── vision/
│   │   ├── ball_detector.py      # 钢球检测器（霍夫圆 + 连续跟踪）
│   │   └── geometry.py           # 像素误差计算、安全区判断
│   ├── communication/
│   │   └── uart.py               # V1 11字节二进制协议编码与发送
│   ├── control/
│   │   └── filter.py             # 一阶指数滤波器
│   └── utils/
│       └── logger.py             # 轻量日志
├── stm32_ball_controller/        # STM32F103C8T6 下位机工程
│   ├── App/Inc/                  # 应用层头文件
│   │   ├── app_config.h          # 控制周期、PWM限制、PD默认参数
│   │   ├── app_uart_rx.h         # UART 中断接收接口
│   │   ├── ball_controller.h     # 球位置 PD 控制器接口
│   │   ├── control_guard.h       # 通信/视觉/安全三级保护接口
│   │   ├── servo_output.h        # 舵机 PWM 输出接口（限幅 + slew）
│   │   ├── vision_protocol.h     # K230 V1 协议解析接口
│   │   ├── telemetry_protocol.h  # 遥测回传协议（待启用）
│   │   └── app_telemetry_tx.h    # 遥测发送接口（待启用）
│   ├── App/Src/                  # 应用层源文件（对应上述头文件）
│   ├── Core/                     # CubeMX 生成的核心代码 + main.c
│   ├── Drivers/                  # HAL 库
│   └── MDK-ARM/                  # Keil MDK-ARM V5 工程文件

├── tools/                        # PC 端辅助工具
│   └── mechanical_model.py       # 机械建模与标定（NumPy/pandas/matplotlib）
├── data/                         # 标定数据模板
│   └── servo_rail_calibration.csv
└── .gitignore
```

## 系统链路

```text
K230 摄像头 → 灰度图 → 霍夫圆检测 → 跟踪+滤波 → UART V1 11字节帧
    ↓
STM32 USART1 中断接收 → 协议解析 → ControlGuard 安全检查
    ↓
BallController PD (50Hz) → ServoOutput 限幅+slew → TIM2 PWM (333Hz)
    ↓
DS215MG 舵机 → 连杆 → 轨道角度 → 钢球位置
```

## K230 视觉 (`src/`)

- **像素格式**：灰度 (`Sensor.GRAYSCALE`)，约 90 FPS（裁剪后）
- **检测**：`grayscale_find_circles()` 在全帧上检测，软件 ROI 过滤
- **跟踪**：基于上一帧位置/半径的候选选择 + 丢帧重置
- **滤波**：一阶指数滤波，alpha 可配
- **ROI 裁剪**：`frame.crop()` 零拷贝视图，CROP_Y=200, CROP_HEIGHT=96
- **UART 输出**：每帧发送（包括 `ball_valid=0` 的无效帧），115200 baud
- **物理中心**：x=361（2026-07-21 三点实测），`error_px = 361 - ball_x`

## STM32 控制 (`stm32_ball_controller/`)

- **MCU**：STM32F103C8T6，内部 HSI 时钟
- **UART**：USART1，单字节中断接收，V1 协议状态机解析
- **控制周期**：20 ms（50 Hz），SysTick 分频
- **控制器**：位置 PD + 可选的积分项（I），死区 4 px
- **PWM**：TIM2 CH1，Prescaler=7，Period=3002，约 333 Hz
- **保护**：UART 超时 (150ms)、丢球、越界、dt 异常、协议错误 → 自动回中位
- **舵机**：DS215MG V8.0，中位 1570 us，闭环软范围 ±600 us
- **调试**：Keil Watch 中展开 `g_dbg` 即可观察全部状态

## 快速开始

### K230 上板

1. CanMV IDE 连接 K230
2. 将 `src/` 内部文件复制到 `/sdcard/K230_Ball_Balance/`（保持子目录结构）
3. 打开 `main.py`，修改顶部 `TUNE_*` 变量调参
4. 运行

### STM32 编译

1. Keil MDK-ARM V5 打开 `stm32_ball_controller/MDK-ARM/Servo_Control.uvprojx`
2. 编译，下载到 STM32F103C8T6
3. 接线：K230 IO9 → STM32 PA10，共地，不接 VCC
4. Keil Watch 中添加 `g_dbg` 观察运行状态

## K230↔STM32 接线

```text
K230 IO9  (UART1_TXD)  →  STM32 PA10 (USART1_RX)
K230 GND               →  STM32 GND
```

双方 3.3V UART。K230 单独 USB-C 供电。不要把 K230 5V 接到 STM32 信号脚。
