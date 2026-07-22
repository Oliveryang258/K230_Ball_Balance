# STM32F103C8T6 下位机框架

这里存放球杆控制专用 STM32 的应用层代码。它与未来负责小车循迹的另一块 STM32
相互独立。

当前里程碑只完成：

1. USART1 中断接收；
2. K230 固定长度数据帧解析；
3. 通信超时、视觉无效、位置不安全三类状态区分；
4. 保留后续控制器入口，但**不产生舵机 PWM**。

## 一、在 STM32CubeMX 中创建工程

1. 新建工程，芯片选择 `STM32F103C8Tx`。
2. `System Core > SYS > Debug` 选择 `Serial Wire`。
3. 初次联调先使用内部 HSI 时钟，避免依赖开发板外部晶振型号。
4. 打开 `Connectivity > USART1`，选择 `Asynchronous`。
5. 设置：`115200 bit/s`、`8 data bits`、`None parity`、`1 stop bit`。
6. 确认 USART1 引脚为：`PA9=TX`、`PA10=RX`。
7. 在 `NVIC Settings` 中启用 `USART1 global interrupt`。
8. `Project Manager > Toolchain/IDE` 选择 `MDK-ARM V5`。
9. 建议将 CubeMX 工程生成到仓库的 `stm32/firmware/`；不要覆盖 `stm32/App/`。

目前 K230 到 STM32 只需要：

```text
K230 IO9 / UART1_TXD  ----  STM32 PA10 / USART1_RX
K230 GND              ----  STM32 GND
```

双方是 3.3 V UART。不要把 K230 的 5 V 电源脚接到 STM32 信号脚。

## 二、找到并把 App 层加入 Keil 工程

这些文件不是 CubeMX 自动生成的。Codex 已经把它们创建在仓库中，实际绝对路径为：

```text
C:\Users\32142\Documents\match\stm32\App\Inc
C:\Users\32142\Documents\match\stm32\App\Src
```

其中需要加入 Keil 的三个源文件是：

```text
C:\Users\32142\Documents\match\stm32\App\Src\vision_protocol.c
C:\Users\32142\Documents\match\stm32\App\Src\app_uart_rx.c
C:\Users\32142\Documents\match\stm32\App\Src\control_guard.c
```

### 推荐方式：直接引用仓库外层 App

如果 CubeMX 工程按建议生成在：

```text
C:\Users\32142\Documents\match\stm32\firmware
```

而 Keil 工程文件位于：

```text
C:\Users\32142\Documents\match\stm32\firmware\MDK-ARM
```

那么从 Keil 工程目录到头文件目录的正确相对路径是：

```text
..\..\App\Inc
```

注意这里是两层 `..`，不是之前写的一层。三个 C 文件可以在 Keil 的文件选择窗口中
直接从 `C:\Users\32142\Documents\match\stm32\App\Src` 选择。

### 备选方式：把 App 复制到 firmware 内

如果把整个 `stm32\App` 复制成 `stm32\firmware\App`，此时从 `MDK-ARM` 出发的
头文件路径才是：

```text
..\App\Inc
```

但这种方式会产生两份 App 代码，修改时容易不同步，所以不推荐。

## 三、接入 CubeMX 生成的 main.c

在 `/* USER CODE BEGIN Includes */` 中加入：

```c
#include "app_uart_rx.h"
#include "control_guard.h"
```

在 `MX_USART1_UART_Init();` 之后启动单字节中断接收：

```c
AppUartRx_Init(&huart1);
```

在用户代码区实现 HAL 回调：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    AppUartRx_OnRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    AppUartRx_OnError(huart);
}
```

在 `while (1)` 中只读取状态，暂时不控制舵机：

```c
VisionMeasurement measurement = {0};
ControlGuardState guard_state;

if (AppUartRx_GetLatest(&measurement))
{
    /* 这里表示收到一帧新的、格式和校验均正确的数据。 */
}

guard_state = ControlGuard_Evaluate(
    &measurement,
    AppUartRx_HasPacket(),
    AppUartRx_GetLastPacketTick(),
    HAL_GetTick()
);

/*
 * 现在只在 Keil Watch 窗口观察 guard_state 和 measurement。
 * 不要在这里添加舵机 PWM。
 */
```

## 四、第一轮无连线调试

即使暂时没有杜邦线，也可以先完成：

1. CubeMX 正常生成工程；
2. Keil 无错误编译；
3. 下载空框架到 STM32 后能够进入 `while (1)`；
4. Watch 中看到 `guard_state` 一直为 `CONTROL_GUARD_LINK_TIMEOUT`；
5. 确认没有任何舵机 PWM 输出。

等连线到位后，再验证 UART 接收并记录实测结果。
