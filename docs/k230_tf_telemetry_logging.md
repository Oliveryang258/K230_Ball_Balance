# K230 TF卡黑匣子数据记录

## 目标

在不增加上位机、HC-06或额外STM32引脚的情况下，利用现有的
K230 ↔ STM32 USART1双向连接记录整车运行数据：

```text
K230 IO9 / UART1_TX  -> STM32 PA10 / USART1_RX：视觉测量
K230 IO10 / UART1_RX <- STM32 PA9  / USART1_TX：控制与车辆遥测
K230 GND             --- STM32 GND：共地
```

不要连接两端VCC。记录介质是插入K230的TF卡，文件路径为
`/sdcard/ball_run.bin`，不是反复擦写K230内部Flash。

## 为什么使用二进制整块写入

STM32控制任务以50 Hz产生一条固定64字节遥测记录：

- 原始数据率：`50 × 64 = 3200 byte/s`；
- K230预先分配1024字节写缓冲区；
- 累计16条记录后写卡一次；
- 写卡周期约为`16 / 50 = 0.32 s`。

K230不在视觉循环中拼接CSV字符串，也不逐帧刷新文件。这样能显著减少
MicroPython对象分配和文件系统调用。TF卡一次写入仍可能造成短暂抖动，
因此必须通过实机比较开启和关闭记录时的FPS和丢帧情况。

## STM32回传帧 V2

所有多字节整数均为小端序，帧长固定64字节：

| 偏移 | 类型 | 含义 |
|---:|---|---|
| 0..1 | `u8[2]` | 帧头`54 4D`（ASCII `TM`） |
| 2 | `u8` | 协议版本2 |
| 3 | `u8` | 帧长64 |
| 4..5 | `u16` | 遥测帧序号 |
| 6..9 | `u32` | STM32毫秒tick |
| 10..11 | `u16` | 最近视觉frame_id |
| 12..13 | `i16` | 球心x |
| 14..15 | `i16` | 位置误差px |
| 16..17 | `i16` | 滤波速度px/s |
| 18..25 | `i16 × 4` | P、I、D、控制输出，单位us |
| 26..29 | `u16 × 2` | 舵机目标/当前脉宽us |
| 30..32 | `u8 × 3` | guard、测量状态、控制flags |
| 33..35 | `u8 × 3` | 车辆状态、车辆flags、车辆链路有效 |
| 36..37 | `i16` | 轨道方向加速度mg |
| 38..39 | `i16` | 横摆角速度0.1 deg/s |
| 40..41 | `u16` | 振动指标mg |
| 42..49 | `i16 × 4` | 线误差、左右轮速、转向命令 |
| 50..53 | `u16 × 2` | 车辆帧序号、车辆帧年龄ms |
| 54..61 | `u8[8]` | 保留为0 |
| 62..63 | `u16` | CRC-16/CCITT-FALSE，覆盖字节2..61 |

STM32的USART1发送使用中断发送。50 Hz控制路径只发布一致性快照，
实际串口启动和发送在主循环完成，不在控制中断里等待。

## K230配置

在`src/config.py`中：

```python
UART_ENABLED = True
TELEMETRY_LOG_ENABLED = True
TELEMETRY_LOG_PATH = "/sdcard/ball_run.bin"
TELEMETRY_RX_BUFFER_SIZE = 256
TELEMETRY_WRITE_BUFFER_SIZE = 1024
TELEMETRY_SYNC_INTERVAL_BLOCKS = 4
```

每次启动会覆盖旧的`ball_run.bin`。正式跑圈前应先把上一轮文件下载并
改名。若没有TF卡或打开失败，控制台会显示`telemetry_log=OFF`，视觉和
K230→STM32测量仍继续运行。

## 第一次联调步骤

1. 给STM32烧录当前Keil工程。
2. 将更新后的`src/main.py`、`src/config.py`、
   `src/communication/uart.py`和
   `src/communication/telemetry_logger.py`保持目录结构复制到K230。
3. 插入TF卡，按上述TX/RX交叉接线并共地。
4. 先让车辆架空或静止运行10秒，观察控制台：
   - `telemetry active=1`；
   - `frames`持续增加；
   - `seq`持续增加；
   - `crc_err=0`、`fmt_err=0`、`write_err=0`。
5. 比较开启/关闭`TELEMETRY_LOG_ENABLED`时的K230 FPS。若持续下降超过
   约5 FPS或出现明显视觉丢帧，先换速度更好的TF卡，再考虑把写缓冲扩大
   到2048字节。
6. 用IDE正常停止脚本，使最后不足16条的尾部数据写入文件，然后下载
   `/sdcard/ball_run.bin`。

脱机运行时每写满4个1 KB块会执行一次文件系统同步，所以直接断电最多
丢失最近约1.28秒及尚未装满的数据，而此前检查点应当保留。第一次测试
仍建议用IDE正常停止，确认记录链路和TF卡性能后再进行脱机跑圈。

## PC转换CSV

在仓库根目录运行：

```powershell
python tools/decode_k230_telemetry.py ball_run.bin ball_run.csv
```

该工具仅使用PC Python标准库，不需要NumPy、pandas或matplotlib。生成的
CSV可导入VOFA+、Excel或后续Python分析工具。
