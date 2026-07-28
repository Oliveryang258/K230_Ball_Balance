# 钢球平衡控制架构

## 当前真实结构

当前系统只有钢球位置这一条反馈链：

```text
K230测得球位置和误差
  → UART V1
  → STM32估计球速度
  → 位置PID计算PWM偏移
  → ServoOutput限幅、slew
  → TIM2输出约333 Hz PWM
  → 舵机和连杆改变轨道角度
```

位置PID在guard为READY时固定以20 ms（50 Hz）运行。`g_apply`只控制
是否把计算结果真正送给舵机，默认仍为0。

## 为什么当前没有“角度内环”

闭环必须有实际反馈量。目前STM32没有轨道角度传感器，也没有实时轨道角度
测量，因此不能把PWM执行层称为角度内环。

当前模块职责：

- `BallController`：唯一闭环，根据球的位置误差、积分和速度计算动作量；
- `ServoOutput`：执行器层，完成PWM硬限幅、20 ms slew和CCR更新；
- 未来的角度—PWM标定表：只能作为两者之间的静态映射层；
- 只有增加IMU、编码器或可靠实时角度测量后，才可能形成真正的角度内环。

现有 `g_kp`、`g_kv` 是直接PWM域的复合经验增益。有可靠角度标定后可以
换算成角度域参数，但不能不换单位就原样使用。

## 当前位置PID

```text
P_us = Kp_us_per_px × error_px

I_us += Ki_us_per_px_s × error_px × measurement_dt_s

D_us = -Kv_us_per_px_s × velocity_px_s

offset_us = direction × (P_us + I_us + D_us)

pwm_target = neutral_us + offset_us
```

I项规则：

- 只在新 `frame_id` 被接受后，按真实UART接收时间差更新一次；
- 重复帧或50 Hz控制任务对旧测量重复运行时不积分；
- 只在 `4 px < |error| ≤ 80 px` 时积分；
- 进入中心死区或离开积分作用区时暂停积分，但保留已有I；
- I项自身按当前配置限制为±40 us；
- PWM已经饱和且新积分继续推向饱和时，撤销本次积分；
- `g_ki=0`时清除I；
- guard不是READY、UART超时、丢球、越界或dt异常时，控制器复位并清除I。

## UART一致性和速度dt

USART1中断在完整有效帧解析完成时记录 `HAL_GetTick()`。
`AppUartRx_TakeSnapshot()`在短临界区内一次复制测量、时间戳、状态和错误计数；
PID运算期间不会关闭中断。

```text
dt_ms = current_receive_tick - previous_receive_tick
velocity = delta_ball_x × 1000 / dt_ms
```

重复 `frame_id` 不更新速度。`dt < 10 ms` 或 `dt > 250 ms` 时丢弃本次
速度更新并清除差分历史。

## Guard定义

| `g_dbg.guard` | 含义 |
| ---: | --- |
| 0 | UART timeout |
| 1 | Ball lost |
| 2 | Position out of range |
| 3 | Invalid dt |
| 4 | Protocol/UART error |
| 5 | Ready |

保护由20 ms任务检查，允许在下一个控制周期内生效，不是零延迟硬件保护。

## 当前PWM限制

配置文件当前值为：

```text
neutral = 1570 us
BALL_CONTROL_PWM_SOFT_RANGE_US = 600 us
闭环计算范围 = 970～2170 us
```

这已经等于人工标定外层范围，不再是此前的保守±40 us闭环范围。
开始调I前应确认这是有意修改并已完成带连杆安全验证。

位置I自身当前限制在±40 us，但P、D和总输出仍可能使用上述完整PWM范围。

## Keil Watch

可调变量：

| 变量 | 含义 |
| --- | --- |
| `g_kp` | 位置P，单位 `us/px` |
| `g_ki` | 位置I，单位 `us/(px*s)` |
| `g_kv` | 速度阻尼D，单位 `us/(px/s)` |
| `g_dir` | 控制方向，只使用1或-1 |
| `g_apply` | 是否应用实际舵机输出 |

`g_dbg`字段：

- `tick`
- `guard`
- `meas_status`
- `x`
- `err`
- `vel`
- `p`
- `i`
- `d`
- `out`
- `i_reset_reason`：最近一次I清零原因，粘滞保存；
- `i_reset_count`：I从非零变为零的累计次数；
- `pwm_target`
- `pwm_now`

`i_reset_reason`定义：

| 值 | 原因 |
| ---: | --- |
| 0 | 上电后尚未记录到I清零 |
| 1 | UART timeout |
| 2 | Ball lost |
| 3 | Position out of range |
| 4 | Invalid dt |
| 5 | Protocol/UART error |
| 6 | `g_ki`被设为0 |
