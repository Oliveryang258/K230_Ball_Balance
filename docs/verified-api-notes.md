# CanMV v1.8.0 已验证 API 记录

目标设备：Yahboom K230 12Pin AI Vision Module  
固件：CanMV K230 Yahboom v1.8.0  
运行时：MicroPython

只有在实体 K230 上观察到结果后，才能把状态标记为“已验证”。电脑 A 的 CPython 检查不计入设备验证。

## 已由项目成员确认

| 模块/API | 状态 | 已知结果 |
| --- | --- | --- |
| `os` | 已验证 | 模块可导入 |
| `sys` | 已验证 | 模块可导入 |
| `aidemo` | 已验证 | 模块可导入 |
| `nncase_runtime` | 已验证 | 模块可导入 |
| `cv_lite` | 已验证 | 模块可导入 |
| `media.sensor.Sensor` | 已验证 | 类可导入 |
| `Sensor.snapshot()` | 已验证 | 方法存在并可调用 |
| `Sensor.set_hmirror()` | 已验证 | 方法存在并可调用 |
| `Sensor.set_vflip()` | 已验证 | 方法存在并可调用 |
| `Sensor.reset()`, `set_framesize()`, `set_pixformat()`, `run()` | 已验证 | 黄色轨道程序已连续取得 640x480 RGB888 图像 |
| `Sensor.auto_exposure()` | 已验证 | 在 `run()` 前调用后程序可正常进入采集循环 |
| `cv_lite.rgb888_find_blobs()` | 已验证 | 实机已检出黄色轨道并返回可用外接框 |
| `cv_lite.rgb888_find_rectangles_with_corners()` | 已验证 | 删除例程中不兼容的增益设置后，可检测标准矩形并返回外接框和四角点；对距离和逐帧边缘波动敏感 |
| 图像 `to_numpy_ref()`, `to_rgb565()` | 已验证 | cv_lite 输入和 LCD RGB565 显示链路正常 |
| `draw_line()`, `draw_rectangle()`, `draw_cross()`, `draw_string_advanced()` | 已验证 | 实机画面已显示框、中心线、十字和状态文字 |
| `Display.ST7701`, `Display.init()`, `show_image()` | 已验证 | Yahboom 12Pin LCD 已正常显示检测画面 |
| `MediaManager.init()` | 已验证 | Sensor 与 Display 媒体链路能够启动 |
| `time.clock()` | 已验证 | 实机画面 FPS 约 23 |

## 实机纠正与版本差异

| API | 实机/文档结果 | 项目处理 |
| --- | --- | --- |
| `Sensor.set_auto_gain()` | Yahboom v1.8.0 实机报告方法不存在；当前官方 Sensor API 也未列出 | 已从运行代码删除，禁止继续按旧接口生成 |
| `Sensor.auto_exposure(enable)` | 当前官方 API 名称；要求在 `sensor.run()` 前调用 | 已用于当前代码，仍需在本机继续运行确认 |
| `Sensor.again([value])` | 当前官方手动模拟增益接口，仅部分传感器支持；设置增益要求传感器已经运行 | 第一阶段不调用 |
| Yahboom矩形例程中的 `sensor.again(k_sensor_gain对象)` | v1.8.0 实机报错 `cannot convert struct to float` | 例程与固件绑定签名不一致；删除可选增益段后再验证矩形接口，不向 `again()` 传结构体 |
| `set_auto_exposure()`, `get_gain_db()`, `get_exposure_us()`, `get_rgb_gain_db()` | 与当前官方 API 不一致，先前“已验证”记录缺少可复现证据 | 撤销已验证状态，除非实体板 `dir(sensor)` 和调用结果重新证明 |

## 当前代码使用但仍需整程序上板确认

| API/假设 | 状态 | 首次验证重点 |
| --- | --- | --- |
| `Sensor.stop()` | 待验证 | 正常停止和异常退出时的资源释放行为 |
| 图像 `save()` | 待验证 | `/sdcard` 挂载、JPEG 支持和写入耗时 |
| `Display.deinit()` | 待验证 | 正常停止和异常退出时的资源释放行为 |
| `MediaManager.deinit()` | 待验证 | 与 Sensor/Display 的释放顺序 |
| `os.exitpoint()` | 待验证 | v1.8.0 行为 |
| `time.ticks_ms()`, `ticks_diff()` | 待验证 | Debug 图片限频计时接口 |
| `machine.UART` 引脚和 UART ID | 未启用 | 先确定 12Pin 板引脚复用和电平 |

## 本轮代码所依据的示例/文档

以下资料用于确认 API 名称和参数，但不能代替 Yahboom v1.8.0 实机验证：

| API | 依据 |
| --- | --- |
| `Sensor`, RGB888, `snapshot()` | CanMV Sensor API 和 Sensor 示例 |
| `cv_lite.rgb888_find_blobs()` | CanMV `23-CV_Lite/rgb888_find_blobs.py` 文档示例；Yahboom“色块识别-彩色图”课程 |
| `cv_lite.rgb888_find_rectangles_with_corners()` | CanMV 四角点矩形文档示例；Yahboom“矩形识别结合角点识别-彩色图”课程 |
| `draw_line()`, `draw_rectangle()`, `draw_cross()`, `draw_string_advanced()` | CanMV 图像绘制 API/例程 |
| `Display.ST7701`, `Display.show_image()` | CanMV 800x480 LCD Display 示例 |

当前轨道程序启动时只用 `hasattr()` 检查 `rgb888_find_blobs()`；缺失时停止并报告。四角点矩形 API 虽已在独立例程中实机验证，但因对距离和边缘波动敏感，已从当前运行链路删除。

## 实机验证记录模板

```text
日期：
提交：
测试人/电脑：
固件：CanMV K230 Yahboom v1.8.0
分辨率：
光照与球颜色：
测试 API：
参数：
观察结果：
FPS/异常：
结论：已验证 / 失败 / 需复测
```

## 2026-07-21 steel-ball circle test

- Firmware: `CanMV v1.8-0-gc2d1f5c`, board string `k230_canmv_yahboom`, sensor `gc2093_csi2`.
- `cv_lite.rgb888_find_circles()` is verified present and callable on the physical device.
- Verified parameters at 640x480 RGB888: `dp=1`, `minDist=30`, `param1=80`, `param2=20`, `minRadius=8`, `maxRadius=35`.
- Observed ball radius: mainly 13-20 pixels; stable FPS: about 20.5.
- Fixed ROI `(20,205,610,90)` retained left/centre/right ball detections and rejected an out-of-ROI circle `[114,192,20]`.
- Empty track produced zero valid ROI circles during the recorded test.
- The successful standalone script passed all six circle-control parameters as
  integers.  An integrated revision that converted `dp=1` to `dp=1.0` produced
  `TypeError: can't convert float to int` on the device.  Runtime code now keeps
  `dp`, `minDist`, `param1`, `param2`, `minRadius`, and `maxRadius` as integers.
  The corrected integrated program was subsequently verified at about 21 FPS.

## 2026-07-21 three-point position calibration

- Camera-right corresponds to the mechanical fixed-pivot end; camera-left
  corresponds to the servo-driven end.
- Stable centres were approximately: camera-right extreme `(625,255)`, physical
  track centre `(361,254)`, and camera-left extreme `(24,256)`.
- The physical centre is intentionally not the image centre because the current
  camera mount is offset. Runtime control error is therefore `361 - ball_x`,
  without asymmetric left/right normalization.
- Provisional software-safe centre range is `60..598`; the detected extremes are
  too close to the image boundary for closed-loop use.

## Pending dynamic tracking validation

- The runtime now adds pure-MicroPython candidate continuity checks and an
  allocation-light exponential filter after the already verified cv_lite call.
- Initial unverified values are: expected radius `17`, per-frame jump limit
  `80 px`, radius-change limit `8 px`, reset after `3` missed frames, and filter
  alpha `0.5`.
- These values are PC-logic tested only. They must not be marked verified until
  free-rolling and vibration tests are observed on the physical K230.

## 2026-07-21 Yahboom 12Pin UART pin inspection

- Firmware: `CanMV v1.8-0-gc2d1f5c`, board string `k230_canmv_yahboom`.
- `machine.UART` exposes `UART1`, `UART2`, and `UART4` constants on this firmware.
- `machine.FPIOA` exposes `UART2_TXD` and `UART2_RXD`, but this does **not** mean
  that the Yahboom 4Pin connector is routed to UART2.
- Physical-board inspection through `FPIOA.get_pin_func()` and `help()` showed:
  - IO9 current function: `UART1_TXD` (internal function value `178`).
  - IO9 supported functions include `UART1_TXD`.
  - IO10 current function: `GPIO10` (internal function value `10`).
  - IO10 supported functions include `UART1_RXD`.
- Therefore the Yahboom 12Pin module's IO9/IO10 communication pair must use
  `UART1`, not `UART2`. The initial one-way test only needs IO9 as TX.
- UART construction and physical byte transmission remain unverified until the
  required 3.3 V wiring is available.
