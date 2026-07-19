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
| `set_auto_exposure()`, `get_gain_db()`, `get_exposure_us()`, `get_rgb_gain_db()` | 与当前官方 API 不一致，先前“已验证”记录缺少可复现证据 | 撤销已验证状态，除非实体板 `dir(sensor)` 和调用结果重新证明 |

## 当前代码使用但仍需整程序上板确认

| API/假设 | 状态 | 首次验证重点 |
| --- | --- | --- |
| `Sensor.stop()` | 待验证 | 正常停止和异常退出时的资源释放行为 |
| `cv_lite.rgb888_find_rectangles_with_corners()` | 待验证/可退化 | Yahboom v1.8.0 是否包含；返回是否为每项 12 个数值 |
| 图像 `save()` | 待验证 | `/sdcard` 挂载、JPEG 支持和写入耗时 |
| `Display.deinit()` | 待验证 | 正常停止和异常退出时的资源释放行为 |
| `MediaManager.deinit()` | 待验证 | 与 Sensor/Display 的释放顺序 |
| `os.exitpoint()` | 待验证 | v1.8.0 行为 |
| `time.ticks_ms()`, `ticks_diff()` | 待验证 | Debug 图片限频计时接口 |
| MicroPython `math.atan2/sqrt/cos/sin/pi` | 待整程序确认 | 四角点方向、长度和中心线计算 |
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

程序启动时会用 `hasattr()` 检查两个 cv_lite 函数。Blob 函数缺失时停止并报告；四角点矩形函数缺失时继续运行，角度来源标记为 `bbox_approx`。

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
