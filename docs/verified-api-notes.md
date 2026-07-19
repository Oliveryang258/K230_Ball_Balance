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
| `Sensor.set_auto_gain()` | 已验证 | 方法存在并可调用 |
| `Sensor.get_gain_db()` | 已验证 | 方法存在并可调用 |
| `Sensor.set_auto_exposure()` | 已验证 | 方法存在并可调用 |
| `Sensor.get_exposure_us()` | 已验证 | 方法存在并可调用 |
| `Sensor.get_rgb_gain_db()` | 已验证 | 方法存在并可调用 |
| `Sensor.set_hmirror()` | 已验证 | 方法存在并可调用 |
| `Sensor.set_vflip()` | 已验证 | 方法存在并可调用 |

## 当前代码使用但仍需整程序上板确认

| API/假设 | 状态 | 首次验证重点 |
| --- | --- | --- |
| `Sensor.reset()`, `set_framesize()`, `set_pixformat()`, `run()`, `stop()` | 待验证 | 初始化顺序、RGB888 输出和停止行为 |
| `cv_lite.rgb888_find_blobs()` | 待验证 | 参数顺序、返回列表结构和实时 FPS |
| `cv_lite.rgb888_find_rectangles_with_corners()` | 待验证/可退化 | Yahboom v1.8.0 是否包含；返回是否为每项 12 个数值 |
| 图像 `to_numpy_ref()`, `to_rgb565()` | 待验证 | 格式兼容和内存占用 |
| 图像绘制 API | 待验证 | 参数形式和 RGB565 显示效果 |
| 图像 `save()` | 待验证 | `/sdcard` 挂载、JPEG 支持和写入耗时 |
| `Display.ST7701` 与 800x480 | 待验证 | Yahboom 12Pin LCD 型号和方向 |
| `Display.init()`, `show_image()`, `deinit()` | 待验证 | 初始化参数和资源释放顺序 |
| `MediaManager.init()`, `deinit()` | 待验证 | 与 Sensor/Display 的调用顺序 |
| `os.exitpoint()` | 待验证 | v1.8.0 行为 |
| `time.clock()`, `ticks_ms()`, `ticks_diff()` | 待验证 | FPS 与计时接口 |
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
