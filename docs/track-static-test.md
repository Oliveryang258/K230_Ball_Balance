# 黄色轨道静态视觉上板测试指南

本指南面向第一次使用 K230/CanMV 的调试者。当前程序只检测黄色轨道；请不要连接舵机控制信号，也不要把钢球识别问题混入本轮测试。

## 1. 为什么采用两级检测

黄色轨道在照片中是一个很长、很窄的亮黄色区域。程序采用：

```text
RGB888 图像
  -> cv_lite 黄色 Blob
  -> 选择 ROI 内最大的细长黄色区域
  -> 尝试匹配带四角点的旋转矩形
  -> 中心、主方向、长度
```

`rgb888_find_blobs()` 的公开返回值只有 `[x, y, w, h]`，不能单独给出旋转角。因此：

- `angle_source=oriented_rect`：四角点矩形成功匹配，角度来自旋转轮廓。
- `angle_source=bbox_approx`：角点 API 缺失或未匹配，角度来自轴对齐外接框，只能作为基础测试近似。

黄色轨道并不是标准实心矩形，而是 U 型槽，因此四角点检测可能偶尔失败。这不代表黄色 Blob 一定失败，要分别观察黄色框和绿色轮廓。

## 2. 先运行 v1.8.0 固件自带示例

公开 CanMV 文档与 Yahboom v1.8.0 定制固件可能有差异。电脑 B 应先在 CanMV IDE/固件示例列表中依次找到并运行：

1. Yahboom“基础例程 → 摄像头显示”，确认 Sensor 和 LCD。
2. Yahboom“CV_lite → 色块识别-彩色图”，确认 `rgb888_find_blobs()`。
3. Yahboom“CV_lite → 矩形识别结合角点识别-彩色图”，确认 `rgb888_find_rectangles_with_corners()`；找不到时记录“API 缺失”，本项目会自动退化。

通用 CanMV 固件中常见的对应文件为：

```text
/sdcard/examples/23-CV_Lite/rgb888_find_blobs.py
/sdcard/examples/23-CV_Lite/rgb888_find_rectangles_with_corners.py
```

文件名可能被 Yahboom 调整，以 IDE 中 v1.8.0 实际示例为准。不要因为示例路径不同而刷入型号不明的通用固件。

参考资料：

- [Yahboom K230 课程目录](https://www.yahboom.com/study/K230)
- [CanMV cv_lite API](https://www.kendryte.com/k230_canmv/v0.7/zh/api/cv_lite/cv_lite.html)
- [CanMV Sensor API](https://www.kendryte.com/k230_canmv/en/main/api/mpp/k230_canmv_sensor_module_api_manual.html)
- [CanMV 图像绘制 API](https://www.kendryte.com/k230_canmv/en/main/api/openmv/image.html)

## 3. 上传文件

把仓库 `src/` 内部内容复制到：

```text
/sdcard/K230_Ball_Balance/
```

设备端应看到：

```text
/sdcard/K230_Ball_Balance/main.py
/sdcard/K230_Ball_Balance/config.py
/sdcard/K230_Ball_Balance/vision/track_detector.py
/sdcard/K230_Ball_Balance/vision/geometry.py
/sdcard/K230_Ball_Balance/utils/logger.py
```

不要只上传 `main.py`，否则模块导入会失败。

## 4. 第一次运行应该看到什么

控制台启动信息应类似：

```text
cv_lite.rgb888_find_blobs=OK
cv_lite.rgb888_find_rectangles_with_corners=OK
[INFO] Yellow track static test started
```

若角点 API 不存在，第二行会显示：

```text
cv_lite.rgb888_find_rectangles_with_corners=MISSING, use bbox_approx
```

这不是程序崩溃，仍可继续测试黄色 Blob。

检测成功时控制台限频输出：

```text
track_valid=1 center_x=320 center_y=240 angle=2.50 length=520.0 angle_source=oriented_rect fps=18.0
```

检测失败时：

```text
track_valid=0 center_x=-1 center_y=-1 angle=0.00 length=0.0 angle_source=none fps=20.0
```

LCD 标记含义：

- 蓝框：配置的 ROI。
- 洋红框：最大黄色 Blob 外接框；使用高对比颜色避免与黄色轨道混在一起。
- 绿框：仅在 `angle_source=oriented_rect` 时显示，表示角点 API 匹配的旋转轮廓。
- 红线：轨道主方向。
- 白色十字：黄色区域中心。

## 5. 黄色阈值怎么调

### CanMV IDE 只改 `main.py` 的现场调参方式

`src/main.py` 顶部提供了 `FIELD_TUNING_ENABLED` 和一组 `TUNE_TRACK_*`
参数。启用时，程序会在创建检测器之前用这些值临时覆盖 `config.py`，因此现场
反复调参只需修改并重新运行 `main.py`，不必反复上传其他模块。

参数稳定后，应把最终值回填到 `src/config.py`，并设置：

```python
FIELD_TUNING_ENABLED = False
```

这样长期配置仍由 `config.py` 统一管理。

初始值位于 `src/config.py`：

```python
TRACK_RGB_THRESHOLD = [130, 255, 90, 255, 0, 150]
```

顺序是 `[R_min, R_max, G_min, G_max, B_min, B_max]`。一次只改一个方向，每次改完记录现象：

| 现象 | 优先调整 |
| --- | --- |
| 黄色轨道完全没有框 | 降低 `R_min`、降低 `G_min`，或略增 `B_max` |
| 只检测到轨道最亮部分 | 小幅降低 `R_min` 和 `G_min` |
| 白色桌面也被选中 | 降低 `B_max` |
| 红色/橙色物体混入 | 提高 `G_min` |
| 绿色物体混入 | 提高 `R_min` |
| 很多小黄点 | 增大 `TRACK_MIN_AREA`，或把 `TRACK_KERNEL_SIZE` 从 1 调到 3 |
| 轨道外黄色物体更大 | 缩小 `TRACK_ROI`，只覆盖轨道运动区域 |

当前 ROI 是全画面。相机固定后，先观察轨道在 640x480 图像中的坐标，再把 ROI 缩小，例如：

```python
TRACK_ROI = (20, 100, 600, 280)
```

本项目当前对 Blob 结果做软件 ROI 筛选；`rgb888_find_blobs()` 仍处理整帧，因为公开 API 没有 ROI 参数。

## 6. 角度怎么判断是否可信

让轨道静止，连续观察 20~30 次输出：

1. `track_valid` 应大部分为 1。
2. `center_x/center_y` 不应大幅跳动。
3. `angle_source` 最好稳定为 `oriented_rect`。
4. 轨道轻微转动后，`angle` 应同方向变化。
5. 若一直是 `bbox_approx`，角度只适合判断“横向/纵向”，暂不用于控制。

注意：主轴角度没有左右方向，规范范围约为 `[-90, 90)`；0 度表示画面水平，正负号受图像 y 轴向下以及镜像/翻转配置影响。此阶段只验证稳定性，不把角度直接发送给舵机。

## 7. Debug 图片

检测有效时，程序每隔至少 5 秒覆盖保存：

```text
/sdcard/track_debug.jpg
```

如果保存失败，控制台会打印错误，但视觉循环继续运行。不要把保存间隔改成每帧，否则会严重降低 FPS 并增加存储写入次数。

## 8. 测试结束后记录

至少记录：固件版本、启动能力信息、阈值、ROI、光照、LCD 是否正常、`angle_source`、FPS、异常全文，并填入 `docs/verified-api-notes.md` 的模板。只有实机成功后才能把 API 状态改成“已验证”。
