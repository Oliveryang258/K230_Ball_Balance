# 黄色轨道静态视觉上板测试指南

当前程序只检测黄色轨道；不要连接舵机控制信号，也不要把钢球识别混入本轮测试。

## 1. 当前检测逻辑

```text
RGB888 图像
  -> cv_lite.rgb888_find_blobs()
  -> 选择 ROI 内最大的细长黄色区域
  -> 黄色 Blob 中心、轴对齐外接框和长边中线
```

四角点矩形检测已经在独立官方例程中验证过，但它对拍摄距离、U 型槽边缘和逐帧波动敏感，当前设备程序不再调用它。

因此当前 `angle` 只是 Blob 轴对齐外接框的方向：横向约为 0°，竖向约为 -90°。它适合确认阈值是否找对轨道，不是可用于闭环控制的真实倾角。后续由固定端 Marker A 与舵机端 Marker B 建立有向轨道轴。

## 2. 先验证固件基础例程

电脑 B 应先在 Yahboom CanMV v1.8.0 的 IDE/固件示例中运行：

1. “摄像头显示”，确认 Sensor 与 LCD。
2. “CV_lite 色块识别-彩色图”，确认 `rgb888_find_blobs()`。

通用 CanMV 固件中常见文件为：

```text
/sdcard/examples/23-CV_Lite/rgb888_find_blobs.py
```

文件名可能被 Yahboom 调整，以设备中 v1.8.0 的实际示例为准，不要因此刷入型号不明的通用固件。

## 3. 上传与运行

把仓库 `src/` 内部内容整体复制到：

```text
/sdcard/K230_Ball_Balance/
```

至少应包括：

```text
/sdcard/K230_Ball_Balance/main.py
/sdcard/K230_Ball_Balance/config.py
/sdcard/K230_Ball_Balance/vision/track_detector.py
/sdcard/K230_Ball_Balance/utils/logger.py
```

不要只上传 `main.py`，否则 `config`、`vision` 等模块会导入失败。打开该目录下的 `main.py` 运行。

启动控制台应包含：

```text
field_tuning=ON
tune_threshold=[...]
tune_roi=(...)
tune_min_area=... kernel=... mode=blob_only
cv_lite.rgb888_find_blobs=OK
track_axis_mode=bbox_only
```

检测成功：

```text
track_valid=1 center_x=320 center_y=240 angle=0.00 length=520.0 axis_source=bbox_only fps=23.0
```

检测失败：

```text
track_valid=0 center_x=-1 center_y=-1 angle=0.00 length=0.0 axis_source=none fps=23.0
```

LCD 标记：

- 蓝框：软件 ROI。
- 洋红框：最大黄色 Blob 的轴对齐外接框。
- 红线：该外接框的长边中线，仅作调试参考。
- 白色十字：黄色 Blob 外接框中心。

## 4. 只改 main.py 现场调参

`src/main.py` 顶部的 `FIELD_TUNING_ENABLED` 与 `TUNE_TRACK_*` 会在启动时临时覆盖 `config.py`，所以 CanMV IDE 现场调参只需修改 `main.py`。

主要参数：

```python
TUNE_TRACK_RGB_THRESHOLD = [130, 255, 90, 255, 0, 150]
TUNE_TRACK_MIN_AREA = 1200
TUNE_TRACK_KERNEL_SIZE = 1
TUNE_TRACK_ROI = (0, 0, 640, 480)
TUNE_TRACK_MIN_BBOX_ASPECT_RATIO = 2.5
```

RGB 阈值顺序为 `[R_min, R_max, G_min, G_max, B_min, B_max]`。

| 现象 | 优先调整 |
| --- | --- |
| 完全没有框 | 降低 `R_min`、`G_min`，或略增 `B_max` |
| 只检出最亮部分 | 小幅降低 `R_min` 与 `G_min` |
| 白色区域混入 | 降低 `B_max` |
| 红色/橙色物体混入 | 提高 `G_min` |
| 绿色物体混入 | 提高 `R_min` |
| 很多小黄点 | 增大 `MIN_AREA`，或把 `KERNEL_SIZE` 从 1 改为 3 |
| 其他黄色物体被选中 | 缩小 ROI，只覆盖轨道运动区域 |

参数稳定后，把最终值回填到 `src/config.py`，再将：

```python
FIELD_TUNING_ENABLED = False
```

## 5. 当前阶段通过标准

固定轨道与相机，连续观察至少 30 秒：

1. `track_valid` 大部分时间为 1。
2. 洋红框持续覆盖黄色轨道主体，不把大块背景选进来。
3. `center_x/center_y` 静止时只有小幅跳动。
4. FPS 能满足后续实时调试。
5. 改变光照或轻微移动相机后，记录需要重调的阈值范围。

当前红线长度随轴对齐外接框变化是正常的；不要把它当作真实轨道端点。后续 Marker A/B 的中心连线才是控制计算使用的轨道轴。

## 6. Debug 图片与记录

检测有效时，程序按限频覆盖保存：

```text
/sdcard/track_debug.jpg
```

不要改成每帧保存，否则会降低 FPS 并增加存储写入。测试后至少记录：固件版本、Git 提交、阈值、ROI、拍摄距离、光照、LCD 现象、FPS 与异常全文。
