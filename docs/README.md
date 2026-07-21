# 开发与部署说明

## 项目目标

系统链路：

```text
Camera -> K230 vision -> UART -> MCU PID -> Servo -> Track/Ball
```

K230 负责图像采集、球/轨道检测、位置计算和 UART 测量输出；下位机负责 PID、舵机和闭环安全策略。当前第一阶段不使用 YOLO 或深度学习，只检测黄色轨道，不识别钢球、不发送 UART、不控制舵机。

## 目录结构

```text
K230_Ball_Balance/
├── .agents/skills/k230-canmv-vision-control/SKILL.md
├── docs/
│   ├── README.md
│   ├── mechanical_model.md
│   ├── track-static-test.md
│   └── verified-api-notes.md
├── data/
│   └── servo_rail_calibration.csv
├── tools/
│   └── mechanical_model.py
├── src/
│   ├── main.py
│   ├── config.py
│   ├── vision/
│   ├── communication/
│   ├── control/
│   └── utils/
├── tests/pc/
├── logs/
├── AGENTS.md
├── README.md
└── .gitignore
```

## 当前 Demo

`src/vision/track_detector.py` 调用 `cv_lite.rgb888_find_blobs()` 找最大黄色细长连通区域。当前运行链路不再调用四角点矩形 API。检测成功后显示：

```text
TRACK OK
CX:xxx CY:xxx
ANGLE:x.x
LEN:xxx BLOB
```

洋红框是黄色 Blob 的轴对齐外接框，红线是该框的长边中线。`ANGLE` 目前只能是水平约 0° 或竖直约 -90°，仅用于调阈值，不代表真实轨道倾角。标注图限频保存到 `/sdcard/track_debug.jpg`。

`src/config.py` 中的黄色 RGB 阈值来自项目照片的目测起始值，必须在真实 K230 摄像头、光照和曝光条件下标定：

```python
[R_min, R_max, G_min, G_max, B_min, B_max]
```

## 上板部署

1. 电脑 B 使用 CanMV IDE 连接 Yahboom K230 12Pin。
2. 将 `src/` **内部的文件和目录**复制到 K230 的 `/sdcard/K230_Ball_Balance/`，保持 `vision/`、`communication/`、`control/`、`utils/` 结构。
3. 在 CanMV IDE 打开 `/sdcard/K230_Ball_Balance/main.py`。
4. 按照 `docs/track-static-test.md` 先运行固件自带示例，再运行本项目。
5. 根据实机调整 `TRACK_RGB_THRESHOLD`、`TRACK_MIN_AREA`、`TRACK_KERNEL_SIZE` 和 `TRACK_ROI`。
6. 把成功或失败的 API 现象记录到 `docs/verified-api-notes.md`。

设备端不需要额外安装 Python 包。不要把仓库的 `tests/`、`docs/` 或 `.agents/` 上传为运行代码。

## 电脑 A 检查

电脑 A 只能运行纯逻辑测试，不能导入 `media.*` 或 `cv_lite`：

```powershell
python -m unittest discover -s tests/pc -p "test_*.py"
python -m compileall -q src tests/pc
```

上述成功不代表 CanMV v1.8.0 上板兼容。

## 开发路线

1. 验证 Sensor、LCD、黄色 Blob 和 `/sdcard/track_debug.jpg`。
2. 固定相机并标定黄色阈值、面积、ROI 和 Blob 稳定性，记录 FPS。
3. 开始钢球识别，并验证钢球不会破坏黄色轨道检测。
4. 使用固定端 Marker A 和舵机端 Marker B 建立有向轨道坐标系。
5. 将球心投影到 AB，输出 A=-1、中点=0、B=1 的 `normalized_error`。
6. 加入滤波、有效标志、整数误差和 UART 帧协议。
7. 完成失联、未检出、越界和舵机限幅策略后，再与下位机 PID 联调。

机械结构、一级近似模型、未验证假设和标定计划见 `docs/mechanical_model.md`。PC 端可使用 `tools/mechanical_model.py` 计算理论趋势、读取标定 CSV、拟合简单模型并绘图；该工具和 NumPy/pandas/matplotlib 都不上传到 K230。后续控制器优先采用正反向实测标定结果，而不是完全依赖理论公式。

## Git 协作

电脑 A 完成并验证一个小改动后提交并推送；电脑 B 调试前拉取最新提交。首次远程仓库尚未配置时，不执行 `git push`。
