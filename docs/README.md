# 开发与部署说明

## 项目目标

系统链路：

```text
Camera -> K230 vision -> UART -> MCU PID -> Servo -> Track/Ball
```

K230 负责图像采集、球/轨道检测、位置计算和 UART 测量输出；下位机负责 PID、舵机和闭环安全策略。当前使用固定俯拍相机测试钢球检测，不使用YOLO或深度学习，不发送UART，也不控制舵机。

## 目录结构

```text
K230_Ball_Balance/
├── .agents/skills/k230-canmv-vision-control/SKILL.md
├── docs/
│   ├── README.md
│   ├── mechanical_model.md
│   ├── ball-static-test.md
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

`src/vision/ball_detector.py`调用`cv_lite.rgb888_find_circles()`，在固定轨道ROI内寻找半径合适的钢球圆，并利用上一有效帧的位置和半径排除跳变候选。该圆检测接口和基础参数已经在Yahboom v1.8.0实机验证；新增的连续跟踪和滤波参数仍需动态实测。检测成功后显示：

```text
BALL OK
RAW:xxx FIL:xxx
ERRPX:+xxx
R:xx
```

蓝框是软件ROI，两条青色竖线是软件安全边界，黄色竖线是实测轨道物理中心。绿色/橙色框分别表示安全/不安全的有效圆，红色十字是原始圆心，紫色小十字是滤波位置，`R`是半径。`ERRPX`采用`361-filtered_x`：负值表示偏向画面右侧固定端，正值表示偏向画面左侧舵机端。标注图限频保存到`/sdcard/ball_debug.jpg`。

当前圆检测实测参数为`dp=1`、`minDist=30`、`param1=80`、`param2=20`、半径`8～35`像素；后续只有在视角、距离或光照改变时才重新标定。

## 上板部署

1. 电脑 B 使用 CanMV IDE 连接 Yahboom K230 12Pin。
2. 将 `src/` **内部的文件和目录**复制到 K230 的 `/sdcard/K230_Ball_Balance/`，保持 `vision/`、`communication/`、`control/`、`utils/` 结构。
3. 在 CanMV IDE 打开 `/sdcard/K230_Ball_Balance/main.py`。
4. 按照 `docs/ball-static-test.md` 运行本项目。
5. 只需打开`main.py`，调整顶部的`TUNE_BALL_*`现场参数。
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

1. 固定相机，验证钢球圆检测ROI、半径范围和检测稳定性。
2. 在当前约20 cm可见区间内验证球心输出连续、单调且没有明显误检。
3. 改善安装或视野，标定完整有效轨道两端及机械方向。
4. 加入滤波、有效标志、整数误差和UART帧协议。
5. 完成失联、未检出、越界和舵机限幅策略后，再与下位机PID联调。

机械结构、一级近似模型、未验证假设和标定计划见 `docs/mechanical_model.md`。PC 端可使用 `tools/mechanical_model.py` 计算理论趋势、读取标定 CSV、拟合简单模型并绘图；该工具和 NumPy/pandas/matplotlib 都不上传到 K230。后续控制器优先采用正反向实测标定结果，而不是完全依赖理论公式。

## Git 协作

电脑 A 完成并验证一个小改动后提交并推送；电脑 B 调试前拉取最新提交。首次远程仓库尚未配置时，不执行 `git push`。
