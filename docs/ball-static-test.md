# 钢球圆检测上板测试指南

## 当前结论

深色Blob方案会把绿色底板、轨道阴影等连成巨大区域，不再作为正式钢球检测链路。当前使用：

```text
RGB888图像
  -> cv_lite.rgb888_find_circles()
  -> 圆心ROI过滤
  -> 位置/半径连续跟踪
  -> 指数滤波
  -> 原始位置、滤波位置、像素误差和软件安全区状态
```

该API已在Yahboom CanMV v1.8.0实机确认存在并成功运行。当前实测稳定帧率约20.5 FPS。

## 已验证起始参数

```python
TUNE_BALL_ROI = (10, 205, 620, 90)
TUNE_CIRCLE_DP = 1
TUNE_CIRCLE_MIN_DIST = 30
TUNE_CIRCLE_PARAM1 = 80
TUNE_CIRCLE_PARAM2 = 20
TUNE_CIRCLE_MIN_RADIUS = 8
TUNE_CIRCLE_MAX_RADIUS = 35

TUNE_BALL_EXPECTED_RADIUS = 17
TUNE_BALL_TRACK_MAX_JUMP_PX = 80
TUNE_BALL_TRACK_MAX_RADIUS_CHANGE = 8
TUNE_BALL_TRACK_LOST_RESET_FRAMES = 3
TUNE_BALL_FILTER_ALPHA = 0.75
```

实测钢球半径主要为13～20像素；左、中、右圆心分别约为`x=158～174`、`x=356～360`、`x=528～534`。一次ROI外圆`[114,192,20]`被正确排除；空轨道持续返回无有效圆。

## 位置标定

2026-07-21三点静态实测结果：画面右侧极限约`x=625`，轨道物理中心
约`x=360～362`，画面左侧极限约`x=24`。画面右侧对应机械固定端。
当前采用：

```python
TUNE_BALL_TARGET_X = 361
TUNE_BALL_SAFE_LEFT_X = 60
TUNE_BALL_SAFE_RIGHT_X = 598
```

控制误差不做左右分段归一化：

```python
error_px = 361 - ball_x
```

`error_px<0`表示偏向画面右侧固定端，`error_px>0`表示偏向画面左侧
舵机驱动端。这样同样的像素位移在左右两侧具有相同权重。

## LCD标记

- 蓝框：允许钢球圆心出现的软件ROI；
- 两条青色竖线：暂定软件安全边界；
- 黄色竖线：轨道物理中心，即`error_px=0`；
- 绿色框：检测圆的外接框；
- 橙色框：检测到钢球，但原始圆心已经越过软件安全边界；
- 红色十字：圆心；
- 紫色小十字：指数滤波后的控制位置；
- `R`：检测半径；
- `BALL NOT FOUND`：当前帧无有效圆，不复用上一帧坐标。

## 当前通过标准

1. 钢球静止在左、中、右位置时，`ball_valid`绝大多数帧为1；
2. 圆心`x`随钢球从左到右单调增加；
3. 半径通常处于13～20像素；
4. 空轨道输出`ball_valid=0`；
5. ROI外误检不能成为有效钢球；
6. 稳定FPS约20或更高。

移动钢球时手遮挡造成短暂无效是测试操作现象。后续应在无人手遮挡条件下测试钢球自由滚动的连续检出率。

当前输出`error_px`和`ball_safe`，UART、PID和舵机仍然关闭。

## 动态跟踪字段

控制台有效帧包含：

```text
ball_valid=1 ball_safe=1 raw_x=360 filtered_x=361 center_y=254
raw_error_px=1 error_px=0 radius=17 raw_circles=2
tracking=follow detector=hough_circle fps=20.0
```

- `raw_x`：本帧霍夫圆原始圆心，用于立即判断是否越过安全线；
- `filtered_x`：指数滤波位置，用于生成控制误差；
- `raw_error_px`：未滤波像素误差，仅用于对比调试；
- `error_px`：未来发送给STM32的整数控制误差；
- `tracking=acquire`：没有可靠历史位置，本帧在整个ROI重新捕获；
- `tracking=follow`：使用上一有效帧的位置和半径选择连续候选；
- `ball_valid=0`：当前帧没有可靠圆，滤波输出也会立即清除，不复用旧值。

参数`max_jump=80`、`max_radius_change=8`、`lost_reset=3`和`alpha=0.75`
目前只是根据已有日志设定的第一轮动态参数，必须在实际自由滚动中复测。
