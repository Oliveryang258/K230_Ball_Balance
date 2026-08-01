# K230 视觉识别与 MJPEG 图传合并实验

本实验采用独立入口 `src/main_stream_test.py`，没有修改原来的
`src/main.py`。任何时候停止实验并重新运行 `src/main.py`，即可恢复原视觉
控制链路。

## 实验目标

- CH0：640×480 YUV420SP，绑定硬件 JPEG VENC，10 FPS 无线 MJPEG；
- CH1：640×480 GRAYSCALE，继续使用当前裁剪、霍夫圆检测、跟踪和滤波；
- 网络发送一次视觉循环最多推进 16 KB；发生拥塞时关闭客户端，不能阻塞 UART；
- 第一阶段关闭控制 UART 和 TF 卡记录，只验证双通道兼容性、FPS、内存和画面；
- 第二阶段才开启 UART，对比图传开启/关闭时的控制结果。

## 随时回退

设备侧：

1. 在 CanMV IDE 中停止 `main_stream_test.py`；
2. 重新打开并运行原来的 `src/main.py`；
3. 原入口和现有视觉参数没有被本实验修改。

Git 侧：实验位于 `codex/k230-mjpeg-integration` 分支。需要完全回到开始前时，
先确认实验日志已经保存，再切回 `main`。仓库原本存在的未提交修改属于现有
项目工作，不要清理或重置它们。

## 上板前配置

在 `src/main_stream_test.py` 顶部填写：

```python
STREAM_TEST_WIFI_SSID = "接收端热点名称"
STREAM_TEST_WIFI_PASSWORD = "热点密码"
```

不要把真实密码提交到 Git 或发到群文件。接收电脑和 K230 应连接同一网络。

## 只有SD卡、不能使用IDE时

在当前能够自动运行的 `main.py` 所在目录操作，所有改名都保留在同一目录：

1. 将原视觉程序 `main.py` 改名为 `vision_main.py`，不要删除；
2. 将 `main_stream_test.py` 原名复制到该目录；
3. 将 `main_sd_boot.py` 复制到该目录并改名为新的 `main.py`；
4. 将 `communication/mjpeg_stream.py` 放进同目录下的 `communication/`；
5. 保留原来的 `config.py`、`vision/`、`control/`、`communication/uart.py`
   和 `utils/`；
6. 上电后等待连接热点，LCD第二行会持续显示
   `http://K230-IP:8080/`，接收端浏览器输入这个完整地址即可。

启动时LCD会先显示 `WIFI CONNECTING...`。连接或运行失败时会短暂显示
`ERROR: CHECK SD LOG`，此时不要继续等待，应断电读取启动日志。

纯SD卡模式下必须同时存在 `vision_main.py`、`main_stream_test.py` 和作为
`main.py` 使用的启动器。若LCD超过30秒仍没有画面，断电后读取SD卡中的
`/sdcard/k230_stream_boot.log`；日志会记录导入阶段或运行阶段的异常类型和信息。

纯SD卡回退：删除或改名实验版 `main.py`，再把 `vision_main.py` 改回
`main.py`。不要同时保留两个同名文件。

## 阶段 A：视觉-only 安全测试

保持：

```python
STREAM_TEST_UART_ENABLED = False
STREAM_TEST_TELEMETRY_LOG_ENABLED = False
STREAM_TEST_DISPLAY_TO_IDE = False
STREAM_TARGET_FPS = 10
```

1. 只给 K230 上电，不让舵机进入闭环；
2. 先运行 60 秒但不打开网页，记录 `vision_fps`、`drop`、`maxsvc` 和 `mem`；
3. 浏览器打开控制台打印的 `http://K230-IP:8080/`；
4. 连续运行至少 3 分钟，缓慢移动钢球和整车，覆盖复杂运动画面；
5. 关闭或远离热点制造弱网，确认浏览器可以断线重连且视觉循环没有长时间停顿；
6. 停止程序后确认资源可以释放，再运行原 `main.py` 验证回退。

建议通过条件：

- 有客户端时视觉平均帧率不低于 55 FPS；
- `maxsvc` 通常不超过 3 ms，不能出现接近 100 ms 的网络服务时间；
- 内存不随运行时间持续下降；
- LCD上的钢球框与原程序一致；
- 弱网时允许图传丢帧/重连，但识别不能冻结。

## 阶段 B：静止车闭环对照

阶段 A 通过后，把 `STREAM_TEST_UART_ENABLED` 改为 `True`。先拆除车辆驱动风险或
保持车辆静止，从舵机中位附近开始。分别采集：

1. 原 `main.py`，图传关闭；
2. `main_stream_test.py`，网页未连接；
3. `main_stream_test.py`，网页持续连接；
4. `main_stream_test.py`，弱网并触发至少一次浏览器重连。

每组至少记录 FPS、UART/guard状态、最大测量间隔、球位置误差、舵机PWM和是否
出现 `UART_TIMEOUT`。任何一次视觉/UART间隔达到 150 ms，都视为失败并立即回退。

## 阶段 C：整车与录像

控制对照通过后再装车。接收端使用 OBS、VLC 或 FFmpeg 保存视频，验证：

- 整个摆杆持续可见；
- 图传不中断或能够在允许时间内恢复；
- 录像文件完整保存；
- 赛后可以正常回放；
- 文件名、测试时间、持续时间与对应遥测日志能够一一对应。

当前HTTP页面只负责显示，不会自动在K230上录像。

## 已知兼容性风险

- 双通道 `CAM_CHN_ID_0/1` 是K230官方接口，但尚未在当前Yahboom v1.8.0组合上验证；
- JPEG创建使用队友代码中的旧版 `media.vencoder` 低层符号；
- `output_fps=10` 是否被旧固件严格执行需要观察实机日志；
- 浏览器画面来自原始YUV通道，因此不包含灰度识别通道上绘制的钢球框；
- 不需要安装任何额外Python模块。
