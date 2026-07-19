"""K230 runtime configuration.

APIs: no hardware API calls; values configure Sensor, Display, cv_lite, and UART.
Hardware: Yahboom K230 12Pin module.
Runtime: CanMV K230 Yahboom v1.8.0 MicroPython.
"""

# -----------------------------------------------------------------------------
# 摄像头配置
# -----------------------------------------------------------------------------
# cv_lite.rgb888_find_blobs() 要求输入 RGB888，因此 main.py 会把 Sensor
# 输出格式设置成 Sensor.RGB888。640x480 适合第一阶段：画面足够清楚，内存和
# 运算量也比 1080P 小得多。
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_HMIRROR = False
CAMERA_VFLIP = False
# CanMV 当前 API 使用 auto_exposure() 控制自动曝光。
# 不配置“自动增益”：官方当前接口只有部分传感器支持的 again() 手动模拟增益。
CAMERA_AUTO_EXPOSURE = True
CAMERA_WARMUP_MS = 1000

# -----------------------------------------------------------------------------
# LCD 配置
# -----------------------------------------------------------------------------
# Yahboom 12Pin 模块常见屏幕为 ST7701 800x480。若电脑 B 上板时屏幕不亮，
# 先对照该模块自带“摄像头显示”例程确认 DISPLAY_TYPE，而不是随意猜测 API。
DISPLAY_TYPE = "ST7701"
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DISPLAY_TO_IDE = True

# -----------------------------------------------------------------------------
# 黄色轨道检测配置（本阶段真正使用）
# -----------------------------------------------------------------------------
# cv_lite 的 RGB 阈值顺序：
# [R最小, R最大, G最小, G最大, B最小, B最大]
#
# 下列数值是根据项目照片给出的“起始阈值”，不是标定结果：
# - 黄色通常 R、G 较高；
# - B 通道通常较低；
# - 现场光照、曝光和白平衡变化后必须重新调节。
TRACK_RGB_THRESHOLD = [130, 255, 90, 255, 0, 150]

# cv_lite 在内部按真实连通区域面积过滤；640x480 下先从 1200 像素起步。
# 如果轨道完全检测不到可逐步减小；如果出现小黄物体干扰则增大。
TRACK_MIN_AREA = 1200

# 第一次上板先使用官方 Blob 示例常见值 1，减少参数差异。
# 若后续出现很多离散噪点，可再尝试改成 3 并重新记录 FPS/轮廓变化。
TRACK_KERNEL_SIZE = 1

# ROI 格式：(x, y, width, height)。第一遍测试使用全画面，保证先看到结果。
# 相机固定后可缩小，例如 (20, 100, 600, 280)，排除桌面和人体背景。
# 当前 cv_lite Blob API 不接收 ROI，本项目在检测结果阶段做 ROI 筛选。
TRACK_ROI = (0, 0, CAMERA_WIDTH, CAMERA_HEIGHT)

# 黄色轨道应当明显细长。Blob 外接框长宽比小于该值时判为干扰物。
TRACK_MIN_BBOX_ASPECT_RATIO = 2.5

# 是否尝试 cv_lite 的“带四角点矩形检测”来估计旋转角。
# 程序会先用 hasattr() 检查固件是否存在该 API；缺失时不会调用。
TRACK_USE_ORIENTED_RECT = True
TRACK_ALLOW_BBOX_FALLBACK = True

# cv_lite.rgb888_find_rectangles_with_corners() 的官方示例参数。
# 这些参数先保持接近官方默认值，上板看到实际轮廓后再调整。
TRACK_RECT_CANNY_LOW = 50
TRACK_RECT_CANNY_HIGH = 150
TRACK_RECT_APPROX_EPSILON = 0.04
TRACK_RECT_MIN_AREA_RATIO = 0.001
TRACK_RECT_MAX_ANGLE_COS = 0.5
TRACK_RECT_GAUSSIAN_SIZE = 5

# 候选旋转矩形必须与黄色 Blob 外接框有一定重叠，并保持细长。
TRACK_RECT_MIN_OVERLAP = 0.20
TRACK_RECT_MIN_ASPECT_RATIO = 2.5

# 控制台每隔多少帧输出一次，避免 print 严重拖慢帧率。
CONSOLE_INTERVAL_FRAMES = 10

DEBUG_IMAGE_ENABLED = True
# 本阶段专用文件名，避免与后续钢球 Debug 图片混淆。
DEBUG_IMAGE_PATH = "/sdcard/track_debug.jpg"
DEBUG_SAVE_INTERVAL_MS = 5000
GC_INTERVAL_FRAMES = 30

# -----------------------------------------------------------------------------
# UART（本阶段明确禁用）
# -----------------------------------------------------------------------------
UART_ENABLED = False
UART_ID = 1
UART_BAUDRATE = 115200

# -----------------------------------------------------------------------------
# 钢球参数暂不添加
# -----------------------------------------------------------------------------
# 本阶段 main.py 不导入 BallDetector，不识别钢球，也不输出控制量。
