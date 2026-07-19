# -*- coding: utf-8 -*-
"""黄色轨道静态视觉测试程序（第一阶段）。

目标：验证 Yahboom K230 12Pin + CanMV v1.8.0 + cv_lite 的基础视觉链路。

本程序只做：
    摄像头采集 -> 黄色轨道检测 -> LCD 标注 -> 控制台输出 -> 限频保存图片

本程序明确不做：
    钢球识别、UART 控制量发送、PID、舵机控制、YOLO 或任何模型推理。

使用的 CanMV API：
    media.sensor.Sensor、media.display.Display、media.media.MediaManager、
    cv_lite、CanMV Image 绘图/转换/保存接口。

硬件：Yahboom K230 12Pin 模块、板载摄像头、板载 LCD、黄色轨道。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython；必须由电脑 B 上板验证。
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

import config
from utils.logger import DebugFrameSaver, log_error, log_info
from vision.track_detector import ORIENTED_RECT_API, TrackDetector


# =============================================================================
# 现场调参区
# =============================================================================
# CanMV IDE 在线运行时通常只方便修改当前打开的 main.py。为了避免每次调整
# 黄色阈值都重新上传 config.py，这里的值会在程序启动时临时覆盖 config.py。
#
# 调试完成后，请把最终稳定值回填到 config.py，并将 FIELD_TUNING_ENABLED
# 改成 False。这样 Git 仓库中的长期配置仍然只有一个正式来源。
FIELD_TUNING_ENABLED = True

# 黄色 RGB 阈值：[R_min, R_max, G_min, G_max, B_min, B_max]
TUNE_TRACK_RGB_THRESHOLD = [130, 255, 90, 255, 0, 150]

# 连通区域和软件 ROI
TUNE_TRACK_MIN_AREA = 1200
TUNE_TRACK_KERNEL_SIZE = 1
TUNE_TRACK_ROI = (0, 0, 640, 480)
TUNE_TRACK_MIN_BBOX_ASPECT_RATIO = 2.5

# 旋转矩形。排查黄色 Blob 时可先把 USE_ORIENTED_RECT 改成 False。
TUNE_TRACK_USE_ORIENTED_RECT = True
TUNE_TRACK_ALLOW_BBOX_FALLBACK = True
TUNE_TRACK_RECT_CANNY_LOW = 50
TUNE_TRACK_RECT_CANNY_HIGH = 150
TUNE_TRACK_RECT_APPROX_EPSILON = 0.04
TUNE_TRACK_RECT_MIN_AREA_RATIO = 0.001
TUNE_TRACK_RECT_MAX_ANGLE_COS = 0.5
TUNE_TRACK_RECT_GAUSSIAN_SIZE = 5
TUNE_TRACK_RECT_MIN_OVERLAP = 0.20
TUNE_TRACK_RECT_MIN_ASPECT_RATIO = 2.5


def _apply_field_tuning():
    """用 main.py 顶部的现场值临时覆盖 config.py，不修改算法模块。"""
    if not FIELD_TUNING_ENABLED:
        print("field_tuning=OFF, use config.py")
        return

    config.TRACK_RGB_THRESHOLD = list(TUNE_TRACK_RGB_THRESHOLD)
    config.TRACK_MIN_AREA = int(TUNE_TRACK_MIN_AREA)
    config.TRACK_KERNEL_SIZE = int(TUNE_TRACK_KERNEL_SIZE)
    config.TRACK_ROI = tuple(TUNE_TRACK_ROI)
    config.TRACK_MIN_BBOX_ASPECT_RATIO = float(TUNE_TRACK_MIN_BBOX_ASPECT_RATIO)

    config.TRACK_USE_ORIENTED_RECT = bool(TUNE_TRACK_USE_ORIENTED_RECT)
    config.TRACK_ALLOW_BBOX_FALLBACK = bool(TUNE_TRACK_ALLOW_BBOX_FALLBACK)
    config.TRACK_RECT_CANNY_LOW = int(TUNE_TRACK_RECT_CANNY_LOW)
    config.TRACK_RECT_CANNY_HIGH = int(TUNE_TRACK_RECT_CANNY_HIGH)
    config.TRACK_RECT_APPROX_EPSILON = float(TUNE_TRACK_RECT_APPROX_EPSILON)
    config.TRACK_RECT_MIN_AREA_RATIO = float(TUNE_TRACK_RECT_MIN_AREA_RATIO)
    config.TRACK_RECT_MAX_ANGLE_COS = float(TUNE_TRACK_RECT_MAX_ANGLE_COS)
    config.TRACK_RECT_GAUSSIAN_SIZE = int(TUNE_TRACK_RECT_GAUSSIAN_SIZE)
    config.TRACK_RECT_MIN_OVERLAP = float(TUNE_TRACK_RECT_MIN_OVERLAP)
    config.TRACK_RECT_MIN_ASPECT_RATIO = float(TUNE_TRACK_RECT_MIN_ASPECT_RATIO)

    print("field_tuning=ON")
    print("tune_threshold={}".format(config.TRACK_RGB_THRESHOLD))
    print("tune_roi={}".format(config.TRACK_ROI))
    print(
        "tune_min_area={} kernel={} oriented_rect={}".format(
            config.TRACK_MIN_AREA,
            config.TRACK_KERNEL_SIZE,
            config.TRACK_USE_ORIENTED_RECT,
        )
    )


def _display_type():
    """把 config.py 中的字符串转换成 Display.ST7701 等真实常量。"""
    if not hasattr(Display, config.DISPLAY_TYPE):
        raise RuntimeError("Display type not found: {}".format(config.DISPLAY_TYPE))
    return getattr(Display, config.DISPLAY_TYPE)


def _clamp(value, minimum, maximum):
    """限制坐标，防止绘图端点落到图像外。"""
    return max(minimum, min(maximum, int(value)))


def _clamp_point(point):
    """把点限制在 640x480 等当前相机画面内。"""
    return (
        _clamp(point[0], 0, config.CAMERA_WIDTH - 1),
        _clamp(point[1], 0, config.CAMERA_HEIGHT - 1),
    )


def _draw_closed_contour(image, corners, color):
    """依次连接四角点，形成轨道估计轮廓。"""
    if corners is None or len(corners) != 4:
        return

    for index in range(4):
        start = _clamp_point(corners[index])
        end = _clamp_point(corners[(index + 1) % 4])
        image.draw_line(
            start[0], start[1], end[0], end[1],
            color=color,
            thickness=2,
        )


def _draw_status(image, result, fps):
    """在 RGB565 显示帧上画 ROI、轮廓、中心线和文字。"""
    # 蓝色框表示软件 ROI。初次测试为全画面，固定机位后可以在 config.py 缩小。
    roi_x, roi_y, roi_width, roi_height = config.TRACK_ROI
    image.draw_rectangle(
        roi_x,
        roi_y,
        roi_width,
        roi_height,
        color=(0, 128, 255),
        thickness=1,
    )

    if result is None:
        image.draw_string_advanced(10, 10, 24, "TRACK NOT FOUND", color=(255, 80, 80))
        image.draw_string_advanced(10, 40, 20, "Check RGB threshold", color=(255, 255, 255))
        image.draw_string_advanced(10, 66, 20, "FPS: {:.1f}".format(fps), color=(255, 255, 0))
        return

    center_x, center_y = result["center"]
    bbox_x, bbox_y, bbox_width, bbox_height = result["bbox"]
    axis_start = _clamp_point(result["axis_start"])
    axis_end = _clamp_point(result["axis_end"])

    # 黄色矩形：cv_lite 最大黄色 Blob 的轴对齐外接框。
    image.draw_rectangle(
        bbox_x,
        bbox_y,
        bbox_width,
        bbox_height,
        color=(255, 255, 0),
        thickness=2,
    )

    # 绿色四边形：旋转矩形轮廓；若角点 API 不可用，则退化成 Blob 外接框。
    _draw_closed_contour(image, result["contour"], color=(0, 255, 0))

    # 红色线：轨道主方向；白色十字：黄色区域中心。
    image.draw_line(
        axis_start[0], axis_start[1], axis_end[0], axis_end[1],
        color=(255, 0, 0),
        thickness=3,
    )
    image.draw_cross(
        center_x,
        center_y,
        color=(255, 255, 255),
        size=12,
        thickness=2,
    )

    # LCD 上只画短英文，避免字体资源差异；代码注释和串口说明保持中文。
    image.draw_string_advanced(10, 10, 24, "TRACK OK", color=(0, 255, 0))
    image.draw_string_advanced(
        10, 38, 20,
        "CX:{} CY:{}".format(center_x, center_y),
        color=(255, 255, 255),
    )
    image.draw_string_advanced(
        10, 64, 20,
        "ANGLE:{:.1f}".format(result["angle"]),
        color=(255, 255, 255),
    )
    image.draw_string_advanced(
        10, 90, 18,
        "LEN:{:.0f} {}".format(result["length"], result["angle_source"]),
        color=(255, 255, 0),
    )
    image.draw_string_advanced(10, 114, 18, "FPS:{:.1f}".format(fps), color=(255, 255, 0))


def _print_result(result, fps):
    """按固定字段输出，方便从 CanMV IDE 控制台复制测试记录。"""
    if result is None:
        print(
            "track_valid=0 center_x=-1 center_y=-1 angle=0.00 "
            "length=0.0 angle_source=none fps={:.1f}".format(fps)
        )
        return

    center_x, center_y = result["center"]
    print(
        "track_valid=1 center_x={} center_y={} angle={:.2f} "
        "length={:.1f} angle_source={} fps={:.1f}".format(
            center_x,
            center_y,
            result["angle"],
            result["length"],
            result["angle_source"],
            fps,
        )
    )


def _create_detector():
    """集中把 config.py 参数交给 TrackDetector，main.py 不保存算法常量。"""
    return TrackDetector(
        image_width=config.CAMERA_WIDTH,
        image_height=config.CAMERA_HEIGHT,
        rgb_threshold=config.TRACK_RGB_THRESHOLD,
        min_area=config.TRACK_MIN_AREA,
        kernel_size=config.TRACK_KERNEL_SIZE,
        roi=config.TRACK_ROI,
        min_bbox_aspect_ratio=config.TRACK_MIN_BBOX_ASPECT_RATIO,
        use_oriented_rect=config.TRACK_USE_ORIENTED_RECT,
        allow_bbox_fallback=config.TRACK_ALLOW_BBOX_FALLBACK,
        rect_canny_low=config.TRACK_RECT_CANNY_LOW,
        rect_canny_high=config.TRACK_RECT_CANNY_HIGH,
        rect_approx_epsilon=config.TRACK_RECT_APPROX_EPSILON,
        rect_min_area_ratio=config.TRACK_RECT_MIN_AREA_RATIO,
        rect_max_angle_cos=config.TRACK_RECT_MAX_ANGLE_COS,
        rect_gaussian_size=config.TRACK_RECT_GAUSSIAN_SIZE,
        rect_min_overlap=config.TRACK_RECT_MIN_OVERLAP,
        rect_min_aspect_ratio=config.TRACK_RECT_MIN_ASPECT_RATIO,
    )


def run():
    """初始化摄像头/LCD，循环检测黄色轨道，并保证异常时释放资源。"""
    sensor = None
    media_initialized = False
    display_initialized = False

    # 必须在创建 TrackDetector 前覆盖配置，因为检测器会在构造时复制参数。
    _apply_field_tuning()
    detector = _create_detector()
    capabilities = detector.capability_report()
    if not capabilities["rgb888_find_blobs"]:
        raise RuntimeError("required API missing: cv_lite.rgb888_find_blobs")

    print("cv_lite.rgb888_find_blobs=OK")
    if capabilities[ORIENTED_RECT_API]:
        print("cv_lite.{}=OK".format(ORIENTED_RECT_API))
    else:
        print("cv_lite.{}=MISSING, use bbox_approx".format(ORIENTED_RECT_API))

    debug_saver = DebugFrameSaver(
        path=config.DEBUG_IMAGE_PATH,
        interval_ms=config.DEBUG_SAVE_INTERVAL_MS,
        enabled=config.DEBUG_IMAGE_ENABLED,
    )
    clock = time.clock()
    frame_count = 0

    try:
        # 1. 创建并复位 Sensor。官方 K230 示例要求 reset() 后再设置输出参数。
        sensor = Sensor(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        sensor.reset()
        sensor.set_hmirror(config.CAMERA_HMIRROR)
        sensor.set_vflip(config.CAMERA_VFLIP)
        # CanMV 当前 Sensor API 没有 set_auto_gain()。
        # 自动增益由传感器默认策略处理；若后续确需手动模拟增益，应先确认
        # 当前摄像头支持 again()，并按照官方要求在 sensor.run() 后设置。
        sensor.auto_exposure(config.CAMERA_AUTO_EXPOSURE)
        sensor.set_framesize(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        # 2. 初始化 LCD，再初始化媒体缓冲，最后启动 Sensor。
        Display.init(
            _display_type(),
            width=config.DISPLAY_WIDTH,
            height=config.DISPLAY_HEIGHT,
            to_ide=config.DISPLAY_TO_IDE,
        )
        display_initialized = True
        MediaManager.init()
        media_initialized = True
        sensor.run()

        # 自动曝光/增益需要一点时间稳定。这里只等待一次，不阻塞主循环。
        time.sleep_ms(config.CAMERA_WARMUP_MS)
        log_info("Yellow track static test started")
        log_info("Threshold: {}".format(config.TRACK_RGB_THRESHOLD))
        log_info("ROI: {}".format(config.TRACK_ROI))

        while True:
            clock.tick()

            # 3. snapshot() 获取一帧 RGB888 图像，TrackDetector 使用 cv_lite 处理。
            frame = sensor.snapshot()
            result = detector.detect(frame)

            # 4. 检测完成后再转 RGB565，避免破坏 cv_lite 所需的 RGB888 输入。
            display_frame = frame.to_rgb565()
            _draw_status(display_frame, result, clock.fps())

            # 640x480 图像在 800x480 LCD 上水平居中显示。
            display_x = max(0, (config.DISPLAY_WIDTH - config.CAMERA_WIDTH) // 2)
            display_y = max(0, (config.DISPLAY_HEIGHT - config.CAMERA_HEIGHT) // 2)
            Display.show_image(display_frame, x=display_x, y=display_y)

            # 5. 控制台输出限频，避免每帧 print 把视觉处理拖慢。
            if frame_count % config.CONSOLE_INTERVAL_FRAMES == 0:
                _print_result(result, clock.fps())

            # 6. 仅在检测有效时限频保存带标注图片，不会每帧写 SD 卡。
            if result is not None:
                debug_saver.save_if_due(display_frame)

            # 7. 同样限频做垃圾回收，兼顾实时性和长时间运行的内存稳定性。
            frame_count += 1
            if frame_count >= config.GC_INTERVAL_FRAMES:
                gc.collect()
                frame_count = 0

            # CanMV 官方示例使用 exitpoint() 响应 IDE 停止操作。
            os.exitpoint()

    except KeyboardInterrupt:
        log_info("Stopped by user")
    except BaseException as exc:
        log_error("Fatal error: {}".format(exc))
        raise
    finally:
        # 官方要求：先 stop Sensor，再 deinit Display，最后释放 MediaManager。
        if sensor is not None:
            try:
                sensor.stop()
            except BaseException:
                pass
        if display_initialized:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_initialized:
            MediaManager.deinit()
        log_info("Resources released")


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    run()
