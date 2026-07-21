# -*- coding: utf-8 -*-
"""钢球静态视觉测试程序。

目标：在当前固定相机、约20 cm可见轨道范围内，验证钢球检测和位置输出。

处理流程：
    Sensor RGB888采集 -> cv_lite霍夫圆检测 -> ROI/半径过滤
    -> 钢球中心 -> 相对物理中心的整数像素误差 -> LCD和控制台调试

本阶段明确不启用UART、PID和舵机控制。当前相机右侧对应固定端，
error_px为负；相机左侧对应舵机端，error_px为正。

使用的CanMV API：media.sensor.Sensor、media.display.Display、
media.media.MediaManager、cv_lite.rgb888_find_circles()、CanMV Image绘图接口。
硬件：Yahboom K230 12Pin、板载摄像头/LCD、浅色轨道、钢球。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython；需要电脑B实机验证参数。
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

import config
from control.filter import ExponentialFilter
from utils.logger import DebugFrameSaver, log_error, log_info
from vision.ball_detector import BallDetector
from vision.geometry import pixel_position_error, position_is_safe


# =============================================================================
# 现场调参区：CanMV IDE只打开main.py时，优先修改这里
# =============================================================================
FIELD_TUNING_ENABLED = True

# 蓝框必须包住钢球可运动区域，但尽量不要包住绿色底板和两端黑色机构。
# 格式：(x, y, width, height)，坐标以640x480原始图像为准。
TUNE_BALL_ROI = (10, 205, 620, 90)

# 下面是2026-07-21在当前Yahboom v1.8.0实机通过的圆检测参数。
# dp：累加器分辨率比例；min_dist：两个圆心的最小距离。
# param1：Canny高阈值；param2：圆心累加器阈值，越小越容易检出也越易误检。
TUNE_CIRCLE_DP = 1
TUNE_CIRCLE_MIN_DIST = 30
TUNE_CIRCLE_PARAM1 = 80
TUNE_CIRCLE_PARAM2 = 20
TUNE_CIRCLE_MIN_RADIUS = 8
TUNE_CIRCLE_MAX_RADIUS = 35
TUNE_CONSOLE_INTERVAL_FRAMES = 10

# -----------------------------------------------------------------------------
# 动态跟踪与滤波参数
# -----------------------------------------------------------------------------
# 没有历史位置时，在多个圆中优先选择半径最接近17 px的候选。
TUNE_BALL_EXPECTED_RADIUS = 17

# 已经跟踪到钢球后，候选圆心单帧最多允许跳动80 px。
# 太小会漏掉高速钢球，太大则可能跳到远处反光圆；先用实机运动测试验证。
TUNE_BALL_TRACK_MAX_JUMP_PX = 80

# 相邻有效帧半径最多变化8 px，排除半径突然变大的背景反光圆。
TUNE_BALL_TRACK_MAX_RADIUS_CHANGE = 8

# 连续丢失3帧后清除旧位置，允许钢球从远处重新进入ROI时被重新捕获。
# 注意：每个丢失帧都会立即输出ball_valid=0，不会等待3帧才报错。
TUNE_BALL_TRACK_LOST_RESET_FRAMES = 3

# 指数滤波系数。0.5表示当前原始位置和上一滤波结果各占一半。
# 越接近1响应越快、抖动越大；越接近0越平滑、延迟越明显。
TUNE_BALL_FILTER_ALPHA = 0.75

# 三点实测得到的物理中心和暂定安全边界。
# 当前相机画面右侧是固定端，左侧是舵机驱动端。
TUNE_BALL_TARGET_X = 361
TUNE_BALL_SAFE_LEFT_X = 60
TUNE_BALL_SAFE_RIGHT_X = 598


def _apply_field_tuning():
    """用main.py现场参数临时覆盖config.py。"""
    if not FIELD_TUNING_ENABLED:
        print("field_tuning=OFF, use config.py")
        return

    config.BALL_ROI = tuple(TUNE_BALL_ROI)
    # 当前Yahboom v1.8.0的cv_lite绑定要求圆检测参数为整数。
    # 独立实机例程使用dp=1成功；不要在这里转换成1.0。
    config.BALL_CIRCLE_DP = int(TUNE_CIRCLE_DP)
    config.BALL_CIRCLE_MIN_DIST = int(TUNE_CIRCLE_MIN_DIST)
    config.BALL_CIRCLE_PARAM1 = int(TUNE_CIRCLE_PARAM1)
    config.BALL_CIRCLE_PARAM2 = int(TUNE_CIRCLE_PARAM2)
    config.BALL_CIRCLE_MIN_RADIUS = int(TUNE_CIRCLE_MIN_RADIUS)
    config.BALL_CIRCLE_MAX_RADIUS = int(TUNE_CIRCLE_MAX_RADIUS)
    config.BALL_EXPECTED_RADIUS = int(TUNE_BALL_EXPECTED_RADIUS)
    config.BALL_TRACK_MAX_JUMP_PX = int(TUNE_BALL_TRACK_MAX_JUMP_PX)
    config.BALL_TRACK_MAX_RADIUS_CHANGE = int(TUNE_BALL_TRACK_MAX_RADIUS_CHANGE)
    config.BALL_TRACK_LOST_RESET_FRAMES = int(TUNE_BALL_TRACK_LOST_RESET_FRAMES)
    config.BALL_FILTER_ALPHA = float(TUNE_BALL_FILTER_ALPHA)
    config.BALL_TARGET_X = int(TUNE_BALL_TARGET_X)
    config.BALL_SAFE_LEFT_X = int(TUNE_BALL_SAFE_LEFT_X)
    config.BALL_SAFE_RIGHT_X = int(TUNE_BALL_SAFE_RIGHT_X)
    config.CONSOLE_INTERVAL_FRAMES = int(TUNE_CONSOLE_INTERVAL_FRAMES)

    if not (
        config.BALL_SAFE_LEFT_X
        < config.BALL_TARGET_X
        < config.BALL_SAFE_RIGHT_X
    ):
        raise ValueError("BALL_TARGET_X must lie inside the safe range")
    if config.BALL_FILTER_ALPHA <= 0.0 or config.BALL_FILTER_ALPHA > 1.0:
        raise ValueError("BALL_FILTER_ALPHA must be in (0, 1]")

    print("field_tuning=ON")
    print("ball_roi={}".format(config.BALL_ROI))
    print(
        "circle dp={} min_dist={} param1={} param2={} radius={}-{}".format(
            config.BALL_CIRCLE_DP,
            config.BALL_CIRCLE_MIN_DIST,
            config.BALL_CIRCLE_PARAM1,
            config.BALL_CIRCLE_PARAM2,
            config.BALL_CIRCLE_MIN_RADIUS,
            config.BALL_CIRCLE_MAX_RADIUS,
        )
    )
    print(
        "target_x={} safe_x={}..{} fixed_side=camera_right".format(
            config.BALL_TARGET_X,
            config.BALL_SAFE_LEFT_X,
            config.BALL_SAFE_RIGHT_X,
        )
    )
    print(
        "tracking expected_r={} max_jump={} max_dr={} lost_reset={} filter_alpha={}".format(
            config.BALL_EXPECTED_RADIUS,
            config.BALL_TRACK_MAX_JUMP_PX,
            config.BALL_TRACK_MAX_RADIUS_CHANGE,
            config.BALL_TRACK_LOST_RESET_FRAMES,
            config.BALL_FILTER_ALPHA,
        )
    )


def _display_type():
    """把配置字符串转换为Display.ST7701等常量。"""
    if not hasattr(Display, config.DISPLAY_TYPE):
        raise RuntimeError("Display type not found: {}".format(config.DISPLAY_TYPE))
    return getattr(Display, config.DISPLAY_TYPE)


def _create_detector():
    """集中创建钢球检测器，避免算法常量散落在主循环。"""
    return BallDetector(
        image_width=config.CAMERA_WIDTH,
        image_height=config.CAMERA_HEIGHT,
        roi=config.BALL_ROI,
        dp=config.BALL_CIRCLE_DP,
        min_dist=config.BALL_CIRCLE_MIN_DIST,
        param1=config.BALL_CIRCLE_PARAM1,
        param2=config.BALL_CIRCLE_PARAM2,
        min_radius=config.BALL_CIRCLE_MIN_RADIUS,
        max_radius=config.BALL_CIRCLE_MAX_RADIUS,
        expected_radius=config.BALL_EXPECTED_RADIUS,
        max_jump_px=config.BALL_TRACK_MAX_JUMP_PX,
        max_radius_change=config.BALL_TRACK_MAX_RADIUS_CHANGE,
        lost_reset_frames=config.BALL_TRACK_LOST_RESET_FRAMES,
    )


def _add_control_measurement(result, position_filter):
    """增加原始/滤波位置、像素误差和软件安全区状态。

    重要原则：
    - 安全判断使用本帧原始位置，避免滤波延迟掩盖越界；
    - 控制误差使用滤波位置，降低霍夫圆中心约±2 px的静止抖动；
    - 检测失败立即清空滤波器并返回None，绝不输出上一帧旧误差。
    """
    if result is None:
        position_filter.reset()
        return None

    raw_x = result["center"][0]
    filtered_value = position_filter.update(raw_x)

    # 图像坐标均为非负数，+0.5后取整可得到最接近的整数像素。
    # 保持后续UART和STM32处理全部使用整数，避免不必要的浮点传输。
    filtered_x = int(filtered_value + 0.5)

    result["raw_x"] = raw_x
    result["filtered_x"] = filtered_x
    result["raw_error_px"] = pixel_position_error(raw_x, config.BALL_TARGET_X)
    result["error_px"] = pixel_position_error(filtered_x, config.BALL_TARGET_X)
    result["ball_safe"] = position_is_safe(
        raw_x,
        config.BALL_SAFE_LEFT_X,
        config.BALL_SAFE_RIGHT_X,
    )
    return result


def _draw_status(image, result, fps):
    """在LCD帧上绘制ROI、目标中心、安全边界和钢球状态。"""
    roi_x, roi_y, roi_width, roi_height = config.BALL_ROI
    image.draw_rectangle(
        roi_x, roi_y, roi_width, roi_height,
        color=(0, 128, 255), thickness=2,
    )

    roi_bottom = roi_y + roi_height - 1

    # 两条青色细线表示闭环软件安全边界，不代表轨道物理端点。
    image.draw_line(
        config.BALL_SAFE_LEFT_X, roi_y,
        config.BALL_SAFE_LEFT_X, roi_bottom,
        color=(0, 255, 255), thickness=1,
    )
    image.draw_line(
        config.BALL_SAFE_RIGHT_X, roi_y,
        config.BALL_SAFE_RIGHT_X, roi_bottom,
        color=(0, 255, 255), thickness=1,
    )

    # 黄色竖线是实测物理中心，也是error_px=0的位置。
    image.draw_line(
        config.BALL_TARGET_X, roi_y,
        config.BALL_TARGET_X, roi_bottom,
        color=(255, 255, 0), thickness=2,
    )

    if result is None:
        image.draw_string_advanced(10, 10, 24, "BALL NOT FOUND", color=(255, 80, 80))
        image.draw_string_advanced(10, 40, 18, "No circle in ROI", color=(255, 255, 255))
        image.draw_string_advanced(10, 64, 18, "FPS:{:.1f}".format(fps), color=(255, 255, 0))
        return

    center_x, center_y = result["center"]
    bbox_x, bbox_y, bbox_width, bbox_height = result["bbox"]

    # 绿色框和红色十字只表示本帧钢球候选，不复用上一帧坐标。
    ball_color = (0, 255, 0) if result["ball_safe"] else (255, 128, 0)
    image.draw_rectangle(
        bbox_x, bbox_y, bbox_width, bbox_height,
        color=ball_color, thickness=3,
    )
    image.draw_cross(
        center_x, center_y, color=(255, 0, 0), size=12, thickness=3,
    )

    # 紫色小十字表示滤波后的控制位置。它通常靠近红色原始圆心；运动时会因
    # 低通滤波略微落后，这是用少量延迟换取更平滑控制量的正常现象。
    image.draw_cross(
        result["filtered_x"], center_y,
        color=(255, 0, 255), size=6, thickness=1,
    )
    status_text = "BALL OK" if result["ball_safe"] else "BALL UNSAFE"
    image.draw_string_advanced(10, 10, 24, status_text, color=ball_color)
    image.draw_string_advanced(
        10, 38, 18,
        "RAW:{} FIL:{}".format(result["raw_x"], result["filtered_x"]),
        color=(255, 255, 255),
    )
    image.draw_string_advanced(
        10, 64, 20, "ERRPX:{:+d}".format(result["error_px"]), color=(255, 255, 0),
    )
    image.draw_string_advanced(
        10, 90, 18, "R:{}".format(result["radius"]), color=(255, 255, 255),
    )
    image.draw_string_advanced(10, 114, 18, "FPS:{:.1f}".format(fps), color=(255, 255, 0))


def _print_result(result, fps):
    """限频输出固定字段，方便复制现场记录。"""
    if result is None:
        print(
            "ball_valid=0 ball_safe=0 raw_x=-1 filtered_x=-1 center_y=-1 "
            "raw_error_px=0 error_px=0 radius=0 detector=none fps={:.1f}".format(fps)
        )
        return

    center_x, center_y = result["center"]
    print(
        "ball_valid=1 ball_safe={} raw_x={} filtered_x={} center_y={} "
        "raw_error_px={} error_px={} radius={} raw_circles={} "
        "tracking={} detector={} fps={:.1f}".format(
            1 if result["ball_safe"] else 0,
            result["raw_x"],
            result["filtered_x"],
            center_y,
            result["raw_error_px"],
            result["error_px"],
            result["radius"],
            result["raw_circle_count"],
            result["tracking_mode"],
            result["detector"],
            fps,
        )
    )


def run():
    """初始化摄像头/LCD，循环检测钢球，并在退出时释放资源。"""
    sensor = None
    media_initialized = False
    display_initialized = False

    _apply_field_tuning()
    detector = _create_detector()
    position_filter = ExponentialFilter(config.BALL_FILTER_ALPHA)
    if not detector.capability_report()["rgb888_find_circles"]:
        raise RuntimeError("required API missing: cv_lite.rgb888_find_circles")
    print("cv_lite.rgb888_find_circles=OK")
    print("ball_detection=hough_circle_in_fixed_roi")
    print("uart=OFF servo=OFF")

    debug_saver = DebugFrameSaver(
        path=config.DEBUG_IMAGE_PATH,
        interval_ms=config.DEBUG_SAVE_INTERVAL_MS,
        enabled=config.DEBUG_IMAGE_ENABLED,
    )
    clock = time.clock()
    frame_count = 0

    try:
        sensor = Sensor(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        sensor.reset()
        sensor.set_hmirror(config.CAMERA_HMIRROR)
        sensor.set_vflip(config.CAMERA_VFLIP)
        sensor.auto_exposure(config.CAMERA_AUTO_EXPOSURE)
        sensor.set_framesize(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

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
        time.sleep_ms(config.CAMERA_WARMUP_MS)

        log_info("Steel-ball static test started")
        log_info("ROI: {}".format(config.BALL_ROI))

        while True:
            clock.tick()
            frame = sensor.snapshot()
            result = _add_control_measurement(
                detector.detect(frame),
                position_filter,
            )

            # cv_lite处理完成后再转RGB565，避免破坏检测输入。
            display_frame = frame.to_rgb565()
            _draw_status(display_frame, result, clock.fps())
            display_x = max(0, (config.DISPLAY_WIDTH - config.CAMERA_WIDTH) // 2)
            display_y = max(0, (config.DISPLAY_HEIGHT - config.CAMERA_HEIGHT) // 2)
            Display.show_image(display_frame, x=display_x, y=display_y)

            if frame_count % config.CONSOLE_INTERVAL_FRAMES == 0:
                _print_result(result, clock.fps())
            if result is not None:
                debug_saver.save_if_due(display_frame)

            frame_count += 1
            if frame_count >= config.GC_INTERVAL_FRAMES:
                gc.collect()
                frame_count = 0
            os.exitpoint()

    except KeyboardInterrupt:
        log_info("Stopped by user")
    except BaseException as exc:
        log_error("Fatal error: {}".format(exc))
        raise
    finally:
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
