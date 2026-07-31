# -*- coding: utf-8 -*-
"""钢球静态视觉测试程序。

目标：在当前固定相机、约20 cm可见轨道范围内，验证钢球检测和位置输出。

处理流程：
    Sensor灰度/RGB888采集 -> 对应的cv_lite霍夫圆检测 -> ROI/半径过滤
    -> 钢球中心 -> 相对物理中心的整数像素误差 -> LCD和控制台调试

本阶段启用K230到STM32的单向UART视觉测量；PID和舵机闭环由STM32负责。
当前相机右侧对应固定端，error_px为负；相机左侧对应舵机端，error_px为正。

使用的CanMV API：media.sensor.Sensor、media.display.Display、
media.media.MediaManager、cv_lite灰度/RGB888圆检测、CanMV Image绘图接口、
machine.FPIOA和machine.UART。
硬件：Yahboom K230 12Pin、板载摄像头/LCD、浅色轨道、钢球。
运行时：CanMV K230 Yahboom v1.8.0
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor

import config
from communication.telemetry_logger import TelemetryLogger
from communication.uart import VisionUart
from control.filter import ExponentialFilter
from utils.logger import DebugFrameSaver, log_error, log_info
from vision.ball_detector import BallDetector
from vision.geometry import pixel_position_error, position_is_safe


# =============================================================================
# 调参区：CanMV IDE只打开main.py时，优先修改这里
# =============================================================================
FIELD_TUNING_ENABLED = True

# 新支架全画面标定开关。
#
# True：关闭裁剪和UART，在完整VGA画面中搜索钢球并显示坐标参考线。
# False：使用下面的新支架裁剪参数进入视觉复测。
CALIBRATION_MODE = False

# 灰度检测比RGB888约快1.5 FPS。当前直接在灰度帧上绘制灰度标记，
# 不再先把灰度帧转换成RGB565。
TUNE_GRAYSCALE_ENABLED = True

# 当前相机高度下的近水平球心约为：
# 标尺实测：左端球心x=22对应-11.6 cm，物理中心x=314，
# 右端球心x=622对应+11.8 cm。
#
# 傻瓜裁剪接口：只修改WIDTH和HEIGHT。
# 程序始终围绕当前相机安装位置生成裁剪画面；球杆控制零点单独使用x=314。
# 为避免K230裁剪/显示缓冲条纹，输入值会自动向下对齐到16的倍数。
# 例如WIDTH填600会实际使用592；HEIGHT填150会实际使用144。
TUNE_CROP_ENABLED = True
TUNE_VIEW_CENTER_X = 320
TUNE_VIEW_CENTER_Y = 290
TUNE_VIEW_WIDTH = 640
TUNE_VIEW_HEIGHT = 76
TUNE_VIEW_ALIGNMENT = 16

# 检测ROI默认等于整个自动裁剪画面，不需要跟着WIDTH/HEIGHT修改。
TUNE_ROI_MARGIN_X = 0
TUNE_ROI_MARGIN_Y = 0

# 当前视觉ROI复测完成，启用K230 UART1向STM32发送视觉测量。
TUNE_UART_ENABLED = True

# True时LCD/IDE只显示自动生成的检测ROI。
# 这只改变调试画面，不改变检测、坐标、UART或闭环安全边界。
TUNE_DISPLAY_ROI_ONLY = True
# True时在画面左上角显示小号FPS；不会启用控制台周期日志。
TUNE_DISPLAY_FPS = True

# 下面是2026-07-21在当前Yahboom v1.8.0实机通过的圆检测参数。
# dp：累加器分辨率比例；min_dist：两个圆心的最小距离。
# param1：Canny高阈值；param2：圆心累加器阈值，越小越容易检出也越易误检。
TUNE_CIRCLE_DP = 1
TUNE_CIRCLE_MIN_DIST = 30
TUNE_CIRCLE_PARAM1 = 80
TUNE_CIRCLE_PARAM2 = 20
TUNE_CIRCLE_MIN_RADIUS = 8
TUNE_CIRCLE_MAX_RADIUS = 35

# 循环日志会占用少量运行时间；重新标定时开启，每10帧打印一行精简状态。
# 标定完成后可改回False，视觉UART发送不受这个开关影响。
TUNE_CONSOLE_ENABLED = True
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
# ★★★ 修改为 4，更严格限制半径变化，抑制跳变 ★★★
TUNE_BALL_TRACK_MAX_RADIUS_CHANGE = 16

# 连续丢失3帧后清除旧位置，允许钢球从远处重新进入ROI时被重新捕获。
# 注意：每个丢失帧都会立即输出ball_valid=0，不会等待3帧才报错。
TUNE_BALL_TRACK_LOST_RESET_FRAMES = 3

# 指数滤波系数。0.5表示当前原始位置和上一滤波结果各占一半。
# 越接近1响应越快、抖动越大；越接近0越平滑、延迟越明显。
# ★★★ 改为 0.55，适应 90 FPS，增强平滑 ★★★
TUNE_BALL_FILTER_ALPHA = 0.55

# 速度外推预测参数（检测可信度门限）。
# 检测圆心相对"上一帧位置+速度外推"偏移超过GATE时判为误检，改用预测
# 位置保持，不让单帧错误直接打进控制器。90 FPS下钢球每帧只移动数像素。
TUNE_BALL_PREDICT_GATE_PX = 20
TUNE_BALL_PREDICT_VELOCITY_ALPHA = 0.40
TUNE_BALL_PREDICT_V_MAX_PX_S = 600

# 重新捕获确认：被追丢后新候选连续几帧在容差内一致才接受。
# 防止反光圆在失球后直接成为新目标。
TUNE_BALL_ACQUIRE_CONFIRM_FRAMES = 3
TUNE_BALL_ACQUIRE_CONFIRM_TOLERANCE_PX = 12

# 新支架三点实测得到的物理中心和控制边界。
# 当前相机画面右侧是固定端，左侧是舵机驱动端；x=22～622是实测仍能
# 检测到完整钢球圆心的范围。K230和STM32必须使用完全相同的边界。
TUNE_BALL_TARGET_X = 314
TUNE_BALL_SAFE_LEFT_X = 22
TUNE_BALL_SAFE_RIGHT_X = 622


def _apply_field_tuning():
    """用main.py现场参数临时覆盖config.py。"""
    if not FIELD_TUNING_ENABLED:
        print("field_tuning=OFF, use config.py")
        return

    requested_width = int(TUNE_VIEW_WIDTH)
    requested_height = int(TUNE_VIEW_HEIGHT)
    alignment = int(TUNE_VIEW_ALIGNMENT)
    if (
        alignment <= 0
        or requested_width < alignment
        or requested_height < alignment
        or requested_width > config.CAMERA_WIDTH
        or requested_height > config.CAMERA_HEIGHT
    ):
        raise ValueError(
            "VIEW width/height must fit VGA and be at least one alignment unit"
        )

    view_width = requested_width - requested_width % alignment
    view_height = requested_height - requested_height % alignment

    crop_x = int(TUNE_VIEW_CENTER_X) - view_width // 2
    crop_y = int(TUNE_VIEW_CENTER_Y) - view_height // 2
    crop_x = max(0, min(config.CAMERA_WIDTH - view_width, crop_x))
    crop_y = max(0, min(config.CAMERA_HEIGHT - view_height, crop_y))

    config.CROP_ENABLED = bool(TUNE_CROP_ENABLED)
    config.CROP_X = crop_x
    config.CROP_Y = crop_y
    config.CROP_WIDTH = view_width
    config.CROP_HEIGHT = view_height
    config.UART_ENABLED = bool(TUNE_UART_ENABLED)

    if CALIBRATION_MODE:
        # 标定阶段不能继续使用旧支架的裁剪框、ROI和控制坐标。
        config.CROP_ENABLED = False
        config.BALL_ROI = (
            0,
            0,
            config.CAMERA_WIDTH,
            config.CAMERA_HEIGHT,
        )
        config.UART_ENABLED = False
    else:
        roi_margin_x = int(TUNE_ROI_MARGIN_X)
        roi_margin_y = int(TUNE_ROI_MARGIN_Y)
        config.BALL_ROI = (
            roi_margin_x,
            roi_margin_y,
            view_width - 2 * roi_margin_x,
            view_height - 2 * roi_margin_y,
        )
    config.BALL_GRAYSCALE_ENABLED = bool(TUNE_GRAYSCALE_ENABLED)
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
    config.BALL_PREDICT_GATE_PX = int(TUNE_BALL_PREDICT_GATE_PX)
    config.BALL_PREDICT_VELOCITY_ALPHA = float(TUNE_BALL_PREDICT_VELOCITY_ALPHA)
    config.BALL_PREDICT_V_MAX_PX_S = float(TUNE_BALL_PREDICT_V_MAX_PX_S)
    config.BALL_ACQUIRE_CONFIRM_FRAMES = int(TUNE_BALL_ACQUIRE_CONFIRM_FRAMES)
    config.BALL_ACQUIRE_CONFIRM_TOLERANCE_PX = int(
        TUNE_BALL_ACQUIRE_CONFIRM_TOLERANCE_PX
    )
    config.BALL_TARGET_X = int(TUNE_BALL_TARGET_X)
    config.BALL_SAFE_LEFT_X = int(TUNE_BALL_SAFE_LEFT_X)
    config.BALL_SAFE_RIGHT_X = int(TUNE_BALL_SAFE_RIGHT_X)
    config.CONSOLE_INTERVAL_FRAMES = int(TUNE_CONSOLE_INTERVAL_FRAMES)

    if config.CROP_ENABLED and (
        config.CROP_X < 0
        or config.CROP_Y < 0
        or config.CROP_WIDTH <= 0
        or config.CROP_HEIGHT <= 0
        or config.CROP_X + config.CROP_WIDTH > config.CAMERA_WIDTH
        or config.CROP_Y + config.CROP_HEIGHT > config.CAMERA_HEIGHT
    ):
        raise ValueError("CROP rectangle must fit inside the VGA frame")

    working_width = (
        config.CROP_WIDTH if config.CROP_ENABLED else config.CAMERA_WIDTH
    )
    working_height = (
        config.CROP_HEIGHT if config.CROP_ENABLED else config.CAMERA_HEIGHT
    )
    roi_x, roi_y, roi_width, roi_height = config.BALL_ROI
    if (
        roi_x < 0
        or roi_y < 0
        or roi_width <= 0
        or roi_height <= 0
        or roi_x + roi_width > working_width
        or roi_y + roi_height > working_height
    ):
        raise ValueError("BALL_ROI must fit inside the processed frame")

    if not (
        config.BALL_SAFE_LEFT_X
        < config.BALL_TARGET_X
        < config.BALL_SAFE_RIGHT_X
    ):
        raise ValueError("BALL_TARGET_X must lie inside the safe range")
    if config.BALL_FILTER_ALPHA <= 0.0 or config.BALL_FILTER_ALPHA > 1.0:
        raise ValueError("BALL_FILTER_ALPHA must be in (0, 1]")

    print("field_tuning=ON")
    print(
        "calibration_mode={} crop={} uart={}".format(
            "ON" if CALIBRATION_MODE else "OFF",
            "ON" if config.CROP_ENABLED else "OFF",
            "ON" if config.UART_ENABLED else "OFF",
        )
    )
    print(
        "view_request={}x{} aligned_view={}x{}".format(
            requested_width,
            requested_height,
            config.CROP_WIDTH,
            config.CROP_HEIGHT,
        )
    )
    print(
        "crop_rect=({}, {}, {}, {})".format(
            config.CROP_X,
            config.CROP_Y,
            config.CROP_WIDTH,
            config.CROP_HEIGHT,
        )
    )
    print(
        "vision_format={}".format(
            "GRAYSCALE" if config.BALL_GRAYSCALE_ENABLED else "RGB888"
        )
    )
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
        image_width=config.CROP_WIDTH if config.CROP_ENABLED else config.CAMERA_WIDTH,
        image_height=config.CROP_HEIGHT if config.CROP_ENABLED else config.CAMERA_HEIGHT,
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
        predict_gate_px=config.BALL_PREDICT_GATE_PX,
        velocity_alpha=config.BALL_PREDICT_VELOCITY_ALPHA,
        v_max_px_s=config.BALL_PREDICT_V_MAX_PX_S,
        acquire_confirm_frames=config.BALL_ACQUIRE_CONFIRM_FRAMES,
        acquire_confirm_tolerance_px=config.BALL_ACQUIRE_CONFIRM_TOLERANCE_PX,
        use_grayscale=config.BALL_GRAYSCALE_ENABLED,
    )


def _add_control_measurement(result, position_filter):
    """增加原始/滤波位置、像素误差和软件安全区状态。

    重要原则：
    - 安全判断使用本帧原始位置，避免滤波延迟掩盖越界；
    - 控制误差使用检测器估计位置（可信帧再过一次指数滤波）；
    - 预测保持帧（extrapolated）直接使用估计位置，不再二次滤波；
    - 只有真正失球（返回None）才清空滤波器。
    """
    if result is None:
        position_filter.reset()
        return None

    crop_offset = config.CROP_X if config.CROP_ENABLED else 0
    local_x = result["center"][0]
    raw_x = local_x + crop_offset
    local_est_x = result["est_x"]
    est_global_x = local_est_x + crop_offset

    if result["extrapolated"]:
        # 预测保持帧：直接使用外推估计，不再二次滤波，避免额外延迟。
        filtered_value = float(est_global_x)
    else:
        filtered_value = position_filter.update(est_global_x)

    # 图像坐标均为非负数，+0.5后取整可得到最接近的整数像素。
    # 保持后续UART和STM32处理全部使用整数，避免不必要的浮点传输。
    filtered_x = int(filtered_value + 0.5)

    result["raw_x"] = raw_x
    result["filtered_x"] = filtered_x
    result["frame_y"] = (
        result["center"][1] + config.CROP_Y
        if config.CROP_ENABLED
        else result["center"][1]
    )
    result["raw_error_px"] = pixel_position_error(raw_x, config.BALL_TARGET_X)
    result["error_px"] = pixel_position_error(filtered_x, config.BALL_TARGET_X)
    result["ball_safe"] = position_is_safe(
        raw_x,
        config.BALL_SAFE_LEFT_X,
        config.BALL_SAFE_RIGHT_X,
    )
    return result


def _draw_status(image, result, fps, grayscale):
    """普通模式画钢球框和可选FPS；标定模式再画全画面参考。

    result为None时绝不能读取上一帧坐标或访问result字段。
    灰度帧使用0~255的单通道颜色值；RGB565帧使用RGB元组。
    """
    draw_color = 255 if grayscale else (255, 255, 255)
    reference_color = 160 if grayscale else (0, 255, 255)
    old_target_color = 220 if grayscale else (255, 255, 0)

    # 普通复测模式只保留本帧钢球框和可选的小号FPS。
    if not CALIBRATION_MODE:
        if result is not None:
            bbox_x, bbox_y, bbox_width, bbox_height = result["bbox"]
            image.draw_rectangle(
                bbox_x, bbox_y, bbox_width, bbox_height,
                color=draw_color, thickness=2,
            )
        if TUNE_DISPLAY_FPS:
            image.draw_string_advanced(
                4, 4, 16,
                "FPS:{:.1f}".format(fps),
                color=draw_color,
            )
        return

    if CALIBRATION_MODE:
        image_center_x = config.CAMERA_WIDTH // 2
        image_center_y = config.CAMERA_HEIGHT // 2

        # 细线表示VGA图像中心，只作坐标参考，不代表机械轨道中心。
        image.draw_line(
            image_center_x, 0,
            image_center_x, config.CAMERA_HEIGHT - 1,
            color=reference_color, thickness=1,
        )
        image.draw_line(
            0, image_center_y,
            config.CAMERA_WIDTH - 1, image_center_y,
            color=reference_color, thickness=1,
        )

        # 粗线表示当前实测物理中心/控制目标x=314。
        image.draw_line(
            config.BALL_TARGET_X, 0,
            config.BALL_TARGET_X, config.CAMERA_HEIGHT - 1,
            color=old_target_color, thickness=2,
        )

    # 当前Yahboom v1.8.0会为draw_string()逐帧打印弃用警告，
    # 因此使用已经过实机验证的draw_string_advanced()。
    image.draw_string_advanced(
        4, 4, 24,
        (
            "CAL VGA FPS:{:.1f}".format(fps)
            if CALIBRATION_MODE
            else "FPS:{:.1f}".format(fps)
        ),
        color=draw_color,
    )

    if result is None:
        if CALIBRATION_MODE:
            image.draw_string_advanced(
                4, 32, 16,
                "BALL NOT FOUND  IMG=320,240 OLD_X={}".format(
                    config.BALL_TARGET_X
                ),
                color=draw_color,
            )
        return

    center_x, center_y = result["center"]
    bbox_x, bbox_y, bbox_width, bbox_height = result["bbox"]
    image.draw_rectangle(
        bbox_x, bbox_y, bbox_width, bbox_height,
        color=draw_color, thickness=2,
    )
    if CALIBRATION_MODE:
        image.draw_cross(
            center_x, center_y,
            color=draw_color, size=10, thickness=2,
        )
        image.draw_string_advanced(
            4, 32, 16,
            "X={} Y={} R={} C={}".format(
                center_x,
                center_y,
                result["radius"],
                result["raw_circle_count"],
            ),
            color=draw_color,
        )



def _print_result(result, fps):
    """按需输出一行精简运行状态，不再打印累计诊断计数。"""
    if result is None:
        print(
            "ball_valid=0 ball_safe=0 fps={:.1f}".format(fps)
        )
        return

    marker = "ext" if result["extrapolated"] else "det"
    print(
        "ball_valid=1 ball_safe={} {} raw_x={} filtered_x={} "
        "error_px={} radius={} fps={:.1f}".format(
            1 if result["ball_safe"] else 0,
            marker,
            result["raw_x"],
            result["filtered_x"],
            result["error_px"],
            result["radius"],
            fps,
        )
    )


def run():
    """初始化摄像头/LCD/UART，循环检测并发送钢球测量。"""
    sensor = None
    vision_uart = None
    telemetry_logger = None
    media_initialized = False
    display_initialized = False

    _apply_field_tuning()
    detector = _create_detector()
    position_filter = ExponentialFilter(config.BALL_FILTER_ALPHA)
    required_api = detector.required_api_name()
    if not detector.capability_report()[required_api]:
        raise RuntimeError("required API missing: cv_lite.{}".format(required_api))
    print("cv_lite.{}=OK".format(required_api))
    print("ball_detection=hough_circle_in_fixed_roi")
    print(
        "uart={} id={} baud={} tx_io={} rx_io={} servo=OFF".format(
            "ON" if config.UART_ENABLED else "OFF",
            config.UART_ID,
            config.UART_BAUDRATE,
            config.UART_TX_PIN,
            config.UART_RX_PIN,
        )
    )

    debug_saver = DebugFrameSaver(
        path=config.DEBUG_IMAGE_PATH,
        interval_ms=config.DEBUG_SAVE_INTERVAL_MS,
        enabled=config.DEBUG_IMAGE_ENABLED,
    )
    clock = time.clock()
    frame_count = 0
    telemetry_status_count = 0

    try:
        vision_uart = VisionUart(
            uart_id=config.UART_ID,
            baudrate=config.UART_BAUDRATE,
            tx_pin=config.UART_TX_PIN,
            rx_pin=config.UART_RX_PIN,
            enabled=config.UART_ENABLED,
        )
        telemetry_logger = TelemetryLogger(
            path=config.TELEMETRY_LOG_PATH,
            rx_chunk=config.TELEMETRY_RX_BUFFER_SIZE,
            write_block=config.TELEMETRY_WRITE_BUFFER_SIZE,
            sync_interval=config.TELEMETRY_SYNC_INTERVAL_BLOCKS,
            enabled=(
                config.TELEMETRY_LOG_ENABLED
                and config.UART_ENABLED
            ),
        )

        sensor = Sensor(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        sensor.reset()
        sensor.set_hmirror(config.CAMERA_HMIRROR)
        sensor.set_vflip(config.CAMERA_VFLIP)
        sensor.auto_exposure(config.CAMERA_AUTO_EXPOSURE)
        sensor.set_framesize(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        # cv_lite的灰度/RGB888圆检测函数要求输入格式与Sensor输出严格一致。
        # 灰度模式由Sensor直接产生单通道图像，不在Python循环里做格式转换。
        if config.BALL_GRAYSCALE_ENABLED:
            sensor.set_pixformat(Sensor.GRAYSCALE)
        else:
            sensor.set_pixformat(Sensor.RGB888)

        Display.init(
            _display_type(),
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

            if config.CROP_ENABLED:
                frame = frame.crop(roi=(config.CROP_X, config.CROP_Y,
                                        config.CROP_WIDTH, config.CROP_HEIGHT))

            result = _add_control_measurement(
                detector.detect(frame),
                position_filter,
            )

            # 每处理完一帧就发送一次，包括“没有找到球”的无效帧。
            # 这样STM32可以区分：通信超时、通信正常但视觉无效、球有效但越界。
            if result is None:
                # 使用位置参数避免在每帧循环中创建关键字参数对象。
                vision_uart.send_measurement(False, False, 0, -1)
            else:
                vision_uart.send_measurement(
                    True,
                    result["ball_safe"],
                    result["error_px"],
                    result["filtered_x"],
                )

            # 同一组UART引脚是全双工的：
            # K230刚刚从IO9发送视觉数据后，立即非阻塞读取IO10上已经到达的
            # STM32遥测字节。poll()只消费现有字节，不会等待，不改变视觉节拍。
            telemetry_logger.poll(vision_uart)

            if config.BALL_GRAYSCALE_ENABLED:
                # 直接在1字节/像素的灰度Sensor帧上画灰度标记。
                # 不调用to_rgb565()，因此没有灰度缓冲扩展和额外整帧转换。
                display_frame = frame
            else:
                # RGB888回退模式仍转换为RGB565后再显示。
                display_frame = frame.to_rgb565()
            _draw_status(
                display_frame,
                result,
                clock.fps(),
                config.BALL_GRAYSCALE_ENABLED,
            )
            full_working_roi = (
                0,
                0,
                config.CROP_WIDTH,
                config.CROP_HEIGHT,
            )
            if (
                TUNE_DISPLAY_ROI_ONLY
                and not CALIBRATION_MODE
                and config.BALL_ROI != full_working_roi
            ):
                display_frame = display_frame.crop(roi=config.BALL_ROI)
            # 使用Yahboom官方例程已验证的ST7701默认显示尺寸和默认原点。
            # 不强制创建800x480显示层，避免IDE可见但脱机LCD没有画面。
            Display.show_image(display_frame)

            if (
                TUNE_CONSOLE_ENABLED
                and frame_count % config.CONSOLE_INTERVAL_FRAMES == 0
            ):
                _print_result(result, clock.fps())
            if result is not None:
                debug_saver.save_if_due(display_frame)

            telemetry_status_count += 1
            if (
                telemetry_logger is not None
                and config.TELEMETRY_LOG_ENABLED
                and telemetry_status_count
                >= config.TELEMETRY_STATUS_INTERVAL_FRAMES
            ):
                print(telemetry_logger.status())
                telemetry_status_count = 0

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
        # 先关闭日志，把最后不足1 KB的尾部帧写入TF卡；再释放UART。
        if telemetry_logger is not None:
            try:
                telemetry_logger.close()
            except BaseException:
                pass
        if vision_uart is not None:
            try:
                vision_uart.deinit()
            except BaseException:
                pass
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
