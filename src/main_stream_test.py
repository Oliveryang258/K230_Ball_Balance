# -*- coding: utf-8 -*-
"""K230钢球视觉与无线MJPEG双通道合并实验入口。

CanMV API：media.sensor双通道、cv_lite灰度圆检测、media.vencoder JPEG、
          network.WLAN、非阻塞socket、Display、UART。
硬件：Yahboom K230 12Pin、GC2093/板载摄像头、ST7701、板载Wi-Fi，
      可选K230 IO9/IO10连接STM32。
兼容性：现有灰度识别和UART已按Yahboom CanMV v1.8.0编写；Sensor双通道与
        旧版JPEG VENC组合为本实验新增能力，必须先在实机视觉-only模式验证。

回退：本文件不替换src/main.py。停止本实验并重新运行main.py即可恢复原链路。
"""

import gc
import os
import time

from media.display import Display
from media.media import MediaManager
from media.sensor import Sensor, CAM_CHN_ID_0, CAM_CHN_ID_1

import config
# IDE手动运行时复用同目录原main.py；纯SD卡自启动时，把原main.py改名为
# vision_main.py，再把本实验文件复制为main.py，优先导入改名后的原视觉模块。
try:
    import vision_main
except ImportError:
    import main as vision_main
from communication.mjpeg_stream import MjpegStreamer
from communication.uart import VisionUart
from control.filter import ExponentialFilter
from utils.logger import log_error, log_info


# =============================================================================
# 实验开关：首轮只测试“识别+图传”，确认稳定后再开启控制UART
# =============================================================================
STREAM_TEST_WIFI_SSID = "OLIVERYANG"
STREAM_TEST_WIFI_PASSWORD = "67676767"

STREAM_TEST_ENABLED = True
STREAM_TEST_UART_ENABLED = False
STREAM_TEST_TELEMETRY_LOG_ENABLED = False
STREAM_TEST_DISPLAY_TO_IDE = False
STREAM_TEST_CONSOLE_ENABLED = False

STREAM_WIDTH = 640
STREAM_HEIGHT = 480
STREAM_TARGET_FPS = 10
STREAM_JPEG_QUALITY = 36
STREAM_PORT = 8080

# 10 FPS、单客户端、一次视觉循环最多发送16 KB；超过180 ms未发完则断开重连。
# 此期限只约束图传客户端；UART和视觉在此期间仍按每个主循环正常执行。
STREAM_SEND_CHUNK_BYTES = 16384
STREAM_FRAME_DEADLINE_MS = 180
STREAM_STATUS_INTERVAL_FRAMES = 180
STREAM_BOOT_LOG_PATH = "/sdcard/k230_stream_boot.log"


def _boot_stage(name):
    """追加一次启动里程碑；仅启动阶段调用，不进入逐帧实时路径。"""
    try:
        log_file = open(STREAM_BOOT_LOG_PATH, "a")
        log_file.write("run_stage={}\n".format(name))
        log_file.close()
    except BaseException:
        pass


def _configure_experiment():
    """复用现有现场参数，只覆盖实验的安全与调试开关。"""
    vision_main._apply_field_tuning()
    if not config.BALL_GRAYSCALE_ENABLED:
        raise RuntimeError("stream test requires grayscale vision channel")
    if config.CAMERA_WIDTH != STREAM_WIDTH or config.CAMERA_HEIGHT != STREAM_HEIGHT:
        raise ValueError("stream size must match current 640x480 sensor mode")
    config.UART_ENABLED = bool(STREAM_TEST_UART_ENABLED)
    config.TELEMETRY_LOG_ENABLED = bool(STREAM_TEST_TELEMETRY_LOG_ENABLED)
    config.DISPLAY_TO_IDE = bool(STREAM_TEST_DISPLAY_TO_IDE)


def _create_streamer():
    return MjpegStreamer(
        width=STREAM_WIDTH,
        height=STREAM_HEIGHT,
        target_fps=STREAM_TARGET_FPS,
        jpeg_quality=STREAM_JPEG_QUALITY,
        port=STREAM_PORT,
        send_chunk_bytes=STREAM_SEND_CHUNK_BYTES,
        frame_deadline_ms=STREAM_FRAME_DEADLINE_MS,
        # 当前640x480灰度视觉基线约90 FPS；双通道后以实机Sensor日志为准。
        source_fps=90,
        output_fps=STREAM_TARGET_FPS,
        output_buffers=4,
    )


def _show_startup_message(sensor, text):
    """在无IDE启动阶段用灰度通道显示一条短状态；失败时静默返回。"""
    try:
        status_frame = sensor.snapshot(chn=CAM_CHN_ID_1)
        if config.CROP_ENABLED:
            status_frame = status_frame.crop(
                roi=(
                    config.CROP_X,
                    config.CROP_Y,
                    config.CROP_WIDTH,
                    config.CROP_HEIGHT,
                )
            )
        status_frame.draw_string_advanced(4, 20, 18, text, color=255)
        Display.show_image(status_frame)
    except BaseException:
        pass


def run():
    """运行双通道实验；网络始终低于识别、UART和本地显示优先级。"""
    sensor = None
    vision_uart = None
    streamer = None
    media_initialized = False
    display_initialized = False
    sensor_started = False

    _boot_stage("run_enter")
    _boot_stage("configure_begin")
    _configure_experiment()
    _boot_stage("configure_ok")
    detector = vision_main._create_detector()
    position_filter = ExponentialFilter(config.BALL_FILTER_ALPHA)
    required_api = detector.required_api_name()
    if not detector.capability_report()[required_api]:
        raise RuntimeError("required API missing: cv_lite." + required_api)
    _boot_stage("detector_ok")

    clock = time.clock()
    frame_count = 0
    status_count = 0
    uart_discard_buffer = bytearray(256)
    stream_lcd_text = "STREAM OFF"

    try:
        if STREAM_TEST_ENABLED:
            streamer = _create_streamer()
            _boot_stage("streamer_created")

        _boot_stage("sensor_create_begin")
        sensor = Sensor(width=config.CAMERA_WIDTH, height=config.CAMERA_HEIGHT)
        _boot_stage("sensor_create_ok")
        _boot_stage("sensor_reset_begin")
        sensor.reset()
        _boot_stage("sensor_reset_ok")
        sensor.set_hmirror(config.CAMERA_HMIRROR)
        sensor.set_vflip(config.CAMERA_VFLIP)
        sensor.auto_exposure(config.CAMERA_AUTO_EXPOSURE)
        _boot_stage("sensor_controls_ok")

        # CH0只供硬件JPEG编码；CH1维持现有灰度识别坐标系和软件裁剪。
        sensor.set_framesize(
            width=STREAM_WIDTH,
            height=STREAM_HEIGHT,
            chn=CAM_CHN_ID_0,
            alignment=12,
        )
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        _boot_stage("sensor_channel0_ok")
        sensor.set_framesize(
            width=config.CAMERA_WIDTH,
            height=config.CAMERA_HEIGHT,
            chn=CAM_CHN_ID_1,
        )
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
        _boot_stage("sensor_channel1_ok")

        _boot_stage("display_init_begin")
        Display.init(
            vision_main._display_type(),
            to_ide=config.DISPLAY_TO_IDE,
        )
        display_initialized = True
        _boot_stage("display_init_ok")

        if streamer is not None:
            _boot_stage("venc_prepare_begin")
            streamer.prepare_media(sensor, CAM_CHN_ID_0)
            _boot_stage("venc_prepare_ok")

        _boot_stage("media_init_begin")
        MediaManager.init()
        media_initialized = True
        _boot_stage("media_init_ok")

        if streamer is not None:
            _boot_stage("encoder_start_begin")
            streamer.start_encoder()
            _boot_stage("encoder_start_ok")
        _boot_stage("sensor_run_begin")
        sensor.run()
        sensor_started = True
        _boot_stage("sensor_run_ok")

        # 先点亮LCD再连接热点，避免20秒连接等待期间被误认为程序完全未启动。
        time.sleep_ms(200)
        if streamer is not None:
            _show_startup_message(sensor, "WIFI CONNECTING...")
            # UART尚未创建，因此热点连接等待不会让实验版输出半截控制数据。
            _boot_stage("wifi_connect_begin")
            streamer.connect_wifi(
                STREAM_TEST_WIFI_SSID,
                STREAM_TEST_WIFI_PASSWORD,
            )
            _boot_stage("wifi_connect_ok")
            streamer.start_server()
            _boot_stage("server_start_ok")
            stream_lcd_text = "http://{}:{}/".format(streamer.ip, STREAM_PORT)

        _boot_stage("uart_create_begin")
        vision_uart = VisionUart(
            uart_id=config.UART_ID,
            baudrate=config.UART_BAUDRATE,
            tx_pin=config.UART_TX_PIN,
            rx_pin=config.UART_RX_PIN,
            enabled=config.UART_ENABLED,
        )
        _boot_stage("uart_create_ok")

        time.sleep_ms(config.CAMERA_WARMUP_MS)
        gc.collect()
        log_info(
            "stream_test started uart={} stream={} target_fps={}".format(
                1 if config.UART_ENABLED else 0,
                1 if streamer is not None else 0,
                STREAM_TARGET_FPS,
            )
        )
        _boot_stage("vision_loop_begin")

        while True:
            clock.tick()
            frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            if config.CROP_ENABLED:
                frame = frame.crop(
                    roi=(
                        config.CROP_X,
                        config.CROP_Y,
                        config.CROP_WIDTH,
                        config.CROP_HEIGHT,
                    )
                )

            result = vision_main._add_control_measurement(
                detector.detect(frame),
                position_filter,
            )

            if result is None:
                vision_uart.send_measurement(False, False, 0, -1)
            else:
                vision_uart.send_measurement(
                    True,
                    result["ball_safe"],
                    result["error_px"],
                    result["filtered_x"],
                )

            # 实验阶段不写TF卡，但开启UART后仍清空STM32回传，避免RX缓冲堆满。
            if config.UART_ENABLED:
                vision_uart.readinto(uart_discard_buffer)

            vision_main._draw_status(
                frame,
                result,
                clock.fps(),
                True,
            )
            # 无IDE时直接从LCD读取接收地址；字符串只在启动后生成一次。
            frame.draw_string_advanced(
                4,
                24,
                14,
                stream_lcd_text,
                color=200,
            )
            Display.show_image(frame)

            # 图传最后执行；一次service最多推进一次非阻塞socket.send。
            if streamer is not None:
                streamer.service()

            if (
                STREAM_TEST_CONSOLE_ENABLED
                and frame_count % config.CONSOLE_INTERVAL_FRAMES == 0
            ):
                vision_main._print_result(result, clock.fps())

            status_count += 1
            if status_count >= STREAM_STATUS_INTERVAL_FRAMES:
                if streamer is not None:
                    print(
                        "vision_fps={:.1f} ".format(clock.fps())
                        + streamer.status()
                    )
                status_count = 0

            frame_count += 1
            if frame_count >= config.GC_INTERVAL_FRAMES:
                gc.collect()
                frame_count = 0
            os.exitpoint()

    except KeyboardInterrupt:
        log_info("Stream test stopped by user")
    except BaseException as exc:
        log_error("Stream test fatal: {}".format(exc))
        if sensor_started and sensor is not None:
            _show_startup_message(sensor, "ERROR: CHECK SD LOG")
            time.sleep_ms(3000)
        raise
    finally:
        # 释放顺序：网络 -> Sensor -> VENC/link -> Display -> Media -> UART。
        if streamer is not None:
            streamer.close_network()
        if sensor_started and sensor is not None:
            try:
                sensor.stop()
            except BaseException:
                pass
        if streamer is not None:
            streamer.stop_encoder_and_unlink()
        if display_initialized:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_initialized:
            MediaManager.deinit()
        if vision_uart is not None:
            vision_uart.deinit()
        gc.collect()
        log_info("Stream test resources released")


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    run()
