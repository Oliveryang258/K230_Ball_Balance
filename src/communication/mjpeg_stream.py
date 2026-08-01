"""K230双通道实验用低阻塞MJPEG发送器。

CanMV API：network.WLAN、socket、media.vencoder、MediaManager.link、
          Sensor.bind_info()、uctypes.bytearray_at()。
硬件：Yahboom K230 12Pin、板载Wi-Fi、支持JPEG VENC的摄像头固件。
兼容性：基于队友已使用的旧版media.vencoder接口和K230多通道Sensor文档；
        Yahboom CanMV v1.8.0上的双通道组合仍必须实机验证。

实时约束：service()每次只调用有上限次数的非阻塞socket.send()。网络拥塞时关闭
客户端并等待浏览器重连，绝不在视觉主循环中等待完整JPEG发送。
"""

import gc
import network
import os
import socket
import time
import uctypes

import media.vencoder as vencmod
from media.media import MediaManager
from media.media import VIDEO_ENCODE_MOD_ID, VENC_DEV_ID
from media.vencoder import Encoder, StreamData
from media.vencoder import VENC_CHN_ID_0, VENC_CHN_ID_MAX


_STREAM_RESPONSE = (
    b"HTTP/1.1 200 OK\r\n"
    b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    b"Cache-Control: no-store, no-cache, must-revalidate\r\n"
    b"Pragma: no-cache\r\n"
    b"Connection: close\r\n\r\n"
)

_INDEX_HTML = (
    b"<!doctype html><html><head><meta charset=utf-8>"
    b"<meta name=viewport content='width=device-width,initial-scale=1'>"
    b"<title>K230 Stream Test</title>"
    b"<style>html,body{margin:0;width:100%;height:100%;background:#111}"
    b"img{display:block;width:100%;height:100%;object-fit:contain}</style>"
    b"</head><body><img src='/stream'></body></html>"
)

_INDEX_RESPONSE = (
    b"HTTP/1.1 200 OK\r\n"
    b"Content-Type: text/html; charset=utf-8\r\n"
    b"Cache-Control: no-store\r\n"
    b"Connection: close\r\n"
    b"Content-Length: " + str(len(_INDEX_HTML)).encode() + b"\r\n\r\n"
    + _INDEX_HTML
)

_PART_PREFIX = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
_PART_HEADER_END = b"\r\n\r\n"
_PART_END = b"\r\n"

_STREAM_RESPONSE_VIEW = memoryview(_STREAM_RESPONSE)
_INDEX_RESPONSE_VIEW = memoryview(_INDEX_RESPONSE)
_PART_END_VIEW = memoryview(_PART_END)


def _error_number(error):
    number = getattr(error, "errno", None)
    if number is None and getattr(error, "args", None):
        number = error.args[0]
    return number


def _temporary_socket_error(error):
    return _error_number(error) in (11, 110, 115)


def _packet_is_complete_jpeg(packet, size):
    return (
        size >= 4
        and packet[0] == 0xFF
        and packet[1] == 0xD8
        and packet[size - 2] == 0xFF
        and packet[size - 1] == 0xD9
    )


def _copy_packet(stream_data, packet_index, destination, offset):
    size = int(stream_data.data_size[packet_index])
    if offset + size > len(destination):
        return -1
    packet = uctypes.bytearray_at(stream_data.data[packet_index], size)
    destination[offset:offset + size] = packet
    return size


def _copy_latest_jpeg(stream_data, destination):
    """把GetStream返回的最新完整JPEG复制到预分配缓冲区。

    返回正数表示JPEG长度，0表示数据不完整，-1表示缓冲区不足。
    兼容“每个pack是一帧”和“多个同PTS pack组成一帧”两种旧固件行为。
    """
    pack_count = int(stream_data.pack_cnt)
    if pack_count <= 0:
        return 0

    latest_pts = stream_data.pts[0]
    all_same_pts = True
    index = 1
    while index < pack_count:
        pts = stream_data.pts[index]
        if pts != latest_pts:
            all_same_pts = False
        if pts > latest_pts:
            latest_pts = pts
        index += 1

    position = 0
    if not all_same_pts:
        index = 0
        while index < pack_count:
            if stream_data.pts[index] == latest_pts:
                copied = _copy_packet(stream_data, index, destination, position)
                if copied < 0:
                    return -1
                position += copied
            index += 1
    else:
        newest_complete = -1
        index = 0
        while index < pack_count:
            size = int(stream_data.data_size[index])
            packet = uctypes.bytearray_at(stream_data.data[index], size)
            if _packet_is_complete_jpeg(packet, size):
                newest_complete = index
            index += 1

        if newest_complete >= 0:
            copied = _copy_packet(stream_data, newest_complete, destination, 0)
            if copied < 0:
                return -1
            position = copied
        else:
            index = 0
            while index < pack_count:
                copied = _copy_packet(stream_data, index, destination, position)
                if copied < 0:
                    return -1
                position += copied
                index += 1

    if position < 4:
        return 0
    if destination[0] != 0xFF or destination[1] != 0xD8:
        return 0

    if destination[position - 2] != 0xFF or destination[position - 1] != 0xD9:
        search_start = max(2, position - 34)
        index = position - 2
        found = 0
        while index >= search_start:
            if destination[index] == 0xFF and destination[index + 1] == 0xD9:
                found = index + 2
                break
            index -= 1
        if found == 0:
            return 0
        position = found

    return position


class _JpegEncoder(Encoder):
    """为旧固件补充MJPEG FIXQP通道创建。"""

    def create_jpeg(self, channel, width, height, quality, src_fps, dst_fps):
        if channel > VENC_CHN_ID_MAX - 1:
            raise ValueError("VENC channel out of range")

        required_names = (
            "kd_mpi_venc_attach_vb_pool",
            "k_venc_chn_attr",
            "K_VENC_RC_MODE_MJPEG_FIXQP",
            "kd_mpi_venc_create_chn",
        )
        for name in required_names:
            if not hasattr(vencmod, name):
                raise RuntimeError("firmware lacks VENC symbol: " + name)

        result = vencmod.kd_mpi_venc_attach_vb_pool(channel, self.private_poolid)
        if result not in (0, None):
            raise OSError("VENC attach pool failed: {}".format(result))

        attributes = vencmod.k_venc_chn_attr()
        attributes.venc_attr.type = self.PAYLOAD_TYPE_JPEG
        attributes.venc_attr.pic_width = int(width)
        attributes.venc_attr.pic_height = int(height)
        # 旧封装保留该字段；JPEG载荷下编码器通常忽略H264 profile。
        attributes.venc_attr.profile = self.H264_PROFILE_MAIN
        attributes.rc_attr.rc_mode = vencmod.K_VENC_RC_MODE_MJPEG_FIXQP
        attributes.rc_attr.mjpeg_fixqp.src_frame_rate = int(src_fps)
        attributes.rc_attr.mjpeg_fixqp.dst_frame_rate = int(dst_fps)
        attributes.rc_attr.mjpeg_fixqp.q_factor = int(quality)

        result = vencmod.kd_mpi_venc_create_chn(channel, attributes)
        if result != 0:
            raise OSError("VENC JPEG channel creation failed: {}".format(result))


class MjpegStreamer:
    """单客户端、协作式、低阻塞MJPEG服务器。"""

    def __init__(
        self,
        width=640,
        height=480,
        target_fps=10,
        jpeg_quality=36,
        port=8080,
        max_jpeg_bytes=128 * 1024,
        send_chunk_bytes=16384,
        max_send_calls_per_service=1,
        frame_deadline_ms=180,
        request_deadline_ms=1000,
        source_fps=60,
        output_fps=10,
        output_buffers=4,
    ):
        self.width = int(width)
        self.height = int(height)
        self.target_fps = max(1, int(target_fps))
        self.frame_interval_ms = max(1, 1000 // self.target_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.port = int(port)
        self.max_jpeg_bytes = int(max_jpeg_bytes)
        self.send_chunk_bytes = max(512, int(send_chunk_bytes))
        self.max_send_calls_per_service = max(
            1, int(max_send_calls_per_service)
        )
        self.frame_deadline_ms = int(frame_deadline_ms)
        self.request_deadline_ms = int(request_deadline_ms)
        self.source_fps = int(source_fps)
        self.output_fps = int(output_fps)
        self.output_buffers = int(output_buffers)

        self.channel = VENC_CHN_ID_0
        self.encoder = None
        self.link = None
        self.encoder_created = False
        self.encoder_started = False

        self.wlan = None
        self.ip = None
        self.server = None
        self.client = None
        self.client_mode = None
        self.client_started_ms = 0
        self.request = bytearray()

        self.frame_buffer = bytearray(self.max_jpeg_bytes)
        self.frame_view = memoryview(self.frame_buffer)
        self.stream_data = StreamData()
        # 每帧复用同一HTTP分段头缓冲，避免10 FPS下反复创建短bytes对象。
        self.part_header_buffer = bytearray(96)
        self.part_header_view = memoryview(self.part_header_buffer)
        self.part_header_prefix_length = len(_PART_PREFIX)
        self.part_header_buffer[:self.part_header_prefix_length] = _PART_PREFIX

        self.tx_segments = None
        self.tx_index = 0
        self.tx_offset = 0
        self.tx_close_after = False
        self.frame_tx_started_ms = 0
        self.next_frame_due_ms = 0
        self.next_drain_due_ms = 0

        self.sent_frames = 0
        self.dropped_frames = 0
        self.disconnects = 0
        self.accepted_clients = 0
        self.max_service_ms = 0

    def connect_wifi(self, ssid, password, timeout_ms=20000):
        """视觉硬件启动前连接Wi-Fi，避免连接等待打断运行中的控制链。"""
        if not ssid:
            raise ValueError("STREAM_TEST_WIFI_SSID is empty")
        wlan = network.WLAN(network.STA_IF)
        if not wlan.isconnected():
            print("wifi_connect ssid=" + str(ssid))
            wlan.connect(ssid, password)
            started = time.ticks_ms()
            while not wlan.isconnected():
                if time.ticks_diff(time.ticks_ms(), started) >= int(timeout_ms):
                    raise RuntimeError("Wi-Fi connection timeout")
                os.exitpoint()
                time.sleep_ms(200)

        network_config = wlan.ifconfig()
        ip = network_config[0]
        if not ip or ip == "0.0.0.0":
            raise RuntimeError("Wi-Fi connected without an IP address")
        self.wlan = wlan
        self.ip = ip
        print("wifi=" + str(network_config))
        return ip

    def prepare_media(self, sensor, sensor_channel):
        """在MediaManager.init()之前配置VENC缓冲、链路和编码通道。"""
        self.encoder = _JpegEncoder()
        self.encoder.SetOutBufs(
            self.channel,
            self.output_buffers,
            self.width,
            self.height,
        )
        self.link = MediaManager.link(
            sensor.bind_info(chn=sensor_channel)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.channel),
        )
        self.encoder.create_jpeg(
            self.channel,
            self.width,
            self.height,
            self.jpeg_quality,
            self.source_fps,
            self.output_fps,
        )
        self.encoder_created = True

    def start_encoder(self):
        if self.encoder is None or not self.encoder_created:
            raise RuntimeError("prepare_media() must run before start_encoder()")
        self.encoder.Start(self.channel)
        self.encoder_started = True

    def start_server(self):
        if self.ip is None:
            raise RuntimeError("connect_wifi() must run before start_server()")
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(socket.getaddrinfo("0.0.0.0", self.port)[0][-1])
        server.listen(2)
        server.setblocking(False)
        self.server = server
        now = time.ticks_ms()
        self.next_frame_due_ms = now
        self.next_drain_due_ms = now
        print("stream_page=http://{}:{}/".format(self.ip, self.port))

    def _prepare_part_header(self, value):
        """在预分配缓冲中生成一个完整multipart帧头并返回其视图。"""
        digit_start = len(self.part_header_buffer) - len(_PART_HEADER_END) - 1
        index = digit_start
        if value == 0:
            self.part_header_buffer[index] = 48
        else:
            while value > 0:
                self.part_header_buffer[index] = 48 + (value % 10)
                value //= 10
                index -= 1
            index += 1

        digit_length = digit_start - index + 1
        write_at = self.part_header_prefix_length
        self.part_header_buffer[write_at:write_at + digit_length] = (
            self.part_header_buffer[index:index + digit_length]
        )
        write_at += digit_length
        end = write_at + len(_PART_HEADER_END)
        self.part_header_buffer[write_at:end] = _PART_HEADER_END
        return self.part_header_view[:end]

    def _queue_tx(self, segments, close_after=False, frame_started_ms=0):
        self.tx_segments = segments
        self.tx_index = 0
        self.tx_offset = 0
        self.tx_close_after = bool(close_after)
        self.frame_tx_started_ms = int(frame_started_ms)

    def _close_client(self, count_disconnect=False):
        if self.client is not None:
            try:
                self.client.close()
            except BaseException:
                pass
        if count_disconnect:
            self.disconnects += 1
        self.client = None
        self.client_mode = None
        self.request = bytearray()
        self.tx_segments = None
        self.tx_index = 0
        self.tx_offset = 0
        self.frame_tx_started_ms = 0

    def _accept_once(self):
        if self.server is None or self.client is not None:
            return
        try:
            client, address = self.server.accept()
        except OSError as error:
            if not _temporary_socket_error(error):
                raise
            return
        client.setblocking(False)
        self.client = client
        self.client_mode = "request"
        self.client_started_ms = time.ticks_ms()
        self.request = bytearray()
        self.accepted_clients += 1
        print("stream_client=" + str(address))

    def _parse_request_path(self):
        first_line = bytes(self.request).split(b"\r\n", 1)[0].split()
        if len(first_line) < 2:
            return "/"
        try:
            return first_line[1].decode().split("?", 1)[0]
        except BaseException:
            return "/"

    def _service_request(self, now):
        try:
            data = self.client.recv(256)
        except OSError as error:
            if not _temporary_socket_error(error):
                self._close_client(True)
            return

        if data:
            self.request.extend(data)
            if len(self.request) > 2048:
                self._close_client(True)
                return
        else:
            self._close_client(True)
            return

        complete = self.request.find(b"\r\n\r\n") >= 0
        if not complete and self.request.find(b"\n\n") >= 0:
            complete = True
        if complete:
            path = self._parse_request_path()
            if path == "/stream":
                self.client_mode = "stream_header"
                self._queue_tx((_STREAM_RESPONSE_VIEW,))
            else:
                self.client_mode = "page"
                self._queue_tx((_INDEX_RESPONSE_VIEW,), close_after=True)
            return

        if time.ticks_diff(now, self.client_started_ms) >= self.request_deadline_ms:
            self._close_client(True)

    def _service_tx_once(self, now):
        if self.tx_segments is None or self.client is None:
            return False
        if (
            self.frame_tx_started_ms
            and time.ticks_diff(now, self.frame_tx_started_ms)
            >= self.frame_deadline_ms
        ):
            self.dropped_frames += 1
            self._close_client(True)
            return False

        segment = self.tx_segments[self.tx_index]
        remaining = len(segment) - self.tx_offset
        length = min(remaining, self.send_chunk_bytes)
        try:
            sent = self.client.send(
                segment[self.tx_offset:self.tx_offset + length]
            )
        except OSError as error:
            if not _temporary_socket_error(error):
                self._close_client(True)
            return False
        if not sent:
            return False

        self.tx_offset += sent
        if self.tx_offset < len(segment):
            return True
        self.tx_index += 1
        self.tx_offset = 0
        if self.tx_index < len(self.tx_segments):
            return True

        was_frame = self.frame_tx_started_ms != 0
        close_after = self.tx_close_after
        previous_mode = self.client_mode
        self.tx_segments = None
        self.frame_tx_started_ms = 0
        if was_frame:
            self.sent_frames += 1
        if close_after:
            self._close_client(False)
        elif previous_mode == "stream_header":
            self.client_mode = "stream"
            self.next_frame_due_ms = now
        return True

    def _service_tx_budget(self, now):
        """在固定次数预算内推进发送；socket不可写时立即让出主循环。"""
        calls = 0
        while (
            calls < self.max_send_calls_per_service
            and self.tx_segments is not None
            and self.client is not None
        ):
            if not self._service_tx_once(now):
                break
            calls += 1

    def _get_jpeg(self):
        result = self.encoder.GetStream(self.channel, self.stream_data, 0)
        if result != 0:
            return 0
        try:
            return _copy_latest_jpeg(self.stream_data, self.frame_view)
        finally:
            self.encoder.ReleaseStream(self.channel, self.stream_data)

    def _discard_encoder_output(self, now):
        if time.ticks_diff(now, self.next_drain_due_ms) < 0:
            return
        self.next_drain_due_ms = time.ticks_add(now, 100)
        result = self.encoder.GetStream(self.channel, self.stream_data, 0)
        if result == 0:
            self.encoder.ReleaseStream(self.channel, self.stream_data)

    def _schedule_frame(self, now):
        if time.ticks_diff(now, self.next_frame_due_ms) < 0:
            return
        self.next_frame_due_ms = time.ticks_add(now, self.frame_interval_ms)
        size = self._get_jpeg()
        if size <= 0:
            self.dropped_frames += 1
            return
        part_header = self._prepare_part_header(size)
        self._queue_tx(
            (
                part_header,
                self.frame_view[:size],
                _PART_END_VIEW,
            ),
            frame_started_ms=now,
        )

    def service(self):
        """主循环每帧调用一次；按固定预算执行非阻塞send。"""
        if self.server is None or not self.encoder_started:
            return
        started = time.ticks_ms()
        now = started
        self._accept_once()

        if self.client is None:
            self._discard_encoder_output(now)
        elif self.client_mode == "request":
            self._service_request(now)
        elif self.tx_segments is not None:
            self._service_tx_budget(now)
        elif self.client_mode == "stream":
            self._schedule_frame(now)
            if self.tx_segments is not None:
                self._service_tx_budget(now)

        elapsed = time.ticks_diff(time.ticks_ms(), started)
        if elapsed > self.max_service_ms:
            self.max_service_ms = elapsed

    def status(self):
        return (
            "stream client={} sent={} drop={} disconnect={} accepted={} maxsvc={}ms mem={}"
        ).format(
            1 if self.client is not None else 0,
            self.sent_frames,
            self.dropped_frames,
            self.disconnects,
            self.accepted_clients,
            self.max_service_ms,
            gc.mem_free(),
        )

    def close_network(self):
        self._close_client(False)
        if self.server is not None:
            try:
                self.server.close()
            except BaseException:
                pass
            self.server = None

    def stop_encoder_and_unlink(self):
        if self.encoder_started and self.encoder is not None:
            try:
                self.encoder.Stop(self.channel)
            except BaseException as error:
                print("encoder.Stop failed: " + str(error))
            self.encoder_started = False
        if self.link is not None:
            try:
                if hasattr(self.link, "destroy"):
                    self.link.destroy()
            except BaseException as error:
                print("link destroy failed: " + str(error))
            self.link = None
        if self.encoder_created and self.encoder is not None:
            try:
                self.encoder.Destroy(self.channel)
            except BaseException as error:
                print("encoder.Destroy failed: " + str(error))
            self.encoder_created = False
        self.encoder = None
