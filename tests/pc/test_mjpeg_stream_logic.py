"""PC端纯逻辑测试；不证明K230固件或硬件VENC兼容。"""

import importlib
import sys
import types
import unittest
from pathlib import Path


PACKETS = {}


def _install_canmv_stubs():
    network = types.ModuleType("network")
    network.STA_IF = 0
    network.WLAN = object
    sys.modules["network"] = network

    uctypes = types.ModuleType("uctypes")
    uctypes.bytearray_at = lambda address, size: PACKETS[address][:size]
    sys.modules["uctypes"] = uctypes

    media = types.ModuleType("media")
    media.__path__ = []
    sys.modules["media"] = media

    media_media = types.ModuleType("media.media")
    media_media.MediaManager = type("MediaManager", (), {})
    media_media.VIDEO_ENCODE_MOD_ID = 1
    media_media.VENC_DEV_ID = 2
    sys.modules["media.media"] = media_media

    vencoder = types.ModuleType("media.vencoder")

    class Encoder:
        PAYLOAD_TYPE_JPEG = 3
        H264_PROFILE_MAIN = 0

    class StreamData:
        pass

    vencoder.Encoder = Encoder
    vencoder.StreamData = StreamData
    vencoder.VENC_CHN_ID_0 = 0
    vencoder.VENC_CHN_ID_MAX = 4
    sys.modules["media.vencoder"] = vencoder


_install_canmv_stubs()
SRC = Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC))
mjpeg_stream = importlib.import_module("communication.mjpeg_stream")


class FakeStreamData:
    def __init__(self, packets, pts):
        PACKETS.clear()
        self.pack_cnt = len(packets)
        self.data = []
        self.data_size = []
        self.pts = list(pts)
        for index, packet in enumerate(packets):
            address = index + 100
            PACKETS[address] = bytearray(packet)
            self.data.append(address)
            self.data_size.append(len(packet))


class FakeClient:
    def __init__(self):
        self.calls = 0

    def send(self, data):
        self.calls += 1
        return len(data)


class BusyClient:
    def __init__(self):
        self.calls = 0

    def send(self, data):
        self.calls += 1
        raise OSError(11)


class MjpegStreamLogicTests(unittest.TestCase):
    def test_selects_latest_complete_jpeg_by_pts(self):
        old = b"\xff\xd8old\xff\xd9"
        new = b"\xff\xd8new\xff\xd9"
        stream = FakeStreamData([old, new], [10, 11])
        output = bytearray(64)
        size = mjpeg_stream._copy_latest_jpeg(stream, memoryview(output))
        self.assertEqual(new, bytes(output[:size]))

    def test_concatenates_same_pts_fragments(self):
        stream = FakeStreamData([b"\xff\xd8abc", b"def\xff\xd9"], [7, 7])
        output = bytearray(64)
        size = mjpeg_stream._copy_latest_jpeg(stream, memoryview(output))
        self.assertEqual(b"\xff\xd8abcdef\xff\xd9", bytes(output[:size]))

    def test_reports_oversize_frame(self):
        stream = FakeStreamData([b"\xff\xd8abcdef\xff\xd9"], [1])
        self.assertEqual(
            -1,
            mjpeg_stream._copy_latest_jpeg(stream, memoryview(bytearray(4))),
        )

    def test_one_tx_call_advances_only_one_segment(self):
        streamer = mjpeg_stream.MjpegStreamer(max_jpeg_bytes=64)
        streamer.client = FakeClient()
        streamer.client_mode = "stream"
        streamer._queue_tx((memoryview(b"a"), memoryview(b"b")))
        streamer._service_tx_once(1)
        self.assertEqual(1, streamer.client.calls)
        self.assertEqual(1, streamer.tx_index)

    def test_tx_budget_can_advance_three_segments(self):
        streamer = mjpeg_stream.MjpegStreamer(
            max_jpeg_bytes=64,
            max_send_calls_per_service=3,
        )
        streamer.client = FakeClient()
        streamer.client_mode = "stream"
        streamer._queue_tx(
            (
                memoryview(b"a"),
                memoryview(b"b"),
                memoryview(b"c"),
                memoryview(b"d"),
            )
        )
        streamer._service_tx_budget(1)
        self.assertEqual(3, streamer.client.calls)
        self.assertEqual(3, streamer.tx_index)

    def test_tx_budget_stops_immediately_when_socket_is_busy(self):
        streamer = mjpeg_stream.MjpegStreamer(
            max_jpeg_bytes=64,
            max_send_calls_per_service=3,
        )
        streamer.client = BusyClient()
        streamer.client_mode = "stream"
        streamer._queue_tx((memoryview(b"a"), memoryview(b"b")))
        streamer._service_tx_budget(1)
        self.assertEqual(1, streamer.client.calls)
        self.assertEqual(0, streamer.tx_index)

    def test_part_header_contains_decimal_length(self):
        streamer = mjpeg_stream.MjpegStreamer(max_jpeg_bytes=64)
        header = bytes(streamer._prepare_part_header(12345))
        self.assertTrue(header.endswith(b"12345\r\n\r\n"))


if __name__ == "__main__":
    unittest.main()
