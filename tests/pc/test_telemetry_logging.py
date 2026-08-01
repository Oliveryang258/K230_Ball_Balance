"""PC端协议联调测试：模拟STM32分段发送，验证K230记录和CSV解码。"""

import struct
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from communication.telemetry_logger import (  # noqa: E402
    TELEMETRY_CRC_OFFSET,
    TELEMETRY_PACKET_SIZE,
    TelemetryLogger,
    crc16_ccitt_false,
)
from tools.decode_k230_telemetry import decode_packet  # noqa: E402


class FakeUart:
    def __init__(self, chunks):
        self._chunks = list(chunks)

    def readinto(self, buffer):
        if not self._chunks:
            return None
        chunk = self._chunks.pop(0)
        buffer[: len(chunk)] = chunk
        return len(chunk)


def make_packet():
    packet = bytearray(TELEMETRY_PACKET_SIZE)
    packet[0:4] = b"TM\x02\x40"
    struct.pack_into("<H", packet, 4, 17)
    struct.pack_into("<I", packet, 6, 123456)
    struct.pack_into("<H", packet, 10, 321)
    struct.pack_into("<h", packet, 12, 314)
    struct.pack_into("<h", packet, 14, -23)
    struct.pack_into("<h", packet, 16, 45)
    struct.pack_into("<h", packet, 18, -62)
    struct.pack_into("<h", packet, 20, 31)
    struct.pack_into("<h", packet, 22, -4)
    struct.pack_into("<h", packet, 24, -35)
    struct.pack_into("<H", packet, 26, 1535)
    struct.pack_into("<H", packet, 28, 1520)
    packet[30:36] = bytes((0, 1, 0x9D, 2, 3, 1))
    struct.pack_into("<h", packet, 36, -120)
    struct.pack_into("<h", packet, 38, 345)
    struct.pack_into("<H", packet, 40, 88)
    struct.pack_into("<h", packet, 42, -9)
    struct.pack_into("<h", packet, 44, 200)
    struct.pack_into("<h", packet, 46, 220)
    struct.pack_into("<h", packet, 48, 20)
    struct.pack_into("<H", packet, 50, 77)
    struct.pack_into("<H", packet, 52, 12)
    crc = crc16_ccitt_false(packet, 2, TELEMETRY_CRC_OFFSET - 2)
    struct.pack_into("<H", packet, TELEMETRY_CRC_OFFSET, crc)
    return packet


class TelemetryLoggingTest(unittest.TestCase):
    def test_fragmented_uart_record_and_decode(self):
        packet = make_packet()
        chunks = (b"\x00\x54", packet[1:19], packet[19:57], packet[57:])

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "ball_run.bin"
            logger = TelemetryLogger(
                str(output_path),
                rx_buffer_size=128,
                write_buffer_size=64,
                enabled=True,
            )
            uart = FakeUart(chunks)
            for _ in chunks:
                logger.poll(uart)
            logger.close()

            self.assertEqual(logger.valid_frames, 1)
            self.assertEqual(logger.crc_errors, 0)
            self.assertEqual(output_path.read_bytes(), packet)

            row = decode_packet(output_path.read_bytes())
            self.assertEqual(row["telemetry_sequence"], 17)
            self.assertEqual(row["stm32_tick_ms"], 123456)
            self.assertEqual(row["error_px"], -23)
            self.assertEqual(row["yaw_rate_dps"], 34.5)
            self.assertEqual(row["motion_age_ms"], 12)

    def test_periodic_checkpoint_is_visible_before_close(self):
        packet = make_packet()
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "ball_run.bin"
            logger = TelemetryLogger(
                str(output_path),
                rx_buffer_size=128,
                write_buffer_size=64,
                sync_interval_blocks=1,
                enabled=True,
            )
            logger.poll(FakeUart((packet,)))

            # 模拟脱机程序仍在运行、尚未进入close()：检查点之后文件已经可读。
            self.assertEqual(output_path.read_bytes(), packet)
            self.assertEqual(logger.sync_count, 1)
            logger.close()


if __name__ == "__main__":
    unittest.main()
