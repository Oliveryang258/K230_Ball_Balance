"""PC-only tests; never deploy this module to the K230."""

import pathlib
import sys
import unittest


SRC_DIR = pathlib.Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC_DIR))

from control.filter import ExponentialFilter
from communication.uart import encode_measurement
from vision.geometry import (
    normalized_track_error,
    pixel_position_error,
    position_is_safe,
    project_ratio,
)


class GeometryTests(unittest.TestCase):
    def test_projection_endpoints_and_center(self):
        fixed = (0, 0)
        servo = (10, 0)
        self.assertEqual(project_ratio(fixed, fixed, servo), 0.0)
        self.assertEqual(normalized_track_error((5, 0), fixed, servo), 0.0)
        self.assertEqual(normalized_track_error(servo, fixed, servo), 1.0)
        self.assertEqual(normalized_track_error(fixed, fixed, servo), -1.0)

    def test_projection_handles_rotated_track(self):
        self.assertEqual(normalized_track_error((5, 5), (0, 0), (10, 10)), 0.0)

    def test_error_is_clamped_by_default(self):
        self.assertEqual(normalized_track_error((20, 0), (0, 0), (10, 0)), 1.0)
        self.assertEqual(normalized_track_error((20, 0), (0, 0), (10, 0), clamp=False), 3.0)

    def test_coincident_markers_are_rejected(self):
        with self.assertRaises(ValueError):
            project_ratio((1, 1), (0, 0), (0, 0))

    def test_pixel_error_uses_fixed_end_negative_direction(self):
        self.assertEqual(pixel_position_error(625, 361), -264)
        self.assertEqual(pixel_position_error(361, 361), 0)
        self.assertEqual(pixel_position_error(24, 361), 337)

    def test_safe_pixel_range_is_inclusive(self):
        self.assertTrue(position_is_safe(60, 60, 598))
        self.assertTrue(position_is_safe(598, 60, 598))
        self.assertFalse(position_is_safe(59, 60, 598))
        self.assertFalse(position_is_safe(599, 60, 598))


class FilterTests(unittest.TestCase):
    def test_exponential_filter(self):
        filter_ = ExponentialFilter(0.5)
        self.assertEqual(filter_.update(2.0), 2.0)
        self.assertEqual(filter_.update(4.0), 3.0)
        filter_.reset()
        self.assertIsNone(filter_.value)


class UartFormatTests(unittest.TestCase):
    def test_valid_safe_measurement_matches_stm32_v1_layout(self):
        frame = encode_measurement(
            frame_id=0,
            ball_valid=True,
            ball_safe=True,
            error_px=123,
            ball_x=238,
        )
        self.assertEqual(
            bytes(frame),
            bytes((0xAA, 0x55, 0x01, 0x03, 0x00, 0x00, 0x7B, 0x00, 0xEE, 0x00, 0x97)),
        )

    def test_invalid_measurement_cannot_look_like_zero_error(self):
        frame = encode_measurement(
            frame_id=0x1234,
            ball_valid=False,
            ball_safe=True,
            error_px=99,
            ball_x=320,
        )
        self.assertEqual(frame[3], 0)
        self.assertEqual(bytes(frame[4:10]), bytes((0x34, 0x12, 0x00, 0x00, 0xFF, 0xFF)))
        checksum = 0
        for value in frame[2:10]:
            checksum ^= value
        self.assertEqual(frame[10], checksum)


if __name__ == "__main__":
    unittest.main()
