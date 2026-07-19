"""PC-only tests; never deploy this module to the K230."""

import pathlib
import sys
import unittest


SRC_DIR = pathlib.Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC_DIR))

from control.filter import ExponentialFilter
from communication.uart import format_measurement
from vision.geometry import estimate_rectangle_axis, normalized_track_error, project_ratio


class GeometryTests(unittest.TestCase):
    def test_axis_aligned_rectangle(self):
        result = estimate_rectangle_axis([(0, 0), (100, 0), (100, 20), (0, 20)])
        self.assertEqual(result["center"], (50, 10))
        self.assertAlmostEqual(result["angle"], 0.0)
        self.assertAlmostEqual(result["length"], 100.0)
        self.assertAlmostEqual(result["width"], 20.0)
        self.assertAlmostEqual(result["aspect_ratio"], 5.0)

    def test_rotated_rectangle(self):
        # 长轴沿 y=x，期望角度约为 45 度。
        result = estimate_rectangle_axis([(0, 0), (10, -10), (60, 40), (50, 50)])
        self.assertEqual(result["center"], (30, 20))
        self.assertAlmostEqual(result["angle"], 45.0)
        self.assertGreater(result["length"], result["width"])

    def test_rectangle_requires_four_corners(self):
        with self.assertRaises(ValueError):
            estimate_rectangle_axis([(0, 0), (1, 1)])

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


class FilterTests(unittest.TestCase):
    def test_exponential_filter(self):
        filter_ = ExponentialFilter(0.5)
        self.assertEqual(filter_.update(2.0), 2.0)
        self.assertEqual(filter_.update(4.0), 3.0)
        filter_.reset()
        self.assertIsNone(filter_.value)


class UartFormatTests(unittest.TestCase):
    def test_valid_measurement_uses_integer_error(self):
        self.assertEqual(format_measurement(0.1256, (320, 240)), "BALL,1,126,320,240\n")

    def test_invalid_measurement_is_explicit(self):
        self.assertEqual(format_measurement(None, valid=False), "BALL,0,0,-1,-1\n")


if __name__ == "__main__":
    unittest.main()
