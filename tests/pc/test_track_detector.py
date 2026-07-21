"""PC-only TrackDetector logic tests with a fake cv_lite module.

These tests verify Blob selection only.  They do not prove real K230 firmware
behaviour; the teammate's physical CanMV v1.8.0 device remains the final test.
"""

import pathlib
import sys
import types
import unittest


SRC_DIR = pathlib.Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC_DIR))

FAKE_CV_LITE = types.ModuleType("cv_lite")
sys.modules["cv_lite"] = FAKE_CV_LITE

from vision.track_detector import TrackDetector


class FakeImage:
    def to_numpy_ref(self):
        return "fake-image-reference"


def make_detector():
    return TrackDetector(
        image_width=640,
        image_height=480,
        rgb_threshold=[130, 255, 90, 255, 0, 150],
        min_area=1200,
        kernel_size=3,
        roi=(0, 0, 640, 480),
        min_bbox_aspect_ratio=2.5,
    )


class TrackDetectorTests(unittest.TestCase):
    def setUp(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [100, 200, 400, 40]

    def test_long_blob_produces_bbox_axis(self):
        result = make_detector().detect(FakeImage())
        self.assertTrue(result["track_valid"])
        self.assertEqual(result["center"], (300, 220))
        self.assertEqual(result["axis_start"], (100, 220))
        self.assertEqual(result["axis_end"], (500, 220))
        self.assertEqual(result["angle"], 0.0)
        self.assertEqual(result["length"], 400.0)
        self.assertEqual(result["axis_source"], "bbox_only")

    def test_vertical_blob_produces_vertical_bbox_axis(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [100, 50, 40, 300]
        result = make_detector().detect(FakeImage())
        self.assertEqual(result["axis_start"], (120, 50))
        self.assertEqual(result["axis_end"], (120, 350))
        self.assertEqual(result["angle"], -90.0)

    def test_largest_elongated_blob_is_selected(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [10, 10, 200, 20, 50, 100, 400, 40]
        result = make_detector().detect(FakeImage())
        self.assertEqual(result["bbox"], (50, 100, 400, 40))

    def test_non_elongated_blob_is_rejected(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [100, 100, 80, 80]
        self.assertIsNone(make_detector().detect(FakeImage()))


if __name__ == "__main__":
    unittest.main()
