"""PC-only TrackDetector logic tests with a fake cv_lite module.

These tests verify selection and fallback logic only. They do not prove that the real
Yahboom CanMV v1.8.0 firmware implements the same API behavior.
"""

import pathlib
import sys
import types
import unittest


SRC_DIR = pathlib.Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC_DIR))

# TrackDetector imports cv_lite at module load time. Computer A does not provide it,
# so install a tiny fake module only inside this PC test process.
FAKE_CV_LITE = types.ModuleType("cv_lite")
sys.modules["cv_lite"] = FAKE_CV_LITE

from vision.track_detector import ORIENTED_RECT_API, TrackDetector


class FakeImage:
    def to_numpy_ref(self):
        return "fake-image-reference"


def make_detector(use_oriented_rect=True, allow_bbox_fallback=True):
    return TrackDetector(
        image_width=640,
        image_height=480,
        rgb_threshold=[130, 255, 90, 255, 0, 150],
        min_area=1200,
        kernel_size=3,
        roi=(0, 0, 640, 480),
        min_bbox_aspect_ratio=2.5,
        use_oriented_rect=use_oriented_rect,
        allow_bbox_fallback=allow_bbox_fallback,
        rect_canny_low=50,
        rect_canny_high=150,
        rect_approx_epsilon=0.04,
        rect_min_area_ratio=0.001,
        rect_max_angle_cos=0.5,
        rect_gaussian_size=5,
        rect_min_overlap=0.20,
        rect_min_aspect_ratio=2.5,
    )


class TrackDetectorTests(unittest.TestCase):
    def setUp(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [100, 200, 400, 40]
        if hasattr(FAKE_CV_LITE, ORIENTED_RECT_API):
            delattr(FAKE_CV_LITE, ORIENTED_RECT_API)

    def test_bbox_fallback_when_oriented_api_is_missing(self):
        detector = make_detector()
        result = detector.detect(FakeImage())
        self.assertTrue(result["track_valid"])
        self.assertEqual(result["center"], (300, 220))
        self.assertEqual(result["angle"], 0.0)
        self.assertEqual(result["length"], 400.0)
        self.assertEqual(result["angle_source"], "bbox_approx")

    def test_oriented_rectangle_is_used_when_available(self):
        setattr(
            FAKE_CV_LITE,
            ORIENTED_RECT_API,
            lambda *args: [[100, 200, 400, 40, 100, 200, 500, 200, 500, 240, 100, 240]],
        )
        detector = make_detector()
        result = detector.detect(FakeImage())
        self.assertEqual(result["angle_source"], "oriented_rect")
        self.assertAlmostEqual(result["angle"], 0.0)
        self.assertAlmostEqual(result["length"], 400.0)

    def test_non_elongated_blob_is_rejected(self):
        FAKE_CV_LITE.rgb888_find_blobs = lambda *args: [100, 100, 80, 80]
        detector = make_detector()
        self.assertIsNone(detector.detect(FakeImage()))

    def test_no_fallback_returns_invalid_without_rectangle(self):
        detector = make_detector(allow_bbox_fallback=False)
        self.assertIsNone(detector.detect(FakeImage()))


if __name__ == "__main__":
    unittest.main()

