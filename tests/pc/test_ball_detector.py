"""PC端圆候选和ROI结果筛选测试，使用假的cv_lite模块。"""

import pathlib
import sys
import types
import unittest


SRC_DIR = pathlib.Path(__file__).resolve().parents[2] / "src"
sys.path.insert(0, str(SRC_DIR))

FAKE_CV_LITE = types.ModuleType("cv_lite")
sys.modules["cv_lite"] = FAKE_CV_LITE

from vision.ball_detector import BallDetector


class FakeImage:
    def to_numpy_ref(self):
        return "fake-full-image-reference"


def make_detector(use_grayscale=False):
    return BallDetector(
        image_width=640,
        image_height=480,
        roi=(20, 205, 610, 90),
        dp=1,
        min_dist=30,
        param1=80,
        param2=20,
        min_radius=8,
        max_radius=35,
        expected_radius=17,
        max_jump_px=80,
        max_radius_change=8,
        lost_reset_frames=3,
        use_grayscale=use_grayscale,
    )


class BallDetectorTests(unittest.TestCase):
    def setUp(self):
        # cv_lite在完整图像上检测，返回640x480原图坐标。
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [358, 254, 17]

    def test_circle_in_roi_is_detected(self):
        image = FakeImage()
        result = make_detector().detect(image)

        self.assertTrue(result["ball_valid"])
        self.assertEqual(result["center"], (358, 254))
        self.assertEqual(result["radius"], 17)

    def test_cv_lite_receives_full_image_shape_and_integer_parameters(self):
        captured = []

        def fake_find_circles(*args):
            captured.extend(args)
            return []

        FAKE_CV_LITE.rgb888_find_circles = fake_find_circles
        make_detector().detect(FakeImage())

        self.assertEqual(captured[0], [480, 640])
        self.assertEqual(captured[1], "fake-full-image-reference")
        for value in captured[2:]:
            self.assertIs(type(value), int)

    def test_grayscale_mode_uses_grayscale_api(self):
        captured = []

        def fake_grayscale_find_circles(*args):
            captured.extend(args)
            return [358, 254, 17]

        FAKE_CV_LITE.grayscale_find_circles = fake_grayscale_find_circles
        result = make_detector(use_grayscale=True).detect(FakeImage())

        self.assertEqual(result["center"], (358, 254))
        self.assertEqual(captured[0], [480, 640])
        self.assertEqual(captured[1], "fake-full-image-reference")

    def test_circle_outside_roi_is_rejected(self):
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [114, 192, 20]
        detector = make_detector()
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertEqual(detector.last_raw_circle_count, 1)
        self.assertEqual(detector.last_reject_roi_count, 1)

    def test_empty_result_is_invalid(self):
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: []
        detector = make_detector()
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertEqual(detector.last_raw_circle_count, 0)
        self.assertEqual(detector.last_reject_radius_range_count, 0)
        self.assertEqual(detector.last_reject_roi_count, 0)
        self.assertEqual(detector.last_reject_jump_count, 0)
        self.assertEqual(detector.last_reject_radius_change_count, 0)
        self.assertEqual(detector.last_invalid_streak, 1)
        self.assertEqual(detector.diagnostic_frame_count, 1)
        self.assertEqual(detector.diagnostic_valid_frame_count, 0)
        self.assertEqual(detector.diagnostic_raw_empty_frame_count, 1)
        self.assertEqual(detector.diagnostic_filtered_out_frame_count, 0)
        self.assertEqual(detector.diagnostic_reject_radius_frame_count, 0)
        self.assertEqual(detector.diagnostic_reject_roi_frame_count, 0)
        self.assertEqual(detector.diagnostic_reject_jump_frame_count, 0)
        self.assertEqual(
            detector.diagnostic_reject_radius_change_frame_count, 0
        )

    def test_first_acquisition_prefers_expected_radius(self):
        FAKE_CV_LITE.rgb888_find_circles = (
            lambda *args: [160, 250, 13, 530, 254, 19]
        )
        result = make_detector().detect(FakeImage())
        self.assertEqual(result["center"], (530, 254))
        self.assertEqual(result["radius"], 19)
        self.assertEqual(result["tracking_mode"], "acquire")

    def test_following_frame_prefers_nearby_circle(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [200, 250, 17]
        detector.detect(FakeImage())

        # 远处圆半径更“标准”，但连续跟踪应选择附近的真实钢球。
        FAKE_CV_LITE.rgb888_find_circles = (
            lambda *args: [210, 252, 15, 500, 250, 17]
        )
        result = detector.detect(FakeImage())
        self.assertEqual(result["center"], (210, 252))
        self.assertEqual(result["tracking_mode"], "follow")

    def test_impossible_jump_is_invalid_until_tracking_resets(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [200, 250, 17]
        detector.detect(FakeImage())

        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [500, 250, 17]
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertIsNone(detector.detect(FakeImage()))

        # 连续丢失3帧后允许在远处重新捕获。
        result = detector.detect(FakeImage())
        self.assertEqual(result["center"], (500, 250))
        self.assertEqual(result["tracking_mode"], "acquire")

    def test_large_radius_change_is_rejected(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [200, 250, 17]
        detector.detect(FakeImage())

        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [205, 250, 30]
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertEqual(detector.last_raw_circle_count, 1)
        self.assertEqual(detector.last_reject_radius_change_count, 1)
        self.assertEqual(detector.diagnostic_frame_count, 2)
        self.assertEqual(detector.diagnostic_valid_frame_count, 1)
        self.assertEqual(detector.diagnostic_raw_empty_frame_count, 0)
        self.assertEqual(detector.diagnostic_filtered_out_frame_count, 1)
        self.assertEqual(
            detector.diagnostic_reject_radius_change_frame_count, 1
        )


if __name__ == "__main__":
    unittest.main()
