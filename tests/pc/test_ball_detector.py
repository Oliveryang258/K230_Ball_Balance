"""PC端圆候选、ROI裁剪和坐标恢复测试，使用假的cv_lite模块。"""

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
    def __init__(self):
        self.copied_roi = None

    def copy(self, roi):
        self.copied_roi = tuple(roi)
        return self

    def to_numpy_ref(self):
        return "fake-roi-image-reference"


def make_detector():
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
    )


class BallDetectorTests(unittest.TestCase):
    def setUp(self):
        # ROI左上角为(20,205)，cv_lite现在返回ROI局部坐标。
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [338, 49, 17]

    def test_circle_in_roi_is_detected_and_restored_to_full_coordinates(self):
        image = FakeImage()
        result = make_detector().detect(image)

        self.assertEqual(image.copied_roi, (20, 205, 610, 90))
        self.assertTrue(result["ball_valid"])
        self.assertEqual(result["center"], (358, 254))
        self.assertEqual(result["radius"], 17)

    def test_cv_lite_receives_roi_shape_and_integer_parameters(self):
        captured = []

        def fake_find_circles(*args):
            captured.extend(args)
            return []

        FAKE_CV_LITE.rgb888_find_circles = fake_find_circles
        make_detector().detect(FakeImage())

        self.assertEqual(captured[0], [90, 610])
        self.assertEqual(captured[1], "fake-roi-image-reference")
        for value in captured[2:]:
            self.assertIs(type(value), int)

    def test_invalid_local_coordinate_is_rejected(self):
        # 正常cv_lite不会返回负坐标；这里验证异常结果不会逃出ROI。
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [-1, 49, 20]
        self.assertIsNone(make_detector().detect(FakeImage()))

    def test_empty_result_is_invalid(self):
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: []
        self.assertIsNone(make_detector().detect(FakeImage()))

    def test_first_acquisition_prefers_expected_radius(self):
        FAKE_CV_LITE.rgb888_find_circles = (
            lambda *args: [140, 45, 13, 510, 49, 19]
        )
        result = make_detector().detect(FakeImage())
        self.assertEqual(result["center"], (530, 254))
        self.assertEqual(result["radius"], 19)
        self.assertEqual(result["tracking_mode"], "acquire")

    def test_following_frame_prefers_nearby_circle(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [180, 45, 17]
        detector.detect(FakeImage())

        # 远处圆半径更“标准”，但连续跟踪应选择附近的真实钢球。
        FAKE_CV_LITE.rgb888_find_circles = (
            lambda *args: [190, 47, 15, 480, 45, 17]
        )
        result = detector.detect(FakeImage())
        self.assertEqual(result["center"], (210, 252))
        self.assertEqual(result["tracking_mode"], "follow")

    def test_impossible_jump_is_invalid_until_tracking_resets(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [180, 45, 17]
        detector.detect(FakeImage())

        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [480, 45, 17]
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertIsNone(detector.detect(FakeImage()))
        self.assertIsNone(detector.detect(FakeImage()))

        # 连续丢失3帧后允许在远处重新捕获。
        result = detector.detect(FakeImage())
        self.assertEqual(result["center"], (500, 250))
        self.assertEqual(result["tracking_mode"], "acquire")

    def test_large_radius_change_is_rejected(self):
        detector = make_detector()
        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [180, 45, 17]
        detector.detect(FakeImage())

        FAKE_CV_LITE.rgb888_find_circles = lambda *args: [185, 45, 30]
        self.assertIsNone(detector.detect(FakeImage()))


if __name__ == "__main__":
    unittest.main()
