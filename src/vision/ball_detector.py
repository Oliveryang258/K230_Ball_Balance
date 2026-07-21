# -*- coding: utf-8 -*-
"""Steel-ball circle detector for Yahboom CanMV K230 v1.8.0.

Device APIs:
- cv_lite.rgb888_find_circles()
- CanMV Image.to_numpy_ref()

Hardware: Yahboom K230 12Pin, onboard camera, reflective steel ball on a
light-coloured rail.
Runtime: CanMV K230 Yahboom v1.8.0 MicroPython.

The API and initial parameters were verified on the physical Yahboom v1.8.0
device on 2026-07-21. Only circles whose centres lie inside the fixed track ROI
are accepted. Candidate continuity is pure MicroPython logic added after the
verified cv_lite call. A failed frame returns None and never reuses stale
coordinates.
"""

import cv_lite


def _center_in_roi(center_x, center_y, roi):
    """判断圆心是否位于固定轨道ROI内。"""
    roi_x, roi_y, roi_width, roi_height = roi
    return (
        center_x >= roi_x
        and center_y >= roi_y
        and center_x < roi_x + roi_width
        and center_y < roi_y + roi_height
    )


def _clamp(value, minimum, maximum):
    """限制外接框坐标，避免靠近图像边缘时产生越界值。"""
    return max(minimum, min(maximum, int(value)))


class BallDetector:
    """使用霍夫圆检测，并利用上一帧位置选择连续的钢球候选。"""

    def __init__(
        self,
        image_width,
        image_height,
        roi,
        dp,
        min_dist,
        param1,
        param2,
        min_radius,
        max_radius,
        expected_radius,
        max_jump_px,
        max_radius_change,
        lost_reset_frames,
    ):
        # cv_lite要求图像形状顺序为[高度, 宽度]。
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.image_shape = [self.image_height, self.image_width]
        self.roi = tuple(roi)
        # Yahboom CanMV v1.8.0的cv_lite绑定在实机上要求这里传整数。
        # 保持与已成功运行的独立例程dp=1完全相同，不能改成1.0。
        self.dp = int(dp)
        self.min_dist = int(min_dist)
        self.param1 = int(param1)
        self.param2 = int(param2)
        self.min_radius = int(min_radius)
        self.max_radius = int(max_radius)

        # 下列参数只参与cv_lite返回后的候选筛选，不会改变底层圆检测调用。
        self.expected_radius = int(expected_radius)
        self.max_jump_px = int(max_jump_px)
        self.max_jump_squared = self.max_jump_px * self.max_jump_px
        self.max_radius_change = int(max_radius_change)
        self.lost_reset_frames = int(lost_reset_frames)

        if self.max_jump_px <= 0:
            raise ValueError("max_jump_px must be positive")
        if self.max_radius_change < 0:
            raise ValueError("max_radius_change must not be negative")
        if self.lost_reset_frames <= 0:
            raise ValueError("lost_reset_frames must be positive")

        # previous_center/previous_radius只是“候选选择参考”，不会作为新的测量输出。
        # 当前帧检测失败时detect()仍然返回None，绝不把旧坐标冒充新坐标。
        self.previous_center = None
        self.previous_radius = None
        self.missed_frames = 0

    def capability_report(self):
        """报告当前算法唯一依赖的cv_lite圆检测函数。"""
        return {
            "rgb888_find_circles": hasattr(cv_lite, "rgb888_find_circles")
        }

    def reset_tracking(self):
        """清除历史位置；下一次有效检测将在整个ROI内重新捕获。"""
        self.previous_center = None
        self.previous_radius = None
        self.missed_frames = 0

    def _record_miss(self):
        """记录一次失败；连续失败达到阈值后忘记旧钢球位置。"""
        self.missed_frames += 1
        if self.missed_frames >= self.lost_reset_frames:
            self.reset_tracking()

    def _select_candidate(self, raw_circles):
        """从[x,y,r,...]中选择与钢球历史最连续的合格圆。"""
        best = None
        best_score = None
        tracking_active = self.previous_center is not None
        roi_center_y = self.roi[1] + self.roi[3] // 2

        for index in range(0, len(raw_circles) - 2, 3):
            center_x = int(raw_circles[index])
            center_y = int(raw_circles[index + 1])
            radius = int(raw_circles[index + 2])

            # cv_lite已经按min/max radius检测，这里再次防御性检查。
            if radius < self.min_radius or radius > self.max_radius:
                continue
            if not _center_in_roi(center_x, center_y, self.roi):
                continue
            if tracking_active:
                # 跟踪阶段：位置连续性优先。远处突然出现的大圆通常是反光或背景，
                # 不应该因为“半径更大”就抢走当前钢球身份。
                delta_x = center_x - self.previous_center[0]
                delta_y = center_y - self.previous_center[1]
                distance_squared = delta_x * delta_x + delta_y * delta_y
                radius_change = abs(radius - self.previous_radius)

                if distance_squared > self.max_jump_squared:
                    continue
                if radius_change > self.max_radius_change:
                    continue

                # 距离平方作为主评分；半径变化只是较小的附加惩罚。
                score = distance_squared + 4 * radius_change * radius_change
                tracking_mode = "follow"
            else:
                # 首次捕获阶段没有上一帧位置可参考。优先选取半径最接近实测
                # 典型值的圆；再用其到ROI纵向中心的距离打破并列。
                score = (
                    100 * abs(radius - self.expected_radius)
                    + abs(center_y - roi_center_y)
                )
                tracking_mode = "acquire"

            if best_score is not None and score >= best_score:
                continue

            x0 = _clamp(center_x - radius, 0, self.image_width - 1)
            y0 = _clamp(center_y - radius, 0, self.image_height - 1)
            x1 = _clamp(center_x + radius, 0, self.image_width - 1)
            y1 = _clamp(center_y + radius, 0, self.image_height - 1)

            best_score = score
            best = {
                "ball_valid": True,
                "center": (center_x, center_y),
                "radius": radius,
                "bbox": (x0, y0, x1 - x0 + 1, y1 - y0 + 1),
                "raw_circle_count": len(raw_circles) // 3,
                "detector": "hough_circle",
                "tracking_mode": tracking_mode,
            }

        return best

    def detect(self, image):
        """处理一帧RGB888 CanMV图像；成功返回字典，失败返回None。"""
        if not hasattr(cv_lite, "rgb888_find_circles"):
            raise RuntimeError("cv_lite.rgb888_find_circles is missing")

        image_array = image.to_numpy_ref()
        raw_circles = cv_lite.rgb888_find_circles(
            self.image_shape,
            image_array,
            self.dp,
            self.min_dist,
            self.param1,
            self.param2,
            self.min_radius,
            self.max_radius,
        )
        result = self._select_candidate(raw_circles)

        if result is None:
            # 只保留有限帧的“身份参考”，但本帧仍立即返回None。
            self._record_miss()
            return None

        # 只有本帧真正检测成功，才能更新历史参考。
        self.previous_center = result["center"]
        self.previous_radius = result["radius"]
        self.missed_frames = 0
        return result
