# -*- coding: utf-8 -*-
"""Yahboom CanMV K230 v1.8.0 钢球圆检测器。

设备 API：
- cv_lite.grayscale_find_circles()
- cv_lite.rgb888_find_circles()
- CanMV Image.to_numpy_ref()

硬件：Yahboom K230 12Pin，板载摄像头，浅色轨道上的反光钢球。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython。

API 及初始参数已于 2026-07-21 在 Yahboom v1.8.0 实机上验证。
只有圆心落在固定轨道 ROI 内的圆才会被接受。候选连续性为纯
MicroPython 逻辑，在已验证的 cv_lite 调用之上实现。失败帧
返回 None，绝不复用旧坐标。
"""

import time

import cv_lite


def _center_in_roi(center_x, center_y, roi):
    """判断圆心是否位于固定轨道 ROI 内。"""
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
        predict_gate_px,
        velocity_alpha,
        v_max_px_s,
        acquire_confirm_frames,
        acquire_confirm_tolerance_px,
        use_grayscale=False,
    ):
        # cv_lite 要求图像形状顺序为 [高度, 宽度]。
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.image_shape = [self.image_height, self.image_width]
        self.roi = tuple(roi)
        self.use_grayscale = bool(use_grayscale)
        # Yahboom CanMV v1.8.0 的 cv_lite 绑定在实机上要求这里传整数。
        # 保持与已成功运行的独立例程 dp=1 完全相同，不能改成 1.0。
        self.dp = int(dp)
        self.min_dist = int(min_dist)
        self.param1 = int(param1)
        self.param2 = int(param2)
        self.min_radius = int(min_radius)
        self.max_radius = int(max_radius)

        # 下列参数只参与 cv_lite 返回后的候选筛选，不会改变底层圆检测调用。
        self.expected_radius = int(expected_radius)
        self.max_jump_px = int(max_jump_px)
        self.max_jump_squared = self.max_jump_px * self.max_jump_px
        self.max_radius_change = int(max_radius_change)
        self.lost_reset_frames = int(lost_reset_frames)

        # 速度外推预测门限。检测圆心偏离预测位置超过该值判为误检。
        self.predict_gate_px = int(predict_gate_px)
        self.velocity_alpha = float(velocity_alpha)
        self.v_max_px_s = float(v_max_px_s)

        # 重新捕获确认：被追丢后，新候选必须连续多帧在容差内一致才接受，
        # 防止单个反光/背景圆直接成为新目标。
        self.acquire_confirm_frames = int(acquire_confirm_frames)
        self.acquire_confirm_tolerance_px = int(acquire_confirm_tolerance_px)

        if not hasattr(time, "ticks_ms"):
            raise RuntimeError("time.ticks_ms() is missing")
        if self.predict_gate_px <= 0:
            raise ValueError("predict_gate_px must be positive")
        if self.velocity_alpha <= 0.0 or self.velocity_alpha > 1.0:
            raise ValueError("velocity_alpha must be in (0, 1]")
        if self.v_max_px_s <= 0.0:
            raise ValueError("v_max_px_s must be positive")
        if self.acquire_confirm_frames <= 0:
            raise ValueError("acquire_confirm_frames must be positive")
        if self.acquire_confirm_tolerance_px <= 0:
            raise ValueError("acquire_confirm_tolerance_px must be positive")

        if self.max_jump_px <= 0:
            raise ValueError("max_jump_px must be positive")
        if self.max_radius_change < 0:
            raise ValueError("max_radius_change must not be negative")
        if self.lost_reset_frames <= 0:
            raise ValueError("lost_reset_frames must be positive")

        # previous_center/previous_radius 只是"候选选择参考"，不会作为新的测量输出。
        # 当前帧检测失败时 detect() 仍然返回 None，绝不把旧坐标冒充新坐标。
        self.previous_center = None
        self.previous_radius = None
        self.missed_frames = 0
        self.last_invalid_streak = 0
        self.last_raw_circle_count = 0
        self.last_reject_radius_range_count = 0
        self.last_reject_roi_count = 0
        self.last_reject_jump_count = 0
        self.last_reject_radius_change_count = 0
        self.last_reject_gate_count = 0

        # 速度外推状态。est_x 是滤波后的估计位置（裁剪帧坐标），
        # v_px_per_s 是估计速度，只用于预测下一帧，不直接进入控制器。
        # pred_x 是本帧的外推预测位置，在 detect() 中计算。
        self.est_x = None
        self.previous_y = None
        self.v_px_per_s = 0.0
        self.last_frame_ms = None
        self.pred_x = None
        self.acquire_confirm_x = None
        self.acquire_confirm_count = 0

        self.diagnostic_frame_count = 0
        self.diagnostic_valid_frame_count = 0
        self.diagnostic_raw_empty_frame_count = 0
        self.diagnostic_filtered_out_frame_count = 0
        self.diagnostic_reject_radius_frame_count = 0
        self.diagnostic_reject_roi_frame_count = 0
        self.diagnostic_reject_jump_frame_count = 0
        self.diagnostic_reject_radius_change_frame_count = 0
        self.diagnostic_reject_gate_frame_count = 0
        self.diagnostic_acquire_pending_frame_count = 0

    def capability_report(self):
        """报告两种像素格式对应的cv_lite圆检测函数。"""
        return {
            "grayscale_find_circles": hasattr(
                cv_lite, "grayscale_find_circles"
            ),
            "rgb888_find_circles": hasattr(cv_lite, "rgb888_find_circles")
        }

    def required_api_name(self):
        """返回当前模式必须具备的cv_lite接口名。"""
        if self.use_grayscale:
            return "grayscale_find_circles"
        return "rgb888_find_circles"

    def reset_tracking(self):
        """清除历史位置；下一次有效检测将在整个 ROI 内重新捕获。"""
        self.previous_center = None
        self.previous_radius = None
        self.previous_y = None
        self.missed_frames = 0
        self.est_x = None
        self.v_px_per_s = 0.0
        self.acquire_confirm_x = None
        self.acquire_confirm_count = 0

    def _record_miss(self):
        """记录一次失败；连续失败达到阈值后忘记旧钢球位置。"""
        self.missed_frames += 1
        if self.missed_frames >= self.lost_reset_frames:
            self.reset_tracking()

    def _select_candidate(self, raw_circles):
        """从 [x, y, r, ...] 中选择与钢球历史最连续的合格圆。"""
        self.last_raw_circle_count = (
            len(raw_circles) // 3 if raw_circles else 0
        )
        self.last_reject_radius_range_count = 0
        self.last_reject_roi_count = 0
        self.last_reject_jump_count = 0
        self.last_reject_radius_change_count = 0

        if not raw_circles:
            return None

        best = None
        best_score = None
        tracking_active = self.previous_center is not None
        roi_center_y = self.roi[1] + self.roi[3] // 2

        for index in range(0, len(raw_circles) - 2, 3):
            # cv_lite在完整640x480图像上检测，因此这里已经是原图坐标。
            # ROI只在候选筛选阶段使用，不再为每一帧复制一张ROI图像。
            center_x = int(raw_circles[index])
            center_y = int(raw_circles[index + 1])
            radius = int(raw_circles[index + 2])

            # cv_lite 已按 min/max radius 检测，这里再次检查。
            if radius < self.min_radius or radius > self.max_radius:
                self.last_reject_radius_range_count += 1
                continue
            if not _center_in_roi(center_x, center_y, self.roi):
                self.last_reject_roi_count += 1
                continue
            if tracking_active:
                # 跟踪阶段：位置连续性优先。远处突然出现的大圆通常是反光
                # 或背景，不应该因为"半径更大"就抢走当前钢球身份。
                delta_x = center_x - self.previous_center[0]
                delta_y = center_y - self.previous_center[1]
                distance_squared = delta_x * delta_x + delta_y * delta_y
                radius_change = abs(radius - self.previous_radius)

                if distance_squared > self.max_jump_squared:
                    self.last_reject_jump_count += 1
                    continue
                if radius_change > self.max_radius_change:
                    self.last_reject_radius_change_count += 1
                    continue

                # 距离平方作为主评分；半径变化只是较小的附加惩罚。
                score = distance_squared + 4 * radius_change * radius_change
                tracking_mode = "follow"
            else:
                # 首次捕获阶段没有上一帧位置可参考。优先选取半径最接近实测
                # 典型值的圆；再用其到 ROI 纵向中心的距离打破并列。
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
        """处理一帧灰度或RGB888图像；成功返回字典，失败返回None。

        在原有半径/ROI/跳变筛选之上增加速度外推预测：
        - 检测圆心相对预测位置偏移超过 predict_gate_px 时判为可疑帧，
          改用预测位置保持，避免单帧误检直接打进控制器；
        - 检测失败但连续失败未超过 lost_reset_frames 时，同样输出预测位置
          （extrapolated=True），而不是立即输出 ball_valid=0。
        """
        api_name = self.required_api_name()
        if not hasattr(cv_lite, api_name):
            raise RuntimeError("cv_lite.{} is missing".format(api_name))
        find_circles = getattr(cv_lite, api_name)

        now_ms = time.ticks_ms()
        dt_ms = 0
        if self.last_frame_ms is not None:
            dt_ms = time.ticks_diff(now_ms, self.last_frame_ms)
        self.last_frame_ms = now_ms

        # 用上一帧估计位置和速度外推本帧预测位置。候选选择以此为参考，
        # 使跳变限制相对预测位置计算，而不是相对上一帧的固定值。
        self.pred_x = None
        if self.est_x is not None:
            if (dt_ms > 0) and (self.v_px_per_s != 0.0):
                self.pred_x = (
                    self.est_x + self.v_px_per_s * (dt_ms / 1000.0)
                )
            else:
                self.pred_x = self.est_x
            ref_y = self.previous_y
            if ref_y is None:
                ref_y = self.roi[1] + self.roi[3] // 2
            self.previous_center = (round(self.pred_x), ref_y)

        # 直接取得Sensor当前帧的零拷贝引用，在整帧上执行霍夫圆。
        # 灰度模式要求Sensor本身输出GRAYSCALE，不能把RGB888引用交给灰度接口。
        # 不调用 image.copy(roi)，避免K230每帧裁剪/分配临时图像造成严重掉帧。
        # cv_lite返回候选后，再由_select_candidate()按BALL_ROI过滤圆心。
        image_array = image.to_numpy_ref()
        raw_circles = find_circles(
            self.image_shape,
            image_array,
            self.dp,
            self.min_dist,
            self.param1,
            self.param2,
            self.min_radius,
            self.max_radius,
        )
        best = self._select_candidate(raw_circles)
        self.diagnostic_frame_count += 1

        if best is not None and self.pred_x is not None:
            residual_px = best["center"][0] - self.pred_x
            if abs(residual_px) <= self.predict_gate_px:
                # 可信：用"预测+残差"更新估计位置和速度。
                self._update_estimate(best, residual_px, dt_ms)
                return self._mark_result(best, False)
            # 检测到圆但偏离预测过多，该候选很可能是反光/背景误检。
            # 用预测位置保持，不把可疑帧直接送进控制器。
            self.last_reject_gate_count += 1
            self.diagnostic_reject_gate_frame_count += 1
            return self._hold_prediction()

        if best is not None:
            # 重新捕获阶段：没有历史速度。不立即接受候选，而是要求连续
            # acquire_confirm_frames 帧在容差内一致才正式建立跟踪，防止
            # 被追丢后单个反光/背景圆直接成为新目标（实拍误检场景）。
            if self.acquire_confirm_x is None:
                self.acquire_confirm_x = best["center"][0]
                self.acquire_confirm_count = 1
            elif (abs(best["center"][0] - self.acquire_confirm_x) <=
                  self.acquire_confirm_tolerance_px):
                self.acquire_confirm_count += 1
                # 缓慢跟随参考点，允许确认期间钢球轻微移动。
                self.acquire_confirm_x = (
                    (self.acquire_confirm_x + best["center"][0]) // 2
                )
            else:
                # 位置跳变超过容差：重新开始确认，累计帧数清零。
                self.acquire_confirm_x = best["center"][0]
                self.acquire_confirm_count = 1

            if self.acquire_confirm_count < self.acquire_confirm_frames:
                # 确认中：不把未确认候选送进控制器。
                self.diagnostic_acquire_pending_frame_count += 1
                return None

            # 连续多帧一致，正式接受并建立跟踪历史。
            self.est_x = best["center"][0]
            self.v_px_per_s = 0.0
            self.previous_center = best["center"]
            self.previous_y = best["center"][1]
            self.previous_radius = best["radius"]
            self.missed_frames = 0
            self.last_invalid_streak = 0
            self.diagnostic_valid_frame_count += 1
            return self._mark_result(best, False)

        # 检测失败。
        if (self.est_x is not None) and (
            self.missed_frames < self.lost_reset_frames
        ):
            self.missed_frames += 1
            if self.last_raw_circle_count == 0:
                self.diagnostic_raw_empty_frame_count += 1
            else:
                self.diagnostic_filtered_out_frame_count += 1
            return self._hold_prediction()

        # 真正失球或尚未捕获：清空重新捕获确认，等待新球连续多帧出现。
        self.acquire_confirm_x = None
        self.acquire_confirm_count = 0
        self._record_miss()
        self.last_invalid_streak += 1
        return None

    def _update_estimate(self, best, residual_px, dt_ms):
        """可信检测：位置做预测+残差的指数滤波，速度做带限幅的更新。"""
        self.est_x = self.pred_x + self.velocity_alpha * residual_px
        if dt_ms > 0:
            instant_v = residual_px * 1000.0 / dt_ms
            self.v_px_per_s = (
                (1.0 - self.velocity_alpha) * self.v_px_per_s
                + self.velocity_alpha * instant_v
            )
            if self.v_px_per_s > self.v_max_px_s:
                self.v_px_per_s = self.v_max_px_s
            elif self.v_px_per_s < -self.v_max_px_s:
                self.v_px_per_s = -self.v_max_px_s
        self.previous_center = (round(self.est_x), best["center"][1])
        self.previous_y = best["center"][1]
        self.previous_radius = best["radius"]
        self.missed_frames = 0
        self.last_invalid_streak = 0
        self.diagnostic_valid_frame_count += 1

    def _hold_prediction(self):
        """检测失败或可疑帧：用预测位置构造测量结果，标记为外推。"""
        center_y = self.previous_y
        if center_y is None:
            center_y = self.roi[1] + self.roi[3] // 2
        radius = self.previous_radius
        if radius is None:
            radius = self.expected_radius
        center_x = round(self.pred_x)
        self.previous_center = (center_x, center_y)

        x0 = _clamp(center_x - radius, 0, self.image_width - 1)
        y0 = _clamp(center_y - radius, 0, self.image_height - 1)
        x1 = _clamp(center_x + radius, 0, self.image_width - 1)
        y1 = _clamp(center_y + radius, 0, self.image_height - 1)
        return {
            "ball_valid": True,
            "center": (center_x, center_y),
            "est_x": center_x,
            "radius": radius,
            "bbox": (x0, y0, x1 - x0 + 1, y1 - y0 + 1),
            "raw_circle_count": self.last_raw_circle_count,
            "detector": "hough_circle",
            "tracking_mode": "predict",
            "extrapolated": True,
        }

    def _mark_result(self, best, extrapolated):
        """给检测结果附加估计位置和外推标记。

        best["center"] 保持原始检测位置（用于安全判断和画框），
        est_x 是滤波估计位置（用于控制误差），extrapolated 标记预测帧。
        """
        result = dict(best)
        result["est_x"] = round(self.est_x)
        result["extrapolated"] = extrapolated
        return result
