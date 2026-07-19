# -*- coding: utf-8 -*-
"""黄色轨道检测器。

使用的 CanMV API：
1. `cv_lite.rgb888_find_blobs()`：黄色阈值分割和连通区域检测；
2. `cv_lite.rgb888_find_rectangles_with_corners()`：若固件提供，则估计旋转轮廓；
3. `image.to_numpy_ref()`：把 RGB888 图像以内存引用形式交给 cv_lite。

硬件：Yahboom K230 12Pin 模块、板载摄像头、黄色轨道。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython。

注意：`rgb888_find_blobs()` 的公开返回值只有 [x, y, w, h]，没有像素级
轮廓或主轴角度。因此，本模块把“最大黄色 Blob”和“带角点矩形”组合使用；
若 v1.8.0 固件没有后者，就退化成外接框长轴近似，并明确返回
`angle_source="bbox_approx"`，不会伪装成精确角度。
"""

import math

import cv_lite

from vision.geometry import estimate_rectangle_axis


ORIENTED_RECT_API = "rgb888_find_rectangles_with_corners"


def _bbox_intersection_ratio(first, second):
    """计算两个外接框的交叠程度，结果范围为 0~1。"""
    ax, ay, aw, ah = first
    bx, by, bw, bh = second

    left = max(ax, bx)
    top = max(ay, by)
    right = min(ax + aw, bx + bw)
    bottom = min(ay + ah, by + bh)

    overlap_width = max(0, right - left)
    overlap_height = max(0, bottom - top)
    overlap_area = overlap_width * overlap_height
    if overlap_area <= 0:
        return 0.0

    # 用两个框中较小的面积归一化。这样轨道的“内轮廓矩形”完全落在黄色
    # Blob 中时，也能得到较高分数。
    reference_area = min(aw * ah, bw * bh)
    if reference_area <= 0:
        return 0.0
    return overlap_area / reference_area


def _center_in_roi(center_x, center_y, roi):
    """判断候选中心是否落在软件 ROI 内。"""
    roi_x, roi_y, roi_width, roi_height = roi
    return (
        center_x >= roi_x
        and center_x < roi_x + roi_width
        and center_y >= roi_y
        and center_y < roi_y + roi_height
    )


def _recenter_axis(center, angle_deg, length):
    """让角度和长度描述的中心线穿过黄色 Blob 中心。"""
    center_x, center_y = center
    angle_rad = angle_deg * math.pi / 180.0
    half_length = length * 0.5
    delta_x = math.cos(angle_rad) * half_length
    delta_y = math.sin(angle_rad) * half_length
    return (
        (int(round(center_x - delta_x)), int(round(center_y - delta_y))),
        (int(round(center_x + delta_x)), int(round(center_y + delta_y))),
    )


class TrackDetector:
    """检测最大黄色轨道，并估计中心、主方向、长度和轮廓。"""

    def __init__(
        self,
        image_width,
        image_height,
        rgb_threshold,
        min_area,
        kernel_size,
        roi,
        min_bbox_aspect_ratio,
        use_oriented_rect,
        allow_bbox_fallback,
        rect_canny_low,
        rect_canny_high,
        rect_approx_epsilon,
        rect_min_area_ratio,
        rect_max_angle_cos,
        rect_gaussian_size,
        rect_min_overlap,
        rect_min_aspect_ratio,
    ):
        # cv_lite 文档规定 image_shape 顺序是 [高度, 宽度]，不要写反。
        self.image_shape = [int(image_height), int(image_width)]
        self.rgb_threshold = list(rgb_threshold)
        self.min_area = int(min_area)
        self.kernel_size = int(kernel_size)
        self.roi = tuple(roi)
        self.min_bbox_aspect_ratio = float(min_bbox_aspect_ratio)

        self.use_oriented_rect = bool(use_oriented_rect)
        self.allow_bbox_fallback = bool(allow_bbox_fallback)
        self.rect_canny_low = int(rect_canny_low)
        self.rect_canny_high = int(rect_canny_high)
        self.rect_approx_epsilon = float(rect_approx_epsilon)
        self.rect_min_area_ratio = float(rect_min_area_ratio)
        self.rect_max_angle_cos = float(rect_max_angle_cos)
        self.rect_gaussian_size = int(rect_gaussian_size)
        self.rect_min_overlap = float(rect_min_overlap)
        self.rect_min_aspect_ratio = float(rect_min_aspect_ratio)

        # v1.8.0 为 Yahboom 定制固件。公开资料虽然列出了角点矩形 API，
        # 但必须在实机检查属性是否存在，不能直接假定。
        self.oriented_api_available = hasattr(cv_lite, ORIENTED_RECT_API)

    def capability_report(self):
        """返回启动时需要打印的 cv_lite 能力状态。"""
        return {
            "rgb888_find_blobs": hasattr(cv_lite, "rgb888_find_blobs"),
            ORIENTED_RECT_API: self.oriented_api_available,
        }

    def _largest_yellow_blob(self, raw_blobs):
        """从扁平 Blob 列表中选择 ROI 内、足够细长且外接框最大的候选。"""
        best = None
        best_bbox_area = -1

        # 每个 Blob 占 4 个数：[x, y, w, h]。尾部若不完整则安全忽略。
        for index in range(0, len(raw_blobs) - 3, 4):
            x = int(raw_blobs[index])
            y = int(raw_blobs[index + 1])
            width = int(raw_blobs[index + 2])
            height = int(raw_blobs[index + 3])
            if width <= 0 or height <= 0:
                continue

            center_x = x + width // 2
            center_y = y + height // 2
            if not _center_in_roi(center_x, center_y, self.roi):
                continue

            bbox_aspect = max(width, height) / max(1.0, min(width, height))
            if bbox_aspect < self.min_bbox_aspect_ratio:
                continue

            # cv_lite 已用 TRACK_MIN_AREA 按连通区域面积过滤。由于它没有返回
            # 真实像素数，这里只能用外接框面积选择“最大候选”。
            bbox_area = width * height
            if bbox_area > best_bbox_area:
                best_bbox_area = bbox_area
                best = {
                    "bbox": (x, y, width, height),
                    "center": (center_x, center_y),
                    "bbox_area": bbox_area,
                    "bbox_aspect_ratio": bbox_aspect,
                }
        return best

    def _rectangle_records(self, raw_rectangles):
        """兼容文档中出现的“嵌套列表”和“扁平 12 数值”两种表示。"""
        if not raw_rectangles:
            return []

        first = raw_rectangles[0]
        if isinstance(first, (list, tuple)):
            return raw_rectangles

        records = []
        for index in range(0, len(raw_rectangles) - 11, 12):
            records.append(raw_rectangles[index:index + 12])
        return records

    def _best_oriented_rectangle(self, image_array, yellow_blob):
        """查找与黄色 Blob 最匹配的旋转矩形。"""
        if not self.use_oriented_rect or not self.oriented_api_available:
            return None

        find_rectangles = getattr(cv_lite, ORIENTED_RECT_API)
        raw_rectangles = find_rectangles(
            self.image_shape,
            image_array,
            self.rect_canny_low,
            self.rect_canny_high,
            self.rect_approx_epsilon,
            self.rect_min_area_ratio,
            self.rect_max_angle_cos,
            self.rect_gaussian_size,
        )

        best = None
        best_score = -1.0
        blob_bbox = yellow_blob["bbox"]

        for record in self._rectangle_records(raw_rectangles):
            if len(record) < 12:
                continue

            rect_bbox = (
                int(record[0]),
                int(record[1]),
                int(record[2]),
                int(record[3]),
            )
            corners = []
            for corner_index in range(4, 12, 2):
                corners.append((int(record[corner_index]), int(record[corner_index + 1])))

            try:
                axis = estimate_rectangle_axis(corners)
            except (ValueError, ZeroDivisionError):
                continue

            if axis["aspect_ratio"] < self.rect_min_aspect_ratio:
                continue

            overlap = _bbox_intersection_ratio(blob_bbox, rect_bbox)
            if overlap < self.rect_min_overlap:
                continue

            # 交叠越高、长轴越长，越可能是黄色轨道而非轨道内部的小矩形。
            score = overlap * axis["length"]
            if score > best_score:
                best_score = score
                axis["rect_bbox"] = rect_bbox
                axis["overlap"] = overlap
                best = axis
        return best

    def _bbox_fallback(self, yellow_blob):
        """角点 API 不可用时，用轴对齐外接框长边给出粗略方向。"""
        x, y, width, height = yellow_blob["bbox"]
        center = yellow_blob["center"]
        if width >= height:
            angle = 0.0
            length = float(width)
            track_width = float(height)
        else:
            angle = -90.0
            length = float(height)
            track_width = float(width)

        axis_start, axis_end = _recenter_axis(center, angle, length)
        return {
            "center": center,
            "axis_start": axis_start,
            "axis_end": axis_end,
            "angle": angle,
            "length": length,
            "width": track_width,
            "aspect_ratio": length / max(track_width, 1.0),
            "corners": [
                (x, y),
                (x + width, y),
                (x + width, y + height),
                (x, y + height),
            ],
        }

    def detect(self, image):
        """处理一帧 RGB888 图像；检测失败返回 None。"""
        if not hasattr(cv_lite, "rgb888_find_blobs"):
            raise RuntimeError("cv_lite.rgb888_find_blobs is missing")

        # to_numpy_ref() 不复制整帧图像，只把底层内存引用交给 cv_lite。
        image_array = image.to_numpy_ref()
        raw_blobs = cv_lite.rgb888_find_blobs(
            self.image_shape,
            image_array,
            self.rgb_threshold,
            self.min_area,
            self.kernel_size,
        )

        yellow_blob = self._largest_yellow_blob(raw_blobs)
        if yellow_blob is None:
            return None

        axis = self._best_oriented_rectangle(image_array, yellow_blob)
        if axis is None:
            if not self.allow_bbox_fallback:
                return None
            axis = self._bbox_fallback(yellow_blob)
            angle_source = "bbox_approx"
        else:
            angle_source = "oriented_rect"

            # “中心”按最大黄色 Blob 外接框定义；方向和长度来自旋转矩形。
            # 将中心线重新平移到黄色区域中心，LCD 上更容易观察两者是否匹配。
            axis["center"] = yellow_blob["center"]
            axis["axis_start"], axis["axis_end"] = _recenter_axis(
                axis["center"], axis["angle"], axis["length"]
            )

        return {
            "track_valid": True,
            "center": axis["center"],
            "bbox": yellow_blob["bbox"],
            "contour": axis["corners"],
            "axis_start": axis["axis_start"],
            "axis_end": axis["axis_end"],
            "angle": axis["angle"],
            "length": axis["length"],
            "width": axis["width"],
            "aspect_ratio": axis["aspect_ratio"],
            "bbox_area": yellow_blob["bbox_area"],
            "angle_source": angle_source,
        }
