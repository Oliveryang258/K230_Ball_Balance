"""CanMV K230 v1.8.0 黄色轨道检测器。

设备 API：
- cv_lite.rgb888_find_blobs()
- CanMV Image.to_numpy_ref()

硬件：Yahboom K230 12Pin，板载摄像头，黄色轨道。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython。

当前阶段不调用四角点矩形检测器。LCD 上显示的红线只是
轴对齐 Blob 外接框的长边中线，仅用于阈值调试，不是透视
校正后的真实轨道轴线。
"""

import cv_lite


def _center_in_roi(center_x, center_y, roi):
    """判断点是否落在软件 ROI 内。"""
    roi_x, roi_y, roi_width, roi_height = roi
    return (
        center_x >= roi_x
        and center_y >= roi_y
        and center_x < roi_x + roi_width
        and center_y < roi_y + roi_height
    )


class TrackDetector:
    """选取画面中最大且细长的黄色连通区域。"""

    def __init__(
        self,
        image_width,
        image_height,
        rgb_threshold,
        min_area,
        kernel_size,
        roi,
        min_bbox_aspect_ratio,
    ):
        # cv_lite 文档规定 image_shape 顺序为 [高度, 宽度]。
        self.image_shape = [int(image_height), int(image_width)]
        self.rgb_threshold = list(rgb_threshold)
        self.min_area = int(min_area)
        self.kernel_size = int(kernel_size)
        self.roi = tuple(roi)
        self.min_bbox_aspect_ratio = float(min_bbox_aspect_ratio)

    def capability_report(self):
        """报告当前阶段唯一依赖的 cv_lite 能力。"""
        return {"rgb888_find_blobs": hasattr(cv_lite, "rgb888_find_blobs")}

    def _largest_yellow_blob(self, raw_blobs):
        """从扁平 Blob 列表中选择 ROI 内、足够细长且外接框最大的候选。

        cv_lite 返回 [x, y, width, height] 扁平序列。API 已对连通像素
        应用 min_area 过滤，但不返回真实像素数，因此仅用外接框面积
        对存活候选排序。
        """
        best = None
        best_bbox_area = -1

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

            short_side = min(width, height)
            long_side = max(width, height)
            bbox_aspect = long_side / max(1.0, short_side)
            if bbox_aspect < self.min_bbox_aspect_ratio:
                continue

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

    def _bbox_axis(self, yellow_blob):
        """从轴对齐 Blob 外接框构造粗略调试轴线。"""
        x, y, width, height = yellow_blob["bbox"]
        center_x, center_y = yellow_blob["center"]

        if width >= height:
            angle = 0.0
            length = float(width)
            track_width = float(height)
            axis_start = (x, center_y)
            axis_end = (x + width, center_y)
        else:
            angle = -90.0
            length = float(height)
            track_width = float(width)
            axis_start = (center_x, y)
            axis_end = (center_x, y + height)

        return {
            "track_valid": True,
            "center": (center_x, center_y),
            "bbox": yellow_blob["bbox"],
            "axis_start": axis_start,
            "axis_end": axis_end,
            "angle": angle,
            "length": length,
            "width": track_width,
            "aspect_ratio": length / max(track_width, 1.0),
            "bbox_area": yellow_blob["bbox_area"],
            "axis_source": "bbox_only",
        }

    def detect(self, image):
        """处理一帧 RGB888 CanMV 图像；成功返回字典，失败返回 None。"""
        if not hasattr(cv_lite, "rgb888_find_blobs"):
            raise RuntimeError("cv_lite.rgb888_find_blobs is missing")

        # to_numpy_ref() 传递引用，不拷贝整帧图像。
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
        return self._bbox_axis(yellow_blob)
