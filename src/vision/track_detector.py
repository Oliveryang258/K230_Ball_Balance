"""Yellow-track detector for CanMV K230 v1.8.0.

Device APIs:
- cv_lite.rgb888_find_blobs()
- CanMV Image.to_numpy_ref()

Hardware: Yahboom K230 12Pin, onboard camera, yellow track.
Runtime: CanMV K230 Yahboom v1.8.0 MicroPython.

This stage deliberately does not call the four-corner rectangle detector.  The red
axis shown on the LCD is only the long centre-line of the axis-aligned Blob bbox;
it is useful for threshold tuning, but is not a perspective-correct track axis.
"""

import cv_lite


def _center_in_roi(center_x, center_y, roi):
    """Return True when the point is inside the software ROI."""
    roi_x, roi_y, roi_width, roi_height = roi
    return (
        center_x >= roi_x
        and center_y >= roi_y
        and center_x < roi_x + roi_width
        and center_y < roi_y + roi_height
    )


class TrackDetector:
    """Select the largest elongated yellow connected region."""

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
        # cv_lite documents image_shape as [height, width].
        self.image_shape = [int(image_height), int(image_width)]
        self.rgb_threshold = list(rgb_threshold)
        self.min_area = int(min_area)
        self.kernel_size = int(kernel_size)
        self.roi = tuple(roi)
        self.min_bbox_aspect_ratio = float(min_bbox_aspect_ratio)

    def capability_report(self):
        """Report the only cv_lite capability required by this stage."""
        return {"rgb888_find_blobs": hasattr(cv_lite, "rgb888_find_blobs")}

    def _largest_yellow_blob(self, raw_blobs):
        """Select the largest elongated candidate whose centre lies in ROI.

        cv_lite returns a flat sequence of [x, y, width, height] records.  The API
        already applies min_area to connected pixels, but does not return the true
        pixel count, so bbox area is used only to rank surviving candidates.
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
        """Build a rough debug axis from the axis-aligned Blob bounding box."""
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
        """Process one RGB888 CanMV Image; return a result dict or None."""
        if not hasattr(cv_lite, "rgb888_find_blobs"):
            raise RuntimeError("cv_lite.rgb888_find_blobs is missing")

        # to_numpy_ref() passes a reference instead of copying the full frame.
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
