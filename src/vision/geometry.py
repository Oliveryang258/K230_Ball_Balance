"""Track-projection geometry without camera dependencies.

APIs: MicroPython arithmetic only; no CanMV hardware API calls.
Hardware: none for unit tests; coordinates originate from the K230 camera.
Runtime: compatible with CanMV K230 Yahboom v1.8.0 MicroPython.
"""

import math


def _normalize_axis_angle(angle_deg):
    """把无方向的长轴角度归一化到 [-90, 90) 度。"""
    while angle_deg >= 90.0:
        angle_deg -= 180.0
    while angle_deg < -90.0:
        angle_deg += 180.0
    return angle_deg


def estimate_rectangle_axis(corners):
    """根据矩形四角点估计中心、长轴方向、长度和宽度。

    `corners` 可以是任意顺序的四个 `(x, y)` 点。算法只处理 4 个点：

    1. 计算六条两点连线的长度；
    2. 最长的两条通常是对角线；
    3. 接下来的两条是矩形的两条长边；
    4. 选择一条长边确定方向，并用两条长边的平均值估计长度。

    该函数只使用 MicroPython 的 `math` 和基础容器，方便在电脑 A 上做
    纯逻辑测试。它不调用 cv_lite，也不触碰摄像头。
    """
    if len(corners) != 4:
        raise ValueError("exactly four corners are required")

    points = []
    for point in corners:
        points.append((float(point[0]), float(point[1])))

    center_x_float = sum(point[0] for point in points) / 4.0
    center_y_float = sum(point[1] for point in points) / 4.0

    # 保存 [距离平方, 点下标1, 点下标2]。只有 6 项，排序开销很小。
    pairs = []
    for first in range(4):
        for second in range(first + 1, 4):
            dx = points[second][0] - points[first][0]
            dy = points[second][1] - points[first][1]
            pairs.append([dx * dx + dy * dy, first, second])
    pairs.sort()
    pairs.reverse()

    # 前两项是对角线；第 3、4 项对应两条长边。
    long_edge_a = pairs[2]
    long_edge_b = pairs[3]
    short_edge_a = pairs[4]
    short_edge_b = pairs[5]

    point_a = points[long_edge_a[1]]
    point_b = points[long_edge_a[2]]
    direction_x = point_b[0] - point_a[0]
    direction_y = point_b[1] - point_a[1]

    raw_angle = math.atan2(direction_y, direction_x) * 180.0 / math.pi
    angle_deg = _normalize_axis_angle(raw_angle)

    length = (math.sqrt(long_edge_a[0]) + math.sqrt(long_edge_b[0])) * 0.5
    width = (math.sqrt(short_edge_a[0]) + math.sqrt(short_edge_b[0])) * 0.5
    aspect_ratio = length / max(width, 1.0)

    angle_rad = angle_deg * math.pi / 180.0
    half_length = length * 0.5
    axis_dx = math.cos(angle_rad) * half_length
    axis_dy = math.sin(angle_rad) * half_length

    center = (int(round(center_x_float)), int(round(center_y_float)))
    axis_start = (
        int(round(center_x_float - axis_dx)),
        int(round(center_y_float - axis_dy)),
    )
    axis_end = (
        int(round(center_x_float + axis_dx)),
        int(round(center_y_float + axis_dy)),
    )

    # 按绕中心的极角排序，便于 LCD 依次连接成闭合轮廓。
    point_angles = []
    for point in points:
        point_angles.append((math.atan2(point[1] - center_y_float, point[0] - center_x_float), point))
    point_angles.sort()
    ordered_corners = []
    for _, point in point_angles:
        ordered_corners.append((int(round(point[0])), int(round(point[1]))))

    return {
        "center": center,
        "axis_start": axis_start,
        "axis_end": axis_end,
        "angle": angle_deg,
        "length": length,
        "width": width,
        "aspect_ratio": aspect_ratio,
        "corners": ordered_corners,
    }


def project_ratio(point, fixed_end, servo_end):
    """Project point onto the directed track and return ratio t.

    t=0 is fixed_end and t=1 is servo_end. Values outside [0, 1] are kept.
    """
    px, py = point
    ax, ay = fixed_end
    bx, by = servo_end
    vx = bx - ax
    vy = by - ay
    length_squared = vx * vx + vy * vy
    if length_squared == 0:
        raise ValueError("track endpoints must be different")
    return ((px - ax) * vx + (py - ay) * vy) / length_squared


def normalized_track_error(point, fixed_end, servo_end, clamp=True):
    """Map the point projection to -1 at A, 0 at center, and 1 at B."""
    error = 2.0 * project_ratio(point, fixed_end, servo_end) - 1.0
    if clamp:
        return max(-1.0, min(1.0, error))
    return error
