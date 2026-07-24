"""轨道投影几何工具，不依赖摄像头。

API：仅使用 MicroPython 算术运算；无 CanMV 硬件调用。
硬件：无，单元测试在 PC 运行；坐标来源于 K230 摄像头。
运行时：兼容 CanMV K230 Yahboom v1.8.0 MicroPython。
"""


def project_ratio(point, fixed_end, servo_end):
    """将点投影到有向轴 A->B 上，返回比例 t。

    fixed_end 是标记 A，servo_end 是标记 B。
    t=0 代表点落在 A，t=1 代表点落在 B。
    [0, 1] 之外的数值会原样保留，便于调用方排查异常检测。
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
    """将投影映射为 A 处 -1、轨道中心 0、B 处 +1 的归一化误差。"""
    error = 2.0 * project_ratio(point, fixed_end, servo_end) - 1.0
    if clamp:
        return max(-1.0, min(1.0, error))
    return error


def pixel_position_error(ball_x, target_x):
    """返回固定相机下的有符号水平像素误差。

    画面右侧是物理固定端，因此 target_x - ball_x：
      负值 → 偏向固定端；正值 → 偏向舵机驱动端。
    """
    return int(target_x) - int(ball_x)


def position_is_safe(ball_x, safe_left_x, safe_right_x):
    """检测圆心是否位于安全区间内。"""
    ball_x = int(ball_x)
    return int(safe_left_x) <= ball_x <= int(safe_right_x)
