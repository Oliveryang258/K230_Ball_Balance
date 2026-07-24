"""像素误差与安全区判断，不依赖摄像头。

API：仅使用 MicroPython 算术运算；无 CanMV 硬件调用。
运行时：兼容 CanMV K230 Yahboom v1.8.0 MicroPython。
"""


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
