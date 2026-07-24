"""轻量分配测量滤波。

API：仅使用 MicroPython 算术运算，无硬件调用。
硬件：无；消耗 K230 视觉测量结果。
运行时：兼容 CanMV K230 Yahboom v1.8.0 MicroPython。
"""


class ExponentialFilter:
    """一阶指数低通滤波器，alpha取值范围为(0, 1]。

    计算公式：
        新滤波值 = 旧滤波值 + alpha * (本次测量 - 旧滤波值)

    alpha越接近1，输出越接近当前测量，响应快但抖动较明显；
    alpha越接近0，输出更平滑，但跟随快速运动时会产生更大延迟。
    该类只保存一个历史数值，不保存帧列表，适合K230的内存限制。
    """

    def __init__(self, alpha):
        if alpha <= 0.0 or alpha > 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.value = None

    def update(self, sample):
        """输入一次新测量并返回滤波结果；第一笔测量直接作为初值。"""
        if self.value is None:
            self.value = sample
        else:
            self.value += self.alpha * (sample - self.value)
        return self.value

    def reset(self):
        """清除历史值；丢球后调用，防止重新出现时混入旧位置。"""
        self.value = None
