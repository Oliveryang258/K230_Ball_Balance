"""Allocation-light measurement filtering.

APIs: MicroPython arithmetic only; no hardware API calls.
Hardware: none; consumes K230 vision measurements.
Runtime: compatible with CanMV K230 Yahboom v1.8.0 MicroPython.
"""


class ExponentialFilter:
    """First-order low-pass filter with alpha in (0, 1]."""

    def __init__(self, alpha):
        if alpha <= 0.0 or alpha > 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.value = None

    def update(self, sample):
        if self.value is None:
            self.value = sample
        else:
            self.value += self.alpha * (sample - self.value)
        return self.value

    def reset(self):
        self.value = None
