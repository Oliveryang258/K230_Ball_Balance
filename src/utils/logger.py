"""极简日志与限频调试帧保存。

API：MicroPython time.ticks_ms/ticks_diff 和 CanMV image.save。
硬件：K230 文件系统或已挂载的 SD 卡。
运行时：CanMV K230 Yahboom v1.8.0；图片保存需实机验证。
"""

import time


def log_info(message):
    print("[INFO] {}".format(message))


def log_error(message):
    print("[ERROR] {}".format(message))


class DebugFrameSaver:
    """按配置的时间间隔限频保存带标注的调试图片。"""

    def __init__(self, path, interval_ms=5000, enabled=True):
        self.path = path
        self.interval_ms = interval_ms
        self.enabled = enabled
        self._last_save_ms = None

    def save_if_due(self, image):
        if not self.enabled:
            return False

        now = time.ticks_ms()
        if self._last_save_ms is not None:
            elapsed = time.ticks_diff(now, self._last_save_ms)
            if elapsed < self.interval_ms:
                return False

        try:
            image.save(self.path, quality=90)
            self._last_save_ms = now
            log_info("Saved {}".format(self.path))
            return True
        except BaseException as exc:
            log_error("Debug image save failed: {}".format(exc))
            return False
