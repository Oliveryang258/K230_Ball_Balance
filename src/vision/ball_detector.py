# -*- coding: utf-8 -*-
"""钢球检测接口占位。

当前黄色轨道静态测试阶段明确不识别钢球，因此本模块不导入 cv_lite、
不执行阈值分割，也不会返回旧坐标。后续只有在黄色轨道检测通过实机验证后，
才为该接口增加钢球算法。

硬件：未来使用 Yahboom K230 12Pin 摄像头。
运行时：CanMV K230 Yahboom v1.8.0 MicroPython 接口占位。
"""


class BallDetector:
    """保留模块边界；当前阶段始终表示“未检测钢球”。"""

    def detect(self, image):
        """返回 None，防止未实现算法被误认为有效测量。"""
        return None
