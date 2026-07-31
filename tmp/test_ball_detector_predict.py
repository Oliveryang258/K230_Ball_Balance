# -*- coding: utf-8 -*-
"""PC 侧 mock 验证 BallDetector 速度外推预测 + 重新捕获确认。

不依赖 cv_lite 与 K230 硬件：用假 time 时钟推进帧间隔，
用假 cv_lite 返回测试者指定的圆候选列表。
运行：PYTHONPATH=src python tmp/test_ball_detector_predict.py
"""
import sys
import types

# ── 假 time：可推进的毫秒时钟 ────────────────────────────────
import time

_clk = {"now": 0}


def _ticks_ms():
    return _clk["now"]


def _ticks_diff(later, earlier):
    return later - earlier


if not hasattr(time, "ticks_ms"):
    time.ticks_ms = _ticks_ms
    time.ticks_diff = _ticks_diff

# ── 假 cv_lite ────────────────────────────────────────────────
_circles = []


def _fake_find_circles(shape, arr, dp, md, p1, p2, minr, maxr):
    return list(_circles)


fake_cv = types.ModuleType("cv_lite")
fake_cv.grayscale_find_circles = _fake_find_circles
sys.modules["cv_lite"] = fake_cv

from vision.ball_detector import BallDetector


class FakeImage:
    def to_numpy_ref(self):
        return bytearray(640 * 96)


def make_detector():
    return BallDetector(
        image_width=640,
        image_height=96,
        roi=(10, 0, 620, 96),
        dp=1,
        min_dist=30,
        param1=80,
        param2=20,
        min_radius=8,
        max_radius=35,
        expected_radius=17,
        max_jump_px=80,
        max_radius_change=8,
        lost_reset_frames=3,
        predict_gate_px=20,
        velocity_alpha=0.40,
        v_max_px_s=600,
        acquire_confirm_frames=3,
        acquire_confirm_tolerance_px=12,
        use_grayscale=True,
    )


def tick(dt_ms=11):
    _clk["now"] += dt_ms


def run_frames(det, positions, dt_ms=11):
    """positions: 每帧的圆候选 [x,y,r,...]，空列表=检测失败。"""
    out = []
    for pos in positions:
        tick(dt_ms)
        global _circles
        _circles = list(pos)
        out.append(det.detect(FakeImage()))
    return out


def acquire(det, x):
    """连续3帧在x处检测，返回最后一帧结果。"""
    results = run_frames(det, [[x, 40, 17]] * 3)
    assert results[0] is None, "第1帧应为确认中"
    assert results[1] is None, "第2帧应为确认中"
    assert results[2] is not None, "第3帧应接受"
    assert results[2]["extrapolated"] is False
    assert abs(results[2]["est_x"] - x) <= 1
    return results[2]


def main():
    global _circles
    det = make_detector()
    img = FakeImage()

    # 1. 重新捕获确认：单帧候选不被接受，需连续3帧一致
    tick()
    _circles = [100, 40, 17]
    r = det.detect(img)
    assert r is None, "首帧候选应处于确认中（None）"
    print("1 首帧候选处于确认中 OK")

    # 2. 单帧候选后无球：确认清零，不误接受
    tick()
    _circles = [100, 40, 17]   # 第2帧同位置
    r = det.detect(img)
    assert r is None
    tick()
    _circles = []              # 打断确认
    r = det.detect(img)
    assert r is None
    tick()
    _circles = [100, 40, 17]   # 重新开始确认，需再3帧
    r = det.detect(img)
    assert r is None, "确认被打断后应重新累计"
    print("2 确认被打断后重新累计 OK")

    # 3. 连续3帧一致 → 接受
    tick()
    _circles = [100, 40, 17]
    r = det.detect(img)
    assert r is None
    tick()
    _circles = [100, 40, 17]
    r = det.detect(img)
    assert r is not None and r["extrapolated"] is False
    assert r["est_x"] == 100
    print("3 连续3帧确认后接受 OK est_x={}".format(r["est_x"]))

    # 4. 每帧漂移超过容差的候选（反光点在远处跳来跳去）无法凑满3帧一致
    det2 = make_detector()
    results = run_frames(det2, [
        [500, 40, 17],
        [525, 40, 17],   # 每帧漂移>容差12 → 每次都重置确认
        [550, 40, 17],
        [575, 40, 17],
    ])
    assert all(r is None for r in results), "持续漂移候选不应被确认接受"
    print("4 持续漂移候选无法凑满3帧一致 OK")

    # 5. 静止跟踪后单帧失败 → 预测保持
    r = acquire(det2, 100)
    prev = r["est_x"]
    tick()
    _circles = []
    r = det2.detect(img)
    assert r is not None and r["extrapolated"] is True
    assert abs(r["est_x"] - prev) <= 1
    print("5 单帧失败预测保持 OK est_x={}".format(r["est_x"]))

    # 6. 恢复检测
    tick()
    _circles = [100, 40, 17]
    r = det2.detect(img)
    assert r is not None and r["extrapolated"] is False
    print("6 恢复可信检测 OK")

    # 7. 错检跳变（候选150 > gate20）→ 拒绝，输出预测
    tick()
    _circles = [150, 40, 17]
    r = det2.detect(img)
    assert r is not None and r["extrapolated"] is True
    assert abs(r["est_x"] - 100) <= 3
    print("7 错检跳变拒绝 OK est_x={}（候选150被拒）".format(r["est_x"]))

    # 8. 连续失败超过阈值 → 真正失球，随后需重新3帧确认
    r = None
    for _ in range(4):
        tick()
        _circles = []
        r = det2.detect(img)
    assert r is None, "连续失败超阈值应返回None"
    tick()
    _circles = [300, 40, 17]
    assert det2.detect(img) is None, "失球后首帧候选应确认中"
    tick()
    _circles = [300, 40, 17]
    assert det2.detect(img) is None, "失球后第2帧候选应确认中"
    tick()
    _circles = [300, 40, 17]
    r = det2.detect(img)
    assert r is not None and r["est_x"] == 300, "失球后3帧确认应重新接受"
    print("8 失球后需3帧确认重新捕获 OK est_x={}".format(r["est_x"]))

    # 9. 运动跟随：先确认，再球匀速2px/帧
    det3 = make_detector()
    results = run_frames(det3, [
        [100, 40, 17],
        [100, 40, 17],
        [100, 40, 17],   # 3帧确认建立
        [102, 40, 17],
        [104, 40, 17],
        [106, 40, 17],
        [108, 40, 17],
        [110, 40, 17],
    ])
    assert results[0] is None and results[1] is None
    assert results[2] is not None and results[2]["est_x"] == 100
    for idx in range(3, len(results)):
        true_x = 100 + 2 * (idx - 2)
        assert abs(results[idx]["est_x"] - true_x) <= 2, (
            "跟随误差过大 idx={} est={} true={}".format(
                idx, results[idx]["est_x"], true_x)
        )
    assert det3.v_px_per_s > 0, "运动时速度估计应为正"
    print("9 运动跟随 OK v_px_per_s={:.1f} est_x={}".format(
        det3.v_px_per_s, results[-1]["est_x"]))

    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
