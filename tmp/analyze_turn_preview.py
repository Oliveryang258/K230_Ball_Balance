# -*- coding: utf-8 -*-
"""定位 turn preview 在右转数据中为什么几乎无输出。

按用户给的 4 个检查项逐个核对：
1. TURN_PREVIEW_RIGHT_SIGN 与 turn_command 实际符号
2. START/FULL 阈值与真实 turn_command 幅值
3. yaw handover 是否提前关闭 preview
4. motion_state / link valid 门控
"""
import csv
import sys
from collections import Counter

CSV = r"C:\Users\32142\Desktop\ball_run.csv"

START = 20.0
FULL = 55.0
RIGHT_SIGN = -1.0
PREVIEW_SIGN = -1.0
MAX_US = 15.0
HANDOVER_DPS = 25.0


def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else 0.0


def load():
    with open(CSV, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["tick"] = float(r["stm32_tick_ms"])
        r["motion_state"] = int(r["motion_state"])
        r["turn_command"] = float(r["turn_command"])
        r["turn_preview_raw_us"] = float(r["turn_preview_raw_us"])
        r["turn_preview_slewed_us"] = float(r["turn_preview_slewed_us"])
        r["turn_scale"] = float(r["turn_scale"])
        r["yaw_handover"] = float(r["yaw_handover"])
        r["yaw_raw_dps"] = float(r["yaw_raw_dps"])
        r["speed_average"] = float(r["speed_average"])
        r["motion_bias_active_us"] = float(r["motion_bias_active_us"])
        r["motion_link_valid"] = int(r["motion_link_valid"])
        r["motion_flags"] = int(r["motion_flags"])
    return rows


def fmt_row(r):
    return ("t={:>7}ms st={} lnk={} | tc={:>6} mag={:>6} scale={:>5} "
            "| preview_raw={:>6} slewed={:>6} yaw={:>5} handover={:>4} "
            "| speed={:>5} bias={:>6}".format(
        int(r["tick"]), r["motion_state"], r["motion_link_valid"],
        int(r["turn_command"]),
        int(RIGHT_SIGN * r["turn_command"]),
        r["turn_scale"], int(r["turn_preview_raw_us"]),
        int(r["turn_preview_slewed_us"]), int(r["yaw_raw_dps"]),
        r["yaw_handover"], int(r["speed_average"]),
        round(r["motion_bias_active_us"], 2)))


def main():
    rows = load()
    print("总帧数:", len(rows))
    print("motion_state 分布:", dict(Counter(r["motion_state"] for r in rows)))
    print("motion_link_valid=1 帧数:", sum(1 for r in rows if r["motion_link_valid"]))

    tr = [r for r in rows if r["motion_state"] == 4]
    print("\n== TURN_RIGHT 段: n={} ==".format(len(tr)))
    if not tr:
        print("  !!! 本数据里没有 TURN_RIGHT(state=4)，preview 无输出的原因就是没进入右转状态")
        return

    tc = [r["turn_command"] for r in tr]
    print("turn_command: min={:+.1f} max={:+.1f} mean={:+.1f}".format(
        min(tc), max(tc), mean(tc)))
    mag = [RIGHT_SIGN * v for v in tc]
    print("turn_mag=-1*tc:  min={:.1f} max={:.1f}  (>START {} 的帧数: {}/{})".format(
        min(mag), max(mag), START, sum(1 for v in mag if v > START), len(mag)))

    p_raw = [r["turn_preview_raw_us"] for r in tr]
    p_slewed = [r["turn_preview_slewed_us"] for r in tr]
    print("preview_raw:   mean={:+.3f} min={:+.1f} max={:+.1f} | 非零帧 {}".format(
        mean(p_raw), min(p_raw), max(p_raw),
        sum(1 for v in p_raw if abs(v) > 0.5)))
    print("preview_slewed:mean={:+.3f} min={:+.1f} max={:+.1f} | 非零帧 {}".format(
        mean(p_slewed), min(p_slewed), max(p_slewed),
        sum(1 for v in p_slewed if abs(v) > 0.5)))
    print("turn_scale:    mean={:.3f} max={:.3f}".format(
        mean(r["turn_scale"] for r in tr), max(r["turn_scale"] for r in tr)))
    print("yaw_handover:  mean={:.3f} max={:.3f}".format(
        mean(r["yaw_handover"] for r in tr), max(r["yaw_handover"] for r in tr)))
    print("yaw_raw_dps:   mean={:+.1f} min={:+.1f} max={:+.1f}".format(
        mean(r["yaw_raw_dps"] for r in tr), min(r["yaw_raw_dps"] for r in tr),
        max(r["yaw_raw_dps"] for r in tr)))
    print("speed_average: mean={:.0f} min={:.0f} max={:.0f}".format(
        mean(r["speed_average"] for r in tr), min(r["speed_average"] for r in tr),
        max(r["speed_average"] for r in tr)))
    print("motion_bias_active_us: mean={:+.2f} min={:+.1f} max={:+.1f}".format(
        mean(r["motion_bias_active_us"] for r in tr),
        min(r["motion_bias_active_us"] for r in tr),
        max(r["motion_bias_active_us"] for r in tr)))

    # 门控检查
    flags = Counter(r["motion_flags"] for r in tr)
    print("motion_flags 分布(IMU_VALID=0x01, FAULT=0x80):", dict(flags))
    lnk_bad = [r for r in tr if r["motion_link_valid"] == 0]
    print("link_valid=0 的 TURN_RIGHT 帧:", len(lnk_bad))

    # 关键段：右转首尾各 12 帧明细
    def seg(rs):
        # 找 preview 非零的帧，若没有则取 turn_mag 最大的连续段
        nz = [i for i, r in enumerate(tr) if abs(r["turn_preview_raw_us"]) > 0.5]
        if nz:
            i0, i1 = max(0, nz[0] - 6), min(len(tr), nz[-1] + 6)
        else:
            # 无 preview 时，找 turn_mag 进入/退出 START 的边沿
            entering = next((i for i, r in enumerate(tr)
                             if RIGHT_SIGN * r["turn_command"] > START), None)
            i0 = max(0, (entering or 0) - 8)
            i1 = min(len(tr), (entering or 0) + 14)
        return tr[i0:i1]

    print("\n-- 右转关键段逐帧（首 6 帧 + 进入阈值 ±） --")
    for r in seg(tr):
        print("  " + fmt_row(r))

    # 进入/退出边沿
    print("\n-- turn_mag 跨越 START={} 的边沿帧 --".format(START))
    prev_on = False
    shown = 0
    for r in tr:
        on = RIGHT_SIGN * r["turn_command"] > START
        if on != prev_on:
            print("  " + fmt_row(r))
            shown += 1
            prev_on = on
            if shown >= 8:
                break


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
