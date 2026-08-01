# -*- coding: utf-8 -*-
"""分析 acc_track_mg 偏置 + AF 在直线匀速段是否拖后腿。

输入：C:\\Users\\32142\\Desktop\\ball_run.csv（V3 遥测解码结果）

关键区分：
- 匀速段（|dSpeed|~0）：物理加速度≈0，此时 acc_track_mg 读数 = 偏置+噪声。
  若长期非零，说明 AF 有固定补偿在抵消反馈。
- 加速/减速段：物理加速度非零，AF 是合理前馈，不视为偏置。
"""
import csv
import sys
from collections import Counter

CSV = r"C:\Users\32142\Desktop\ball_run.csv"


def load():
    with open(CSV, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in (
            "stm32_tick_ms", "ball_x_px", "error_px", "velocity_px_s",
            "p_term_us", "i_term_us", "d_term_us", "control_offset_us",
            "servo_target_us", "acc_track_mg", "acc_filtered_mg",
            "af_raw_us", "af_clamped_us", "af_slewed_us", "yf_slewed_us",
            "feedforward_total_us", "yaw_rate_dps", "yaw_raw_dps",
            "speed_average", "turn_command",
        ):
            r[k] = float(r[k])
        r["motion_state"] = int(r["motion_state"])
    return rows


def mean(v):
    return sum(v) / len(v) if v else 0.0


def print_rows(title, rs):
    print(title)
    for r in rs:
        print("  t={:>6}ms state={} ball={:>4} err={:>5} | P={:>6} I={:>5} D={:>5} | "
              "acc_track={:>5} acc_filt={:>5} af_slewed={:>5} yf={:>5} | "
              "speed={:>5} turn_cmd={:>5}".format(
            int(r["stm32_tick_ms"]), r["motion_state"], int(r["ball_x_px"]),
            int(r["error_px"]), int(r["p_term_us"]), int(r["i_term_us"]),
            int(r["d_term_us"]), int(r["acc_track_mg"]), int(r["acc_filtered_mg"]),
            int(r["af_slewed_us"]), int(r["yf_slewed_us"]),
            int(r["speed_average"]), int(r["turn_command"])))


def main():
    rows = load()
    valid = [r for r in rows if r["ball_x_px"] > 0 and r["motion_state"] in (1, 2, 3, 4)]
    print("有效控制帧(state 1-4, 球有效):", len(valid))

    # ---- MAE / max error（与用户报告的 9.57/24 对比）-----------------
    errs = [abs(r["error_px"]) for r in valid]
    print("MAE={:.2f}  max|error|={:.0f}".format(mean(errs), max(errs)))

    # ---- 匀速段检测：|dSpeed| 小的帧 = 物理加速度≈0 ---------------
    # 用连续两帧 speed_average 之差近似加速度
    for i, r in enumerate(valid):
        if i == 0:
            r["dspeed"] = 0.0
        else:
            r["dspeed"] = r["speed_average"] - valid[i - 1]["speed_average"]
    const = [r for r in valid if abs(r["dspeed"]) <= 8]   # 匀速段
    accel = [r for r in valid if r["dspeed"] > 8]          # 加速段
    decel = [r for r in valid if r["dspeed"] < -8]         # 减速段
    print("\n按 |dspeed| 分段: 匀速({}) 加速({}) 减速({})".format(
        len(const), len(accel), len(decel)))

    def bias_report(name, rs):
        a = [r["acc_track_mg"] for r in rs]
        f = [r["acc_filtered_mg"] for r in rs]
        af = [r["af_slewed_us"] for r in rs]
        e = [r["error_px"] for r in rs]
        print("  [{}] n={}  acc_track mean={:+.1f} med={:.0f} std={:.1f} | "
              "af_slewed mean={:+.1f} | err mean={:+.1f}".format(
            name, len(rs), mean(a), sorted(a)[len(a) // 2],
            (sum((v - mean(a)) ** 2 for v in a) / len(a)) ** 0.5,
            mean(af), mean(e)))

    print("acc 偏置检查（匀速段物理 acc≈0，读数应≈0）：")
    bias_report("匀速(state1-4)", const)
    bias_report("仅匀速直线(state2)", [r for r in const if r["motion_state"] == 2])
    bias_report("加速段", accel)
    bias_report("减速段", decel)

    kaf = 2.2
    print("Kaf={:.1f} x 匀速直线段 acc_track mean => {:.1f} us 持续补偿".format(
        kaf, mean([r["acc_track_mg"] for r in const if r["motion_state"] == 2]) * kaf))

    # ---- AF 与 error 方向一致性（有反馈作用的帧） --------------------
    act = [r for r in valid if abs(r["af_slewed_us"]) >= 3 and abs(r["error_px"]) >= 3]
    same = sum(1 for r in act if (r["af_slewed_us"] > 0) == (r["error_px"] > 0))
    oppose = len(act) - same
    print("\nAF 有效帧(|af|>=3 且 |err|>=3): {}  同号(AF助) {}  异号(AF抵) {}".format(
        len(act), same, oppose))

    # ---- 复现用户报告的最差段：t≈27.1s -------------------------------
    print("\n-- t=26800~27600ms 逐帧（用户报告的 27.1s 最差段）--")
    seg = [r for r in rows if 26800 <= r["stm32_tick_ms"] <= 27600]
    print_rows("", seg)

    # ---- 找出所有 |error|>15 且 motion_state 非0 的段 -----------------
    print("\n[|error|>15 且 state 1-4] 最长段:")
    segs = []
    cur = []
    for r in valid:
        if abs(r["error_px"]) > 15:
            cur.append(r)
        else:
            if cur:
                segs.append(cur)
                cur = []
    if cur:
        segs.append(cur)
    segs.sort(key=len, reverse=True)
    for s in segs[:5]:
        span_ms = s[-1]["stm32_tick_ms"] - s[0]["stm32_tick_ms"]
        st = Counter(r["motion_state"] for r in s)
        accel_f = sum(1 for r in s if r["dspeed"] > 8)
        print("  t={}~{} ({}ms) n={} err[{}..{}] af[{}..{}] "
              "acc_track[{:.0f}..{:.0f}] speed[{:.0f}..{:.0f}] state={} 加速帧={}".format(
            int(s[0]["stm32_tick_ms"]), int(s[-1]["stm32_tick_ms"]), span_ms, len(s),
            int(min(r["error_px"] for r in s)), int(max(r["error_px"] for r in s)),
            int(min(r["af_slewed_us"] for r in s)), int(max(r["af_slewed_us"] for r in s)),
            min(r["acc_track_mg"] for r in s), max(r["acc_track_mg"] for r in s),
            min(r["speed_average"] for r in s), max(r["speed_average"] for r in s),
            dict(st), accel_f))
        # 段首 8 帧明细
        print_rows("    段首8帧:", s[:8])


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
