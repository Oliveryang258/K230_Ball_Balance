# -*- coding: utf-8 -*-
"""评估最新一轮(21:43 CSV)的前馈参数表现：AF 和 YF。

关注：
1. 确认 motion_bias / turn_preview 是否真的为0（本轮配置）；
2. 整体误差指标（MAE / P95 / max / ±15px 占比）；
3. AF 在加速/减速/匀速/转弯各段的符号一致性（助 or 抵）；
4. YF 在右转段的覆盖是否够（yaw 建立后误差是否下降）；
5. I 是否在目标点饱和（判断是平衡问题还是前馈问题）。
"""
import csv
import sys
from collections import Counter

CSV = r"C:\Users\32142\Desktop\ball_run.csv"

KP = 3.5
KY = -0.45
KAF = 2.4


def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else 0.0


def pct(v, q):
    v = sorted(v)
    if not v:
        return 0.0
    return v[min(len(v) - 1, int(round(len(v) * q)) )]


def load():
    with open(CSV, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["tick"] = float(r["stm32_tick_ms"])
        r["state"] = int(r["motion_state"])
        r["err"] = float(r["error_px"])
        r["ball"] = float(r["ball_x_px"])
        r["p"] = float(r["p_term_us"])
        r["i"] = float(r["i_term_us"])
        r["d"] = float(r["d_term_us"])
        r["af"] = float(r["af_slewed_us"])
        r["yf"] = float(r["yf_slewed_us"])
        r["yaw"] = float(r["yaw_raw_dps"])
        r["speed"] = float(r["speed_average"])
        r["acc"] = float(r["acc_track_mg"])
        r["accf"] = float(r["acc_filtered_mg"])
        r["bias"] = float(r["motion_bias_active_us"])
        r["pv"] = float(r["turn_preview_slewed_us"])
        r["scale"] = float(r["speed_scale_x1000"])
        r["link"] = int(r["motion_link_valid"])
        r["feed"] = float(r["feedforward_total_us"])
        r["hold"] = float(r["hold_pwm_effective_us"])
    return rows


def seg_stats(rows, title):
    errs = [abs(r["err"]) for r in rows]
    print("  [{}] n={}  MAE={:.1f}  P95={:.0f}  max={:.0f}  ±15px={:.1f}%  ±25px={:.1f}%".format(
        title, len(rows), mean(errs), pct(errs, 0.95), max(errs),
        100 * sum(1 for e in errs if e <= 15) / max(1, len(errs)),
        100 * sum(1 for e in errs if e <= 25) / max(1, len(errs))))


def main():
    rows = load()
    valid = [r for r in rows if r["ball"] > 0 and r["state"] in (1, 2, 3, 4) and r["link"]]
    print("总帧:", len(rows), " 有效控制帧(state1-4&link):", len(valid))
    print("motion_bias_active_us 非零帧:", sum(1 for r in rows if abs(r["bias"]) > 0.5),
          " turn_preview_slewed 非零帧:", sum(1 for r in rows if abs(r["pv"]) > 0.5))

    print("\n== 整体误差 ==")
    seg_stats(valid, "全部")
    seg_stats([r for r in valid if r["state"] == 2], "直线STRAIGHT")
    seg_stats([r for r in valid if r["state"] == 4], "右转TURN_RIGHT")
    seg_stats([r for r in valid if r["state"] in (1, 5)], "起停START/STOP")

    # 按加速度分段（近似 dSpeed）
    for i, r in enumerate(valid):
        r["dspeed"] = r["speed"] - valid[i - 1]["speed"] if i else 0.0
    accel = [r for r in valid if r["dspeed"] > 8]
    decel = [r for r in valid if r["dspeed"] < -8]
    const = [r for r in valid if -8 <= r["dspeed"] <= 8]
    print("\n== 按 |dSpeed| 分段（AF 主要作用于加减速）==")
    seg_stats(accel, "加速")
    seg_stats(decel, "减速")
    seg_stats(const, "匀速")

    # ---- AF 方向一致性：|af|>=3 且 |err|>=3 ----
    print("\n== AF 方向与误差方向（>0=AF助，<0=AF抵）==")
    for name, rs in (("加速", accel), ("减速", decel), ("匀速", const), ("右转", [r for r in valid if r["state"] == 4])):
        act = [r for r in rs if abs(r["af"]) >= 3 and abs(r["err"]) >= 3]
        if not act:
            print("  [{}] 无有效AF帧".format(name))
            continue
        same = sum(1 for r in act if (r["af"] > 0) == (r["err"] > 0))
        print("  [{}] 有效AF帧={}  助={}  抵={}".format(name, len(act), same, len(act) - same))

    # ---- AF 实际大小 vs 理想 KAF*accf ----
    print("\n== AF 幅度核对 ==")
    for name, rs in (("加速", accel), ("减速", decel), ("匀速", const)):
        af = [r["af"] for r in rs]
        ideal = [KAF * r["accf"] for r in rs]
        print("  [{}] af_slewed mean={:+.1f} | KAF*accf mean={:+.1f} | 两者接近率={:.0f}%".format(
            name, mean(af), mean(ideal),
            100 * sum(1 for a, b in zip(af, ideal) if abs(a - b) < 5) / max(1, len(af))))

    # ---- YF 在右转段的表现 ----
    tr = [r for r in valid if r["state"] == 4]
    print("\n== 右转段 YF ==")
    if tr:
        yaw = [r["yaw"] for r in tr]
        yf = [r["yf"] for r in tr]
        err = [abs(r["err"]) for r in tr]
        print("  yaw_raw: min={:.0f} mean={:.0f} | yf_slewed mean={:+.1f} max={:+.1f} | 误差 MAE={:.1f}".format(
            min(yaw), mean(yaw), mean(yf), max(yf), mean(err)))
        # 进入弯道后 1s 内的误差（yaw 建立前后对比）
        t0 = tr[0]["tick"]
        early = [r for r in tr if r["tick"] - t0 < 1000]
        late = [r for r in tr if r["tick"] - t0 >= 1000]
        if early and late:
            print("  入弯前1s MAE={:.1f} | 入弯后 MAE={:.1f}".format(
                mean(abs(r["err"]) for r in early), mean(abs(r["err"]) for r in late)))

    # ---- I 饱和检查 ----
    print("\n== I 项极限情况 ==")
    isat = [r for r in valid if abs(r["i"]) > 75]
    print("  |I|>75us 帧: {} ({:.1f}%)".format(len(isat), 100 * len(isat) / max(1, len(valid))))
    if isat:
        for r in isat[:8]:
            print("    t={}ms st={} ball={} err={:+.0f} I={:+.0f} af={:+.0f} yf={:+.0f} speed={}".format(
                int(r["tick"]), r["state"], int(r["ball"]), r["err"], r["i"], r["af"], r["yf"], int(r["speed"])))

    # ---- 最差段定位（|err|>25）----
    print("\n== |err|>25px 段 ==")
    segs, cur = [], []
    for r in valid:
        if abs(r["err"]) > 25:
            cur.append(r)
        else:
            if cur:
                segs.append(cur)
                cur = []
    if cur:
        segs.append(cur)
    segs.sort(key=len, reverse=True)
    for s in segs[:4]:
        st = Counter(r["state"] for r in s)
        afr = Counter(1 if (r["af"] > 0) == (r["err"] > 0) else -1 for r in s if abs(r["af"]) >= 3)
        print("  t={}~{}ms ({}ms) n={} err[{}..{}] I[{}..{}] af[{}..{}] yf[{}..{}] accf[{}..{}] state={}".format(
            int(s[0]["tick"]), int(s[-1]["tick"]), int(s[-1]["tick"] - s[0]["tick"]), len(s),
            int(min(r["err"] for r in s)), int(max(r["err"] for r in s)),
            int(min(r["i"] for r in s)), int(max(r["i"] for r in s)),
            int(min(r["af"] for r in s)), int(max(r["af"] for r in s)),
            int(min(r["yf"] for r in s)), int(max(r["yf"] for r in s)),
            int(min(r["accf"] for r in s)), int(max(r["accf"] for r in s)),
            dict(st)))
        print("    AF助/抵:", dict(afr))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
