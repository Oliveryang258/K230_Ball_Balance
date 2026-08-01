# -*- coding: utf-8 -*-
"""用21:43 CSV实证校验前馈符号，回答"前馈参数要不要改"。

核心问题：
A. PWM→球 映射方向（direction=-1 下，err>0 是否=降低PWM→球右移收敛）。
B. AF 在加速/减速段是助还是抵（脚本标签 vs 实际物理）。
C. YF 在右转段是助还是抵；yaw 建立后误差是否下降。
"""
import csv
import sys
from collections import Counter

CSV = r"C:\Users\32142\Desktop\ball_run.csv"


def mean(v):
    v = list(v)
    return sum(v) / len(v) if v else 0.0


def load():
    with open(CSV, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["tick"] = float(r["stm32_tick_ms"])
        r["state"] = int(r["motion_state"])
        r["err"] = float(r["error_px"])
        r["co"] = float(r["control_offset_us"])
        r["af"] = float(r["af_slewed_us"])
        r["yf"] = float(r["yf_slewed_us"])
        r["yaw"] = float(r["yaw_raw_dps"])
        r["speed"] = float(r["speed_average"])
        r["accf"] = float(r["acc_filtered_mg"])
        r["hold"] = float(r["hold_pwm_effective_us"])
        r["feed"] = float(r["feedforward_total_us"])
        r["link"] = int(r["motion_link_valid"])
    # d_err/dt：下一帧误差变化（正=球更偏右/误差增大）
    for i, r in enumerate(rows):
        r["derr"] = rows[i + 1]["err"] - r["err"] if i + 1 < len(rows) else 0.0
    return rows


def main():
    rows = load()
    valid = [r for r in rows if r["state"] in (1, 2, 3, 4) and r["link"]]

    # ---- A. PWM→球映射：err>0 时 control_offset 应为负，且误差应收敛 ----
    print("== A. PWM→球映射（direction=-1）==")
    on = [r for r in valid if abs(r["err"]) >= 5 and abs(r["co"]) >= 5]
    neg_co_when_err_pos = sum(1 for r in on if (r["co"] < 0) == (r["err"] > 0))
    print("  |err|>=5 且 |co|>=5 帧: {}  其中 co 与 err 异号(负反馈): {} ({:.1f}%)".format(
        len(on), neg_co_when_err_pos, 100 * neg_co_when_err_pos / max(1, len(on))))
    # 误差收敛性：co 异号帧中，下一帧 |err| 是否变小
    shrink = 0
    for r in on:
        if (r["co"] < 0) == (r["err"] > 0) and abs(r["err"]) > abs(r["derr"] + r["err"]) - 0.001:
            # |err_new| < |err_old| 当 err 同号且向零收
            new_err = r["err"] + r["derr"]
            if (new_err > 0) == (r["err"] > 0) and abs(new_err) < abs(r["err"]):
                shrink += 1
    print("  负反馈帧中下一帧误差向零收敛: {:.1f}%".format(100 * shrink / max(1, neg_co_when_err_pos)))

    # ---- B. AF 符号 ----
    for i, r in enumerate(valid):
        r["dspeed"] = r["speed"] - valid[i - 1]["speed"] if i else 0.0
    accel = [r for r in valid if r["dspeed"] > 8]
    decel = [r for r in valid if r["dspeed"] < -8]
    const = [r for r in valid if -8 <= r["dspeed"] <= 8]

    print("\n== B. AF 助/抵（物理映射：err>0=球左→需降PWM；af<0=降PWM→助）==")
    print("  约定：af 与 err 异号 = 助，同号 = 抵")
    for name, rs in (("加速", accel), ("减速", decel), ("匀速", const),
                     ("右转", [r for r in valid if r["state"] == 4])):
        act = [r for r in rs if abs(r["af"]) >= 5 and abs(r["accf"]) >= 5]
        if not act:
            print("  [{}] 无有效AF帧".format(name))
            continue
        help_ = sum(1 for r in act if (r["af"] > 0) != (r["err"] > 0))
        hurt = len(act) - help_
        # 误差更大时才统计：|err|>=3
        act2 = [r for r in act if abs(r["err"]) >= 3]
        h2 = sum(1 for r in act2 if (r["af"] > 0) != (r["err"] > 0)) if act2 else 0
        print("  [{}] 有效帧={} 助={}({:.0f}%) 抵={} | 其中|err|>=3: {}帧 助={:.0f}%".format(
            name, len(act), help_, 100 * help_ / len(act), len(act) - help_,
            len(act2), 100 * h2 / max(1, len(act2))))

    # ---- C. YF 右转段 ----
    tr = [r for r in valid if r["state"] == 4]
    print("\n== C. 右转段 YF ==")
    if tr:
        # 切分连续右转段
        segs, cur = [], []
        for r in tr:
            if cur and r["tick"] - cur[-1]["tick"] > 50:
                segs.append(cur); cur = []
            cur.append(r)
        if cur:
            segs.append(cur)
        print("  右转段数: {}".format(len(segs)))
        for s in segs:
            t0 = s[0]["tick"]
            e = [r["err"] for r in s]
            yf = [r["yf"] for r in s]
            yaw = [r["yaw"] for r in s]
            # 段前200ms误差（入弯前）
            pre = [r for r in valid if r["tick"] < t0 and r["tick"] >= t0 - 300]
            pre_err = mean(abs(r["err"]) for r in pre) if pre else float("nan")
            print("  t={}~{}ms ({}ms) n={} 误差均值={:+.1f} MAE={:.1f} yaw[{:.0f}..{:.0f}] yf[{:+.0f}..{:+.0f}] mean={:+.1f} | 入弯前300ms MAE={:.1f}".format(
                int(t0), int(s[-1]["tick"]), int(s[-1]["tick"] - t0), len(s),
                mean(e), mean(abs(x) for x in e), min(yaw), max(yaw),
                min(yf), max(yf), mean(yf), pre_err))
            # 误差趋势：后1/3 vs 前1/3
            n = len(s)
            e1 = mean(abs(x) for x in e[:n // 3])
            e3 = mean(abs(x) for x in e[2 * n // 3:])
            print("    误差趋势: 前1/3 MAE={:.1f} → 后1/3 MAE={:.1f}  {}".format(
                e1, e3, "恶化" if e3 > e1 * 1.2 else ("改善" if e3 < e1 * 0.8 else "持平")))
        # YF 与 err 的符号关系（YF 活跃帧）
        yf_on = [r for r in tr if r["yf"] >= 3 and abs(r["err"]) >= 3]
        if yf_on:
            help_ = sum(1 for r in yf_on if (r["yf"] > 0) != (r["err"] > 0))
            print("  YF活跃(|yf|>=3,|err|>=3): {}帧  助(异号)={} 抵(同号)={}".format(
                len(yf_on), help_, len(yf_on) - help_))

    # ---- D. 最差段重新标定 ----
    print("\n== D. |err|>25px 段（含AF/YF/accf方向）==")
    segs2, cur = [], []
    for r in valid:
        if abs(r["err"]) > 25:
            cur.append(r)
        else:
            if cur:
                segs2.append(cur); cur = []
    if cur:
        segs2.append(cur)
    segs2.sort(key=len, reverse=True)
    for s in segs2[:3]:
        af_h = sum(1 for r in s if abs(r["af"]) >= 3 and (r["af"] > 0) != (r["err"] > 0))
        af_a = sum(1 for r in s if abs(r["af"]) >= 3)
        print("  t={}~{}ms n={} err[{:.0f}..{:.0f}] af[{:.0f}..{:.0f}] accf[{:.0f}..{:.0f}] yf[{:+.0f}..{:+.0f}] | AF活跃{}帧 助{:.0f}%".format(
            int(s[0]["tick"]), int(s[-1]["tick"]), len(s),
            min(r["err"] for r in s), max(r["err"] for r in s),
            min(r["af"] for r in s), max(r["af"] for r in s),
            min(r["accf"] for r in s), max(r["accf"] for r in s),
            min(r["yf"] for r in s), max(r["yf"] for r in s),
            af_a, 100 * af_h / max(1, af_a)))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
