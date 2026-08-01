# -*- coding: utf-8 -*-
"""细看长右转段(t=20282~26962ms)的误差波形、I项、YF建速。"""
import csv
import sys

CSV = r"C:\Users\32142\Desktop\ball_run.csv"


def load():
    with open(CSV, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        out.append({
            "t": float(r["stm32_tick_ms"]),
            "state": int(r["motion_state"]),
            "err": float(r["error_px"]),
            "co": float(r["control_offset_us"]),
            "i": float(r["i_term_us"]),
            "af": float(r["af_slewed_us"]),
            "yf": float(r["yf_slewed_us"]),
            "yaw": float(r["yaw_raw_dps"]),
            "accf": float(r["acc_filtered_mg"]),
            "hold": float(r["hold_pwm_effective_us"]),
            "feed": float(r["feedforward_total_us"]),
            "sp": float(r["speed_average"]),
            "sc": float(r["speed_scale_x1000"]),
            "pv": float(r["turn_preview_slewed_us"]),
        })
    return out


def main():
    rows = load()
    seg = [r for r in rows if 20000 <= r["t"] <= 27200 and r["state"] == 4]
    # 每 ~1s 采样一行
    print("t(ms)  err  MAE1s  I     yf    yaw   af    accf  co    hold   state")
    last_t = -1
    bucket = []
    for r in seg:
        bucket.append(r)
        if r["t"] - last_t >= 1000:
            e = [x["err"] for x in bucket]
            mae = sum(abs(x) for x in e) / len(e)
            avg = lambda f: sum(f(x) for x in bucket) / len(bucket)
            print("{:6.0f} {:+5.0f}  {:4.1f}  {:+5.1f} {:+5.1f} {:5.1f} {:+6.1f} {:6.1f} {:+7.1f} {:4.0f}  {}".format(
                bucket[-1]["t"], avg(lambda x: x["err"]), mae,
                avg(lambda x: x["i"]), avg(lambda x: x["yf"]),
                avg(lambda x: x["yaw"]), avg(lambda x: x["af"]),
                avg(lambda x: x["accf"]), avg(lambda x: x["co"]),
                avg(lambda x: x["hold"]), r["state"]))
            last_t = r["t"]
            bucket = []
    # 误差符号变化率（判断是否振荡）
    signs = [1 if r["err"] > 2 else (-1 if r["err"] < -2 else 0) for r in seg]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != 0 and b != 0 and a != b)
    print("\n误差符号翻转次数(>2px): {} 段长={}ms".format(flips, int(seg[-1]["t"] - seg[0]["t"])))
    big = [r for r in seg if abs(r["err"]) > 15]
    print("|err|>15 帧: {} / {} ({:.0f}%)  err范围[{:.0f}..{:.0f}]".format(
        len(big), len(seg), 100 * len(big) / max(1, len(seg)),
        min(r["err"] for r in seg), max(r["err"] for r in seg)))
    isat = [r for r in seg if abs(r["i"]) > 60]
    print("|I|>60 帧: {}   max I={:+.1f}".format(len(isat), max(abs(r["i"]) for r in seg)))
    yfmax = [r for r in seg if r["yf"] >= 24]
    print("yf>=24(接近限幅25) 帧: {}   max yf={:+.1f}".format(len(yfmax), max(r["yf"] for r in seg)))
    # 误差与yf的关系：yf大时误差是否更小
    yf_hi = [r for r in seg if r["yf"] >= 10]
    yf_lo = [r for r in seg if r["yf"] < 10]
    m = lambda rs: sum(abs(r["err"]) for r in rs) / len(rs) if rs else 0
    print("yf>=10 帧MAE={:.1f} (n={})  yf<10 帧MAE={:.1f} (n={})".format(
        m(yf_hi), len(yf_hi), m(yf_lo), len(yf_lo)))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
