import csv
import json
import math
import statistics
from collections import Counter

PATH = r"C:\Users\32142\Desktop\ball_run.csv"


def percentile(values, q):
    values = sorted(values)
    if not values:
        return None
    pos = (len(values) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - pos) + values[hi] * (pos - lo)


with open(PATH, "r", encoding="utf-8-sig", newline="") as f:
    rows = list(csv.DictReader(f))

numeric_rows = []
for row in rows:
    numeric_rows.append({key: float(value) for key, value in row.items()})

ticks = [int(row["stm32_tick_ms"]) for row in numeric_rows]
dts = [b - a for a, b in zip(ticks, ticks[1:])]
targets = [
    int(row["ball_x_px"] + row["error_px"])
    for row in numeric_rows
    if int(row["ball_x_px"]) != 0
]

ready = [row for row in numeric_rows if int(row["guard_state"]) == 5]
moving = [
    row for row in ready
    if int(row["motion_state"]) in (1, 2, 3, 4, 5)
    and int(row["motion_link_valid"]) == 1
]
stopped = [
    row for row in ready
    if int(row["motion_state"]) == 0
    and int(row["motion_link_valid"]) == 1
]


def metrics(subset):
    errors = [row["error_px"] for row in subset]
    absolute = [abs(value) for value in errors]
    return {
        "n": len(subset),
        "signed_mean_error_px": statistics.fmean(errors) if errors else None,
        "mae_px": statistics.fmean(absolute) if absolute else None,
        "median_abs_px": percentile(absolute, 0.5),
        "p90_abs_px": percentile(absolute, 0.9),
        "p95_abs_px": percentile(absolute, 0.95),
        "max_abs_px": max(absolute) if absolute else None,
        "pass_abs_15_ratio": (
            sum(value <= 15 for value in absolute) / len(absolute)
            if absolute else None
        ),
        "pass_abs_25_ratio": (
            sum(value <= 25 for value in absolute) / len(absolute)
            if absolute else None
        ),
    }


kp_samples = [
    row["p_term_us"] / row["error_px"]
    for row in ready
    if abs(row["error_px"]) > 4
]
kv_samples = [
    -row["d_term_us"] / row["velocity_px_s"]
    for row in ready
    if abs(row["velocity_px_s"]) >= 20
]
dir_samples = [
    row["control_offset_us"]
    / (row["p_term_us"] + row["i_term_us"] + row["d_term_us"])
    for row in ready
    if abs(row["p_term_us"] + row["i_term_us"] + row["d_term_us"]) >= 20
]
acc_ff_samples = []
for row in moving:
    acceleration = row["acc_track_mg"]
    equilibrium_estimate = row["servo_target_us"] - row["control_offset_us"]
    feedforward_estimate = equilibrium_estimate - 1410.0
    if abs(acceleration) >= 3 and abs(feedforward_estimate) < 50:
        acc_ff_samples.append(feedforward_estimate / acceleration)

result = {
    "rows": len(numeric_rows),
    "duration_s": (ticks[-1] - ticks[0]) / 1000.0,
    "tick_dt_ms": {
        "median": statistics.median(dts),
        "p95": percentile(dts, 0.95),
        "min": min(dts),
        "max": max(dts),
    },
    "target_x_mode": Counter(targets).most_common(5),
    "guard_counts": Counter(int(row["guard_state"]) for row in numeric_rows),
    "measurement_status_counts": Counter(
        int(row["measurement_status"]) for row in numeric_rows
    ),
    "motion_state_counts": Counter(
        int(row["motion_state"]) for row in numeric_rows
    ),
    "ready": metrics(ready),
    "moving": metrics(moving),
    "stopped": metrics(stopped),
    "inferred": {
        "kp_median_us_per_px": statistics.median(kp_samples),
        "kv_median_us_per_px_s": statistics.median(kv_samples),
        "direction_median": statistics.median(dir_samples),
        "kaf_median_us_per_mg": statistics.median(acc_ff_samples),
    },
}

print(json.dumps(result, ensure_ascii=False, indent=2, default=dict))
