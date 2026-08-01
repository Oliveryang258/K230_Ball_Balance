import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


CSV_PATH = Path(r"C:\Users\32142\Desktop\ball_run.csv")
TARGET_X_EXPECTED = 214
TEST_HOLD_PWM_US = 1440.0
STRICT_ERROR_LIMIT_PX = 25.0


def summary(frame):
    if frame.empty:
        return {"n": 0}
    abs_error = frame["error_px"].abs()
    return {
        "n": int(len(frame)),
        "duration_s": round(
            (frame["stm32_tick_ms"].iloc[-1] - frame["stm32_tick_ms"].iloc[0])
            / 1000.0,
            3,
        ),
        "error_median_px": round(float(frame["error_px"].median()), 3),
        "error_mean_px": round(float(frame["error_px"].mean()), 3),
        "mae_px": round(float(abs_error.mean()), 3),
        "p95_abs_error_px": round(float(abs_error.quantile(0.95)), 3),
        "max_abs_error_px": round(float(abs_error.max()), 3),
        "within_25_ratio": round(float((abs_error <= 25).mean()), 5),
        "outside_25_count": int((abs_error > 25).sum()),
        "velocity_median_px_s": round(float(frame["velocity_px_s"].median()), 3),
        "i_median_us": round(float(frame["i_term_us"].median()), 3),
        "i_mean_us": round(float(frame["i_term_us"].mean()), 3),
        "i_p10_us": round(float(frame["i_term_us"].quantile(0.10)), 3),
        "i_p90_us": round(float(frame["i_term_us"].quantile(0.90)), 3),
        "hold_median_us": round(float(frame["hold_pwm_effective_us"].median()), 3),
        "af_median_us": round(float(frame["af_slewed_us"].median()), 3),
        "yf_median_us": round(float(frame["yf_slewed_us"].median()), 3),
        "servo_prelimit_median_us": round(
            float(frame["servo_prelimit_us"].median()), 3
        ),
    }


def contiguous_runs(frame, mask, minimum_rows=1):
    records = []
    start = None
    previous_index = None
    for index, active in zip(frame.index, mask):
        if active and start is None:
            start = index
        if not active and start is not None:
            end = previous_index
            if end - start + 1 >= minimum_rows:
                records.append((start, end))
            start = None
        previous_index = index
    if start is not None:
        end = previous_index
        if end - start + 1 >= minimum_rows:
            records.append((start, end))
    return records


df = pd.read_csv(CSV_PATH, encoding="utf-8-sig")
df = df.apply(pd.to_numeric, errors="coerce")
df = df.dropna(subset=["stm32_tick_ms", "error_px", "ball_x_px"])
df = df.sort_values("stm32_tick_ms").reset_index(drop=True)
df["target_x_derived"] = df["ball_x_px"] + df["error_px"]

dt = df["stm32_tick_ms"].diff().dropna()
ready = df[(df["guard_state"] == 5) & (df["measurement_status"] == 0)].copy()
first_ready_tick = float(ready["stm32_tick_ms"].iloc[0])
ready["elapsed_ready_s"] = (ready["stm32_tick_ms"] - first_ready_tick) / 1000.0
post_2s = ready[ready["elapsed_ready_s"] >= 2.0].copy()

# control_flags bit5 is output saturation; servo_flags may also carry detailed saturation.
not_saturated = (post_2s["control_flags"].astype(int) & 0x20) == 0
stable_strict = post_2s[
    (post_2s["error_px"].abs() <= 5)
    & (post_2s["velocity_px_s"].abs() <= 10)
    & (post_2s["af_slewed_us"].abs() <= 2)
    & (post_2s["yf_slewed_us"].abs() <= 2)
    & (post_2s["i_term_us"].abs() < 85)
    & not_saturated
].copy()

stable_relaxed = post_2s[
    (post_2s["error_px"].abs() <= 8)
    & (post_2s["velocity_px_s"].abs() <= 15)
    & (post_2s["af_slewed_us"].abs() <= 3)
    & (post_2s["yf_slewed_us"].abs() <= 3)
    & (post_2s["i_term_us"].abs() < 85)
    & not_saturated
].copy()

outside_mask = post_2s["error_px"].abs() > STRICT_ERROR_LIMIT_PX
runs = []
for start, end in contiguous_runs(post_2s.reset_index(drop=True), outside_mask.to_numpy()):
    part = post_2s.reset_index(drop=True).iloc[start : end + 1]
    runs.append(
        {
            "start_s": round(float(part["elapsed_ready_s"].iloc[0]), 3),
            "end_s": round(float(part["elapsed_ready_s"].iloc[-1]), 3),
            "duration_s": round(
                float(
                    part["stm32_tick_ms"].iloc[-1]
                    - part["stm32_tick_ms"].iloc[0]
                    + 20
                )
                / 1000.0,
                3,
            ),
            "peak_error_px": int(part.loc[part["error_px"].abs().idxmax(), "error_px"]),
            "i_median_us": round(float(part["i_term_us"].median()), 1),
            "af_median_us": round(float(part["af_slewed_us"].median()), 1),
            "yf_median_us": round(float(part["yf_slewed_us"].median()), 1),
            "hold_median_us": round(float(part["hold_pwm_effective_us"].median()), 1),
        }
    )

last_8s = ready[
    ready["stm32_tick_ms"] >= ready["stm32_tick_ms"].max() - 8000
].copy()

# 找连续稳定段。单纯把全程所有“恰好稳定”的离散样本混在一起，可能会把
# 不同工况（例如后段前馈介入）误当成同一个平衡状态。
post_reset = post_2s.reset_index(drop=True)
strict_mask_reset = (
    (post_reset["error_px"].abs() <= 5)
    & (post_reset["velocity_px_s"].abs() <= 10)
    & (post_reset["af_slewed_us"].abs() <= 2)
    & (post_reset["yf_slewed_us"].abs() <= 2)
    & (post_reset["i_term_us"].abs() < 85)
    & ((post_reset["control_flags"].astype(int) & 0x20) == 0)
)
stable_runs = []
for start, end in contiguous_runs(post_reset, strict_mask_reset.to_numpy(), minimum_rows=10):
    part = post_reset.iloc[start : end + 1]
    stable_runs.append(
        {
            "start_s": round(float(part["elapsed_ready_s"].iloc[0]), 3),
            "end_s": round(float(part["elapsed_ready_s"].iloc[-1]), 3),
            "duration_s": round(float(len(part)) * 0.02, 3),
            "n": int(len(part)),
            "error_median_px": round(float(part["error_px"].median()), 3),
            "i_median_us": round(float(part["i_term_us"].median()), 3),
            "hold_median_us": round(float(part["hold_pwm_effective_us"].median()), 3),
            "af_median_us": round(float(part["af_slewed_us"].median()), 3),
            "recommended_pwm_us": round(
                float(part["hold_pwm_effective_us"].median())
                - float(part["i_term_us"].median()),
                3,
            ),
        }
    )
stable_runs.sort(key=lambda item: item["n"], reverse=True)

# 两秒窗口用于观察测试过程中工况有没有发生变化。
window_rows = []
window_source = ready.copy()
window_source["window_2s"] = (window_source["elapsed_ready_s"] // 2).astype(int)
for window_id, part in window_source.groupby("window_2s"):
    window_rows.append(
        {
            "start_s": int(window_id) * 2,
            "n": int(len(part)),
            "err_med": round(float(part["error_px"].median()), 1),
            "err_abs_max": round(float(part["error_px"].abs().max()), 1),
            "vel_med": round(float(part["velocity_px_s"].median()), 1),
            "i_med": round(float(part["i_term_us"].median()), 1),
            "hold_med": round(float(part["hold_pwm_effective_us"].median()), 1),
            "af_med": round(float(part["af_slewed_us"].median()), 1),
            "yf_med": round(float(part["yf_slewed_us"].median()), 1),
        }
    )

stable_source = stable_strict if len(stable_strict) >= 20 else stable_relaxed
steady_i = float(stable_source["i_term_us"].median()) if len(stable_source) else math.nan
steady_hold = (
    float(stable_source["hold_pwm_effective_us"].median())
    if len(stable_source)
    else TEST_HOLD_PWM_US
)
recommended_pwm = steady_hold - steady_i if not math.isnan(steady_i) else math.nan

result = {
    "file": {
        "rows": int(len(df)),
        "duration_s": round(
            float(df["stm32_tick_ms"].iloc[-1] - df["stm32_tick_ms"].iloc[0])
            / 1000.0,
            3,
        ),
        "dt_ms_median": float(dt.median()),
        "dt_ms_p95": float(dt.quantile(0.95)),
        "target_x_counts": {
            str(int(k)): int(v)
            for k, v in ready["target_x_derived"].value_counts().head(5).items()
        },
    },
    "ready": summary(ready),
    "post_2s": summary(post_2s),
    "last_8s": summary(last_8s),
    "stable_strict": summary(stable_strict),
    "stable_relaxed": summary(stable_relaxed),
    "longest_continuous_strict_stable_runs": stable_runs[:10],
    "two_second_windows": window_rows,
    "guard_state_counts": {
        str(int(k)): int(v) for k, v in df["guard_state"].value_counts().items()
    },
    "measurement_status_counts": {
        str(int(k)): int(v)
        for k, v in df["measurement_status"].value_counts().items()
    },
    "stable_filter_used_for_recommendation": (
        "strict" if stable_source is stable_strict else "relaxed"
    ),
    "outside_25_runs_after_2s": runs,
    "saturation_rows_post_2s": int((~not_saturated).sum()),
    "integral_at_or_above_89_rows_post_2s": int(
        (post_2s["i_term_us"].abs() >= 89).sum()
    ),
    "recommendation": {
        "steady_hold_median_us": round(steady_hold, 3),
        "steady_i_median_us": round(steady_i, 3),
        "formula": "H_true = H_effective - median(I)",
        "recommended_balance_pwm_us": round(recommended_pwm, 3),
        "rounded_candidate_us": int(round(recommended_pwm))
        if not math.isnan(recommended_pwm)
        else None,
    },
}

print(json.dumps(result, ensure_ascii=False, indent=2))
