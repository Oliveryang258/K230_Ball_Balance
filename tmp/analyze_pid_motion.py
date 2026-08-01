from pathlib import Path
import json
import pandas as pd

path = Path(r"C:\Users\32142\Desktop\ball_run.csv")
df = pd.read_csv(path, encoding="utf-8-sig").apply(pd.to_numeric, errors="coerce")
df = df.dropna(subset=["stm32_tick_ms", "error_px", "motion_state"])
ready = df[(df.guard_state == 5) & (df.measurement_status == 0)].copy()
t0 = ready.stm32_tick_ms.iloc[0]
ready["t_s"] = (ready.stm32_tick_ms - t0) / 1000.0
ready = ready[ready.t_s >= 2.0]

out = {}
for state, part in ready.groupby("motion_state"):
    out[str(int(state))] = {
        "n": int(len(part)),
        "duration_s": round(len(part) * 0.02, 2),
        "err_median": round(float(part.error_px.median()), 2),
        "mae": round(float(part.error_px.abs().mean()), 2),
        "max_abs_err": int(part.error_px.abs().max()),
        "outside25": int((part.error_px.abs() > 25).sum()),
        "vel_abs_median": round(float(part.velocity_px_s.abs().median()), 2),
        "p_abs_median": round(float(part.p_term_us.abs().median()), 2),
        "i_median": round(float(part.i_term_us.median()), 2),
        "d_abs_median": round(float(part.d_term_us.abs().median()), 2),
        "af_median": round(float(part.af_slewed_us.median()), 2),
        "af_p10": round(float(part.af_slewed_us.quantile(.1)), 2),
        "af_p90": round(float(part.af_slewed_us.quantile(.9)), 2),
        "yf_median": round(float(part.yf_slewed_us.median()), 2),
        "yf_p90": round(float(part.yf_slewed_us.quantile(.9)), 2),
    }

print(json.dumps(out, ensure_ascii=False, indent=2))
