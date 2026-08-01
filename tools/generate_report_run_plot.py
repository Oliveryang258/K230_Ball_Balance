#!/usr/bin/env python3
"""Generate the report figure and summary for the retained 2026-08-01 run."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = ROOT / "report" / "data" / "r6_engineering_run_20260801.csv"
FIG_PATH = ROOT / "report" / "figures" / "r6-engineering-run.png"
JSON_PATH = ROOT / "report" / "data" / "r6_engineering_run_20260801_summary.json"
PX_PER_CM = 25.0


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


with CSV_PATH.open("r", encoding="utf-8-sig", newline="") as f:
    rows = list(csv.DictReader(f))

moving = [
    r for r in rows
    if int(r["guard_state"]) == 5
    and int(r["motion_state"]) in (1, 2, 4)
    and int(r["motion_link_valid"]) == 1
]
if not moving:
    raise SystemExit("No valid moving-control frames found")

start_tick = int(moving[0]["stm32_tick_ms"])
stop_rows = [r for r in rows if int(r["stm32_tick_ms"]) >= start_tick and int(r["motion_state"]) == 5]
stop_tick = int(stop_rows[0]["stm32_tick_ms"]) if stop_rows else int(moving[-1]["stm32_tick_ms"])


def summarize(items: list[dict[str, str]]) -> dict[str, float | int]:
    errors = [abs(float(r["error_px"])) for r in items]
    return {
        "samples": len(items),
        "duration_s": round(len(items) * 0.02, 2),
        "mae_px": round(sum(errors) / len(errors), 1),
        "mae_cm": round(sum(errors) / len(errors) / PX_PER_CM, 2),
        "p95_px": round(percentile(errors, 0.95), 1),
        "p95_cm": round(percentile(errors, 0.95) / PX_PER_CM, 2),
        "max_px": round(max(errors), 1),
        "max_cm": round(max(errors) / PX_PER_CM, 2),
        "within_1cm_pct": round(100.0 * sum(e <= PX_PER_CM for e in errors) / len(errors), 1),
    }


groups = {
    "all": moving,
    "straight": [r for r in moving if int(r["motion_state"]) == 2],
    "right_turn": [r for r in moving if int(r["motion_state"]) == 4],
    "start_stop": [r for r in moving if int(r["motion_state"]) == 1],
}
summary = {
    "source": CSV_PATH.name,
    "px_per_cm_conservative": PX_PER_CM,
    "inferred_target_px": round(sum(float(r["ball_x_px"]) + float(r["error_px"]) for r in moving) / len(moving), 1),
    "lap_time_s": round((stop_tick - start_tick) / 1000.0, 2),
    "ball_loss_events": sum(
        1 for a, b in zip(rows, rows[1:])
        if int(a["guard_state"]) == 5 and int(b["guard_state"]) == 1
        and start_tick <= int(a["stm32_tick_ms"]) <= stop_tick
    ),
    "segments": {name: summarize(items) for name, items in groups.items()},
}
JSON_PATH.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

t = [(int(r["stm32_tick_ms"]) - start_tick) / 1000.0 for r in moving]
e = [float(r["error_px"]) / PX_PER_CM for r in moving]
state = [int(r["motion_state"]) for r in moving]

plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.25})
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 3.7), sharex=True,
                               gridspec_kw={"height_ratios": [3.2, 1]})
ax1.axhspan(-1, 1, color="#2ca02c", alpha=0.10, label="±1 cm requirement band")
ax1.plot(t, e, color="#145ea8", linewidth=0.9, label="Ball position error")
ax1.axhline(0, color="black", linewidth=0.7)
ax1.set_ylabel("Error / cm")
ax1.set_ylim(-2.05, 2.05)
ax1.legend(loc="lower left", frameon=False, ncol=2)
ax1.text(0.99, 0.96,
         f"lap {summary['lap_time_s']:.2f} s   MAE {summary['segments']['all']['mae_cm']:.2f} cm   "
         f"max {summary['segments']['all']['max_cm']:.2f} cm",
         transform=ax1.transAxes, ha="right", va="top")
ax2.step(t, state, where="post", color="#c44e52", linewidth=1.0)
ax2.set_yticks([1, 2, 4], ["START/STOP", "STRAIGHT", "TURN"])
ax2.set_ylabel("State")
ax2.set_xlabel("Time / s")
ax2.set_xlim(0, summary["lap_time_s"])
fig.tight_layout(pad=0.8)
fig.savefig(FIG_PATH, dpi=240, bbox_inches="tight")
print(json.dumps(summary, ensure_ascii=False, indent=2))
