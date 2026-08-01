"""控制量分解诊断脚本。

输入来自 decode_k230_telemetry.py 生成的 CSV（V2 或 V3）。
自动定位 abs(error_px) > 15 的最长连续事件，逐周期拆分 P/I/D/AF/YF/舵机链路。

用法：
    python tools/analyze_control_decomposition.py ball_run.csv
    python tools/analyze_control_decomposition.py ball_run.csv --output-dir tmp/
"""

import argparse
import csv
import os
import sys
from pathlib import Path

# 尝试导入 matplotlib；不可用时仍输出文本诊断
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


DIRECTION = -1
HOLD_PWM_US = 1410.0
KP = 3.5
KI = 1.5
KV = 0.9
ERROR_LIMIT_PX = 15.0
PWM_SOFT_MIN = 810.0
PWM_SOFT_MAX = 2010.0


def load_csv(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = [r for r in reader]
    print(f"Loaded {len(rows)} rows from {path}")
    return rows


def filter_ready(rows):
    """只保留 guard_state == 5 的控制有效行。"""
    out = [r for r in rows if r.get("guard_state", "").strip() == "5"]
    print(f"  guard_state==5 rows: {len(out)} / {len(rows)}")
    return out


def as_float(row, key, default=0.0):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def as_int(row, key, default=0):
    try:
        return int(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def detect_v3(rows):
    """检测是否有 V3 分解字段。"""
    if not rows:
        return False
    v = rows[0].get("af_slewed_us", "").strip()
    return v != "" and v is not None


def find_error_events(rows):
    """找到所有 abs(error_px) > ERROR_LIMIT_PX 的连续段，返回 (start_idx, end_idx, duration_ticks)。"""
    events = []
    in_event = False
    start = 0
    for i, r in enumerate(rows):
        err = as_float(r, "error_px")
        if abs(err) > ERROR_LIMIT_PX:
            if not in_event:
                start = i
                in_event = True
        else:
            if in_event:
                events.append((start, i - 1, i - start))
                in_event = False
    if in_event:
        events.append((start, len(rows) - 1, len(rows) - start))

    events.sort(key=lambda x: x[2], reverse=True)
    return events


def compute_derived(rows, is_v3):
    """为每一行计算派生量。"""
    for r in rows:
        err = as_float(r, "error_px")
        vel = as_float(r, "velocity_px_s")
        p = as_float(r, "p_term_us")
        i = as_float(r, "i_term_us")
        d = as_float(r, "d_term_us")
        ctrl_off = as_float(r, "control_offset_us")
        s_target = as_float(r, "servo_target_us")
        s_current = as_float(r, "servo_current_us")

        r["_feedback_us"] = ctrl_off
        r["_net_target_offset_us"] = s_target - HOLD_PWM_US
        r["_tracking_error_us"] = s_target - s_current

        if is_v3:
            r["_af_us"] = as_float(r, "af_slewed_us")
            r["_yf_us"] = as_float(r, "yf_slewed_us")
            r["_ff_total"] = as_float(r, "feedforward_total_us")
            r["_prelimit"] = as_float(r, "servo_prelimit_us")
            r["_hold_pwm_eff"] = as_float(r, "hold_pwm_effective_us")
            r["_preview_raw"] = as_float(r, "turn_preview_raw_us")
            r["_preview_slewed"] = as_float(r, "turn_preview_slewed_us")
            r["_turn_scale"] = as_float(r, "turn_scale")
            r["_yaw_handover"] = as_float(r, "yaw_handover")
        else:
            r["_af_us"] = None
            r["_yf_us"] = None
            r["_ff_total"] = None
            r["_prelimit"] = None
            r["_hold_pwm_eff"] = None
            r["_preview_raw"] = None
            r["_preview_slewed"] = None
            r["_turn_scale"] = None
            r["_yaw_handover"] = None


def analyze_worst_event(rows, event, is_v3, output_dir):
    """对最严重事件输出详细分析。"""
    start_i, end_i, duration = event
    pre_start = max(0, start_i - 50)  # ~1s before
    post_end = min(len(rows) - 1, end_i + 50)  # ~1s after

    event_rows = rows[start_i:end_i + 1]
    pre_rows = rows[pre_start:start_i]
    post_rows = rows[end_i + 1:post_end + 1]

    event_start_tick = as_int(event_rows[0], "stm32_tick_ms")

    def t_s(r):
        return (as_int(r, "stm32_tick_ms") - event_start_tick) / 1000.0

    print(f"\n{'='*70}")
    print(f"Worst event analysis")
    print(f"{'='*70}")
    print(f"  CSV row range: {start_i} → {end_i}  ({duration} ticks, ~{duration*20/1000:.2f}s)")
    print(f"  stm32_tick range: {as_int(event_rows[0],'stm32_tick_ms')} → {as_int(event_rows[-1],'stm32_tick_ms')}")
    print(f"  Error range: {min(as_float(r,'error_px') for r in event_rows):.0f} → {max(as_float(r,'error_px') for r in event_rows):.0f} px")

    # Peak error row
    peak_row = max(event_rows, key=lambda r: abs(as_float(r, "error_px")))
    peak_idx = rows.index(peak_row)
    print(f"  Peak error: {as_float(peak_row,'error_px'):.0f} px at idx={peak_idx}, tick={as_int(peak_row,'stm32_tick_ms')}")

    # Recovery row (first row after event where error <= 15)
    recovery_rows = [r for r in rows[end_i + 1:end_i + 60]
                     if abs(as_float(r, "error_px")) <= ERROR_LIMIT_PX]
    recovery_row = recovery_rows[0] if recovery_rows else None

    # Print peak moment decomposition
    print(f"\n  --- Peak moment (idx={peak_idx}) ---")
    _print_row_diag(peak_row, is_v3)

    # Print recovery moment
    if recovery_row:
        rec_idx = rows.index(recovery_row)
        print(f"\n  --- Recovery (error ≤ 15px, idx={rec_idx}) ---")
        _print_row_diag(recovery_row, is_v3)

    # Print last event row (just before recovery)
    last_event_row = event_rows[-1]
    print(f"\n  --- Last event row (idx={end_i}) ---")
    _print_row_diag(last_event_row, is_v3)

    # Motion state analysis
    motion_states = {}
    for r in event_rows:
        ms = r.get("motion_state", "?").strip()
        motion_states[ms] = motion_states.get(ms, 0) + 1
    print(f"\n  Motion states during event: {motion_states}")

    # Generate plots
    if HAS_MPL:
        _generate_plots(rows, pre_start, post_end, event_start_tick,
                        start_i, end_i, peak_idx, recovery_row, rows.index(recovery_row) if recovery_row else None,
                        is_v3, output_dir)
    else:
        print("\n  [WARNING] matplotlib not available, skipping plots")

    # Answer diagnostic questions
    _diagnose(rows, event, peak_row, recovery_row, is_v3)


def _print_row_diag(row, is_v3):
    """打印单行的控制量分解。"""
    fields = [
        ("error_px", "error", "px", 0),
        ("velocity_px_s", "velocity", "px/s", 1),
        ("p_term_us", "P", "us", 1),
        ("i_term_us", "I", "us", 1),
        ("d_term_us", "D", "us", 1),
        ("control_offset_us", "control_offset", "us", 1),
        ("servo_target_us", "servo_target", "us", 0),
        ("servo_current_us", "servo_current", "us", 0),
    ]
    for key, label, unit, decimals in fields:
        v = as_float(row, key)
        fmt = f"{v:.{decimals}f}"
        print(f"    {label:20s} = {fmt:>10s} {unit}")

    if is_v3:
        v3_fields = [
            ("af_slewed_us", "AF slewed", "us"),
            ("yf_slewed_us", "YF slewed", "us"),
            ("feedforward_total_us", "FF total", "us"),
            ("servo_prelimit_us", "servo_prelimit", "us"),
            ("turn_preview_slewed_us", "preview slewed", "us"),
            ("turn_scale", "turn_scale", ""),
            ("yaw_handover", "yaw_handover", ""),
        ]
        for key, label, unit in v3_fields:
            v = as_float(row, key)
            print(f"    {label:20s} = {v:>10.3f} {unit}" if unit == "" else f"    {label:20s} = {v:>10.1f} {unit}")

    tracking = as_float(row, "servo_target_us") - as_float(row, "servo_current_us")
    print(f"    {'tracking_error':20s} = {tracking:>10.0f} us")


def _generate_plots(rows, pre_start, post_end, event_start_tick,
                    ev_start_i, ev_end_i, peak_idx, rec_row, rec_idx,
                    is_v3, output_dir):
    """生成5张分解图。"""
    plot_rows = rows[pre_start:post_end + 1]

    def t_s(r):
        return (as_int(r, "stm32_tick_ms") - event_start_tick) / 1000.0

    times = [t_s(r) for r in plot_rows]
    row_indices = list(range(pre_start, post_end + 1))

    # Markers
    def _vline(label, idx, color, ymax_frac=0.95):
        if idx is not None and pre_start <= idx <= post_end:
            t = (as_int(rows[idx], "stm32_tick_ms") - event_start_tick) / 1000.0
            for ax in axes:
                ax.axvline(x=t, color=color, linestyle="--", alpha=0.6, linewidth=0.8)
            axes[0].text(t, axes[0].get_ylim()[1] * ymax_frac, label,
                         color=color, fontsize=7, rotation=90, va="top")

    # Find key time markers
    t_error_start = ev_start_i
    t_error_peak = peak_idx
    t_error_end = rec_idx

    # Find motion state transitions
    t_turn_start = None
    t_turn_end = None
    t_straight_after = None
    for i in range(pre_start, post_end + 1):
        ms = rows[i].get("motion_state", "").strip()
        if ms == "4" and t_turn_start is None:  # TURN_RIGHT
            t_turn_start = i
        if t_turn_start is not None and ms != "4" and t_turn_end is None:
            t_turn_end = i
        if t_turn_end is not None and ms == "2" and t_straight_after is None:  # STRAIGHT
            t_straight_after = i

    # Figure 1: Position and velocity
    fig1, (ax1a, ax1b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig1.suptitle("Fig 1: Ball Position Error & Velocity", fontsize=12)

    errs = [as_float(r, "error_px") for r in plot_rows]
    vels = [as_float(r, "velocity_px_s") for r in plot_rows]

    ax1a.plot(times, errs, 'b-', linewidth=0.8, label='error_px')
    ax1a.axhline(y=ERROR_LIMIT_PX, color='r', linestyle='--', alpha=0.5, label=f'+{ERROR_LIMIT_PX:.0f} px')
    ax1a.axhline(y=-ERROR_LIMIT_PX, color='r', linestyle='--', alpha=0.5, label=f'-{ERROR_LIMIT_PX:.0f} px')
    ax1a.axhline(y=0, color='gray', alpha=0.3)
    ax1a.set_ylabel('Error (px)')
    ax1a.legend(fontsize=7, loc='upper right')
    ax1a.grid(True, alpha=0.3)

    ax1b.plot(times, vels, 'g-', linewidth=0.8, label='velocity_px_s')
    ax1b.axhline(y=0, color='gray', alpha=0.3)
    ax1b.set_ylabel('Velocity (px/s)')
    ax1b.set_xlabel('Time from event start (s)')
    ax1b.legend(fontsize=7)
    ax1b.grid(True, alpha=0.3)

    axes = [ax1a, ax1b]
    _vline("err>15", t_error_start, 'red')
    _vline("peak", t_error_peak, 'darkred')
    _vline("err<15", t_error_end, 'green')
    _vline("turn", t_turn_start, 'orange')
    _vline("turn_end", t_turn_end, 'orange')

    fig1.tight_layout()
    fig1.savefig(os.path.join(output_dir, "fig1_position_velocity.png"), dpi=150)
    plt.close(fig1)
    print("  Saved fig1_position_velocity.png")

    # Figure 2: PID decomposition
    fig2, (ax2a, ax2b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig2.suptitle("Fig 2: PID Decomposition", fontsize=12)

    p_terms = [as_float(r, "p_term_us") for r in plot_rows]
    i_terms = [as_float(r, "i_term_us") for r in plot_rows]
    d_terms = [as_float(r, "d_term_us") for r in plot_rows]
    ctrl_offs = [as_float(r, "control_offset_us") for r in plot_rows]

    ax2a.plot(times, p_terms, 'b-', linewidth=0.8, label='P (raw)')
    ax2a.plot(times, i_terms, 'r-', linewidth=0.8, label='I (raw)')
    ax2a.plot(times, d_terms, 'g-', linewidth=0.8, label='D (raw)')
    ax2a.axhline(y=0, color='gray', alpha=0.3)
    ax2a.set_ylabel('PID terms (us)')
    ax2a.legend(fontsize=7)
    ax2a.grid(True, alpha=0.3)

    ax2b.plot(times, ctrl_offs, 'm-', linewidth=0.8, label='control_offset_us (directed)')
    ax2b.axhline(y=0, color='gray', alpha=0.3)
    ax2b.set_ylabel('Control offset (us)')
    ax2b.set_xlabel('Time from event start (s)')
    ax2b.legend(fontsize=7)
    ax2b.grid(True, alpha=0.3)

    axes = [ax2a, ax2b]
    _vline("err>15", t_error_start, 'red')
    _vline("peak", t_error_peak, 'darkred')
    _vline("err<15", t_error_end, 'green')
    _vline("turn", t_turn_start, 'orange')
    _vline("turn_end", t_turn_end, 'orange')

    fig2.tight_layout()
    fig2.savefig(os.path.join(output_dir, "fig2_pid_decomposition.png"), dpi=150)
    plt.close(fig2)
    print("  Saved fig2_pid_decomposition.png")

    # Figure 3: Feedforward decomposition (AF + YF)
    if is_v3:
        fig3, (ax3a, ax3b, ax3c) = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        fig3.suptitle("Fig 3: Feedforward Decomposition", fontsize=12)

        af_raw = [as_float(r, "af_raw_us") for r in plot_rows]
        af_clamped = [as_float(r, "af_clamped_us") for r in plot_rows]
        af_slewed = [as_float(r, "af_slewed_us") for r in plot_rows]

        ax3a.plot(times, af_raw, 'b-', linewidth=0.5, alpha=0.5, label='AF raw')
        ax3a.plot(times, af_clamped, 'b--', linewidth=0.8, alpha=0.7, label='AF clamped')
        ax3a.plot(times, af_slewed, 'b-', linewidth=1.2, label='AF slewed')
        ax3a.axhline(y=0, color='gray', alpha=0.3)
        ax3a.set_ylabel('AF (us)')
        ax3a.legend(fontsize=7)
        ax3a.grid(True, alpha=0.3)

        yf_raw = [as_float(r, "yf_raw_us") for r in plot_rows]
        yf_clamped = [as_float(r, "yf_clamped_us") for r in plot_rows]
        yf_slewed = [as_float(r, "yf_slewed_us") for r in plot_rows]

        ax3b.plot(times, yf_raw, 'r-', linewidth=0.5, alpha=0.5, label='YF raw')
        ax3b.plot(times, yf_clamped, 'r--', linewidth=0.8, alpha=0.7, label='YF clamped')
        ax3b.plot(times, yf_slewed, 'r-', linewidth=1.2, label='YF slewed')
        ax3b.axhline(y=0, color='gray', alpha=0.3)
        ax3b.set_ylabel('YF (us)')
        ax3b.legend(fontsize=7)
        ax3b.grid(True, alpha=0.3)

        # Preview subplot
        preview_raw = [as_float(r, "turn_preview_raw_us") for r in plot_rows]
        preview_slewed = [as_float(r, "turn_preview_slewed_us") for r in plot_rows]
        turn_scales = [as_float(r, "turn_scale") for r in plot_rows]

        ax3c.plot(times, preview_raw, 'm-', linewidth=0.5, alpha=0.5, label='preview raw')
        ax3c.plot(times, preview_slewed, 'm-', linewidth=1.2, label='preview slewed')
        ax3c_twin = ax3c.twinx()
        ax3c_twin.plot(times, turn_scales, 'orange', linewidth=0.5, alpha=0.5, linestyle='--', label='turn_scale')
        ax3c_twin.set_ylabel('turn_scale', color='orange', fontsize=8)
        ax3c.axhline(y=0, color='gray', alpha=0.3)
        ax3c.set_ylabel('Preview (us)')
        ax3c.set_xlabel('Time from event start (s)')
        ax3c.legend(fontsize=7, loc='upper left')
        ax3c.grid(True, alpha=0.3)

        axes = [ax3a, ax3b, ax3c]
        _vline("err>15", t_error_start, 'red')
        _vline("peak", t_error_peak, 'darkred')
        _vline("err<15", t_error_end, 'green')
        _vline("turn", t_turn_start, 'orange')
        _vline("turn_end", t_turn_end, 'orange')

        fig3.tight_layout()
        fig3.savefig(os.path.join(output_dir, "fig3_feedforward_decomposition.png"), dpi=150)
        plt.close(fig3)
        print("  Saved fig3_feedforward_decomposition.png")
    else:
        # Fallback: plot raw IMU data for V2
        fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        fig3.suptitle("Fig 3: Raw IMU Data (V2 — AF/YF not in telemetry)", fontsize=12)

        acc_raw = [as_float(r, "acc_track_mg") for r in plot_rows]
        yaw_raw = [as_float(r, "yaw_rate_dps") for r in plot_rows]

        ax3a.plot(times, acc_raw, 'b-', linewidth=0.8, label='acc_track_mg (raw)')
        ax3a.axhline(y=0, color='gray', alpha=0.3)
        ax3a.set_ylabel('Acc (mg)')
        ax3a.legend(fontsize=7)
        ax3a.grid(True, alpha=0.3)

        ax3b.plot(times, yaw_raw, 'r-', linewidth=0.8, label='yaw_rate_dps')
        ax3b.axhline(y=0, color='gray', alpha=0.3)
        ax3b.set_ylabel('Yaw (deg/s)')
        ax3b.set_xlabel('Time from event start (s)')
        ax3b.legend(fontsize=7)
        ax3b.grid(True, alpha=0.3)

        axes = [ax3a, ax3b]
        _vline("err>15", t_error_start, 'red')
        _vline("peak", t_error_peak, 'darkred')
        _vline("err<15", t_error_end, 'green')

        fig3.tight_layout()
        fig3.savefig(os.path.join(output_dir, "fig3_imu_raw.png"), dpi=150)
        plt.close(fig3)
        print("  Saved fig3_imu_raw.png (V2 fallback)")

    # Figure 4: Servo output
    fig4, (ax4a, ax4b) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig4.suptitle("Fig 4: Servo Output Chain", fontsize=12)

    s_targets = [as_float(r, "servo_target_us") for r in plot_rows]
    s_currents = [as_float(r, "servo_current_us") for r in plot_rows]
    tracking = [s_targets[i] - s_currents[i] for i in range(len(s_targets))]

    ax4a.plot(times, s_targets, 'b-', linewidth=0.8, label='servo_target_us')
    ax4a.plot(times, s_currents, 'r-', linewidth=0.8, label='servo_current_us')
    ax4a.axhline(y=HOLD_PWM_US, color='gray', linestyle='--', alpha=0.7, label=f'hold_pwm={HOLD_PWM_US:.0f}')
    ax4a.axhline(y=PWM_SOFT_MIN, color='orange', linestyle=':', alpha=0.5)
    ax4a.axhline(y=PWM_SOFT_MAX, color='orange', linestyle=':', alpha=0.5)
    ax4a.set_ylabel('PWM (us)')
    ax4a.legend(fontsize=7, loc='upper right')
    ax4a.grid(True, alpha=0.3)

    if is_v3:
        prelimits = [as_float(r, "servo_prelimit_us") for r in plot_rows]
        ax4a.plot(times, prelimits, 'g--', linewidth=0.5, alpha=0.6, label='servo_prelimit')

    ax4b.plot(times, tracking, 'm-', linewidth=0.8, label='tracking_error (target - current)')
    ax4b.axhline(y=0, color='gray', alpha=0.3)
    ax4b.set_ylabel('Tracking error (us)')
    ax4b.set_xlabel('Time from event start (s)')
    ax4b.legend(fontsize=7)
    ax4b.grid(True, alpha=0.3)

    axes = [ax4a, ax4b]
    _vline("err>15", t_error_start, 'red')
    _vline("peak", t_error_peak, 'darkred')
    _vline("err<15", t_error_end, 'green')
    _vline("turn", t_turn_start, 'orange')
    _vline("turn_end", t_turn_end, 'orange')

    fig4.tight_layout()
    fig4.savefig(os.path.join(output_dir, "fig4_servo_output.png"), dpi=150)
    plt.close(fig4)
    print("  Saved fig4_servo_output.png")

    # Figure 5: Vehicle state
    fig5, axes5 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig5.suptitle("Fig 5: Vehicle State", fontsize=12)

    yaws = [as_float(r, "yaw_rate_dps") for r in plot_rows]
    accs = [as_float(r, "acc_track_mg") for r in plot_rows]
    l_speeds = [as_float(r, "left_speed") for r in plot_rows]
    r_speeds = [as_float(r, "right_speed") for r in plot_rows]
    avg_speeds = [(l_speeds[i] + r_speeds[i]) / 2.0 for i in range(len(l_speeds))]
    turn_cmds = [as_float(r, "turn_command") for r in plot_rows]
    m_states = [as_int(r, "motion_state") for r in plot_rows]

    axes5[0].plot(times, yaws, 'b-', linewidth=0.8, label='yaw_rate (deg/s)')
    axes5[0].axhline(y=0, color='gray', alpha=0.3)
    axes5[0].set_ylabel('Yaw (deg/s)')
    axes5[0].legend(fontsize=7)
    axes5[0].grid(True, alpha=0.3)

    if is_v3:
        acc_filt = [as_float(r, "acc_filtered_mg") for r in plot_rows]
        axes5[1].plot(times, accs, 'b-', linewidth=0.5, alpha=0.5, label='acc_raw (mg)')
        axes5[1].plot(times, acc_filt, 'b-', linewidth=1.0, label='acc_filt (mg)')
    else:
        axes5[1].plot(times, accs, 'b-', linewidth=0.8, label='acc_track_mg')
    axes5[1].axhline(y=0, color='gray', alpha=0.3)
    axes5[1].set_ylabel('Acc (mg)')
    axes5[1].legend(fontsize=7)
    axes5[1].grid(True, alpha=0.3)

    axes5[2].plot(times, avg_speeds, 'b-', linewidth=0.8, label='avg wheel speed')
    axes5[2].plot(times, turn_cmds, 'r-', linewidth=0.8, alpha=0.7, label='turn_command')
    axes5[2].axhline(y=0, color='gray', alpha=0.3)
    axes5[2].set_ylabel('Speed / Turn')
    axes5[2].set_xlabel('Time from event start (s)')
    axes5[2].legend(fontsize=7)
    axes5[2].grid(True, alpha=0.3)

    # Add motion_state annotations
    ms_names = {'0': 'STOP', '1': 'START', '2': 'STRAIGHT', '3': 'TURN_L',
                '4': 'TURN_R', '5': 'STOPPING'}
    prev_ms = None
    for i, ms in enumerate(m_states):
        if ms != prev_ms:
            axes5[2].annotate(ms_names.get(str(ms), str(ms)),
                              (times[i], axes5[2].get_ylim()[1] * 0.9),
                              fontsize=6, color='darkgreen', rotation=45)
            prev_ms = ms

    axes = list(axes5)
    _vline("err>15", t_error_start, 'red')
    _vline("peak", t_error_peak, 'darkred')
    _vline("err<15", t_error_end, 'green')
    _vline("turn", t_turn_start, 'orange')
    _vline("turn_end", t_turn_end, 'orange')

    fig5.tight_layout()
    fig5.savefig(os.path.join(output_dir, "fig5_vehicle_state.png"), dpi=150)
    plt.close(fig5)
    print("  Saved fig5_vehicle_state.png")


def _diagnose(rows, event, peak_row, recovery_row, is_v3):
    """回答诊断问题。"""
    print(f"\n{'='*70}")
    print(f"DIAGNOSIS")
    print(f"{'='*70}")

    ev_start_i, ev_end_i, _ = event
    peak_idx = rows.index(peak_row)

    # 10.1: Build-up phase (start → peak)
    print(f"\n--- 10.1: Was correction timely during error buildup? ---")
    buildup_rows = rows[ev_start_i:peak_idx + 1]

    first_err = as_float(buildup_rows[0], "error_px")
    peak_err = as_float(buildup_rows[-1], "error_px")

    p_start = as_float(buildup_rows[0], "p_term_us")
    p_peak = as_float(buildup_rows[-1], "p_term_us")
    d_start = as_float(buildup_rows[0], "d_term_us")
    d_peak = as_float(buildup_rows[-1], "d_term_us")

    print(f"  Error: {first_err:.0f} → {peak_err:.0f} px")
    print(f"  P:     {p_start:.0f} → {p_peak:.0f} us  (Δ = {p_peak - p_start:+.0f} us)")
    print(f"  D:     {d_start:.0f} → {d_peak:.0f} us  (Δ = {d_peak - d_start:+.0f} us)")
    print(f"  Kp*Δerr = {KP * abs(peak_err - first_err):.0f} us expected P growth")

    # Check D direction during buildup
    avg_vel_buildup = sum(as_float(r, "velocity_px_s") for r in buildup_rows) / max(len(buildup_rows), 1)
    print(f"  Avg velocity during buildup: {avg_vel_buildup:.1f} px/s")
    if abs(avg_vel_buildup) > 5:
        d_sign = "opposing" if as_float(buildup_rows[-1], "d_term_us") * as_float(buildup_rows[-1], "p_term_us") < 0 else "assisting"
        print(f"  D direction: {d_sign} correction (D={d_peak:.1f}, P={p_peak:.1f})")

    # 10.2: Was it at capacity limit at peak?
    print(f"\n--- 10.2: Was capacity limit reached at peak? ---")
    s_target = as_float(peak_row, "servo_target_us")
    s_current = as_float(peak_row, "servo_current_us")
    print(f"  servo_target = {s_target:.0f} us  (soft limit: {PWM_SOFT_MIN:.0f}..{PWM_SOFT_MAX:.0f})")
    print(f"  servo_current = {s_current:.0f} us")
    saturated = (s_target <= PWM_SOFT_MIN + 1) or (s_target >= PWM_SOFT_MAX - 1)
    print(f"  At soft limit: {'YES' if saturated else 'NO'}")
    tracking_at_peak = s_target - s_current
    print(f"  Tracking lag: {tracking_at_peak:.0f} us  (max slew: 50 us/20ms)")

    if is_v3:
        af_at_peak = as_float(peak_row, "af_slewed_us")
        yf_at_peak = as_float(peak_row, "yf_slewed_us")
        print(f"  AF at peak: {af_at_peak:.1f} us  (limit: ±70 us)")
        print(f"  YF at peak: {yf_at_peak:.1f} us  (limit: ±30 us)")
        af_sat = abs(af_at_peak) >= 69.0
        yf_sat = abs(yf_at_peak) >= 29.0
        print(f"  AF saturated: {'YES' if af_sat else 'NO'}")
        print(f"  YF saturated: {'YES' if yf_sat else 'NO'}")

    # 10.3: Why did correction back off before error returned to ±15?
    print(f"\n--- 10.3: Why did servo_target return to ~{HOLD_PWM_US:.0f} us while error still >15px? ---")
    if recovery_row:
        rec_idx = rows.index(recovery_row)
        # Find row just before recovery (last row with error > 15)
        last_bad = None
        for i in range(rec_idx - 1, ev_start_i - 1, -1):
            if abs(as_float(rows[i], "error_px")) > ERROR_LIMIT_PX:
                last_bad = rows[i]
                break

        if last_bad:
            err_last = as_float(last_bad, "error_px")
            p_last = as_float(last_bad, "p_term_us")
            i_last = as_float(last_bad, "i_term_us")
            d_last = as_float(last_bad, "d_term_us")
            co_last = as_float(last_bad, "control_offset_us")
            st_last = as_float(last_bad, "servo_target_us")
            sc_last = as_float(last_bad, "servo_current_us")

            print(f"  Last error>15px row:")
            print(f"    error={err_last:.0f} px  P={p_last:.0f}  I={i_last:.0f}  D={d_last:.0f}")
            print(f"    control_offset={co_last:.0f}  servo_target={st_last:.0f}  servo_current={sc_last:.0f}")
            print(f"    servo_target - hold_pwm = {st_last - HOLD_PWM_US:.0f} us")

            if is_v3:
                af_last = as_float(last_bad, "af_slewed_us")
                yf_last = as_float(last_bad, "yf_slewed_us")
                ff_last = as_float(last_bad, "feedforward_total_us")
                print(f"    AF={af_last:.1f}  YF={yf_last:.1f}  FF_total={ff_last:.1f}")

            # Determine dominant cause
            if is_v3:
                ff_last_val = as_float(last_bad, "feedforward_total_us")
                feedback = co_last
                if abs(feedback) < 20 and abs(ff_last_val) > 20:
                    print(f"  → Dominant: feedforward (AF+YF={ff_last_val:.0f}) counteracting feedback ({feedback:.0f})")
                elif abs(d_last) > abs(p_last) * 0.5:
                    print(f"  → Dominant: D-term braking (D={d_last:.0f} vs P={p_last:.0f})")
                else:
                    print(f"  → Mixed: feedback={feedback:.0f}, FF={ff_last_val:.0f}")
            else:
                # Without AF/YF, check D vs P
                if abs(d_last) > abs(p_last) * 0.4:
                    print(f"  → D-term likely contributes significantly (D={d_last:.0f} vs P={p_last:.0f})")
                print(f"  → Net control_offset = {co_last:.0f} us")
                print(f"  → servo_target = hold_pwm + servo_slew_state")
                print(f"  (Exact AF/YF contribution unknown without V3 telemetry)")

    # 10.4: Root cause classification
    print(f"\n--- 10.4: Root Cause Assessment ---")
    causes = []

    # Check A: Absolute capacity
    if saturated:
        causes.append(("A: Absolute capacity", "HIGH"))
    else:
        causes.append(("A: Absolute capacity", "LOW — servo not at limit"))

    # Check D: D-term braking
    near_recovery_rows = rows[max(ev_start_i, ev_end_i - 30):ev_end_i + 1]
    d_values = [as_float(r, "d_term_us") for r in near_recovery_rows]
    p_values = [as_float(r, "p_term_us") for r in near_recovery_rows]
    d_dominance = sum(1 for d, p in zip(d_values, p_values) if abs(d) > abs(p) * 0.5)
    if d_dominance > 5:
        causes.append(("F: D-term premature braking", "HIGH — D > 50% of P in {} of last 30 ticks".format(d_dominance)))
    else:
        causes.append(("F: D-term premature braking", "MEDIUM"))

    # Check H: Servo lag
    tracking_values = [as_float(r, "servo_target_us") - as_float(r, "servo_current_us")
                       for r in near_recovery_rows]
    max_lag = max(abs(v) for v in tracking_values)
    if max_lag > 45:
        causes.append(("H: Servo output lag", "HIGH — max lag={:.0f} us".format(max_lag)))
    else:
        causes.append(("H: Servo output lag", "LOW"))

    # Check G: I accumulation
    i_at_peak = as_float(peak_row, "i_term_us")
    causes.append(("G: Integral too slow", "I at peak={:.0f} us (max=90)".format(i_at_peak)))

    # Check general
    causes.append(("I: Multiple factors", "Likely — see detailed breakdown above"))

    for label, assessment in causes:
        print(f"  {label}: {assessment}")

    print(f"\n  Root cause order (preliminary, verify with V3 data):")
    print(f"  1. Check if AF/YF counteracted feedback during recovery")
    print(f"  2. Check if D-term braked while error still > 15px")
    print(f"  3. Check if servo_current lagged servo_target significantly")
    print(f"  4. Burn V3 telemetry for confirmed AF/YF decomposition")


def main():
    parser = argparse.ArgumentParser(
        description="Control decomposition analysis for ball controller."
    )
    parser.add_argument("input", type=Path, help="CSV from decode_k230_telemetry.py")
    parser.add_argument("--output-dir", type=Path, default=Path("tmp"),
                        help="Output directory for plots (default: tmp/)")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_csv(args.input)
    if not rows:
        print("ERROR: empty CSV")
        sys.exit(1)

    rows = filter_ready(rows)
    if len(rows) < 10:
        print("ERROR: too few guard_state==5 rows")
        sys.exit(1)

    is_v3 = detect_v3(rows)
    print(f"  V3 decomposition fields: {'YES' if is_v3 else 'NO (V2 only)'}")

    compute_derived(rows, is_v3)

    # Find error events
    events = find_error_events(rows)
    print(f"\nTotal |error| > {ERROR_LIMIT_PX:.0f}px events: {len(events)}")
    for i, (s, e, d) in enumerate(events[:5]):
        tick_s = as_int(rows[s], "stm32_tick_ms")
        tick_e = as_int(rows[e], "stm32_tick_ms")
        max_err = max(abs(as_float(rows[j], "error_px")) for j in range(s, e + 1))
        print(f"  #{i+1}: idx={s}→{e}  dur={d} ticks ({d*20}ms)  "
              f"tick={tick_s}→{tick_e}  max_err={max_err:.0f}px")

    if not events:
        print("\nNo error events found. Controller is performing well!")
        return

    worst = events[0]
    print(f"\nAnalyzing worst event (#1, {worst[2]} ticks)...")

    analyze_worst_event(rows, worst, is_v3, str(args.output_dir))

    if not is_v3:
        print(f"\n{'='*70}")
        print(f"NOTE: Current CSV is V2 format (no AF/YF decomposition).")
        print(f"To get full decomposition, set in app_config.h:")
        print(f"  #define BALL_TELEMETRY_CONTROL_DECOMPOSITION 1U")
        print(f"Then re-burn STM32, update K230 telemetry_logger.py, and re-decode.")
        print(f"{'='*70}")


if __name__ == "__main__":
    main()
