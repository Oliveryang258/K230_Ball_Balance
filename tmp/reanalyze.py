"""Re-analyze with correct guard state enum (5=READY, not INVALID_DT)."""
import csv
import math

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

# Filter: only rows where guard=5 (READY) and ball is detected
ready_rows = []
for i, row in enumerate(rows):
    if row['guard_state'] == '5' and int(row['ball_x_px']) > 0:
        ready_rows.append((i, row))

print(f'Rows with guard=READY and ball detected: {len(ready_rows)}')

# Basic tracking metrics
errors = [abs(int(r['error_px'])) for _, r in ready_rows]
if errors:
    errors_sorted = sorted(errors)
    n = len(errors_sorted)
    print(f'\n|error| stats: p50={errors_sorted[n//2]:.0f}px, '
          f'p90={errors_sorted[int(n*0.9)]:.0f}px, '
          f'max={max(errors):.0f}px')

# Servo stability
servo_targets = [int(r['servo_target_us']) for _, r in ready_rows]
servo_currents = [int(r['servo_current_us']) for _, r in ready_rows]

# Servo step-to-step changes
servo_steps = [abs(servo_targets[i] - servo_targets[i-1])
               for i in range(1, len(servo_targets))]
if servo_steps:
    ss = sorted(servo_steps)
    print(f'\nServo target step |diff|: p50={ss[len(ss)//2]:.0f}us, '
          f'p90={ss[int(len(ss)*0.9)]:.0f}us, max={max(servo_steps):.0f}us')

# Servo range
print(f'Servo target range: {min(servo_targets)} - {max(servo_targets)} us '
      f'(swing={max(servo_targets)-min(servo_targets)} us)')

# Check ball_x over time to see if tracking
ball_x_vals = [int(r['ball_x_px']) for _, r in ready_rows]
# Target is around 442 (center)
target = 442
in_position = sum(1 for x in ball_x_vals if abs(x - target) <= 25)  # 1cm
print(f'\nBall within 1cm (±25px) of target 442: '
      f'{in_position}/{len(ball_x_vals)} ({100*in_position/len(ball_x_vals):.1f}%)')

# Show the motion_state transitions - when does car enter different modes
print('\n--- Motion state transitions ---')
prev_ms = None
for i, row in enumerate(rows):
    ms = row['motion_state']
    if ms != prev_ms:
        ms_names = {'0':'STOP','1':'STARTING','2':'STRAIGHT',
                    '3':'TURN_L','4':'TURN_R','5':'STOPPING'}
        print(f'  idx={i:>4} tick={row["stm32_tick_ms"]:>6} '
              f'motion: {prev_ms} -> {ms} ({ms_names.get(ms, "?")})  '
              f'ball_x={row["ball_x_px"]:>5} guard={row["guard_state"]} '
              f'servo={row["servo_target_us"]:>5}')
        prev_ms = ms

# Check the worst oscillation section - pick a segment and show continuity
print('\n--- Mid-run sample: rows where servo oscillates most ---')
# Find region with largest servo variance over a 50-row window
max_var = 0
max_var_start = 0
for i in range(len(servo_targets) - 50):
    window = servo_targets[i:i+50]
    var = sum((v - sum(window)/len(window))**2 for v in window) / len(window)
    if var > max_var:
        max_var = var
        max_var_start = i

start_idx = max(0, max_var_start - 5)
for j in range(start_idx, min(start_idx + 40, len(ready_rows))):
    orig_idx, r = ready_rows[j]
    marker = ' <--' if j == max_var_start else ''
    print(f'  idx={orig_idx:>4} tick={r["stm32_tick_ms"]:>6} '
          f'ball_x={r["ball_x_px"]:>5} err={r["error_px"]:>5} '
          f'servo_tgt={r["servo_target_us"]:>5} servo_cur={r["servo_current_us"]:>5} '
          f'acc={r["acc_track_mg"]:>5} p={r["p_term_us"]:>6} '
          f'i={r["i_term_us"]:>6} d={r["d_term_us"]:>6} '
          f'offset={r["control_offset_us"]:>6} m_state={r["motion_state"]}{marker}')

# Compute what AF and YF would be with new code
print('\n--- Feedforward contribution check (simulated with new code) ---')
# Simulate the EMA + slew on this data
alpha = 0.30
kaf = 2.8
ky = -0.9
af_lim = 70.0
yf_lim = 30.0
af_slew = 20.0
yf_slew = 5.0

acc_filt = 0.0
filt_init = False
af_prev = 0.0
yf_prev = 0.0
total_af = 0.0
total_yf = 0.0
af_count = 0
yf_count = 0

for i, row in enumerate(rows):
    if row['guard_state'] != '5':
        continue
    ms = int(row['motion_state'])
    mlv = int(row['motion_link_valid'])
    flags = int(row['motion_flags'])
    acc = float(row['acc_track_mg'])
    yaw = float(row['yaw_rate_dps'])

    # AF calculation (same conditions as main.c)
    if (mlv == 1 and (flags & 0x01) and not (flags & 0x80) and
        1 <= ms <= 5):
        if not filt_init:
            acc_filt = acc
            filt_init = True
        acc_filt += alpha * (acc - acc_filt)

        af_raw = kaf * acc_filt
        if af_raw > af_lim:
            af_raw = af_lim
        elif af_raw < -af_lim:
            af_raw = -af_lim

        # Slew limit
        af_step = af_raw - af_prev
        if af_step > af_slew:
            af_raw = af_prev + af_slew
        elif af_step < -af_slew:
            af_raw = af_prev - af_slew
        af_prev = af_raw
        total_af += abs(af_raw)
        af_count += 1
    else:
        # AF goes to 0 when conditions not met
        af_step = 0.0 - af_prev
        if af_step > af_slew:
            af_prev += af_slew
        elif af_step < -af_slew:
            af_prev -= af_slew
        else:
            af_prev = 0.0

    # YF calculation
    if (mlv == 1 and (flags & 0x01) and not (flags & 0x80) and ms == 4):
        ls = float(row['left_speed'])
        rs = float(row['right_speed'])
        ws = abs(ls + rs) * 0.5
        vref = 200.0
        ss = ws / vref
        if ss > 1.0:
            ss = 1.0
        yf_raw = ky * yaw * ss
        if yf_raw > yf_lim:
            yf_raw = yf_lim
        elif yf_raw < -yf_lim:
            yf_raw = -yf_lim

        yf_step = yf_raw - yf_prev
        if yf_step > yf_slew:
            yf_raw = yf_prev + yf_slew
        elif yf_step < -yf_slew:
            yf_raw = yf_prev - yf_slew
        yf_prev = yf_raw
        total_yf += abs(yf_raw)
        yf_count += 1
    else:
        yf_step = 0.0 - yf_prev
        if yf_step > yf_slew:
            yf_prev += yf_slew
        elif yf_step < -yf_slew:
            yf_prev -= yf_slew
        else:
            yf_prev = 0.0

print(f'AF: mean_abs={total_af/af_count:.1f} us (n={af_count})')
print(f'YF: mean_abs={total_yf/yf_count:.1f} us (n={yf_count})')

# Now do the OLD code (no EMA, no slew) for comparison
acc_filt_old = 0.0
filt_init_old = False
af_prev_old = 0.0
yf_prev_old = 0.0
total_af_old = 0.0
total_yf_old = 0.0

for i, row in enumerate(rows):
    if row['guard_state'] != '5':
        continue
    ms = int(row['motion_state'])
    mlv = int(row['motion_link_valid'])
    flags = int(row['motion_flags'])
    acc = float(row['acc_track_mg'])
    yaw = float(row['yaw_rate_dps'])

    if (mlv == 1 and (flags & 0x01) and not (flags & 0x80) and
        1 <= ms <= 5):
        af_raw_old = kaf * acc  # NO EMA!
        if af_raw_old > af_lim:
            af_raw_old = af_lim
        elif af_raw_old < -af_lim:
            af_raw_old = -af_lim
        # NO slew limit
        total_af_old += abs(af_raw_old)

    if (mlv == 1 and (flags & 0x01) and not (flags & 0x80) and ms == 4):
        ls = float(row['left_speed'])
        rs = float(row['right_speed'])
        ws = abs(ls + rs) * 0.5
        ss = ws / 200.0
        if ss > 1.0:
            ss = 1.0
        yf_raw_old = ky * yaw * ss
        if yf_raw_old > yf_lim:
            yf_raw_old = yf_lim
        elif yf_raw_old < -yf_lim:
            yf_raw_old = -yf_lim
        total_yf_old += abs(yf_raw_old)

print(f'\nOLD code AF: mean_abs={total_af_old/af_count:.1f} us')
print(f'OLD code YF: mean_abs={total_yf_old/yf_count:.1f} us')

# Show AF step-to-step with old code (no EMA, no slew)
af_old_steps = []
af_prev_val = 0.0
for i, row in enumerate(rows):
    if row['guard_state'] != '5':
        continue
    ms = int(row['motion_state'])
    mlv = int(row['motion_link_valid'])
    flags = int(row['motion_flags'])
    acc = float(row['acc_track_mg'])

    if (mlv == 1 and (flags & 0x01) and not (flags & 0x80) and
        1 <= ms <= 5):
        af_val = kaf * acc
        if af_val > af_lim:
            af_val = af_lim
        elif af_val < -af_lim:
            af_val = -af_lim
        af_old_steps.append(abs(af_val - af_prev_val))
        af_prev_val = af_val
    else:
        af_old_steps.append(abs(0.0 - af_prev_val))
        af_prev_val = 0.0

af_old_steps_sorted = sorted(af_old_steps)
n_steps = len(af_old_steps_sorted)
print(f'\nOLD code AF step |diff|: p50={af_old_steps_sorted[n_steps//2]:.1f}us, '
      f'p90={af_old_steps_sorted[int(n_steps*0.9)]:.1f}us, '
      f'max={max(af_old_steps):.1f}us')
