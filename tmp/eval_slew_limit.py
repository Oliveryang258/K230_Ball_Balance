"""Simulate EMA + slew limit on actual CSV data to evaluate AF_MAX_STEP_US."""
import csv
import math

acc_vals = []
motion_states = []

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            ms = int(row['motion_state'])
            acc = float(row['acc_track_mg'])
            if 1 <= ms <= 5:
                acc_vals.append(acc)
                motion_states.append(ms)
        except (ValueError, KeyError):
            pass

n = len(acc_vals)
print(f'Active motion samples: {n}')

# Parameters
alpha = 0.30
kaf = 2.8   # us/mg
af_lim = 70.0  # us

# Step 1: Apply EMA filter to acc_track_mg
acc_filt = [0.0] * n
acc_filt[0] = acc_vals[0]
for i in range(1, n):
    acc_filt[i] = acc_filt[i-1] + alpha * (acc_vals[i] - acc_filt[i-1])

# Step 2: Convert to AF (before slew limit)
af_raw = [kaf * a for a in acc_filt]
# Apply absolute clamp
af_clamped = [max(-af_lim, min(af_lim, a)) for a in af_raw]

# Step 3: Apply slew rate limits for various thresholds
def apply_slew(values, max_step):
    result = [0.0] * len(values)
    result[0] = values[0]
    clipped_count = 0
    total_clip_amount = 0.0
    for i in range(1, len(values)):
        step = values[i] - result[i-1]
        if step > max_step:
            result[i] = result[i-1] + max_step
            clipped_count += 1
            total_clip_amount += step - max_step
        elif step < -max_step:
            result[i] = result[i-1] - max_step
            clipped_count += 1
            total_clip_amount += abs(step) - max_step
        else:
            result[i] = values[i]
    return result, clipped_count, total_clip_amount

# Test different slew limits
for slew in [10, 15, 20, 25, 30, 40, 999]:
    af_slew, clipped, clip_sum = apply_slew(af_clamped, slew)
    pct = 100.0 * clipped / n
    avg_clip = clip_sum / clipped if clipped > 0 else 0

    # Max absolute difference between slewed and unslewed (error introduced)
    max_err = max(abs(af_slew[i] - af_clamped[i]) for i in range(n))

    print(f'slew={slew:>4} us: clipped {clipped:>4}/{n} ({pct:>5.1f}%), '
          f'avg_clip={avg_clip:>5.1f} us, max_err={max_err:>5.1f} us')

# Show what legitimate large transients look like (not noise)
print('\n--- Largest AF transients after EMA (top 20) ---')
af_steps = [(i, abs(af_clamped[i] - af_clamped[i-1]))
            for i in range(1, n)]
af_steps.sort(key=lambda x: -x[1])
for rank, (idx, step) in enumerate(af_steps[:20]):
    ms = motion_states[idx]
    ms_names = {0:'STOP',1:'STARTING',2:'STRAIGHT',3:'TURN_L',4:'TURN_R',5:'STOPPING'}
    ms_name = ms_names.get(ms, str(ms))
    print(f'  #{rank+1}: idx={idx:>4} step={step:>5.1f} us  '
          f'acc_raw={acc_vals[idx]:>6.1f} mg  acc_filt={acc_filt[idx]:>6.1f} mg  '
          f'state={ms_name}')

# Context: show the 5 samples around each large step to distinguish noise from real transients
print('\n--- Context around largest steps (real transient vs noise) ---')
for rank, (idx, step) in enumerate(af_steps[:5]):
    start = max(0, idx - 2)
    end = min(n, idx + 3)
    print(f'\nRank #{rank+1}: idx={idx}, step={step:.1f} us')
    for j in range(start, end):
        marker = ' <--' if j == idx else ''
        ms = motion_states[j]
        ms_names = {0:'STOP',1:'STARTING',2:'STRAIGHT',3:'TURN_L',
                    4:'TURN_R',5:'STOPPING'}
        print(f'  [{j:>4}] raw={acc_vals[j]:>6.1f} mg  filt={acc_filt[j]:>6.1f} mg  '
              f'af={af_clamped[j]:>6.1f} us  state={ms_names.get(ms,str(ms))}{marker}')
