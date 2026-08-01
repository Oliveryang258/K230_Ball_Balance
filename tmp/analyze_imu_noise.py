"""Analyze IMU noise characteristics from ball_run.csv."""
import csv
import math
import sys

acc_vals = []
yaw_vals = []
motion_states = []

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            ms = int(row['motion_state'])
            acc = float(row['acc_track_mg'])
            yaw = float(row['yaw_rate_dps'])
            if ms >= 1 and ms <= 5:
                acc_vals.append(acc)
                yaw_vals.append(yaw)
                motion_states.append(ms)
        except (ValueError, KeyError):
            pass

if not acc_vals:
    print('No active motion data found')
    sys.exit(0)

n = len(acc_vals)
print(f'Active motion samples: {n}')

# Basic stats
acc_mean = sum(acc_vals) / n
yaw_mean = sum(yaw_vals) / n
acc_std = math.sqrt(sum((v - acc_mean) ** 2 for v in acc_vals) / n)
yaw_std = math.sqrt(sum((v - yaw_mean) ** 2 for v in yaw_vals) / n)
acc_abs = [abs(v) for v in acc_vals]
yaw_abs = [abs(v) for v in yaw_vals]
print(f'acc_track_mg: mean={acc_mean:.1f}, std={acc_std:.1f}, '
      f'max_abs={max(acc_abs):.1f}')
print(f'yaw_rate_dps: mean={yaw_mean:.2f}, std={yaw_std:.2f}, '
      f'max_abs={max(yaw_abs):.2f}')

# Step-to-step differences
acc_diffs = [abs(acc_vals[i] - acc_vals[i-1]) for i in range(1, n)]
yaw_diffs = [abs(yaw_vals[i] - yaw_vals[i-1]) for i in range(1, n)]
acc_ds = sorted(acc_diffs)
yaw_ds = sorted(yaw_diffs)

print(f'\nStep-to-step |diff| (20ms steps):')
print(f'acc_track_mg: p50={acc_ds[len(acc_ds)//2]:.1f}, '
      f'p90={acc_ds[int(len(acc_ds)*0.9)]:.1f}, '
      f'p99={acc_ds[int(len(acc_ds)*0.99)]:.1f}, '
      f'max={max(acc_diffs):.1f}')
print(f'yaw_rate_dps: p50={yaw_ds[len(yaw_ds)//2]:.2f}, '
      f'p90={yaw_ds[int(len(yaw_ds)*0.9)]:.2f}, '
      f'p99={yaw_ds[int(len(yaw_ds)*0.99)]:.2f}, '
      f'max={max(yaw_diffs):.2f}')

# Isolated spikes: diff > 3x p90 and both neighbors < p90
p90_acc = acc_ds[int(len(acc_ds) * 0.9)]
spike_count = 0
for i in range(2, n - 1):
    if (acc_diffs[i] > 3 * p90_acc and
            acc_diffs[i-1] < p90_acc and
            acc_diffs[i+1] < p90_acc):
        spike_count += 1
print(f'\nIsolated acc spikes (diff > 3*p90={3*p90_acc:.1f}, '
      f'neighbors < p90): {spike_count}')

# Yaw during TURN_RIGHT (state 4)
yaw_turn = [yaw_vals[i] for i, ms in enumerate(motion_states) if ms == 4]
if yaw_turn:
    tm = sum(yaw_turn) / len(yaw_turn)
    ts = math.sqrt(sum((v - tm) ** 2 for v in yaw_turn) / len(yaw_turn))
    print(f'\nYaw during TURN_RIGHT (n={len(yaw_turn)}): '
          f'mean={tm:.2f}, std={ts:.2f}')

# Acc during STRAIGHT (state 2) - reveals noise floor
acc_str = [acc_vals[i] for i, ms in enumerate(motion_states) if ms == 2]
if acc_str:
    am = sum(acc_str) / len(acc_str)
    as_std = math.sqrt(sum((v - am) ** 2 for v in acc_str) / len(acc_str))
    print(f'\nAcc during STRAIGHT (n={len(acc_str)}): '
          f'mean={am:.1f}, std={as_std:.1f}')
    sd = sorted([abs(acc_str[i] - acc_str[i-1])
                 for i in range(1, len(acc_str))])
    print(f'  Straight acc |diff|: p50={sd[len(sd)//2]:.1f}, '
          f'p90={sd[int(len(sd)*0.9)]:.1f}')
