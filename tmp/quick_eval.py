"""Quick overview of the new ball_run.csv after code changes."""
import csv

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print(f'Total rows: {len(rows)}')

# Check key columns for anomalies
states = {}
acc_vals = []
yaw_vals = []
af_vals = []  # no direct column, but we have acc_track_mg
guard_states = {}
motion_link_valid_count = 0
has_ball = 0

for row in rows:
    gs = row.get('guard_state', '?')
    states[gs] = states.get(gs, 0) + 1

    ms = row.get('motion_state', '?')
    if ms not in states:
        states[ms] = 0
    states[ms] = states.get(ms, 0) + 1

    mlv = row.get('motion_link_valid', '0')
    if mlv == '1':
        motion_link_valid_count += 1

    bx = int(row.get('ball_x_px', 0))
    if bx > 0:
        has_ball += 1

    try:
        acc_vals.append(float(row['acc_track_mg']))
        yaw_vals.append(float(row['yaw_rate_dps']))
    except:
        pass

print(f'\nNon-zero ball_x samples: {has_ball}/{len(rows)}')
print(f'motion_link_valid=1 samples: {motion_link_valid_count}/{len(rows)}')
print(f'\nguard_state distribution:')
for k in sorted(states.keys(), key=lambda x: int(x) if x.isdigit() else 999):
    print(f'  {k}: {states[k]}')

if acc_vals:
    non_zero_acc = [a for a in acc_vals if a != 0]
    print(f'\nNon-zero acc_track_mg samples: {len(non_zero_acc)}/{len(acc_vals)}')
    if non_zero_acc:
        print(f'  range: {min(non_zero_acc):.1f} to {max(non_zero_acc):.1f}')

# Show transitions: first and last 20 rows, and state transitions
print('\n--- First 30 rows ---')
for i, row in enumerate(rows[:30]):
    print(f'  [{i:>4}] tick={row["stm32_tick_ms"]:>6} '
          f'ball_x={row["ball_x_px"]:>5} err={row["error_px"]:>5} '
          f'servo_tgt={row["servo_target_us"]:>5} servo_cur={row["servo_current_us"]:>5} '
          f'guard={row["guard_state"]} m_state={row["motion_state"]} '
          f'mlv={row["motion_link_valid"]} acc={row["acc_track_mg"]:>5} '
          f'yaw={row["yaw_rate_dps"]:>5} ctrl_flags={row["control_flags"]}')

print('\n--- Last 30 rows ---')
for i, row in enumerate(rows[-30:]):
    idx = len(rows) - 30 + i
    print(f'  [{idx:>4}] tick={row["stm32_tick_ms"]:>6} '
          f'ball_x={row["ball_x_px"]:>5} err={row["error_px"]:>5} '
          f'servo_tgt={row["servo_target_us"]:>5} servo_cur={row["servo_current_us"]:>5} '
          f'guard={row["guard_state"]} m_state={row["motion_state"]} '
          f'mlv={row["motion_link_valid"]} acc={row["acc_track_mg"]:>5} '
          f'yaw={row["yaw_rate_dps"]:>5} ctrl_flags={row["control_flags"]}')
