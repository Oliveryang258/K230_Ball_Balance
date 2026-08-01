"""Deep-dive analysis of the new ball_run.csv to find what went wrong."""
import csv

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print(f'Total rows: {len(rows)}')

# Separate guard_state and motion_state distributions
guard_dist = {}
motion_dist = {}
ctrl_flags_dist = {}

for row in rows:
    gs = row['guard_state']
    guard_dist[gs] = guard_dist.get(gs, 0) + 1
    ms = row['motion_state']
    motion_dist[ms] = motion_dist.get(ms, 0) + 1
    cf = row['control_flags']
    ctrl_flags_dist[cf] = ctrl_flags_dist.get(cf, 0) + 1

print('\nguard_state distribution:')
for k in sorted(guard_dist.keys(), key=lambda x: int(x)):
    print(f'  {k}: {guard_dist[k]}')

print('\nmotion_state distribution:')
for k in sorted(motion_dist.keys(), key=lambda x: int(x)):
    print(f'  {k}: {motion_dist[k]}')

print('\ncontrol_flags distribution:')
for k in sorted(ctrl_flags_dist.keys(), key=lambda x: int(x)):
    print(f'  {k}: {ctrl_flags_dist[k]}')

# Find when guard_state changes
print('\n--- guard_state transitions ---')
prev_gs = None
for i, row in enumerate(rows):
    gs = row['guard_state']
    if gs != prev_gs:
        print(f'  idx={i:>4} tick={row["stm32_tick_ms"]:>6} '
              f'guard: {prev_gs} -> {gs}  '
              f'ball_x={row["ball_x_px"]:>5} err={row["error_px"]:>5} '
              f'servo_tgt={row["servo_target_us"]:>5} '
              f'm_state={row["motion_state"]} ctrl_flags={row["control_flags"]}')
        prev_gs = gs

# Find when control actually started (first non-zero ball_x)
print('\n--- First ball detection ---')
for i, row in enumerate(rows):
    if int(row['ball_x_px']) > 0:
        print(f'  idx={i:>4} tick={row["stm32_tick_ms"]:>6} '
              f'ball_x={row["ball_x_px"]:>5} err={row["error_px"]:>5} '
              f'servo_tgt={row["servo_target_us"]:>5} '
              f'guard={row["guard_state"]} m_state={row["motion_state"]} '
              f'mlv={row["motion_link_valid"]}')
        # Show next 20 rows
        for j in range(i+1, min(i+21, len(rows))):
            r = rows[j]
            print(f'  idx={j:>4} tick={r["stm32_tick_ms"]:>6} '
                  f'ball_x={r["ball_x_px"]:>5} err={r["error_px"]:>5} '
                  f'servo_tgt={r["servo_target_us"]:>5} servo_cur={r["servo_current_us"]:>5} '
                  f'guard={r["guard_state"]} m_state={r["motion_state"]} '
                  f'acc={r["acc_track_mg"]:>5} p={r["p_term_us"]:>6} '
                  f'i={r["i_term_us"]:>6} d={r["d_term_us"]:>6} '
                  f'offset={r["control_offset_us"]:>6}')
        break

# Find sections where servo jumps a lot
print('\n--- Large servo target jumps (>30 us step) ---')
count = 0
for i in range(1, len(rows)):
    prev_tgt = int(rows[i-1]['servo_target_us'])
    curr_tgt = int(rows[i]['servo_target_us'])
    jump = abs(curr_tgt - prev_tgt)
    if jump > 30:
        count += 1
        if count <= 15:
            r = rows[i]
            print(f'  idx={i:>4} tick={r["stm32_tick_ms"]:>6} '
                  f'jump={jump:>4} us  '
                  f'servo_tgt={curr_tgt:>5} servo_cur={r["servo_current_us"]:>5} '
                  f'err={r["error_px"]:>5} guard={r["guard_state"]} '
                  f'p={r["p_term_us"]:>6} i={r["i_term_us"]:>6} '
                  f'd={r["d_term_us"]:>6} offset={r["control_offset_us"]:>6} '
                  f'acc={r["acc_track_mg"]:>5}')

print(f'  Total large jumps (>30 us): {count}/{len(rows)-1}')

# Show the guard=5 (invalid dt) section in detail
print('\n--- First 30 rows of guard=5 ---')
guard5_count = 0
for i, row in enumerate(rows):
    if row['guard_state'] == '5':
        guard5_count += 1
        if guard5_count <= 30:
            print(f'  idx={i:>4} tick={row["stm32_tick_ms"]:>6} '
                  f'ball_x={row["ball_x_px"]:>5} err={row["error_px"]:>5} '
                  f'servo_tgt={row["servo_target_us"]:>5} servo_cur={row["servo_current_us"]:>5} '
                  f'guard=5 m_state={row["motion_state"]} '
                  f'p={row["p_term_us"]:>6} i={row["i_term_us"]:>6} '
                  f'offset={row["control_offset_us"]:>6} vel={row["velocity_px_s"]:>6}')

# Check if guard=5 means we're not controlling
print(f'\nTotal guard=5 rows: {guard5_count}')
