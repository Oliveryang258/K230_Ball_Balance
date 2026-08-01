"""Check error violations (>15px) and segment by motion phase."""
import csv
from collections import Counter

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    rows = list(csv.DictReader(f))

ready = [(i, r) for i, r in enumerate(rows)
         if r['guard_state'] == '5' and int(r['ball_x_px']) > 0]

violations = [(i, r) for i, r in ready if abs(int(r['error_px'])) > 15]
print(f'READY samples: {len(ready)}')
print(f'|error| > 15px: {len(violations)} ({100*len(violations)/len(ready):.1f}%)')

ms_names = {'0': 'STOP', '1': 'STARTING', '2': 'STRAIGHT',
            '3': 'TURN_L', '4': 'TURN_R', '5': 'STOPPING'}

# Show first 40 violations
print('\n--- First 40 violations ---')
for idx, (orig_i, r) in enumerate(violations[:40]):
    ms = ms_names.get(r['motion_state'], r['motion_state'])
    print(f'  idx={orig_i:>4} tick={r["stm32_tick_ms"]:>6} err={r["error_px"]:>5} '
          f'ball_x={r["ball_x_px"]:>5} servo={r["servo_target_us"]:>5} '
          f'acc={r["acc_track_mg"]:>5} m_state={ms}')

# Violations by motion state
ms_violations = Counter()
for orig_i, r in violations:
    ms_violations[r['motion_state']] += 1
print('\n--- Violations by motion state ---')
for ms in sorted(ms_violations.keys()):
    print(f'  {ms_names.get(ms, ms)}: {ms_violations[ms]}')

# By phase: pre-car-motion (motion_state=0) vs car moving (1-5)
pre_car = sum(1 for _, r in violations if r['motion_state'] == '0')
car_moving = sum(1 for _, r in violations if r['motion_state'] != '0')
print(f'\nViolations during STOP (before car moves): {pre_car}')
print(f'Violations during car motion (1-5): {car_moving}')

# Show continuous violation segments
print('\n--- Violation segments (>=3 consecutive, during car motion) ---')
seg_start = None
for i in range(len(ready)):
    orig_i, r = ready[i]
    err = abs(int(r['error_px']))
    ms = r['motion_state']
    if err > 15 and ms != '0':
        if seg_start is None:
            seg_start = i
    else:
        if seg_start is not None:
            seg_len = i - seg_start
            if seg_len >= 3:
                s_orig, s_r = ready[seg_start]
                e_orig, e_r = ready[i - 1]
                print(f'  rows {s_orig}-{e_orig} ({seg_len} samples, '
                      f'{seg_len*20}ms) '
                      f'err: {s_r["error_px"]}->{e_r["error_px"]}px '
                      f'ball: {s_r["ball_x_px"]}->{e_r["ball_x_px"]} '
                      f'm_state={ms_names.get(ms, ms)}')
            seg_start = None

# Also check the last segment if it ends at the last row
if seg_start is not None:
    seg_len = len(ready) - seg_start
    if seg_len >= 3:
        s_orig, s_r = ready[seg_start]
        e_orig, e_r = ready[-1]
        print(f'  rows {s_orig}-{e_orig} ({seg_len} samples, '
              f'{seg_len*20}ms) '
              f'err: {s_r["error_px"]}->{e_r["error_px"]}px '
              f'ball: {s_r["ball_x_px"]}->{e_r["ball_x_px"]} '
              f'(final segment)')

# Show the car motion phases timeline
print('\n--- Motion phase timeline ---')
prev_ms = None
for i, row in enumerate(rows):
    ms = row['motion_state']
    if ms != prev_ms:
        tick = int(row['stm32_tick_ms'])
        print(f'  tick={tick:>6}ms  {ms_names.get(prev_ms, str(prev_ms))} -> {ms_names.get(ms, ms)}')
        prev_ms = ms

# Error stats during each motion state
print('\n--- Error stats by motion state ---')
for ms in ['0', '1', '2', '4', '5']:
    errs = [abs(int(r['error_px'])) for _, r in ready if r['motion_state'] == ms]
    if errs:
        e_sorted = sorted(errs)
        n_e = len(e_sorted)
        over15 = sum(1 for e in errs if e > 15)
        print(f'  {ms_names.get(ms, ms):>10}: n={n_e:>4}  p50={e_sorted[n_e//2]:>4.0f}px  '
              f'p90={e_sorted[int(n_e*0.9)]:>4.0f}px  max={max(errs):>4.0f}px  '
              f'>15px: {over15}/{n_e} ({100*over15/n_e:.1f}%)')
