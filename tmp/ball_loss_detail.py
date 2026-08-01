"""Cross-reference ball loss from both ball_x and guard_state transitions."""
import csv

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    rows = list(csv.DictReader(f))

ms_names = {'0': 'STOP', '1': 'STARTING', '2': 'STRAIGHT',
            '3': 'TURN_L', '4': 'TURN_R', '5': 'STOPPING'}

# Find ALL guard transitions to BALL_LOST
print('=== All guard transitions 5->1 (READY->BALL_LOST) with context ===')
for i in range(1, len(rows)):
    prev_gs = rows[i-1]['guard_state']
    curr_gs = rows[i]['guard_state']
    if prev_gs == '5' and curr_gs == '1':
        r = rows[i]
        # Show surrounding: 2 before, the loss row, 2 after
        for j in range(max(0, i-2), min(len(rows), i+3)):
            r2 = rows[j]
            marker = ' <-- LOST' if j == i else ''
            err = int(r2['error_px'])
            i_term = float(r2['i_term_us'])
            print(f'  idx={j:>4} tick={r2["stm32_tick_ms"]:>6} '
                  f'guard={r2["guard_state"]} ball_x={r2["ball_x_px"]:>5} '
                  f'err={err:>5} i_term={i_term:>6.0f} '
                  f'm_state={ms_names.get(r2["motion_state"], "?")}{marker}')
        print()

# Count losses per TURN_R segment
print('=== TURN_R segment summary ===')
# Segment boundaries from motion transitions
segments = [
    ('TURN_R #1 (blip)', 18082, 18102),
    ('TURN_R #2 (blip)', 21382, 21422),
    ('TURN_R #3 (~6.8s)', 22562, 29362),
    ('TURN_R #4 (~8.0s)', 35242, 43222),
]

for name, t_start, t_end in segments:
    duration = t_end - t_start
    losses = []
    for i in range(1, len(rows)):
        tick = int(rows[i]['stm32_tick_ms'])
        if t_start <= tick <= t_end:
            prev_gs = rows[i-1]['guard_state']
            curr_gs = rows[i]['guard_state']
            if prev_gs == '5' and curr_gs == '1':
                losses.append((i, tick))
    if duration > 0:
        rate = len(losses) / (duration / 1000.0)  # per second
        print(f'  {name}: {len(losses)} losses in {duration}ms '
              f'({rate:.1f}/s, avg gap={duration/max(len(losses),1):.0f}ms)')
    else:
        print(f'  {name}: {len(losses)} losses in {duration}ms')

# Count I-term resets during TURN_R (all causes)
print('\n=== I-term resets during TURN_R ===')
prev_i = 0.0
reset_count = 0
for i, row in enumerate(rows):
    ms = row['motion_state']
    if ms != '4':
        continue
    i_term = float(row['i_term_us'])
    if prev_i > 10 and i_term < 3:
        reset_count += 1
        err = int(row['error_px'])
        ball_x = int(row['ball_x_px'])
        print(f'  I reset #{reset_count}: idx={i} tick={row["stm32_tick_ms"]} '
              f'i_term: {prev_i:.0f}->{i_term:.0f}  err={err}  '
              f'ball_x={ball_x}  guard={row["guard_state"]}')
    prev_i = i_term

print(f'\nTotal I-term resets during TURN_R: {reset_count}')
