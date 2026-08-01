"""Quantify ball loss frequency during each motion phase."""
import csv

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    rows = list(csv.DictReader(f))

# Find all ball loss events (ball_x flips between 0/-1 and positive)
ms_names = {'0': 'STOP', '1': 'STARTING', '2': 'STRAIGHT',
            '3': 'TURN_L', '4': 'TURN_R', '5': 'STOPPING'}

# Track ball loss events
loss_events = []
prev_has_ball = False
for i, row in enumerate(rows):
    ball_x = int(row['ball_x_px'])
    has_ball = ball_x > 0
    ms = row['motion_state']

    if prev_has_ball and not has_ball:
        # Ball just lost
        loss_events.append({
            'idx': i,
            'tick': int(row['stm32_tick_ms']),
            'motion_state': ms,
            'ball_x': ball_x,
        })
    prev_has_ball = has_ball

print(f'Total ball loss events: {len(loss_events)}')

# Group by motion state
from collections import Counter
ms_losses = Counter(e['motion_state'] for e in loss_events)
print('\nBall loss events by motion state:')
for ms in sorted(ms_losses.keys()):
    print(f'  {ms_names.get(ms, ms):>10}: {ms_losses[ms]}')

# Show all loss events
print('\n--- All ball loss events ---')
for e in loss_events:
    ms = ms_names.get(e['motion_state'], e['motion_state'])
    print(f'  idx={e["idx"]:>4} tick={e["tick"]:>6} ms  m_state={ms}  ball_x={e["ball_x"]}')

# Time between consecutive ball losses during TURN_R
turn_losses = [e for e in loss_events if e['motion_state'] == '4']
print(f'\n--- TURN_R ball loss intervals ---')
for i in range(1, len(turn_losses)):
    dt = turn_losses[i]['tick'] - turn_losses[i-1]['tick']
    print(f'  loss#{i}: {turn_losses[i-1]["tick"]} -> {turn_losses[i]["tick"]} ms, '
          f'gap={dt} ms  (idx {turn_losses[i-1]["idx"]} -> {turn_losses[i]["idx"]})')

# For TURN_R, show loss duration (how many consecutive rows without ball)
print('\n--- TURN_R loss durations ---')
for e in turn_losses:
    # Count consecutive rows with ball_x <= 0
    loss_start = e['idx']
    loss_end = loss_start
    for j in range(loss_start, min(loss_start + 50, len(rows))):
        if int(rows[j]['ball_x_px']) > 0:
            break
        loss_end = j
    duration = (loss_end - loss_start + 1) * 20  # ms
    print(f'  loss at idx={e["idx"]} tick={e["tick"]}: '
          f'duration={duration} ms ({loss_end - loss_start + 1} rows)')

# TURN_R total duration vs loss time
turn_start_tick = None
turn_end_tick = None
for i, row in enumerate(rows):
    if row['motion_state'] == '4':
        if turn_start_tick is None:
            turn_start_tick = int(row['stm32_tick_ms'])
        turn_end_tick = int(row['stm32_tick_ms'])

if turn_start_tick and turn_end_tick:
    total_turn_ms = turn_end_tick - turn_start_tick
    # Sum all loss durations during turn
    total_loss_ms = 0
    for e in turn_losses:
        loss_start = e['idx']
        loss_end = loss_start
        for j in range(loss_start, min(loss_start + 50, len(rows))):
            if int(rows[j]['ball_x_px']) > 0:
                break
            loss_end = j
        total_loss_ms += (loss_end - loss_start + 1) * 20

    print(f'\nTURN_R total time: {total_turn_ms} ms')
    print(f'TURN_R total ball-loss time: {total_loss_ms} ms '
          f'({100*total_loss_ms/total_turn_ms:.1f}%)')
    print(f'Number of loss events in TURN_R: {len(turn_losses)}')
