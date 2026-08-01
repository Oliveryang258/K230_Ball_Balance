import csv
rows = list(csv.DictReader(open(r'C:\Users\32142\Desktop\ball_run.csv')))

losses = []
for i in range(1, len(rows)):
    if rows[i-1]['guard_state']=='5' and rows[i]['guard_state']=='1':
        tick = int(rows[i]['stm32_tick_ms'])
        ms = rows[i]['motion_state']
        i_before = 0.0
        for j in range(i-1, max(0, i-5), -1):
            v = float(rows[j]['i_term_us'])
            if abs(v) > 0:
                i_before = v
                break
        losses.append((i, tick, ms, i_before))

ms_names = {'0':'STOP','1':'STARTING','2':'STRAIGHT','3':'TURN_L','4':'TURN_R','5':'STOPPING'}
print(f'Total ball loss events (guard 5->1): {len(losses)}')

for ms in ['2','4']:
    ms_losses = [l for l in losses if l[2]==ms]
    print(f'  {ms_names[ms]}: {len(ms_losses)}')

# All losses
print('\n--- All losses ---')
for idx, tick, ms, i_before in losses:
    gap = ''
    print(f'  idx={idx:>4} tick={tick:>6}  m_state={ms_names.get(ms,ms)}  i_before={i_before:.0f} us')

# Clusters: consecutive losses within 2 seconds
print('\n--- Clusters (>=3 losses within 2s) ---')
ticks = [l[1] for l in losses]
for t in ticks:
    cluster = [t2 for t2 in ticks if t <= t2 < t+2000]
    if len(cluster) >= 3:
        print(f'  from tick={t}: {len(cluster)} losses in 2s window: {cluster}')
        break  # just show first cluster
