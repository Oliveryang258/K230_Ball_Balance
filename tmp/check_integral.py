"""Check i_term during violation segments."""
import csv

with open(r'C:\Users\32142\Desktop\ball_run.csv', 'r') as f:
    rows = list(csv.DictReader(f))

# TURN_R violation segment (rows 1769-1895)
print('=== TURN_R violation segment (rows 1769-1895): every 5th sample ===')
for i in range(1769, min(1896, len(rows)), 5):
    r = rows[i]
    print(f'  idx={i:>4} tick={r["stm32_tick_ms"]:>6} err={r["error_px"]:>5} '
          f'vel={r["velocity_px_s"]:>7} i_term={r["i_term_us"]:>7} '
          f'p_term={r["p_term_us"]:>7} servo={r["servo_target_us"]:>5} '
          f'acc={r["acc_track_mg"]:>5}')

# STRAIGHT violation at rows 1520-1572
print('\n=== STRAIGHT violation segment (rows 1520-1572): every 3rd ===')
for i in range(1520, min(1573, len(rows)), 3):
    r = rows[i]
    print(f'  idx={i:>4} tick={r["stm32_tick_ms"]:>6} err={r["error_px"]:>5} '
          f'vel={r["velocity_px_s"]:>7} i_term={r["i_term_us"]:>7} '
          f'p_term={r["p_term_us"]:>7} servo={r["servo_target_us"]:>5} '
          f'acc={r["acc_track_mg"]:>5}')

# Check i_term evolution across ENTIRE run (every 50th sample during READY)
print('\n=== i_term evolution across run (every 50th READY sample) ===')
ready_samples = [(i, r) for i, r in enumerate(rows)
                 if r['guard_state'] == '5' and int(r['ball_x_px']) > 0]
for j in range(0, len(ready_samples), 50):
    orig_i, r = ready_samples[j]
    print(f'  idx={orig_i:>4} tick={r["stm32_tick_ms"]:>6} err={r["error_px"]:>5} '
          f'vel={r["velocity_px_s"]:>7} i_term={r["i_term_us"]:>7} '
          f'm_state={r["motion_state"]}')

# When does i_term become non-zero?
print('\n=== First non-zero i_term ===')
for i, row in enumerate(rows):
    if float(row['i_term_us']) != 0:
        print(f'  idx={i:>4} tick={row["stm32_tick_ms"]:>6} '
              f'i_term={row["i_term_us"]:>7} err={row["error_px"]:>5} '
              f'vel={row["velocity_px_s"]:>7} m_state={row["motion_state"]}')
        # show next 30 rows
        for j in range(i+1, min(i+31, len(rows))):
            r2 = rows[j]
            print(f'  idx={j:>4} tick={r2["stm32_tick_ms"]:>6} '
                  f'i_term={r2["i_term_us"]:>7} err={r2["error_px"]:>5} '
                  f'vel={r2["velocity_px_s"]:>7} m_state={r2["motion_state"]}')
        break

# Check what the integral limits are in config
print('\n=== Integral config ===')
print('INTEGRAL_ERROR_MIN_PX = 8')
print('INTEGRAL_ERROR_MAX_PX = 60')
print('INTEGRAL_SPEED_MAX_PX_S = 8.0')
print('INTEGRAL_CONFIRM_MS = 150')
print('INTEGRAL_MAX_US = 80.0')
