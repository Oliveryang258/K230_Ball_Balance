# 2026 辽宁赛区 H 题约束与评分

## Source and authority

Primary source: `辽宁赛区H题_车载平衡滚球运动控制系统.pdf`, 4 A4 pages, created 2026-07-29.

SHA-256: `78FAAED3DCCC0F34AF3879AC53F046FB7D57920A2982FECB798B2C24826E25E3`

This reference is a faithful working extraction, not a replacement for the PDF. Re-check the PDF when wording matters. A separate 2026 Liaoning submission notice is still required for page limit, font, anonymity, binding, and file-naming rules.

## Task

Build a line-following wheeled vehicle carrying a grooved tilting rod, actuator, and approximately 1 cm steel ball. The vehicle runs clockwise on a black closed course while the ball remains stable at a commanded rod position. The rod’s hinged end is at least 5 cm above the vehicle deck.

## Requirements and thresholds

| ID | Required behavior | Threshold | Score |
|---|---|---:|---:|
| R1 | Vehicle-mounted ball-position video transmitter; receiver/display/storage outside track; real-time clear view, complete recording, replay | Functional evidence | 6 |
| R2 | Start at A, clockwise one lap, stop at A, display total time | time ≤ 20 s; stop error ≤ 2 cm | 16 |
| R3 | Vehicle stationary; ball O → +5 cm → −5 cm and stabilize | total time ≤ 5 s; max absolute error at ±5 cm ≤ 1 cm | 13 |
| R4 | Start at A with ball at O; drive through B | A–B time ≤ 8 s; ball absolute error ≤ 1 cm throughout | 20 |
| R5 | Start at A with ball at O; one lap through A | lap time ≤ 30 s; ball absolute error ≤ 1 cm throughout | 20 |
| R6 | Start at A with arbitrary commanded ball position; one lap through A | lap time ≤ 30 s; ball absolute error ≤ 1 cm throughout | 20 |
| R7 | Other features | Demonstrated value | 5 |
| R8 | Design report | See report rubric | 20 |

Do not report only mean error for R3–R6. The wording uses maximum/throughout constraints; report worst-case absolute error and make the measurement window explicit.

## Physical and procedural constraints

- Track: white background, black line width `1.8 ± 0.2 cm`.
- Geometry: straight AB and CD are each `1.5 m`; BC and DA are semicircles of radius `0.5 m`.
- A stop line: length `5 cm`, width `1.8 ± 0.2 cm`, perpendicular to loop; center reference dashed line is `0.1 cm × 30 cm`.
- Vehicle envelope: at most `35 cm × 25 cm`; wheeled, battery powered.
- Mark one unique vehicle test point on the centerline for all start/stop deviation measurements.
- No human intervention or remote control while running.
- Vehicle projection must remain on the track line; complete departure is a failed trial.
- Line sensors must be infrared photoelectric modules; quantity unrestricted.
- Vehicle requires a start button and an observable display no larger than 2 inches. Timing starts with the button.
- Video sender is fixed on the vehicle; camera may be above the groove. Receiver and recording device are sealed with the work after the contest.
- Rod: 25 cm length of 4-fen PPR pipe, outer diameter 2 cm, wall 0.34 cm; groove must stay smooth and unmodified for friction.
- Approximate rod cross-section radii: outer 1 cm, inner 0.65 cm.
- Ball diameter: approximately 1 cm.
- Position scale spacing: 0.1 cm, attached on groove edge, never inside groove.
- Ball position detection inside the groove must use a camera.
- Entire rod must remain within the vehicle envelope.

## Design-report rubric: 20 points

| Rubric item | Required visible content | Score |
|---|---|---:|
| 方案论证 | 小车循迹控制方案 | 3 |
| 理论分析 | 小车循迹控制理论；摆杆系统控制理论 | 5 |
| 电路与程序设计 | 控制电路及程序流程 | 5 |
| 测试方案与测试结果 | 测试数据完整性；测试结果分析 | 4 |
| 设计报告结构及规范性 | 摘要；正文结构；图表规范性 | 3 |

The report score is additive to 100 functional points, for a total of 120.
