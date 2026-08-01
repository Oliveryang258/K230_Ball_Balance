# Score-aligned report blueprint

## Core narrative

The report should demonstrate two coordinated control systems:

1. The vehicle uses infrared sensing to estimate line deviation and controls differential wheel motion, start/stop detection, and timing.
2. The camera measures ball position; the rod actuator closes the ball-position loop. Vehicle acceleration and turning act as disturbances, so the report must explain disturbance handling rather than presenting the rod as a stationary bench system.

## Recommended structure

### Title and abstract

Use the exact problem title. Keep the abstract factual: architecture, two key control methods, implemented distinguishing features, and measured results. Do not quote thresholds as achievements. Add final numbers only after raw tests exist.

### 1 系统方案与论证

- Overall block diagram: sensing, vehicle controller, ball controller, actuators, display, video transmitter/receiver.
- Vehicle line-following option comparison and selected solution.
- Ball-position sensing and rod-actuation choices.
- Explain why the control camera and wireless video evidence path are separate if separate hardware is used.

Primary rubric target: 方案论证 3 points.

### 2 理论分析与控制方法

#### 2.1 小车循迹控制理论

- Define infrared sensor geometry and line-error calculation.
- Give differential-drive kinematics and steering-control law.
- Explain straight/curve behavior, speed scheduling, start-line recognition, and stopping logic.

#### 2.2 摆杆系统控制理论

- Define coordinates, sign convention, camera pixels-to-centimetres calibration, and target conversion.
- Derive or justify the rolling-ball model. If using the common pure-rolling approximation for a solid sphere on a tilted rail, state its assumptions before using the `5g/7` factor.
- Give the implemented controller, actuator mapping, deadband/anti-windup/stiction compensation, limits, and fail-safe behavior.
- Explain chassis acceleration/turning disturbance and any inertial compensation only if implemented and verified.

Primary rubric target: 理论分析 5 points.

### 3 电路与程序设计

- Power tree and grounding.
- Infrared line sensor, motor driver, start button, ≤2-inch display, timing subsystem.
- K230 camera/vision processing, UART measurement protocol, STM32 ball controller, servo PWM.
- Separate video-transmission sender/receiver/recording path.
- Main vehicle state machine and ball-control dataflow/failsafe flowchart.

Primary rubric target: 电路与程序设计 5 points.

### 4 测试方案、结果与分析

Use one subsection per R1–R6. Each must contain:

- apparatus and setup;
- initial conditions;
- measurement definition;
- raw repeated trials;
- required threshold;
- worst case and summary statistics;
- pass/fail conclusion;
- failure/uncertainty analysis.

Primary rubric target: 测试 4 points.

### 5 结论

Summarize only demonstrated functions, the strongest measured values, limitations, and unfinished evidence. Keep it short.

## Required figures

1. System architecture and signal/power flow.
2. Vehicle geometry and infrared sensor placement.
3. Line-error and differential-drive control block diagram.
4. Rod, steel ball, coordinate/sign convention, and actuator geometry.
5. Ball-position visual processing pipeline.
6. Ball closed-loop control diagram.
7. Essential control circuit or interface diagram.
8. Vehicle and ball-control software state/flow diagrams.
9. Test setup with reference points and measurement definitions.
10. At least one representative time-series plot: target and measured ball position, with ±1 cm band.

Use fewer figures if page-constrained, but do not remove the evidence chain.

## Requirement-level test tables

### R1 video transmission

Record trial duration, display continuity, dropped/blank intervals, whether full rod is visible, file saved, and replay verified. Preserve the corresponding video filename.

### R2 one-lap stop

Record lap time and signed/absolute stop error for repeated trials. Use the unique marked vehicle point and A reference line.

### R3 O → +5 → −5 cm

Record transition time, settling definition, error at +5 cm, error at −5 cm, maximum absolute error in each evaluation window, and video/log identifier.

### R4 A → B with center hold

Record A–B time, sample count, maximum absolute ball error over the whole interval, RMS error as supplementary evidence, and track-loss events.

### R5 full lap with center hold

Record lap time and ball error over the whole lap. Segment AB, BC, CD, and DA when possible to expose curve disturbances.

### R6 arbitrary target full lap

Use multiple commanded positions unless judging instructions select one. Record command, calibration mapping, lap time, maximum absolute error, RMS error, and invalid-camera frames.

## Evidence ledger

Maintain a table beside the draft:

| Claim ID | Report claim | State | Source path/video/log | Figure/table |
|---|---|---|---|---|

Never delete failed trials. Explain exclusions with a predefined rule.

## Page allocation

Do not fix page counts until the 2026 Liaoning format notice is obtained. When a limit is known, allocate roughly by rubric weight: theory and circuit/program receive the most space; testing must remain complete; generic background and code listings receive the least.
