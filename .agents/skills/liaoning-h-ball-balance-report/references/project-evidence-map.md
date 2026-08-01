# Current repository evidence map

This file routes evidence; it is not a frozen description. Re-open the cited sources because the worktree is actively changing.

## Verified architecture currently present

### K230 ball vision

- `src/main.py`: camera/display loop, field-tuning overrides, crop/ROI, filtering, visual status, UART output.
- `src/config.py`: camera, circle detection, tracking, filter, target, safety, and UART defaults.
- `src/vision/ball_detector.py`: `cv_lite` grayscale/RGB circle detection and temporal candidate selection.
- `src/vision/geometry.py`: pixel error and safe-region logic.
- `src/control/filter.py`: exponential position filter.
- `src/communication/uart.py`: vision measurement packet generation.

Safe claim pattern: the current implementation detects the ball with a camera, filters its image position, and sends a measurement to the STM32.

Do not convert configured or commented FPS estimates into measured performance without a log.

### STM32 ball control

- `stm32_ball_controller/App/Inc/app_config.h`: current limits, target, timing, PID/velocity gains, stiction and integral conditions.
- `stm32_ball_controller/App/Src/ball_controller.c`: target setting, measurement acceptance, velocity estimate, PID-related control, stiction compensation, and output target.
- `stm32_ball_controller/App/Src/control_guard.c`: communication/vision/safety guard logic.
- `stm32_ball_controller/App/Src/servo_output.c`: PWM limits and slew behavior.
- `stm32_ball_controller/Core/Src/main.c`: integration, timing, debug state, and runtime target variables.
- `stm32_ball_controller/App/Src/icm20602.c` and `soft_i2c.c`: current inertial-sensor work; inspect integration before claiming chassis-disturbance compensation.
- `tests/pc/`: host-side controller tests; these validate software behavior, not physical competition performance.
- `data/servo_rail_calibration.csv` and `tools/mechanical_model.py`: calibration/modeling resources; inspect data completeness before using results.

Safe claim pattern: the STM32 receives visual measurements and computes a bounded servo command with guard behavior. Name the controller exactly as implemented at the inspected commit.

## Missing or separate evidence

The current repository snapshot does not visibly contain the complete vehicle line-following controller, drive-motor control, start-line recognition, ≤2-inch timer display, or wireless video receiver/recording implementation.

Therefore:

- Do not draft those subsystems as implemented from this repository alone.
- Request or locate the vehicle-controller project, schematics, wiring, and test logs.
- Treat the K230 ball-control camera as distinct from R1 wireless video transmission unless the actual hardware and recording chain prove otherwise.
- Do not claim R2, R4, R5, or R6 completion from bench ball-control code.

## Data needed before final report

- current BOM and schematic or wiring diagrams;
- vehicle MCU/source tree and line-sensor layout;
- motor driver and wheel/track geometry;
- start/stop/timer display implementation;
- video sender/receiver/recording hardware and sample file;
- pixel-to-centimetre calibration with uncertainty;
- servo pulse-to-rod-angle calibration;
- synchronized ball position, target, servo command, vehicle state, and timestamps;
- repeated R1–R6 test logs and video filenames;
- final dimensions, mass, battery, and power measurements;
- exact software/firmware revision used for each test.

## Provenance rule

When writing a numerical claim, cite a stable artifact path and preferably a Git commit or file hash in the internal evidence ledger. Report prose need not expose repository paths, but the team must be able to trace every number back to raw evidence.
