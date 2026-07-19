# Repository Instructions

These rules apply to the entire repository.

## Required context

Before changing K230 runtime code:

1. Read this file.
2. Read `docs/verified-api-notes.md`.
3. Read `.agents/skills/k230-canmv-vision-control/SKILL.md`.
4. Inspect existing code and verified examples before selecting an API.

## Target and dependencies

- Target the Yahboom K230 12Pin AI Vision Module running CanMV K230 Yahboom v1.8.0 MicroPython.
- Do not assume CPython compatibility on the device.
- Do not import `cv2`, NumPy, Pandas, or Matplotlib in `src/`.
- Keep explicitly requested PC-side utilities under `tests/pc/` or a future `tools/pc/` directory.
- Prefer `media.sensor.Sensor` and `cv_lite` for the first-stage vision pipeline.
- Do not use K210 or old OpenMV APIs such as `sensor.find_blobs()` or `image.find_blobs()` unless verified on this exact firmware.
- Do not introduce YOLO or other model inference unless explicitly requested.
- Do not invent CanMV functions. Mark unverified APIs and require physical-device testing.

## Architecture

- Keep deployable K230 code under `src/`.
- Keep `src/main.py` limited to initialization, coordination, the main loop, error handling, and resource cleanup.
- Keep vision algorithms in `src/vision/`, UART code in `src/communication/`, filtering in `src/control/`, and lightweight support code in `src/utils/`.
- Keep tunable values in `src/config.py`.
- Keep PC-only tests under `tests/pc/`.
- Keep test observations in ignored files under `logs/`; record durable verified API facts in `docs/verified-api-notes.md`.

## Runtime behavior

- Process one frame at a time and avoid unnecessary allocations or image copies.
- Rate-limit console output, garbage collection, and debug-image writes.
- Return an explicit invalid result when detection fails; never reuse stale coordinates as fresh data.
- Keep UART disabled until vision output and the protocol have been independently validated.
- Keep actuator PID and servo output on the downstream MCU unless the project scope explicitly changes.

## Validation and handoff

- Treat Computer A checks as syntax and pure-logic validation only.
- Treat Computer B plus the physical K230 as the final runtime test environment.
- For each runtime code change, report: used CanMV APIs, extra dependencies, v1.8.0 compatibility status, and remaining hardware/version/performance risks.
- Do not claim an API is verified until an observed result is recorded in `docs/verified-api-notes.md`.
- Use Git as the source of truth. Do not commit or push unless the user requests it; never hide unverified hardware behavior in a commit message.

