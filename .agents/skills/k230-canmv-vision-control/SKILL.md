---
name: k230-canmv-vision-control
description: Develop and review embedded vision-control code for the Yahboom K230 12Pin AI Vision Module running CanMV K230 Yahboom v1.8.0 MicroPython. Use for camera capture, cv_lite image processing, ball or track detection, geometric position/error estimation, LCD debugging, debug-image saving, UART measurement protocols, filtering, project architecture, performance reviews, and K230 compatibility checks in this ball-balance project.
---

# K230 CanMV Vision Control

Develop for the real K230 MicroPython target. Treat PC Python only as an optional static-checking or offline-analysis environment, never as proof that firmware code works.

Before changing code, read the repository `AGENTS.md`, `docs/verified-api-notes.md`, and existing verified examples. Do not invent CanMV functions; mark any API absent from the verified notes as requiring physical-device validation.

## Target environment

- Target the Yahboom K230 12Pin AI Vision Module.
- Target CanMV K230 Yahboom v1.8.0 and MicroPython.
- Acquire frames through `media.sensor.Sensor` and `snapshot()`.
- Prefer APIs confirmed on the device: `cv_lite`, `media.sensor.Sensor`, `aidemo`, and `nncase_runtime`.
- Use `aidemo` or `nncase_runtime` only when the user explicitly requests AI inference; do not select them for the first-stage traditional-vision workflow.
- Use confirmed camera controls when needed: `set_auto_gain()`, `get_gain_db()`, `set_auto_exposure()`, `get_exposure_us()`, `get_rgb_gain_db()`, `set_hmirror()`, and `set_vflip()`.

## Enforce dependency boundaries

- Never generate `import cv2`.
- Never import NumPy, Pandas, or Matplotlib in K230 code.
- Allow those packages only in a clearly separated PC-side analysis tool when the user explicitly requests it.
- Do not assume CPython standard-library modules exist in MicroPython.
- Do not use K210 or old OpenMV APIs such as `sensor.find_blobs()` or `image.find_blobs()` unless compatibility with this exact firmware has been established.
- Do not introduce YOLO, model training, or deep-learning inference unless explicitly requested.
- Do not suggest installing Python packages on the K230 when firmware modules provide the required function.

## Follow the vision pipeline

Implement the smallest sufficient pipeline:

1. Initialize `media.sensor.Sensor` once.
2. Capture one frame with `snapshot()`.
3. Process the frame with `cv_lite` using the pixel format required by the selected function.
4. Apply threshold segmentation and connected-region/blob analysis.
5. Reject candidates with inexpensive geometric checks such as area, ROI, aspect ratio, or expected position.
6. Compute the target center as `ball_x` and `ball_y`.
7. Draw concise status information on the LCD.
8. Print low-rate diagnostic values and save debug images only when useful.
9. Send measurements over UART only after the visual result and protocol have been validated.

Keep color thresholds, ROI, minimum area, display settings, UART settings, and debug intervals in configuration rather than scattering constants through the algorithm.

## Preserve real-time behavior

- Process one frame at a time and avoid retaining frame histories by default.
- Avoid unnecessary image copies, repeated initialization, large temporary lists, and per-frame object construction.
- Prefer references exposed by CanMV image APIs when the selected `cv_lite` operation requires them.
- Keep morphology kernels and iteration counts small.
- Restrict processing to an ROI when measurements show it is beneficial and the API supports it safely.
- Rate-limit `print()` and SD-card writes; never save a debug image every frame.
- Add complexity only after measuring detection quality and FPS on the device.
- Release Sensor, Display, media buffers, and UART resources in the correct CanMV order after exceptions or user interruption.

## Maintain project architecture

Keep responsibilities separated:

- `main.py`: initialize hardware, coordinate modules, run the loop, handle exceptions, and release resources.
- `vision/ball_detector.py`: detect the ball and return a stable result or `None`.
- `vision/track_detector.py`: detect fixed-end Marker A and servo-end Marker B.
- `vision/geometry.py`: perform projection and normalized-error calculations without camera dependencies.
- `communication/uart.py`: encode and transmit measurements.
- `control/filter.py`: filter measurements; keep actuator PID on the external MCU unless explicitly changed.
- `config.py`: hold tunable hardware and vision parameters.
- `debug/`: provide lightweight logging and debug-image helpers.

Do not put the full algorithm in `main.py`. Include a concise header docstring in each generated runtime module stating its CanMV APIs, required hardware, and CanMV v1.8.0 compatibility assumptions.

## Compute track-relative error

Treat Marker A as the fixed/left end and Marker B as the servo/right end. For ball center `P`, project `P` onto directed vector `AB`:

```text
t = dot(P - A, B - A) / dot(B - A, B - A)
normalized_error = 2*t - 1
```

- Return `-1` at A, `0` at the midpoint, and `1` at B.
- Reject coincident endpoints before dividing.
- Clamp to `[-1, 1]` only when the consumer expects a bounded command; retain the unclamped value when diagnosing out-of-track detections.
- Return an explicit invalid result when the ball or either marker is missing. Never reuse a stale coordinate as a fresh detection.

## Design UART output conservatively

- Keep UART disabled during visual-only milestones.
- Prefer integer transport such as `error_milli = round(normalized_error * 1000)`.
- Plan fields for `valid`, `error_milli`, and position.
- Add framing, versioning, checksum, byte order, rate, and timeout behavior before closed-loop use.
- Ensure an invalid visual measurement cannot be interpreted as a valid zero error.

## Use device-oriented debugging

Prefer, in order:

1. LCD overlays showing detection state, coordinates, bounding geometry, and low-rate FPS.
2. Rate-limited `print()` output for thresholds and measurements.
3. Rate-limited debug-image saving to the device filesystem or SD card.

Do not make PC-only visualization part of the runtime path.

## Apply the development workflow

1. Inspect the existing repository and preserve its module boundaries.
2. Verify the intended API against known v1.8.0 device capability or clearly flag it as unverified.
3. Implement one independently testable change.
4. Run syntax and pure-logic checks on Computer A, while labeling them as non-hardware validation.
5. Hand the change to Computer B for CanMV IDE upload and K230 testing.
6. Record firmware, image size, light conditions, thresholds, FPS, and observed errors with the Git change.
7. Treat GitHub as the source of truth once a remote is configured.

## Report every code change

State all of the following in the handoff:

1. Which CanMV/K230 APIs the code uses.
2. Whether any extra module installation is required.
3. Whether compatibility with Yahboom CanMV v1.8.0 is confirmed, inferred, or still requires device testing.
4. Any version, display, pin-mapping, filesystem, memory, or performance risk.

Before finishing, check that the code is K230-compatible, uses no unavailable dependency, avoids old APIs, and remains plausible for real-time single-frame execution.
