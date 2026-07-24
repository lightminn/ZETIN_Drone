# BMM350 Magnetometer Yaw Fusion — Design (2026-07-24)

## Goal
Stop yaw (heading) drift in `firmware/flight/dual_imu_cascade_pwm` by fusing the
on-board BMM350 magnetometer's absolute heading with the gyro-integrated yaw.
**Relative heading-hold only** — no true-north / declination / navigation.

## Constraints / non-goals
- Do **NOT** modify the validated roll/pitch estimator, mixer, or cascade control.
  Change is **isolated to the yaw (angleZ) channel**.
- Mag fusion **OFF by default** (compile + runtime gated). With it off the firmware
  must behave identically to the currently-validated hover build (roll/pitch/hover
  untouched). Flashing must not jeopardize the validated first hover.
- No soft-iron (hard-iron only, first pass), no declination, no navigation. YAGNI.

## Hardware
- BMM350 on **I2C (Wire) @ 0x14**, `DFRobot_BMM350` library. IMUs stay on **SPI**
  (unchanged) — separate bus, no conflict.
- Reference read code: `firmware/archive/other_sensors/board_v1_5_2_bmm350/`.

## Architecture
- **Mag read (core 0, ~50 Hz):** the BMM350 read is a blocking I2C transaction and
  must NOT run inside the 1 kHz `pid_task` (core 1). Read it on core 0 (a small task
  or the existing `udp_task` cadence). Apply hard-iron offsets; publish the latest
  calibrated mag vector to a `volatile` shared with `pid_task`.
- **Tilt-compensated heading:** project the calibrated mag vector to horizontal using
  the current roll/pitch (`angleX`/`angleY` globals), then `magHeading = atan2(...)`.
- **Fusion (yaw channel of `pid_task`):** keep the existing `angleZ += bodyGz*dt`
  (fast). Add a slow complementary pull toward the mag heading:
  `angleZ += K_MAG * wrapDeg(magHeading - angleZ)` per outer-loop tick, with a small
  `K_MAG` (time constant ~ few seconds). Must handle ±180° wrap correctly. Gyro gives
  fast dynamics; mag removes long-term drift.
- `yaw_enabled` (heading-hold) then holds a **drift-free, mag-referenced** heading.

## Calibration (hard-iron, one-time)
- A calibration mode/command: user rotates the drone through orientations (spin about
  yaw + some tilt); firmware collects per-axis min/max; `offset = (max+min)/2`. Print
  offsets over serial → hardcode as constants (same pattern as `IMU2_SIGN_*`). Provide
  it as a runnable step (diagnostic sketch or a gated firmware cal mode).

## Interference handling
- Motor/ESC current can corrupt the mag. The slow fusion τ rejects brief spikes. Bench
  test measures heading stability vs throttle. If severe (deferred options): gate the
  mag correction when throttle is high / `|bodyGz|` large, or lower `K_MAG`. Document
  the measured interference.

## Testing
- **SIL / native (software, completed by us):** extend `tools/native_tests/` — feed
  synthetic gyro (with an injected yaw-rate bias/drift) + synthetic mag (known true
  heading, incl. rotating and tilted cases). Assert the fused `angleZ` (a) converges to
  the true heading, (b) rejects the gyro drift over time, (c) tilt-compensation is
  correct at nonzero roll/pitch, (d) correct ±180° wrap. Test must FAIL if the fusion
  sign or tilt-comp is wrong (discriminating), pass when correct.
- **Bench HW (user runs — cannot be automated):** (1) hard-iron calibration rotation;
  (2) slowly rotate the drone, heading tracks true; (3) hold still, heading holds with
  no drift over minutes (vs gyro-only baseline); (4) spin motors to hover throttle,
  measure heading stability (interference). Report pass/interference level.

## Gating / safety
- Runtime `mag <0|1>` command (and/or compile flag), **default OFF**. With mag OFF the
  build is behaviorally identical to the validated firmware. Roll/pitch/hover untouched.
