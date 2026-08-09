# Magnetometer Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Include the linked cascade PID source, stop presenting radial fit residuals as heading accuracy, and make `Mag_Cal_Active` atomically describe the `Mag_X/Y/Z` domain.

**Architecture:** Keep the 65-field append-only wire schema. The host fitter reports radial residuals only. Firmware separates internal calibration control state from a mutex-protected wire snapshot containing XYZ plus its domain flag; start waits for the first raw sample before advertising raw data, and stop restores a corrected snapshot atomically.

**Tech Stack:** Typst, Python 3 with NumPy and `unittest`, ESP32 Arduino C++, 32-bit native `g++`/QEMU harness, `arduino-cli`.

## Global Constraints

- Follow `docs/superpowers/specs/2026-08-09-review-fixes-design.md` exactly.
- Work in the existing `feat/magcal-ellipsoid-fit` checkout; do not touch `main`, flash hardware, merge, push, stage, or commit.
- Preserve fields 1-65, existing hard/soft-iron constants, `mag_comp`, yaw fusion, roll/pitch, mixer, and controller gains.
- Preserve unrelated dirty and untracked files, including the generated cascade PDF.
- Use `/home/light/anaconda3/bin/python`; put caches, native executables, clean-tree simulations, and Arduino build products under `/tmp`.
- Every production behavior change requires a test that fails for the expected reason before implementation.

---

### Task 1: Include the Linked Cascade PID Source

**Files:**
- Modify: `docs/cascade_vs_single_pid.typ:1`
- Preserve: `docs/cascade_vs_single_pid.pdf`
- Verify: `docs/README.md:10`

**Interfaces:**
- Consumes: the existing README link `cascade_vs_single_pid.typ`.
- Produces: a self-contained Typst source present in the delivered change set and compilable to a four-page PDF.

- [x] **Step 1: Preserve the clean-tree RED evidence**

  Archive HEAD into a temporary directory, apply only the current tracked README diff, and run:

  ```bash
  /home/light/anaconda3/bin/python tools/check_repo_layout.py
  ```

  Expected in the temporary tree: exit 1 with `docs/README.md: missing link target cascade_vs_single_pid.typ`.

- [x] **Step 2: Mark the existing Typst source as an intentional delivered file**

  Add this source comment at line 1 without changing rendered content:

  ```typst
  // README에서 링크하는 유지 문서 원본. PDF는 이 파일에서 재생성한다.
  ```

  Keep the README link and do not edit or delete the existing PDF.

- [x] **Step 3: Verify formatting and rendering**

  Run:

  ```bash
  typstyle --check docs/cascade_vs_single_pid.typ
  typst compile docs/cascade_vs_single_pid.typ /tmp/zetin-review-cascade.pdf
  ```

  Expected: both exit 0; `pdfinfo` reports four A4 pages.

- [x] **Step 4: Verify a clean delivered tree**

  Archive HEAD again, apply the tracked diff, and copy the new Typst source plus the new review spec/plan into the temporary tree. Run `tools/check_repo_layout.py` there.

  Expected: exit 0 with `repository layout checks passed`.

---

### Task 2: Report Radial Calibration Residuals Honestly

**Files:**
- Modify: `tools/test_magcal_fit.py`
- Modify: `scripts/magcal_fit.py:34-55,324-342,446-470`
- Modify: `docs/bmm350_yaw_bench_test.md:43-44`
- Modify: `docs/superpowers/specs/2026-08-04-magnetometer-calibration-design.md:104-113`
- Modify: `docs/superpowers/plans/2026-08-04-magnetometer-calibration.md:56-61`

**Interfaces:**
- Produces: `CalibrationResult.radial_residual_95_ut: float`.
- Removes: `CalibrationResult.heading_error_95_deg` and every acceptance-facing radial-to-heading conversion.
- Preserves: synthetic tests that compare corrected headings to independently known true headings.

- [x] **Step 1: Add a failing report regression test**

  Import `format_report`. Fit the deterministic full-coverage fixture, independently calculate

  ```python
  corrected = apply_calibration(samples[result.inlier_mask], result)
  want_ut = float(
      np.percentile(
          np.abs(np.linalg.norm(corrected, axis=1) - result.target_radius_ut),
          95.0,
      )
  )
  ```

  and assert:

  ```python
  self.assertAlmostEqual(want_ut, result.radial_residual_95_ut, places=10)
  report = format_report(result)
  self.assertIn("95p |B| radial residual", report)
  self.assertIn("uT", report)
  self.assertIn("%", report)
  self.assertNotIn("heading error", report.lower())
  ```

  Add a 30° Z-axis rotation of corrected vectors and independently assert that their norms stay unchanged while their headings change by 30°. This fixture explains why the report must remain radial; the report assertion above is the production boundary.

- [x] **Step 2: Run focused RED**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-review-magcal-red \
    /home/light/anaconda3/bin/python -m unittest \
    tools.test_magcal_fit.MagnetometerCalibrationFitTests.test_report_exposes_only_radial_fit_residual -v
  ```

  Expected: FAIL because `CalibrationResult` has no `radial_residual_95_ut`.

- [x] **Step 3: Replace the unsafe result field and report**

  In `fit_calibration`, retain the existing independently computed 95th-percentile magnitude residual in µT:

  ```python
  radial_residual_95_ut = float(
      np.percentile(np.abs(corrected_norms - target_radius), 95.0)
  )
  ```

  Store it as `radial_residual_95_ut`. Remove the `asin` conversion. In `format_report`, calculate only the dimensionless percentage for display:

  ```python
  radial_residual_95_pct = (
      100.0 * result.radial_residual_95_ut / result.target_radius_ut
  )
  ```

  Emit `95p |B| radial residual: <µT> uT (<percent>%)` and no heading claim.

- [x] **Step 4: Correct operator and design prose**

  Replace only the invalid residual-derived heading wording. Keep measured historical heading errors and synthetic known-orientation heading tests because those use independent references. State explicitly that radial residual measures sphere fit, not heading accuracy.

- [x] **Step 5: Run focused GREEN and source audit**

  Run the full `tools.test_magcal_fit` module and search production/report documentation for `heading_error_95_deg` and `estimated heading error from 95p magnitude residual`.

  Expected: all fitter tests pass and both obsolete identifiers/phrases have zero matches.

---

### Task 3: Publish XYZ and Calibration Domain as One Snapshot

**Files:**
- Modify: `tools/native_tests/test_mag_yaw_fusion.cpp`
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino:668-679,793-835,884-963,1332-1344,1803-1837`
- Modify: `docs/udp_protocol.md:157-164,235-266`
- Modify: `logs/README.md:55-57`

**Interfaces:**
- Produces: `publishMagTelemetry(x, y, z, calibration_raw)`, `readMagTelemetry(x, y, z, calibration_raw)`, and `magTelemCalActive`.
- `publishMagTelemetry(..., false)` must decline a late normal-domain write while `mag_calibrating` is true.
- `sendTelemetry()` must read the four-value snapshot once and send that local copy.

- [x] **Step 1: Replace the existing native capture case with a transition regression**

  Add a test-only CSV splitter. In one real-sketch case:

  1. Reset state and publish corrected sentinel `(1.25, -2.50, 3.75, false)`.
  2. Call `startMagCalibration()` and assert the packet still contains those XYZ values and field 65 `0`.
  3. Inject raw values `MAG_HARD_IRON + (10, 20, 30)`, sample at 20 ms, and assert raw XYZ with field 65 `1`.
  4. Call `publishMagTelemetry(99, 98, 97, false)` while active and assert the raw snapshot remains unchanged.
  5. Call `stopMagCalibration()` and assert field 65 `0` with independently calculated corrected XYZ `(10.65519563, 19.77457855, 31.10798149)` at `base_throttle=1000`.

  The packet assertions parse fields 32-34 and 65, not only globals.

- [x] **Step 2: Run native RED**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-review-mag-native-red \
    /home/light/anaconda3/bin/python -m unittest tools.test_mag_yaw_fusion -v
  ```

  Expected: native compile failure because `publishMagTelemetry` does not exist.

- [x] **Step 3: Add the coherent telemetry snapshot helpers**

  Add `volatile bool magTelemCalActive = false`. Under `magSnapshotMux`, implement:

  ```cpp
  static inline void publishMagTelemetry(
      float x, float y, float z, bool calibration_raw) {
    portENTER_CRITICAL(&magSnapshotMux);
    if (calibration_raw || !mag_calibrating) {
      magTelemX = x;
      magTelemY = y;
      magTelemZ = z;
      magTelemCalActive = calibration_raw;
    }
    portEXIT_CRITICAL(&magSnapshotMux);
  }
  ```

  Implement `readMagTelemetry(float&, float&, float&, bool&)` using the same lock. Set `mag_calibrating=true` under this lock in `startMagCalibration()` without changing the wire snapshot.

- [x] **Step 4: Publish raw and normal domains only through the helper**

  Replace direct raw assignments in `sampleMagnetometer()` with `publishMagTelemetry(raw..., true)`. Replace Core 1's three direct compensated assignments with `publishMagTelemetry(mag..., false)`. The helper's in-lock guard rejects any compensated writer that entered the Core 1 block before calibration disabled `mag_enabled`.

- [x] **Step 5: Restore corrected telemetry atomically on stop**

  Read the last calibrated `MagSnapshot`, apply the existing throttle coefficient subtraction using current `base_throttle`, then enter `magSnapshotMux` once to set `mag_calibrating=false`, publish corrected XYZ, and set `magTelemCalActive=false`. With zero samples, clear only the internal state and wire flag while preserving the pre-calibration corrected XYZ.

- [x] **Step 6: Send one local coherent snapshot**

  At the start of `sendTelemetry()`, read XYZ plus the domain flag under the helper. Pass those local values to `udp.printf`, and use the local domain flag as field 65 instead of `mag_calibrating`.

- [x] **Step 7: Update the wire documentation**

  Clarify that field 65 describes the domain of fields 32-34 in the same packet: it changes to 1 with the first raw sample and returns to 0 only with restored corrected XYZ. Do not change field count or order.

- [x] **Step 8: Run native GREEN and the control/schema regressions**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-review-mag-native-green \
    /home/light/anaconda3/bin/python -m unittest \
    tools.test_mag_yaw_fusion tools.test_native_control_math tools.test_telemetry_schema -v
  ```

  Expected: all cases pass.

---

### Task 4: Full Verification and Scope Audit

**Files:**
- Verify all modified and newly delivered files; do not make unrelated cleanup changes.

**Interfaces:**
- Consumes all deliverables from Tasks 1-3.

- [x] **Step 1: Run the complete repository test suite**

  ```bash
  MPLCONFIGDIR=/tmp/zetin-review-full-mpl \
  PYTHONPYCACHEPREFIX=/tmp/zetin-review-full-pycache \
    /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
  ```

  Record the exact test count, skips, failures, exit status, and any QEMU timeout separately.

- [x] **Step 2: Compile the flight firmware with warnings enabled**

  ```bash
  XDG_CACHE_HOME=/tmp/zetin-review-arduino-cache \
    arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
    --build-path /tmp/zetin-review-arduino-build \
    firmware/flight/dual_imu_cascade_pwm
  ```

  Record exit status, warning count, and binary size. Do not flash.

- [x] **Step 3: Verify Typst, clean-tree layout, and diff hygiene**

  Re-run the Task 1 Typst checks and clean-tree layout simulation. Run `git diff --check`, `git status --short`, and inspect targeted diffs plus untracked-file content.

- [x] **Step 4: Run discriminating negative controls in temporary copies**

  Under `/tmp`, mutate the report label back to a heading claim and confirm the new report test fails. Mutate `sendTelemetry()` to use internal `mag_calibrating` or allow a late normal writer and confirm the transition regression fails. Discard only the temporary copies.

- [x] **Step 5: Final contract audit**

  Confirm: the linked Typst source is delivered; no radial-to-heading claim remains; every packet's flag matches its XYZ domain; field count stays 65; no gain/calibration constants changed; no commit, stage, push, flash, or hardware action occurred.
