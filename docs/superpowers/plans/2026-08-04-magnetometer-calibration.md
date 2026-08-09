# Magnetometer Calibration Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace min/max-only BMM350 calibration with an observable raw-capture flow and a robust Li & Griffiths constrained ellipsoid fitter that produces firmware-ready hard/soft-iron constants.

**Architecture:** The firmware remains responsible only for exposing raw calibration samples and applying baked constants. A NumPy host tool owns constrained fitting, three MAD clipping passes, coverage/fit validation, reporting, and constant generation. The existing append-only telemetry contract gains one final field, and the real-sketch 32-bit native harness proves both the matrix math and the calibration capture wire path.

**Tech Stack:** ESP32 Arduino C++, Python 3, NumPy, `unittest`, 32-bit `g++` plus `qemu-i386-static`, `arduino-cli`.

## Global Constraints

- Treat `docs/superpowers/specs/2026-08-04-magnetometer-calibration-design.md` sections 4.1-4.4 and section 5 as the contract; do not revisit the measured rationale.
- Preserve `MAG_HARD_IRON = {-1.831376, 9.318241, -12.517762}` and make the default `MAG_SOFT_IRON` the identity matrix so existing flight behavior is unchanged.
- Apply calibration as `out_i = MAG_BODY_SIGN_i * sum_j(MAG_SOFT_IRON[i][j] * (raw_j - MAG_HARD_IRON[j]))` before the existing throttle compensation.
- Append `Mag_Cal_Active` after the current field 64 `Flow_Quality`; all earlier packet prefixes and field numbers remain unchanged.
- Use `/home/light/anaconda3/bin/python` and redirect caches/build products to `/tmp`; do not flash hardware.
- Preserve the user's untracked design spec and unrelated work. This environment cannot write `.git`, so checkpoints inspect diffs instead of committing.

---

### Task 1: Product Ellipsoid Fitter

**Files:**
- Create: `tools/test_magcal_fit.py`
- Create: `scripts/magcal_fit.py`
- Modify: `requirements.txt`
- Modify: `.github/workflows/ci.yml`

**Interfaces:**
- Produces: `CalibrationError`, `CoverageError`, `FitError`, `CalibrationResult`, `fit_calibration(samples)`, `apply_calibration(samples, result)`, `load_calibration_csv(path, use_all=False)`, `format_firmware_constants(result)`, and `recalibrate_mag_comp(result.soft_iron, coefficients)`.
- Consumes: CSV columns `Mag_X`, `Mag_Y`, `Mag_Z`, and normally `Mag_Cal_Active`; `--all` selects all finite nonzero magnetic rows.

- [x] **Step 1: Write the four required failing tests and CLI boundary tests**

  Use deterministic synthetic samples with field magnitude 50.4 µT, hard iron `[-1.8, 9.3, -12.5]`, and symmetric soft iron
  `[[1.12, 0.05, -0.03], [0.05, 0.92, 0.04], [-0.03, 0.04, 1.05]]`.
  Assert center error below 0.35 µT, corrected heading 95th-percentile error below 1.5 degrees after removal of a constant relative-heading offset, 75-spike fitting succeeds for 20 deterministic seeds, planar/yaw-only coverage raises `CoverageError`, and hyperboloid samples raise `FitError`. Add a CSV fixture proving the default selector keeps only rows where `Mag_Cal_Active == 1`, while `use_all=True` bypasses that selector.

- [x] **Step 2: Run the focused test and verify RED**

  Run:
  `PYTHONPYCACHEPREFIX=/tmp/zetin-magcal-red /home/light/anaconda3/bin/python -m unittest tools.test_magcal_fit -v`

  Expected: import failure because `scripts/magcal_fit.py` does not exist.

- [x] **Step 3: Implement normalized Li & Griffiths fitting**

  Normalize samples around their mean by the RMS centered radius, construct
  `D = [x²,y²,z²,2xy,2xz,2yz,2x,2y,2z,1]`, partition `S=D.T@D`, eliminate the four linear/constant coefficients with a Schur complement, and enforce the six-term `4J-I²=1` constraint. Solve the symmetric generalized problem without `np.linalg.eig`: Cholesky-whiten the positive scatter matrix, solve the resulting symmetric matrix with `np.linalg.eigh`, and select the unique positive-constraint eigenvector. Normalize the fitted quadratic by `trace(M)>0`, reject non-positive `M` or `k`, compute `c=-solve(M,b)`, and compute `W=V@diag(sqrt(lambda))@V.T` from `np.linalg.eigh(M/k)`.

- [x] **Step 4: Add three MAD clipping passes and physical-unit scaling**

  Fit, calculate radial residuals, retain samples within `median ± 3 * 1.4826 * MAD`, and refit for exactly three clipping passes. Reject a pass that leaves too few samples or an implausibly small inlier fraction. Multiply the unit-sphere matrix by the mean of the three fitted semi-axis radii so corrected vectors remain in µT, and expose the final original-row inlier mask.

- [x] **Step 5: Add validation, reporting, and CLI output**

  Compute unit-direction covariance with `np.linalg.eigh`; warn above a ratio of 10 and reject degenerate coverage at 100 or non-positive minimum eigenvalue. Reject non-ellipsoid and poor-fit results rather than emitting NaN constants. Report selected/inlier sample counts, coverage eigenvalues/ratio, raw and corrected `|B|` spread, fitted radii/axis ratio, and the 95th-percentile radial residual in µT and percent. Do not infer heading accuracy without independent known-orientation data. Print paste-ready `MAG_HARD_IRON[3]`, `MAG_SOFT_IRON[3][3]`, and `W @ [0.007497, -0.001218, -0.000640]` with an explicit instruction to revalidate `mag_comp`.

- [x] **Step 6: Declare NumPy and make CI install the direct test dependency**

  Add `numpy>=1.26` to `requirements.txt`. In the Python CI job, install `numpy>=1.26` before byte-compilation and test discovery so the new tests do not rely on runner-global packages.

- [x] **Step 7: Run focused GREEN and robustness evidence**

  Run the focused unittest command from Step 2 and the CLI against a temporary valid CSV. Confirm every synthetic assertion passes and the output contains finite firmware constants and transformed `mag_comp` coefficients.

### Task 2: Firmware Matrix Application and Raw Calibration Capture

**Files:**
- Modify: `tools/native_tests/test_mag_yaw_fusion.cpp`
- Modify: `tools/native_tests/test_control_math.cpp`
- Modify: `tools/native_tests/test_sil_attitude.cpp`
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`

**Interfaces:**
- Produces: `applyMagCalibration(raw, hard_iron, soft_iron, out)` used by `sampleMagnetometer()` and directly exercised by native SIL.
- Preserves: Core-0 BMM350 read and `publishMagSample()` handoff; Core-1 throttle compensation remains downstream of the baked hard/soft-iron transform.

- [x] **Step 1: Add native tests before firmware changes**

  In `test_mag_yaw_fusion.cpp`, add one case with identity `soft_iron` proving exact equality to the legacy `MAG_BODY_SIGN_i * (raw_i - old_offset_i)` outputs, and one hand-calculated non-diagonal case proving every row of `W(raw-b)` is applied with the correct orientation. Add a calibration-capture case that injects shim BMM350 raw values, calls `sampleMagnetometer()`, asserts `magTelemX/Y/Z` equal the uncalibrated values, calls `sendTelemetry()`, and asserts the final field is `1`.

  In `test_control_math.cpp`, replace the old min/max-offset-output assertion with exact sample-count, X/Y/Z span, and `scripts/magcal_fit.py` guidance assertions; explicitly reject any printed `MAG_HARD_IRON_OFFSET_*` line.
  In `test_sil_attitude.cpp`, extend the direct wire fixture from 64 to 65 fields and assert the appended inactive value is `0`.

- [x] **Step 2: Run focused native wrappers and verify RED**

  Run:
  `PYTHONPYCACHEPREFIX=/tmp/zetin-mag-native-red /home/light/anaconda3/bin/python -m unittest tools.test_mag_yaw_fusion tools.test_native_control_math -v`

  Expected: compile/assertion failures because the matrix helper, raw calibration telemetry, final active field, and new stop message are absent.

- [x] **Step 3: Implement default-preserving hard/soft-iron constants and transform**

  Replace the three scalar offset constants with `MAG_HARD_IRON[3]` and add identity `MAG_SOFT_IRON[3][3]`. Implement one loop-based helper with a temporary centered vector and call it from `sampleMagnetometer()` before `publishMagSample()`.

- [x] **Step 4: Publish raw samples during calibration**

  In the `mag_calibrating` branch, copy finite `raw[0..2]` directly to `magTelemX/Y/Z`, then update extrema and sample count. Do not add a critical section: `startMagCalibration()` disables `mag_enabled`, so calibration and Core-1 compensated telemetry writers remain mutually exclusive.

- [x] **Step 5: Replace min/max result output with span diagnostics**

  Keep extrema solely to compute `span = max-min`. `stopMagCalibration()` must clear the active flag, print the sample count and all three spans, and direct the operator to capture the CSV and run `scripts/magcal_fit.py`; it must never calculate or print an offset.

- [x] **Step 6: Append active calibration state to the wire packet**

  Add `,%d` to the end of `sendTelemetry()` and append `(int)mag_calibrating` after `msp_flow_quality`. Do not insert it near `Mag_Enabled` or renumber fields 60-64.

- [x] **Step 7: Run focused GREEN**

  Re-run the two native wrappers from Step 2 and confirm the new identity/non-identity/raw-capture cases and migrated stop-output case pass.

### Task 3: Host Telemetry Contract and Operator Documentation

**Files:**
- Modify: `tools/test_telemetry_schema.py`
- Modify: `scripts/telemetry_schema.py`
- Modify: `docs/udp_protocol.md`
- Modify: `logs/README.md`
- Modify: `docs/bmm350_yaw_bench_test.md`

**Interfaces:**
- Produces: current 65-field schema ending in integer `Mag_Cal_Active` while all 10-64-field packets remain prefix-compatible.

- [x] **Step 1: Write schema RED tests**

  Append `Mag_Cal_Active` to the literal expected field tuple and integer type checks. Add a 64-field packet assertion returning `None` and a 65-field packet assertion parsing final values `0` and `1` as integers. Preserve every prior legacy packet assertion.

- [x] **Step 2: Run schema test and verify RED**

  Run:
  `PYTHONPYCACHEPREFIX=/tmp/zetin-mag-schema-red /home/light/anaconda3/bin/python -m unittest tools.test_telemetry_schema -v`

  Expected: the literal schema and 65th-field assertions fail because the field is absent.

- [x] **Step 3: Append the schema field and update parser descriptions**

  Add `Mag_Cal_Active` after `Flow_Quality`, map it to `int`, and update docstrings from the stale 59-field wording to the current 65-field contract. Parsing remains prefix-based, so no new branch is needed.

- [x] **Step 4: Update protocol and log documentation**

  Document 65 exact fields, field 65 semantics, and the fact that fields 32-34 are raw uncalibrated µT while `Mag_Cal_Active=1`, otherwise they retain their existing corrected-domain meaning. Update `magcal` to the CSV capture plus host-fit workflow and state that stop prints only count/span.

- [x] **Step 5: Correct the bench procedure without rewriting dated results**

  Replace Step 1 with `magcal 1` → rotate → log rows with `Mag_Cal_Active=1` → `magcal 0` → run `/home/light/anaconda3/bin/python scripts/magcal_fit.py <csv>` → paste hard/soft-iron and transformed `mag_comp` constants → rebuild/reflash → recapture/revalidate. Keep the 2026-07-27 result section unchanged and add a dated correction note explaining that its min/max offsets are historical evidence, not the current procedure.

- [x] **Step 6: Run schema GREEN and repository link check**

  Run the schema unittest and `/home/light/anaconda3/bin/python tools/check_repo_layout.py`.

### Task 4: Full Verification and Mutation Audit

**Files:**
- Verify only; do not add unrelated source changes.

**Interfaces:**
- Consumes every deliverable from Tasks 1-3.

- [x] **Step 1: Run focused product tests with complete output**

  Run:
  `MPLCONFIGDIR=/tmp/zetin-mag-mpl PYTHONPYCACHEPREFIX=/tmp/zetin-mag-focused /home/light/anaconda3/bin/python -m unittest tools.test_magcal_fit tools.test_mag_yaw_fusion tools.test_native_control_math tools.test_telemetry_schema -v`

- [x] **Step 2: Run discriminating negative controls**

  In temporary copies under `/tmp`, mutate one off-diagonal matrix multiplication index and remove the calibration raw-telemetry assignment; confirm the relevant native case fails. Mutate MAD clipping to retain all rows and confirm the 75-spike robustness test fails. Restore by discarding only the temporary copies.

- [x] **Step 3: Run the complete repository test suite**

  Run:
  `MPLCONFIGDIR=/tmp/zetin-mag-mpl PYTHONPYCACHEPREFIX=/tmp/zetin-mag-full /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v`

- [x] **Step 4: Compile the flight firmware**

  Run:
  `XDG_CACHE_HOME=/tmp/zetin-mag-arduino-cache arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 --build-path /tmp/zetin-mag-arduino-build firmware/flight/dual_imu_cascade_pwm`

- [x] **Step 5: Audit scope and acceptance criteria**

  Inspect `git diff --check`, `git status --short`, and targeted diffs. Re-read spec sections 4 and 5 and map every requirement to fresh evidence. Report synthetic heading 95p, all 20 spike-trial outcomes, focused/native/full test counts, Arduino compile exit status, and explicitly distinguish host/native proof from unperformed physical BMM350 capture or flashing.
