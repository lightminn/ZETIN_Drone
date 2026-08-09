import assert from "node:assert/strict";
import test from "node:test";

import {
  detectSensorCapability,
  OrientationModel,
  requestSensorAccess,
} from "../src/imu.mjs";

test("detectSensorCapability selects touch fallback when orientation is unavailable", () => {
  assert.deepEqual(detectSensorCapability({ isSecureContext: true }), {
    available: false,
    secure: true,
    reason: "unsupported",
  });
});

test("detectSensorCapability explains the HTTPS requirement on an insecure origin", () => {
  assert.deepEqual(
    detectSensorCapability({
      isSecureContext: false,
      DeviceOrientationEvent: class DeviceOrientationEvent {},
    }),
    { available: false, secure: false, reason: "https-required" },
  );
});

test("requestSensorAccess resolves granted permissions", async () => {
  const result = await requestSensorAccess({
    isSecureContext: true,
    DeviceOrientationEvent: {
      requestPermission: async () => "granted",
    },
    DeviceMotionEvent: {
      requestPermission: async () => "granted",
    },
  });

  assert.deepEqual(result, {
    orientation: "granted",
    motion: "granted",
    reason: "granted",
  });
});

test("requestSensorAccess invokes permission methods before yielding from the click path", async () => {
  let calls = 0;
  const access = requestSensorAccess({
    isSecureContext: true,
    DeviceOrientationEvent: {
      requestPermission() {
        calls += 1;
        return Promise.resolve("granted");
      },
    },
    DeviceMotionEvent: {
      requestPermission() {
        calls += 1;
        return Promise.resolve("granted");
      },
    },
  });

  assert.equal(calls, 2);
  await access;
});

test("requestSensorAccess preserves denied and error permission branches", async () => {
  const denied = await requestSensorAccess({
    isSecureContext: true,
    DeviceOrientationEvent: {
      requestPermission: async () => "denied",
    },
    DeviceMotionEvent: {
      requestPermission: async () => "granted",
    },
  });
  const errored = await requestSensorAccess({
    isSecureContext: true,
    DeviceOrientationEvent: {
      requestPermission: async () => {
        throw new TypeError("permission unavailable");
      },
    },
    DeviceMotionEvent: {
      requestPermission: async () => "granted",
    },
  });

  assert.deepEqual(denied, {
    orientation: "denied",
    motion: "granted",
    reason: "denied",
  });
  assert.deepEqual(errored, {
    orientation: "error",
    motion: "granted",
    reason: "permission-error",
  });
});

test("OrientationModel calibration subtracts the current neutral sample", () => {
  const model = new OrientationModel({ smoothing: 1 });
  model.updateOrientation({ beta: 10, gamma: -5 });
  model.calibrate();
  model.updateOrientation({ beta: 14, gamma: 3 });

  assert.deepEqual(model.snapshot().orientation, { roll: 8, pitch: 4 });
});

test("OrientationModel applies the literal new-sample smoothing formula", () => {
  const model = new OrientationModel({ smoothing: 0.18 });
  model.updateOrientation({ beta: 10, gamma: 20 });
  model.updateOrientation({ beta: 20, gamma: 30 });

  assert.deepEqual(model.snapshot().orientation, { roll: 21.8, pitch: 11.8 });
});

test("OrientationModel uses the shortest neutral angle and clamps display axes", () => {
  const model = new OrientationModel({ smoothing: 1 });
  model.updateOrientation({ beta: 170, gamma: 179 });
  model.calibrate();
  model.updateOrientation({ beta: -170, gamma: -179 });

  assert.deepEqual(model.snapshot().orientation, { roll: 2, pitch: 20 });

  model.updateOrientation({ beta: -100, gamma: 100 });
  assert.deepEqual(model.snapshot().orientation, { roll: -45, pitch: 45 });
});

test("OrientationModel smooths optional acceleration and reset clears it", () => {
  const model = new OrientationModel({ smoothing: 0.5 });
  model.updateMotion({ accelerationIncludingGravity: { x: 2, y: -4, z: 8 } });
  model.updateMotion({ accelerationIncludingGravity: { x: 6, y: 4, z: 0 } });

  assert.deepEqual(model.snapshot().motion, { ax: 4, ay: 0, az: 4 });

  model.reset();
  assert.deepEqual(model.snapshot(), {
    orientation: { roll: 0, pitch: 0 },
    motion: { ax: null, ay: null, az: null },
  });
});
