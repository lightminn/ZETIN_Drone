const DISPLAY_LIMIT = 45;
const DEFAULT_SMOOTHING = 0.18;

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function finiteOrNull(value) {
  return Number.isFinite(value) ? value : null;
}

function shortestDelta(sample, neutral) {
  return ((sample - neutral + 540) % 360) - 180;
}

function permissionStatus(sensorEvent) {
  if (sensorEvent == null) return Promise.resolve("unavailable");
  if (typeof sensorEvent.requestPermission !== "function") {
    return Promise.resolve("granted");
  }

  try {
    return Promise.resolve(sensorEvent.requestPermission())
    .then((status) => (status === "granted" ? "granted" : "denied"))
    .catch(() => "error");
  } catch {
    return Promise.resolve("error");
  }
}

function reasonForStatus(status) {
  if (status === "denied") return "denied";
  if (status === "error") return "permission-error";
  if (status === "unavailable") return "unsupported";
  return "granted";
}

function accessReason(orientation, motion) {
  if (orientation !== "granted") return reasonForStatus(orientation);
  if (motion === "denied") return "motion-denied";
  if (motion === "error") return "motion-permission-error";
  if (motion === "unavailable") return "motion-unavailable";
  return "granted";
}

export function detectSensorCapability(env) {
  const secure = env?.isSecureContext === true;
  if (!secure) {
    return { available: false, secure, reason: "https-required" };
  }

  const available = env?.DeviceOrientationEvent != null;
  return {
    available,
    secure,
    reason: available ? "ready" : "unsupported",
  };
}

export async function requestSensorAccess(env) {
  const capability = detectSensorCapability(env);
  if (!capability.available) {
    return {
      orientation: "unavailable",
      motion: "unavailable",
      reason: capability.reason,
    };
  }

  const [orientation, motion] = await Promise.all([
    permissionStatus(env.DeviceOrientationEvent),
    permissionStatus(env.DeviceMotionEvent),
  ]);

  return {
    orientation,
    motion,
    reason: accessReason(orientation, motion),
  };
}

export class OrientationModel {
  constructor({ smoothing = DEFAULT_SMOOTHING } = {}) {
    this.smoothing = clamp(Number(smoothing) || 0, 0, 1);
    this.reset();
  }

  updateOrientation({ beta, gamma } = {}) {
    if (!Number.isFinite(beta) || !Number.isFinite(gamma)) return this.snapshot();

    const roll = clamp(shortestDelta(gamma, this.neutral.gamma), -DISPLAY_LIMIT, DISPLAY_LIMIT);
    const pitch = clamp(shortestDelta(beta, this.neutral.beta), -DISPLAY_LIMIT, DISPLAY_LIMIT);
    this.orientation = this.hasOrientation
      ? {
          roll: this.orientation.roll + this.smoothing * (roll - this.orientation.roll),
          pitch: this.orientation.pitch + this.smoothing * (pitch - this.orientation.pitch),
        }
      : { roll, pitch };
    this.sample = { beta, gamma };
    this.hasOrientation = true;
    return this.snapshot();
  }

  updateMotion({ accelerationIncludingGravity } = {}) {
    const acceleration = accelerationIncludingGravity ?? {};
    const ax = finiteOrNull(acceleration.x);
    const ay = finiteOrNull(acceleration.y);
    const az = finiteOrNull(acceleration.z);
    if (ax === null || ay === null || az === null) return this.snapshot();

    this.motion = this.hasMotion
      ? {
          ax: this.motion.ax + this.smoothing * (ax - this.motion.ax),
          ay: this.motion.ay + this.smoothing * (ay - this.motion.ay),
          az: this.motion.az + this.smoothing * (az - this.motion.az),
        }
      : { ax, ay, az };
    this.hasMotion = true;
    return this.snapshot();
  }

  calibrate() {
    if (this.sample !== null) {
      this.neutral = { ...this.sample };
      this.orientation = { roll: 0, pitch: 0 };
      this.hasOrientation = true;
    }
    return this.snapshot();
  }

  snapshot() {
    return {
      orientation: { ...this.orientation },
      motion: { ...this.motion },
    };
  }

  reset() {
    this.neutral = { beta: 0, gamma: 0 };
    this.sample = null;
    this.orientation = { roll: 0, pitch: 0 };
    this.motion = { ax: null, ay: null, az: null };
    this.hasOrientation = false;
    this.hasMotion = false;
    return this.snapshot();
  }
}
