import { createChallengeState, restartChallenge, stepChallenge } from "./challenge.mjs";
import { OrientationModel, requestSensorAccess } from "./imu.mjs";
import { joystickVector } from "./joystick.mjs";
import { submitScore } from "./score-client.mjs";

const FIXED_STEP = 1 / 60;
const MAX_FRAME_SECONDS = 0.25;
const SENSOR_SAMPLE_TIMEOUT_MS = 4000;

const app = document.querySelector("#student-app");
const screenPanels = [...document.querySelectorAll("[data-screen-panel]")];
const horizon = document.querySelector("[data-horizon]");
const rollValue = document.querySelector("[data-roll-value]");
const pitchValue = document.querySelector("[data-pitch-value]");
const motionValues = {
  ax: document.querySelector("[data-ax-value]"),
  ay: document.querySelector("[data-ay-value]"),
  az: document.querySelector("[data-az-value]"),
};
const stabilityBar = document.querySelector("[data-stability-bar]");
const stabilityValue = document.querySelector("[data-stability-value]");
const sensorReason = document.querySelector("[data-sensor-reason]");
const joystickWrap = document.querySelector("[data-joystick-wrap]");
const joystick = document.querySelector("[data-joystick]");
const joystickStick = document.querySelector("[data-joystick-stick]");
const nicknameInput = document.querySelector("[data-nickname]");

const orientationModel = new OrientationModel();
let mode = "none";
let sensorAccess = null;
let firstOrientationReceived = false;
let sensorTimeout = 0;
let sensorAttemptGeneration = 0;
let activePointer = null;
let touchVector = { x: 0, y: 0, magnitude: 0 };
const pressedArrowKeys = new Set();
let challenge = createChallengeState();
let accumulator = 0;
let lastFrameTime = null;
let challengeFrame = 0;
let localResult = null;

function nextChallengeSeed() {
  const seed = new Uint32Array(1);
  crypto.getRandomValues(seed);
  return seed[0] || 0x6d2b79f5;
}

function showScreen(name) {
  app.dataset.screen = name;
  screenPanels.forEach((panel) => {
    panel.hidden = panel.dataset.screenPanel !== name;
  });
  updateJoystickVisibility();
}

function setMode(nextMode) {
  mode = nextMode;
  app.dataset.mode = mode;
  updateJoystickVisibility();
}

function updateJoystickVisibility() {
  joystickWrap.hidden = !(
    mode === "touch" && (app.dataset.screen === "imu" || app.dataset.screen === "challenge")
  );
}

function formatAngle(value) {
  const rounded = Math.abs(value) < 0.05 ? 0 : value;
  const prefix = rounded > 0 ? "+" : "";
  return `${prefix}${rounded.toFixed(1)}°`;
}

function formatMotion(value) {
  return Number.isFinite(value) ? value.toFixed(1) : "—";
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function displaySnapshot() {
  if (app.dataset.screen === "challenge" || app.dataset.screen === "result") {
    return {
      roll: challenge.roll,
      pitch: challenge.pitch,
      motion: orientationModel.snapshot().motion,
    };
  }
  if (mode === "touch") {
    return {
      roll: touchVector.x * 20,
      pitch: touchVector.y * 20,
      motion: { ax: null, ay: null, az: null },
    };
  }
  const snapshot = orientationModel.snapshot();
  return { ...snapshot.orientation, motion: snapshot.motion };
}

function currentStability() {
  if (app.dataset.screen === "challenge" || app.dataset.screen === "result") {
    if (!(challenge.elapsed > 0)) return 100;
    return clamp(challenge.stabilityIntegral / challenge.elapsed, 0, 100);
  }
  const { roll, pitch } = displaySnapshot();
  return clamp(100 - Math.hypot(roll, pitch) * 2, 0, 100);
}

function renderInstrument() {
  const snapshot = displaySnapshot();
  const stability = currentStability();
  horizon.style.setProperty("--roll", `${snapshot.roll}deg`);
  horizon.style.setProperty("--pitch", `${snapshot.pitch * 1.35}px`);
  horizon.setAttribute(
    "aria-label",
    `Roll ${snapshot.roll.toFixed(1)}도, Pitch ${snapshot.pitch.toFixed(1)}도`,
  );
  rollValue.textContent = formatAngle(snapshot.roll);
  pitchValue.textContent = formatAngle(snapshot.pitch);
  motionValues.ax.textContent = formatMotion(snapshot.motion.ax);
  motionValues.ay.textContent = formatMotion(snapshot.motion.ay);
  motionValues.az.textContent = formatMotion(snapshot.motion.az);
  stabilityBar.style.width = `${stability.toFixed(1)}%`;
  stabilityValue.textContent = `${Math.round(stability)}%`;
}

function sensorReasonText(reason) {
  const messages = {
    "https-required": "HTTP에서는 센서가 제한됩니다. HTTPS 주소에서 다시 열거나 터치 체험을 이용하세요.",
    unsupported: "이 브라우저에서 기울임 센서를 찾지 못했습니다. 터치 체험은 그대로 사용할 수 있습니다.",
    denied: "센서 권한이 거부되었습니다. 브라우저 설정을 바꾸지 않아도 터치로 계속할 수 있습니다.",
    "permission-error": "센서 권한을 확인하지 못했습니다. 터치 체험으로 같은 과정을 진행하세요.",
    "motion-denied": "자세 센서는 준비됐지만 가속도 권한이 거부되어 AX·AY·AZ는 표시하지 않습니다.",
    "motion-permission-error": "자세 센서는 준비됐지만 가속도 권한을 확인하지 못했습니다.",
    "motion-unavailable": "자세 센서는 준비됐지만 이 브라우저는 가속도 값을 제공하지 않습니다.",
    timeout: "센서값이 도착하지 않았습니다. 데스크톱이거나 센서가 비활성 상태일 수 있습니다.",
  };
  return messages[reason] ?? "스마트폰을 천천히 기울여 첫 센서값을 기다리고 있습니다.";
}

function clearSensorTimeout() {
  if (sensorTimeout) {
    window.clearTimeout(sensorTimeout);
    sensorTimeout = 0;
  }
}

function handleOrientation(event) {
  if (!Number.isFinite(event.beta) || !Number.isFinite(event.gamma)) return;
  orientationModel.updateOrientation(event);
  firstOrientationReceived = true;
  renderInstrument();
  if (sensorAccess?.orientation === "granted" && app.dataset.screen === "permission") {
    clearSensorTimeout();
    showScreen("calibration");
  }
}

function handleMotion(event) {
  orientationModel.updateMotion(event);
  renderInstrument();
}

function attachSensorListeners() {
  window.removeEventListener("deviceorientation", handleOrientation);
  window.removeEventListener("devicemotion", handleMotion);
  window.addEventListener("deviceorientation", handleOrientation);
  window.addEventListener("devicemotion", handleMotion);
}

function cancelSensorAttempt() {
  sensorAttemptGeneration += 1;
  clearSensorTimeout();
  sensorAccess = null;
  firstOrientationReceived = false;
  window.removeEventListener("deviceorientation", handleOrientation);
  window.removeEventListener("devicemotion", handleMotion);
}

async function beginSensorExperience() {
  cancelSensorAttempt();
  const attemptGeneration = sensorAttemptGeneration;
  const accessPromise = requestSensorAccess(window);
  showScreen("permission");
  sensorReason.textContent = "센서 권한을 확인하고 있습니다.";
  app.dataset.sensorState = "requesting";
  attachSensorListeners();
  const nextSensorAccess = await accessPromise;
  if (attemptGeneration !== sensorAttemptGeneration) return;
  sensorAccess = nextSensorAccess;

  if (sensorAccess.orientation !== "granted") {
    app.dataset.sensorState = "fallback";
    sensorReason.textContent = sensorReasonText(sensorAccess.reason);
    return;
  }

  setMode("motion");
  app.dataset.sensorState = "waiting";
  sensorReason.textContent = sensorReasonText(sensorAccess.reason);
  if (firstOrientationReceived) {
    showScreen("calibration");
    return;
  }

  sensorTimeout = window.setTimeout(() => {
    if (
      attemptGeneration !== sensorAttemptGeneration
      || app.dataset.screen !== "permission"
      || firstOrientationReceived
    ) return;
    app.dataset.sensorState = "fallback";
    sensorReason.textContent = sensorReasonText("timeout");
  }, SENSOR_SAMPLE_TIMEOUT_MS);
}

function resetTouchInput() {
  activePointer = null;
  pressedArrowKeys.clear();
  touchVector = { x: 0, y: 0, magnitude: 0 };
  joystickStick.style.transform = "translate(-50%, -50%)";
  renderInstrument();
}

function renderJoystickInput() {
  const rect = joystick.getBoundingClientRect();
  const radius = Math.min(rect.width, rect.height) / 2 - 23;
  joystickStick.style.transform =
    `translate(calc(-50% + ${touchVector.x * radius}px), calc(-50% + ${-touchVector.y * radius}px))`;
  renderInstrument();
}

function beginTouchExperience() {
  cancelSensorAttempt();
  setMode("touch");
  orientationModel.reset();
  touchVector = { x: 0, y: 0, magnitude: 0 };
  showScreen("imu");
  document.querySelector("[data-imu-title]").textContent = "조이스틱으로 축을 확인하세요";
  document.querySelector("[data-imu-guidance]").textContent =
    "왼쪽·오른쪽은 Roll, 위·아래는 Pitch입니다.";
  renderInstrument();
}

function updateJoystick(event) {
  if (activePointer !== event.pointerId) return;
  const rect = joystick.getBoundingClientRect();
  touchVector = joystickVector(event.clientX, event.clientY, rect);
  renderJoystickInput();
}

joystick.addEventListener("pointerdown", (event) => {
  if (activePointer !== null) return;
  pressedArrowKeys.clear();
  activePointer = event.pointerId;
  try {
    joystick.setPointerCapture(event.pointerId);
  } catch {
    // Pointer capture is an enhancement; the pointer state is still bounded below.
  }
  updateJoystick(event);
});

joystick.addEventListener("pointermove", updateJoystick);

for (const eventName of ["pointerup", "pointercancel", "lostpointercapture"]) {
  joystick.addEventListener(eventName, (event) => {
    if (activePointer !== event.pointerId) return;
    resetTouchInput();
  });
}

const arrowAxes = new Map([
  ["ArrowLeft", { x: -1, y: 0 }],
  ["ArrowRight", { x: 1, y: 0 }],
  ["ArrowUp", { x: 0, y: 1 }],
  ["ArrowDown", { x: 0, y: -1 }],
]);

function updateKeyboardInput() {
  let x = 0;
  let y = 0;
  for (const key of pressedArrowKeys) {
    const axis = arrowAxes.get(key);
    x += axis.x;
    y += axis.y;
  }
  const length = Math.hypot(x, y);
  if (length > 1) {
    x /= length;
    y /= length;
  }
  touchVector = { x, y, magnitude: Math.min(length, 1) };
  renderJoystickInput();
}

joystick.addEventListener("keydown", (event) => {
  if (!arrowAxes.has(event.key) || activePointer !== null) return;
  event.preventDefault();
  pressedArrowKeys.add(event.key);
  updateKeyboardInput();
});

joystick.addEventListener("keyup", (event) => {
  if (!arrowAxes.has(event.key)) return;
  event.preventDefault();
  pressedArrowKeys.delete(event.key);
  updateKeyboardInput();
});

joystick.addEventListener("blur", () => {
  if (pressedArrowKeys.size > 0) resetTouchInput();
});

function challengeInput() {
  if (mode === "touch") {
    return { roll: touchVector.x, pitch: touchVector.y };
  }
  const orientation = orientationModel.snapshot().orientation;
  return {
    roll: clamp(orientation.roll / 20, -1, 1),
    pitch: clamp(orientation.pitch / 20, -1, 1),
  };
}

function renderChallenge() {
  const remaining = Math.max(0, 20 - challenge.elapsed);
  document.querySelector("[data-challenge-time]").textContent = remaining.toFixed(1);
  document.querySelector("[data-live-score]").textContent = String(challenge.score).padStart(4, "0");
  renderInstrument();
}

function challengeTick(timestamp) {
  if (app.dataset.screen !== "challenge" || challenge.finished) return;
  if (lastFrameTime === null) lastFrameTime = timestamp;
  const elapsed = Math.min(Math.max(0, (timestamp - lastFrameTime) / 1000), MAX_FRAME_SECONDS);
  lastFrameTime = timestamp;
  accumulator += elapsed;

  while (accumulator + Number.EPSILON >= FIXED_STEP && !challenge.finished) {
    challenge = stepChallenge(challenge, challengeInput(), FIXED_STEP);
    accumulator -= FIXED_STEP;
  }
  renderChallenge();

  if (challenge.finished) {
    finishChallenge();
    return;
  }
  challengeFrame = window.requestAnimationFrame(challengeTick);
}

function createSubmissionId() {
  if (typeof crypto.randomUUID === "function") return crypto.randomUUID();
  const bytes = crypto.getRandomValues(new Uint8Array(16));
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  const hex = [...bytes].map((value) => value.toString(16).padStart(2, "0")).join("");
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
}

function finishChallenge() {
  const stability = challenge.elapsed > 0
    ? clamp(challenge.stabilityIntegral / challenge.elapsed, 0, 100)
    : 0;
  localResult = Object.freeze({
    submission_id: createSubmissionId(),
    nickname: nicknameInput.value.trim() || "익명",
    score: challenge.score,
    stability,
    duration_ms: Math.round(challenge.elapsed * 1000),
    mode,
  });

  const resultPanel = document.querySelector("[data-result]");
  resultPanel.dataset.submissionId = localResult.submission_id;
  document.querySelector("[data-result-score]").textContent = String(localResult.score);
  document.querySelector("[data-result-stability]").textContent = `${localResult.stability.toFixed(1)}%`;
  document.querySelector("[data-result-mode]").textContent =
    mode === "motion" ? "기기 센서 조작 결과" : "터치 조작 결과";
  document.querySelector("[data-submit-status]").textContent =
    "결과는 이 화면에 먼저 저장되었습니다. 점수판 제출은 선택입니다.";
  showScreen("result");
  renderInstrument();
}

function startChallenge({ restart = false } = {}) {
  if (challengeFrame) window.cancelAnimationFrame(challengeFrame);
  challenge = restart
    ? restartChallenge(challenge, { seed: nextChallengeSeed() })
    : createChallengeState({ seed: nextChallengeSeed() });
  accumulator = 0;
  lastFrameTime = null;
  localResult = null;
  delete document.querySelector("[data-result]").dataset.submissionId;
  resetTouchInput();
  showScreen("challenge");
  renderChallenge();
  challengeFrame = window.requestAnimationFrame(challengeTick);
}

async function submitLocalResult() {
  if (localResult === null) return;
  const status = document.querySelector("[data-submit-status]");
  status.textContent = "선택 점수판에 제출하고 있습니다.";
  const result = await submitScore(window.fetch.bind(window), localResult);
  if (result.status === "submitted") {
    status.textContent = "점수판에 제출했습니다. 로컬 결과도 그대로 유지됩니다.";
  } else if (result.status === "rejected") {
    status.textContent = "점수판이 제출을 받지 않았습니다. 로컬 결과는 저장되어 있습니다.";
  } else {
    status.textContent = "점수판에 연결할 수 없습니다. 로컬 결과는 저장되어 있습니다.";
  }
}

document.querySelector('[data-action="sensor"]').addEventListener("click", beginSensorExperience);
document.querySelector('[data-action="touch"]').addEventListener("click", beginTouchExperience);
document.querySelector('[data-action="touch-fallback"]').addEventListener("click", beginTouchExperience);
document.querySelector('[data-action="back-start"]').addEventListener("click", () => {
  cancelSensorAttempt();
  setMode("none");
  showScreen("start");
  renderInstrument();
});
document.querySelector('[data-action="calibrate"]').addEventListener("click", () => {
  orientationModel.calibrate();
  showScreen("imu");
  renderInstrument();
});
document.querySelector('[data-action="start-challenge"]').addEventListener("click", () => startChallenge());
document.querySelector('[data-action="restart"]').addEventListener("click", () => startChallenge({ restart: true }));
document.querySelector('[data-action="submit-score"]').addEventListener("click", submitLocalResult);

document.addEventListener("visibilitychange", () => {
  lastFrameTime = null;
});

showScreen("start");
renderInstrument();
