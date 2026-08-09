import {
  accumulateScore,
  calculateScore,
  calculateStability,
} from "./scoring.mjs";

const CHALLENGE_DURATION = 20;
const INPUT_TORQUE = 18;
const RESTORING_FORCE = 0.9;
const DAMPING = 1.2;
const ANGLE_LIMIT = 90;

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function inputAxis(value) {
  return clamp(Number.isFinite(value) ? value : 0, -1, 1);
}

function disturbance(elapsed) {
  return {
    roll: 1.4 * Math.sin(2.1 * elapsed) + 0.6 * Math.sin(0.7 * elapsed),
    pitch: 1.1 * Math.sin(1.7 * elapsed + 0.8) + 0.5 * Math.sin(0.5 * elapsed),
  };
}

export function createChallengeState() {
  return {
    elapsed: 0,
    roll: 0,
    pitch: 0,
    rollRate: 0,
    pitchRate: 0,
    stabilityIntegral: 0,
    score: 0,
    finished: false,
    input: { roll: 0, pitch: 0 },
  };
}

export function stepChallenge(state, input = {}, dt) {
  if (state.finished) return state;

  const remaining = CHALLENGE_DURATION - state.elapsed;
  const duration = Math.min(Math.max(0, Number.isFinite(dt) ? dt : 0), remaining);
  if (duration === 0) return state;

  const command = { roll: inputAxis(input.roll), pitch: inputAxis(input.pitch) };
  const force = disturbance(state.elapsed + duration);
  const rollRate = state.rollRate + duration * (
    INPUT_TORQUE * command.roll + force.roll - RESTORING_FORCE * state.roll - DAMPING * state.rollRate
  );
  const pitchRate = state.pitchRate + duration * (
    INPUT_TORQUE * command.pitch + force.pitch - RESTORING_FORCE * state.pitch - DAMPING * state.pitchRate
  );
  const roll = clamp(state.roll + duration * rollRate, -ANGLE_LIMIT, ANGLE_LIMIT);
  const pitch = clamp(state.pitch + duration * pitchRate, -ANGLE_LIMIT, ANGLE_LIMIT);
  const elapsed = duration === remaining ? CHALLENGE_DURATION : state.elapsed + duration;
  const metrics = accumulateScore(
    state,
    calculateStability({ roll, pitch, rollRate, pitchRate }),
    duration,
  );

  return {
    ...state,
    ...metrics,
    elapsed,
    roll,
    pitch,
    rollRate,
    pitchRate,
    score: calculateScore(metrics),
    finished: elapsed === CHALLENGE_DURATION,
    input: command,
  };
}

export function restartChallenge() {
  return createChallengeState();
}
