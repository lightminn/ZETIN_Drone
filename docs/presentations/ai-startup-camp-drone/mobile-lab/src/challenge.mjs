import {
  accumulateScore,
  calculateScore,
  calculateStability,
} from "./scoring.mjs";

const CHALLENGE_DURATION = 20;
const INPUT_TORQUE = 18;
const RESTORING_FORCE = 0.7;
const DAMPING = 0.9;
const ANGLE_LIMIT = 90;
const DEFAULT_SEED = 0x6d2b79f5;
const WIND_DECAY = 0.1;
const WIND_TURBULENCE = 12.5;
const WIND_LIMIT = 14;
const GUST_RATE = 0.18;
const GUST_MIN_DURATION = 0.6;
const GUST_DURATION_RANGE = 0.6;
const GUST_MIN_FORCE = 18;
const GUST_FORCE_RANGE = 14;
const CROSS_AXIS_WAVE = 1.5;

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function inputAxis(value) {
  return clamp(Number.isFinite(value) ? value : 0, -1, 1);
}

function normalizedSeed(seed) {
  return (Number.isFinite(seed) ? seed : DEFAULT_SEED) >>> 0 || DEFAULT_SEED;
}

function nextRandom(state) {
  let next = state >>> 0;
  next ^= next << 13;
  next ^= next >>> 17;
  next ^= next << 5;
  next >>>= 0;
  return { state: next || DEFAULT_SEED, value: next / 0x100000000 };
}

function updateWind(state, duration) {
  const rollNoise = nextRandom(state.randomState);
  const pitchNoise = nextRandom(rollNoise.state);
  const gustChance = nextRandom(pitchNoise.state);
  let randomState = gustChance.state;
  const windDecay = Math.exp(-WIND_DECAY * duration);
  const noiseScale = WIND_TURBULENCE * Math.sqrt(duration);
  let gustRoll = state.gustRoll;
  let gustPitch = state.gustPitch;
  let gustRemaining = Math.max(0, state.gustRemaining - duration);
  let gustDuration = state.gustDuration;

  if (gustRemaining === 0) {
    gustRoll = 0;
    gustPitch = 0;
    gustDuration = 0;
    if (gustChance.value < 1 - Math.exp(-GUST_RATE * duration)) {
      const gustDurationRandom = nextRandom(randomState);
      const gustRollRandom = nextRandom(gustDurationRandom.state);
      const gustPitchRandom = nextRandom(gustRollRandom.state);
      randomState = gustPitchRandom.state;
      gustDuration = GUST_MIN_DURATION + GUST_DURATION_RANGE * gustDurationRandom.value;
      gustRemaining = gustDuration;
      gustRoll = (gustRollRandom.value * 2 - 1) * (GUST_MIN_FORCE + GUST_FORCE_RANGE * gustPitchRandom.value);
      gustPitch = (gustPitchRandom.value * 2 - 1) * (GUST_MIN_FORCE + GUST_FORCE_RANGE * gustRollRandom.value);
    }
  }

  const gustEnvelope = gustDuration > 0
    ? Math.sin(Math.PI * (1 - gustRemaining / gustDuration))
    : 0;
  return {
    randomState,
    windRoll: clamp(
      state.windRoll * windDecay + (rollNoise.value * 2 - 1) * noiseScale,
      -WIND_LIMIT,
      WIND_LIMIT,
    ),
    windPitch: clamp(
      state.windPitch * windDecay + (pitchNoise.value * 2 - 1) * noiseScale,
      -WIND_LIMIT,
      WIND_LIMIT,
    ),
    gustRoll,
    gustPitch,
    gustRemaining,
    gustDuration,
    gustEnvelope,
  };
}

export function createChallengeState({ seed } = {}) {
  const normalized = normalizedSeed(seed);
  return {
    seed: normalized,
    randomState: normalized,
    elapsed: 0,
    roll: 0,
    pitch: 0,
    rollRate: 0,
    pitchRate: 0,
    windRoll: 0,
    windPitch: 0,
    gustRoll: 0,
    gustPitch: 0,
    gustRemaining: 0,
    gustDuration: 0,
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
  const wind = updateWind(state, duration);
  const elapsed = duration === remaining ? CHALLENGE_DURATION : state.elapsed + duration;
  const force = {
    roll: wind.windRoll + CROSS_AXIS_WAVE * Math.sin(0.9 * elapsed + wind.windPitch)
      + wind.gustRoll * wind.gustEnvelope,
    pitch: wind.windPitch + CROSS_AXIS_WAVE * Math.sin(1.1 * elapsed + wind.windRoll)
      + wind.gustPitch * wind.gustEnvelope,
  };
  const rollRate = state.rollRate + duration * (
    INPUT_TORQUE * command.roll + force.roll - RESTORING_FORCE * state.roll - DAMPING * state.rollRate
  );
  const pitchRate = state.pitchRate + duration * (
    INPUT_TORQUE * command.pitch + force.pitch - RESTORING_FORCE * state.pitch - DAMPING * state.pitchRate
  );
  const roll = clamp(state.roll + duration * rollRate, -ANGLE_LIMIT, ANGLE_LIMIT);
  const pitch = clamp(state.pitch + duration * pitchRate, -ANGLE_LIMIT, ANGLE_LIMIT);
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
    randomState: wind.randomState,
    windRoll: wind.windRoll,
    windPitch: wind.windPitch,
    gustRoll: wind.gustRoll,
    gustPitch: wind.gustPitch,
    gustRemaining: wind.gustRemaining,
    gustDuration: wind.gustDuration,
    score: calculateScore(metrics),
    finished: elapsed === CHALLENGE_DURATION,
    input: command,
  };
}

export function restartChallenge(previousState, { seed } = {}) {
  return createChallengeState({ seed: seed === undefined ? previousState?.seed : seed });
}
