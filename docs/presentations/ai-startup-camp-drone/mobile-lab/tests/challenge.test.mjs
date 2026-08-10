import assert from "node:assert/strict";
import test from "node:test";

import {
  createChallengeState,
  restartChallenge,
  stepChallenge,
} from "../src/challenge.mjs";

const CALIBRATION_SEEDS = [
  0x10203040, 0x1badb002, 0x31415926, 0x5eed1234,
  0x7f4a7c15, 0x89abcdef, 0xc001d00d, 0xf00dcafe,
];

const FIXED_STEP = 1 / 60;

function runSequence(sequence, seed) {
  return sequence.reduce(
    (state, input) => stepChallenge(state, input, 1 / 60),
    createChallengeState({ seed }),
  );
}

function runSeed(seed, inputForState = () => ({})) {
  let state = createChallengeState({ seed });
  const history = [];
  let maxTilt = 0;

  for (let index = 0; index < 1200; index += 1) {
    const previousState = state;
    const beforeUpdate = structuredClone(previousState);
    state = stepChallenge(previousState, inputForState(previousState), FIXED_STEP);
    assert.deepEqual(previousState, beforeUpdate);
    history.push([state.roll, state.pitch]);
    maxTilt = Math.max(maxTilt, Math.hypot(state.roll, state.pitch));
  }

  const elapsedMs = state.elapsed * 1000;
  assert.equal(elapsedMs, 20_000, `seed ${seed.toString(16)} calibration duration`);
  assert.equal(state.finished, true, `seed ${seed.toString(16)} calibration completion`);
  return { state, history, maxTilt, elapsedMs };
}

test("identical seeded fixed-step inputs produce byte-for-byte equal terminal states", () => {
  const sequence = Array.from({ length: 1200 }, (_, index) => ({
    roll: index % 120 < 60 ? 0.35 : -0.35,
    pitch: index % 90 < 45 ? -0.2 : 0.2,
  }));
  const first = runSequence(sequence, CALIBRATION_SEEDS[0]);
  const second = runSequence(sequence, CALIBRATION_SEEDS[0]);

  assert.equal(first.elapsed, 20);
  assert.equal(first.finished, true);
  assert.equal(JSON.stringify(first), JSON.stringify(second));
});

test("different seeds generate different Roll/Pitch histories", () => {
  const first = runSeed(CALIBRATION_SEEDS[0]);
  const distinctHistory = CALIBRATION_SEEDS.slice(1).some((seed) => {
    const candidate = runSeed(seed);
    return JSON.stringify(candidate.history) !== JSON.stringify(first.history);
  });

  assert.equal(distinctHistory, true);
});

test("calibration seed bank remains difficult but playable without input", () => {
  const results = CALIBRATION_SEEDS.map((seed) => runSeed(seed));
  const scores = results.map(({ state }) => state.score);
  const meanScore = scores.reduce((total, score) => total + score, 0) / scores.length;

  assert.ok(meanScore >= 400 && meanScore <= 550, `mean no-input score ${meanScore}`);
  for (const score of scores) {
    assert.ok(score >= 300 && score <= 650, `no-input score ${score}`);
  }
  assert.ok(results.some(({ maxTilt }) => maxTilt > 8), "a seed must exceed 8 degrees");
});

test("literal recovery pilot improves every calibration seed", () => {
  const noInput = CALIBRATION_SEEDS.map((seed) => runSeed(seed).state.score);
  const pilot = CALIBRATION_SEEDS.map((seed) => runSeed(seed, (state) => ({
    roll: Math.max(-1, Math.min(1, -0.075 * state.roll - 0.12 * state.rollRate)),
    pitch: Math.max(-1, Math.min(1, -0.075 * state.pitch - 0.12 * state.pitchRate)),
  })).state.score);
  const pilotMean = pilot.reduce((total, score) => total + score, 0) / pilot.length;

  assert.ok(pilotMean >= 700, `mean pilot score ${pilotMean}`);
  pilot.forEach((score, index) => {
    assert.ok(score >= noInput[index] + 180, `seed ${CALIBRATION_SEEDS[index].toString(16)}`);
  });
});

test("challenge reaches a 20-second terminal state and ignores further updates", () => {
  const done = stepChallenge(createChallengeState(), { roll: 1, pitch: -1 }, 20);

  assert.equal(done.elapsed, 20);
  assert.equal(done.finished, true);
  assert.ok(done.score >= 0 && done.score <= 1000);
  assert.equal(stepChallenge(done, { roll: -1, pitch: 1 }, 1 / 60), done);
});

test("restartChallenge clears transient state and normalizes the replacement seed", () => {
  let active = createChallengeState({ seed: CALIBRATION_SEEDS[0] });
  for (let index = 0; index < 180; index += 1) {
    active = stepChallenge(active, { roll: 0.7, pitch: -0.4 }, FIXED_STEP);
  }
  const restarted = restartChallenge(active, { seed: -1 });

  assert.deepEqual(restarted, createChallengeState({ seed: 0xffffffff }));
  assert.equal(restarted.elapsed, 0);
  assert.equal(restarted.rollRate, 0);
  assert.equal(restarted.pitchRate, 0);
  assert.equal(restarted.windRoll, 0);
  assert.equal(restarted.windPitch, 0);
  assert.equal(restarted.gustRoll, 0);
  assert.equal(restarted.gustPitch, 0);
  assert.equal(restarted.gustRemaining, 0);
  assert.equal(restarted.score, 0);
  assert.deepEqual(restarted.input, { roll: 0, pitch: 0 });
  assert.equal(createChallengeState({ seed: -0 }).seed, 0x6d2b79f5);
});
