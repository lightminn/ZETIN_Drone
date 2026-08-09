import assert from "node:assert/strict";
import test from "node:test";

import {
  accumulateScore,
  calculateScore,
  calculateStability,
} from "../src/scoring.mjs";

test("calculateStability rewards level, still attitude and bounds unstable input", () => {
  assert.equal(calculateStability({ roll: 0, pitch: 0, rollRate: 0, pitchRate: 0 }), 100);
  assert.equal(calculateStability({ roll: 25, pitch: 0, rollRate: 0, pitchRate: 0 }), 50);
  assert.equal(calculateStability({ roll: 90, pitch: 90, rollRate: 50, pitchRate: 50 }), 0);
});

test("accumulateScore returns new elapsed stability metrics without mutating the old metrics", () => {
  const metrics = { stabilityIntegral: 25, elapsed: 0.5 };
  const next = accumulateScore(metrics, 80, 0.25);

  assert.deepEqual(next, { stabilityIntegral: 45, elapsed: 0.75 });
  assert.deepEqual(metrics, { stabilityIntegral: 25, elapsed: 0.5 });
});

test("calculateScore rounds averaged stability and clamps the 0 to 1000 range", () => {
  assert.equal(calculateScore({ stabilityIntegral: 87.65, elapsed: 1 }), 877);
  assert.equal(calculateScore({ stabilityIntegral: -5, elapsed: 1 }), 0);
  assert.equal(calculateScore({ stabilityIntegral: 500, elapsed: 1 }), 1000);
  assert.equal(calculateScore({ stabilityIntegral: 50, elapsed: 0 }), 0);
});
