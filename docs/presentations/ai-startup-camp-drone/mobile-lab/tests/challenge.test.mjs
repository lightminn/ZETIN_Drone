import assert from "node:assert/strict";
import test from "node:test";

import {
  createChallengeState,
  restartChallenge,
  stepChallenge,
} from "../src/challenge.mjs";

function runSequence(sequence) {
  return sequence.reduce(
    (state, input) => stepChallenge(state, input, 1 / 60),
    createChallengeState(),
  );
}

test("identical fixed-step inputs produce byte-for-byte equal terminal states", () => {
  const sequence = Array.from({ length: 1200 }, (_, index) => ({
    roll: index % 120 < 60 ? 0.35 : -0.35,
    pitch: index % 90 < 45 ? -0.2 : 0.2,
  }));
  const first = runSequence(sequence);
  const second = runSequence(sequence);

  assert.equal(first.elapsed, 20);
  assert.equal(first.finished, true);
  assert.equal(JSON.stringify(first), JSON.stringify(second));
});

test("challenge reaches a 20-second terminal state and ignores further updates", () => {
  const done = stepChallenge(createChallengeState(), { roll: 1, pitch: -1 }, 20);

  assert.equal(done.elapsed, 20);
  assert.equal(done.finished, true);
  assert.ok(done.score >= 0 && done.score <= 1000);
  assert.equal(stepChallenge(done, { roll: -1, pitch: 1 }, 1 / 60), done);
});

test("restartChallenge clears elapsed physics score and stored input", () => {
  const active = stepChallenge(createChallengeState(), { roll: 0.7, pitch: -0.4 }, 1);

  assert.deepEqual(restartChallenge(active), createChallengeState());
});
