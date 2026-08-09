import assert from "node:assert/strict";
import test from "node:test";

import { joystickVector } from "../src/joystick.mjs";

const rect = { left: 100, top: 50, width: 100, height: 100 };

test("joystickVector returns neutral data at the pointer-reset center", () => {
  assert.deepEqual(joystickVector(150, 100, rect), { x: 0, y: 0, magnitude: 0 });
});

test("joystickVector normalizes an edge vector", () => {
  assert.deepEqual(joystickVector(200, 100, rect), { x: 1, y: 0, magnitude: 1 });
  assert.deepEqual(joystickVector(150, 50, rect), { x: 0, y: 1, magnitude: 1 });
});

test("joystickVector clamps a pointer outside the circular control", () => {
  assert.deepEqual(joystickVector(250, 100, rect), { x: 1, y: 0, magnitude: 1 });
  assert.deepEqual(joystickVector(200, 50, rect), {
    x: 0.7071067811865475,
    y: 0.7071067811865475,
    magnitude: 1,
  });
});
