import assert from "node:assert/strict";
import test from "node:test";

import { anonymousNickname } from "../src/identity.mjs";


function deterministicCrypto(value) {
  return {
    draws: 0,
    getRandomValues(target) {
      this.draws += 1;
      target[0] = value;
      return target;
    },
  };
}


test("anonymousNickname creates and stores a literal eight-digit browser alias", () => {
  const values = new Map();
  const storage = {
    getItem: (key) => values.get(key) ?? null,
    setItem: (key, value) => values.set(key, value),
  };
  const cryptoSource = deterministicCrypto(0x12ab34cd);

  assert.equal(anonymousNickname(storage, cryptoSource), "익명-12AB34CD");
  assert.equal([...values.values()][0], "익명-12AB34CD");
  assert.equal(cryptoSource.draws, 1);
});


test("anonymousNickname reuses a valid stored alias without drawing randomness", () => {
  const cryptoSource = deterministicCrypto(0xffffffff);
  const storage = {
    getItem: () => "익명-89ABCDEF",
    setItem: () => assert.fail("a valid alias must not be overwritten"),
  };

  assert.equal(anonymousNickname(storage, cryptoSource), "익명-89ABCDEF");
  assert.equal(cryptoSource.draws, 0);
});


test("anonymousNickname replaces malformed storage and survives blocked storage", () => {
  let replacement = null;
  const cryptoSource = deterministicCrypto(7);
  const malformedStorage = {
    getItem: () => "익명-123",
    setItem: (_key, value) => { replacement = value; },
  };
  assert.equal(anonymousNickname(malformedStorage, cryptoSource), "익명-00000007");
  assert.equal(replacement, "익명-00000007");

  const blockedStorage = {
    getItem: () => { throw new Error("blocked"); },
    setItem: () => { throw new Error("blocked"); },
  };
  assert.equal(
    anonymousNickname(blockedStorage, deterministicCrypto(0x42)),
    "익명-00000042",
  );
});
