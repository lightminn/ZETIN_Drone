import assert from "node:assert/strict";
import test from "node:test";

import { submitScore } from "../src/score-client.mjs";

test("submitScore sends a JSON copy and returns the accepted server response", async () => {
  const payload = { score: 876, nickname: "하늘01" };
  const result = await submitScore(async (url, options) => {
    assert.equal(url, "/api/scores");
    assert.equal(options.method, "POST");
    assert.equal(options.headers["content-type"], "application/json");
    assert.deepEqual(JSON.parse(options.body), payload);
    return { ok: true, json: async () => ({ accepted: true, score: 876 }) };
  }, payload);

  assert.deepEqual(result, { status: "submitted", response: { accepted: true, score: 876 } });
  assert.deepEqual(payload, { score: 876, nickname: "하늘01" });
});

test("submitScore resolves rejected fetches as offline without changing the caller payload", async () => {
  const payload = { score: 500, result: { stability: 50 } };
  const before = structuredClone(payload);

  assert.deepEqual(
    await submitScore(() => Promise.reject(new TypeError("offline")), payload),
    { status: "offline" },
  );
  assert.deepEqual(payload, before);
});

test("submitScore keeps a local result object independent from later response changes", async () => {
  const serverResponse = { accepted: true };
  const result = await submitScore(
    async () => ({ ok: true, json: async () => serverResponse }),
    { score: 1 },
  );
  serverResponse.accepted = false;

  assert.deepEqual(result, { status: "submitted", response: { accepted: true } });
});
