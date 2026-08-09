function copyJson(value) {
  return JSON.parse(JSON.stringify(value));
}

const SCORE_REQUEST_TIMEOUT_MS = 5000;

export async function submitScore(fetchFn, payload, { timeoutMs = SCORE_REQUEST_TIMEOUT_MS } = {}) {
  const requestTimeout = Number.isFinite(timeoutMs) && timeoutMs > 0
    ? timeoutMs
    : SCORE_REQUEST_TIMEOUT_MS;
  const controller = typeof AbortController === "function" ? new AbortController() : null;
  let timeoutId;
  try {
    const request = (async () => {
      const response = await fetchFn("/api/scores", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify(payload),
        ...(controller === null ? {} : { signal: controller.signal }),
      });
      const body = typeof response.json === "function" ? copyJson(await response.json()) : undefined;

      if (!response.ok) {
        return body === undefined ? { status: "rejected" } : { status: "rejected", response: body };
      }
      return body === undefined ? { status: "submitted" } : { status: "submitted", response: body };
    });
    const timeout = new Promise((_, reject) => {
      timeoutId = setTimeout(() => {
        controller?.abort();
        reject(new Error("score request timed out"));
      }, requestTimeout);
    });
    return await Promise.race([request(), timeout]);
  } catch {
    return { status: "offline" };
  } finally {
    clearTimeout(timeoutId);
  }
}
