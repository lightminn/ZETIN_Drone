function copyJson(value) {
  return JSON.parse(JSON.stringify(value));
}

export async function submitScore(fetchFn, payload) {
  try {
    const response = await fetchFn("/api/scores", {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = typeof response.json === "function" ? copyJson(await response.json()) : undefined;

    if (!response.ok) {
      return body === undefined ? { status: "rejected" } : { status: "rejected", response: body };
    }
    return body === undefined ? { status: "submitted" } : { status: "submitted", response: body };
  } catch {
    return { status: "offline" };
  }
}
