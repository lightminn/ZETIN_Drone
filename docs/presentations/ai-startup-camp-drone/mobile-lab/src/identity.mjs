const STORAGE_KEY = "uos-mobile-lab-anonymous-nickname-v1";
const ANONYMOUS_NICKNAME_PATTERN = /^익명-[0-9A-F]{8}$/;


function randomUint32(cryptoSource) {
  try {
    if (typeof cryptoSource?.getRandomValues === "function") {
      const values = new Uint32Array(1);
      cryptoSource.getRandomValues(values);
      return values[0];
    }
  } catch {
    // The alias is not a security credential; keep the no-login flow available.
  }
  return (Date.now() ^ Math.floor(Math.random() * 0x100000000)) >>> 0;
}


export function anonymousNickname(storage, cryptoSource) {
  try {
    const stored = storage?.getItem(STORAGE_KEY);
    if (ANONYMOUS_NICKNAME_PATTERN.test(stored ?? "")) return stored;
  } catch {
    // Storage can be disabled in private or locked-down browser modes.
  }

  const nickname = `익명-${randomUint32(cryptoSource).toString(16).padStart(8, "0").toUpperCase()}`;
  try {
    storage?.setItem(STORAGE_KEY, nickname);
  } catch {
    // The current page keeps the generated alias even when persistence is blocked.
  }
  return nickname;
}
