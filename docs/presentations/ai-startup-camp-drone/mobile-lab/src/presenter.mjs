const urlInput = document.querySelector("[data-student-url]");
const qrTarget = document.querySelector("[data-qr]");
const copyStatus = document.querySelector("[data-copy-status]");
const boardStatus = document.querySelector("[data-board-status]");
const scoreCount = document.querySelector("[data-score-count]");
const scoreList = document.querySelector("[data-score-list]");
const SCORE_REQUEST_TIMEOUT_MS = 5000;
const SCORE_POLL_INTERVAL_MS = 3000;
let scoreRequestActive = false;

function defaultStudentUrl() {
  const url = new URL("./index.html", window.location.href);
  url.search = "";
  url.hash = "";
  return url.href;
}

function renderQr(value) {
  let url;
  try {
    url = new URL(value, window.location.href).href;
  } catch {
    copyStatus.textContent = "올바른 학생 URL을 입력하세요.";
    return;
  }
  urlInput.value = url;
  const code = window.qrcode(0, "M");
  code.addData(url, "Byte");
  code.make();
  qrTarget.innerHTML = code.createSvgTag({
    cellSize: 4,
    margin: 16,
    scalable: true,
    title: "모바일 드론 랩 학생 접속 QR 코드",
    alt: "학생 접속 URL을 담은 QR 코드",
  });
  copyStatus.textContent = "QR이 현재 URL로 준비되었습니다.";
}

async function copyUrl() {
  const value = urlInput.value;
  try {
    await navigator.clipboard.writeText(value);
    copyStatus.textContent = "학생 URL을 복사했습니다.";
  } catch {
    urlInput.select();
    const copied = document.execCommand("copy");
    copyStatus.textContent = copied
      ? "학생 URL을 복사했습니다."
      : "자동 복사가 제한되었습니다. URL을 길게 눌러 복사하세요.";
  }
}

function renderScores(records) {
  if (records.length === 0) {
    scoreList.innerHTML = '<li class="score-empty">아직 제출된 점수가 없습니다</li>';
    return;
  }
  scoreList.replaceChildren(
    ...records.slice(0, 10).map((record) => {
      const item = document.createElement("li");
      const name = document.createElement("span");
      name.className = "score-name";
      name.textContent = record.nickname;
      const score = document.createElement("strong");
      score.className = "score-value";
      score.textContent = String(record.score).padStart(4, "0");
      item.append(name, score);
      return item;
    }),
  );
}

async function refreshScores() {
  if (scoreRequestActive) return;
  scoreRequestActive = true;
  const controller = typeof AbortController === "function" ? new AbortController() : null;
  let timeoutId;
  try {
    const request = (async () => {
      const response = await fetch("/api/scores", {
        cache: "no-store",
        ...(controller === null ? {} : { signal: controller.signal }),
      });
      if (!response.ok) throw new Error("score API unavailable");
      return response.json();
    })();
    const timeout = new Promise((_, reject) => {
      timeoutId = window.setTimeout(() => {
        controller?.abort();
        reject(new Error("score API timed out"));
      }, SCORE_REQUEST_TIMEOUT_MS);
    });
    const body = await Promise.race([request, timeout]);
    if (!Number.isInteger(body.count) || !Array.isArray(body.scores)) {
      throw new Error("invalid score response");
    }
    scoreCount.textContent = String(body.count);
    boardStatus.textContent = "선택 점수 서버와 연결되어 있습니다.";
    renderScores(body.scores);
  } catch {
    scoreCount.textContent = "0";
    boardStatus.textContent = "점수판은 선택 기능입니다. QR과 학생 실습은 그대로 사용할 수 있습니다.";
    renderScores([]);
  } finally {
    window.clearTimeout(timeoutId);
    scoreRequestActive = false;
  }
}

document.querySelector('[data-action="update-qr"]').addEventListener("click", () => {
  renderQr(urlInput.value);
});
document.querySelector('[data-action="copy-url"]').addEventListener("click", copyUrl);
urlInput.addEventListener("change", () => renderQr(urlInput.value));

urlInput.value = defaultStudentUrl();
renderQr(urlInput.value);
refreshScores();
window.setInterval(refreshScores, SCORE_POLL_INTERVAL_MS);
