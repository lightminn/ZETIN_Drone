# Drone Deck Professor Feedback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` and execute tasks sequentially because both decks share
> the same evidence model and presentation tests.

**Goal:** 교수 피드백을 근거 중심의 시각 자료로 반영하고, 84장 본편과 14장 결과발표를
학습 목적·디버깅·래피드 프로토타이핑·제작 규모·단계별 목표가 명확한 자료로 개편한다.

**Architecture:** HTML을 편집 기준본으로 유지한다. 변동 가능한 시간·비용은 별도 근거
문서에서 계산하고, Python/Manim 정지 PNG가 그 데이터를 읽어 같은 기준선의 비교 도판을
만든다. 두 HTML은 해당 PNG와 실제 사진을 사용하고 PPTX/PDF는 생성하지 않는다.

**Tech Stack:** HTML/CSS, UOS slide design system, Python/Manim, Chrome DevTools Protocol,
Python unittest

**Spec:** `docs/superpowers/specs/2026-08-19-drone-deck-professor-feedback-design.md`

## Global Constraints

- 본편 84장, 결과발표 14장 유지
- HTML·로컬 자산만 갱신, PPTX/PDF 미생성
- 시간·비용은 범위와 가정이 있는 예비 산정
- 실제 사진 우선, 개념 비교는 정지 PNG
- 사용자 소유 dirty 파일 보존, 경로 제한 staging

### Task 1: 제작 규모 근거와 정지 도판

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/PRODUCTION_ESTIMATE.md`
- Create: `docs/presentations/ai-startup-camp-drone/production_estimate.json`
- Modify: `docs/presentations/ai-startup-camp-drone/visualizations/static_diagram_visualizations.py`
- Modify: `docs/presentations/ai-startup-camp-drone/visualizations/render_visualizations.sh`
- Create: `docs/presentations/ai-startup-camp-drone/assets/production-estimate.png`
- Modify: `tools/test_presentation_visualizations.py`

- [x] **Step 1: Write the failing tests**

근거 문서에 시간의 장비 점유/직접 작업 구분, 1대/10대 범위, 비용 범주, 가정과 제외
항목이 모두 있는지 검사한다. 렌더 장면은 1280×720, 1대와 10대의 같은 축, 비용 누적
막대, 최소 글자 크기와 필수 라벨을 검사한다.

- [x] **Step 2: Run tests to verify RED**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_visualizations -v`

- [x] **Step 3: Implement the evidence model and diagram**

저장소 BOM과 현재 판매 근거를 구분해 범위를 기록하고, 슬라이드에서는 간단한 범위
막대와 가정 한 줄만 보이도록 렌더한다.

- [x] **Step 4: Render and verify GREEN**

PNG 원본과 1280×720 캡처를 확인하고, 라벨 또는 데이터 하나를 변이해 회귀 테스트가
RED가 되는지 확인한 뒤 복구한다.

### Task 2: 본편 교수 피드백 반영

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`
- Modify: `tools/test_presentation_video_autoplay.py`

- [ ] **Step 1: Write the failing content and browser tests**

14·22·25·43·83장의 제목·필수 메시지·자산, 84장 수, 금지 표현, 글자 경계와 1280×720
레이아웃을 고정한다.

- [ ] **Step 2: Run focused tests to verify RED**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay -v`

- [ ] **Step 3: Implement the five slide replacements**

상용/자작 목적, 제작 규모, 래피드 프로토타이핑, 디버깅 순환, 단·중·장기 목표를
실제 사진과 정지 도판 중심으로 배치한다.

- [ ] **Step 4: Verify browser rendering and mutation**

다섯 장을 1280×720로 캡처해 전수 확인하고, 필수 메시지·자산 변이 하나가 RED인 것을
확인한 뒤 복구한다.

### Task 3: 10분 결과발표 개편

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone-10min/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone-10min/BRIEF.md`
- Modify: `docs/presentations/ai-startup-camp-drone-10min/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone-10min/README.md`
- Modify: `tools/test_presentation_10min.py`

- [ ] **Step 1: Write the failing result-deck tests**

14장, 결과 중심 흐름, 상용/자작 목적 구분, 래피드 프로토타이핑, 제작 규모 요약,
문제 추적, 상대 시간척도, 조건 기반 안전, 단·중·장기 목표를 검사한다.

- [ ] **Step 2: Run tests to verify RED**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v`

- [ ] **Step 3: Rework slides 2, 4, 7, 9, 11, 13, 14**

기존 실제 사진·그래프·실기 영상을 유지하면서 문구와 레이아웃을 결과 발표용으로
압축한다. 고정된 보편 숫자는 화면에서 제거하고 근거 문서로 옮긴다.

- [ ] **Step 4: Verify the 14-slide contact sheet**

Chrome 렌더 전체를 확인하고 필수 메시지 변이 proof 후 복구한다.

### Task 4: 통합 검증과 문서 정합성

**Files:**
- Modify only if needed: both deck READMEs, SOURCES, tests, progress ledger

- [ ] **Step 1: Run full presentation suites**

Run the visualization, main deck, and 10-minute deck test modules. Do not use stale PPTX/PDF
success as proof of current HTML content.

- [ ] **Step 2: Inspect final screenshots**

본편 84장과 결과발표 14장의 contact sheet, 수정 대상 원본 캡처, 실제 영상 자동재생과
정지 PNG 로드를 확인한다.

- [ ] **Step 3: Independent review**

기술 사실, 근거 경계, 비용·시간 가정, 문장 어투, 시각 밀도를 독립 검수하고 Important
이상을 모두 수정한다.

- [ ] **Step 4: Commit only intended files**

사용자 소유 PDF·ZIP·media와 기타 dirty 파일을 stage하지 않는다. 최종 보고에서 HTML이
최신이며 PPTX/PDF는 의도적으로 재생성하지 않았다고 명시한다.
