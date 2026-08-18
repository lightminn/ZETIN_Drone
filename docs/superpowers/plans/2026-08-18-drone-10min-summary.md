# Drone 10-Minute Summary Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 기존 드론 발표를 기술 중심 14장 요약본으로 재구성하고 영상 포함 PPTX를 생성한다.

**Architecture:** 새 발표 폴더에 HTML 기준본과 선택 자산을 독립적으로 보관한다. 기존 `deck-stage`와 UOS 슬라이드 디자인 시스템을 재사용하고, 브라우저 캡처 기반 변환기로 화면·발표자 노트·H.264 영상을 PPTX에 패키징한다.

**Tech Stack:** HTML/CSS, UOS slide design system, Chrome DevTools Protocol, PptxGenJS, Python unittest, LibreOffice

**Spec:** `docs/presentations/ai-startup-camp-drone-10min/BRIEF.md`

## Global Constraints

- 14장 16:9, 기술 중심 구성
- 팀 이름, 구체적인 날짜, 커밋 해시, 발표·시연 시간 미표기
- 실제 기체·SIL·계획의 증거 수준 구분
- 기존 84장 기준본과 사용자 소유 dirty 파일 미수정

---

### Task 1: 요약본 계약과 독립 실행 환경

**Files:**
- Create: `tools/test_presentation_10min.py`
- Create: `docs/presentations/ai-startup-camp-drone-10min/index.html`
- Create: `docs/presentations/ai-startup-camp-drone-10min/present.sh`
- Create: `docs/presentations/ai-startup-camp-drone-10min/deck-stage.js`
- Create: `docs/presentations/ai-startup-camp-drone-10min/support.js`
- Create: `docs/presentations/ai-startup-camp-drone-10min/vendor/`

**Interfaces:**
- Consumes: 기존 발표의 `deck-stage`와 UOS 디자인 시스템
- Produces: 브라우저에서 14장으로 로드되는 독립 HTML 발표자료

- [x] **Step 1: Write the failing test**

`tools/test_presentation_10min.py`에서 14장, 모든 발표자 노트, 금지어 부재, 정확히 한 개의 자동재생용 영상, 필수 실제 자산을 검사한다.

- [x] **Step 2: Run test to verify it fails**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v`

Expected: FAIL because `index.html` does not exist.

- [x] **Step 3: Write minimal implementation**

참고 PPTX의 ‘제목-증거-하단 결론’ 구조를 적용해 14장 HTML을 만들고 로컬 실행 파일을 복사한다.

- [x] **Step 4: Run test to verify it passes**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v`

Expected: PASS.

### Task 2: PPTX 변환과 산출물 검증

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone-10min/export_pptx.cjs`
- Create: `docs/presentations/ai-startup-camp-drone-10min/드론_10분_요약본.pptx`
- Modify: `tools/test_presentation_10min.py`

**Interfaces:**
- Consumes: 14장 HTML, H.264 실기 영상, 발표자 노트
- Produces: 14장·14개 노트·영상 1개를 포함한 와이드 PPTX

- [x] **Step 1: Write the failing test**

PPTX의 장수, 화면 비율, 노트 수, 포함된 H.264 영상 수를 검사한다.

- [x] **Step 2: Run test to verify it fails**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min.Presentation10MinutePptxTests -v`

Expected: FAIL because the PPTX does not exist.

- [x] **Step 3: Write minimal implementation**

기존 변환기를 14장·영상 1개·중립 메타데이터에 맞춰 복제하고 PPTX를 생성한다.

- [x] **Step 4: Run test to verify it passes**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v`

Expected: PASS.

### Task 3: 전 슬라이드 렌더 검수

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone-10min/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone-10min/드론_10분_요약본.pptx`

**Interfaces:**
- Consumes: HTML 기준본과 생성된 PPTX
- Produces: 겹침·잘림·잘못된 도식이 없는 최종 요약본

- [x] **Step 1: Render the real artifact**

PPTX를 PDF와 PNG로 변환하고 14장 contact sheet를 만든다.

- [x] **Step 2: Inspect and correct**

제목, 본문 글자, SVG 방향, 실제 사진/그래프 캡션, 영상 포스터를 전수 확인한다.

- [x] **Step 3: Run full verification**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v`

Expected: PASS with 0 failures.
