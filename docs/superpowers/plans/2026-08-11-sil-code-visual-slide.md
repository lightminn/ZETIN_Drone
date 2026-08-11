# SIL Code Visual Slide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실제 펌웨어와 SIL 하니스 코드가 1ms 폐루프를 만드는 방식을 발표자료에서 한눈에 설명한다.

**Architecture:** 기존 SIL 개념·물리 흐름 두 장 뒤에 코드 기반 시각자료 한 장을 추가한다. 새 장은 실제 `.ino` 포함, `vTaskDelayUntil()` 훅, `integratePlant()`, `injectImuFromPlant()`, `pid_task()`의 데이터 흐름과 S1 정상·부호 반전 실행 결과를 연결한다.

**Tech Stack:** HTML 기반 deck-stage, UOS 슬라이드 디자인 시스템, C++ native SIL, Python unittest, headless Chrome

## Global Constraints

- HTML 원본만 수정하며 PPTX와 PDF는 재생성하지 않는다.
- 실제 코드와 현재 실행 결과만 사용한다.
- 완전한 문장은 `~다/~이다`, 짧은 설명은 명사형으로 끝낸다.
- 현재 UOS 파란색 시각 체계와 1280×720 배치를 유지한다.
- 실제 비행 검증과 SIL 검증을 혼동하지 않는다.

---

### Task 1: SIL 코드 근거와 실행값 확정

**Files:**
- Read: `tools/native_tests/test_sil_attitude.cpp`
- Read: `tools/native_tests/shims/Arduino.h`
- Read: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Read: `tools/test_sil_attitude.py`

**Interfaces:**
- Consumes: 실제 `pid_task()`, `motorOut[4]`, Arduino shim tick hook
- Produces: 슬라이드에 사용할 함수명, 순서, 정상·고장 실행값

- [x] **Step 1: 실제 코드 연결점 확인**

  `#include "dual_imu_cascade_pwm.ino"`, `pre_tick_hook`, `integratePlant()`, `injectImuFromPlant()`, `pid_task()`의 호출 순서를 현재 코드에서 확인한다.

- [x] **Step 2: 현재 SIL 정상 경로 실행**

  Run: `/home/light/anaconda3/bin/python -m unittest tools.test_sil_attitude -v`

  Expected: exit 0, native attitude SIL 통과.

- [x] **Step 3: 정상·부호 반전 S1 값 수집**

  동일한 32비트 하니스를 정상 조건과 `-DSIL_INJECT_SIGN_FAULT` 조건으로 각각 실행한다.

  Expected: 정상은 S1 통과, 부호 반전은 S1 실패와 약 1,292° 롤 발산.

### Task 2: 발표자료에 코드 기반 시각자료 추가

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone/export_pptx.cjs`

**Interfaces:**
- Consumes: Task 1의 함수 연결과 실행값
- Produces: 74장 HTML 발표자료와 갱신된 슬라이드 번호·근거표

- [x] **Step 1: 실패 검증 실행**

  74장, 새 SIL 코드 슬라이드, 실제 함수명, 정상·고장 실행값을 요구하는 일회성 검사기를 실행한다.

  Expected: 새 슬라이드가 없으므로 실패.

- [x] **Step 2: 새 SIL 코드 슬라이드 추가**

  39번 뒤에 `실제 SIL의 1ms 폐루프` 슬라이드를 넣는다. 상단에는 실제 `.ino` 포함 관계, 중앙에는 `motorOut → integratePlant → injectImuFromPlant → pid_task` 순환, 하단에는 S1 정상·부호 반전 결과를 배치한다.

- [x] **Step 3: 후속 번호와 문서 갱신**

  40번 이후 슬라이드를 1씩 올리고, README 영상 위치, SOURCES 근거 번호, exporter의 `SLIDE_COUNT`를 74에 맞춘다.

- [x] **Step 4: 정적 검사 통과**

  Run: 74장 순번, 문체, 퀴즈 부재, 모바일 체험 경계, SOURCES 범위를 확인하는 Python 검사

  Expected: exit 0.

### Task 3: 실제 브라우저 검증과 버전 관리

**Files:**
- Test: `tools/test_presentation_video_autoplay.py`
- Test: `tools/test_presentation_launcher.py`

**Interfaces:**
- Consumes: Task 2의 74장 HTML 원본
- Produces: Chrome 렌더링 증거와 원격 Git 커밋

- [x] **Step 1: 발표 회귀 테스트 실행**

  Run: `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay tools.test_presentation_launcher`

  Expected: 5 tests pass.

- [x] **Step 2: Chrome 화면과 넘침 검사**

  새 SIL 슬라이드와 앞뒤 흐름을 1280×720에서 캡처하고, 74장 전체에 잘리지 않은 넘침이 없는지 확인한다.

- [x] **Step 3: 생성 산출물 비변경 확인**

  PPTX/PDF가 변경되지 않았는지 확인하고, 사용자 소유의 `docs/cascade_vs_single_pid.pdf`는 제외한다.

- [x] **Step 4: 관련 파일만 커밋·푸시**

  Commit message: `docs: show the native SIL control loop`

