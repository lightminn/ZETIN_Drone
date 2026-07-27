# Failsafe 자동착륙 — 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** RC 링크 두절 시 즉시 모터를 끄는 대신, 트림을 유지한 자세로 천천히 하강해 착륙한다.

**Architecture:** 착지 감지와 상태 전이를 순수 함수로 `failsafe_land.h`에 분리해 플랜트 없이 단위 테스트한다(기존 `yaw_command.h` 패턴). 트림을 지상국에서 펌웨어로 옮겨 하강 목표 자세가 실제 수평이 되게 한다. `pid_task`는 헬퍼를 호출하고 상태를 배선하기만 한다.

**Tech Stack:** Arduino/ESP32-S3 (arduino-cli), C++17 호스트 네이티브 테스트(g++ `-m32` + shims), Python 3 unittest, pygame.

## Global Constraints

- 설계 근거: `docs/superpowers/specs/2026-07-27-failsafe-autoland-design.md`
- 자동착륙은 **RC 타임아웃에만** 적용. IMU 전멸·과도 기울기·`stop`은 즉시 컷 유지
- `FS_DESCENT_DELTA_US = 60` (벤치 확정 대상), **반드시 `CTRL_MARGIN`(150) 미만**
- `FS_MIN_DESCEND_MS = 1000`, `FS_LAND_ACCEL_TOL_G = 0.05f`, `FS_LAND_CONFIRM_MS = 400`
- `FS_MAX_MS = 5000` — 백스톱. 벤치 검증 후 10000으로 올린다
- `TRIM_MAX_DEG = 10.0f`
- 텔레메트리는 **append-only**. 35 → 38필드, CSV 36 → 39열
- **하강 중 주기적 Serial 출력 금지** (워치독 재부팅 위험, 스펙 §1)
- 각 태스크 종료 시 `python tools/check_repo_layout.py`와 전체 unittest 통과 필수
- python은 `/home/light/anaconda3/bin/python`을 쓴다

## File Structure

| 파일 | 책임 |
|---|---|
| `firmware/flight/dual_imu_cascade_pwm/failsafe_land.h` (신규) | 착지 감지기 + 하강 스로틀 + 종료 판정. 순수 함수만 |
| `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | 트림 명령·합성, 상태기계 배선, 중단 경로, 텔레메트리 |
| `tools/native_tests/test_failsafe_land.cpp` (신규) | `failsafe_land.h` 단위 테스트 |
| `tools/test_failsafe_land.py` (신규) | 위 네이티브 테스트의 unittest 래퍼 |
| `tools/native_tests/test_control_math.cpp` | 트림 명령·합성, 상태 전이 통합 테스트 |
| `scripts/telemetry_schema.py` | `Failsafe_Phase`, `Trim_Roll`, `Trim_Pitch` |
| `scripts/control_dualsense.py` | 트림 절대값 전송, `stop` 전송 경로 2곳 제거 |

---

## Task 1: `failsafe_land.h` 순수 헬퍼

**Files:**
- Create: `firmware/flight/dual_imu_cascade_pwm/failsafe_land.h`
- Create: `tools/native_tests/test_failsafe_land.cpp`
- Create: `tools/test_failsafe_land.py`

**Interfaces:**
- Produces: `enum FailsafePhase : uint8_t { FS_NONE=0, FS_DESCENDING=1, FS_CUT_LANDED=2, FS_CUT_TIMEOUT=3, FS_CUT_ABORT=4 };`,
  `struct LandDetector { bool saw_sub_1g; bool settling; uint32_t settle_start_ms; };`,
  `bool updateLandDetector(LandDetector&, float accel_magnitude_g, uint32_t elapsed_ms, float accel_tol_g, uint32_t min_descend_ms, uint32_t confirm_ms)`,
  `int failsafeDescentThrottle(int entry_throttle, int descent_delta_us)`,
  `FailsafePhase failsafeStep(bool landed, uint32_t elapsed_ms, uint32_t max_ms)`

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_failsafe_land.cpp`:

```cpp
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "failsafe_land.h"

static int g_failures = 0;

static void runCase(const std::string &name, const std::function<void()> &body) {
  try {
    body();
    std::cout << "[PASS] " << name << '\n';
  } catch (const std::exception &error) {
    std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    g_failures++;
  }
}

#define CHECK(expr)                                                          \
  do {                                                                       \
    if (!(expr)) throw std::runtime_error(#expr);                            \
  } while (0)

static const float kTol = 0.05f;
static const uint32_t kMinDescend = 1000;
static const uint32_t kConfirm = 400;

int main() {
  runCase("하강 스로틀은 진입값에서 delta를 뺀 값이다", [] {
    CHECK(failsafeDescentThrottle(1360, 60) == 1300);
  });

  runCase("하강 스로틀은 1000 미만으로 내려가지 않는다", [] {
    CHECK(failsafeDescentThrottle(1020, 60) == 1000);
  });

  runCase("회귀: 진입 직후 1g여도 착지로 판정하지 않는다", [] {
    // 진입 순간 스로틀이 아직 호버라 |accel|=1g다. 여기서 착지로 보면
    // 공중에서 모터가 꺼진다.
    LandDetector det = {};
    for (uint32_t t = 0; t < kMinDescend; t += 10) {
      CHECK(!updateLandDetector(det, 1.0f, t, kTol, kMinDescend, kConfirm));
    }
  });

  runCase("sub-1g를 못 본 채 1g가 계속되면 착지가 아니다", [] {
    // min_descend 시간이 지나도 하강 증거가 없으면 인정하지 않는다.
    LandDetector det = {};
    for (uint32_t t = 0; t < 4000; t += 10) {
      CHECK(!updateLandDetector(det, 1.0f, t, kTol, kMinDescend, kConfirm));
    }
    CHECK(!det.saw_sub_1g);
  });

  runCase("sub-1g 뒤 1g가 confirm 시간 유지되면 착지다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm));
    CHECK(det.saw_sub_1g);
    CHECK(!updateLandDetector(det, 1.00f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 1.00f, 1800, kTol, kMinDescend, kConfirm));
    CHECK(updateLandDetector(det, 1.00f, 1900, kTol, kMinDescend, kConfirm));
  });

  runCase("confirm 도중 다시 sub-1g면 확정이 취소된다", [] {
    LandDetector det = {};
    updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm);
    CHECK(!updateLandDetector(det, 1.00f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 0.80f, 1600, kTol, kMinDescend, kConfirm));
    CHECK(!det.settling);
    // 다시 처음부터 confirm을 채워야 한다
    CHECK(!updateLandDetector(det, 1.00f, 1700, kTol, kMinDescend, kConfirm));
    CHECK(!updateLandDetector(det, 1.00f, 2000, kTol, kMinDescend, kConfirm));
    CHECK(updateLandDetector(det, 1.00f, 2100, kTol, kMinDescend, kConfirm));
  });

  runCase("허용오차 밖의 1g 초과도 착지로 보지 않는다", [] {
    LandDetector det = {};
    updateLandDetector(det, 0.80f, 1200, kTol, kMinDescend, kConfirm);
    // 접지 충격 스파이크(1.5g)는 지면 지지 상태가 아니다
    CHECK(!updateLandDetector(det, 1.50f, 1500, kTol, kMinDescend, kConfirm));
    CHECK(!det.settling);
  });

  runCase("min_descend 이전의 sub-1g는 기억하지 않는다", [] {
    LandDetector det = {};
    CHECK(!updateLandDetector(det, 0.80f, 500, kTol, kMinDescend, kConfirm));
    CHECK(!det.saw_sub_1g);
  });

  runCase("failsafeStep: 착지가 timeout보다 우선한다", [] {
    CHECK(failsafeStep(true, 9999, 5000) == FS_CUT_LANDED);
  });

  runCase("failsafeStep: 백스톱 시간이 지나면 종료한다", [] {
    CHECK(failsafeStep(false, 5000, 5000) == FS_CUT_TIMEOUT);
    CHECK(failsafeStep(false, 4999, 5000) == FS_DESCENDING);
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " failsafe-land case(s) failed\n";
    return 1;
  }
  std::cout << "10/10 failsafe-land cases passed\n";
  return 0;
}
```

- [ ] **Step 2: 실패 확인**

```bash
g++ -std=c++17 -O0 -g -Wall -m32 \
  -I tools/native_tests/shims \
  -I firmware/flight/dual_imu_cascade_pwm \
  tools/native_tests/test_failsafe_land.cpp -o /tmp/test_failsafe_land
```
Expected: FAIL — `failsafe_land.h: No such file or directory`

- [ ] **Step 3: 헤더 구현**

`firmware/flight/dual_imu_cascade_pwm/failsafe_land.h`:

```cpp
#pragma once

#include <Arduino.h>

// 자동착륙 상태. 텔레메트리 Failsafe_Phase로 그대로 나간다.
enum FailsafePhase : uint8_t {
  FS_NONE        = 0,
  FS_DESCENDING  = 1,
  FS_CUT_LANDED  = 2,
  FS_CUT_TIMEOUT = 3,
  FS_CUT_ABORT   = 4,
};

// 착지 감지기의 누적 상태. pid_task가 소유하고 진입 시 {}로 초기화한다.
struct LandDetector {
  bool     saw_sub_1g = false;      // 실제로 하강 가속 중이었다는 증거
  bool     settling = false;        // 1g 복귀가 진행 중인가
  uint32_t settle_start_ms = 0;
};

// 매 tick 호출. 착지가 확정되면 true를 돌려준다.
//
// 원리: 공중에서 스로틀이 호버보다 낮으면 아래로 가속하므로 |accel| < 1g다.
// 지면이 받치면 스로틀과 무관하게 1g로 돌아온다. 진입 순간에는 스로틀이 아직
// 호버라 이미 1g이므로, sub-1g를 한 번이라도 본 뒤에만 1g 복귀를 인정한다.
static inline bool updateLandDetector(
    LandDetector &det, float accel_magnitude_g, uint32_t elapsed_ms,
    float accel_tol_g, uint32_t min_descend_ms, uint32_t confirm_ms) {
  if (elapsed_ms < min_descend_ms) {
    det.settling = false;
    return false;
  }
  if (accel_magnitude_g < 1.0f - accel_tol_g) {
    det.saw_sub_1g = true;
    det.settling = false;
    return false;
  }
  if (!det.saw_sub_1g) {
    det.settling = false;
    return false;
  }
  if (fabsf(accel_magnitude_g - 1.0f) > accel_tol_g) {
    det.settling = false;      // 접지 충격 스파이크 등
    return false;
  }
  if (!det.settling) {
    det.settling = true;
    det.settle_start_ms = elapsed_ms;
    return false;
  }
  return (elapsed_ms - det.settle_start_ms) >= confirm_ms;
}

static inline int failsafeDescentThrottle(int entry_throttle,
                                          int descent_delta_us) {
  const int throttle = entry_throttle - descent_delta_us;
  return throttle < 1000 ? 1000 : throttle;
}

static inline FailsafePhase failsafeStep(bool landed, uint32_t elapsed_ms,
                                         uint32_t max_ms) {
  if (landed) return FS_CUT_LANDED;
  if (elapsed_ms >= max_ms) return FS_CUT_TIMEOUT;
  return FS_DESCENDING;
}
```

- [ ] **Step 4: 통과 확인**

```bash
g++ -std=c++17 -O0 -g -Wall -m32 \
  -I tools/native_tests/shims \
  -I firmware/flight/dual_imu_cascade_pwm \
  tools/native_tests/test_failsafe_land.cpp -o /tmp/test_failsafe_land \
  && /tmp/test_failsafe_land
```
Expected: PASS, 마지막 줄 `10/10 failsafe-land cases passed`

- [ ] **Step 5: unittest 래퍼 작성**

`tools/test_failsafe_land.py`를 만든다. `tools/test_yaw_command.py`의 **툴체인
처리 부분을 그대로** 가져온다(뮤테이션 기계장치가 없는 깨끗한 래퍼다).

유지: `g++` 존재 확인 → `skipTest`, `-m32` multilib probe, `-I` 2개
(`NATIVE_TEST_DIR / "shims"`, `SKETCH_DIR`), SIGSYS → `qemu-i386-static` 폴백,
컴파일 60초 / 실행 20초 타임아웃과 실패 시 커맨드·stdout·stderr 포함.

교체: 클래스 `FailsafeLandTest`, 메서드 `test_failsafe_land`, 실행 파일
`tmp_path / "test_failsafe_land"`, 소스 `NATIVE_TEST_DIR / "test_failsafe_land.cpp"`,
진행 출력 `[FS-LAND] runner=...`.

- [ ] **Step 6: 전체 확인**

```bash
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
/home/light/anaconda3/bin/python tools/check_repo_layout.py
```
Expected: `OK`, `repository layout checks passed`

- [ ] **Step 7: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/failsafe_land.h \
        tools/native_tests/test_failsafe_land.cpp tools/test_failsafe_land.py
git commit -m "feat(flight): add pure failsafe landing-detector helpers"
```

---

## Task 2: `trim` 명령과 목표각 합성

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Test: `tools/native_tests/test_control_math.cpp`

**Interfaces:**
- Produces: `volatile float trim_roll`, `volatile float trim_pitch`,
  `const float TRIM_MAX_DEG`

> **디스패치 주의:** 명령 사슬(1134줄~)은 접두사 매칭이라 순서가 중요하다.
> `trim`은 기존 어떤 명령과도 접두사가 겹치지 않는다(`th`는 `strncmp(buf,"th",2)`,
> `handleGainCommand`의 2글자 접두사 목록에 `tr` 없음). `rcr`처럼
> `buf[4]`가 공백/탭인지 확인해 `trimfoo` 같은 입력을 배제한다.

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_control_math.cpp`의 `main()`에 추가:

```cpp
  runCase("trim: 절대값이 저장된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 1.5 -2.0");
    CHECK_NEAR(trim_roll, 1.5f, 1e-4f);
    CHECK_NEAR(trim_pitch, -2.0f, 1e-4f);
  });

  runCase("trim: +-TRIM_MAX_DEG로 클램프된다", [] {
    trim_roll = 0.0f; trim_pitch = 0.0f;
    sendUdpCommandOnce("trim 50 -50");
    CHECK_NEAR(trim_roll, TRIM_MAX_DEG, 1e-4f);
    CHECK_NEAR(trim_pitch, -TRIM_MAX_DEG, 1e-4f);
  });

  runCase("trim: 잘못된 입력은 무시된다", [] {
    trim_roll = 1.0f; trim_pitch = 2.0f;
    sendUdpCommandOnce("trim abc 1");
    sendUdpCommandOnce("trim 1");
    sendUdpCommandOnce("trim 1 2 3");
    CHECK_NEAR(trim_roll, 1.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, 2.0f, 1e-4f);
  });

  runCase("trim: 무장 중에도 수락된다", [] {
    trim_roll = 0.0f;
    safety_lock = false;
    sendUdpCommandOnce("trim 3 0");
    CHECK_NEAR(trim_roll, 3.0f, 1e-4f);
    safety_lock = true;
  });

  runCase("목표각은 트림을 더한 뒤 +-30도로 클램프된다", [] {
    resetRcState();
    trim_roll = 10.0f; trim_pitch = -10.0f;
    sendRcr("rcr 1 25 -25 0");
    // 25 + 10 = 35 -> 30 으로 클램프 (35가 아니다)
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("트림이 목표각에 실제로 더해진다", [] {
    resetRcState();
    trim_roll = 2.0f; trim_pitch = -1.0f;
    sendRcr("rcr 1 0 0 0");
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("start와 stop은 트림을 지우지 않는다", [] {
    trim_roll = 4.0f; trim_pitch = -3.0f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    sendUdpCommandOnce("stop");
    CHECK_NEAR(trim_roll, 4.0f, 1e-4f);
    CHECK_NEAR(trim_pitch, -3.0f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });
```

- [ ] **Step 2: 실패 확인**

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_native_control_math -v 2>&1 | tail -20
```
Expected: FAIL — `trim_roll`, `TRIM_MAX_DEG` 미정의로 컴파일 에러

- [ ] **Step 3: 전역과 상수 추가**

`MAX_TARGET_ANGLE_RP` 선언 근처에 추가:

```cpp
const float TRIM_MAX_DEG = 10.0f;   // 트림 절대값 상한
```

`targetAngleX` 선언 근처에 추가:

```cpp
// 기체 트림(도). 추정기 0°와 진짜 수평의 차이를 보정한다. 비행별 상태가 아니라
// 기체 속성이므로 start/stop이 지우지 않는다.
volatile float trim_roll = 0.0f;
volatile float trim_pitch = 0.0f;
```

- [ ] **Step 4: `setRcTargets`에서 트림 합산**

현행:

```cpp
  targetAngleX = constrain(x, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
```

교체:

```cpp
  // 트림을 더한 뒤 클램프해야 총합이 ±30°로 제한된다.
  targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
  targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
```

- [ ] **Step 5: `trim` 명령 핸들러 추가**

디스패치 사슬에서 `else if (strcmp(buf, "gains") == 0) {` **앞에** 삽입:

```cpp
        else if (strncmp(buf, "trim", 4) == 0 &&
                 (buf[4] == ' ' || buf[4] == '\t')) {
          char *save = nullptr;
          (void)strtok_r(buf, " \t", &save);          // "trim"
          char *arg0 = strtok_r(nullptr, " \t", &save);
          char *arg1 = strtok_r(nullptr, " \t", &save);
          if (arg0 != nullptr && arg1 != nullptr &&
              strtok_r(nullptr, " \t", &save) == nullptr) {
            float r, p;
            if (parseFloatStrict(arg0, r) && parseFloatStrict(arg1, p)) {
              trim_roll  = constrain(r, -TRIM_MAX_DEG, TRIM_MAX_DEG);
              trim_pitch = constrain(p, -TRIM_MAX_DEG, TRIM_MAX_DEG);
              Serial.printf(">>> Trim R:%.2f P:%.2f\n", trim_roll, trim_pitch);
            }
          }
        }
```

- [ ] **Step 6: 통과 확인**

```bash
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-fs firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, 빌드 성공

- [ ] **Step 7: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_control_math.cpp
git commit -m "feat(flight): move trim into the firmware"
```

---

## Task 3: 자동착륙 상태기계 배선

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Test: `tools/native_tests/test_control_math.cpp`

**Interfaces:**
- Consumes: Task 1의 `FailsafePhase`/`LandDetector`/`updateLandDetector`/
  `failsafeDescentThrottle`/`failsafeStep`, Task 2의 `trim_roll`/`trim_pitch`
- Produces: `volatile uint8_t fs_phase` (텔레메트리가 읽는다)

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_control_math.cpp`에 추가:

```cpp
  runCase("RC 타임아웃이 즉시 컷이 아니라 자동착륙으로 간다", [] {
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK(!safety_lock);
    fs_phase = FS_NONE;
    base_throttle = 1360;

    // rc가 끊긴 지 오래된 상태를 만든다
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK(!safety_lock);                       // 아직 컷이 아니다
    CHECK_EQ(base_throttle, 1360 - FS_DESCENT_DELTA_US);
  });

  runCase("stop은 자동착륙 중에도 즉시 컷이다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    sendUdpCommandOnce("stop");
    CHECK(safety_lock);
    CHECK_EQ(base_throttle, 1000);
  });

  runCase("start가 fs_phase를 FS_NONE으로 되돌린다", [] {
    fs_phase = FS_CUT_TIMEOUT;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    CHECK_EQ((int)fs_phase, (int)FS_NONE);
  });

  runCase("회귀: 하강 목표는 0이 아니라 트림값이다", [] {
    // 사용자가 잡은 결함. 0을 명령하면 트림이 보정하던 흐름이 그대로 재현돼
    // 5초에 수 미터를 흘러간다(설계 문서 §3).
    trim_roll = 2.0f; trim_pitch = -1.5f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    fs_phase = FS_NONE;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100;
    runPidTicks(1);

    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);
    CHECK_NEAR(targetAngleX, 2.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -1.5f, 1e-4f);
    trim_roll = 0.0f; trim_pitch = 0.0f;
  });

  runCase("회귀: 하강 중 rc가 돌아와도 제어권이 복귀하지 않는다", [] {
    // 매 tick 덮어쓰기가 없으면 rc 파서가 targetAngle을 다시 써서
    // '자동 복귀 없음' 결정과 어긋난다(설계 문서 §1).
    trim_roll = 0.0f; trim_pitch = 0.0f;
    calibration_ok = true;
    safety_lock = true;
    sendUdpCommandOnce("start");
    fs_phase = FS_DESCENDING;
    fs_phase_test_entry_setup();      // 아래 주석 참조

    resetRcState();
    sendRcr("rcr 1 25 -25 80");        // 링크 복귀: 큰 조종 입력
    runPidTicks(1);

    CHECK_NEAR(targetAngleX, 0.0f, 1e-4f);   // 스틱 입력이 무시된다
    CHECK_NEAR(targetAngleY, 0.0f, 1e-4f);
    CHECK_NEAR(targetYawRate, 0.0f, 1e-4f);
    CHECK_EQ((int)fs_phase, (int)FS_DESCENDING);   // 상태도 그대로
  });

  runCase("자동착륙 종료 후에는 재시동 없이 날 수 없다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = false;
    base_throttle = 1360;
    lastRcMs = 0;
    arduino_fake::millis_value = RC_TIMEOUT_MS + 100 + FS_MAX_MS;
    runPidTicks(2);
    CHECK(safety_lock);                        // 백스톱으로 잠겼다
    CHECK_EQ(motorOut[0], 1000);
  });
```

> `fs_phase_test_entry_setup()`은 별도 헬퍼가 아니라, 위 "RC 타임아웃이 즉시
> 컷이 아니라 자동착륙으로 간다" 케이스와 같은 방식으로 진입시키라는 뜻이다.
> 실제 코드에서는 그 케이스의 진입 4줄(`base_throttle`, `lastRcMs`,
> `arduino_fake::millis_value`, `runPidTicks(1)`)을 그대로 반복해 쓴다.

`runPidTicks(n)` 헬퍼가 파일에 없으면 다음을 `sendUdpCommandOnce` 옆에 추가한다:

```cpp
void runPidTicks(uint32_t ticks) {
  arduino_fake::tick_limit = ticks;
  try {
    pid_task(nullptr);
  } catch (const arduino_fake::TaskDelayExit &) {
  }
  arduino_fake::tick_limit = 0;
}
```

- [ ] **Step 2: 실패 확인**

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_native_control_math -v 2>&1 | tail -20
```
Expected: FAIL — `fs_phase`, `FS_DESCENDING`, `FS_DESCENT_DELTA_US` 미정의

- [ ] **Step 3: 상수·상태 추가**

`#include "yaw_command.h"` 다음 줄에:

```cpp
#include "failsafe_land.h"
```

`RC_TIMEOUT_MS` 선언 근처에 추가:

```cpp
// 자동착륙. 전부 벤치 조정 대상(설계 문서 §4).
// FS_DESCENT_DELTA_US는 반드시 CTRL_MARGIN(150) 미만이어야 한다 — 넘으면
// 하강 스로틀이 min_throttle 아래로 가서 믹서 collective 하한에 걸려
// 실제 하강이 일어나지 않는다.
const int      FS_DESCENT_DELTA_US = 60;
const uint32_t FS_MIN_DESCEND_MS   = 1000;
const float    FS_LAND_ACCEL_TOL_G = 0.05f;
const uint32_t FS_LAND_CONFIRM_MS  = 400;
const uint32_t FS_MAX_MS           = 5000;
```

전역 상태에 추가:

```cpp
volatile uint8_t fs_phase = FS_NONE;   // 텔레메트리 Failsafe_Phase
```

- [ ] **Step 4: `pid_task` 지역 상태 추가**

`bool wasLocked = true;` 근처에 추가:

```cpp
  LandDetector landDet = {};
  uint32_t fs_enter_ms = 0;
  int fs_entry_throttle = 1000;
```

- [ ] **Step 5: RC 타임아웃 블록 교체 (진입)**

현행 808~811줄:

```cpp
    if (!safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true; safety_lock = true;
      Serial.println("[FAULT] RC TIMEOUT");
    }
```

교체:

```cpp
    // RC 타임아웃은 즉시 컷이 아니라 자동착륙으로 간다. 진입 블록 전체를
    // fs_phase 가드 안에 둔다 — 가드가 없으면 하강 내내 rcTimedOut이 참이라
    // 이 블록이 1kHz로 재실행되고, Serial.println이 TX 버퍼를 포화시켜
    // pid_task를 블로킹하면 500ms 태스크 워치독이 비행 중 재부팅을 일으킨다.
    if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
      fault_rc = true;
      fs_phase = FS_DESCENDING;
      fs_entry_throttle = base_throttle;
      fs_enter_ms = nowMs;
      landDet = {};
      base_throttle = failsafeDescentThrottle(fs_entry_throttle,
                                              FS_DESCENT_DELTA_US);
      Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
    }

    // 하강 중에는 목표를 매 tick 덮어쓴다. 링크가 돌아와도 rc/rcr 파서가 쓴
    // 값이 무해해지므로 파서 쪽에 failsafe 분기를 넣을 필요가 없다.
    if (fs_phase == FS_DESCENDING) {
      targetAngleX = trim_roll;
      targetAngleY = trim_pitch;
      targetYawRate = 0.0f;
      base_throttle = failsafeDescentThrottle(fs_entry_throttle,
                                              FS_DESCENT_DELTA_US);

      const float accelMag = sqrtf(accX*accX + accY*accY + accZ*accZ);
      const uint32_t elapsed = nowMs - fs_enter_ms;
      const bool landed = updateLandDetector(
          landDet, accelMag, elapsed, FS_LAND_ACCEL_TOL_G,
          FS_MIN_DESCEND_MS, FS_LAND_CONFIRM_MS);
      const FailsafePhase next = failsafeStep(landed, elapsed, FS_MAX_MS);
      if (next != FS_DESCENDING) {
        fs_phase = next;
        safety_lock = true;      // 아래 safety_lock 블록이 이번 tick에 정리한다
        Serial.printf(">>> AUTO-LAND END phase=%d\n", (int)next);
      }
    }
```

> `accX/accY/accZ`는 757줄에서 이미 이번 tick 값으로 갱신돼 있다.

- [ ] **Step 6: `start` 핸들러에 리셋 추가**

`start` 성공 분기의 `safety_lock = false;` **앞에** 추가:

```cpp
            fs_phase = FS_NONE;
```

- [ ] **Step 7: 통과 확인**

```bash
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-fs firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, 빌드 성공

- [ ] **Step 8: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_control_math.cpp
git commit -m "feat(flight): auto-land on RC timeout instead of cutting motors"
```

---

## Task 4: 중단 경로 (IMU 전멸 · 과도 기울기)

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Test: `tools/native_tests/test_control_math.cpp`

**Interfaces:**
- Consumes: Task 3의 `fs_phase`

> 중단 처리를 **두 곳**에 넣어야 한다. IMU 전멸 경로(744줄)는 RC 타임아웃
> 검사(808줄)에 도달하기 전에 `continue`하므로 그 분기에서 직접 세워야 하고,
> 과도 기울기는 `safety_lock`을 세우고 812줄 블록으로 흘러간다. 한 곳만
> 처리하면 모터는 꺼졌는데 `Failsafe_Phase`가 1로 남아 로그가 거짓말을 한다.

- [ ] **Step 1: 실패하는 테스트 작성**

```cpp
  runCase("자동착륙 중 safety_lock이 서면 FS_CUT_ABORT로 끝난다", [] {
    fs_phase = FS_DESCENDING;
    safety_lock = true;
    runPidTicks(1);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_ABORT);
  });

  runCase("이미 끝난 상태는 ABORT로 덮어쓰지 않는다", [] {
    fs_phase = FS_CUT_LANDED;
    safety_lock = true;
    runPidTicks(1);
    CHECK_EQ((int)fs_phase, (int)FS_CUT_LANDED);
  });
```

- [ ] **Step 2: 실패 확인**

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_native_control_math -v 2>&1 | tail -20
```
Expected: FAIL — `fs_phase`가 `FS_DESCENDING`인 채로 남음

- [ ] **Step 3: IMU 전멸 분기에 추가**

732~745줄 블록의 `active_imus = 0;` 다음 줄에 추가:

```cpp
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
```

- [ ] **Step 4: `safety_lock` 블록에 추가**

812줄 `if (safety_lock) {` 블록의 `mixer_scaled = false;` **앞에** 추가:

```cpp
      if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;
```

> `FS_CUT_LANDED`/`FS_CUT_TIMEOUT`은 Task 3에서 `safety_lock`을 세우고 이
> 블록으로 들어오므로, `fs_phase == FS_DESCENDING` 조건이 정상 종료를
> `ABORT`로 덮어쓰는 것을 막는다.

- [ ] **Step 5: 통과 확인**

```bash
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-fs firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, 빌드 성공

- [ ] **Step 6: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_control_math.cpp
git commit -m "feat(flight): abort auto-land on IMU loss or over-tilt"
```

---

## Task 5: 텔레메트리 35 → 38필드

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino:1085`
- Modify: `scripts/telemetry_schema.py`
- Test: `tools/test_telemetry_schema.py`

**Interfaces:**
- Consumes: Task 3의 `fs_phase`, Task 2의 `trim_roll`/`trim_pitch`
- Produces: `TELEMETRY_FIELDS[35..37] == ("Failsafe_Phase", "Trim_Roll", "Trim_Pitch")`

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/test_telemetry_schema.py`에 추가:

```python
    def test_38_field_packet_parses_new_fields(self):
        packet = ",".join(["1"] * 35 + ["2", "1.5", "-2.5"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Failsafe_Phase"], 2)
        self.assertAlmostEqual(sample["Trim_Roll"], 1.5)
        self.assertAlmostEqual(sample["Trim_Pitch"], -2.5)

    def test_35_field_packet_leaves_new_fields_none(self):
        packet = ",".join(["1"] * 35)
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertIsNone(sample["Failsafe_Phase"])
        self.assertIsNone(sample["Trim_Roll"])
        self.assertIsNone(sample["Trim_Pitch"])

    def test_csv_has_39_columns(self):
        self.assertEqual(len(telemetry_schema.CSV_FIELDS), 39)
```

- [ ] **Step 2: 실패 확인**

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_telemetry_schema -v 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `FAILED` — `KeyError: 'Failsafe_Phase'`

- [ ] **Step 3: 스키마 갱신**

`scripts/telemetry_schema.py`:
- `TELEMETRY_FIELDS` 끝에 `"Failsafe_Phase", "Trim_Roll", "Trim_Pitch",` 추가
- `TELEMETRY_FIELD_TYPES`에 `"Failsafe_Phase": int, "Trim_Roll": float, "Trim_Pitch": float,` 추가
- 모듈 docstring과 `parse_telemetry_packet` docstring의 필드 수 서술을 38로 갱신하고
  허용 길이 목록에 `38`을 추가

- [ ] **Step 4: 펌웨어 텔레메트리 확장**

1085줄 포맷 문자열 끝에 `,%d,%.2f,%.2f`를 붙이고, 인자 목록 끝에
`(int)fs_phase, trim_roll, trim_pitch`를 추가한다.

- [ ] **Step 5: 통과 확인**

```bash
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
/home/light/anaconda3/bin/python -c "
import sys; sys.path.insert(0,'scripts'); import telemetry_schema as t
print(len(t.TELEMETRY_FIELDS), len(t.CSV_FIELDS))"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-fs firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, `38 39`, 빌드 성공

- [ ] **Step 6: 포맷 지시자 수 확인**

```bash
/home/light/anaconda3/bin/python -c "
import re
s=open('firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino').read()
i=s.find('static void sendTelemetry')
m=re.search(r'udp\.printf\(\"([^\"]+)\"', s[i:])
print('지시자:', m.group(1).count('%'))"
```
Expected: `지시자: 38`

- [ ] **Step 7: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        scripts/telemetry_schema.py tools/test_telemetry_schema.py
git commit -m "feat(telemetry): append Failsafe_Phase and trim fields"
```

---

## Task 6: 지상국

**Files:**
- Modify: `scripts/control_dualsense.py`

**Interfaces:**
- Consumes: Task 2의 `trim` 명령, Task 3의 자동착륙 동작

> **`stop`을 보내면 안 되는 경로가 두 개다.** 텔레메트리 타임아웃과 `Fault_RC`
> 상승 엣지. 특히 후자는 업링크만 죽는 비대칭 고장에서 드론이 자동착륙 중일 때
> `stop`을 쏴 **공중에서 모터를 끄는** 경로다. `Fault_Critical`과 조종자 수동
> disarm은 `stop`을 그대로 유지한다.

- [ ] **Step 1: 헬퍼 두 개 추가**

`disarm()` 아래에 **둘 다** 추가한다. 뒤 단계에서 쓰기 전에 먼저 정의한다.

```python
def send_trim():
    """트림을 절대값으로 드론에 보낸다. 손실되면 자동착륙 방향이 틀어지므로
    start/stop과 같은 급으로 반복 전송한다."""
    reliable_send(f"trim {trim_roll:.2f} {trim_pitch:.2f}")


def stop_streaming_only(reason: str):
    """드론이 자동착륙 중일 때 쓰는 로컬 해제. stop을 보내지 않는다.

    stop을 보내면 하강 중인 드론의 모터를 공중에서 꺼버린다. 업링크만 죽는
    비대칭 고장에서는 링크가 간헐적으로 열려 그 stop이 실제로 도착한다.
    """
    global is_armed, current_throttle, throttle_f
    is_armed = False
    current_throttle = 1000
    throttle_f = 1000.0
    print(f"\n>>> [SYSTEM] 로컬 해제 ({reason}) - stop 미전송")
```

- [ ] **Step 2: `arm()`에서 트림 재전송**

`arm()`의 `reliable_send("mag 1")` **앞에** 추가:

```python
    send_trim()   # 드론 재부팅으로 트림이 소실됐을 수 있다
```

`send_trim()`은 `trim_roll`/`trim_pitch`를 읽기만 하므로 `global` 선언이
필요 없다. `arm()`도 그대로 둔다.

- [ ] **Step 3: DPAD 트림 변경 시 전송**

`controller_thread()`의 DPAD 처리에서 `print(f" [TRIM] ...")` 다음 줄에 추가:

```python
                    send_trim()
```

트림 리셋 분기(`trim_roll = trim_pitch = 0.0`)의 `print(" [TRIM] RESET (0.0, 0.0)")`
다음 줄에도 추가:

```python
                send_trim()
```

- [ ] **Step 4: 송신에서 트림 합산 제거**

현행 357~358줄:

```python
            final_roll  = deadzone(joy.get_axis(0))  * MAX_ANGLE + trim_roll
            final_pitch = deadzone(-joy.get_axis(1)) * MAX_ANGLE + trim_pitch
```

교체(이제 드론이 더한다):

```python
            final_roll  = deadzone(joy.get_axis(0))  * MAX_ANGLE
            final_pitch = deadzone(-joy.get_axis(1)) * MAX_ANGLE
```

- [ ] **Step 5: 텔레메트리 타임아웃 경로에서 `stop` 제거**

현행:

```python
                if elapsed > TELEM_TIMEOUT_SEC:
                    print(f"\n[FAULT] 텔레메트리 {elapsed:.1f}s 수신 없음 - 긴급 정지")
                    disarm("연결 끊김")
```

교체:

```python
                if elapsed > TELEM_TIMEOUT_SEC:
                    print(f"\n[FAULT] 텔레메트리 {elapsed:.1f}s 수신 없음 "
                          f"- rc 중단, 드론 자동착륙에 맡김")
                    stop_streaming_only("텔레메트리 끊김")
```

- [ ] **Step 6: `Fault_RC` 경로에서 `stop` 제거**

현행 210~213줄:

```python
        if fault_rc_drone and not prev_fault_rc:
            print("\n[FAULT] 드론: RC 타임아웃 감지됨")
            if is_armed:
                disarm("드론 RC 타임아웃")
```

교체:

```python
        if fault_rc_drone and not prev_fault_rc:
            print("\n[FAULT] 드론: RC 타임아웃 - 자동착륙 진행 중")
            if is_armed:
                stop_streaming_only("드론 RC 타임아웃")
```

- [ ] **Step 7: 확인**

```bash
/home/light/anaconda3/bin/python -m py_compile scripts/*.py && echo "py_compile OK"
grep -n "trim_roll" scripts/control_dualsense.py
grep -n 'disarm("드론 RC 타임아웃")\|disarm("연결 끊김")' scripts/control_dualsense.py \
  || echo "  두 경로의 stop 전송 제거 확인 ✓"
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `py_compile OK`, 357~358줄에 `trim_roll` 없음, 제거 확인 메시지, `OK`

- [ ] **Step 8: 커밋**

```bash
git add scripts/control_dualsense.py
git commit -m "feat(scripts): send trim to the drone, stop killing auto-land"
```

---

## Task 7: 문서 갱신과 벤치 Stage E

**Files:**
- Modify: `docs/udp_protocol.md`, `logs/README.md`, `scripts/README.md`,
  `docs/power_on_bench_procedure.md`, `docs/ground_station_link.md`,
  `docs/project_overview.md`

- [ ] **Step 1: `docs/udp_protocol.md`**

- 명령 블록에 `trim <roll_deg> <pitch_deg>` 추가(절대값, ±10° 클램프, 무장 중 수락,
  `start`/`stop`이 지우지 않음)
- **RC 타임아웃 동작 변경 명시**: 즉시 컷이 아니라 자동착륙 진입. `stop`은 여전히
  즉시 컷
- 텔레메트리 35 → 38필드, 표에 추가:
  `| 36 | Failsafe_Phase | int | 0=정상 1=하강중 2=착지컷 3=백스톱컷 4=중단컷 |`,
  `| 37~38 | Trim_Roll, Trim_Pitch | float | 드론이 적용 중인 트림(도) |`
- 레거시 목록에 35필드 항목 추가, CSV 36 → 39열

- [ ] **Step 2: `logs/README.md`, `scripts/README.md`**

필드 수 38, CSV 39열. CSV 열 목록 끝에 세 필드 추가. `scripts/README.md`에는
트림이 이제 드론에 저장되며 시동 시 재전송된다는 점을 한 줄 추가.

- [ ] **Step 3: `docs/power_on_bench_procedure.md` — Stage B-2 수정**

RC 타임아웃 검증 항목의 기대 동작이 바뀐다. 현행 "0.5초 내 `Fault_RC` 세워지고
모터 1000으로"를 다음으로 교체:

> rc 스트림을 끊으면 0.5초 내 `Fault_RC`가 서고 **`Failsafe_Phase`가 1(하강)로
> 바뀐다.** 모터는 즉시 1000이 되지 않고 하강 스로틀로 낮아진다. `FS_MAX_MS`(5초)
> 뒤 `Failsafe_Phase`가 3(백스톱컷)이 되며 모터가 1000이 된다. **프롭 OFF에서
> 먼저 확인한다.**

- [ ] **Step 4: `docs/power_on_bench_procedure.md` — Stage E 신설**

Stage D 뒤에 추가:

```markdown
## Stage E — 자동착륙 (반드시 이 순서로)

### E-1. 프롭 OFF, 상태 전이만
`start` 후 rc 스트림을 끊는다. `Failsafe_Phase` 0 → 1, 5초 뒤 3(백스톱컷),
모터 1000. 프롭이 없어 하강은 일어나지 않으므로 백스톱 경로만 확인한다.

### E-2. 프롭 OFF, 중단 경로
하강 중(`Phase=1`) 기체를 60° 이상 기울인다. `Phase=4`(중단컷)로 즉시 끝나야 한다.

### E-3. 프롭 ON, 손 파지
하강 스로틀에서 손에 느껴지는 추력이 줄어드는지. **기체를 놓지 않는다.**

### E-4. 프롭 ON, 약 50cm
실제로 내려와 착지하고 `Phase=2`(착지컷)로 끝나는지. 여기서
`FS_DESCENT_DELTA_US`를 조정해 하강률을 확정한다. 접지 후 튀거나 넘어지면
감지가 늦은 것이다 — `FS_LAND_CONFIRM_MS`를 줄인다.

### E-4b. 낮은 스로틀 진입 (스펙 §4)
`th 1150` 상태에서 rc를 끊는다. 하강 스로틀이 1090이 되어 **적분기 세 개가
0으로 리셋**되므로 자세 트랜지언트가 생기는지 관찰한다. 손 파지로 확인한다.

### E-5. 고도 단계 상승
1m → 2m → 3m. 각 단계에서 `Failsafe_Phase` 최종값과 하강 소요 시간을 기록한다.
3m에서 5초 안에 못 내려오면 `FS_MAX_MS`를 10000으로 올린다.

**주의:** 하강 중 `start`는 무시된다. 조종자가 착륙을 중단할 수단은 `stop`(즉시
컷)뿐이다. 넓은 공간에서, 사람과 장애물이 없는 곳에서 진행한다. 위치 유지
수단이 없어 바람에 흘러간다.

### 통과 기록
- [ ] E-1: 백스톱 경로 (`Phase` 0→1→3)
- [ ] E-2: 중단 경로 (`Phase` 1→4)
- [ ] E-3: 하강 추력 감소 확인
- [ ] E-4b: 낮은 스로틀 진입 시 자세 트랜지언트 관찰
- [ ] E-4: 실제 착지 감지 (`Phase` 1→2), 확정한 `FS_DESCENT_DELTA_US` = ___
- [ ] E-5: 3m 하강 시간 = ___초, `FS_MAX_MS` 최종값 = ___
```

준비 절의 확인 필드 목록에 `Failsafe_Phase`, `Trim_Roll/Pitch`를 추가한다.

- [ ] **Step 5: `docs/ground_station_link.md`**

RC 타임아웃 설명을 갱신한다. "RC 타임아웃은 고장이 아니라 안전장치다" 절에
자동착륙 도입을 반영하고, 지상국이 `Fault_RC`를 보고 `stop`을 보내면 자동착륙을
죽인다는 점을 명시한다.

- [ ] **Step 6: `docs/project_overview.md`**

페일세이프 동작 서술을 갱신한다: RC 타임아웃은 자동착륙, 나머지 fault는 즉시 컷.

- [ ] **Step 7: 확인**

```bash
/home/light/anaconda3/bin/python tools/check_repo_layout.py
/home/light/anaconda3/bin/python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
grep -rn "35개 필드\|36개 열" docs/ logs/README.md scripts/README.md \
  | grep -v "레거시\|끝난다\|이전 펌웨어" || echo "  구 필드 수 잔여 없음 ✓"
```
Expected: 통과, `OK`, 잔여 없음

- [ ] **Step 8: 커밋**

```bash
git add docs/ logs/README.md scripts/README.md
git commit -m "docs: failsafe auto-land, trim command, bench Stage E"
```

---

## 실기 검증 (구현 완료 후, 사람이 수행)

**코드가 끝나도 비행 가능 상태가 아니다.** SIL은 이 기능을 검증하지 못한다 —
플랜트에 수직 동역학이 없어 하강도 착지도 시뮬레이션할 수 없다(스펙 §8).

`docs/power_on_bench_procedure.md` 전 항목을 재실행하고, 특히:

- Stage B-2: RC 타임아웃의 **새 기대 동작** 확인
- Stage E-1 ~ E-5 순서대로. **E-4 전에는 자동착륙을 신뢰하지 않는다**
- 확정한 `FS_DESCENT_DELTA_US`, `FS_MAX_MS`, 하강률을 통과 기록에 남긴다
