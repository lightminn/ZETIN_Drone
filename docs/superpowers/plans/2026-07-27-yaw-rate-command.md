# Yaw 각속도 명령 전환 — 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** yaw 스틱을 절대 각도 명령에서 각속도 명령으로 바꾸고, 스틱을 놓으면 그 자리의 heading을 자동으로 잠근다.

**Architecture:** yaw 모드 판정과 setpoint 갱신을 순수 함수로 새 헤더 `yaw_command.h`에 분리해 플랜트 없이 단위 테스트한다(기존 `mag_yaw_fusion.h` + `test_mag_yaw_fusion.cpp` 패턴 그대로). 펌웨어 `pid_task`는 그 헬퍼를 호출하기만 한다. 지상국은 상태 없는 각속도만 신규 명령 `rcr`로 보낸다.

**Tech Stack:** Arduino/ESP32-S3 (arduino-cli), C++17 호스트 네이티브 테스트(g++ `-m32` + shims), Python 3 unittest, pygame.

## Global Constraints

- 설계 근거: `docs/superpowers/specs/2026-07-27-yaw-rate-command-design.md`
- `YAW_RATE_DEADZONE = 3.0f` (dps), `YAW_HOLD_SETTLE_DPS = 10.0f` (dps) — 벤치 조정 대상
- `MAX_TARGET_RATE_YAW = 180.0f` 유지 (두 모드 공통 하드 실링)
- `YAW_RATE_MAX_DPS = 90.0` — 지상국 상수, 펌웨어에 두지 않는다
- `rc` 명령은 **변경하지 않는다** (벤치 도구 3종이 사용)
- 텔레메트리는 **append-only**. 34 → 35필드, CSV 35 → 36열
- 모든 native 테스트는 `tools/test_*.py` 래퍼를 통해 `python -m unittest discover -s tools -p "test_*.py"`로 수집된다
- 각 태스크 종료 시 `python tools/check_repo_layout.py`와 전체 unittest가 통과해야 한다
- 커밋 메시지 말미: `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`

## File Structure

| 파일 | 책임 |
|---|---|
| `firmware/flight/dual_imu_cascade_pwm/yaw_command.h` (신규) | yaw 모드 판정 + setpoint 슬레이빙 + 목표 각속도 산출. 순수 함수만 |
| `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` | 헬퍼 호출 배선, `rcr` 파서, `yaw 1` 게이트, 오버라이드 수명, 텔레메트리 |
| `tools/native_tests/test_yaw_command.cpp` (신규) | `yaw_command.h` 단위 테스트 |
| `tools/test_yaw_command.py` (신규) | 위 네이티브 테스트의 unittest 래퍼 |
| `tools/native_tests/test_control_math.cpp` | `rcr` 파서, `yaw 1` 무장 게이트 통합 테스트 |
| `scripts/telemetry_schema.py` | `Yaw_Hold` 필드 |
| `scripts/control_dualsense.py` | `target_yaw` 삭제, `rcr` 전송 |
| `scripts/bench_yaw_test.py` | `yaw 1`을 `start` 앞으로 |

---

## Task 1: `yaw_command.h` 순수 헬퍼

**Files:**
- Create: `firmware/flight/dual_imu_cascade_pwm/yaw_command.h`
- Create: `tools/native_tests/test_yaw_command.cpp`
- Create: `tools/test_yaw_command.py`

**Interfaces:**
- Consumes: `wrapDeg()` from `mag_yaw_fusion.h`
- Produces: `struct YawOuter { float target_angle_deg; bool hold; };`,
  `YawOuter updateYawOuter(float target_yaw_rate_dps, float body_gz_dps, float angle_z_deg, float prev_target_angle_deg, bool override_hold, float rate_deadzone_dps, float settle_dps)`,
  `float yawTargetRateDps(const YawOuter &s, float target_yaw_rate_dps, float angle_z_deg, float kp_angle_yaw, float max_rate_dps)`

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_yaw_command.cpp`:

```cpp
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "yaw_command.h"

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

static void checkNear(float actual, float expected, float tol,
                      const char *what) {
  if (std::fabs(actual - expected) > tol) {
    throw std::runtime_error(std::string(what) + ": got " +
                             std::to_string(actual) + " want " +
                             std::to_string(expected));
  }
}

static const float kDead = 3.0f;
static const float kSettle = 10.0f;

int main() {
  runCase("스틱 편향 중에는 rate 모드이고 setpoint가 현재 heading을 따라간다", [] {
    YawOuter s = updateYawOuter(45.0f, 40.0f, 137.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "슬레이빙");
    checkNear(yawTargetRateDps(s, 45.0f, 137.0f, 3.0f, 180.0f), 45.0f, 1e-4f,
              "rate 명령 통과");
  });

  runCase("스틱을 놓아도 아직 회전 중이면 잠기지 않는다", [] {
    YawOuter s = updateYawOuter(0.0f, 40.0f, 137.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "계속 슬레이빙");
    checkNear(yawTargetRateDps(s, 0.0f, 137.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "감쇠 목표 0");
  });

  runCase("스틱 중립 + 정착이면 그 자리에서 잠긴다", [] {
    YawOuter s = updateYawOuter(0.0f, 2.0f, 137.0f, 137.0f, false, kDead, kSettle);
    CHECK(s.hold);
    checkNear(s.target_angle_deg, 137.0f, 1e-4f, "setpoint 동결");
    checkNear(yawTargetRateDps(s, 0.0f, 137.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "오차 0이면 명령 0");
  });

  runCase("잠긴 뒤 heading이 밀리면 복원 방향으로 명령한다", [] {
    YawOuter s = updateYawOuter(0.0f, 1.0f, 130.0f, 137.0f, false, kDead, kSettle);
    CHECK(s.hold);
    // 오차 = wrapDeg(137 - 130) = +7 → +7 * 3.0 = +21
    checkNear(yawTargetRateDps(s, 0.0f, 130.0f, 3.0f, 180.0f), 21.0f, 1e-3f,
              "복원 명령");
  });

  runCase("회귀: 큰 heading 드리프트 상태로 진입해도 슬램이 없다", [] {
    // 문제 3 재현 조건. 슬레이빙이 매 tick 돌았으므로 prev setpoint는 현재값이다.
    YawOuter s = updateYawOuter(0.0f, 1.0f, 150.0f, 150.0f, false, kDead, kSettle);
    CHECK(s.hold);
    checkNear(yawTargetRateDps(s, 0.0f, 150.0f, 3.0f, 180.0f), 0.0f, 1e-4f,
              "슬램 없음");
  });

  runCase("오버라이드는 스틱·각속도와 무관하게 잠금을 강제한다", [] {
    YawOuter s = updateYawOuter(45.0f, 40.0f, 130.0f, 0.0f, true, kDead, kSettle);
    CHECK(s.hold);
    checkNear(s.target_angle_deg, 0.0f, 1e-4f, "setpoint 동결 유지");
    // 오차 = wrapDeg(0 - 130) = -130 → -390 → -180으로 클램프
    checkNear(yawTargetRateDps(s, 45.0f, 130.0f, 3.0f, 180.0f), -180.0f, 1e-4f,
              "클램프");
  });

  runCase("rate 명령도 하드 실링으로 클램프된다", [] {
    YawOuter s = updateYawOuter(500.0f, 0.0f, 0.0f, 0.0f, false, kDead, kSettle);
    CHECK(!s.hold);
    checkNear(yawTargetRateDps(s, 500.0f, 0.0f, 3.0f, 180.0f), 180.0f, 1e-4f,
              "양의 클램프");
    checkNear(yawTargetRateDps(s, -500.0f, 0.0f, 3.0f, 180.0f), -180.0f, 1e-4f,
              "음의 클램프");
  });

  runCase("±180 경계에서 짧은 쪽으로 복원한다", [] {
    YawOuter s = updateYawOuter(0.0f, 1.0f, 179.0f, -179.0f, false, kDead, kSettle);
    CHECK(s.hold);
    // wrapDeg(-179 - 179) = +2 → +6
    checkNear(yawTargetRateDps(s, 0.0f, 179.0f, 3.0f, 180.0f), 6.0f, 1e-3f,
              "짧은 경로");
  });

  if (g_failures != 0) {
    std::cerr << g_failures << " yaw-command case(s) failed\n";
    return 1;
  }
  std::cout << "8/8 yaw-command cases passed\n";
  return 0;
}
```

- [ ] **Step 2: 실패 확인**

```bash
g++ -std=c++17 -O0 -g -Wall -m32 \
  -I tools/native_tests/shims \
  -I firmware/flight/dual_imu_cascade_pwm \
  tools/native_tests/test_yaw_command.cpp -o /tmp/test_yaw_command
```
Expected: FAIL — `yaw_command.h: No such file or directory`

- [ ] **Step 3: 헤더 구현**

`firmware/flight/dual_imu_cascade_pwm/yaw_command.h`:

```cpp
#pragma once

#include <Arduino.h>

#include "mag_yaw_fusion.h"   // wrapDeg

// 바깥 yaw 루프의 모드와 heading setpoint.
// hold=false 인 동안 setpoint를 현재 heading으로 계속 슬레이빙하므로
// setpoint가 stale해질 수 없고, 어떤 전환도 정의상 bumpless다.
struct YawOuter {
  float target_angle_deg;
  bool  hold;
};

static inline YawOuter updateYawOuter(
    float target_yaw_rate_dps, float body_gz_dps, float angle_z_deg,
    float prev_target_angle_deg, bool override_hold,
    float rate_deadzone_dps, float settle_dps) {
  const bool stick_centered = fabsf(target_yaw_rate_dps) < rate_deadzone_dps;
  const bool settled = fabsf(body_gz_dps) < settle_dps;
  YawOuter out;
  out.hold = override_hold || (stick_centered && settled);
  out.target_angle_deg = out.hold ? prev_target_angle_deg : angle_z_deg;
  return out;
}

static inline float yawTargetRateDps(
    const YawOuter &state, float target_yaw_rate_dps, float angle_z_deg,
    float kp_angle_yaw, float max_rate_dps) {
  const float raw = state.hold
      ? wrapDeg(state.target_angle_deg - angle_z_deg) * kp_angle_yaw
      : target_yaw_rate_dps;
  if (raw > max_rate_dps) return max_rate_dps;
  if (raw < -max_rate_dps) return -max_rate_dps;
  return raw;
}
```

- [ ] **Step 4: 통과 확인**

```bash
g++ -std=c++17 -O0 -g -Wall -m32 \
  -I tools/native_tests/shims \
  -I firmware/flight/dual_imu_cascade_pwm \
  tools/native_tests/test_yaw_command.cpp -o /tmp/test_yaw_command && /tmp/test_yaw_command
```
Expected: PASS, 마지막 줄 `8/8 yaw-command cases passed`

- [ ] **Step 5: unittest 래퍼 작성**

`tools/test_yaw_command.py`를 만든다. `tools/test_mag_yaw_fusion.py`의 **툴체인
처리 부분만** 가져오고 뮤테이션 기계장치는 가져오지 않는다.

**반드시 유지할 것** (그대로 복사):
- `g++` 존재 확인 → 없으면 `skipTest`
- `-m32` multilib probe → 실패 시 `skipTest`, 타임아웃 시 `fail`
- 컴파일 커맨드의 `-I` 2개: `NATIVE_TEST_DIR / "shims"`, `SKETCH_DIR`
- 실행 후 `returncode == -signal.SIGSYS`이면 `qemu-i386-static`으로 재실행,
  없으면 `skipTest`
- 컴파일 60초 / 실행 20초 타임아웃, 실패 시 커맨드와 stdout·stderr를 메시지에 포함

**반드시 제거할 것** (이 테스트는 순수 헤더 단위 테스트라 해당 없음):
- `SUPPORTED_MUTATIONS` 상수
- `BMM350_SIL_MUTATION` 환경변수 읽기와 검증
- `if mutation != "none":` 블록 전체 (`shutil.copytree` + 헤더 문자열 치환)
- `sketch_dir` 지역변수 — `SKETCH_DIR`을 직접 `-I`에 쓴다
- `print`의 `mutation=` 부분
- 이제 쓰이지 않는 `import os`

> 이 제거가 왜 중요한가: 뮤테이션 블록은 `mag_yaw_fusion.h`를 변조한다.
> 그대로 두면 `BMM350_SIL_MUTATION=inverted`로 실행할 때 mag 헤더를 변조한 뒤
> **mag과 무관한** yaw_command 테스트를 컴파일해 통과시킨다. 뮤테이션을 잡은
> 것처럼 보이지만 실제로는 아무것도 검증하지 않아, mag 쪽 뮤테이션 검증의
> 신뢰성을 망친다.

**문구 교체**: 클래스 `YawCommandTest`, 메서드 `test_yaw_command`, 실행 파일
`tmp_path / "test_yaw_command"`, 소스 `NATIVE_TEST_DIR / "test_yaw_command.cpp"`.
docstring과 모든 실패 메시지의 `BMM350 yaw-fusion SIL` 표현을
`yaw-command unit test`로 바꾸고, 진행 출력은 `[YAW-CMD] runner=...`로 한다.

- [ ] **Step 6: 전체 스위트 확인**

```bash
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
python tools/check_repo_layout.py
```
Expected: `OK`, `repository layout checks passed`

- [ ] **Step 7: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/yaw_command.h \
        tools/native_tests/test_yaw_command.cpp tools/test_yaw_command.py
git commit -m "feat(flight): add pure yaw outer-loop helpers with unit tests"
```

---

## Task 2: `rcr` 명령 파서와 디스패치

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- Test: `tools/native_tests/test_control_math.cpp`

**Interfaces:**
- Consumes: 기존 `parseFloatStrict`, `setRcTargets`, seq 상태(`lastRcSeq`, `rcSeqValid`, `rcTotalPkts`, `rcDroppedPkts`)
- Produces: `volatile float targetYawRate` (dps), `static void handleRcrCommand(char *buf)`

> **주의:** 기존 디스패치는 `strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')`이다.
> `"rcr ..."`는 `buf[2] == 'r'`이라 이 분기에 걸리지 않으므로 순서 자체는 안전하지만,
> 가독성을 위해 `rcr` 분기를 `rc` 분기 **바로 앞**에 둔다.

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_control_math.cpp`의 `sendRc` 헬퍼 옆에 추가:

```cpp
void sendRcr(const std::string &command) {
  std::vector<char> buffer(command.begin(), command.end());
  buffer.push_back('\0');
  handleRcrCommand(buffer.data());
}
```

그리고 `main()`의 rc 파서 케이스들 뒤에 추가:

```cpp
  runCase("rcr: 정상 패킷이 yaw 각속도와 roll/pitch를 설정한다", [] {
    resetRcState();
    sendRcr("rcr 1 5.5 -6.5 45.0");
    CHECK_NEAR(targetAngleX, 5.5f, 1e-4f);
    CHECK_NEAR(targetAngleY, -6.5f, 1e-4f);
    CHECK_NEAR(targetYawRate, 45.0f, 1e-4f);
  });

  runCase("rcr: roll/pitch는 +-30도로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 90 -90 0");
    CHECK_NEAR(targetAngleX, 30.0f, 1e-4f);
    CHECK_NEAR(targetAngleY, -30.0f, 1e-4f);
  });

  runCase("rcr: yaw 각속도는 +-180dps로 클램프된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 500");
    CHECK_NEAR(targetYawRate, 180.0f, 1e-4f);
    sendRcr("rcr 2 0 0 -500");
    CHECK_NEAR(targetYawRate, -180.0f, 1e-4f);
  });

  runCase("rcr: 역순/중복 seq는 폐기되고 드롭으로 계수된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 20");
    const uint32_t dropped_before = rcDroppedPkts;
    sendRcr("rcr 9 0 0 99");
    CHECK_NEAR(targetYawRate, 20.0f, 1e-4f);
    CHECK_EQ(rcDroppedPkts, dropped_before + 1U);
  });

  runCase("rcr: seq 건너뜀이 드롭 수에 반영된다", [] {
    resetRcState();
    sendRcr("rcr 10 0 0 0");
    sendRcr("rcr 14 0 0 0");
    CHECK_EQ(rcDroppedPkts, 3U);
  });

  runCase("rcr: 인자 수 불일치와 비수치는 거부된다", [] {
    resetRcState();
    sendRcr("rcr 1 0 0 30");
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 2 0 0");          // 인자 부족
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 3 0 0 abc");      // 비수치
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
    sendRcr("rcr 4 0 0 10 99");    // 여분 필드
    CHECK_NEAR(targetYawRate, 30.0f, 1e-4f);
  });

  runCase("rcr: udp_task 디스패치가 rc와 구분해서 처리한다", [] {
    resetRcState();
    sendUdpCommandOnce("rcr 1 0 0 60");
    CHECK_NEAR(targetYawRate, 60.0f, 1e-4f);
  });
```

`resetRcState()`는 102줄에 이미 있다. 마지막 줄에 다음을 추가한다:

```cpp
  targetYawRate = 0.0f;
```

- [ ] **Step 2: 실패 확인**

```bash
python -m unittest tools.test_native_control_math -v 2>&1 | tail -20
```
Expected: FAIL — `handleRcrCommand` / `targetYawRate` 미정의로 컴파일 에러

- [ ] **Step 3: 펌웨어 구현**

전역 선언부(`volatile float targetAngleX...` 근처)에 추가:

```cpp
volatile float targetYawRate = 0.0f;   // rcr이 준 yaw 각속도 명령 (dps)
```

`handleRcCommand` 바로 뒤에 추가:

```cpp
// rcr <seq> <roll> <pitch> <yaw_rate>
// yaw는 각속도(dps) 명령이다. seq 처리와 워치독 급이는 rc와 상태를 공유한다.
static void handleRcrCommand(char *buf) {
  char *save = nullptr;
  (void)strtok_r(buf, " \t", &save); // "rcr"
  char *arg[4] = {nullptr, nullptr, nullptr, nullptr};
  int count = 0;
  while (count < 4 && (arg[count] = strtok_r(nullptr, " \t", &save)) != nullptr) count++;
  if (count != 4) return;
  if (strtok_r(nullptr, " \t", &save) != nullptr) return; // 여분 필드 거부

  if (arg[0][0] == '-' || arg[0][0] == '+') return;
  char *seqEnd;
  errno = 0;
  unsigned long seqLong = strtoul(arg[0], &seqEnd, 10);
  if (seqEnd == arg[0] || *seqEnd != '\0' || errno == ERANGE ||
      seqLong > 0xFFFFFFFFUL) return;

  float x, y, rate;
  if (!parseFloatStrict(arg[1], x) || !parseFloatStrict(arg[2], y) ||
      !parseFloatStrict(arg[3], rate)) return;

  uint32_t seq = (uint32_t)seqLong;
  rcTotalPkts = rcTotalPkts + 1;
  if (rcSeqValid) {
    int32_t advance = (int32_t)(seq - lastRcSeq);
    if (advance <= 0) {
      rcDroppedPkts = rcDroppedPkts + 1;
      return;
    }
    if (advance > 1) rcDroppedPkts += (uint32_t)(advance - 1);
  }
  lastRcSeq = seq;
  rcSeqValid = true;

  if (!setRcTargets(x, y, 0.0f, false)) return;   // roll/pitch만, yaw 각도는 건드리지 않는다
  targetYawRate = constrain(rate, -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}
```

디스패치(현행 1073줄)의 `rc` 분기 **바로 앞**에 삽입:

```cpp
        if (strncmp(buf, "rcr", 3) == 0 && (buf[3] == ' ' || buf[3] == '\t')) {
          handleRcrCommand(buf);
        }
        else if (strncmp(buf, "rc", 2) == 0 && (buf[2] == ' ' || buf[2] == '\t')) {
```

(기존 `if`를 `else if`로 바꾸는 것에 주의.)

- [ ] **Step 4: 통과 확인**

```bash
python -m unittest tools.test_native_control_math -v 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `OK`

- [ ] **Step 5: 실기 빌드 확인**

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-yaw firmware/flight/dual_imu_cascade_pwm
```
Expected: 성공, 신규 경고 없음

- [ ] **Step 6: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_control_math.cpp
git commit -m "feat(flight): add rcr yaw-rate command parser"
```

---

## Task 3: yaw 상태기계를 `pid_task`에 배선

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` (상수, include, 바깥 루프, safety_lock 분기)

**Interfaces:**
- Consumes: Task 1의 `YawOuter`/`updateYawOuter`/`yawTargetRateDps`, Task 2의 `targetYawRate`
- Produces: `volatile bool yaw_hold_now` (Task 6의 텔레메트리가 읽는다)

- [ ] **Step 1: 상수와 include 추가**

`#include "mag_yaw_fusion.h"` 다음 줄에:

```cpp
#include "yaw_command.h"
```

`MAX_TARGET_RATE_YAW` 선언 다음에:

```cpp
// yaw 모드 판정 임계치. 벤치에서 조정한다(설계 문서 §1 참조).
const float YAW_RATE_DEADZONE   = 3.0f;    // 명령 각속도 절댓값이 이하면 스틱 중립
const float YAW_HOLD_SETTLE_DPS = 10.0f;   // bodyGz 절댓값이 이하면 정착
```

전역 상태에 추가:

```cpp
volatile bool yaw_hold_now = false;   // 텔레메트리용: 현재 heading 잠금 중인가
```

`volatile bool yaw_enabled = false;`를 다음으로 **이름 변경**:

```cpp
volatile bool yaw_hold_override = false;   // "yaw 1": heading 고정 + 재슬레이빙 금지
```

`yaw_enabled`를 참조하는 모든 지점을 `yaw_hold_override`로 바꾼다
(`grep -n yaw_enabled` 로 전수 확인).

- [ ] **Step 2: 바깥 루프 교체**

현행 822~836줄 블록을 다음으로 교체:

```cpp
    // ---------- Outer loop (250Hz): 각도 -> 목표 각속도 ----------
    // yaw는 각속도 명령 + 자동 heading 잠금. hold가 아닌 동안 setpoint를
    // 매 tick 현재 heading으로 슬레이빙해 stale setpoint를 원천 차단한다.
    const YawOuter yawOuter = updateYawOuter(
        targetYawRate, bodyGz, angleZ, targetAngleZ,
        yaw_hold_override, YAW_RATE_DEADZONE, YAW_HOLD_SETTLE_DPS);
    targetAngleZ = yawOuter.target_angle_deg;
    yaw_hold_now = yawOuter.hold;

    if (outerCnt == 0) {
      targetRateRoll = constrain((targetAngleX - angleX) * Kp_Angle_Roll,
                                 -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRatePitch = constrain((targetAngleY - angleY) * Kp_Angle_Pitch,
                                  -MAX_TARGET_RATE_RP, MAX_TARGET_RATE_RP);
      targetRateYaw = yawTargetRateDps(yawOuter, targetYawRate, angleZ,
                                       Kp_Angle_Yaw, MAX_TARGET_RATE_YAW);
    }
    outerCnt++;
    if (outerCnt >= OUTER_DIV) outerCnt = 0;
```

`const bool yawOn = yaw_enabled;`와 `if (!yawOn) targetRateYaw = 0.0f;` 두 줄은 삭제한다.
`yawOn`을 쓰던 적분 블록은 Task 4에서 정리하므로, 이 태스크에서는
`const bool yawOn = yawOuter.hold;`를 임시로 남겨 컴파일을 유지한다.

- [ ] **Step 3: safety_lock 분기에 stale setpoint 차단 추가**

현행 799줄 `if (safety_lock) {` 블록 안, `targetRateRoll = ... = 0.0f;` 다음 줄에 추가:

```cpp
      targetYawRate = 0.0f;
      targetAngleZ = angleZ;
      yaw_hold_now = false;
```

- [ ] **Step 4: 빌드와 회귀 확인**

```bash
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-yaw firmware/flight/dual_imu_cascade_pwm
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: 빌드 성공, `OK`

> 기존 SIL(`test_sil_attitude.cpp`)은 `yaw_enabled = false`로 초기화한다.
> 이름 변경 때문에 컴파일이 깨지므로 `yaw_hold_override = false`로 함께 고친다.

- [ ] **Step 5: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_sil_attitude.cpp
git commit -m "feat(flight): wire yaw rate command with auto heading hold"
```

---

## Task 4: yaw 적분항 해금

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino:866-878`
- Test: `tools/native_tests/test_sil_attitude.cpp`

**Interfaces:**
- Consumes: Task 3의 `yawOuter`
- Produces: `RunConfig::ki_yaw` (기본 0.05f),
  `RunConfig constantYawDisturbance(float ki_yaw, double torque_nm, uint32_t ticks)`,
  `double tailMeanAbsYawRateDps(const RunResult &result, std::size_t tail)`

- [ ] **Step 1: 실패하는 플랜트 테스트 작성**

`tools/native_tests/test_sil_attitude.cpp`의 `RunConfig`에 필드를 추가한다:

```cpp
  float ki_yaw = 0.05f;
```

`runSil()`에서 `Ki_Rate_Pitch = config.ki_pitch;` 바로 다음 줄에 추가한다:

```cpp
  Ki_Rate_Yaw = config.ki_yaw;
```

`constantRollDisturbance` 헬퍼 옆에 추가한다:

```cpp
RunConfig constantYawDisturbance(float ki_yaw, double torque_nm,
                                 uint32_t ticks) {
  RunConfig config;
  config.ticks = ticks;
  config.ki_yaw = ki_yaw;
  config.disturbance_for_interval = [torque_nm](uint32_t) {
    return Disturbance{0.0, 0.0, torque_nm};
  };
  return config;
}

double tailMeanAbsYawRateDps(const RunResult &result, std::size_t tail) {
  const std::size_t n = result.samples.size();
  const std::size_t start = n > tail ? n - tail : 0;
  double sum = 0.0;
  for (std::size_t index = start; index < n; index++) {
    sum += std::fabs(result.samples[index].plant.r) * kRadToDeg;
  }
  return sum / static_cast<double>(n - start);
}
```

`main()`에 케이스를 추가한다:

```cpp
  runCase("S6 yaw 적분 해금: P 단독은 정착 임계치를 못 넘고 적분은 넘긴다", [] {
    // yaw 권한은 roll의 6%다(yaw_moment_arm_m = 0.06 * arm_projection_m).
    // P 단독 정상상태 각속도가 대략 20dps가 되도록 외란을 잡는다:
    //   tau = yaw_torque_per_us * Kp_Rate_Yaw * r_ss
    const double yaw_torque_per_us =
        4.0 * kPlantParameters.yaw_moment_arm_m() *
        kPlantParameters.thrust_per_us_n;
    const double disturbance_nm = yaw_torque_per_us * 1.50 * 20.0;

    const RunResult p_only =
        runSil(constantYawDisturbance(0.0f, disturbance_nm, 10000));
    const double p_only_dps = tailMeanAbsYawRateDps(p_only, 500);
    std::cout << "[SIL] S6 P-only tail |r|=" << p_only_dps << "dps\n";

    // 판별력 확인: 외란이 너무 작으면 이 테스트는 아무것도 증명하지 못한다.
    CHECK_GE(p_only_dps, static_cast<double>(YAW_HOLD_SETTLE_DPS));

    const RunResult with_i =
        runSil(constantYawDisturbance(0.05f, disturbance_nm, 10000));
    const double with_i_dps = tailMeanAbsYawRateDps(with_i, 500);
    std::cout << "[SIL] S6 Ki=0.05 tail |r|=" << with_i_dps << "dps\n";

    // 적분이 살아야 정착 임계치 아래로 내려가 자동 잠금이 걸린다.
    CHECK_LE(with_i_dps, static_cast<double>(YAW_HOLD_SETTLE_DPS));
    CHECK(with_i.all_finite);
  });
```

`CHECK_GE`/`CHECK_LE`는 파일 상단(66·78줄)에 이미 있는 헬퍼를 쓴다.

- [ ] **Step 2: 실패 확인**

```bash
python -m unittest tools.test_sil_attitude -v 2>&1 | tail -20
```
Expected: FAIL — 현재는 `yawOn`이 false면 `iTermYaw`가 0으로 묶여 `Ki=0.05` 실행도
P 단독과 같아지므로 마지막 `CHECK_LE`에서 실패

- [ ] **Step 3: 적분 블록 교체**

현행 866~878줄을 다음으로 교체:

```cpp
    if (throttle > 1100 && !mix.scaled) {
      iTermRoll  = constrain(iTermRoll  + Ki_Rate_Roll  * eRoll  * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      iTermPitch = constrain(iTermPitch + Ki_Rate_Pitch * ePitch * realDt,
                             -I_TERM_MAX_US, I_TERM_MAX_US);
      // yaw 적분은 rate/hold 두 모드 모두에서 돈다. 안쪽 루프의 임무가
      // "목표 각속도 추종"이고, P 단독으로는 모터 토크 불균형 같은 일정
      // 외란에서 정상상태 각속도 오차가 남아 정착 임계치 아래로 내려가지
      // 않는다(2026-07-27 실측 최대 +16.6dps). 그러면 자동 잠금이 영영
      // 걸리지 않는다.
      iTermYaw = constrain(iTermYaw + Ki_Rate_Yaw * eYaw * realDt,
                           -I_TERM_MAX_US, I_TERM_MAX_US);
    } else if (throttle <= 1100) {
      iTermRoll = iTermPitch = iTermYaw = 0.0f;
    }
```

Task 3에서 임시로 남긴 `const bool yawOn = yawOuter.hold;`를 삭제한다
(더 이상 참조하는 곳이 없어야 한다 — `grep -n yawOn`으로 확인).

- [ ] **Step 4: 통과 확인**

```bash
python -m unittest tools.test_sil_attitude -v 2>&1 | grep -E "S6|^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-yaw firmware/flight/dual_imu_cascade_pwm
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `[PASS] S6 ...`, 빌드 성공(`yawOn` 미사용 경고 없음), `OK`

> S6의 P-only 값이 `YAW_HOLD_SETTLE_DPS` 미만이라 첫 `CHECK_GE`에서 실패하면
> 외란이 너무 작은 것이다. `disturbance_nm`의 `20.0`을 키워 판별력을 회복시킨다.

- [ ] **Step 5: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_sil_attitude.cpp
git commit -m "fix(flight): let the yaw rate integrator run in both yaw modes"
```

---

## Task 5: `yaw 1` 무장 게이트와 오버라이드 수명

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino` (yaw 핸들러, safety_lock 분기)
- Test: `tools/native_tests/test_control_math.cpp`

**Interfaces:**
- Consumes: Task 3의 `yaw_hold_override`, 기존 `safety_lock`, `wasLocked`
- Produces: 없음

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/native_tests/test_control_math.cpp`의 `main()`에 추가:

```cpp
  runCase("yaw 1은 무장 중 거부되고 yaw 0은 언제나 수락된다", [] {
    yaw_hold_override = false;
    safety_lock = false;                       // 무장 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(!yaw_hold_override);                 // 거부

    safety_lock = true;                        // 시동 해제 상태
    sendUdpCommandOnce("yaw 1");
    CHECK(yaw_hold_override);                  // 수락

    safety_lock = false;
    sendUdpCommandOnce("yaw 0");
    CHECK(!yaw_hold_override);                 // 끄기는 무장 중에도 허용
  });
```

- [ ] **Step 2: 실패 확인**

```bash
python -m unittest tools.test_native_control_math -v 2>&1 | tail -20
```
Expected: FAIL — 현재는 무장 중 `yaw 1`이 수락되어 첫 `CHECK`에서 실패

- [ ] **Step 3: yaw 핸들러 교체**

현행 1126~1132줄을 다음으로 교체:

```cpp
        else if (strncmp(buf, "yaw", 3) == 0) {
          long enabled;
          if (parseIntStrict(buf + 3, enabled) && (enabled == 0 || enabled == 1)) {
            if (enabled == 0) {
              yaw_hold_override = false;
              Serial.println(">>> Yaw hold OFF (auto)");
            } else if (!safety_lock) {
              // 비행 중 임의로 켜면 지상국 setpoint와 어긋난 채 최대 권한
              // 슬램이 날 수 있다. 시동 해제 상태에서만 켤 수 있게 한다.
              Serial.println(">>> Yaw hold refused (armed)");
            } else {
              targetAngleZ = angleZ;
              yaw_hold_override = true;
              Serial.println(">>> Yaw hold ON (override)");
            }
          }
        }
```

- [ ] **Step 4: 무장 해제 엣지에서 오버라이드 해제**

현행 799줄 `if (safety_lock) {` 블록의 **첫 줄**로 추가:

```cpp
      if (!wasLocked) yaw_hold_override = false;   // 무장 해제 엣지에서만 1회
```

> 이 분기는 잠긴 동안 매 tick 실행된다. 조건 없이 지우면 시동 해제 상태에서
> `yaw 1`을 켤 수 없게 되므로 반드시 `!wasLocked` 엣지로 한정한다.
> `wasLocked`는 `true`로 초기화되어 부팅 직후에는 발동하지 않는다.

- [ ] **Step 5: 통과 확인**

```bash
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-yaw firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, 빌드 성공

- [ ] **Step 6: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        tools/native_tests/test_control_math.cpp
git commit -m "feat(flight): gate yaw-hold override to disarmed, clear on disarm edge"
```

---

## Task 6: `Yaw_Hold` 텔레메트리 (34 → 35필드)

**Files:**
- Modify: `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino:1024`
- Modify: `scripts/telemetry_schema.py`
- Test: `tools/test_telemetry_schema.py`

**Interfaces:**
- Consumes: Task 3의 `yaw_hold_now`
- Produces: `TELEMETRY_FIELDS[34] == "Yaw_Hold"` (int)

- [ ] **Step 1: 실패하는 테스트 작성**

`tools/test_telemetry_schema.py`에 추가:

```python
    def test_35_field_packet_parses_yaw_hold(self):
        packet = ",".join(["1"] * 34 + ["1"])
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertEqual(sample["Yaw_Hold"], 1)

    def test_34_field_packet_leaves_yaw_hold_none(self):
        packet = ",".join(["1"] * 34)
        sample = telemetry_schema.parse_telemetry_packet(packet)
        self.assertIsNone(sample["Yaw_Hold"])

    def test_csv_has_36_columns(self):
        self.assertEqual(len(telemetry_schema.CSV_FIELDS), 36)
```

- [ ] **Step 2: 실패 확인**

```bash
python -m unittest tools.test_telemetry_schema -v 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `FAILED` — `KeyError: 'Yaw_Hold'`

- [ ] **Step 3: 스키마 갱신**

`scripts/telemetry_schema.py`:
- `TELEMETRY_FIELDS` 끝에 `"Yaw_Hold",` 추가
- `TELEMETRY_FIELD_TYPES`에 `"Yaw_Hold": int,` 추가
- 모듈 docstring의 필드 수 서술을 35필드로 갱신
- `parse_telemetry_packet` docstring의 허용 길이 목록에 `35` 추가

- [ ] **Step 4: 펌웨어 텔레메트리 확장**

1024줄 포맷 문자열 끝에 `,%d`를 붙이고, 인자 목록 끝에 `(int)yaw_hold_now`를 추가한다.

- [ ] **Step 5: 통과 확인**

```bash
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-yaw firmware/flight/dual_imu_cascade_pwm
```
Expected: `OK`, 빌드 성공

- [ ] **Step 6: 커밋**

```bash
git add firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino \
        scripts/telemetry_schema.py tools/test_telemetry_schema.py
git commit -m "feat(telemetry): append Yaw_Hold mode field"
```

---

## Task 7: 지상국 전환

**Files:**
- Modify: `scripts/control_dualsense.py`
- Modify: `scripts/bench_yaw_test.py`

**Interfaces:**
- Consumes: Task 2의 `rcr`, Task 5의 무장 게이트
- Produces: 없음

- [ ] **Step 1: `control_dualsense.py` 상수 교체**

`YAW_RATE = 1.0` 줄을 다음으로 교체:

```python
YAW_RATE_MAX_DPS = 90.0     # 스틱 최대 편향 시 yaw 각속도 (dps). 조종감은 여기서 조정
```

- [ ] **Step 2: `target_yaw` 상태 제거**

다음을 모두 삭제한다:
- 전역 `target_yaw = 0.0`
- `arm()`의 `global ... target_yaw` 항목과 `target_yaw = 0.0`
- `disarm()`의 `global ... target_yaw` 항목과 `target_yaw = 0.0`
- `controller_thread()`의 `global ... target_yaw` 항목
- `arm()`의 `reliable_send("yaw 0")` 한 줄 (펌웨어가 무장 해제 엣지에서 리셋한다)

`arm()`의 출력 문자열도 `>>> [SYSTEM] ARMED (시동 ON, mag ON)`으로 고친다.

- [ ] **Step 3: 전송 교체**

`controller_thread()` 말미의 rc 전송 블록을 다음으로 교체:

```python
            # --- RC 명령 전송 (yaw는 각속도 dps) ---
            rc_seq += 1
            final_roll  = deadzone(joy.get_axis(0))  * MAX_ANGLE + trim_roll
            final_pitch = deadzone(-joy.get_axis(1)) * MAX_ANGLE + trim_pitch
            yaw_rate    = deadzone(joy.get_axis(3), 0.12) * YAW_RATE_MAX_DPS

            send_cmd(f"rcr {rc_seq} {final_roll:.2f} {final_pitch:.2f} {yaw_rate:.1f}")
```

조작 안내 출력의 yaw 줄을 다음으로 고친다:

```python
    print(f" [L-Stick]  Roll / Pitch (max ±{MAX_ANGLE}°),  [R-Stick↔] Yaw 각속도 (max ±{YAW_RATE_MAX_DPS:.0f}°/s)")
```

- [ ] **Step 4: `bench_yaw_test.py` 순서 수정**

`sender()`에서 `send("yaw 1")`을 `start` 전송 **앞으로** 옮긴다:

```python
def sender():
    global _seq
    send("connect")
    time.sleep(0.2)
    send("yaw 1")          # 시동 해제 상태에서만 수락된다 (start보다 먼저)
    time.sleep(0.05)
    for _ in range(3):
        send("start")
        time.sleep(0.05)
    send(THROTTLE_CMD)
    while _running:
        _seq += 1
        send(f"rc {_seq} 0 0 0")   # 오버라이드 중이므로 yaw 절대각 0 유지
        time.sleep(0.05)
```

- [ ] **Step 5: 확인**

```bash
python -m py_compile scripts/*.py && echo "py_compile OK"
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
grep -n "target_yaw" scripts/control_dualsense.py || echo "target_yaw 잔여 없음 ✓"
```
Expected: `py_compile OK`, `OK`, `target_yaw 잔여 없음 ✓`

- [ ] **Step 6: 커밋**

```bash
git add scripts/control_dualsense.py scripts/bench_yaw_test.py
git commit -m "feat(scripts): send yaw as rate via rcr, drop target_yaw state"
```

---

## Task 8: 문서 갱신

**Files:**
- Modify: `docs/udp_protocol.md`, `logs/README.md`, `scripts/README.md`,
  `docs/power_on_bench_procedure.md`, `docs/project_overview.md`

**Interfaces:**
- Consumes: Task 2·5·6의 최종 동작
- Produces: 없음

- [ ] **Step 1: `docs/udp_protocol.md`**

- 명령 블록에 `rcr <seq> <roll> <pitch> <yaw_rate>` 추가 (주석: yaw는 dps)
- `yaw <0|1>` 설명을 교체: `yaw 1` = heading 고정 + 재슬레이빙 금지, **시동 해제 상태에서만 수락**, 무장 해제 엣지에서 자동 해제. `yaw 0` = 자동 모드(기본)
- `rc`는 벤치 도구용으로 유지되며 `yaw 1` 오버라이드와 함께 쓸 때만 yaw 필드가 의미를 갖는다고 명시
- 텔레메트리 34 → 35필드, 필드 표에 `| 35 | Yaw_Hold | int | 0=각속도 모드, 1=heading 잠금 |` 추가
- 레거시 목록에 34필드 항목 추가, CSV 35 → 36열

- [ ] **Step 2: `logs/README.md`, `scripts/README.md`**

필드 수 34 → 35, CSV 열 35 → 36. CSV 열 목록 끝에 `Yaw_Hold` 추가.

- [ ] **Step 3: `docs/power_on_bench_procedure.md`**

Stage C-3을 다음으로 갱신:
- `yaw 1`은 **시동 전에** 보낸다(무장 중 거부됨)
- 신규 항목: 스틱을 밀면 명령 방향으로 회전하고, 놓으면 `Yaw_Hold`가 1로 바뀌며 heading이 그 자리에 머무는지
- 신규 항목: 놓는 순간 오버슛(바운스백)이 없는지. 있으면 `YAW_HOLD_SETTLE_DPS`를 낮춘다
- 신규 항목: 프롭 ON에서 잠금이 안 걸리면 `YAW_HOLD_SETTLE_DPS`를 올린다
- Stage D에 yaw 적분 해금 관찰 추가: 진동 시 `Ki_Rate_Yaw`(0.05)를 낮춘다

통과 기록 체크리스트에 위 항목들을 추가한다.

- [ ] **Step 4: `docs/project_overview.md`**

"yaw 각도는 자기계 없이 자이로 적분만으로 추정하므로 드리프트한다. yaw 제어를 켜는
순간 현재 각도로 setpoint를 동기화한다"는 서술을 현재 동작으로 교체: yaw 스틱은
각속도 명령이고, 스틱 중립 + 각속도 정착 시 자동으로 heading을 잠근다. mag 융합이
켜져 있으면 그 잠금이 드리프트 없이 유지된다.

- [ ] **Step 5: 확인**

```bash
python tools/check_repo_layout.py
python -m unittest discover -s tools -p "test_*.py" 2>&1 | grep -E "^(Ran|OK|FAILED)"
```
Expected: `repository layout checks passed`, `OK`

- [ ] **Step 6: 커밋**

```bash
git add docs/ logs/README.md scripts/README.md
git commit -m "docs: yaw rate command, Yaw_Hold field, bench steps"
```

---

## 실기 검증 (구현 완료 후, 사람이 수행)

코드 작업이 끝나도 **비행 가능 상태가 아니다.** `docs/power_on_bench_procedure.md`
전 항목을 재실행해야 한다. 특히:

- Stage B-3: roll/pitch 부호 회귀 없음 (제어경로를 건드렸으므로 재확인)
- Stage C-3: yaw 부호 (`yaw 1` 오버라이드, `bench_yaw_test.py`)
- 신규: 각속도 명령 추종, 놓을 때 잠금 동작, 바운스백 없음
- Stage D: yaw 적분 해금 후 진동 없음
- 임계치 2개(`YAW_RATE_DEADZONE`, `YAW_HOLD_SETTLE_DPS`)를 실측으로 확정하고 값을 기록
