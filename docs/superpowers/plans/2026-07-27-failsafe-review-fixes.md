# 자동착륙 코드리뷰 지적사항 수정 계획

2026-07-27. `/code-review max` 결과 15건 중 실코드에서 확인된 13건을 수정한다.
근거 데이터와 물리는 아래 §0에 정리한다.

## 0. 착지 감지기가 왜 틀렸나 (설계 오류)

기존 판정: "min_descend 이후 sub-1g를 본 적 있고, |accel|이 1g±0.05 안에서
400ms 유지되면 착지."

세 가지가 동시에 틀렸다.

1. **증거를 게이트가 버린다.** `elapsed_ms < min_descend_ms`가 sub-1g 기록보다
   **먼저** return하므로, 진입 직후 하강 과도구간의 sub-1g가 `saw_sub_1g`에
   기록되지 않는다. 게이트는 *판정*을 미뤄야지 *증거*를 버리면 안 된다.
2. **등속 하강은 정확히 1g다.** 가속도계는 비력을 재므로 정상 하강 중에도
   1g를 읽는다. "1g 복귀"는 접지 신호가 아니다.
3. **노이즈가 확정 창을 못 채운다.** 최근 비행로그 6개 실측:

   | 로그 | 정지 \|accel\| 중앙값 | 무장 중 sd | ±0.05g 내 비율 |
   |---|---|---|---|
   | 041032 | 1.0075 | 0.028 | 93.7% |
   | 040904 | 1.0071 | 0.037 | 87.4% |
   | 040715 | 1.0070 | 0.050 | 88.5% |
   | 035231 | 1.0084 | 0.035 | 85.1% |

   20Hz 텔레메트리 샘플 기준으로도 10% 남짓이 창 밖이다. 1kHz 원신호는 더
   심하다. 400ms(=400샘플) 연속 유지는 성립하지 않는다. 바이어스 자체는
   +0.8%로 작아 문제가 아니다 — **노이즈가 문제다.**

즉 현행 구현은 **FS_CUT_LANDED에 절대 도달하지 못하고** 모든 자동착륙이 5초
타임아웃 컷으로 끝난다.

**새 판정**: 접지의 유일한 명확한 신호는 지면 반력에 의한 **1g 초과 스파이크**다.
등속 하강은 원리적으로 1g를 넘지 않으므로 공중 오검출이 물리적으로 불가능하다.
0.4m/s로 다리에 닿으면 감속거리 ~2cm 기준 v²/2s ≈ 4m/s² ≈ 0.4g 스파이크가 난다.
프롭 진동은 100~300Hz라 5Hz LPF로 뭉갠다.

## 1. 펌웨어 — 착지 감지기 재설계 (`failsafe_land.h`)

`LandDetector`에 필터 상태와 충격 플래그를 추가한다.

```c
struct LandDetector {
  bool     filt_init = false;
  float    filt = 1.0f;         // LPF된 |accel| (g)
  bool     saw_sub_1g = false;  // 실제로 하강 가속했다는 증거
  bool     saw_impact = false;  // 지면 반력 스파이크
  bool     settling = false;
  uint32_t settle_start_ms = 0;
};
```

`updateLandDetector(det, accel_g, elapsed_ms, lpf_alpha, settle_tol_g,
impact_g, min_descend_ms, confirm_ms)`:

1. **증거 수집 — 게이트보다 먼저, 항상 실행.**
   - `filt` 갱신 (1차 LPF; 첫 샘플은 대입)
   - `filt < 1 - settle_tol` → `saw_sub_1g = true`, `settling = false`, return false
   - `filt > 1 + impact_g` → `saw_impact = true`, `settling = false`, return false
2. **판정 게이트.**
   - `elapsed_ms < min_descend_ms` → `settling = false`, return false
   - `!saw_sub_1g || !saw_impact` → `settling = false`, return false
   - `fabsf(filt - 1) > settle_tol` → `settling = false`, return false
   - `settling`이 아니면 시작 시각 기록 후 return false
   - `elapsed_ms - settle_start_ms >= confirm_ms` → **true**

상수(전부 벤치 조정 대상, `.ino`):

```c
const float    FS_LAND_LPF_ALPHA   = 0.03f;   // 1kHz에서 약 5Hz
const float    FS_LAND_SETTLE_TOL_G= 0.10f;   // ACC_DEV_SOFT와 동일
const float    FS_LAND_IMPACT_G    = 0.25f;   // 1g 초과 스파이크 임계
const uint32_t FS_LAND_CONFIRM_MS  = 400;
```

기존 `FS_LAND_ACCEL_TOL_G`는 삭제한다.

## 2. 펌웨어 — 지상 무장 중 RC 끊김은 즉시 컷

진입 스로틀이 호버에 못 미치면 공중에 있을 수 없다. 예전의 1ms 즉시 컷을
그 경우에만 되살린다.

```c
const int FS_GROUND_THROTTLE_US = 1200;   // 벤치에서 확정
```

진입 블록에서 `base_throttle < FS_GROUND_THROTTLE_US`면 `fs_phase`는
`FS_NONE`으로 두고 `fault_rc = true; safety_lock = true;`만 세운다(로그 문구
구분). 그 외에는 기존대로 `FS_DESCENDING`.

## 3. 펌웨어 — 하강 중 스로틀 창 고정

`th` 핸들러는 `base_throttle`뿐 아니라 `min_throttle`/`max_throttle`도 쓴다.
하강 블록은 `base_throttle`만 매 tick 되돌리므로, 하강 중 도착한 `th 1800`이
`min_throttle`을 1650으로 올리고 믹서의 collective 바닥이 그걸 강제해 기체가
**상승한다**(telemetry는 여전히 하강 스로틀과 Phase=1을 보고). 하강 블록에서
같이 고정한다.

```c
min_throttle = max(1050, base_throttle - CTRL_MARGIN);
max_throttle = min(1900, base_throttle + CTRL_MARGIN);
```

## 4. 펌웨어 — `CTRL_MARGIN` 파일 스코프 + static_assert

`CTRL_MARGIN`은 `udp_task` 지역변수(ino:1158)라 1000줄 앞의 주석 불변식을
컴파일 타임에 검사할 수 없다. 파일 스코프 `constexpr int CTRL_MARGIN = 150;`로
올리고 `static_assert(FS_DESCENT_DELTA_US < CTRL_MARGIN, ...)`를 건다.

주석에 실제 제약도 남긴다: 진짜 하한은
`delta < min(CTRL_MARGIN, entry - 1050)`이며 낮은 호버 스로틀에서 더 빡빡하다
(예: `th 1150`에서는 delta=120도 이미 실패). 이건 런타임 조건이라
static_assert로 못 잡는다.

## 5. 펌웨어 — `fs_phase` 단일 소유자

`fs_phase`는 pid_task(core 1)와 udp_task(core 0) 양쪽이 쓴다. pid_task의
`if (fs_phase == FS_DESCENDING) fs_phase = FS_CUT_ABORT;`는 비원자적 RMW라,
그 사이 udp_task의 `start`가 `fs_phase = FS_NONE; safety_lock = false;`를
끝내면 **무장 상태인데 fs_phase=4**로 래치된다. 그러면 진입 가드
`fs_phase == FS_NONE`이 영구히 거짓이 되어 **RC 끊김 감지가 그 비행 내내 죽는다**
(자동착륙도, 컷도, fault_rc도 없음 — 배터리가 닳을 때까지 마지막 명령대로 비행).

수정: `start` 핸들러에서 `fs_phase = FS_NONE;`을 **삭제**하고, pid_task가
잠금 해제 첫 tick(`if (wasLocked)` 블록, ino:883)에서 `fs_phase = FS_NONE;`을
수행한다. 이러면 `fs_phase`의 쓰기 주체가 pid_task 하나뿐이다.

## 6. 펌웨어 — 하강 중 `targetAngleZ` 주입 차단

하강 블록은 `targetAngleX/Y`와 `targetYawRate`는 덮어쓰지만 `targetAngleZ`는
안 덮는다. 4인자 `rc <seq> 0 0 90`은 `setRcTargets`의 `if (hasYaw)
targetAngleZ = z;`(ino:1009, 클램프·가드 없음)로 들어가고, 하강 중엔
`targetYawRate`가 0으로 고정돼 stick_centered가 항상 참이라 yaw-hold가 그 값을
붙잡는다 → `wrapDeg(90) * Kp` = 180dps 클램프. 무인 하강 중 최대 권한 회전이다.
벤치 스크립트 3종이 모두 4인자 형식을 20Hz로 스트리밍한다.

수정: 진입 시 `fs_hold_yaw = angleZ;`를 스냅샷하고, 하강 블록에서
`targetAngleZ = fs_hold_yaw;`를 매 tick 재적용한다.

## 7. 지상국 `control_dualsense.py`

### 7-1. `is_armed`와 `is_streaming` 분리 (치명)

`stop_streaming_only`가 `is_armed`를 지우는 바람에:
- X 버튼이 disarm→**arm**으로 뒤바뀐다. 하강 중인 드론 앞에서 킬 버튼을 누르면
  `trim`/`mag 1`/`start`가 나간다. 컷이 이미 끝난 뒤라면 그 `start`는 **수락되어**
  손에 든 드론의 모터를 1100으로 돌린다.
- `except KeyboardInterrupt: if is_armed: disarm()`이 no-op이 되어 Ctrl-C가
  `stop`을 안 보낸다.
- `Fault_Critical` 상승엣지와 `Armed==0` 워치독의 `disarm()`도 `if is_armed:`
  뒤라 **영구히 도달 불가**가 된다. 설계상 "유지"로 분류했던 두 자동 `stop`
  경로가 자동착륙 진입 직후 통째로 사라진다.

`docs/ground_station_link.md`가 "수동 킬은 X 버튼, Ctrl+C, stdin `stop`"이라
적어놨는데 셋 중 둘이 무력하다.

수정: 상태를 둘로 나눈다.
- `is_streaming` — rc/rcr/th 송신 여부. `stop_streaming_only`가 이것만 끈다.
- `is_armed` — 드론이 무장 상태라는 지상국의 믿음. `stop`을 실제로 보낸
  `disarm()`에서만 내린다.

송신 게이트는 `is_streaming`, 버튼·Ctrl-C·고장 핸들러의 킬 판단은 `is_armed`.

### 7-2. `arm()`이 드론 트림을 파괴한다 (치명)

`arm()`은 무조건 `send_trim()`한다. 스크립트를 재시작하면 모듈 전역이 0.0이라
첫 시동에서 `trim 0.00 0.00`이 나가 **드론에 저장된 트림을 지운다**. 트림을
펌웨어로 옮긴 목적 자체를 무효화하며, 다음 RC 끊김 자동착륙이 목표 0으로
내려가 설계문서가 2°에서 4.3m로 산정한 그 드리프트를 그대로 낸다.

수정: `arm()`에서 `send_trim()` 호출을 제거한다. 대신 텔레메트리의
`Trim_Roll`/`Trim_Pitch`를 첫 수신 시 지역 변수에 **읽어와 동기화**한다
(스키마에 이미 있다). `send_trim()`은 조종자가 D-pad로 실제 변경했을 때만
보낸다. 드론이 재부팅했으면 그 트림은 0이고 지상국도 0을 채택한다 — 소실은
정직하게 드러나되 파괴는 안 한다.

### 7-3. `rc_seq = 0` 리셋 제거

`arm()`이 `rc_seq = 0`으로 되돌리는데, 드론이 아직 무장 상태면 동반 `start`가
거부되어 펌웨어의 `lastRcSeq = 0; rcSeqValid = false;`(ino:1219)가 실행되지
않는다. 이후 모든 `rcr`이 `advance <= 0`으로 폐기되고, `lastRcMs` 갱신은
`setRcTargets` 안에 있으므로 **워치독도 안 먹인다**. 직전 비행이 60초였다면
약 60초간 RC 완전 두절이며 콘솔은 ARMED로 보인다.

수정: `arm()`에서 `rc_seq = 0`을 제거하고 세션 내내 단조 증가시킨다. 펌웨어가
`start` 수락 시 `rcSeqValid = false`로 만들므로 첫 패킷은 어차피 무조건
받아들여진다.

### 7-4. 트림 입력 정리

- 트림 리셋 버튼(`:373`)만 엣지 검출이 없어, 누르고 있으면 20Hz마다 블로킹
  100ms `send_trim()`을 호출한다. 엣지 검출을 넣는다.
- D-pad 트림 누산에 상한이 없어 펌웨어의 ±10° 클램프와 콘솔 표시가 어긋난다.
  지상국에서도 ±10°로 클램프한다.

## 8. 벤치 스크립트 3종의 트림 누출

`bench_thrust_ramp.py`, `bench_sign_test.py`, `bench_yaw_test.py`는 모두
`rc <seq> 0 0 0`을 "수평 유지"로 쓰는데, 이제 `setRcTargets`가 펌웨어 트림을
더한다. 트림은 start/stop을 의도적으로 살아남으므로,
`control_dualsense.py` 세션 뒤 전원을 안 내리고 벤치 스크립트를 돌리면 최대
±10° 뱅크를 조용히 명령한다. `bench_thrust_ramp.py:280`은 그 결과를
"spread 큼=CG/트림 치우침"이라 출력해 **펌웨어 트림 누출을 기체 결함으로
오진**시킨다.

수정: 세 스크립트 모두 시작 시 `trim 0 0`을 보낸다.

## 9. 테스트 하네스 — 시계가 안 흐른다

`arduino_fake::millis()`는 고정된 `millis_value`를 돌려주고 `runPidTicks`는
`pre_tick_hook`을 걸지 않는다. 따라서 `elapsed = nowMs - fs_enter_ms`가 호출
내내 상수이고, **failsafe 상태머신을 통과하는 테스트가 하나도 없다**:

- `fs_enter_ms = nowMs;`를 지워도 실패하는 테스트가 없다.
- `회귀: 하강 중 rc가 돌아와도 제어권이 복귀하지 않는다`의 두 번째
  `runPidTicks`는 `pid_task`를 처음부터 재진입해 `fs_entry_throttle`을 재초기화하고
  `failsafeDescentThrottle(1000, 60)` = 1000(하강이 아니라 idle)을 계산한다 —
  테스트 전제인 하강 스로틀이 사라진다.
- `FS_CUT_LANDED`에 도달하는 통합 테스트가 없어 §0의 결함이 CI에 안 보였다.
- `회귀: 진입 로그는 하강 내내 딱 한 번만 나간다`가 세는 부분문자열
  `"AUTO-LAND"`는 새 `>>> AUTO-LAND END phase=%d`에도 매칭된다(현재는 시계가
  멈춰 그 printf가 안 돌아 우연히 통과).

수정:
- shim에 `us_per_tick`(기본 0)을 추가해 tick마다 `micros_value`/`millis_value`를
  전진시킨다. `runPidTicks(ticks, us_per_tick = 1000)`.
- 로그 카운트를 `"[FAULT] RC TIMEOUT -> AUTO-LAND"` 전체 문자열로 바꾼다.
- 진입→하강→접지 스파이크→안정 → `FS_CUT_LANDED` 통합 테스트를 추가한다.
- 진입→타임아웃 → `FS_CUT_TIMEOUT` 통합 테스트를 추가한다.
- `fs_enter_ms = nowMs;` 삭제가 실패로 잡히는지 변조로 확인한다.

## 10. 검증

각 수정마다 단위/통합 테스트를 먼저 RED로 확인한 뒤 GREEN으로 만든다.
전체 통과 조건: `tools/native_tests` 전량 + Python 28건 + 아두이노 컴파일.

**이 수정 뒤에도 벤치 검증 없이는 비행 금지다.** SIL 플랜트에 수직 동역학이
없어 하강도 착지도 시뮬레이션할 수 없다. 특히 `FS_LAND_IMPACT_G`,
`FS_LAND_LPF_ALPHA`, `FS_GROUND_THROTTLE_US`, `FS_DESCENT_DELTA_US`는
전부 벤치 실측으로 확정해야 한다.
