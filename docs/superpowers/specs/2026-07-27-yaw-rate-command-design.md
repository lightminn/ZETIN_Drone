# Yaw 각속도 명령 전환 — 설계 (2026-07-27)

대상: `firmware/flight/dual_imu_cascade_pwm`, `scripts/control_dualsense.py`

## 목표

조종 스틱의 yaw 입력을 **절대 각도 명령에서 각속도 명령으로** 바꾼다. 스틱을
놓으면 그 자리의 heading을 자동으로 잠근다(rate + heading hold).

## 배경 — 왜 바꾸는가

### 문제 1: yaw 스틱이 아예 동작하지 않는다

`arm()`이 시동할 때마다 `yaw 0`을 보내 `yaw_enabled = false`가 된다. 그러면
바깥 yaw 각도 루프가 실행되지 않아, 스틱이 만든 `target_yaw`는 전송되고
`targetAngleZ`에 저장까지 되지만 **읽는 코드가 돌지 않는다.**

근거: 2026-07-27 무장 구간 21개 전부에서 `TgtRate_Yaw` ≡ 0.00 (예외 0건).

### 문제 2: 기수가 지속적으로 회전한다

yaw-hold가 꺼져 있으면 펌웨어가 `iTermYaw`를 강제로 0으로 묶는다
(`else iTermYaw = 0.0f;`). `Kd_Rate_Yaw`도 0이므로 yaw 안쪽 루프는
**P 단독**(`Kp_Rate_Yaw = 1.50`)이다. P만으로는 프로펠러·모터 토크 불균형 같은
일정한 외란을 0으로 만들 수 없어 정상상태 각속도 오차가 남는다.

감쇠 자체는 정상 동작한다 — `Gyro_Z`와 yaw축 모터 차동의 상관계수가 **-1.000**
(회전에 저항). 그런데도 실측 회전이 남는다:

| 구간 | 지속 | Gyro_Z 평균 | Yaw 총변화 |
|---|---|---|---|
| 04:07:37 | 13.0s | -11.40 dps | -135.6° |
| 04:11:56 | 10.6s | **+16.62 dps** | **+189.3°** |
| 04:13:35 | 12.2s | -3.08 dps | -49.2° |

### 문제 3: 그냥 `yaw 1`로 켜면 최대 권한 슬램이 난다

펌웨어의 점프 방지(`targetAngleZ = angleZ` on enable)는 **50ms 뒤 도착하는 다음
rc 패킷이 덮어쓴다.** 지상국의 `target_yaw`는 자체 적분값(시동 시 0)이고 드론의
`angleZ`와 한 번도 동기화되지 않는다. 로그상 `angleZ`는 ±189°까지 갔으므로:

```text
오차 = wrapDeg(0 − 189°) ≈ +171°
목표각속도 = 171 × Kp_Angle_Yaw(3.0) = 513 dps → 180 dps로 클램프
```

즉 최대 권한 yaw 슬램. 아직 한 번도 켜본 적이 없어 사고는 없었다.

**근본 원인은 셋 다 하나다: setpoint가 두 곳(PC·드론)에 있고 동기화되지 않는다.**

## 설계 원칙

setpoint를 **드론에만** 둔다. 지상국은 각속도(상태 없는 순간 명령)만 보낸다.
heading 캡처도 펌웨어에서 한다 — 지상국이 하면 지금 문제를 그대로 재생산한다.

## 1. 펌웨어 yaw 상태기계

### 새 상수

| 이름 | 값 | 의미 |
|---|---|---|
| `YAW_RATE_DEADZONE` | 3.0 dps | 명령 각속도의 절댓값이 이하이면 스틱 중립으로 간주 |
| `YAW_HOLD_SETTLE_DPS` | 10.0 dps | `bodyGz`의 절댓값이 이하이면 정착으로 간주 |

기존 `MAX_TARGET_RATE_YAW = 180.0`은 두 모드 공통 하드 실링으로 유지한다.

### 새 상태

- `volatile float targetYawRate` — `rcr`이 준 각속도 명령(dps)
- `yaw_enabled` → `yaw_hold_override`로 의미 변경 (`yaw <0|1>`이 설정)

### 바깥 루프 (현행 823~836줄 교체)

```c
const bool stickCentered = fabsf(targetYawRate) < YAW_RATE_DEADZONE;
const bool settled       = fabsf(bodyGz) < YAW_HOLD_SETTLE_DPS;
const bool hold          = yaw_hold_override || (stickCentered && settled);

if (!hold) targetAngleZ = angleZ;          // 연속 슬레이빙 (1kHz, 매 tick)

if (outerCnt == 0) {                        // 250Hz
  targetRateYaw = hold
    ? constrain(wrapDeg(targetAngleZ - angleZ) * Kp_Angle_Yaw,
                -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW)
    : constrain(targetYawRate,
                -MAX_TARGET_RATE_YAW, MAX_TARGET_RATE_YAW);
}
```

슬레이빙은 1kHz 매 tick, 목표 각속도 계산은 기존대로 250Hz(`OUTER_DIV = 4`)다.

**히스테리시스를 넣지 않는다.** `hold` 경계에서 채터링이 나도 슬레이빙은
setpoint를 현재값으로 맞추는 것뿐이라 불연속이 생기지 않는다.

**정착하지 못하면 잠그지 않는다.** 타임아웃 강제 잠금을 두지 않고 rate 모드에
머문다. 정착 못 할 만큼 회전이 남는 상황에서 각도 루프를 얹으면 싸움만 난다.
그 경우 동작은 현행(감쇠만)과 같아 퇴화가 안전한 쪽이다.

### 잠금 해제 경로의 stale setpoint 차단

`safety_lock` 분기에서 `targetRateYaw`와 함께 다음을 리셋한다:

```c
targetYawRate = 0.0f;
targetAngleZ  = angleZ;
```

`safety_lock`은 `start`로만 해제되고 `start`가 `angleZ = 0; targetAngleZ = 0;`을
하므로 현재 코드에서도 일관되지만, stale setpoint 버그 클래스를 구조적으로
막기 위해 명시한다.

### `start` 처리 추가

`yaw_hold_override = false`로 리셋한다. 남아 있던 `yaw 1`이 다음 비행까지
따라오면 안 된다.

## 2. yaw 적분항 해금 (필수)

```c
if (yawOn) iTermYaw = constrain(...); else iTermYaw = 0.0f;   // ← else 제거
```

두 모드 모두에서 적분한다. `throttle > 1100 && !mix.scaled` 조건과
`I_TERM_MAX_US`(±50µs) 클램프는 유지한다.

**이것 없이는 설계가 성립하지 않는다.** 문제 2에서 측정된 정상상태 회전이
최대 +16.6 dps인데 정착 임계치가 10 dps다. 적분이 죽어 있으면 회전율이 임계치
아래로 내려가지 않아 **잠금이 영영 걸리지 않는다.**

알려진 부작용: 스틱을 오래 물고 있으면 적분이 그 회전을 지탱하려 쌓이고, 놓을
때 잠깐 남는다. 벤치 관찰 항목으로 잡는다.

## 3. 프로토콜

### 신설 명령

```text
rcr <seq> <roll> <pitch> <yaw_rate>
```

- `seq` 처리(중복·역순 폐기, 드롭 집계)와 `lastRcMs` 갱신은 `rc`와 **동일 상태를
  공유**한다(`lastRcSeq`, `rcSeqValid`, `rcTotalPkts`, `rcDroppedPkts`).
  지상국은 `rc`와 `rcr` 중 하나만 쓴다.
- `roll`/`pitch`: 기존과 동일하게 ±`MAX_TARGET_ANGLE_RP`(30°) 클램프
- `yaw_rate`: dps. 펌웨어에서 ±`MAX_TARGET_RATE_YAW`(180) 하드 클램프
- 파싱 실패·인자 수 불일치는 기존 `rc`와 동일하게 거부

### `rc`는 변경하지 않는다

벤치 도구 3종(`bench_sign_test.py`, `bench_thrust_ramp.py`, `bench_yaw_test.py`)이
그대로 동작한다. `yaw 1` + `rc`는 "절대 heading 지정"이라는 일관된 의미가 된다.

`rc`가 쓴 `targetAngleZ`는 `hold`가 아닐 때 다음 tick에 슬레이빙으로 덮인다.
무해하며, 오버라이드가 켜져 있을 때만 의미를 갖는다.

### `yaw <0|1>` 의미 변경

| 값 | 의미 |
|---|---|
| `yaw 0` (기본) | 자동 모드 — 스틱 중립 + 정착 시 잠금 |
| `yaw 1` | heading 고정 + **재슬레이빙 금지** (오버라이드) |

`bench_yaw_test.py`가 이 오버라이드에 의존한다. 자동 모드에서는 손으로 비틀어
유지해도 컨트롤러가 그 새 heading을 채택해버려 복원 토크가 나오지 않으므로,
yaw 부호 판정에는 고정 setpoint가 필요하다.

### 최대 회전 속도는 지상국 상수

`YAW_RATE_MAX_DPS = 90.0`을 `control_dualsense.py`에 둔다. 펌웨어에는 180 dps
하드 실링만 남긴다. 조종감 튜닝에 재플래시가 필요 없고, hold 복원에 쓸 여유
(90 ↔ 180)도 남는다.

## 4. 텔레메트리 (34 → 35필드)

`Yaw_Hold`(int, 0 = rate / 1 = hold)를 35번째 필드로 append한다. CSV는 36열.

`TgtRate_Yaw`(30번)는 두 모드 모두의 최종 목표 각속도라 이미 관측되지만 **어느
모드인지** 알 방법이 현재 없다. 벤치에서 잠금 동작을 검증하려면 필요하다.

`scripts/telemetry_schema.py`에 필드와 타입(`int`)을 추가한다. append-only이므로
기존 34/31/30/22/21/14/10필드 패킷 하위호환은 자동으로 유지된다.

## 5. 지상국 (`control_dualsense.py`)

- `target_yaw` 누적 상태를 **전면 삭제**한다(전역 선언, `arm()`·`disarm()`의
  리셋 포함). 이 상태가 문제 3의 근원이었다.
- `YAW_RATE_MAX_DPS = 90.0` 추가, 기존 `YAW_RATE = 1.0` 제거
- 매 루프: `yaw_rate_cmd = deadzone(joy.get_axis(3), 0.12) * YAW_RATE_MAX_DPS`
- 전송: `send_cmd(f"rcr {rc_seq} {final_roll:.2f} {final_pitch:.2f} {yaw_rate_cmd:.1f}")`
- `arm()`의 `reliable_send("yaw 0")` 제거 — 펌웨어가 `start`에서 리셋하므로 중복
- 조작 안내 출력의 yaw 설명을 각속도 기준으로 수정

## 6. 테스트

### 네이티브 SIL (`tools/native_tests/test_sil_attitude.cpp`)

1. 스틱 편향 중 → `targetRateYaw`가 명령을 추종하고 `targetAngleZ`가 `angleZ`를
   따라가 각도 오차가 누적되지 않음
2. 빠르게 회전 중 스틱 놓음 → **즉시 잠기지 않고** rate 모드 유지, `bodyGz`의
   절댓값이 `YAW_HOLD_SETTLE_DPS` 아래로 내려간 뒤에 잠김
3. 잠긴 뒤 외란으로 heading이 밀리면 복원 방향으로 `targetRateYaw` 발생(부호 검증)
4. `yaw 1` 오버라이드 → 슬레이빙 정지, 각도 오차가 유지됨
   (`bench_yaw_test.py` 의미론)
5. **회귀 테스트(문제 3)**: `angleZ`가 크게(예: 150°) 드리프트한 상태에서 hold로
   진입해도 `targetRateYaw` ≈ 0 이어야 한다. 슬램이 재발하면 실패한다.
6. rate 명령이 ±`MAX_TARGET_RATE_YAW`로 클램프됨
7. 적분 해금 확인: 일정한 yaw 외란 토크 하에서 정상상태 회전율이 0으로 수렴하고
   정착 임계치 아래로 내려가 잠금이 걸림 (문제 2의 회귀 테스트)

### 제어수학 (`tools/native_tests/test_control_math.cpp`)

`rcr` 파서 — 유효/무효 인자, `seq` 중복·역순 폐기, 인자 수 불일치 거부.
기존 `rc` 파서 테스트를 미러링한다.

### 스키마 (`tools/test_telemetry_schema.py`)

35필드 파싱, `Yaw_Hold` 정수 타입, 34/31/30/22/21/14/10필드 하위호환.

### 실기 벤치 (필수)

제어경로 변경이므로 `power_on_bench_procedure.md`를 갱신하고 재실행한다.

- Stage C-3(yaw 부호)을 `yaw 1` 오버라이드 기준으로 갱신
- 신규: 스틱을 밀면 기체가 명령 방향으로 회전하고, 놓으면 `Yaw_Hold`가 1로 뜨며
  heading이 그 자리에 머무는지
- 신규: 놓는 순간 오버슛(바운스백)이 없는지
- Stage D: yaw 적분 해금 후 진동이 없는지. 진동 시 `Ki_Rate_Yaw`(현재 0.05)를 낮춘다

## 7. 위험

- **비행 제어경로 변경 → 첫 호버 전 벤치 전 항목 재검증 필수**
- yaw 적분 해금이 yaw 동역학을 바꾼다. 진동 발생 시 `Ki_Rate_Yaw`를 낮춘다
- 펌웨어/지상국 버전 불일치 → 구펌웨어가 `rcr`을 무시 → RC 워치독 500ms 발동 →
  안전한 시동 거부. **설계된 실패 모드**이며 조용한 오해석보다 낫다
- `yaw 1` 오버라이드를 켠 채 비행하면 문제 3의 슬램 조건이 그대로 남는다.
  오버라이드는 벤치 전용으로 문서화하고 `start`에서 리셋한다

## 8. 문서 갱신 대상

- `docs/udp_protocol.md` — `rcr`, `yaw <0|1>` 의미 변경, `Yaw_Hold` 필드
- `logs/README.md` — 35필드 / 36열
- `scripts/README.md` — 필드 수
- `docs/power_on_bench_procedure.md` — Stage C-3, 신규 yaw 확인 항목
- `docs/project_overview.md` — yaw 드리프트 한계 서술 갱신

## 비목표 (YAGNI)

- 각속도 명령 전용 진단 스케치
- yaw 최대 각속도의 런타임 변경 명령 (지상국 상수로 충분)
- 잠금 임계치의 런타임 변경 명령
- `rc` 명령 폐기 (벤치 도구가 쓴다)
