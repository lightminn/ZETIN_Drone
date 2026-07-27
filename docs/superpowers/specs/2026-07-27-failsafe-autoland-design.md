# Failsafe 자동착륙 — 설계 (2026-07-27)

대상: `firmware/flight/dual_imu_cascade_pwm`, `scripts/control_dualsense.py`

## 목표

RC 링크가 끊겼을 때 즉시 모터를 끄는 대신, 수평 자세를 유지하며 천천히 하강해
착륙한다. 착지를 감지하면 모터를 끄고, 감지에 실패해도 정해진 시간에 반드시 끈다.

## 물리적 제약 — 설계를 규정하는 조건

이 세 가지가 무엇을 만들 수 있는지 결정한다.

- **고도 센서가 없다.** 기압계도 거리계도 없다(`us100`은 archive에만 존재).
  따라서 하강률을 폐루프로 제어할 수 없고, 고도로 착지를 감지할 수 없다.
  하강은 **개루프**다.
- **위치 센서가 없다.** GPS도 옵티컬플로우도 없다. 수평 위치를 유지할 수단이
  없으므로 **"제자리 착륙"은 불가능하다.** 만들 수 있는 것은 "수평 자세를
  유지하며 하강"이고, 착지 지점은 바람과 트림 오차가 결정한다.
- **배터리 전압 센싱이 없다.** 저전압 자동착륙은 이번 범위 밖이다.

## 적용 범위 — fault 종류에 따라 자동착륙이 불가능하다

| fault | 자동착륙 | 이유 |
|---|---|---|
| **RC 타임아웃** (`fault_rc`) | **적용** | 기체는 정상이고 링크만 끊겼다. 자세 추정을 신뢰할 수 있다 |
| IMU 전멸·중재 불가 | 즉시 컷 | **자세 추정 자체가 무효**다. 수평을 유지할 수 없어 확실히 날아간다 |
| 과도 기울기(60°+) | 즉시 컷 | 이미 뒤집히는 중이다. 모터를 돌리면 지면으로 박는 방향이 된다 |
| `stop` 명령 | 즉시 컷 | 조종자가 지금 멈추라고 지시한 것이다 |
| 캘리브레이션 실패 | 해당 없음 | 시동 전 조건이다 |

지상국의 **텔레메트리 손실**도 이 기능의 대상이지만 드론 입장에서는 RC
타임아웃과 같은 사건이다(드론이 아는 것은 "rc가 오지 않는다"뿐이다).
§7의 지상국 변경으로 연결된다.

## 1. 상태기계

```text
FS_NONE ──fault_rc 상승──▶ FS_DESCENDING ──착지 감지──▶ FS_CUT_LANDED
                                │
                                ├──FS_MAX_MS 경과─────▶ FS_CUT_TIMEOUT
                                └──IMU 전멸/과도기울기──▶ FS_CUT_ABORT
```

### 진입 (`FS_NONE` → `FS_DESCENDING`)

진입 조건은 **`fs_phase == FS_NONE` 이면서 `rcTimedOut()`** 이다. `fault_rc`는
latch되고 RC 타임아웃 검사도 매 tick 참이 되므로, 재진입을 막는 것은
`fault_rc`가 아니라 **상태 자체**다.

- `fs_entry_throttle = base_throttle` — 이 값이 호버 스로틀 추정치다
- `base_throttle = fs_entry_throttle - FS_DESCENT_DELTA_US`
- `targetAngleX = trim_roll; targetAngleY = trim_pitch;` — **0이 아니라 트림값**.
  이유는 §3 참조
- `targetYawRate = 0.0f` — 조종자의 마지막 yaw 명령을 지운다. 이후 yaw 자동
  잠금이 정착 후 heading을 잡는다(별도 처리 불필요)
- `fs_enter_ms = millis()`
- **`safety_lock`은 세우지 않는다.** 이것이 현행 동작과의 핵심 차이다

현행 코드(808~811줄)는 다음과 같다:

```c
if (!safety_lock && rcTimedOut(nowMs, lastRcMs)) {
  fault_rc = true; safety_lock = true;
  Serial.println("[FAULT] RC TIMEOUT");
}
```

### ⚠️ `safety_lock`이 재진입 방지 역할까지 하고 있다

지금은 `safety_lock = true`가 되면서 `!safety_lock` 가드 때문에 이 블록이 **한 번만**
실행된다. `safety_lock` 설정을 제거하면 하강 내내 `rcTimedOut()`이 참이므로 이 블록이
**1kHz로 재실행된다.** 그러면 `Serial.println`이 초당 1000번 나간다.

115200 baud는 약 11.5KB/s인데 20바이트 × 1000 = 20KB/s다. **TX 버퍼가 포화되면
`Serial.println`이 블로킹되고, `pid_task`가 멈추면 500ms `esp_task_wdt`가
비행 중 panic 재부팅을 일으킨다.**

따라서 진입 블록 전체(로그 포함)를 `fs_phase == FS_NONE` 안에 넣어야 한다:

```c
if (fs_phase == FS_NONE && !safety_lock && rcTimedOut(nowMs, lastRcMs)) {
  fault_rc = true;
  fs_phase = FS_DESCENDING;
  fs_entry_throttle = base_throttle;
  fs_enter_ms = nowMs;
  Serial.println("[FAULT] RC TIMEOUT -> AUTO-LAND");   // 한 번만
}
```

**하강 중에는 어떤 주기적 Serial 출력도 넣지 않는다.** 상태 관측은 텔레메트리
`Failsafe_Phase`로 한다.

### `start`는 하강 중 무시된다

`start` 핸들러는 `if (!safety_lock)`이면 "START ignored (already armed)"로 빠진다.
하강 중에는 `safety_lock`이 false이므로 **조종자가 착륙을 중단하고 제어를 되찾을 수
없다.** "재시동 요구" 결정과 일치하지만, 나쁜 지점으로 내려가는 것을 보면서도 개입할
수 없다는 뜻이므로 절차 문서에 명시한다. 개입 수단은 `stop`(즉시 컷)뿐이다.

### `fs_phase` 리셋

`start` 성공 시 `fs_phase = FS_NONE`으로 되돌린다. 리셋하지 않으면 이전 비행의
`FS_CUT_*`가 남아 텔레메트리가 잘못된 상태를 계속 보고한다.

### 하강 중 (`FS_DESCENDING`)

정상 제어 루프가 그대로 돈다. roll/pitch는 수평 목표를 추종하고, yaw는 자동
잠금이 heading을 유지한다.

**목표값을 매 tick 강제로 덮어쓴다:**

```c
targetAngleX = trim_roll;
targetAngleY = trim_pitch;
targetYawRate = 0.0f;
base_throttle = fs_entry_throttle - FS_DESCENT_DELTA_US;
```

진입 시 한 번만 설정하면 안 된다. **링크가 하강 중에 돌아오면 `rc`/`rcr`
파서가 `targetAngleX/Y`와 `targetYawRate`를 다시 쓰기 때문**이다. 그러면
"자동 복귀 없음" 결정과 달리 기체가 조종 입력을 따르기 시작하면서 동시에
하강한다. 매 tick 덮어쓰면 파서가 무엇을 쓰든 무해해지므로, 파서 쪽에
failsafe 분기를 넣을 필요가 없다.

같은 이유로 링크가 돌아와 `rcTimedOut()`이 거짓이 되어도 **상태는 바뀌지
않는다.** 종료는 아래 세 경로뿐이다.

### 종료 (세 경로 모두 동일한 처리)

`safety_lock = true` + `stopMotors()`. **자동 복귀는 없다.** 조종자가 `start`로
재시동해야 한다. 링크가 하강 중에 돌아와도 착륙을 끝까지 진행한다.

## 2. 착지 감지

### 원리

- **공중에서 스로틀이 호버보다 낮으면** 기체가 아래로 가속하므로 측정 가속도
  크기가 **1g보다 작아진다**
- **지면에 닿으면** 지면이 받쳐주므로 스로틀과 무관하게 **1g로 돌아온다**

### 오탐 차단 — 이게 없으면 진입 즉시 착지로 오판한다

진입 순간에는 스로틀이 아직 호버 근처라 `|accel|`이 이미 1g다. 그대로 판정하면
**하강을 시작하기도 전에 착지로 오판**한다. 두 겹으로 막는다:

1. **`FS_MIN_DESCEND_MS` 게이트** — 진입 후 이 시간 전에는 착지 판정을 하지 않는다
2. **sub-1g 선행 조건** — `|accel| < 1 - FS_LAND_ACCEL_TOL_G`를 **한 번이라도 본
   뒤**에만 1g 복귀를 착지로 인정한다. 이것이 "실제로 하강 중이었다"는 증거다

### 판정

sub-1g를 본 뒤 `|accel|`이 `1 ± FS_LAND_ACCEL_TOL_G` 범위에
`FS_LAND_CONFIRM_MS` 동안 유지되면 착지로 확정한다.

가속도는 이미 필터된 융합값(`accX/accY/accZ`)을 쓴다.

## 3. 트림을 펌웨어로 이전

### 왜 필요한가

현재 트림은 지상국에만 있다.

```text
control_dualsense.py:52-53    trim_roll / trim_pitch          전역 상태
control_dualsense.py:357-358  final_roll = 스틱 + trim_roll   송신 전 합산
펌웨어                         트림 개념 없음                   합산된 각도만 받는다
```

트림이 필요하다는 것은 **추정기의 0°가 진짜 수평이 아니라는 뜻**이다(가속도계
바이어스, IMU 장착 각도). 조종자가 `trim_roll = +2°`를 넣어야 흐르지 않는다면
진짜 수평은 추정기 기준 +2°다.

따라서 자동착륙에서 `targetAngle = 0`을 명령하면 **트림을 넣기 전의 흐름이
그대로 재현된다.** 수평 가속도는 `g·tan(θ)`이고 이동거리는 시간의 제곱으로 늘어난다.

| 트림 | 수평 가속도 | 5초 후 이동 | 접지 시 수평속도 |
|---|---|---|---|
| 1° | 0.17 m/s² | 2.1 m | 0.9 m/s |
| 2° | 0.34 m/s² | **4.3 m** | **1.7 m/s** |
| 3° | 0.51 m/s² | 6.4 m | 2.6 m/s |

수평속도 1.7m/s로 접지하면 기체가 구른다. **"손상 없는 부드러운 착륙"이라는 이
기능의 목적 자체가 무너진다.**

rate 적분기는 이것을 고치지 못한다. 적분기는 각속도 오차만 없애므로 `angleX = 0`을
정확히 유지할 뿐이고, 그 0이 기울어져 있다는 것이 문제다.

### 부수 효과 — 기존 버그가 같이 고쳐진다

`trim_roll`은 스크립트 전역변수라 `control_dualsense.py`를 재시작할 때마다 0으로
초기화된다. 매 세션 트림을 다시 잡아야 하고, 잊으면 이륙부터 흐른다. 지금은 아무
경고도 없다.

이는 `target_yaw`와 같은 안티패턴이다 — **드론이 필요로 하는 상태가 지상국에만
있다.** 같은 이유로 같은 방향의 해법을 쓴다.

### 프로토콜

```text
trim <roll_deg> <pitch_deg>     # 절대값. 누적이 아니다
```

- 펌웨어가 `trim_roll`/`trim_pitch`에 저장하고 각각 ±`TRIM_MAX_DEG`(10°)로 클램프
- 파싱 실패·비유한값은 기존 명령들과 동일하게 무시
- **무장 중에도 수락한다.** 비행 중 트림 조정은 정상 작업이다
- `start`/`stop`이 트림을 지우지 않는다. **기체 속성이지 비행별 상태가 아니다**
- 재부팅 시 소실되지만 지상국이 시동 때 재전송하므로 복구된다

### 펌웨어 목표각 합성

`setRcTargets`에서 트림을 더한 뒤 클램프한다:

```c
targetAngleX = constrain(x + trim_roll,  -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
targetAngleY = constrain(y + trim_pitch, -MAX_TARGET_ANGLE_RP, MAX_TARGET_ANGLE_RP);
```

합산 **후** 클램프해야 총합이 ±30°로 제한된다. 트림이 ±10°이므로 최악의 경우에도
스틱 권한이 20° 이상 남는다.

### 지상국

- DPAD 트림 UI는 그대로 두고, 누적값을 **절대값으로** 보낸다
- 전송 시점: 트림이 바뀔 때마다, 그리고 **시동할 때마다**
- `reliable_send`(5회 반복)를 쓴다. UDP 손실로 트림이 어긋나면 자동착륙 방향이
  틀어지므로 `start`/`stop`과 같은 급으로 취급한다
- `final_roll`/`final_pitch`에서 트림 합산을 **제거**한다. 이제 드론이 더한다

시동 시 재전송이 드론 재부팅 케이스를 덮는다 — 재부팅되면 무장이 풀려 재시동이
필요하고, 그때 트림이 다시 간다.

## 4. 상수 — 전부 벤치 조정 대상

| 상수 | 초기값 | 근거와 조정 기준 |
|---|---|---|
| `FS_DESCENT_DELTA_US` | 60 | **추정치다.** 호버 대비 스로틀 하강폭. 목표 하강률 약 0.4m/s. **벤치에서 실제 하강률을 재서 확정한다** |
| `FS_MIN_DESCEND_MS` | 1000 | 이 시간 전에는 착지 판정 금지 |
| `FS_LAND_ACCEL_TOL_G` | 0.05 | 프롭 진동 노이즈에서 1g 판정이 흔들리면 키운다 |
| `FS_LAND_CONFIRM_MS` | 400 | 짧으면 오탐, 길면 착지 후 모터가 오래 돈다 |
| `FS_MAX_MS` | **5000** | **백스톱이지 정상 종료 수단이 아니다.** 감지기가 벤치에서 검증되기 전까지 노출을 짧게 유지한다. 검증 후 3m 고도를 커버하는 10000으로 올린다 |

### 하강 스로틀이 1100 이하면 적분기가 리셋된다

898줄에 `else if (throttle <= 1100) { iTermRoll = iTermPitch = iTermYaw = 0.0f; }`가
있다. 조종자가 낮은 스로틀(예: 1150)에서 링크를 잃으면 하강 스로틀이 1090이 되어
**적분기 세 개가 모두 0으로 리셋되고 자세 트랜지언트가 생긴다.**

다만 그 스로틀이면 이미 추력이 거의 없어 사실상 낙하 중이고 지면에 가깝다고
봐야 한다. 별도 처리를 넣지 않되 벤치에서 이 조건을 재현해 관찰한다.

### `FS_DESCENT_DELTA_US`는 `CTRL_MARGIN`(150)보다 작아야 한다

`th` 명령은 `min_throttle = max(1050, nb - 150)`으로 스로틀 창을 잡는다.
하강폭이 150 이상이면 하강 스로틀이 `min_throttle` 아래로 내려가고,
`mixAndDesaturate`의 collective 하한(`minMotor - minDiff`)에 걸려 **실제 모터
출력이 더 내려가지 않는다.** 즉 하강이 먹지 않는다. 60은 이 조건을 만족하지만,
벤치에서 값을 올릴 때 이 상한을 넘기지 않아야 한다. 넘겨야 한다면
`min_throttle`도 함께 낮추는 별도 처리가 필요하다.

## 5. 다른 fault는 자동착륙 중에도 즉시 컷

`FS_DESCENDING` 상태에서 IMU 전멸이나 과도 기울기가 발생하면 즉시
`FS_CUT_ABORT`로 끝낸다. 자세 추정이 죽은 채 모터를 돌리는 것은 fly-away
그 자체다. `stop` 명령도 언제나 즉시 컷이다.

### 중단 처리를 두 곳에 넣어야 한다

IMU 전멸 경로(732~745줄)는 **RC 타임아웃 검사(808줄)에 도달하기 전에
`continue`한다.** 따라서 그 분기 안에서 직접 `fs_phase`를 `FS_CUT_ABORT`로
세워야 한다. 과도 기울기는 `safety_lock`을 세우고 812줄 블록으로 흘러가므로
그 블록에서 `fs_phase == FS_DESCENDING`이면 `FS_CUT_ABORT`로 바꾼다.

한 곳만 처리하면 IMU 전멸로 모터는 꺼지지만 `Failsafe_Phase`가 1(하강 중)로
남아 로그가 거짓말을 한다.

즉 자동착륙은 **기존 안전장치를 대체하지 않고 RC 타임아웃 한 갈래만 바꾼다.**

## 6. 텔레메트리 (35 → 38필드)

`Failsafe_Phase`(int, 36), `Trim_Roll`(float, 37), `Trim_Pitch`(float, 38)를
append한다. CSV는 39열.

| 값 | 의미 |
|---|---|
| 0 | `FS_NONE` — 정상 |
| 1 | `FS_DESCENDING` — 자동착륙 하강 중 |
| 2 | `FS_CUT_LANDED` — 착지 감지로 종료 |
| 3 | `FS_CUT_TIMEOUT` — 백스톱으로 종료 |
| 4 | `FS_CUT_ABORT` — 다른 fault로 중단 |

**어느 경로로 끝났는지 봐야 감지기를 튜닝할 수 있다.** 벤치에서 3이 계속 나오면
감지가 실패하는 것이고, `FS_LAND_ACCEL_TOL_G`나 `FS_LAND_CONFIRM_MS`를 조정해야
한다는 신호다.

`Armed`는 하강 중 1을 유지한다(실제로 무장 상태다). 종료 시 0이 된다.

### 트림을 텔레메트리로 내보내는 이유

`trim` 명령은 UDP라 **손실될 수 있다.** 손실되면 지상국 화면의 트림과 드론이
실제로 쓰는 트림이 어긋나고, 정상 비행에서는 조종자가 감으로 다시 트림을 잡아
넘어가지만 **자동착륙에서는 하강 방향이 그만큼 틀어진다.**

`gains` readback을 만든 것과 같은 이유다(Tier 1 설계: "게인 명령엔 readback이
없어 반영 확인이 불가했던 공백"). 다만 one-shot 응답 대신 텔레메트리에 실으면
항상 최신이고 새 패킷 종류를 만들지 않아도 된다. 벤치 Stage E에서 하강 목표
자세를 눈으로 확인하는 데도 필요하다.

지상국이 불일치를 감지해 자동 재전송하는 것은 비목표다. 우선은 **보이게만** 한다.

## 7. 지상국 (`control_dualsense.py`)

`telemetry_thread`의 텔레메트리 타임아웃 경로가 지금은 `disarm()`을 호출하고,
`disarm()`은 `stop`을 보낸다. 그러면 자동착륙이 아니라 **즉시 컷**이 된다.

그 경로에서 `stop` 전송을 제거하고 **rc 스트리밍만 중단**해 드론의 RC 워치독이
발동하게 한다. `stop`은 조종자의 수동 킬 전용으로 남긴다.

이 구분이 실제로 중요한 이유: 2026-07-27 블루투스 사례에서 확인했듯 **업링크만
죽고 다운링크는 살아 있는 비대칭 고장이 실재한다.** 반대 경우(다운링크만 죽음)도
가능하며, 그때 PC가 보내는 `stop`은 멀쩡히 도착해 자동착륙을 무력화한다.

### ⚠️ `Fault_RC` 핸들러도 `stop`을 보낸다 — 이쪽이 더 위험하다

205~213줄에 별도 경로가 있다:

```python
if fault_rc_drone and not prev_fault_rc:
    print("\\n[FAULT] 드론: RC 타임아웃 감지됨")
    if is_armed:
        disarm("드론 RC 타임아웃")     # ← stop 전송
```

**우리가 실제로 겪은 비대칭 고장에서 정확히 이것이 터진다.** 업링크만 죽으면
드론이 자동착륙에 들어가고 텔레메트리로 `Fault_RC=1`을 내보낸다. 다운링크는
살아 있으므로 PC가 그것을 보고 `stop`을 쏜다. 업링크가 간헐적으로 열리는
상황(블루투스 사례에서 관측된 250~450ms 버스트)이라면 **그 `stop`이 도착해
공중에서 모터가 꺼진다.**

즉 자동착륙을 가장 필요로 하는 고장 모드에서 지상국이 자동착륙을 죽인다.

이 경로에서도 `stop` 전송을 제거한다. 로컬 상태만 `is_armed = False`로 내리고
rc 스트리밍을 멈춘다. 조종자에게는 자동착륙이 진행 중임을 알린다.

`fault_critical_drone` 경로(216~222줄)는 그대로 둔다. `Fault_Critical`은
`active_imus == 0 || fault_attitude || !calibration_ok`이고 이 셋은 모두
자동착륙 대상이 아니라 즉시 컷 대상이므로, `stop`을 보내는 것이 맞다.

### 정리 — `stop`을 보내면 안 되는 두 경로

| 경로 | 현재 | 변경 후 |
|---|---|---|
| 텔레메트리 타임아웃 | `disarm()` → `stop` | rc 스트리밍만 중단 |
| `Fault_RC` 상승 엣지 | `disarm()` → `stop` | 로컬 상태만 내림 |
| **게임패드 분리** | `disarm()` → `stop` | **로컬 상태만 내림** |
| `Fault_Critical` 상승 엣지 | `disarm()` → `stop` | **그대로 유지** |
| 조종자 수동 disarm | `disarm()` → `stop` | **그대로 유지** |
| Ctrl+C | `disarm()` → `stop` | **그대로 유지** |
| 드론 `Armed=0` 감지 | `disarm()` → `stop` | **그대로 유지** |

### 게임패드 분리도 자동착륙 대상이다

패드가 뽑혀도 WiFi 링크는 멀쩡하다. 사라진 것은 **조종 입력 수단**뿐이고, 이는
`Fault_RC`와 위험 구조가 같다 — 조종자가 기체를 조종할 수 없는 상태. 자동착륙을
만든 이유가 정확히 그 상황에서 떨어뜨리지 않는 것이므로 이 경로만 예외로 둘 이유가
없다.

수동 킬 경로는 남는다. 스크립트 stdin에 `stop`을 타이핑하면 즉시 컷이다.

`disarm()`을 `stop_streaming_only()`로 바꾸면 `is_armed`가 false가 되어 rc
스트리밍이 멈추고, 드론의 RC 워치독이 500ms 뒤 자동착륙을 시작한다. 패드를 다시
꽂아도 `is_armed`는 false이므로 조종자가 재시동해야 한다 — "재시동 요구" 결정과
일관된다.

`드론 Armed=0 감지` 경로는 유지한다. 하강 중에는 `Armed`가 1이라 발동하지 않고,
착륙이 끝난 뒤에만 도는데 그때는 이미 드론이 시동 해제된 상태라 `stop`이 무해하다.

## 8. 테스트

### 순수 헬퍼 단위 테스트

yaw와 같은 패턴으로 `failsafe_land.h`에 순수 함수를 분리하고
`tools/native_tests/test_failsafe_land.cpp` + `tools/test_failsafe_land.py`로
테스트한다.

1. 진입 시 하강 스로틀이 `entry - FS_DESCENT_DELTA_US`로 계산됨
2. **진입 직후 `|accel| = 1g`여도 착지로 판정하지 않음** (오탐 회귀 테스트)
3. `FS_MIN_DESCEND_MS` 이전에는 sub-1g를 봤어도 착지 판정하지 않음
4. sub-1g → 1g 복귀 → `FS_LAND_CONFIRM_MS` 지속 시 착지 확정
5. sub-1g를 한 번도 못 본 채 1g가 계속되면 착지로 판정하지 않음
6. 1g 복귀가 `FS_LAND_CONFIRM_MS`를 못 채우고 다시 sub-1g로 가면 확정 취소
7. `FS_MAX_MS` 경과 시 착지 감지와 무관하게 종료
8. abort 입력(IMU 전멸/과도 기울기)이 다른 모든 조건보다 우선

### 통합 테스트 (`test_control_math.cpp`)

9. RC 타임아웃이 즉시 `safety_lock`이 아니라 `FS_DESCENDING`으로 진입
10. `stop` 명령은 자동착륙 중에도 즉시 컷
11. 자동착륙 종료 후 재시동 없이는 다시 날 수 없음
12. **하강 중 rc가 돌아와도 제어권이 복귀하지 않음** — `rcr`로 큰 roll 명령을
    보내도 `targetAngleX`가 0으로 유지되고 상태가 `FS_DESCENDING`에 남는지
    (§1의 "매 tick 덮어쓰기" 회귀 테스트)

### 트림 테스트 (`test_control_math.cpp`)

13. `trim 1.5 -2.0`이 `trim_roll/trim_pitch`에 저장됨
14. 트림이 ±`TRIM_MAX_DEG`(10°)로 클램프됨
15. `setRcTargets`가 **트림을 더한 뒤** ±30°로 클램프함
    (스틱 25° + 트림 10° → 30°, 35°가 아님)
16. `start`와 `stop`이 트림을 지우지 않음
17. 무장 중에도 `trim`이 수락됨
18. **자동착륙 하강 목표가 0이 아니라 트림값** — 트림이 걸린 상태에서
    failsafe에 진입하면 `targetAngleX == trim_roll`
    (§3의 흐름 방지 회귀 테스트)

### 스키마 (`tools/test_telemetry_schema.py`)

38필드 파싱, `Failsafe_Phase` 정수 타입과 `Trim_Roll`/`Trim_Pitch` 실수 타입,
35/34/31/30/22/21/14/10필드 하위호환.

### ⚠️ SIL이 이 기능을 검증하지 못한다

`test_sil_attitude.cpp`의 플랜트에는 **수직 동역학이 없다.** `PlantState`는
`phi/theta/psi`와 `p/q/r`만 가지며 고도도 수직속도도 없다. 따라서 하강도 착지도
시뮬레이션할 수 없다.

**착지 감지기는 단위 테스트와 실기 벤치로만 검증된다.** 특히 프롭 진동 속에서
`|accel|` 1g 판정이 얼마나 깨끗한지는 실측해야 한다. 그때까지 `FS_MAX_MS`
백스톱이 유일한 보장이며, 그래서 초기값을 5초로 짧게 잡는다.

이는 yaw 각속도 명령과 같은 상황이다
(`2026-07-27-yaw-rate-command-design.md` §6 참조).

## 9. 벤치 절차 (신규 Stage E)

`docs/power_on_bench_procedure.md`에 추가한다. **반드시 이 순서로 진행한다.**

1. **프롭 OFF, 상태 전이만** — `start` 후 rc 스트림을 끊는다.
   `Failsafe_Phase`가 0 → 1로 가고, 5초 뒤 3(백스톱)으로 끝나며 모터가 1000이
   되는지. 프롭이 없으므로 하강은 일어나지 않고 백스톱 경로만 확인한다.
2. **프롭 OFF, abort 경로** — 하강 중(`Phase=1`) 기체를 60° 이상 기울여
   `Phase=4`로 즉시 끝나는지.
3. **프롭 ON, 손 파지** — 하강 스로틀에서 손에 느껴지는 추력이 줄어드는지.
   **기체를 놓지 않는다.**
4. **프롭 ON, 낮은 고도(약 50cm)** — 실제로 내려와 착지하고 `Phase=2`로
   끝나는지. 여기서 `FS_DESCENT_DELTA_US`를 조정해 하강률을 확정한다.
   접지 순간 튀거나 넘어지면 감지가 늦은 것이다.
5. **고도를 단계적으로 올림** — 1m → 2m → 3m. 각 단계에서 `Phase`와 하강
   시간을 기록한다. 3m에서 5초 안에 못 내려오면 `FS_MAX_MS`를 10000으로 올린다.

각 단계에서 **종료 경로(`Failsafe_Phase` 최종값)와 소요 시간을 기록**한다.

## 10. 위험

- **조종 불가 상태로 모터가 돈다.** 이것이 이 기능의 본질적 위험이며,
  `FS_MAX_MS`가 유일한 상한이다. 초기 5초는 이 노출을 최소화하기 위한 값이다
- **위치 유지 수단이 없어 바람에 흘러간다.** 실외에서는 착지 지점을 예측할 수
  없다. 좁은 공간이나 사람 근처에서 시험하지 않는다
- **착지 감지기가 미검증이다.** 오탐하면 공중에서 모터가 꺼지고(추락),
  미탐하면 백스톱까지 모터가 돈다. 벤치 4단계 전까지 신뢰하지 않는다
- **RC 타임아웃의 의미가 바뀐다.** 지금까지 "링크 끊김 = 즉시 정지"였던 것이
  "링크 끊김 = 5초간 자동 하강"이 된다. 벤치에서 링크를 끊는 모든 기존 절차
  (Stage B-2의 RC 타임아웃 검증 포함)의 기대 동작이 달라지므로 문서를 함께
  갱신한다
- 제어경로 변경이므로 **첫 자유비행 전 벤치 전 항목 재검증 필수**

## 11. 문서 갱신 대상

- `docs/udp_protocol.md` — `trim` 명령, `Failsafe_Phase`·`Trim_Roll`·`Trim_Pitch`
  필드, RC 타임아웃 동작 변경
- `logs/README.md`, `scripts/README.md` — 38필드 / 39열
- `docs/power_on_bench_procedure.md` — Stage B-2의 RC 타임아웃 기대 동작 수정,
  Stage E 신설
- `docs/ground_station_link.md` — RC 타임아웃이 이제 자동착륙을 유발한다는 점
- `docs/project_overview.md` — 페일세이프 동작 서술

## 비목표 (YAGNI)

- 고도 센서 추가 (별도 하드웨어 작업)
- 위치 유지·복귀(RTL) — 위치 센서가 없다
- 저전압 자동착륙 — 전압 센싱이 없다
- 링크 복구 시 자동 제어권 복귀 — 재시동을 요구한다
- 하강률 폐루프 제어 — 수직속도를 측정할 수 없다
- 자동착륙 상수의 런타임 변경 명령 — 벤치에서 상수로 확정한다
- 트림의 NVS 영구 저장 — 시동 시 재전송으로 충분하다
- 지상국이 텔레메트리의 트림 불일치를 감지해 자동 재전송하는 것 — 우선은
  보이게만 하고, 실제로 어긋나는 일이 관측되면 그때 만든다
