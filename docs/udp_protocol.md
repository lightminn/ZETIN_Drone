# UDP 제어·텔레메트리 프로토콜

현행 비행 제어 후보와 지상 도구는 이 엔드포인트를 사용한다.

```text
전송: UDP
드론 주소: 192.168.4.1
포트: 4210
등록: 수신되는 모든 패킷이 지상국 엔드포인트를 식별한다.
      현행 수신기는 주기적으로 "connect"를 보낸다.
```

펌웨어는 SSID `Drone_Tuning`으로 SoftAP를 운영한다. 주소와 포트는
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)에
정의돼 있다.
보류 트랙인
[`dual_imu_flix_quat_pwm`](../firmware/flight/dual_imu_flix_quat_pwm/)은 같은
엔드포인트와 조종 명령을 쓰지만 이 문서와 완전히 일치하지는 않는다. 게인
명령의 단위가 SI(rad 기반)이고 yaw 각도 부호가 반대(CCW+)이며,
`gains`·`mag`·`magcal`을 구현하지 않고 텔레메트리도 `Armed`까지 22필드만
보낸다. [해당 README](../firmware/flight/dual_imu_flix_quat_pwm/README.md)를
참조한다.

## 지상국 → 드론

명령은 UTF-8/ASCII 텍스트 데이터그램이다. 인자는 공백으로 구분한다.

```text
start
stop
resume
rc <seq> <roll> <pitch> <yaw>
rcr <seq> <roll> <pitch> <yaw_rate>  # yaw_rate는 dps
th <microseconds>
trim <roll_deg> <pitch_deg>          # 절대 트림(도), 각 축 ±10°로 클램프
yaw <0|1>
mag <0|1>                # 자기계 yaw 드리프트 보정 ON/OFF (기본 OFF)
magcal <0|1>             # 하드아이언 캘리브레이션 시작/종료 (시동 해제 상태에서만)
magc <x> <y> <z>         # 모터 전류 간섭 보정 계수(µT/µs) 3축. "0 0 0" = 보정 off
raw <0|1>                # 1kHz dual-IMU 원시 배치 스트림 ON/OFF (기본 OFF)
gains                    # 현재 PID 게인 12개를 1회 응답

# 안쪽 각속도 PID 게인
pa|ia|da <value>      # roll+pitch 공통 P/I/D
pr|ir|dr <value>      # roll P/I/D
pp|ip|dp <value>      # pitch P/I/D
py|iy|dy <value>      # yaw P/I/D
rp|ri|rd <value>      # roll+pitch 공통 P/I/D (pa|ia|da와 동일)
yp|yi|yd <value>      # yaw P/I/D (py|iy|dy와 동일)

# 바깥 각도 P 게인
ap <value>            # roll+pitch 공통
ar|at|ay <value>      # roll / pitch / yaw
```

- `start`는 캘리브레이션 성공, 기울기 정상, 사용 가능한 IMU 존재, IMU
  일치, `magcal` 미진행 조건을 모두 통과한 뒤에만 시동하며, latch된 fault를
  해제하고 스로틀 창을 기본값(base 1100, min 1050, max 1250)으로 리셋한다.
  이때 yaw 추정(`angleZ`)을 0으로 초기화하고 mag 기준 heading도 다시 잡는다.
  이미 시동됐거나 Core 1의 arm 적용을 기다리는 상태에서 도착한 중복 `start`는
  무시되므로, 지연 도착한 재전송이 fault latch나 스로틀 창을 되돌리지 않는다.
  자동착륙 하강 중(`Failsafe_Phase=1`)인 `start`는 `safety_lock` 값과 관계없이
  거부한다. 따라서 `stop` 적용과 phase 정리 사이에도 재무장하지 않는다.
  `Failsafe_Phase=2`(착지컷), 3(타임아웃컷), 4(중단컷)에서는 정상 재시동
  경로를 계속 허용한다. `stop`은 즉시 시동을 해제한다.
- 자동착륙 하강 중에는 설정 변경 명령 `trim`, `magc`,
  `pa`, `ia`, `da`, `pr`, `ir`, `dr`, `pp`, `ip`, `dp`, `py`, `iy`, `dy`,
  `rp`, `ri`, `rd`, `yp`, `yi`, `yd`, `ap`, `ar`, `at`, `ay`를 거부하고
  수신 패킷 처리 시 거부 로그를 한 번 남긴다. 읽기 전용 `gains` 조회는
  하강 중에도 계속 허용한다.
- `resume`은 자동착륙 하강 중(`Failsafe_Phase=1`)에만 조종권 복귀를
  요청한다. RC 패킷이 최근 500ms 안에 수신됐고, 과도 기울기가 아니고, 사용
  가능한 IMU가 하나 이상이며, `Hover_Valid=1`일 때만 수락한다. 수락하면
  `Failsafe_Phase=0`, `Fault_RC=0`, `base_throttle=round(Hover_Est)`로
  복원하고 스로틀 창도 ±150µs로 다시 잡는다. 조건이 하나라도 맞지 않으면
  시리얼에 `RESUME REFUSED <phase|rc|tilt|imu|hover|cumulative>`를 남기고
  하강을 계속한다. 한 비행의 최초 자동착륙 진입부터
  `FS_RESUME_MAX_MS = 3 × FS_MAX_MS`(초기 15초)가 지나면 `cumulative`로
  거부한다. 이 시각은 앞선 `resume`이 지우지 않으며, 거부 자체는 현재
  자동착륙을 강제 종료하지 않는다. 링크가 돌아온 것만으로 자동 복귀하지
  않으며 명시적 `resume`이 반드시 필요하다.
- **`resume`은 고도를 되찾지 못한다.** `Hover_Est` 복원은 추가 하강 가속도를
  0 근처로 줄일 뿐, 이미 쌓인 하강속도는 없애지 않는다. 링크가 돌아오면 즉시
  `resume`하고, 곧바로 스로틀을 올려 고도를 회복해야 한다. 늦게 보낼수록
  잔류 하강속도와 이후 고도 손실이 커진다.
- RC 스트림이 `RC_TIMEOUT_MS`(500ms) 동안 끊기면 `Fault_RC`를 세운다.
  - 아직 `Hover_Valid=0`이면 호버한 적이 없어 공중에 있을 수 없다고 보고
    **즉시 컷**한다. `Failsafe_Phase`는 0에 머문다.
  - `Hover_Valid=1`이어도 `base_throttle ≤ FS_GROUND_CUT_MAX_US`(1150µs)일
    때만 지상 스로틀로 보고 즉시 컷한다. 지상 즉시컷 경계에는 `Hover_Est`를
    쓰지 않으므로, 등속 상승을 잘못 학습한 높은 추정치가 공중 즉시컷을 만들지
    못한다.
  - 그 외에는 **자동착륙**에 진입한다(`Failsafe_Phase=1`). 자세 목표는 적용
    중인 트림, yaw 각도는 진입 시점 heading 스냅샷, yaw 각속도 목표는 0이며,
    `Hover_Est - FS_DESCENT_DELTA_US`(초기 60µs)를 하강 스로틀로 쓴다.
    조종자가 상승 중이었어도 진입 스로틀을 기준으로 삼지 않는다. 하강 중에는
    정상 하강 collective 기준 스로틀 창(`min_throttle`/`max_throttle`)도 매
    tick 고정되므로 뒤늦게 도착한 `th`가 하강을 뒤집지 못한다.
- 호버 추정기는 무장·`FS_NONE` 상태에서 roll/pitch가 각각 ±10° 안이고,
  `|accel|`이 1g±0.05g이며, 스로틀이 1150µs를 넘는 샘플만 3초 시정수 LPF로
  추적한다. 해당 샘플 시간이 누적 1.5초가 되면 `Hover_Valid=1`이다. 이
  임계·시간 값은 모두 Stage E-0 벤치 조정 대상이다.
- 착지 감지는 **능동 스로틀 프로브**로 한다. `FS_MIN_DESCEND_MS`(1초) 뒤부터
  400ms마다 정상 하강 collective를
  `round((Hover_Est − 1000) × FS_PROBE_DIP_FRAC)`만큼 낮춰 120ms 유지한다.
  `FS_PROBE_DIP_FRAC=0.118`이며 결과는 20µs 이상,
  `CTRL_MARGIN`(150µs) 미만으로 런타임 clamp된다. 명목
  `Hover_Est=1340µs`에서는 기존과 같은 40µs이고, clamp가 걸리면 시리얼에
  `AUTO-LAND PROBE DIP CLAMPED`가 남는다. 딥 직전과 첫 30ms를 제외한 딥 중
  5Hz LPF `|accel|` 최솟값의 차분이 0.06g를 넘으면 공중 반응으로 보고 연속
  무반응 카운트를 지운다. 연속 두 번 무반응이고 LPF `|accel|`이
  1g±`FS_LAND_ACCEL_TOL_G`(0.10g) 안일 때만 `Phase=2`로 컷한다.
  1g 근처는 착지의 필요조건일 뿐 충분조건이 아니며, 주 판별은 계속 능동
  프로브의 연속 무반응이다. 딥 tick에는 collective 하한만 요청 딥만큼
  넓혀 자세 차동이 딥을 자르지 않게 한다. 그래도 표본 구간의 어느 tick에서든
  믹서 적용 collective가 요청 딥의 80% 미만이면 프로브 전체를
  `BLOCKED`(4)로 폐기하고 연속 무반응 카운트를 올리거나 지우지 않는다.
  이 경고의 시리얼 출력은 하강당 한 번으로 제한한다. 모든 프로브가 막히면
  조기 착지 확정 대신 `FS_MAX_MS = 5000`의 `Phase=3` 백스톱 컷에 맡긴다.
  적용 딥 뒤 collective가 1000µs 미만이면 유효한 프로브를 만들 수 없어
  상태를 `UNAVAILABLE`로 남기고 같은 백스톱 컷에 맡긴다.
  `FS_MAX_MS`는 착지 감지기를 벤치 검증한 뒤에만 10000으로 올린다.
- IMU 전멸·과도 기울기·명시적 `stop`은 계속 즉시 컷이다. 하강 중 이들이
  걸리면 `Phase=4`(중단컷)로 끝난다.
- `rcr`은 패킷 시퀀스와 목표 roll·pitch 각도, yaw 각속도(dps)를 담는 현행
  조종 명령이다. roll·pitch 목표는 ±30°, yaw 각속도는 펌웨어에서 ±180dps로
  제한된다. `control_dualsense.py`는 이 펌웨어 한도와 별개인
  `YAW_RATE_MAX_DPS = 90.0`으로 최대 스틱 명령을 ±90dps로 제한한다.
  시퀀스·드롭 계수·RC 워치독 상태는 `rc`와 공유한다.
- `rc`는 기존 벤치 도구 호환을 위해 그대로 유지되는 roll·pitch·yaw **각도**
  명령이다. roll·pitch 목표는 ±30°로 제한된다. yaw 필드는 벤치 지향
  `yaw 1` heading-hold 오버라이드에서만 사용하는 지원 인터페이스이며, 자동
  모드 조종에는 `rcr`을 사용한다. 시퀀스가 이전보다 작거나 같은 패킷(지연
  도착·중복)은 폐기된다.
- `th`는 기본 ESC 펄스 폭을 마이크로초 단위로 설정한다. 1000~1900으로
  제한되며, min/max 스로틀 창을 기본값 ±150 마진으로 함께 재설정한다.
- `trim`은 roll·pitch 트림의 절대값(도)을 설정한다. 각 축은 ±10°로
  클램프되며 자동착륙 하강 중이 아니면 시동 중에도 수락된다. 기체
  속성이므로 `start`와 `stop`은 지우지 않는다. roll·pitch 조종 목표에는
  트림을 더한 뒤 총합 ±30° 제한을 적용한다.
- `yaw 0`은 항상 수락되며 자동 모드로 돌아간다. 자동 모드가 기본값이다.
  스틱이 편향되면 `rcr`의 yaw 각속도를 추종하고, 스틱이 중립이면서 실제 yaw
  각속도까지 정착하면 회전이 멈춘 그 heading을 자동으로 잠근다. 실제 각속도가
  정착하지 않으면 강제로 잠그지 않고 rate 모드에 남는다.
- `yaw 1`은 heading을 고정하고 현재 heading으로 setpoint를 다시 슬레이빙하지
  않는 수동 오버라이드다. **시동 해제 상태에서만 수락**되고 무장 해제
  엣지에서 자동으로 지워지므로 비행마다 다시 켜야 하는 벤치 지향 opt-in이다.
  자동 잠금 판정 상수 `YAW_RATE_DEADZONE = 3dps`와
  `YAW_HOLD_SETTLE_DPS = 10dps`는 실기 벤치에서 조정한다.
- `mag`는 BMM350 자기계 기반 yaw 드리프트 보정을 켜거나 끈다. **기본값은
  OFF**이며, OFF일 때 yaw는 자이로 적분만으로 추정한다. 켜는 순간 현재 추정
  yaw를 기준으로 상대 heading 오프셋을 잡으므로 기수가 자북으로 돌아가지
  않는다(상대 heading 유지 전용, 자북·편각·항법 아님). 보정은 250Hz 바깥
  루프에서 시정수 약 4초로 천천히 끌어당긴다. `magcal` 진행 중에는 거부된다.
- `magcal 1`은 하드아이언 캘리브레이션을 시작한다(시동 상태에서는 거부).
  기체를 모든 방향으로 회전시킨 뒤 `magcal 0`을 보내면 축별 offset 상수
  3줄이 시리얼로 출력된다. 이 값을 펌웨어의 `MAG_HARD_IRON_OFFSET_*`에
  붙여 넣고 재빌드·재플래시해야 적용된다. 전체 절차와 통과 기준은
  [`bmm350_yaw_bench_test.md`](bmm350_yaw_bench_test.md)를 따른다.
- `magc x y z`는 모터 전류가 만드는 body-frame 자기 간섭을 보정하는 3축
  계수(µT/µs)를 런타임 설정한다. `mag.{x,y,z} -= k·(base_throttle − 1000)`을
  틸트보정 전에 적용한다. heading(°)이 아닌 raw XYZ(µT) 도메인에서 빼므로
  기수 방위에 무관하다. 벤치 특성화로 얻은 기본값이 펌웨어에 박혀 있고
  (`mag_comp_x/y/z`), `"0 0 0"`으로 보내면 보정을 끈다. 자동착륙 하강
  중에는 변경을 거부한다.
- `raw 1`은 1kHz 제어 루프에서 읽은 두 IMU의 int16 레지스터 원값을
  `ZIMU` 배치로 보내며 `raw 0`은 중지한다. 기본값은 **OFF**다. 부호,
  bias/scale, 소프트웨어 LPF, body-frame 변환은 이 스트림에 적용하지 않는다.
  `control_dualsense.py` stdin에서는 각각 `raw on`, `raw off`로 보낸다.
- `gains`는 현재 cascade PID 게인 12개를 `GAINS,<...>` 데이터그램 한 개로
  요청을 보낸 지상국에 응답한다. 값은 소수점 아래 4자리로 전송한다.
- 게인 값은 0~100 범위의 유한한 수만 수락하며, 범위를 벗어나거나 파싱에
  실패한 명령은 무시된다. 자동착륙 하강 중에는 유효한 값도 변경하지 않는다.

수신된 데이터그램은 그 출발지 주소를 텔레메트리 목적지로도 등록한다.
[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 이를 위해 주기적으로
`connect`를 보낸다.

## 드론 → 지상국

텔레메트리는 다음 58개 필드를 정확한 순서로 담은 쉼표 구분 데이터그램이다.

```text
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK,
Armed,
Motor_M1, Motor_M2, Motor_M3, Motor_M4, PID_Loop_Hz,
TgtRate_Roll, TgtRate_Pitch, TgtRate_Yaw,
MagHeading, Mag_X, Mag_Y, Mag_Z,
Yaw_Hold,
Failsafe_Phase, Trim_Roll, Trim_Pitch,
Hover_Est, Hover_Valid,
Failsafe_Probe_State, Failsafe_Probe_NoResponse,
Failsafe_Probe_Response_G,
IMU1_Gyro_X, IMU1_Gyro_Y, IMU1_Gyro_Z,
IMU1_Accel_X, IMU1_Accel_Y, IMU1_Accel_Z,
IMU2_Gyro_X, IMU2_Gyro_Y, IMU2_Gyro_Z,
IMU2_Accel_X, IMU2_Accel_Y, IMU2_Accel_Z,
TgtAngle_Roll, TgtAngle_Pitch, TgtAngle_Yaw
```

기존 21개 필드 뒤에 `Armed`(22), Tier 1 관측 필드(23~30), `MagHeading`(31),
`Mag_X`/`Mag_Y`/`Mag_Z`(32~34), `Yaw_Hold`(35), `Failsafe_Phase`(36),
`Trim_Roll`/`Trim_Pitch`(37~38), `Hover_Est`(39), `Hover_Valid`(40),
프로브 진단(41~43), IMU1 gyro/accel(44~49), IMU2 gyro/accel(50~55)을
차례로 append하고 목표 roll/pitch/yaw 각도(56~58)를 마지막에 append한다.

- `Armed`는 펌웨어 safety lock의 반전값이다. `start`가 거부되거나
  펌웨어가 스스로 시동을 해제한 것을 지상국이 이 필드로 감지한다.

| 순서 | 필드 | 타입 | 의미 |
|---|---|---|---|
| 23~26 | `Motor_M1`~`Motor_M4` | int | 실제 모터 PWM 출력(µs), 시동 해제 시 1000 |
| 27 | `PID_Loop_Hz` | int | `pid_task`의 실측 루프 주파수(Hz) |
| 28~30 | `TgtRate_Roll`, `TgtRate_Pitch`, `TgtRate_Yaw` | float | 바깥 각도 루프가 만든 목표 각속도(dps) |
| 31 | `MagHeading` | float | BMM350 틸트보정 heading(deg). throttle 간섭 보정 적용값. `mag 0`이면 갱신되지 않는다 |
| 32~34 | `Mag_X`, `Mag_Y`, `Mag_Z` | float | 하드아이언·throttle 간섭 보정 후 자기장 성분(µT). `magc` 계수 0이면 raw(=보정 전) |
| 35 | `Yaw_Hold` | int | 0=각속도 모드, 1=heading 잠금 |
| 36 | `Failsafe_Phase` | int | 0=정상, 1=하강 중, 2=착지컷, 3=백스톱컷, 4=중단컷 |
| 37~38 | `Trim_Roll`, `Trim_Pitch` | float | 드론이 적용 중인 roll·pitch 트림(도) |
| 39 | `Hover_Est` | float | 유효한 호버 후보에서 LPF로 추정한 collective 스로틀(µs) |
| 40 | `Hover_Valid` | int | 0=미확정, 1=호버 후보 시간이 유효 기준을 충족 |
| 41 | `Failsafe_Probe_State` | int | 0=WAIT, 1=DIP, 2=EVALUATE, 3=UNAVAILABLE(1000µs 아래 딥 가드), 4=BLOCKED(표본 구간 중 요청 딥의 80% 미만 전달) |
| 42 | `Failsafe_Probe_NoResponse` | int | 연속 무반응 프로브 수. 공중 반응이면 0으로 초기화 |
| 43 | `Failsafe_Probe_Response_G` | float | 최근 프로브의 `딥 직전 LPF − 딥 중 LPF 최솟값`(g) |
| 44 | `IMU1_Gyro_X` | float | IMU1 body-frame X 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 45 | `IMU1_Gyro_Y` | float | IMU1 body-frame Y 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 46 | `IMU1_Gyro_Z` | float | IMU1 body-frame Z 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 47 | `IMU1_Accel_X` | float | IMU1 body-frame X 가속도(g), 소프트웨어 LPF 없음 |
| 48 | `IMU1_Accel_Y` | float | IMU1 body-frame Y 가속도(g), 소프트웨어 LPF 없음 |
| 49 | `IMU1_Accel_Z` | float | IMU1 body-frame Z 가속도(g), 소프트웨어 LPF 없음 |
| 50 | `IMU2_Gyro_X` | float | IMU2 body-frame X 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 51 | `IMU2_Gyro_Y` | float | IMU2 body-frame Y 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 52 | `IMU2_Gyro_Z` | float | IMU2 body-frame Z 각속도(dps), gyro 소프트웨어 LPF 적용 후 |
| 53 | `IMU2_Accel_X` | float | IMU2 body-frame X 가속도(g), 소프트웨어 LPF 없음 |
| 54 | `IMU2_Accel_Y` | float | IMU2 body-frame Y 가속도(g), 소프트웨어 LPF 없음 |
| 55 | `IMU2_Accel_Z` | float | IMU2 body-frame Z 가속도(g), 소프트웨어 LPF 없음 |
| 56 | `TgtAngle_Roll` | float | 펌웨어가 사용하는 roll 목표 각도(deg) |
| 57 | `TgtAngle_Pitch` | float | 펌웨어가 사용하는 pitch 목표 각도(deg) |
| 58 | `TgtAngle_Yaw` | float | yaw outer loop의 목표 방위(deg) |

목표 각도 필드는 다음처럼 해석해야 한다.

- 일반 비행의 `TgtAngle_Roll`/`TgtAngle_Pitch`는
  `constrain(command + trim, ...)` 결과이므로 트림이 이미 합산된 setpoint다.
  `Trim_Roll`/`Trim_Pitch`를 다시 더하면 이중 계산이다.
- 자동착륙(`Failsafe_Phase != 0`)에서는 `trim_roll`, `trim_pitch`,
  `fs_hold_yaw`가 들어간다. 즉 하강 중 수평 setpoint와 유지 방위를 나타낸다.
- `TgtAngle_Yaw`는 `yawOuter.target_angle_deg`다. `Yaw_Hold=1`이면 유지 중인
  방위이고, `Yaw_Hold=0`이면 현재 heading에 슬레이빙된 값이다.

`Yaw`(3번, 융합 결과)와 `MagHeading`(31번, mag heading)의 차이를 보면
융합이 실제로 동작하는지 확인할 수 있다. `Mag_X`~`Mag_Z`는 모터 전류 간섭
특성화/검증에 쓴다(스로틀 램프 중 값이 평평하면 간섭 보정이 잘 된 것).

IMU별 필드는 융합 `Gyro_*`/`Accel_*`와 동일한 body frame이다.
`Active_IMUs == 2`이고 `Fault_Disagree == 0`이면 각 축에서 다음 항등식이
부동소수 반올림 오차 안에서 성립한다.

```text
Gyro_axis  = (IMU1_Gyro_axis  + IMU2_Gyro_axis)  / 2
Accel_axis = (IMU1_Accel_axis + IMU2_Accel_axis) / 2
```

이 필드는 1kHz 센서 루프의 원본 스트림이 아니라 20Hz 텔레메트리
스냅샷이다. Nyquist 주파수는 10Hz이므로 프롭 진동은 앨리어싱되며, 이
필드만으로 진동·순간 disagree·freeze를 판정할 수 없다. 용도는 IMU별 느린
bias/scale 드리프트와 정상상태 오프셋 관찰로 한정한다. accel은 소프트웨어
LPF가 없으므로 20Hz 로그에서도 gyro보다 시끄러운 것이 정상이다.

`gains` 명령의 one-shot 응답은 텔레메트리와 별도인 다음 형식이다.

```text
GAINS,
Kp_Angle_Roll, Kp_Angle_Pitch, Kp_Angle_Yaw,
Kp_Rate_Roll, Kp_Rate_Pitch, Kp_Rate_Yaw,
Ki_Rate_Roll, Ki_Rate_Pitch, Ki_Rate_Yaw,
Kd_Rate_Roll, Kd_Rate_Pitch, Kd_Rate_Yaw
```

위 이름은 설명을 위한 것이며 실제 패킷에는 같은 순서의 숫자 12개만 들어간다.
수신기는 `GAINS,` 접두사를 먼저 분리하므로 이 응답을 텔레메트리 불량으로
계수하지 않는다. `tune_pid.py`는 각 이름과 값을 한 줄로 출력한다.


`telemetry_schema.py`는 과거 패킷 길이도 함께 받아들인다.

- 10필드 패킷은 `Throttle`에서 끝난다.
- 14필드 패킷은 `RC_Dropped_Pkts`에서 끝난다.
- 21필드 패킷은 `Calibration_OK`에서 끝난다 (`Armed` 도입 이전 펌웨어).
- 22필드 패킷은 `Armed`에서 끝난다 (Tier 1 관측 도입 이전 펌웨어).
- 30필드 패킷은 `TgtRate_Yaw`에서 끝난다 (BMM350 mag 융합 도입 이전 펌웨어).
- 31필드 패킷은 `MagHeading`에서 끝난다 (Mag XYZ 텔레메트리 도입 이전 펌웨어).
- 34필드 패킷은 `Mag_Z`에서 끝난다 (`Yaw_Hold` 도입 이전 펌웨어).
- 35필드 패킷은 `Yaw_Hold`에서 끝난다 (자동착륙·기체 트림 도입 이전 펌웨어).
- 38필드 패킷은 `Trim_Pitch`에서 끝난다 (호버 추정 텔레메트리 도입 이전 펌웨어).
- 40필드 패킷은 `Hover_Valid`에서 끝난다 (능동 프로브 진단 도입 이전 펌웨어).
- 43필드 패킷은 `Failsafe_Probe_Response_G`에서 끝난다 (IMU별 텔레메트리 도입 이전 펌웨어).
- 55필드 패킷은 `IMU2_Accel_Z`에서 끝난다 (목표 각도 텔레메트리 도입 이전 펌웨어).
- 과거 패킷에 없는 값은 정규화된 CSV에서 빈 셀이 된다.
- `Timestamp`는 드론이 보내지 않는다. 지상 도구가 CSV의 첫 열로 추가하므로
  현행 CSV는 59개 열이 된다.

공유 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)에 있다.

### 1kHz IMU 원시 배치 (`ZIMU`)

`raw 1`일 때만 전송하는 리틀엔디언 바이너리 데이터그램이다. 1kHz 샘플을
50개씩 묶어 정상상태 패킷률을 20pkt/s로 제한한다. 소비자는 10ms마다 확인해
50개가 모이면 보내고, 마지막 `ZIMU` 전송 후 50ms가 지나면 1~49개 부분
배치를 보낸다. 한 `udp_task` 반복에서 추가 raw 데이터그램은 최대 한 개다.

헤더는 20바이트다.

| 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `magic` | `char[4]` | 4 | ASCII `ZIMU` |
| 4 | `version` | `uint8` | 1 | `1` |
| 5 | `n_samples` | `uint8` | 1 | 1~50 |
| 6 | `reserved` | `uint16` | 2 | `0` |
| 8 | `batch_seq` | `uint32` | 4 | 배치마다 단조 증가, uint32 래핑 허용 |
| 12 | `base_t_us` | `uint32` | 4 | 첫 샘플의 `micros()` |
| 16 | `dropped` | `uint32` | 4 | 생산자 링 포화로 버린 신규 샘플 누적 수 |

헤더 뒤에는 다음 26바이트 샘플이 `n_samples`개 이어진다.

| 샘플 내 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `dt_us` | `uint16` | 2 | 직전 기록 샘플과의 시간차. 배치 첫 샘플도 이전 배치 마지막 샘플 기준 |
| 2 | `imu1_gyro` | `int16[3]` | 6 | `e1.gyro[0..2]` 레지스터 원값 |
| 8 | `imu1_accel` | `int16[3]` | 6 | `e1.accel[0..2]` 레지스터 원값 |
| 14 | `imu2_gyro` | `int16[3]` | 6 | `e2.gyro[0..2]` 레지스터 원값 |
| 20 | `imu2_accel` | `int16[3]` | 6 | `e2.accel[0..2]` 레지스터 원값 |

최대 크기는 `20 + 50 × 26 = 1320B`로 1472바이트 UDP payload 한도보다
작다. 링이 가득 차면 기존 샘플을 보존하고 새 샘플만 버린다. 따라서
`dropped` 증가는 생산자/CPU/링 부족이고, `batch_seq` 불연속은 별도의 무선
손실이다. 분석 시 두 결측 원인을 따로 보고해야 한다.

### 보정상수 (`ZCAL`)

raw 스트림이 켜져 있는 동안 1초에 한 번 보내는 60바이트 리틀엔디언
데이터그램이다.

| 오프셋 | 필드 | 타입 | 크기 | 의미 |
|---:|---|---|---:|---|
| 0 | `magic` | `char[4]` | 4 | ASCII `ZCAL` |
| 4 | `version` | `uint8` | 1 | `1` |
| 5 | `reserved` | `uint8[3]` | 3 | 모두 `0` |
| 8 | `gyro_bias1` | `float[3]` | 12 | IMU1 부팅 보정 bias(dps) |
| 20 | `gyro_bias2` | `float[3]` | 12 | IMU2 부팅 보정 bias(dps), IMU1 센서축 기준 |
| 32 | `accel_scale1` | `float` | 4 | IMU1 부팅 accel 보정 배율 |
| 36 | `accel_scale2` | `float` | 4 | IMU2 부팅 accel 보정 배율 |
| 40 | `gyro_scale` | `float` | 4 | `GYRO_SCALE` |
| 44 | `accel_scale` | `float` | 4 | `ACCEL_SCALE` |
| 48 | `imu2_sign` | `float[3]` | 12 | `IMU2_SIGN_X/Y/Z` |

`ZIMU`, `ZCAL`, 현행 58필드 ASCII 텔레메트리는 모두 **같은 UDP 포트
4210, 같은 `laptopIP`/`laptopPort` 목적지**로 온다. 지상국은 UTF-8
디코드보다 먼저 첫 4바이트를 검사해 `ZIMU`/`ZCAL`을 바이너리 로그로
분기해야 한다. 그 외 데이터그램만 ASCII/`GAINS` 파서로 넘긴다.

하드웨어 LPF는 gyro 121Hz, accel 25Hz로 설정돼 있다
(`inv_imu_set_gyro_ln_bw`, `inv_imu_set_accel_ln_bw`). 따라서 1kHz로
기록해도 accel에는 25Hz 위의 진동 정보가 없다. 이 로그로 accel 프롭 진동
스펙트럼을 해석하면 안 된다. 기록 주파수를 올리는 것은 센서 ODR이나 하드웨어
LPF 대역을 바꾸지 않는다.
