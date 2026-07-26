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
rc <seq> <roll> <pitch> <yaw>
rcr <seq> <roll> <pitch> <yaw_rate>  # yaw_rate는 dps
th <microseconds>
yaw <0|1>
mag <0|1>                # 자기계 yaw 드리프트 보정 ON/OFF (기본 OFF)
magcal <0|1>             # 하드아이언 캘리브레이션 시작/종료 (시동 해제 상태에서만)
magc <x> <y> <z>         # 모터 전류 간섭 보정 계수(µT/µs) 3축. "0 0 0" = 보정 off
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
  이미 시동된 상태에서 도착한 중복 `start`는 무시되므로, 지연 도착한 재전송이
  비행 중 fault latch나 스로틀 창을 되돌리지 않는다. `stop`은 즉시 시동을
  해제한다.
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
  (`mag_comp_x/y/z`), `"0 0 0"`으로 보내면 보정을 끈다.
- `gains`는 현재 cascade PID 게인 12개를 `GAINS,<...>` 데이터그램 한 개로
  요청을 보낸 지상국에 응답한다. 값은 소수점 아래 4자리로 전송한다.
- 게인 값은 0~100 범위의 유한한 수만 수락하며, 범위를 벗어나거나 파싱에
  실패한 명령은 무시된다.

수신된 데이터그램은 그 출발지 주소를 텔레메트리 목적지로도 등록한다.
[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 이를 위해 주기적으로
`connect`를 보낸다.

## 드론 → 지상국

텔레메트리는 다음 35개 필드를 정확한 순서로 담은 쉼표 구분 데이터그램이다.

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
Yaw_Hold
```

기존 21개 필드 뒤에 `Armed`(22), Tier 1 관측 필드(23~30), `MagHeading`(31),
`Mag_X`/`Mag_Y`/`Mag_Z`(32~34), `Yaw_Hold`(35)를 차례로 append한다.

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

`Yaw`(3번, 융합 결과)와 `MagHeading`(31번, mag heading)의 차이를 보면
융합이 실제로 동작하는지 확인할 수 있다. `Mag_X`~`Mag_Z`는 모터 전류 간섭
특성화/검증에 쓴다(스로틀 램프 중 값이 평평하면 간섭 보정이 잘 된 것).

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
- 과거 패킷에 없는 값은 정규화된 CSV에서 빈 셀이 된다.
- `Timestamp`는 드론이 보내지 않는다. 지상 도구가 CSV의 첫 열로 추가하므로
  현행 CSV는 36개 열이 된다.

공유 구현은
[`telemetry_schema.py`](../scripts/telemetry_schema.py)에 있다.
