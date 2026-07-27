# ZETIN Drone 프로젝트 개요

이 문서는 현재 지원하는 경로와 역사 보관 경로를 구분해 처음 보는 사람이
올바른 파일에서 시작하도록 안내한다.

## 지원 범위와 성숙도

현행 하드웨어·제어 조합은 **ESP32-S3 + 듀얼 ICM42670 + PWM ESC**다.

| 기능 | 상태 | 시작점 |
|---|---|---|
| 4채널 ESC PWM | 벤치 검증됨 | [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/) |
| 단일·듀얼 ICM42670 raw 읽기 | 벤치 검증됨 | [`firmware/diagnostics/`](../firmware/diagnostics/) |
| 듀얼 IMU 캐스케이드 제어 | 실험 중인 비행 후보 | [`dual_imu_cascade_pwm`](../firmware/flight/dual_imu_cascade_pwm/) |
| 안정 자세제어·호버링 | 검증 완료 아님 | 제한된 테스트 리그에서만 평가 |

모터 PWM과 raw IMU가 동작한다는 사실은 안정 비행을 보장하지 않는다.
폐쇄루프 자세제어는 계속 검증해야 하며, 보관 코드의 과거 성공 기록을 현행
지원 근거로 사용하지 않는다.

## 저장소 구조

```text
firmware/
  flight/                 current flight-controller candidate
  diagnostics/            current ESP32-S3/ICM42670/PWM bench sketches
  archive/                unsupported historical firmware
scripts/
  archive/                deprecated GPS/TCP PC tools
docs/                     maintained, historical, and archived documents
logs/                     generated flight CSV logs
tools/                    repository and telemetry checks
```

초기 PlatformIO `src/`, `lib/`, `include/` 골격은 실제 센서 대신 시뮬레이션
값을 쓰고 옛 TCP 전송을 포함한다. 현재 코드로 사용하지 말고
[`firmware/archive/platformio_skeleton/README.md`](../firmware/archive/platformio_skeleton/README.md)에서
역사 자료로만 확인한다.

## 현행 비행 제어 후보

대표 소스는
[`dual_imu_cascade_pwm.ino`](../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)다.
ESP32-S3의 FreeRTOS 태스크에서 듀얼 IMU를 읽고, 자세 바깥 루프와 각속도
안쪽 루프를 거쳐 4개 ESC PWM을 계산한다. 통신은 ESP32 SoftAP와 UDP
4210을 쓴다. 자세한 문자열 명령과 필드 순서는
[`udp_protocol.md`](udp_protocol.md)를 따른다.

yaw 스틱은 절대 각도가 아니라 `rcr` 각속도 명령이다. 스틱이 중립이고 실제
yaw 각속도까지 정착하면 회전이 멈춘 heading을 자동으로 잠그며, 정착하지
않으면 강제 잠금 없이 rate 모드에 남는다. 옵션인 BMM350 자기계 융합(`mag 1`,
기본 OFF)을 켜면 그 잠금을 자이로 적분 드리프트 없이 유지한다. 모터 전류
간섭은 raw XYZ 도메인 throttle 보정으로 상쇄한다
(`docs/bmm350_yaw_bench_test.md`, branch `feat/bmm350-yaw-fusion`). SIL S5/S6b는
플랜트 yaw 토크 부호가 미해결이므로 폐루프 yaw를 검증하지 못하며, 자동 잠금과
스틱 해제 후 지속 회전 제거는 전원 인가 벤치에서 확인해야 한다. PID 태스크가
SPI 행업 등으로 정지하면 태스크 워치독(500ms)이 재부팅을 강제해 마지막
PWM으로 모터가 고정되는 것을 막는다.

RC 입력이 500ms 끊기면 즉시 모터를 끄지 않고, 펌웨어에 저장된 roll·pitch
트림을 유지하며 자동착륙한다. 초기 하강값 `FS_DESCENT_DELTA_US=60`은
`CTRL_MARGIN=150`보다 작게 유지하며, 백스톱 `FS_MAX_MS`는 처음 5000ms다.
착지 감지기를 벤치 검증한 뒤에만 10000ms로 올린다. RC 타임아웃만 이 경로를
사용하고, IMU 전멸·과도 기울기·`stop`은 즉시 컷한다.

### 핀 배치

```text
SPI: SCK=12, MISO=13, MOSI=11
IMU1 CS=10, IMU2 CS=9
Motor PWM: M1=4 (FL), M2=5 (RR), M3=6 (FR), M4=7 (RL)
```

단일 IMU raw 진단만 별도 배치(SCK/MISO/MOSI/CS = 18/19/23/5)를 쓴다.
각 진단별 핀은 [`firmware/README.md`](../firmware/README.md)에 정리돼 있다.

### 센서축과 기체축

IMU2는 IMU1 기준 X와 Z 부호가 반대이고 Y는 같다. 두 센서를 평균하기
전에 IMU2에 X=-1, Y=+1, Z=-1을 적용한다. 융합 센서축에서 기체축으로의
현재 변환은 다음과 같다.

```text
body roll rate X  = -sensor Y
body pitch rate Y =  sensor X
body yaw rate Z   = -sensor Z
body accel X      =  sensor Y
body accel Y      = -sensor X
body accel Z      =  sensor Z
```

### 모터 믹서

M1/M2/M3/M4는 FL/RR/FR/RL 순서다. `T`를 collective throttle,
`R`, `P`, `Y`를 roll/pitch/yaw 보정량이라 하면 기본 차동 부호는 다음과
같다. 실제 코드는 허용 범위를 넘으면 모든 자세 명령을 같은 비율로 줄여
토크 비율을 보존한다.

```text
M1 = T - P + R - Y
M2 = T + P - R - Y
M3 = T - P - R + Y
M4 = T + P + R + Y
```

## PC 도구

| Tool | Role |
|---|---|
| [`control_dualsense.py`](../scripts/control_dualsense.py) | DualSense command sender |
| [`tune_pid.py`](../scripts/tune_pid.py) | Manual UDP command and gain tuning |
| [`receive_telemetry.py`](../scripts/receive_telemetry.py) | Terminal receiver and CSV logger |
| [`monitor_telemetry.py`](../scripts/monitor_telemetry.py) | Live plots and CSV logger |
| [`analyze_flight_log.py`](../scripts/analyze_flight_log.py) | Offline CSV analysis |
| [`receive_dual_imu_debug.py`](../scripts/receive_dual_imu_debug.py) | Paired loop diagnostic receiver |

현행 펌웨어는 UDP 패킷마다 40개 텔레메트리 필드를 보낸다. 마지막 두 필드는
`Hover_Est`(39), `Hover_Valid`(40)이며, PC 수신기는 맨 앞에 수신
`Timestamp`를 추가하므로 CSV는 41개 열이다. 공유 파서는 오래된
10·14·21·22·30·31·34·35·38필드 패킷도 받아들이며, 없는 값은 빈 CSV
셀로 남긴다.

## 빌드와 실행

저장소 루트에서 현행 비행 후보를 다음처럼 빌드한다. Arduino 빌드 파일은
저장소가 아닌 `/tmp`에 둔다.

```bash
arduino-cli compile --warnings all \
  --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/zetin-flight-build \
  firmware/flight/dual_imu_cascade_pwm
```

PC를 펌웨어의 `Drone_Tuning` SoftAP에 연결한 뒤 필요한 도구를 실행한다.
기본 드론 주소는 `192.168.4.1`, 포트는 4210이다. 실행 명령은
[`scripts/README.md`](../scripts/README.md)에 있다.

## 보관된 기능

다음 항목은 모두 deprecated이며 현재 빌드 대상으로 고치지 않는다.

- 옛 단일·듀얼 PID, 단일 IMU 캐스케이드, Kalman 비행 변형:
  [`firmware/archive/legacy_flight/`](../firmware/archive/legacy_flight/),
  [`firmware/archive/filter_experiments/`](../firmware/archive/filter_experiments/)
- DShot와 MPU6500 실험:
  [`firmware/archive/dshot/`](../firmware/archive/dshot/)
- BMM350, GPS, US100:
  [`firmware/archive/other_sensors/`](../firmware/archive/other_sensors/)
- STM32 F411 UART 실험:
  [`firmware/archive/other_mcus/`](../firmware/archive/other_mcus/)
- 옛 GPS 수신기와 TCP 테스트:
  [`scripts/archive/`](../scripts/archive/)

23개 스케치의 상태는 [`firmware_catalog.md`](firmware_catalog.md), 모든 옛
경로와 새 경로의 대응은 [`migration_map.md`](migration_map.md)에서 찾는다.

## 안전한 확인 순서

1. 프로펠러를 제거한다.
2. [`motor_pwm_bench`](../firmware/diagnostics/motor_pwm_bench/)로 모터 번호와
   PWM 반응을 확인한다.
3. [`icm42670_single_raw`](../firmware/diagnostics/icm42670_single_raw/)과
   [`icm42670_dual_raw`](../firmware/diagnostics/icm42670_dual_raw/)로 센서축과
   두 IMU 부호를 확인한다.
4. [`icm42670_dual_loop_debug`](../firmware/diagnostics/icm42670_dual_loop_debug/)로
   추정각·루프 주기·보정 방향을 확인한다.
5. 전원 극성, 비상 정지, RC timeout 자동착륙, 과도 기울기 정지를 확인한
   뒤에만 제한된 테스트 리그에서 비행 후보를 평가한다.
