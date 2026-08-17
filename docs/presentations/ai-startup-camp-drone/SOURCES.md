# 발표자료 주장과 근거

이 문서는 발표자료의 수치와 현재 상태를 저장소 근거로 추적하기 위한 목록이다.
펌웨어 상수·프로토콜 순서의 정답은 항상 현재 코드와 프로토콜 문서다. 날짜가
붙은 실험 기록은 그 시점의 증거이며, 현재 구현 상태와 구분해서 읽는다.

| 슬라이드 | 주장 | 저장소 근거 | 증거 시점·경계 |
|---|---|---|---|
| 1, 10~12, 22, 76 | 기술 중심 발표 방향, 교육 아이템 맥락, PCB 주문·납땜·부품 납기 상태, 다중 드론 확장 목표 | [`BRIEF.md`](BRIEF.md) | 2026-08-10 사용자 제공 발표 준비 정리. 발표 방향과 실행 현황이며 비행 검증 증거가 아님 |
| 1, 6~8, 10, 11, 37, 68 | 실제 기체·보드·CAD·코드·모바일 체험 시각자료 | [`assets/`](assets/), [`mobile-lab/`](mobile-lab/) | 직접 제작·구현한 산출물과 벤치 장면. 사진 자체가 비행 성능 검증을 뜻하지 않음 |
| 14, 26, 71 | 상태 텔레메트리 65개 필드, 별도 1kHz 원시 IMU | [`udp_protocol.md`](../../udp_protocol.md), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 현행 프로토콜. CSV는 수신 시각까지 66열 |
| 30~32, 46 | Roll·Pitch 적응형 α = 0.999 / 0.9995 / 0.9998 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `ALPHA_STATIC`, `ALPHA_NORMAL`, `ALPHA_DYN`, `compute_alpha` | 현행 펌웨어 |
| 38~41 | 실제 비행 스케치 직접 포함, 1ms tick hook, 독립 물리·듀얼 IMU 합성, S1 정상 수렴과 부호 반전 실패 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp), [`Arduino.h`](../../../tools/native_tests/shims/Arduino.h), [`test_sil_attitude.py`](../../../tools/test_sil_attitude.py) | 현행 32비트 host SIL. 정상 S1은 Roll/Pitch 마지막 500 tick 평균 0.0513°/0.1179°, 부호 반전은 최대 Roll 1,291.8°에서 실패. 실비행 증거가 아님 |
| 46, 48, 49 | Yaw 보정 250Hz, K=0.001, 시정수 약 4초 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `K_MAG`, `magFusionCnt` | 현행 펌웨어. 1kHz α 값은 설명용 등가식일 뿐 구현값이 아님 |
| 35 | 기체축 자이로 +Y/−X/−Z, 가속도 +Y/−X/+Z | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `bodyGx`~`bodyAz` | 현행 펌웨어 변환식 |
| 47, 51 | 정지 기체에 0.6°/s 자이로 바이어스를 30초 주입했을 때 자이로 단독 오차 18.3091°, 지자기 융합 오차 2.4394° | [`test_mag_yaw_fusion.cpp`](../../../tools/native_tests/test_mag_yaw_fusion.cpp) `gyro bias drift rejection`, [`test_mag_yaw_fusion.py`](../../../tools/test_mag_yaw_fusion.py) | 현행 32비트 host SIL. 실제 센서 벤치나 비행 증거가 아님 |
| 50, 51 | 모터 전류 간섭 보정 전후 헤딩 기울기 +3.64° → +0.02°/100µs | [`bmm350_yaw_bench_test.md`](../../bmm350_yaw_bench_test.md), [`chartdata.json`](chartdata.json) | 2026-07-27 벤치 로그 결과. 당시 min/max 보정 기반이며 현행 타원체 보정과 구분 |
| 50~52 | 제약 타원체 hard/soft-iron 보정과 보드·상하행 램프 재검증 | [`bmm350_yaw_bench_test.md`](../../bmm350_yaw_bench_test.md), [`magcal_fit.py`](../../../scripts/magcal_fit.py), [`control_dualsense.py`](../../../scripts/control_dualsense.py), [캘리브레이션 설계](../../superpowers/specs/2026-08-04-magnetometer-calibration-design.md) | 2026-08-04 실측. 정상 지상국 운용은 Mag ON, 펌웨어 부팅값은 OFF |
| 56~57, 61~63 | 프로브는 기록 전용, `landed=false`, 3초 시간 기반 하강 | [`failsafe_land_research.md`](../../failsafe_land_research.md), [`failsafe_land.h`](../../../firmware/flight/dual_imu_cascade_pwm/failsafe_land.h), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 2026-08-01 정정 이후 현행 상태. 공중 프로브 분포 미측정 |
| 58 | PMW3901 광류와 VL53L0X ToF 거리의 결합 원리, 8cm~2m 작동 범위, 940nm 광원 | [MATEKSYS 제품 자료](https://www.mateksys.com/?portfolio=3901-l0x), [ST VL53L0X 자료](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) | 제조사 원리·사양. 우리 기체의 실측 성능을 뜻하지 않음 |
| 59 | 하부 장착·렌즈 하향·화살표 전방, 5V/GND/TX 배선과 GPIO16 수신 | [ArduPilot 장착 안내](https://ardupilot.org/plane/docs/common-mateksys-optflow-3901L0X.html), [MATEKSYS 제품 자료](https://www.mateksys.com/?portfolio=3901-l0x), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 기구·배선 계획과 현행 핀 정의. 최종 브래킷·오프셋 실측은 HW 확인 전 |
| 57~60, 63, 75, 76 | 3901-L0X의 UART 115200bps·MSPv2 수신, 500ms 신선도, 필드 60~64 기록과 향후 고도·위치 제어 계획 | [`udp_protocol.md`](../../udp_protocol.md), [`msp_sensor.h`](../../../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [ArduPilot 활용 안내](https://ardupilot.org/plane/docs/common-mateksys-optflow-3901L0X.html) | 수신·파싱·신선도 표시는 구현됨. 착지·고도·위치 폐루프와 실기 효과는 아직 검증하지 않음 |
| 70, 71 | 테더 구간 106.4초, Roll σ=1.65°, Pitch σ=1.23°, M1 1334.2µs, M3 1360.0µs | [`chartdata.json`](chartdata.json), [`flight_log_2026-07-27_041032.csv`](../../../logs/flight_log_2026-07-27_041032.csv) 4,194~6,328행 | 2026-07-27 테더 세션의 2,135행. 안정 비행 완료 증거가 아님 |
| 69, 77 | 실제 제작 기체의 자세 제어 구동 영상 | [`hover_demo.mp4`](assets/hover_demo.mp4) | 테더를 건 실기 시험 기록. 반복 가능한 안정 호버나 자유 비행 완료를 뜻하지 않음 |
| 75 | 첫 실비행 176.2초, 고장 0건, PID 평균 1000Hz·최소 999Hz, Mag ON, Yaw hold 91.9% | [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md), [`flight_log_2026-08-01_003319.csv`](../../../logs/flight_log_2026-08-01_003319.csv) | 2026-08-01 실비행 관측. 안정 호버·자동착륙 완료를 뜻하지 않음 |

## 성숙도 경계

- 코드·호스트 테스트로 확인: 센서 수집, 축 변환, 필터·제어 계산, 텔레메트리,
  프로브 기록 경로, 3901-L0X 프레임 파싱.
- 날짜가 붙은 벤치·비행으로 확인: 위 표의 7월 27일, 8월 1일, 8월 4일 기록.
- 아직 확인하지 않음: 반복 가능한 안정 호버, 신뢰할 수 있는 실내 저고도
  자동착륙, 거리·광류 폐루프 제어, 실외 자유 비행, 경로·군집 제어.

발표자료에서 `검증`이라고 쓸 때는 위 세 범주 중 무엇인지 함께 밝힌다.
