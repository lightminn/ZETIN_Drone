# 요약본 근거 자료

## 구성·레이아웃 참고

- `/home/light/Downloads/2족보행로봇 심사용 발표.pptx`
- `/home/light/Downloads/ZETIN_로봇팔_10분_요약본.pptx`

두 참고본에서 상단 제목, 실제 제작 이미지 중심의 본문, 하단 결론 문장을
가져왔다. 사업 파트의 분량은 드론 발표의 기술 중심 요구에 맞춰 줄였다.

## 슬라이드별 주장과 근거

| 슬라이드 | 핵심 주장 | 저장소 근거 | 증거 수준과 한계 |
|---|---|---|---|
| 1 | 프레임·보드·제어 코드·검증 과정을 직접 연결한 프로젝트 | [`BRIEF.md`](BRIEF.md), [`../ai-startup-camp-drone/index.html`](../ai-startup-camp-drone/index.html) | 발표 범위와 해석. 비행 성능 증거가 아님 |
| 2 | 기성 비행제어기 대신 센서부터 모터까지 열어 둔 개발 목표 | [`BRIEF.md`](BRIEF.md), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [`control_dualsense.py`](../../../scripts/control_dualsense.py) | 프로젝트 목표와 구현 범위. 상용 비행제어기보다 우수하다는 주장이 아님 |
| 3 | 센서 → 추정 → 목표 → 제어 → 믹서 → 기체의 폐루프 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) | 현행 펌웨어와 SIL 구조의 요약 |
| 4 | X자 프레임·PCB·실물 보드를 반복 제작 | [`cad-top.png`](assets/cad-top.png), [`frame-iterations.jpeg`](assets/frame-iterations.jpeg), [`pcb-layout.png`](assets/pcb-layout.png), [`pcb-built.jpeg`](assets/pcb-built.jpeg) | 실제 제작 자산. 사진만으로 비행 성능을 증명하지 않음 |
| 5 | M1 전좌 CW, M2 후우 CW, M3 전우 CCW, M4 후좌 CCW와 차동 믹서 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `pinM1`~`pinM4`, `mixAndDesaturate()`; [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md) Stage A | 현행 핀·믹서 계약과 실물 회전 방향 확인 절차. 실제 장착 결과는 매 비행 전 다시 확인할 항목 |
| 6 | 가속도 신뢰도에 따라 α=0.999 / 0.9995 / 0.9998 선택 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `compute_alpha()`, `angleX`, `angleY` | 현행 펌웨어 계산. 시각화는 개념 설명용 |
| 7 | 듀얼 IMU 장착축·기체축·모터 번호·믹서 부호의 일관성 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `bodyGx`~`bodyAz`, `mixAndDesaturate()` | 현행 코드 계약. 배선·회전 방향은 실물 sign test가 필요한 항목 |
| 8 | 실제 비행 스케치를 포함한 1ms 폐루프 SIL과 Roll 믹서 부호 변이 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) `#include`, `pre_tick_hook`, `integratePlant()`, `injectImuFromPlant()`, `inject_roll_sign_fault`; [`Arduino.h`](../../../tools/native_tests/shims/Arduino.h) `pre_tick_hook` | 32비트 host SIL. 실물 센서·모터·비행 증거가 아님 |
| 9 | 250Hz 각도 바깥 루프, 1kHz 각속도 안쪽 루프, 포화 시 동시 축소 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `OUTER_DIV`, `pid_task()`, `mixAndDesaturate()` | 현행 펌웨어 구조. SIL 수렴이 자유비행 안정성을 뜻하지 않음 |
| 10 | 0.6°/s 바이어스를 30초 주입하면 자이로 단독 18.3°, 지자기 융합 2.4° | [`test_mag_yaw_fusion.cpp`](../../../tools/native_tests/test_mag_yaw_fusion.cpp) `gyro bias drift rejection`, [`test_mag_yaw_fusion.py`](../../../tools/test_mag_yaw_fusion.py) | 현행 host SIL. 실제 자기장 간섭 환경의 벤치·비행 증거가 아님 |
| 11 | RC 두절 시 조건별 즉시 컷 또는 시간 기반 하강, 단일 writer 안전 상태, WDT panic 설정 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `rcTimedOut()`, `fs_phase`, `safety_lock`, `PID_WDT_TIMEOUT_MS`; [`esp_task_wdt.h`](../../../tools/native_tests/shims/esp_task_wdt.h) | RC 분기와 상태 소유권은 host·SIL 증거. host WDT shim은 no-op이므로 실제 panic 재부팅은 보드 검증 전 |
| 12 | 실제 테더 자세 제어 영상과 별도 106.4초 Roll·Pitch 로그 | [`hover_demo.mp4`](assets/hover_demo.mp4), [`chart-attitude.png`](assets/chart-attitude.png), [`../ai-startup-camp-drone/chartdata.json`](../ai-startup-camp-drone/chartdata.json), [`flight_log_2026-07-27_041032.csv`](../../../logs/flight_log_2026-07-27_041032.csv) 4,194~6,328행 | 영상과 로그는 각각의 테더 기록이며 동일 촬영 세션으로 연결하지 않음. 자유비행 완료 증거가 아님 |
| 13 | host·SIL·테더까지 확인, 반복 호버·3901-L0X 폐루프·착지 판정은 진행 중 | [`../ai-startup-camp-drone/SOURCES.md`](../ai-startup-camp-drone/SOURCES.md), [`msp_sensor.h`](../../../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h), [`failsafe_land_research.md`](../../failsafe_land_research.md) | 현재 성숙도 경계. 경로 계획과 군집 제어는 후속 목표 |
| 14 | 하드웨어·소프트웨어·검증 기록을 함께 재사용하는 구조 | 위 2~13번 근거의 종합 | 프로젝트 의의에 대한 해석. 별도의 성능 수치 주장이 아님 |

## 증거 수준

- `host`: 노트북 단위 시험
- `SIL`: 실제 펌웨어를 포함한 소프트웨어 폐루프 시험
- `실기 테더`: 실제 센서·모터·기체에서 얻은 제한된 이동 범위의 기록
- `후속 목표`: 거리·광류 폐루프, 자유비행, 경로 계획과 군집 제어

SIL과 테더 시험 결과를 안정 자유비행 완료 증거로 확대하지 않는다.

슬라이드 12의 영상과 로그는 각각 실제 테더 기록이지만, 공통 세션 식별자가
없으므로 같은 시험이나 같은 구간이라고 연결하지 않는다.
