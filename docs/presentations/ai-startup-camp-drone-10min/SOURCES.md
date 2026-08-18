# 개발 결과 발표자료 근거

## 구성·레이아웃 참고

- `/home/light/Downloads/2족보행로봇 심사용 발표.pptx`
- `/home/light/Downloads/ZETIN_로봇팔_10분_요약본.pptx`

두 참고본에서 상단 제목, 실제 제작 이미지 중심의 본문, 하단 결론 문장을
가져왔다. 사업 파트의 분량은 드론 발표의 기술 중심 요구에 맞춰 줄였다.

## 슬라이드별 주장과 근거

| 슬라이드 | 핵심 주장 | 저장소 근거 | 증거 수준과 한계 |
|---|---|---|---|
| 1 | 기체·제어 보드·펌웨어·검증 환경을 연결한 개발 결과 | [`BRIEF.md`](BRIEF.md), [`../ai-startup-camp-drone/index.html`](../ai-startup-camp-drone/index.html) | 결과발표의 범위. 자유비행 완료 주장이 아님 |
| 2 | 상용 제품은 현장 운용, 자작 시스템은 측정·오류 주입·디버깅 학습에 적합 | [`BRIEF.md`](BRIEF.md), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) | 목적에 따른 선택. 상용 비행제어기보다 우수하다는 주장이 아님 |
| 3 | 센서 → 추정 → 목표 → 제어 → 믹서 → 기체의 폐루프 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) | 현행 펌웨어와 SIL 구조의 요약 |
| 4 | CAD → 출력 → 조립 → 수정의 래피드 프로토타이핑, 프린터 점유·직접 작업 계획과 사용자 확인 프로젝트 추정 | [`cad-top.png`](assets/cad-top.png), [`frame-iterations.jpeg`](assets/frame-iterations.jpeg), [`pcb-built.jpeg`](assets/pcb-built.jpeg), 사용자 확인 프로젝트 추정(2026-08-19) | 사진은 실제 제작 자산. 2~3일·10~20일은 프린터 한 대 기준 계획 경과이며 납기 약속이 아님. 비용은 판매처 가격 증빙이 아니며 완성기 총원가나 검증된 BOM도 아님 |
| 5 | M1 전좌 CW, M2 후우 CW, M3 전우 CCW, M4 후좌 CCW와 차동 믹서 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `pinM1`~`pinM4`, `mixAndDesaturate()`; [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md) Stage A | 현행 핀·믹서 계약과 실물 회전 방향 확인 절차. 실제 장착 결과는 매 비행 전 다시 확인할 항목 |
| 6 | 가속도 신뢰도에 따라 α=0.999 / 0.9995 / 0.9998 선택 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `compute_alpha()`, `angleX`, `angleY` | 현행 펌웨어 계산. 시각화는 개념 설명용 |
| 7 | Roll 믹서 보정 방향 오류를 증상 → 로그·축 비교 → 원인 → SIL 재현 → 수정 검증으로 추적 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) `SIL_INJECT_SIGN_FAULT`, `inject_roll_sign_fault`, `integratePlant()` 488~501행 | 변이는 자이로 부호가 아니라 Roll 믹서 보정 R 전체를 R→−R로 뒤집음. host SIL이며 실기 결함 주입 증거가 아님 |
| 8 | 실제 비행 스케치를 포함한 1ms 폐루프 SIL과 Roll 믹서 부호 변이 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) `#include`, `pre_tick_hook`, `integratePlant()`, `injectImuFromPlant()`, `inject_roll_sign_fault`; [`Arduino.h`](../../../tools/native_tests/shims/Arduino.h) `pre_tick_hook` | 32비트 host SIL. 실물 센서·모터·비행 증거가 아님 |
| 9 | 안쪽 각속도 루프가 바깥 자세 루프보다 빠르게 반응하고 믹서가 네 모터로 배분 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `pid_task()`, `OUTER_DIV`, `mixAndDesaturate()` | 화면은 상대 시간척도만 설명. 현행 코드는 1ms 안쪽 루프와 `OUTER_DIV=4`이며 이 구현값이 보편 정답이라는 주장이 아님 |
| 10 | 0.6°/s 바이어스를 30초 주입하면 자이로 단독 18.3°, 지자기 융합 2.4° | [`test_mag_yaw_fusion.cpp`](../../../tools/native_tests/test_mag_yaw_fusion.cpp) `gyro bias drift rejection`, [`test_mag_yaw_fusion.py`](../../../tools/test_mag_yaw_fusion.py) | 현행 host SIL. 실제 자기장 간섭 환경의 벤치·비행 증거가 아님 |
| 11 | 조종 신호 두절 뒤 호버 추정이 없거나 저스로틀이면 즉시 컷, 호버 추정이 유효하고 저스로틀을 초과하면 제한 하강으로 전환 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `rcTimedOut()`, `hover_valid`, `FS_GROUND_CUT_MAX_US`, `fs_phase`, `safety_lock`, `PID_WDT_TIMEOUT_MS`; [`esp_task_wdt.h`](../../../tools/native_tests/shims/esp_task_wdt.h) | 직접적인 지상·공중 판별이 아님. RC 분기와 상태 소유권은 host·SIL 증거. host WDT shim은 no-op이므로 실제 panic 재부팅은 보드 검증 전. 착지 판단과 자유비행 안전성은 실기 검증 전 |
| 12 | 실제 테더 자세 제어 영상과 별도 106.4초 Roll·Pitch 로그 | [`hover_demo.mp4`](assets/hover_demo.mp4), [`chart-attitude.png`](assets/chart-attitude.png), [`../ai-startup-camp-drone/chartdata.json`](../ai-startup-camp-drone/chartdata.json), [`flight_log_2026-07-27_041032.csv`](../../../logs/flight_log_2026-07-27_041032.csv) 4,194~6,328행 | 영상과 로그는 각각의 테더 기록이며 동일 촬영 세션으로 연결하지 않음. 자유비행 완료 증거가 아님 |
| 13 | 단기 단일 기체 → 중기 반복 제작·sim-to-real → 장기 군집 안전 제어의 순차 목표 | [`../ai-startup-camp-drone/SOURCES.md`](../ai-startup-camp-drone/SOURCES.md), [`msp_sensor.h`](../../../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h), [`failsafe_land_research.md`](../../failsafe_land_research.md) | 단기는 진행 중 검증, 중·장기는 구현 전 계획. 경로 계획과 군집 제어를 현재 성과로 제시하지 않음 |
| 14 | 하드웨어·소프트웨어·재현 가능한 검증·디버깅 과정을 함께 남긴 결과 | 위 2~13번 근거의 종합 | 기술 결과의 종합과 학습 활용에 대한 해석. 별도의 비행 성능 수치 주장이 아님 |

## 제작 규모 산정 해석

- 1대: 프린터 점유 24~48시간, 직접 작업 6~10인시, 프린터 한 대에서 약 2~3일
- 10대: 프린터 점유 240~480시간, 직접 작업 60~100인시, 출력과 조립 병행 시
  프린터 한 대의 최소 점유 약 10~20일
- 가정: 부품 재고, 설계·펌웨어 준비, 조립 완료 FC PCB, 프린터 한 대,
  기체당 출력 질량 0.4~0.8kg, 유효 출력량 16.7g/h, 첫 출력 성공
- 사용자 확인 프로젝트 추정 합계: 1대 221,900~233,100원, 10대 2,219,000~2,331,000원
- 1대 추정 품목: 모터 4개 8.0만 원, ESC 4개 4.0만 원, MCU·센서 2.3만 원,
  프레임 1.1~2.2만 원, 배터리·프로펠러 6.8만 원
- 가격 미포함: 자체 FC PCB와 전원·보호 회로, 배선·커넥터·XT60·수축튜브,
  나사·인서트·스탠드오프, 배송·관부가세·환전 수수료, 인건비·장비 감가,
  실패 출력·재작업, 자유비행 튜닝과 반복 비행 검증

위 시간은 실측이나 슬라이서 출력이 아닌 처리량 계획 가정이다. 가격 합계도 판매처
가격 증빙이 아닌 사용자 확인 프로젝트 추정이며, 실제 구매 SKU와 자체 PCB BOM이
확정된 완성기 총원가가 아니다.

## 증거 수준

- `host`: 노트북 단위 시험
- `SIL`: 실제 펌웨어를 포함한 소프트웨어 폐루프 시험
- `실기 테더`: 실제 센서·모터·기체에서 얻은 제한된 이동 범위의 기록
- `후속 목표`: 거리·광류 폐루프, 자유비행, 경로 계획과 군집 제어

SIL과 테더 시험 결과를 안정 자유비행 완료 증거로 확대하지 않는다.

슬라이드 12의 영상과 로그는 각각 실제 테더 기록이지만, 공통 세션 식별자가
없으므로 같은 시험이나 같은 구간이라고 연결하지 않는다.
