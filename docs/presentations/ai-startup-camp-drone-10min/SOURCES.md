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
| 3 | 실제 조립 기체에서 직접 설계한 모듈형 프레임·비행제어 PCB·배선·검증 환경과 조달해 통합한 모터·ESC·배터리·센서를 구분하고 실제 전원 경로를 제시 | [`assembled-bench.jpeg`](assets/assembled-bench.jpeg), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [`../ai-startup-camp-drone/SOURCES.md`](../ai-startup-camp-drone/SOURCES.md) | 사진으로 실제 조립·통합 범위를 확인. 정확한 구매 SKU·영수증·완성 BOM을 주장하지 않으며 보호용 MOSFET이나 별도 FC 5V 레귤레이터를 전제로 하지 않음 |
| 4 | 설계 → 출력 → 시험·파손 → 측정 → 개선의 래피드 프로토타이핑과 손상된 모듈형 암의 재출력 교체 | [`cad-top.png`](assets/cad-top.png), [`frame-iterations.jpeg`](assets/frame-iterations.jpeg), [`modular-arm.png`](assets/modular-arm.png), 사용자 제공 제작 경험 | 사진과 CAD는 실제 제작 자산. 2~3일·10~20일은 프린터 한 대 기준 계획 경과이며 납기 약속이 아님. 기본 합계 22.2~23.3만 원에 선택 센서 3901-L0X 4.8~4.9만 원/대는 포함하지 않음 |
| 5 | M1 전좌 CW, M2 후우 CW, M3 전우 CCW, M4 후좌 CCW와 차동 믹서 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `pinM1`~`pinM4`, `mixAndDesaturate()`; [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md) Stage A | 현행 핀·믹서 계약과 실물 회전 방향 확인 절차. 실제 장착 결과는 매 비행 전 다시 확인할 항목 |
| 6 | 가속도 신뢰도에 따라 α=0.999 / 0.9995 / 0.9998 선택 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `compute_alpha()`, `angleX`, `angleY` | 현행 펌웨어 계산. 시각화는 개념 설명용 |
| 7 | 실제 센서축→기체축 부호 문제의 디버깅과 별도 Roll 믹서 변이의 검출력 확인 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `bodyGx`~`bodyAz`, [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) `SIL_INJECT_SIGN_FAULT`, `inject_roll_sign_fault`, `integratePlant()` 488~501행 | 실제 문제는 센서축·기체축 부호 약속 불일치이며, host SIL 변이는 Roll 보정 R 전체를 R→−R로 뒤집은 별도 오류 유형. 같은 사건을 재현했다는 주장이 아님 |
| 8 | 실제 비행 스케치를 포함한 1ms 폐루프 SIL과 Roll 믹서 부호 변이 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp) `#include`, `pre_tick_hook`, `integratePlant()`, `injectImuFromPlant()`, `inject_roll_sign_fault`; [`Arduino.h`](../../../tools/native_tests/shims/Arduino.h) `pre_tick_hook` | 32비트 host SIL. 실물 센서·모터·비행 증거가 아님 |
| 9 | 안쪽 각속도 루프가 바깥 자세 루프보다 빠르게 반응하고 믹서가 네 모터로 배분 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `pid_task()`, `OUTER_DIV`, `mixAndDesaturate()` | 화면은 상대 시간척도만 설명. 현행 코드는 1ms 안쪽 루프와 `OUTER_DIV=4`이며 이 구현값이 보편 정답이라는 주장이 아님 |
| 10 | 0.6°/s 바이어스를 30초 주입하면 자이로 단독 18.3°, 지자기 융합 2.4° | [`test_mag_yaw_fusion.cpp`](../../../tools/native_tests/test_mag_yaw_fusion.cpp) `gyro bias drift rejection`, [`test_mag_yaw_fusion.py`](../../../tools/test_mag_yaw_fusion.py) | 현행 host SIL. 실제 자기장 간섭 환경의 벤치·비행 증거가 아님 |
| 11 | 조종 신호 두절 전환과 착지 프로브의 판정 제외 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `rcTimedOut()`, `hover_valid`, `FS_GROUND_CUT_MAX_US`, `fs_phase`, `safety_lock`; [`landing-probe-evidence.png`](assets/landing-probe-evidence.png); [`failsafe_land_research.md`](../../failsafe_land_research.md) | 공중 2회 0.061g·0.097g, 접지 10회 0.059~1.147g으로 범위가 겹치고 접지 반응이 남음. 현재는 로깅만 하고 `landed=false`를 유지하며 착지 판정에서 제외 |
| 12 | 실제 테더 자세 제어 영상과 별도 로그 2,135행의 전체 구간 모터 평균 | [`hover_demo.mp4`](assets/hover_demo.mp4), [`telemetry-motor-balance.png`](assets/telemetry-motor-balance.png), [`flight_log_2026-07-27_041032.csv`](../../../logs/flight_log_2026-07-27_041032.csv) | M3 1360.0µs, M1 1334.2µs로 차이는 약 25.8µs. M3 근처 테더 줄의 하중 영향으로 해석하지만 모터·프로펠러·프레임·공력 차이를 분리한 시험이 아니므로 확정 원인이 아님. 영상과 로그도 동일 촬영 구간으로 연결하지 않음 |
| 13 | 단기 단일 기체·3901-L0X → 중기 제작·충전 인프라와 sim-to-real → 장기 지상 로봇·유선 드론 협업·군집 안전의 순차 계획 | [`../ai-startup-camp-drone/SOURCES.md`](../ai-startup-camp-drone/SOURCES.md), [`msp_sensor.h`](../../../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h), [`failsafe_land_research.md`](../../failsafe_land_research.md), [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md) | 배터리 계측, BMS 상태 공유, 충전 도크, 전용 검증 공간, 로봇개·지상 로봇 협업, 경로·군집 제어는 아직 구현·검증 근거가 없는 후속 계획. 현재 테더는 전원 공급용이 아니라 이동 범위를 제한한 안전줄이며, 장기 계획의 유선 드론과 구분함 |
| 14 | 감사 인사와 질의응답 | [`BRIEF.md`](BRIEF.md) | 추가 성과 주장 없이 발표 종료 |

## 제작 규모 산정 해석

- 1대: 프린터 점유 24~48시간, 직접 작업 6~10인시, 프린터 한 대에서 약 2~3일
- 10대: 프린터 점유 240~480시간, 직접 작업 60~100인시, 출력과 조립 병행 시
  프린터 한 대의 최소 점유 약 10~20일
- 가정: 부품 재고, 설계·펌웨어 준비, 조립 완료 FC PCB, 프린터 한 대,
  기체당 출력 질량 0.4~0.8kg, 유효 출력량 16.7g/h, 첫 출력 성공
- 현재 부품비 산정 합계: 1대 221,900~233,100원, 10대 2,219,000~2,331,000원
- 1대 추정 품목: 모터 4개 8.0만 원, ESC 4개 4.0만 원, MCU·센서 2.3만 원,
  프레임 1.1~2.2만 원, 배터리·프로펠러 6.8만 원
- 선택 센서: 3901-L0X 4.8~4.9만 원/대, 위 기본 합계에서 제외
- 가격 미포함: 자체 FC PCB, 배선·커넥터·XT60·수축튜브,
  나사·인서트·스탠드오프, 배송·관부가세·환전 수수료, 인건비·장비 감가,
  실패 출력·재작업, 자유비행 튜닝과 반복 비행 검증

위 시간은 실측이나 슬라이서 출력이 아닌 처리량 계획 가정이다. 가격 합계도 판매처
가격 증빙이 아닌 현재 부품비 산정이며, 실제 구매 SKU와 자체 PCB BOM이
확정된 완성기 총원가가 아니다.

## 증거 수준

- `host`: 노트북 단위 시험
- `SIL`: 실제 펌웨어를 포함한 소프트웨어 폐루프 시험
- `실기 테더`: 실제 센서·모터·기체에서 얻은 제한된 이동 범위의 기록
- `후속 목표`: 거리·광류 폐루프, 자유비행, 배터리·충전 인프라, 지상 로봇·유선 드론 협업, 경로 계획과 군집 제어

SIL과 테더 시험 결과를 안정 자유비행 완료 증거로 확대하지 않는다.

슬라이드 12의 영상과 로그는 각각 실제 테더 기록이지만, 공통 세션 식별자가
없으므로 같은 시험이나 같은 구간이라고 연결하지 않는다.
