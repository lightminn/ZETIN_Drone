# 발표자료 주장과 근거

이 문서는 발표자료의 수치와 현재 상태를 저장소 근거로 추적하기 위한 목록이다.
펌웨어 상수·프로토콜 순서의 정답은 항상 현재 코드와 프로토콜 문서다. 날짜가
붙은 실험 기록은 그 시점의 증거이며, 현재 구현 상태와 구분해서 읽는다.

| 슬라이드 | 주장 | 저장소 근거 | 증거 시점·경계 |
|---|---|---|---|
| 1, 8, 11, 17~19, 29, 83 | 기술 중심 발표 방향, 교육 아이템 맥락, PCB 주문·납땜·부품 납기 상태, 다중 드론 확장 목표 | [`BRIEF.md`](BRIEF.md) | 사용자 제공 발표 준비 정리. 발표 방향과 실행 현황이며 비행 검증 증거가 아님 |
| 1, 4, 13~15, 17, 18, 44, 75 | 실제 기체·보드·CAD·코드·모바일 체험 시각자료 | [`assets/`](assets/), [`mobile-lab/`](mobile-lab/) | 직접 제작·구현한 산출물과 벤치 장면. 사진 자체가 비행 성능 검증을 뜻하지 않음 |
| 4 | 국내 법령상 초경량비행장치와 무인동력비행장치의 분류, 무인비행기·무인헬리콥터·무인멀티콥터·무인수직이착륙기 구분 | [항공안전법 시행규칙 제5조](https://www.law.go.kr/LSW/lsLinkCommonInfo.do?lspttninfSeq=140094), [교수 피드백 참고 영상](https://www.youtube.com/watch?v=agnIMXGlBHU) | 법적 분류를 발표용 계층도로 단순화. 우리 기체는 로터 4개의 무인멀티콥터 |
| 5 | 무인동력비행장치 조종자 증명 1~4종의 무게 구간, 4종 온라인 교육, 250g 이하 별도 증명 제외 | [국가법령정보센터 행정규칙](https://www.law.go.kr/LSW/admRulInfoP.do?admRulSeq=2100000259052&chrClsCd=010201), [한국교통안전공단 FAQ](https://main.kotsa.or.kr/portal/bbs/faq_list.do?cateCode=C07&menuCode=04010000) | 종별은 실제 최대이륙중량 측정 후 판단. 비행구역과 안전 규칙은 자격과 별도 |
| 6 | 고정익·헬리콥터·멀티콥터·eVTOL의 교환관계와 UAM의 기체·운항·인프라·교통관리 체계 | [FAA Air Taxis](https://www.faa.gov/air-taxis), [FAA UAM Blueprint](https://www.faa.gov/air-taxis/uam_blueprint), [FAA Helicopter Flying Handbook](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/helicopter_flying_handbook), [우주항공청 참고 영상](https://www.youtube.com/shorts/8UypNyBrKXY) | 개념·구조 비교. 특정 eVTOL 기체의 성능이나 상용화 시점을 주장하지 않음 |
| 7 | 엔터테인먼트·물류·안전감시·국방 임무와 임무별 필요 사양 | [FAA UAS Integration Pilot Program 보고서](https://www.faa.gov/sites/faa.gov/files/uas/programs_partnerships/completed/integration_pilot_program/IPP_Final_Report_20210712.pdf), [FAA UAS Field Test](https://www.faa.gov/uas/research_development/traffic_management/field_test), [미 육군 단거리 정찰 드론 소개](https://www.army.mil/article/280609/send_in_the_drones) | 대표적인 설계 요구조건의 정성 비교. 국방 내용은 작전 절차가 아닌 시스템 요구 수준 |
| 9~10 | 비행체에 작용하는 양력·중력·추력·항력, 네 로터 추력의 벡터 합, 차동 추력에 따른 롤·피치·요 토크, 헬리콥터 꼬리로터 비교 | [NASA Four Forces of Flight](https://www.nasa.gov/wp-content/uploads/2020/04/four_forces_of_flight.pdf), [FAA Helicopter Flying Handbook](https://www.faa.gov/regulations_policies/handbooks_manuals/aviation/helicopter_flying_handbook), [PX4 Control Allocation](https://docs.px4.io/v1.14/en/concept/control_allocation), [교수 피드백 참고 영상](https://www.youtube.com/watch?v=agnIMXGlBHU) | 작은 각도·준정상 상태의 설명용 모델. 실제 제어기는 센서 피드백과 모터 동특성을 함께 고려 |
| 11, 83 | 단일 기체 위에 추가되는 공통 시간·좌표, 통신, 상대 위치, 대형·경로 계획, 충돌 회피, 집단 안전장치 | [Caltech 다중 드론 자율 협력 연구](https://authors.library.caltech.edu/records/me2vb-qg826), [Frontiers 군집 로봇 리뷰](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.00018/full), [YTN 사이언스 참고 영상](https://www.youtube.com/watch?v=I4S6nJA0IKM&t=17s) | 일반적인 군집 시스템 요구조건. 이 프로젝트에서는 후속 목표이며 아직 구현·비행 검증하지 않음 |
| 21, 33, 78 | 상태 텔레메트리 65개 필드, 별도 1kHz 원시 IMU | [`udp_protocol.md`](../../udp_protocol.md), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 현행 프로토콜. CSV는 수신 시각까지 66열 |
| 37~39, 53 | Roll·Pitch 적응형 α = 0.999 / 0.9995 / 0.9998 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `ALPHA_STATIC`, `ALPHA_NORMAL`, `ALPHA_DYN`, `compute_alpha` | 현행 펌웨어 |
| 45~48 | 실제 비행 스케치 직접 포함, 1ms tick hook, 독립 물리·듀얼 IMU 합성, S1 정상 수렴과 부호 반전 실패 | [`test_sil_attitude.cpp`](../../../tools/native_tests/test_sil_attitude.cpp), [`Arduino.h`](../../../tools/native_tests/shims/Arduino.h), [`test_sil_attitude.py`](../../../tools/test_sil_attitude.py) | 현행 32비트 host SIL. 정상 S1은 Roll/Pitch 마지막 500 tick 평균 0.0513°/0.1179°, 부호 반전은 최대 Roll 1,291.8°에서 실패. 실비행 증거가 아님 |
| 53, 55, 56 | Yaw 보정 250Hz, K=0.001, 시정수 약 4초 | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `K_MAG`, `magFusionCnt` | 현행 펌웨어. 1kHz α 값은 설명용 등가식일 뿐 구현값이 아님 |
| 42 | 기체축 자이로 +Y/−X/−Z, 가속도 +Y/−X/+Z | [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) `bodyGx`~`bodyAz` | 현행 펌웨어 변환식 |
| 54, 58 | 정지 기체에 0.6°/s 자이로 바이어스를 30초 주입했을 때 자이로 단독 오차 18.3091°, 지자기 융합 오차 2.4394° | [`test_mag_yaw_fusion.cpp`](../../../tools/native_tests/test_mag_yaw_fusion.cpp) `gyro bias drift rejection`, [`test_mag_yaw_fusion.py`](../../../tools/test_mag_yaw_fusion.py) | 현행 32비트 host SIL. 실제 센서 벤치나 비행 증거가 아님 |
| 57, 58 | 모터 전류 간섭 보정 전후 헤딩 기울기 +3.64° → +0.02°/100µs | [`bmm350_yaw_bench_test.md`](../../bmm350_yaw_bench_test.md), [`chartdata.json`](chartdata.json) | 2026-07-27 벤치 로그 결과. 당시 min/max 보정 기반이며 현행 타원체 보정과 구분 |
| 57~59 | 제약 타원체 hard/soft-iron 보정과 보드·상하행 램프 재검증 | [`bmm350_yaw_bench_test.md`](../../bmm350_yaw_bench_test.md), [`magcal_fit.py`](../../../scripts/magcal_fit.py), [`control_dualsense.py`](../../../scripts/control_dualsense.py), [캘리브레이션 설계](../../superpowers/specs/2026-08-04-magnetometer-calibration-design.md) | 2026-08-04 실측. 정상 지상국 운용은 Mag ON, 펌웨어 부팅값은 OFF |
| 63~64, 68~70 | 프로브는 기록 전용, `landed=false`, 3초 시간 기반 하강 | [`failsafe_land_research.md`](../../failsafe_land_research.md), [`failsafe_land.h`](../../../firmware/flight/dual_imu_cascade_pwm/failsafe_land.h), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 2026-08-01 정정 이후 현행 상태. 공중 프로브 분포 미측정 |
| 65 | PMW3901 광류와 VL53L0X ToF 거리의 결합 원리, 8cm~2m 작동 범위, 940nm 광원 | [MATEKSYS 제품 자료](https://www.mateksys.com/?portfolio=3901-l0x), [ST VL53L0X 자료](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) | 제조사 원리·사양. 우리 기체의 실측 성능을 뜻하지 않음 |
| 66 | 하부 장착·렌즈 하향·화살표 전방, 5V/GND/TX 배선과 GPIO16 수신 | [ArduPilot 장착 안내](https://ardupilot.org/plane/docs/common-mateksys-optflow-3901L0X.html), [MATEKSYS 제품 자료](https://www.mateksys.com/?portfolio=3901-l0x), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino) | 기구·배선 계획과 현행 핀 정의. 최종 브래킷·오프셋 실측은 HW 확인 전 |
| 64~67, 70, 82, 83 | 3901-L0X의 UART 115200bps·MSPv2 수신, 500ms 신선도, 필드 60~64 기록과 향후 고도·위치 제어 계획 | [`udp_protocol.md`](../../udp_protocol.md), [`msp_sensor.h`](../../../firmware/flight/dual_imu_cascade_pwm/msp_sensor.h), [`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino), [ArduPilot 활용 안내](https://ardupilot.org/plane/docs/common-mateksys-optflow-3901L0X.html) | 수신·파싱·신선도 표시는 구현됨. 착지·고도·위치 폐루프와 실기 효과는 아직 검증하지 않음 |
| 77, 78 | 테더 구간 106.4초, Roll σ=1.65°, Pitch σ=1.23°, M1 1334.2µs, M3 1360.0µs | [`chartdata.json`](chartdata.json), [`flight_log_2026-07-27_041032.csv`](../../../logs/flight_log_2026-07-27_041032.csv) 4,194~6,328행 | 2026-07-27 테더 세션의 2,135행. 안정 비행 완료 증거가 아님 |
| 76, 84 | 실제 제작 기체의 자세 제어 구동 영상 | [`hover_demo.mp4`](assets/hover_demo.mp4) | 테더를 건 실기 시험 기록. 반복 가능한 안정 호버나 자유 비행 완료를 뜻하지 않음 |
| 82 | 첫 실비행 176.2초, 고장 0건, PID 평균 1000Hz·최소 999Hz, Mag ON, Yaw hold 91.9% | [`power_on_bench_procedure.md`](../../power_on_bench_procedure.md), [`flight_log_2026-08-01_003319.csv`](../../../logs/flight_log_2026-08-01_003319.csv) | 2026-08-01 실비행 관측. 안정 호버·자동착륙 완료를 뜻하지 않음 |

## 성숙도 경계

- 코드·호스트 테스트로 확인: 센서 수집, 축 변환, 필터·제어 계산, 텔레메트리,
  프로브 기록 경로, 3901-L0X 프레임 파싱.
- 날짜가 붙은 벤치·비행으로 확인: 위 표의 7월 27일, 8월 1일, 8월 4일 기록.
- 아직 확인하지 않음: 반복 가능한 안정 호버, 신뢰할 수 있는 실내 저고도
  자동착륙, 거리·광류 폐루프 제어, 실외 자유 비행, 경로·군집 제어.

발표자료에서 `검증`이라고 쓸 때는 위 세 범주 중 무엇인지 함께 밝힌다.
