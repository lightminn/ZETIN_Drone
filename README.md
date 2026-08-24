# ZETIN Drone

상용 FC 펌웨어를 쓰지 않고 센서 읽기·자세 추정·캐스케이드 PID·모터 믹싱·
안전장치를 직접 구현한 쿼드콥터 비행제어기다. 현재 스택은 ESP32-S3, 듀얼
ICM42670 IMU, BMM350 자기계, PWM ESC이며 지상국과는 WiFi SoftAP UDP로 통신한다.

**어디까지 왔나 (2026-08-24 기준)**

- 2026-08-01 첫 실비행에서 176초 연속 무장·고장 플래그 0건·제어루프 1000Hz를
  기록했다. roll/pitch 자세 유지, yaw heading 자동 잠금, 자기계 드리프트 보정,
  RC 두절 자동착륙이 공중에서 동작했다.
- **고도·위치를 폐루프에 넣는 센서가 아직 없다.** 스틱을 중립에 두면 기체가
  계속 흘러가므로 실내 호버는 조종자의 지속적인 역방향 조향에 의존한다.
- 자동착륙의 **착지 감지는 미해결**이다. IMU 기반 판별식 6종을 실측으로 배제한
  뒤 시간 기반 하강(`FS_MAX_MS=3000`ms)으로 대체돼 있다.

한 번의 로그는 반복 가능한 비행을 뜻하지 않는다. 성숙도 구분은
[프로젝트 개요](docs/project_overview.md)의 표를 따른다.

## 여기서 시작

- [프로젝트 개요](docs/project_overview.md)
- [현행 비행 제어 후보](firmware/flight/dual_imu_cascade_pwm/)
- [flix 기반 쿼터니언 제어 후보](firmware/flight/dual_imu_flix_quat_pwm/) (보류)
- [펌웨어 및 진단 가이드](firmware/README.md)
- [PC 제어·텔레메트리 도구](scripts/README.md)
- [UDP 프로토콜과 텔레메트리 스키마](docs/udp_protocol.md)
- [전원 인가 벤치 절차](docs/power_on_bench_procedure.md)
- [자동착륙 착지 감지 연구](docs/failsafe_land_research.md)
- [펌웨어 수명주기 카탈로그](docs/firmware_catalog.md)
- [문서 전체 목록](docs/README.md)

## 빠른 확인

```bash
for sketch in firmware/flight/*/; do
  arduino-cli compile --warnings all --fqbn esp32:esp32:esp32s3 \
    --build-path "/tmp/zetin-$(basename "$sketch")" "$sketch"
done

python3 -m py_compile scripts/*.py
python3 tools/check_repo_layout.py
python3 -m unittest discover -s tools -p "test_*.py"
```

## 안전

벤치 테스트에서는 프로펠러를 제거한다. 제한된 비행 테스트 전에 전원 극성,
핀 배치, 모터 순서, 보정 방향을 확인한다. 보관된 실험 코드는 지원 대상이
아니며 안전하지 않을 수 있다. 전체 절차는
[`docs/power_on_bench_procedure.md`](docs/power_on_bench_procedure.md)에 있다.

특히 다음 둘은 실기에서 확인된 함정이다.

- **게임패드는 USB로 연결한다.** 블루투스 패드는 노트북 WiFi와 2.4GHz 무선부를
  공유해 업링크를 수백 ms씩 막고, 그대로 RC 타임아웃 자동착륙을 유발한다
  (근거: [`docs/ground_station_link.md`](docs/ground_station_link.md)).
- **자동착륙 시험은 낙하해도 안전한 환경에서만 한다.** 착지를 감지하지 않고
  정해진 시간만 하강하므로, 접지 후에도 백스톱까지 프롭이 계속 돈다.
