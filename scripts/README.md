# PC tools

현행 ESP32-S3 펌웨어의 UDP 제어, 텔레메트리 수신, 모니터링과 로그 분석
도구다. 아래 명령은 저장소 루트에서 실행한다.

```bash
python scripts/control_dualsense.py
python scripts/tune_pid.py
python scripts/receive_telemetry.py
python scripts/monitor_telemetry.py
python scripts/analyze_flight_log.py [optional-log.csv]
python scripts/receive_dual_imu_debug.py
python scripts/test_dualsense_input.py
```

`receive_telemetry.py`와 `monitor_telemetry.py`는 모두 UDP 4210을 사용하고
수신 내용을 `logs/`에 기록한다. 두 도구는
[`telemetry_schema.py`](telemetry_schema.py)의 동일한 필드 정의와 파서를
공유하므로 10개, 14개, 21개, 22개, 30개, 31개 필드 레거시 텔레메트리와 현재
35개 필드 패킷을 같은 방식으로 해석한다. `Yaw_Hold`가 현행 패킷과 CSV의
마지막 필드이며, `Mag_Z`에서 끝나는 34필드 패킷도 레거시로 받아들인다.
CSV에는 PC 수신 시각까지 포함해 36개 열을 쓴다.

`control_dualsense.py`는 yaw 스틱을 상태 없는 `rcr` 각속도 명령으로 보내며,
`YAW_RATE_MAX_DPS = 90.0`에 따라 최대 편향을 ±90dps로 제한한다. 펌웨어의
±180dps 하드 제한과는 별도의 지상국 조종감 상수다.

주요 의존성은 `pygame`, `pandas`, `matplotlib`이며 저장소 루트의
`requirements.txt`에 정리돼 있다. 가상환경 기준 설치 방법:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

`control_dualsense.py`, `receive_telemetry.py`, `monitor_telemetry.py`는 모두
UDP 4210을 bind하므로 한 번에 하나만 실행한다. 펌웨어 프로토콜상 텔레메트리
목적지가 하나뿐이라 동시 실행은 지원하지 않으며, DualSense 조종 중 로그는
`control_dualsense.py`가 직접 `logs/`에 기록한다.

⚠️ `control_dualsense.py`로 조종할 때 **게임패드는 USB로 연결한다.** 블루투스
패드는 노트북 WiFi와 무선부를 공유해 드론으로 가는 업링크를 수백 ms씩 막고,
비행 중 RC 타임아웃(모터 정지)을 유발한다. 근거와 진단법은
[`docs/ground_station_link.md`](../docs/ground_station_link.md) 참조.

오래된 GPS/TCP 실험 도구는 [`archive/README.md`](archive/README.md)에서만
찾을 수 있으며 현행 지원 범위가 아니다.
