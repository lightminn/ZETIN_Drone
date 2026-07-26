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
python scripts/gamepad_probe.py
```

`gamepad_probe.py`는 게임패드의 축·버튼·햇 인덱스와 휴지값을 실측하는 단독
진단 도구다(드론 무관). 각 물리 컨트롤을 하나씩 움직여 어떤 axis/button이
반응하는지 확인해 `control_dualsense.py`의 매핑을 맞출 때 쓴다.

`receive_telemetry.py`와 `monitor_telemetry.py`는 모두 UDP 4210을 사용하고
수신 내용을 `logs/`에 기록한다. 두 도구는
[`telemetry_schema.py`](telemetry_schema.py)의 동일한 필드 정의와 파서를
공유하므로 10개, 14개, 21개, 22개, 30개, 31개 필드 레거시 텔레메트리와 현재
34개 필드 패킷을 같은 방식으로 해석한다. CSV에는 PC 수신 시각까지 포함해
35개 열을 쓴다.

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

오래된 GPS/TCP 실험 도구는 [`archive/README.md`](archive/README.md)에서만
찾을 수 있으며 현행 지원 범위가 아니다.
