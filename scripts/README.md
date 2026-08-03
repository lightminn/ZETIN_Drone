# PC tools

현행 ESP32-S3 펌웨어의 UDP 제어, 텔레메트리 수신, 모니터링과 로그 분석
도구다. 아래 명령은 저장소 루트에서 실행한다.

```bash
python scripts/control_dualsense.py [--no-raw] [--no-mag]
python scripts/tune_pid.py
python scripts/receive_telemetry.py
python scripts/monitor_telemetry.py
python scripts/analyze_flight_log.py [optional-log.csv]
python scripts/analyze_probe_response.py <csv> [<csv> ...] [--label ground|air]
python scripts/decode_imu_raw.py <input.bin> [output.csv]
python scripts/receive_dual_imu_debug.py
python scripts/test_dualsense_input.py
```

`receive_telemetry.py`와 `monitor_telemetry.py`는 모두 UDP 4210을 사용하고
수신 내용을 `logs/`에 기록한다. 두 도구는
[`telemetry_schema.py`](telemetry_schema.py)의 동일한 필드 정의와 파서를
공유하므로 10개, 14개, 21개, 22개, 30개, 31개 필드 레거시 텔레메트리와 현재
64개 필드 패킷을 같은 방식으로 해석한다. 필드 44~55는
body frame의 `IMU1_Gyro_X/Y/Z`, `IMU1_Accel_X/Y/Z`,
`IMU2_Gyro_X/Y/Z`, `IMU2_Accel_X/Y/Z`다. gyro는 소프트웨어 LPF 적용
후 값이고 accel에는 소프트웨어 LPF가 없다. 마지막 3개 필드는
`TgtAngle_Roll/Pitch/Yaw` 목표 각도와 실제 융합 상태 `Mag_Enabled`가
뒤따른다. `Mag_Z`에서 끝나는 34필드,
`Yaw_Hold`에서 끝나는 35필드, `Trim_Pitch`에서 끝나는 38필드 패킷도
레거시로 받아들인다. `Hover_Valid`에서 끝나는 40필드도 프로브 진단 도입 전
패킷으로 수락하며, `Failsafe_Probe_Response_G`에서 끝나는 43필드는 IMU별
텔레메트리 도입 전 패킷으로 수락한다. `IMU2_Accel_Z`에서 끝나는 55필드도
목표 각도 텔레메트리 도입 전 패킷으로 수락하고, `TgtAngle_Yaw`에서 끝나는
58필드는 mag 상태 텔레메트리 도입 전 패킷으로 수락하며, `Mag_Enabled`에서
끝나는 59필드는 3901-L0X 텔레메트리 도입 전 패킷으로 수락한다. 필드 60~64는
`Range_MM`, `Range_Quality`, `Flow_X`, `Flow_Y`, `Flow_Quality`다. CSV에는
PC 수신 시각까지 포함해 65개 열을 쓴다.

`analyze_probe_response.py`는 Stage E-4a용으로 프로브 판정 이벤트와
`Hover_Est` 일관성을 확인하고, 지면/공중 응답 분포 및 1.5배 여유를 판정한다.
두 로그를 비교할 때는
`python scripts/analyze_probe_response.py ground.csv air.csv --label ground --label air`
처럼 CSV 순서대로 `--label`을 반복한다. 같은 라벨의 CSV만 여러 개 분석할
때는 `--label`을 한 번만 주면 모든 입력에 적용된다. `--plot`을 지정할 때만
matplotlib을 불러오므로 터미널 요약에는 matplotlib이 필요 없다.

`control_dualsense.py`는 정상 조종·벤치 작업에서 쓰는 **유일한 대화형
지상국**이다. DualSense RC 스트리밍, stdin 명령·PID 조정, `gains` 확인,
상태/고장 표시와 CSV 기록을 한 프로세스에서 처리한다.
자기계 yaw 융합 선택은 기본 **ON**이다(BMM350 벤치 검증 완료, 자이로 단독은
`YAW_DEADZONE` 이하 회전을 되돌릴 수단이 없다 → `docs/bmm350_yaw_bench_test.md`).
stdin의 `mag on`/`mag off`가 선택을 바꾸고 드론에 즉시 `mag 1`/`mag 0`을
보내며, 다음 시동에서도 지상국이 그 선택을 `start`보다 먼저 다시 보낸다.
꺼진 상태로 시작하려면 `python scripts/control_dualsense.py --no-mag`를
사용한다. `[STATUS]`의
`Mag=`는 지상국 선택이 아니라 텔레메트리 `Mag_Enabled`이므로 명령 거부·유실도
드러나며, 구형 패킷에서는 `-`다.
`AGL=`은 Matek 3901-L0X 거리계다. `1.23m/q200` 형식이고, 범위 밖이면
`OOR/q<품질>`, 신선한 프레임이 없으면 `-`다. 거리 원값이 범위 밖에서 음수라
숫자를 그대로 보여주면 오해를 만들기 때문에 품질로 신선도를 먼저 판단한다.
무장 중 `[DIAG]` 줄의 `dG`는 세 body-frame gyro 축에서 계산한
`max(|IMU1 − IMU2|)`를 소수 한 자리로 표시한다. 구형 패킷처럼 IMU별
12필드 중 하나라도 없으면 `dG=-`로 표시하며, CSV에는 수신한 12개 원본값을
모두 기록한다. 같은 줄의 `eR`/`eP`는 각각
`TgtAngle_Roll − Roll`/`TgtAngle_Pitch − Pitch` 각도 추종 오차다. 목표 각도가
없는 구형 패킷은 `-`로 표시한다. yaw 목표 각도는 래핑과 `Yaw_Hold` 상태를
함께 봐야 하므로 `[DIAG]`에서 오차로 요약하지 않고 CSV에만 기록한다.

IMU별 값도 20Hz 스냅샷이므로 Nyquist 주파수는 10Hz다. 프롭 진동은
앨리어싱되며, 이 값이나 `dG`만으로 진동·순간 disagree·freeze를 판정할 수
없다. 느린 bias/scale 드리프트와 정상상태 오프셋 관찰에만 사용한다.

1kHz 원시 스트림은 펌웨어에서 기본 ON이다. 첫 `ZIMU` 또는 `ZCAL`
데이터그램을 받을 때만 기존 flight log와 같은 타임스탬프의
`logs/imuraw_<timestamp>.bin`을 lazy-open하므로, raw 패킷을 받지 않은
세션에는 빈 `.bin`이 남지 않는다. `[STATUS]`의
`Raw=<batches>b/<dropped>d`는 받은 `ZIMU` 배치 수와 펌웨어 생산자 누적
드롭을 뜻한다. stdin의 `raw on`/`raw off`로 런타임 게이트를 그대로 바꿀 수
있다.

지상국을 종료하면 `.bin`을 닫은 뒤 같은 이름의 `.csv`로 자동 변환하고,
`batches`, `samples`, `wireless_lost`, `producer_dropped`,
`average_sample_rate_hz`, `dt_us_min`, `dt_us_max`를 요약한다. 변환 전에
진행 안내를 출력하며, 실측 비용은 기록 시간의 약 2%라 10분 세션은 약
12초가 걸린다. 변환이 실패해도 경고만 출력하고 정상 종료하며 원본 `.bin`은
그대로 남긴다.

`.bin`은 실시간으로 파싱하지 않고 데이터그램 바이트를 그대로 이어 쓴 와이어
포맷 원본이다. 자동 변환과 별개로 다음처럼 언제든 다시 디코딩할 수 있다.

```bash
python scripts/decode_imu_raw.py logs/imuraw_2026-07-30_120000.bin
python scripts/decode_imu_raw.py logs/imuraw_2026-07-30_120000.bin /tmp/imu.csv
```

출력을 생략하면 입력과 같은 경로의 `.csv`를 만든다. 디코더는 원시 int16
12축과, `ZCAL`이 있으면 bias/scale 및 IMU2 부호를 적용한 IMU1 센서 프레임
dps/g 12축을 기록한다. stderr 요약의 `wireless_lost_batches`는
`batch_seq` 구멍, `producer_dropped`는 링 포화이며 서로 다른 결측이다.
`ZCAL`이 없으면 원시 컬럼만 만들고 경고한다. 잘린/알 수 없는/미지원 버전
레코드는 건너뛰되 각각 개수를 보고한다.

저장 공간이나 RF 부하 때문에 세션 단위로 raw를 쓰지 않으려면
`python scripts/control_dualsense.py --no-raw`로 시작한다. 시작 시 드론에
`raw 0`을 반복 전송하고, 수신된 raw 데이터그램도 기록하지 않으므로 해당
세션에는 `.bin`과 raw `.csv`가 생기지 않는다. 기본 인자 없는 동작은 raw
기록과 종료 시 자동 변환이다.

하드웨어 LPF는 gyro 121Hz, accel 25Hz다. 1kHz raw 파일이라도 accel에는
25Hz 위 내용이 없으므로 accel 진동 스펙트럼 분석에는 사용할 수 없다. 이
기능은 제어 루프, FreeRTOS tick, 센서 ODR, 하드웨어 LPF 설정을 바꾸지 않는다.

`control_dualsense.py`는 yaw 스틱을 상태 없는 `rcr` 각속도 명령으로 보내며,
`YAW_RATE_MAX_DPS = 90.0`에 따라 최대 편향을 ±90dps로 제한한다. 펌웨어의
±180dps 하드 제한과는 별도의 지상국 조종감 상수다.
무장 중 `[DIAG]`의 `Trim=<roll>/<pitch>`는 **드론이 실제로 들고 있는**
`Trim_Roll`/`Trim_Pitch`다. 지상국의 D-pad 값과 다르면 끝에 `!`가 붙는다
(`Trim=+0.0/+1.2!`) — 명령이 아직 반영되지 않았다는 뜻이고, 지상국이
0.2초 간격으로 다시 보내 곧 사라진다. 구형 패킷은 `-`다. 이 값을 지상국
변수로 표시하면 유실·거부가 성공과 똑같이 보이므로 텔레메트리만 쓴다.

roll·pitch 트림은 `trim <roll> <pitch>` 절대값 명령으로 드론에 저장된다.
D-pad 전송은 **블로킹하지 않는다.** 예전에는 `reliable_send`로 5회
재전송하며 `time.sleep`을 돌았는데, 그 호출이 `rcr`을 50Hz로 보내는
스레드에서 일어나 누를 때마다 RC 업링크가 100ms 멈췄다(RC 타임아웃은
500ms). 지금은 한 번만 보내고, 유실은 텔레메트리와 비교해 다시 보내는
닫힌 루프로 메운다 — `trim`이 증분이 아닌 절대값이라 재전송이 안전하다.
지상국은 **시동할 때 트림을 보내지 않는다** — 스크립트를 재시작하면 지역
변수가 0이라 그 전송이 드론에 저장된 트림을 지워버리기 때문이다. 대신 첫
텔레메트리의 `Trim_Roll`/`Trim_Pitch`를 읽어 지역 값을 맞추고, 조종자가
D-pad로 실제로 바꿨을 때만 전송한다. 드론이 재부팅했다면 그 트림은 0이고
지상국도 0을 채택한다.

⚠️ 벤치 스크립트(`bench_*.py`)는 `rc <seq> 0 0 0`을 "수평 유지"로 쓰지만
펌웨어가 여기에 트림을 더하므로, 세 스크립트 모두 시작 시 `trim 0 0`을
보내 드론의 트림을 지운다. 조종용 트림을 유지한 채 벤치를 돌릴 수는 없다.

주요 의존성은 `pygame`, `pandas`, `matplotlib`이며 저장소 루트의
`requirements.txt`에 정리돼 있다. 가상환경 기준 설치 방법:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

`tune_pid.py`와 `monitor_telemetry.py`는 게임패드가 없는 상황이나 수동 UDP
확인에 쓰는 독립 보조 도구다. 두 도구 중 하나라도
`control_dualsense.py`와 **동시에 실행하면 안 된다.** 펌웨어는 들어온 모든
UDP 패킷의 발신자 IP/포트로 텔레메트리 목적지를 다시 지정하므로, 보조 도구가
명령을 보내는 순간 텔레메트리를 가져간다. 20Hz RC를 보내는
`control_dualsense.py`와 다른 대화형 도구는 구조적으로 공존할 수 없다.
DualSense 조종 중 텔레메트리 확인과 로그 기록은 `control_dualsense.py` 하나로
수행한다. `receive_telemetry.py`도 같은 UDP 4210을 쓰는 독립 수신 도구이므로
동시에 실행하지 않는다.

⚠️ `control_dualsense.py`로 조종할 때 **게임패드는 USB로 연결한다.** 블루투스
패드는 노트북 WiFi와 무선부를 공유해 드론으로 가는 업링크를 수백 ms씩 막고,
비행 중 RC 타임아웃(자동착륙)을 유발한다. 근거와 진단법은
[`docs/ground_station_link.md`](../docs/ground_station_link.md) 참조.

오래된 GPS/TCP 실험 도구는 [`archive/README.md`](archive/README.md)에서만
찾을 수 있으며 현행 지원 범위가 아니다.
