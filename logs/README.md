# 비행 로그

[`receive_telemetry.py`](../scripts/receive_telemetry.py)와
[`monitor_telemetry.py`](../scripts/monitor_telemetry.py)는 다음 규칙으로 이곳에
파일을 생성한다.

```text
flight_log_YYYY-MM-DD_HHMMSS.csv
```

현행 CSV 파일은 PC 수신 시각 다음에 펌웨어 텔레메트리 65개 필드가 이어져
총 66개 열을 가진다.

```text
Timestamp,
Roll, Pitch, Yaw,
Gyro_X, Gyro_Y, Gyro_Z,
Accel_X, Accel_Y, Accel_Z,
Throttle,
Fault_RC, Fault_Critical,
RC_Total_Pkts, RC_Dropped_Pkts,
Fault_IMU1, Fault_IMU2, Fault_Disagree,
Active_IMUs, Mixer_Scaled, Fault_Attitude, Calibration_OK,
Armed,
Motor_M1, Motor_M2, Motor_M3, Motor_M4, PID_Loop_Hz,
TgtRate_Roll, TgtRate_Pitch, TgtRate_Yaw,
MagHeading, Mag_X, Mag_Y, Mag_Z,
Yaw_Hold,
Failsafe_Phase, Trim_Roll, Trim_Pitch,
Hover_Est, Hover_Valid,
Failsafe_Probe_State, Failsafe_Probe_NoResponse,
Failsafe_Probe_Response_G,
IMU1_Gyro_X, IMU1_Gyro_Y, IMU1_Gyro_Z,
IMU1_Accel_X, IMU1_Accel_Y, IMU1_Accel_Z,
IMU2_Gyro_X, IMU2_Gyro_Y, IMU2_Gyro_Z,
IMU2_Accel_X, IMU2_Accel_Y, IMU2_Accel_Z,
TgtAngle_Roll, TgtAngle_Pitch, TgtAngle_Yaw,
Mag_Enabled,
Range_MM, Range_Quality, Flow_X, Flow_Y, Flow_Quality,
Mag_Cal_Active
```

공유 파서는 더 짧은 과거 패킷도 받아들인다: `Throttle`에서 끝나는 10필드,
`RC_Dropped_Pkts`에서 끝나는 14필드, `Calibration_OK`에서 끝나는 21필드,
`Armed`에서 끝나는 22필드, `TgtRate_Yaw`에서 끝나는 30필드, `MagHeading`에서
끝나는 31필드, `Mag_Z`에서 끝나는 34필드, `Yaw_Hold`에서 끝나는 35필드.
`Trim_Pitch`에서 끝나는 38필드, `Hover_Valid`에서 끝나는 40필드,
`Failsafe_Probe_Response_G`에서 끝나는 43필드, `IMU2_Accel_Z`에서 끝나는
55필드, `TgtAngle_Yaw`에서 끝나는 58필드, `Mag_Enabled`에서 끝나는 59필드,
`Flow_Quality`에서 끝나는 64필드 패킷도 레거시로 받아들인다.
이 과거 패킷에 없는 필드는 빈 셀로 기록되므로, 오래된 로그는 뒤쪽 열이
비어 있다.
`Timestamp`는 항상 PC에서 추가하며 UDP 데이터그램의 일부가 아니다.

`Mag_Cal_Active`는 같은 행의 `Mag_X/Y/Z` 도메인을 식별한다. 1이면 세 필드는
보정 전 BMM350 raw µT이고 `scripts/magcal_fit.py`가 기본으로 이 행만 선택한다.
0이면 hard/soft-iron과 현재 throttle 간섭 보정 후 heading 계산에 사용하는
도메인이다. 시작 시 첫 raw 샘플과 함께 1로, 종료 시 보정값 복원과 함께 0으로
전환되므로 flag와 XYZ가 서로 다른 도메인을 가리키는 행은 만들지 않는다.

저장소 루트에서 생성된 로그를 분석한다.

```bash
python scripts/analyze_flight_log.py logs/flight_log_YYYY-MM-DD_HHMMSS.csv
```

이 스키마는 배터리 전압과 PID 항 분해(P/I/D 개별 기여) 열을 주장하지 않는다.
개별 모터 출력은 `Motor_M1`~`Motor_M4`로 기록된다. 와이어 포맷은
[`udp_protocol.md`](../docs/udp_protocol.md)를 참고한다.
