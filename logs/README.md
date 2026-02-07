# Logs (로그)

이 디렉토리는 드론 운용 및 비행 중에 발생한 데이터 로그 파일들을 저장합니다. 주로 CSV 형식으로 저장되어 사후 분석 및 튜닝에 활용됩니다.

## 로그 파일 형식

- **파일명 규칙**: `flight_log_HHMMSS.csv` 또는 `drone_log_YYYYMMDD_HHMMSS.csv`
- **형식**: CSV (Comma-Separated Values)

## 포함 데이터 항목

일반적으로 다음과 같은 센서 및 상태 데이터가 기록됩니다:
- **Timestamp**: 기록 시간
- **IMU (Roll, Pitch, Yaw)**: 드론의 자세 데이터
- **PID Output**: 자세 제어를 위한 PID 제어기 출력값
- **Motor Output**: 각 모터에 전달되는 제어 신호 값
- **Battery Voltage**: 현재 배터리 전압

## 활용 방법
1. `scripts/Drone_Analasys.py` 스크립트를 사용하여 로그 파일을 시각화하고 분석할 수 있습니다.
2. 비행 중 불안정한 거동이 발생했을 때, 로그 분석을 통해 PID 이득 값을 조정하는 기초 자료로 사용합니다.
