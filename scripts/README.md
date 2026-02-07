# Scripts (스크립트)

이 디렉토리는 드론과 PC 간의 통신을 통해 제어, 모니터링 및 데이터 분석을 수행하는 Python 스크립트들을 포함합니다.

## 주요 기능 및 스크립트

- **`Drone_Control_Dualsense.py`**: PS5 DualSense 컨트롤러를 사용하여 드론을 원격 제어합니다.
- **`Drone_Monitor.py`**: 실시간으로 드론의 센서 데이터와 상태를 그래프나 텍스트로 모니터링합니다.
- **`Drone_Analasys.py`**: `logs/` 디렉토리에 저장된 비행 로그 데이터를 불러와 시각화하고 분석합니다.
- **`Drone_Tuning.py`**: 드론의 PID 파라미터 등을 실시간으로 조정하기 위한 툴입니다.
- **`Controller_test.py`**: 연결된 컨트롤러의 입력이 정상적으로 들어오는지 확인하는 테스트 스크립트입니다.

## 실행 방법

1. **의존성 설치**:
   ```bash
   pip install pygame pandas matplotlib
   ```
   *(필요에 따라 추가 라이브러리 설치가 필요할 수 있습니다.)*

2. **스크립트 실행**:
   ```bash
   python Drone_Control_Dualsense.py
   ```

## 통신 설정
- 대부분의 스크립트는 UDP 또는 TCP/IP를 통해 드론과 통신합니다.
- 스크립트 내의 IP 주소 설정이 드론의 IP와 일치하는지 확인하십시오.
