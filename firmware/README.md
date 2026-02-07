# Firmware (펌웨어)

이 디렉토리는 드론의 핵심 제어 로직을 담당하는 펌웨어 소스 코드를 포함하고 있습니다. PlatformIO 및 아두이노 프레임워크를 기반으로 구성되어 있습니다.

## 디렉토리 구조

- `src/`: 드론의 주 실행 로직이 담긴 소스 코드 (`main.cpp`, `motor.cpp`, `sensor.cpp` 등).
- `lib/`: 센서 제어, 모터 드라이버, 통신 프로토콜 등을 위한 사용자 정의 라이브러리.
- `include/`: 프로젝트 전반에서 사용되는 헤더 파일.
- `examples/`: 각 하드웨어 모듈별 독립 기능 테스트를 위한 코드 모음.
    - `DSHOT_TEST`: DShot 통신 방식의 모터 제어 테스트.
    - `PWM_TEST`: PWM 방식의 모터 제어 테스트.
    - `BMM_TEST`: 지자기 센서 테스트.
    - `US100_TEST`: 초음파 거리 센서 테스트.
- `platformio_config/`: PlatformIO 빌드 설정 파일 (`platformio.ini`).

## 개발 환경 설정

1. **PlatformIO 설치**: VS Code 확장에서 PlatformIO IDE를 설치합니다.
2. **보드 설정**: `platformio_config/platformio.ini` 파일을 확인하여 대상 보드(예: ESP32)에 맞는 설정을 확인하세요.
3. **빌드 및 업로드**: IDE의 빌드 버튼을 눌러 컴파일하고, 업로드 버튼을 눌러 드론에 펌웨어를 기록합니다.

## 주의 사항
- 모터 테스트를 진행할 때는 반드시 프로펠러를 제거한 상태에서 진행하십시오.
- 배터리 전원 연결 시 전압과 극성에 유의하십시오.
