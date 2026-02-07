# ZETIN Drone Project

이 프로젝트는 드론 펌웨어 개발, 실시간 제어 스크립트, 그리고 비행 데이터 분석을 위한 통합 솔루션을 제공합니다.

## 프로젝트 구조

- [**firmware/**](./firmware): ESP32 기반 드론 제어 펌웨어 (PlatformIO)
- [**scripts/**](./scripts): Python 기반 드론 제어(DualSense), 모니터링 및 분석 툴
- [**logs/**](./logs): 비행 중 기록된 CSV 데이터 로그
- [**docs/**](./docs): 프로젝트 관련 문서 및 프레젠테이션 자료
- [**test/**](./test): 네트워크 및 모듈별 기능 테스트 스크립트

## 시작하기

### 1. 펌웨어 설정
`firmware/` 디렉토리에서 상세 가이드를 확인하세요.
- PlatformIO를 사용하여 소스 코드를 빌드하고 드론 보드에 업로드합니다.
- 다양한 하드웨어 테스트 예제는 `firmware/examples/`에서 확인할 수 있습니다.

### 2. 제어 스크립트 실행
드론과 통신하기 위한 Python 환경을 구축합니다.
```bash
pip install pygame pandas matplotlib
cd scripts
python Drone_Control_Dualsense.py
```

### 3. 데이터 분석
비행 완료 후 생성된 로그는 다음 스크립트로 분석할 수 있습니다.
```bash
python scripts/Drone_Analasys.py
```

## 라이선스 및 기여
이 프로젝트의 저작권은 ZETIN 팀에 있습니다.
