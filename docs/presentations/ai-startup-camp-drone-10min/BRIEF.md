# 자작 드론 요약 발표자료 제작 기준

## 목적

기존 84장 발표자료에서 자작 드론의 하드웨어, 비행제어 소프트웨어,
검증 과정과 실기 증거를 골라 짧은 발표용 요약본을 만든다.

## 구성 원칙

- 14장, 16:9 화면
- 기술 내용이 본론이며 사업성과 교육 확장은 도입과 결론에서만 언급
- 한 장마다 주장 하나, 상단 제목과 하단 결론 문장
- 필요성은 개념도, 중반은 실제 CAD·PCB·코드·그래프, 후반은 실기 영상
- 팀 이름, 구체적인 날짜, 커밋 해시, 구체적인 발표·시연 시간 미표기
- 완전한 문장은 `~이다` 계열, 짧은 설명은 명사형
- 실제 비행·벤치·SIL·계획을 서로 구분하고 미검증 결과를 성과처럼 표현하지 않음
- 실제 기체 조종 체험과 중간 퀴즈 미포함

## 참고 자료

- `/home/light/Downloads/2족보행로봇 심사용 발표.pptx`
- `/home/light/Downloads/ZETIN_로봇팔_10분_요약본.pptx`
- `docs/presentations/ai-startup-camp-drone/index.html`
- `firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino`
- `tools/native_tests/test_sil_attitude.cpp`

## 산출물

- 편집 기준본 `index.html`
- 로컬 발표 실행기 `present.sh`
- 재생성 가능한 PPTX 변환기 `export_pptx.cjs`
- 영상이 포함된 `드론_10분_요약본.pptx`
