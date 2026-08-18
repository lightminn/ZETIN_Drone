# 자작 드론 기술 요약 발표자료

`index.html`이 편집 기준본이고 `드론_10분_요약본.pptx`는 발표용 산출물이다.
기존 84장 자료를 14장으로 압축했으며, 사업 설명은 도입과 결론에만 두고
하드웨어·비행제어·검증을 본론으로 구성했다.

## 흐름

1. 직접 만든 이유와 시스템 지도
2. 기체·PCB와 비행 원리
3. 자세 추정과 부호 계약
4. 실제 펌웨어 SIL과 캐스케이드 제어
5. Yaw 기준과 안전 계층
6. 실기 테더 기록, 현재 상태와 후속 목표

## 로컬 발표

```bash
cd docs/presentations/ai-startup-camp-drone-10min
./present.sh
```

Chrome 발표 창이 닫히면 로컬 서버도 함께 종료된다. 실기 영상은 해당 장이
열리면 음소거 상태로 자동 재생된다.

## PPTX 재생성

```bash
cd docs/presentations/ai-startup-camp-drone-10min
node export_pptx.cjs
```

PPTX는 14장의 화면을 고해상도 배경으로 저장하고, 발표자 노트와 H.264 실기
영상 한 개를 포함한다. 영상 재생은 데스크톱 PowerPoint 기준이며 웹 미리보기는
정지 포스터만 표시할 수 있다.

## 검증

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v
```

슬라이드 수, 발표자 노트, 금지 문구, 로컬 자산, PPTX 비율과 포함 영상을 검사한다.
