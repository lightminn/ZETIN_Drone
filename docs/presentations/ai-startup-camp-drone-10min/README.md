# 자작 드론 비행 제어 개발 결과 발표자료

`index.html`이 최신 편집 기준본이다. 14장 안에서 실제 기체·제어 보드 제작,
비행제어 펌웨어, SIL 오류 주입, 안전 전환과 실기 테더 기록을 결과 중심으로
설명한다. 사업·교육 가능성은 기술 결과의 활용 맥락으로만 짧게 둔다.

## 결과 흐름

1. 상용 제품과 자작 학습의 목적 구분
2. 센서부터 모터까지 이어지는 시스템 구조
3. 전원 경로와 CAD → 출력 → 조립 → 수정의 래피드 프로토타이핑
4. 비행 원리, 자세 추정과 Roll 믹서 오류 추적
5. 실제 펌웨어 SIL, 캐스케이드 제어와 Yaw 기준
6. 호버 추정·스로틀 조건 기반 안전 전환과 착지 프로브 판정 한계
7. 테더 영상·모터 출력 해석과 단기·중기·장기 계획
8. 감사 인사와 질의응답

## 로컬 발표

```bash
cd docs/presentations/ai-startup-camp-drone-10min
./present.sh
```

Chrome 발표 창이 닫히면 로컬 서버도 함께 종료된다. 실기 영상은 해당 장이
열리면 음소거 상태로 자동 재생된다.

## 제작 규모 근거

4장의 시간·비용은 실적이 아니라 계획 시나리오이다. 화면에는 모터·ESC와 기본
합계, 기본 합계에서 제외한 3901-L0X 선택 비용만 표시한다. 부품 보유, 조립 완료된
FC PCB, 프린터 한 대와 첫 출력 성공을 가정한 상세 계산과 제외 범위는
[`../ai-startup-camp-drone/PRODUCTION_ESTIMATE.md`](../ai-startup-camp-drone/PRODUCTION_ESTIMATE.md)에 있다.

## PPTX

```bash
cd docs/presentations/ai-startup-camp-drone-10min
node export_pptx.cjs
```

`드론_10분_요약본.pptx`는 현재 `index.html`을 기준으로 다시 생성한 최신
산출물이다. 14장 발표자 노트와 실기 영상 1개를 포함하며, HTML을 다시 수정한
뒤에는 위 명령으로 PPTX도 함께 갱신한다.

## 검증

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_presentation_10min -v
```

슬라이드 수, 발표자 노트, 필수 결과 메시지, 로컬 자산, 줄 쪼개짐과 1280×720
렌더 경계를 검사한다. PPTX가 최신 발표자 노트와 한 개의 실제 테더 영상을
포함하는지도 함께 확인한다.
