# 발표용 드론 원리 시각화

`assets/드론시각화.zip`의 Manim 원본을 발표 청중용으로 다시 설계한 소스이다.
기존 영상과 같은 파일명으로 출력하므로 HTML 슬라이드는 수정하지 않아도 된다.

## 디자인 원칙

- 1280×720, 30fps, H.264, `yuv420p`
- 발표자료 영상 프레임과 같은 진한 남색 배경
- Noto Sans CJK KR과 큰 직접 라벨 사용
- 상단 조건, 중앙 원인·결과 동작, 하단 결론의 고정된 읽기 순서
- 한 영상에 한 가지 결론만 제시

## 렌더링

기본 렌더러는 conda base의 Python이다. 최초 한 번 아래 의존성을 설치한다.

```bash
conda activate base
conda install pango
python -m pip install -r requirements-render.txt
```

이후 아래 스크립트를 실행한다.

```bash
./render_visualizations.sh
```

다른 Python 환경을 쓰려면 `PYTHON_BIN=/경로/python`을 앞에 붙인다. 스크립트는
`PYTHON_BIN`, conda base, 이 폴더의 `.venv` 순으로 렌더러를 찾는다.

출력 대상은 다음 9개이다.

- `accelerometer.mp4`
- `gyro.mp4`
- `complementary-filter.mp4`
- `gyro-bias.mp4`
- `imu-axis-signs.mp4`
- `pi-error-correction.mp4`
- `cascade-loop-timing.mp4`
- `yaw-correction.mp4`
- `landing-ambiguity.mp4`

`yaw-correction.mp4`는 기존의 스로틀 간섭 비교 장면을 사용하지 않는다. 슬라이드
48의 설명과 맞게 자이로 헤딩 표류와 지자기 장기 기준으로의 복귀를 보여준다.
