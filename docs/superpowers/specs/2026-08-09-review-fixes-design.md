# 캘리브레이션 리뷰 지적 수정 설계

작성: 2026-08-09
대상 브랜치: `feat/magcal-ellipsoid-fit`

## 1. 목표와 범위

코드 리뷰에서 확인된 세 결함을 기존 프로토콜과 비행 제어 동작을 바꾸지 않는
최소 범위로 수정한다.

1. `docs/README.md`가 가리키는 캐스케이드 PID Typst 원본을 실제 변경 집합에
   포함한다.
2. 자기장 크기 잔차를 heading 정확도로 잘못 환산하는 출력과 문서를 제거한다.
3. `Mag_Cal_Active`와 `Mag_X/Y/Z`가 항상 같은 데이터 도메인을 설명하도록
   전환을 동기화한다.

필드 1~65의 순서와 형식, hard/soft-iron 상수, `mag_comp`, yaw 융합, roll/pitch,
믹서 및 비행 제어 게인은 바꾸지 않는다. 하드웨어 플래시와 실비행은 범위 밖이다.

## 2. 저장소 문서 링크

`docs/README.md`의 링크는 유지하고 `docs/cascade_vs_single_pid.typ`을 변경 집합에
포함한다. 레이아웃 검증은 현재 작업트리뿐 아니라 HEAD 아카이브에 tracked diff와
새 Typst 원본을 적용한 임시 깨끗한 트리에서도 실행한다.

생성된 PDF는 현재 파일을 보존하지만 이번 결함 수정의 필수 입력으로 삼지 않는다.
저장소에 tracked PDF 선례가 없고 README 링크도 재생성 가능한 Typst 원본을
가리키기 때문이다.

## 3. 캘리브레이션 품질 리포트

`scripts/magcal_fit.py`의 `CalibrationResult.heading_error_95_deg`를 제거하고,
95퍼센타일 자기장 크기 잔차를 물리량 그대로 보존한다.

- 결과 필드: `radial_residual_95_ut`
- CLI 출력: 95퍼센타일 `|B|` radial residual의 µT와 목표 반경 대비 %
- 금지 표현: heading error, heading accuracy, 각도 오차 상한

크기 잔차는 보정된 점이 구면에 얼마나 가까운지만 나타낸다. 센서/기체 축 회전은
벡터 방향을 틀리게 하면서도 크기를 정확히 보존할 수 있으므로, 독립적인 정답 자세
데이터가 없는 캘리브레이션 캡처에서는 heading 정확도를 계산하지 않는다.

합성 데이터 테스트가 사용하는 정답 방향 기반 heading 비교는 독립 기준이 있으므로
유효하며 유지한다. 새 회귀 테스트는 모든 벡터를 30° 회전한 사례에서 radial
residual이 0에 가까워도 실제 heading 오차가 30°임을 보여, CLI가 이를 각도로
표현하지 못하게 한다. 운용 문서의 품질 확인 항목도 radial residual로 고친다.

## 4. 텔레메트리 도메인 동기화

### 4.1 상태 분리

내부 제어 상태 `mag_calibrating`과 wire 도메인 상태를 분리한다. 새
`magTelemCalActive`는 `Mag_X/Y/Z` 스냅샷과 함께만 바뀌며, 텔레메트리 65번 필드는
`mag_calibrating` 대신 이 값을 보낸다.

기존 `magSnapshotMux` 임계구역 안에서 다음 네 값을 한 스냅샷으로 게시하고 읽는다.

- `magTelemX`
- `magTelemY`
- `magTelemZ`
- `magTelemCalActive`

### 4.2 시작 전환

`magcal 1`은 먼저 `mag_enabled=false`로 만들고 내부 캘리브레이션 상태를 시작한다.
이 시점에는 새 raw 샘플이 없으므로 wire 스냅샷은 기존 보정 XYZ와 flag 0을 유지한다.
첫 유효 raw BMM350 샘플을 받을 때 raw XYZ와 flag 1을 같은 임계구역에서 게시한다.

Core 1의 보정 XYZ 게시 함수는 같은 임계구역에서 `mag_calibrating`을 확인한다.
캘리브레이션 시작 전에 이미 진입했던 늦은 writer도 raw 캡처 상태를 덮어쓸 수 없다.

### 4.3 종료 전환

`magcal 0`은 마지막 유효 raw 샘플에서 이미 계산·게시된 hard/soft-iron 보정
`MagSnapshot`을 읽는다. 현재 `base_throttle`에 해당하는 기존 `mag_comp`를 적용한
정상 도메인 XYZ를 만든 뒤, 내부 캘리브레이션 종료·정상 XYZ·wire flag 0을 같은
임계구역에서 게시한다.

샘플이 한 건도 없으면 wire 스냅샷은 처음부터 flag 0이므로 기존 정상 값을
보존한다. 필드 수를 늘리거나 별도 raw XYZ 필드를 추가하지 않는다.

### 4.4 wire 계약

- `Mag_Cal_Active=1`: 같은 패킷의 `Mag_X/Y/Z`는 보정 전 BMM350 raw µT
- `Mag_Cal_Active=0`: 같은 패킷의 `Mag_X/Y/Z`는 hard/soft-iron 및 현재 throttle
  간섭 보정 후 값

패킷은 계속 65필드이며 과거 10~64필드 prefix 호환성을 유지한다.

## 5. 테스트와 검증

모든 생산 코드 수정은 회귀 테스트를 먼저 추가하고 현재 코드에서 기대한 이유로
실패하는 것을 확인한 뒤 적용한다.

1. `tools/test_magcal_fit.py`
   - report가 radial residual을 µT/%로만 노출한다.
   - 30° 순수 회전 반례가 크기 잔차로 heading 정확도를 추론할 수 없음을 고정한다.
2. `tools/native_tests/test_mag_yaw_fusion.cpp`
   - 시작 직후: 보정 XYZ + flag 0
   - 첫 raw 샘플 후: raw XYZ + flag 1
   - 종료 직후: 다시 보정 XYZ + flag 0
   - 늦은 Core 1 정상 writer가 active raw 스냅샷을 덮어쓰지 못한다.
3. 저장소 레이아웃
   - 새 Typst 원본을 포함한 임시 깨끗한 트리에서 `tools/check_repo_layout.py`가
     통과한다.
4. 최종 회귀
   - 관련 Python/native 테스트
   - 전체 `tools/test_*.py` discovery
   - `typstyle --check` 및 Typst PDF 임시 컴파일
   - ESP32-S3 `arduino-cli compile --warnings all`
   - `git diff --check`

호스트/native/컴파일 결과는 실물 BMM350, 보드 플래시, 프로펠러 구동 또는 비행
검증으로 확대 해석하지 않는다.
