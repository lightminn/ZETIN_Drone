# 문서

## 유지 문서

- [프로젝트 개요](project_overview.md)
- [펌웨어 카탈로그](firmware_catalog.md)
- [UDP 프로토콜](udp_protocol.md)
- [전원 인가 벤치 절차 (첫 호버 전 안전 관문)](power_on_bench_procedure.md)
- [지상국 링크 요구사항과 RC 타임아웃 진단](ground_station_link.md)
- [단일 PID 대비 캐스케이드(듀얼) PID의 이점](cascade_vs_single_pid.typ) (Typst → PDF)
- [BMM350 yaw 융합 벤치 시험](bmm350_yaw_bench_test.md)
- [자동착륙 착지 감지 연구](failsafe_land_research.md)
- [노션 문서 컨벤션](notion_doc_convention.md)
- [Oracle 재사용 웹 호스팅 운영 가이드](oracle_web_hosting.md)
- [저장소 마이그레이션 맵](migration_map.md)

## 발표자료

- [발표자료 디렉터리 안내](presentations/README.md) — AI 창업캠프 드론 기술 교안과
  10분 요약본. 편집 원본은 HTML이고 PDF·PPTX는 배포 시점 산출물이다.
- 주장과 저장소 근거의 대응은 각 덱의 `SOURCES.md`가 갖는다. 수치의 정답은
  항상 `firmware/`와 이 디렉터리의 규격 문서다.

## 설계·계획 기록

날짜가 박힌 기록이다. 사후 수정하지 않고, 틀린 것으로 밝혀지면 정정을 병기한다.

- [`superpowers/specs/`](superpowers/) — 기능별 설계 문서 (자동착륙, yaw 각속도
  명령, 자기계 캘리브레이션, Tier 1 관측성, 발표자료·웹 배포 등)
- [`superpowers/plans/`](superpowers/) — 그 설계의 구현 계획
- [저장소 정리 설계](design/2026-07-13-repository-cleanup-design.md) ·
  [구현 계획](plans/2026-07-13-repository-cleanup.md)

## 과거 문서

- [2026-05-14 듀얼 IMU PID 설계](history/2026-05-14-dual-imu-pid-design.md)
- [2026-05-14 듀얼 IMU PID 구현 계획](history/2026-05-14-dual-imu-pid-implementation-plan.md)

## 보관 문서

- [PID/DShot 발표용 코드 조각](archive/pid_dshot_presentation_snippet.c)
- [과거 커밋 메시지 스냅샷](archive/commit_message_snapshot.txt)
