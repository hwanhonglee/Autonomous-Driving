# 9월 16일 — Town03 직진 DEV 수집·변환 완료

<!-- HH_260906 - Add a resumable partial-work index without rewriting prior sealed results. -->

[최신: Town03 직진 DEV 경로 분리·수집·변환·실제 화면](02_dev_route_and_capture/README.md)

기존 DEV와 가까운 p04를 분석하고, 사전 고정 규칙으로 8개 후보 중 p07을 선택했다.
기존 Town03 DEV와 최소 269.636 m, 공유 road/section 없음. 로컬 CARLA 전문가 주행 1회를
완료하고 32.7초·328개 VAL 표본을 Common10으로 변환했다. 실제 loader, 영상 해시,
원본에서 재계산한 미래 20,992점 검증을 통과했다. TRAIN 0 / VAL 328 / TEST 0이다.

관련 회귀 **311 passed**. 차량 중심 6카메라 PNG 6장, 실제 기록 요약 GIF와 경로 비교 PNG를
02 폴더에 정리했다. 전체 원본의 출발·정리 제동 위반도 삭제하지 않고 공개한다.
선택 정속 구간만 기존 9개 필요조건 위반 0이며, 전체 제동 문제 해결을 뜻하지 않는다.

**이번에는 학습·SSH·설치·모델 채택을 하지 않았다.** 영상은 BasicAgent 전문가 주행이고
Portable 모델/Autoware 직접 주행 성공이 아니다. 기능 승인 **0/30 유지**.
다음은 Town07 새 TRAIN + Town03 새 DEV를 고정한 데이터 버전 구성과 직진 보강 학습 A/B다.

[01 당시 사전검사 기록](01_dev_route_preflight/README.md)은 당시 상태 그대로 보존한다.

[직전 실제 학습과 Town07 데이터 확보 결과](../../2026-09-15/portable_e2e_learning_cycle_v1/CURRENT.md)는
그대로 유지한다. Town03 데이터 수집·변환 완료와 모델 직접 운전 성공을 구분한다.
