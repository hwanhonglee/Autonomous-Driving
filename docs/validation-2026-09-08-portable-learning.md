# 2026-09-08 Portable E2E 학습·수집 개선

<!-- HH_260906 - Link the dated work ledger without presenting an ongoing campaign as deployment completion. -->

[최신 결과 폴더](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/README.md)에
정답 데이터 진단, 생성기를 고정한 선택기 9회 학습, 로컬 자연 정지 수집 개선을 분리해 기록합니다.

원격은 개인 venv·GPU0의 학습/평가 전용, 로컬은 CARLA 데이터 수집·추론/제어 검증 전용입니다.
새 선택기들은 세 seed 전체 기준을 통과하지 못해 승격하지 않았습니다. 학습 모델의
폐루프 주행·실차 제어 승인은 없으며 기존 shadow 모델을 유지합니다.

사용자 요청에 따른 이번 작업 경계는 **2026-09-08 오전 10시 KST**입니다.
실제 완료·실패·진행 상태와 다음 순서는 위 결과 폴더의 기록을 확인합니다.
