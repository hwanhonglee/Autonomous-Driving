# Portable E2E — GPU 0 실제 학습 재개

<!-- HH_260906 - Prioritize a bounded real-data training experiment while separating development findings from deployment qualification. -->

2026-09-09 사용자의 학습 우선 요청에 따라, 추가 설치 없이 기존 개인 venv와 GPU 0을
사용하는 작업을 준비합니다. [01 가속도 입력 A/B](01_no_accel_development_ab/README.md)에
계획과 실제 실행 상태를 구분해 기록합니다. 현재 문서는 실행 준비 기록이며 학습 완료가 아닙니다.

이전 [2026-09-08 결과](../../2026-09-08/portable_e2e_learning_cycle_v1/README.md)는 그대로 보존합니다.
원격은 학습·평가, 로컬은 코드 검증·결과 분석을 담당합니다. 이번 캠페인은 CARLA나
Autoware를 실제로 실행하거나 차량을 제어하지 않습니다.

| 카테고리 | 작업 | 상태 |
|---|---|---|
| [01 가속도 입력의 사용/제외 비교 학습](01_no_accel_development_ab/README.md) | 기존 physical-v1과 no-accel 모델을 같은 데이터·3개 seed로 각각 새로 학습 | 실행 준비; 시작 기록 확인 필요 |
