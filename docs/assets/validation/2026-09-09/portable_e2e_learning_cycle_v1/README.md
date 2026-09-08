# Portable E2E — GPU 0 전체 모델 6회 학습·평가 완료

<!-- HH_260906 - Prioritize a bounded measured CARLA-data training experiment while separating development findings from deployment qualification. -->

2026-09-09 사용자의 학습 우선 요청에 따라, 추가 설치 없이 기존 개인 venv와 GPU 0을
사용해 **00:36:22–00:58:53 KST에 실제 전체 모델 학습 6회·평가 6회·감사 6회를 완료했습니다.**
총 9,240 optimizer step입니다. [01 가속도 입력 A/B](01_no_accel_development_ab/README.md)에
실제 학습 곡선·예측 경로 PNG 72장·세 seed의 결과·원본 해시·로컬 코드 검증을 분류했습니다.
가속도 입력 제외 모델은 상대 비교 1/3만 통과하고 절대 품질에 미달해 **미채택**입니다.
이 캠페인의 학습 프로세스는 완료 후 종료됐으며, 지금도 계속 학습 중이라는 뜻은 아닙니다.

이전 [2026-09-08 결과](../../2026-09-08/portable_e2e_learning_cycle_v1/README.md)는 그대로 보존합니다.
원격은 학습·평가, 로컬은 코드 검증·결과 분석을 담당합니다. 이번 캠페인은 CARLA나
Autoware를 실제로 실행하거나 차량을 제어하지 않습니다.

| 카테고리 | 작업 | 상태 |
|---|---|---|
| [01 가속도 입력의 사용/제외 비교 학습](01_no_accel_development_ab/README.md) | 기존 physical-v1과 no-accel 모델을 같은 데이터·3개 seed로 각각 새로 학습 | 18/18 실행 완료; 상대·절대 품질 FAIL, 미채택 |

기존 개인 venv만 사용했고 설치·GPU 1 사용·다른 작업 종료·재부팅은 하지 않았습니다.
로컬은 **4,884 passed / 6 skipped / 0 failed**로 코드 회귀를 확인했습니다.
9월 7일 캠페인 전체 학습 15회와 이번 6회를 합친 후속 캠페인 실적은 21회이며,
9월 8일 선택기 전용 9회는 별도로 셉니다. learned closed-loop 기능 통과는 여전히 0개입니다.
