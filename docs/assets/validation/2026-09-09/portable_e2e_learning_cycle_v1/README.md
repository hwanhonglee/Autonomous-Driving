# Portable E2E — 9월 9일 학습·데이터 품질 실험

<!-- HH_260906 - Prioritize a bounded measured CARLA-data training experiment while separating development findings from deployment qualification. -->

2026-09-09 사용자의 학습 우선 요청에 따라, 추가 설치 없이 기존 개인 venv와 GPU 0을
사용해 **00:36:22–00:58:53 KST에 실제 전체 모델 학습 6회·평가 6회·감사 6회를 완료했습니다.**
총 9,240 optimizer step입니다. [01 가속도 입력 A/B](01_no_accel_development_ab/README.md)에
실제 학습 곡선·예측 경로 PNG 72장·세 seed의 결과·원본 해시·로컬 코드 검증을 분류했습니다.
가속도 입력 제외 모델은 상대 비교 1/3만 통과하고 절대 품질에 미달해 **미채택**입니다.
이 캠페인의 학습 프로세스는 완료 후 종료됐으며, 지금도 계속 학습 중이라는 뜻은 아닙니다.

이어 **02:21:35–02:44 KST에 선택 손실 A/B 전체 학습 6회·평가 6회·감사 6회**를 추가로
완료했습니다. 상대 비교 2/3 통과·절대 기준 미달로 미채택입니다. 같은 시간 로컬 C-track
expert 수집 4회를 계측했고 모두 데이터 미승인으로 보존했습니다. 네 시도의 차량 중심
PNG 30장·GIF 4개, 원격 모델의 오프라인 예측 PNG 72장과 학습 곡선을 02 범주에 정리했습니다.

이전 [2026-09-08 결과](../../2026-09-08/portable_e2e_learning_cycle_v1/README.md)는 그대로 보존합니다.
원격은 학습·평가, 로컬은 코드 검증·CARLA expert 수집·결과 분석을 담당합니다.
01의 학습 캠페인은 CARLA를 실행하지 않았고, 02의 로컬 수집은 별도 expert 제어입니다.
새 Portable 모델이 Autoware 또는 CARLA 차량을 제어한 결과로 표시하지 않습니다.

| 카테고리 | 작업 | 상태 |
|---|---|---|
| [01 가속도 입력의 사용/제외 비교 학습](01_no_accel_development_ab/README.md) | 기존 physical-v1과 no-accel 모델을 같은 데이터·3개 seed로 각각 새로 학습 | 18/18 실행 완료; 상대·절대 품질 FAIL, 미채택 |
| [02 오전 10시까지 후속 학습·수집 품질 비교](02_overnight_data_and_learning/README.md) | GPU 0 선택 손실 A/B와 로컬 C-track 물리 적분 A/B | 02:44 KST 새 학습 18/18 완료·수집 4/4 종료; 모델·데이터 미승인. 다음 모델 준비 중 |

기존 개인 venv만 사용했고 설치·GPU 1 사용·다른 작업 종료·재부팅은 하지 않았습니다.
01 완료 시점의 로컬 전체 회귀는 **4,884 passed / 6 skipped / 0 failed**였습니다.
그 이후 02에서 추가한 코드의 부분 검사는 각 범주에 별도 기록하며, 위 수치를 새 전체 검사라고 표시하지 않습니다.
9월 7일 캠페인 전체 학습 15회와 오늘 완료한 12회를 합친 후속 캠페인 실적은 27회이며,
9월 8일 선택기 전용 9회는 별도로 셉니다. learned closed-loop 기능 통과는 여전히 0개입니다.
