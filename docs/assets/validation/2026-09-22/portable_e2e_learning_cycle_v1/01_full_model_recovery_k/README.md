# K0/K1 전체 모델 학습·최종 평가

<!-- HH_260906 - Keep training completion distinct from driving approval. -->

상태: `FAIL_K_RECOVERY_SIGNAL_NOT_PROMOTED`

두 모델 각각 15,000 step 전체 학습 후 같은 DEV 665개로 평가했다. TEST·실차·폐루프 제어는 수행하지 않았다.

| 범위 | 지표 | K0 | K1 | K1−K0 |
| --- | --- | ---: | ---: | ---: |
| town03_right_turn_dev | ade_6p4s_m | 3.270103 | 3.832534 | +0.562431 |
| town03_right_turn_dev | fde_6p4s_m | 8.914888 | 11.423511 | +2.508622 |
| town03_right_turn_dev | speed_mae_6p4s_mps | 1.393862 | 1.746087 | +0.352226 |
| town03_straight_dev | ade_6p4s_m | 2.696427 | 0.511270 | -2.185156 |
| town03_straight_dev | fde_6p4s_m | 8.725562 | 1.170265 | -7.555297 |
| town03_straight_dev | speed_mae_6p4s_mps | 1.524892 | 0.257303 | -1.267589 |
| combined_val | ade_6p4s_m | 2.987147 | 2.194377 | -0.792770 |
| combined_val | fde_6p4s_m | 8.821506 | 6.366270 | -2.455236 |
| combined_val | speed_mae_6p4s_mps | 1.458490 | 1.011770 | -0.446721 |

낮을수록 좋다. ADE/FDE는 m, 속도 MAE는 m/s다.

| 모델 | 고정 지표 통과 | 형상 기준 통과 | 학습 시간(분) | 첫/마지막 epoch TRAIN loss |
| --- | ---: | ---: | ---: | ---: |
| K0_current1475 | 6/9 | 3/3 | 29.7 | 2.7575 / 0.8605 |
| K1_current_plus_successful4 | 6/9 | 3/3 | 29.6 | 2.3904 / 0.7533 |

K1의 K0 대비 비퇴행: 6/9.

이는 반복 관측한 DEV의 연구 결과다. 통과하더라도 새로운 경로·seed·정지 보존·폐루프 검증 전에는 배포하지 않는다.

증거 검증: 소스·계획·체크포인트·평가 SHA, 두 모델 동일 초기 가중치, 20개 epoch/각 15,000 step, 독립 기준 재계산.

![Measured training losses](training_loss.png)

![Final DEV metrics](final_dev_comparison.png)

## 후속 원인 분석 — 후보 생성과 선택을 분리

<!-- HH_260906 - Keep future-label oracle diagnostics separate from deployable model decisions. -->

두 모델의 최종 checkpoint를 고정한 **두 번째, 사후 진단용 DEV 재생**을 별도로 수행했다.
이는 원래 1회 최종 평가를 대체하지 않으며, 학습·가중치 변경·기준 조정은 없었다.
재생된 selected/oracle ADE가 원래 평가와 `1e-5 m` 이내로 일치했다.

| Town03 우회전 episode 지표 | K0 | K1 |
| --- | ---: | ---: |
| 모델이 선택한 후보의 평균 ADE | 3.270103 m | 3.832534 m |
| 정답을 보고 고른 최선 후보의 평균 ADE | 1.424836 m | 1.299094 m |
| 선택에 따른 평균 추가 ADE | 1.845266 m | 2.533439 m |
| 선택 후보가 최선 후보와 같은 표본 | 122/337 | 116/337 |
| 선택에 따른 추가 ADE가 1 m 초과인 표본 | 155/337 | 200/337 |

직진 episode의 K1은 328/328에서 최선 후보와 같은 후보를 선택했고 ADE는 0.511270 m다.
따라서 **이번 데이터 확장은 직진에 강한 개선을 보였지만 우회전 episode의 후보 선택은 악화**했다.
전체 후보 중 최선의 우회전 경로가 개선되었다는 사실은 선택 문제를 우선 분석할 근거다.
후보 생성기가 완성되었다는 뜻은 아니며, 최선 후보에도 평균 1.299 m의 오차가 남는다.

`oracle`은 미래 정답을 보고 고르는 분석용 하한이다. 실제 차량에서는 미래 정답을 알 수 없으므로
oracle 수치를 현재 모델의 운용 성능으로 보고하거나 oracle로 차량을 제어하지 않는다.

## 경로 그림

차량의 현재 위치를 모든 패널 중심에 고정했다. 같은 episode index 0/164/327을 K0/K1에 사용했다.
빨강은 선택 경로, 초록은 정답을 보고 고른 후보, 검정은 기록된 미래 정답, 회색은 navigation이다.
episode 이름이 우회전이어도 출발·진입 직진·회전·회전 후 구간을 포함한다. 아래 그림은
**고정 위치의 오프라인 예측 비교이며 CARLA/Autoware 폐루프 주행 화면이 아니다.**

![Ego-centered right-episode diagnostics](diagnostics/right_trajectory_diagnostics.png)

![Ego-centered straight-episode diagnostics](diagnostics/straight_trajectory_diagnostics.png)

## 다음 개선의 범위

1. 신규 ACC·회피·차선변경 head와 NuRec 혼합은 보류한다.
2. 기존 후보 선택 손실이 TRAIN의 우회전·속도·정지/이동 상태를 어떻게 구분하는지 검사한다.
   현재 관측만으로 데이터 불균형, 손실 설계 또는 표현 부족 중 하나를 확정하지 않는다.
3. 직진 4개 추가의 효과를 유지하면서 회전 데이터의 물리 경로 다양성을 늘린다. 현재 회전 TRAIN은
   좌회전 1개 episode·우회전 1개 episode여서 표본 반복만으로 독립 상황 수가 늘지 않는다.
4. 데이터/선택 손실 중 검증된 변경 하나를 고정하고 **기존 모델 전체를 다시 학습**한다.
   작은 신규 기능 head를 추가한 결과로 기본 성능 회복을 대신하지 않는다.
5. 직진 개선과 우회전 비퇴행을 함께 통과한 뒤 신규 seed·독립 경로·정지 유지 검증으로 넘어간다.

실행 시간: 2026-09-22 02:38:52–03:38:57 KST, 약 60분. 두 전체 학습 29.7분/29.6분과
최종 평가를 완료했다. 그 뒤 별도 사후 재생·원본 결과 동일성 검사·PNG 작성을 수행했다.
자체 모델의 폐루프 승인 기능은 **여전히 0/30**이고 두 checkpoint는 모두 미채택이다.

원본 결과: `artifacts/training/2026-09-22/moving_recovery_k_v1`.
사후 진단: `artifacts/training/2026-09-22/moving_recovery_k_posthoc_selection_v1`.
