# 차선 복귀 학습 결과 — 신규 V4 자료 / V3 주행 모델

사전에 정한 48개 진단 중 **20개 통과 / 28개 미달**입니다. 미달 결과도 모두 아래에 남겼습니다. 최종 3000회 가중치만 비교했으며 중간 체크포인트를 골라내지 않았습니다.

정식 주기능 승인 **0/9**, 하위 기능 승인 **0/30**. 이 보고서는 모델 학습·숫자 검증 결과이며 자율주행 성공이나 실차 사용 승인이 아닙니다. 실제 3×45초 폐루프 주행은 별도 실행과 결과로 확인해야 합니다.

## 이번에 바꾼 부분

기존 주행 모델의 97개 tensor는 원본과 바이트까지 동일하게 동결했습니다. 카메라와 실제 현재 위치 기준의 경로를 사용하는 작은 차선 복귀 branch 4개 tensor만 학습했습니다. 새 기능 전체를 완성한 것이 아니라, 경로에서 벗어났을 때 원래 경로 쪽으로 조향할 수 있도록 경로의 좌우 정보 전달을 보강한 단계입니다.

- 부모는 mixed3000이며 사전 진단 7/10 통과 상태였습니다. 그 상태를 성공 모델로 바꾸어 기록하지 않았습니다.
- 새 branch 3000회 추가 → 알려진 누적 업데이트 6200회. 본격 전체 기능 학습 완료가 아닙니다.
- 18개 TRAIN 기록: 새 실제 회복 9개 + 기존 저속 1개 + 기존 직진·회전 8개. 새 HOLDOUT 2개는 학습·체크포인트 선택에 사용하지 않았습니다.
- 실제 12000회 노출: 회복 6000, 기존 저속 3000, 기존 직진·회전 3000. 배치 4개는 실제 비중앙 상태 1 + 전체 회복 자료 1 + 기존 저속 1 + 기존 회전/직진 번갈아 1입니다.
- 새 회복 자료는 주행 상대시간 **4.0~34.1초**의 302개×9 기록입니다. 초기 정지·출발 회복은 이번 새 자료의 학습 범위가 아닙니다. 원시 startup은 지우지 않고 보존했습니다.
- 신규 HOLDOUT은 같은 Town07 경로의 다른 실제 위치·각도·seed입니다. 다른 모든 맵이나 상황에 대한 일반화 증거가 아닙니다.
- 기존 known DEV 665개 중 고정된 이동 599개를 진단했습니다. 이전부터 사용한 자료이므로 독립 검증으로 부르지 않습니다.
- 속도·STOP head와 원래 backbone은 동결되어 있습니다. 속도 지표도 그대로 보고하며 절대 속도 추종 성공을 주장하지 않습니다. 지도 모델은 이번 학습에 포함하지 않았습니다.

## 사전에 고정한 48개 판정 — 실패 포함 전부

| 기준 | 최종 실제 값 | 허용 상한 | 결과 |
|---|---:|---:|---|
| recovery_train/carla_town07_town07_signed_recovery_train_00_ClearNoon_s2026100710_4847458ce66b/ADE | 0.646399062 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_00_ClearNoon_s2026100710_4847458ce66b/FDE | 1.191190126 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_00_ClearNoon_s2026100710_4847458ce66b/speed_MAE_parent_regression | 0.213993791 | 0.223993791 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_01_ClearNoon_s2026100711_1d087d0ec5d1/ADE | 0.645351324 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_01_ClearNoon_s2026100711_1d087d0ec5d1/FDE | 1.189250808 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_01_ClearNoon_s2026100711_1d087d0ec5d1/speed_MAE_parent_regression | 0.213721189 | 0.223721189 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_02_ClearNoon_s2026100712_4c59f3872ced/ADE | 0.646594136 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_02_ClearNoon_s2026100712_4c59f3872ced/FDE | 1.190304002 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_02_ClearNoon_s2026100712_4c59f3872ced/speed_MAE_parent_regression | 0.214037712 | 0.224037712 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_03_ClearNoon_s2026100713_ee072fdeaef0/ADE | 0.641889260 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_03_ClearNoon_s2026100713_ee072fdeaef0/FDE | 1.182847418 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_03_ClearNoon_s2026100713_ee072fdeaef0/speed_MAE_parent_regression | 0.212886503 | 0.222886503 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_04_ClearNoon_s2026100714_f421d97cfecb/ADE | 0.641389363 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_04_ClearNoon_s2026100714_f421d97cfecb/FDE | 1.181482674 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_04_ClearNoon_s2026100714_f421d97cfecb/speed_MAE_parent_regression | 0.212770021 | 0.222770021 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_05_ClearNoon_s2026100715_3c461aebc905/ADE | 0.636995439 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_05_ClearNoon_s2026100715_3c461aebc905/FDE | 1.174628407 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_05_ClearNoon_s2026100715_3c461aebc905/speed_MAE_parent_regression | 0.211681196 | 0.221681196 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_06_ClearNoon_s2026100716_105072a8936e/ADE | 0.641573090 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_06_ClearNoon_s2026100716_105072a8936e/FDE | 1.181169120 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_06_ClearNoon_s2026100716_105072a8936e/speed_MAE_parent_regression | 0.212805465 | 0.222805465 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_07_ClearNoon_s2026100717_b88900f9bd75/ADE | 0.646136690 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_07_ClearNoon_s2026100717_b88900f9bd75/FDE | 1.188022097 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_07_ClearNoon_s2026100717_b88900f9bd75/speed_MAE_parent_regression | 0.213903204 | 0.223903204 | PASS |
| recovery_train/carla_town07_town07_signed_recovery_train_08_ClearNoon_s2026100718_b264a23159f2/ADE | 0.641853924 | 0.100000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_08_ClearNoon_s2026100718_b264a23159f2/FDE | 1.180972344 | 0.250000000 | FAIL |
| recovery_train/carla_town07_town07_signed_recovery_train_08_ClearNoon_s2026100718_b264a23159f2/speed_MAE_parent_regression | 0.212816053 | 0.222816053 | PASS |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_00_ClearNoon_s2026100810_668ab812450f/ADE | 0.640457259 | 0.100000000 | FAIL |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_00_ClearNoon_s2026100810_668ab812450f/FDE | 1.180878783 | 0.250000000 | FAIL |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_00_ClearNoon_s2026100810_668ab812450f/speed_MAE_parent_regression | 0.212508964 | 0.222508964 | PASS |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_01_ClearNoon_s2026100811_7696127af0ae/ADE | 0.642008277 | 0.100000000 | FAIL |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_01_ClearNoon_s2026100811_7696127af0ae/FDE | 1.182049963 | 0.250000000 | FAIL |
| recovery_holdout/carla_town07_town07_signed_recovery_holdout_01_ClearNoon_s2026100811_7696127af0ae/speed_MAE_parent_regression | 0.212898468 | 0.222898468 | PASS |
| oldV4_TRAIN/mean_ade_6p4s_m | 0.078347913 | 0.114860066 | PASS |
| oldV4_TRAIN/mean_fde_6p4s_m | 0.176098733 | 0.283327126 | PASS |
| oldV4_TRAIN/mean_instantaneous_speed_mae_mps | 0.024229169 | 0.026752086 | PASS |
| known599/aggregate/raw_curvature | 0.023550020 | 0.024112039 | PASS |
| known599/carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9/raw_curvature | 0.048576117 | 0.049892001 | PASS |
| known599/carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320/raw_curvature | 0.002872971 | 0.002812132 | FAIL |
| legacy/aggregate/raw_curvature | 0.009454073 | 0.012044579 | PASS |
| legacy/carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7/raw_curvature | 0.031288370 | 0.040922853 | PASS |
| legacy/carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633/raw_curvature | 0.001085057 | 0.000798806 | FAIL |
| legacy/carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb/raw_curvature | 0.000999798 | 0.000856444 | FAIL |
| legacy/carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f/raw_curvature | 0.029974740 | 0.041750342 | PASS |
| legacy/carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646/raw_curvature | 0.001299511 | 0.001275135 | FAIL |
| legacy/carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a/raw_curvature | 0.012429700 | 0.014749122 | PASS |
| legacy/carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240/raw_curvature | 0.002216206 | 0.001991269 | FAIL |
| legacy/carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2/raw_curvature | 0.002109656 | 0.001571550 | FAIL |

## 파일과 해석

[전체 전후 비교](comparisons.md)에 새 TRAIN 9개·HOLDOUT 2개·기존 저속 327개와 기존 raw 평가 전체·각 기록·직진/회전을 공개했습니다.

[결과·계보 JSON](result.json), [원자료 SHA](evidence_manifest.json), [숫자 전용 검증](numeric_loader_receipt.json).

숫자 검사: `PASS_FULL101_NUMERIC_CPU_ONLY_NOT_CONTROL`. 모든 6개 export의 101개 tensor와 원본97 바이트를 검산했습니다. Torch `.pt`는 SHA만 확인하고 역직렬화하지 않았습니다. 숫자 검사 PASS는 simulator/실차 제어 승인이나 전체 10Hz 성공을 뜻하지 않습니다.

실제 GPU 소유 1821.8초: baseline 포함 fit 1281.7초, 최종 평가 540.1초. fit 예산은 2700초이며 최종 평가 시간은 별도입니다.

다음 확인은 최종 고정 가중치의 실제 폐루프 차선 복귀·90m/45초·반복 재현성입니다. 이 발행자는 추가 학습이나 물리 주행을 시작하지 않습니다.

원자료: `/home/a/autoware_e2e/storage/experiments/training/2026-10-01/temporal_route_recovery_v3_v1`
