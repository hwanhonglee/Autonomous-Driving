# 3000회 균형 재학습: 개선 확인, 승인 보류

저속 직진 데이터와 기존 회전·직진 데이터를 함께 학습했습니다. 기존 자료의 raw 손실은 개선됐지만, 사전 기준 10개 중 **7개 통과 / 3개 미달**입니다. 최종 3000회 가중치만 비교했으며 좋은 중간 결과를 골라내지 않았습니다.

정식 기능 승인 **0/30**. 이번 자료는 연구용 학습 진단이며, 모든 맵 주행·회전 성공·실차 사용을 입증하지 않습니다. 지도 모델은 이번 주행 모델 학습에 포함되지 않았습니다.

## 무엇이 달라졌나

| 지표 | 부모 raw200 | 최종 mixed3000 | 읽는 방법 |
|---|---:|---:|---|
| 저속 경로 평균 오차 ADE (m) | 0.202943 | 0.104327 | 같은 327개 TRAIN 재평가, 작을수록 좋음 |
| 저속 6.4초 끝점 오차 FDE (m) | 0.516862 | 0.257479 | 같은 327개 TRAIN 재평가, 작을수록 좋음 |
| 저속 순간 속도 오차 MAE (m/s) | 0.015724 | 0.024229 | 같은 327개 TRAIN 재평가, 작을수록 좋음 |
| 기존 known DEV 8개 raw 합성 손실 | 0.276283 | 0.226199 | 이전에 여러 번 본 진단 표본, 독립 검증 아님 |

속도 오차는 부모보다 커졌지만 절대 기준은 통과했습니다. 손실 감소 비율을 실제 주행 성능 개선율로 해석하면 안 됩니다.

## 사전에 고정한 기준 10개 — 모두 공개

| 기준 | 실제 최종 값 | 허용 상한 | 결과 |
|---|---:|---:|---|
| V4_TRAIN_ADE | 0.104327358 | 0.100000000 | FAIL |
| V4_TRAIN_FDE | 0.257479293 | 0.250000000 | FAIL |
| V4_TRAIN_instantaneous_speed_MAE | 0.024229174 | 0.030000000 | PASS |
| known_DEV8_raw_total | 0.226199019 | 0.303911000 | PASS |
| known_DEV599/aggregate/raw_curvature | 0.021829126 | 0.032438122 | PASS |
| known_DEV599/aggregate/raw_preferred_speed | 0.135705923 | 0.197974446 | PASS |
| known_DEV599/carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9/raw_curvature | 0.045265452 | 0.058443417 | PASS |
| known_DEV599/carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9/raw_preferred_speed | 0.230027621 | 0.409045734 | PASS |
| known_DEV599/carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320/raw_curvature | 0.002465575 | 0.010952039 | PASS |
| known_DEV599/carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320/raw_preferred_speed | 0.057775496 | 0.023583230 | FAIL |

Town03 직진 raw-speed 손실은 악화되어 보존 기준에 미달했습니다. 전체 평균 개선만으로 이 실패를 숨기거나 승인하지 않습니다.

## 학습량·데이터·계보

- 부모: raw200 → 새 AdamW로 3000회 추가. 알려진 누적 업데이트는 3200회입니다.
- 9개 TRAIN 주행 기록 / 3개 맵: Town07, Town01, C_track. 가용 3114개 중 저속 327개와 적격 기존 2487개를 사용했습니다.
- 배치 4개: 저속 joint 2 + 실제 곡률 회전 raw 1 + 직진 raw 1. 실제 12000회 노출: 저속 6000, 기존 회전 3000, 기존 직진 3000.
- 저속만 실제 v0·실제 미래 궤적과 decoder를 함께 학습했습니다. 기존 데이터의 최대 약 8.425 m/s 속도는 잘라내지 않고 raw-only로 유지했습니다.
- known DEV는 665개 중 고정된 이동 599개를 전후 평가했습니다. 이미 반복 사용한 자료이며 새 독립 DEV는 없습니다. TEST를 학습·선택에 사용하지 않았습니다.
- STOP/기타 기능 head를 새로 학습하지 않았습니다. GO는 저속 cruise 시나리오의 명시적 조건입니다.

실제 GPU 소유 구간 939.5초 (15.66분): baseline 포함 fit 685.2초, optimizer loop 437.4초, 최종 진단 254.3초.

## 결과 파일

- [전후 전체 비교](raw_comparisons.md): known DEV599 전체·각 주행 기록과 기존 TRAIN 회전/직진·각 기록의 raw 성분을 전부 공개합니다.
- [결과·계보 JSON](result.json), [원자료 SHA 목록](evidence_manifest.json), [숫자 전용 loader 검사](numeric_loader_receipt.json).

숫자 loader: `PASS_NUMERIC_CPU_ONLY_NOT_CONTROL`. 이는 CPU float32 가중치 파일 검사만 의미합니다. simulator/실차 제어 승인, 주행 성공, 전체 10 Hz 증명이 아닙니다.

원자료: `storage/experiments/training/2026-10-01/temporal_v2_balanced_replay_v1`

다음은 미달한 Town03 직진 속도와 저속 ADE/FDE의 실제 출력을 분석해, 독립 자료·폐루프 기준을 사전에 고정한 개선을 검토하는 것입니다. 이번 보고서는 추가 학습이나 물리 주행을 시작하지 않습니다.
