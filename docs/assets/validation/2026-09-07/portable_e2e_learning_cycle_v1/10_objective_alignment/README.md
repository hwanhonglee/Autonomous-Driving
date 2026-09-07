# C/D CPU 학습 목표·평가 목표 일치 진단

<!-- HH_260906 - Publish verified aggregates without pooling train and validation or exposing per-sample records. -->

C는 score weight 0.1, D는 0.5입니다. 각각 3개 seed, 동일 v3 데이터의 train 1,147개와 val 337개를 CPU에서 따로 분석했습니다. 아래 집계는 원래 GPU 평가를 대체하지 않습니다.
Train은 학습에 사용한 자료의 진단이지 일반화 성능이 아닙니다. Val은 개발 검증이며 Town04 held-out test는 열지 않았습니다.

![학습·검증 분리 일치율](01_train_val_objective_agreement.png)

| Seed | Arm | Split | N | 선택↔loss oracle | 선택↔ADE oracle | loss↔ADE oracle | Selected ADE m | Composite ADE m | ADE oracle m |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 20260903 | C_expanded_data | train | 1147 | 65.13% | 71.75% | 91.37% | 2.99039 | 1.05927 | 0.99295 |
| 20260903 | C_expanded_data | val | 337 | 65.88% | 68.84% | 86.35% | 3.87433 | 1.64756 | 1.55218 |
| 20260903 | D_selector_weight | train | 1147 | 72.89% | 75.07% | 94.86% | 2.03810 | 1.04440 | 1.02423 |
| 20260903 | D_selector_weight | val | 337 | 56.08% | 51.93% | 85.46% | 4.30488 | 1.67786 | 1.59071 |
| 20260904 | C_expanded_data | train | 1147 | 67.22% | 67.48% | 91.28% | 2.62738 | 1.02560 | 0.98986 |
| 20260904 | C_expanded_data | val | 337 | 50.15% | 49.55% | 92.88% | 6.25854 | 1.51667 | 1.48367 |
| 20260904 | D_selector_weight | train | 1147 | 72.97% | 73.41% | 95.12% | 2.15028 | 1.06673 | 1.04031 |
| 20260904 | D_selector_weight | val | 337 | 59.35% | 50.15% | 83.38% | 5.35538 | 1.97837 | 1.87572 |
| 20260905 | C_expanded_data | train | 1147 | 63.82% | 70.10% | 90.93% | 3.30160 | 1.02708 | 0.99017 |
| 20260905 | C_expanded_data | val | 337 | 48.37% | 51.63% | 82.49% | 4.78955 | 1.64040 | 1.55011 |
| 20260905 | D_selector_weight | train | 1147 | 74.89% | 76.90% | 94.94% | 2.21760 | 1.04027 | 1.01626 |
| 20260905 | D_selector_weight | val | 337 | 49.26% | 48.37% | 91.10% | 3.82863 | 1.79261 | 1.74830 |

## CPU 재분석과 원래 GPU val 평가

같은 checkpoint라도 아래 원본 GPU 평가 수치는 그대로 보존합니다. histogram 불일치는 선택 결과가 달라졌음을 보여주지만 near-tie나 backend가 원인인지는 저장된 paired logits·margin이 없어 확정하지 않습니다.

| Seed | Arm | CPU val ADE m | 원래 GPU val ADE m | CPU−GPU m | 선택 histogram 동일 |
| --- | --- | ---: | ---: | ---: | --- |
| 20260903 | C_expanded_data | 3.87433402 | 3.87432761 | +0.00000641 | YES |
| 20260903 | D_selector_weight | 4.30488312 | 4.30487240 | +0.00001073 | YES |
| 20260904 | C_expanded_data | 6.25853936 | 6.25854536 | -0.00000600 | YES |
| 20260904 | D_selector_weight | 5.35538210 | 5.33699542 | +0.01838668 | NO |
| 20260905 | C_expanded_data | 4.78954685 | 4.78954706 | -0.00000021 | YES |
| 20260905 | D_selector_weight | 3.82862601 | 3.82862185 | +0.00000416 | YES |

원래 학습 loss의 최적 후보(composite oracle)와 XY ADE만 최소인 후보(ADE oracle)는 정의가 다릅니다.
공개 JSON은 aggregate-only 치환본이며 `per_sample` 8,904개를 생략했습니다. 원본은 삭제하지 않았고 각 공개 파일에 원본 SHA를 남겼습니다.
히스토그램·혼동행렬·일치율·평균을 원본 sample별 기록으로 재계산했지만 이 발행 단계에서 모델이나 loss 텐서를 다시 실행하지는 않았습니다.
평균이나 train/val을 합쳐 하나의 성능 점수로 만들지 않습니다. CPU 진단 시간은 10 Hz 추론 성능 또는 차량 안전의 증거가 아닙니다.

- [검증 집계](summary.json)
- [원본·공개본 해시](publication_manifest.json)
- [전체 발행 파일 체크섬](SHA256SUMS)
