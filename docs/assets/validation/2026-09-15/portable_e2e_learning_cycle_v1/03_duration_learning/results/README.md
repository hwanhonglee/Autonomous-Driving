# 20 epoch 실제 이어학습 결과 — 6개 모델 전체 비교

<!-- HH_260906 - Separate completed optimization from model quality, function acceptance, and deployment. -->

실행·증거 검산 완료. **모델 채택·차량 제어 승인 아님.**

- 새 optimizer 갱신 25,200회, 새 표본 노출 100,710회. 모델마다 누적 20 epoch / 5,740 step.
- 90개 학습 종점·282개 단계 완료. TRAIN 3 episode / 1,147표본, 개발 VAL 1 episode / 337표본 그대로.
- 새로 편입한 데이터 0 episode. 반복 노출을 독립 장면 수로 계산하지 않는다.
- 최종 네 가지 경로 오차 목표를 모두 만족한 모델: 0/6. 기능 통과나 안전성 보장이 아니다.
- 모델 직접 주행 통과 0/30, TEST 미사용, 실차 미승인. 아래 그림은 오프라인 예측/분석이다.

## 부모 → 10 → 20 epoch

경로 오차 단위 m. 변화가 음수면 개선, 양수면 악화. 모든 seed를 유지한다.

| seed / 손실 | 부모 ADE 6.4s | epoch 10 | epoch 20 | 부모 대비 변화 | 최종 ADE 1s / 3s | 최종 FDE 6.4s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| seed_20260903/A_original_loss | 6.2177 | 5.4057 | 4.5743 | -1.6433 | 0.3161 / 1.5955 | 10.3018 |
| seed_20260903/B_cost_aware | 3.6452 | 4.4754 | 3.8871 | +0.2419 | 0.1872 / 1.1184 | 10.0617 |
| seed_20260904/A_original_loss | 5.1648 | 5.2058 | 4.0958 | -1.0690 | 0.2655 / 1.4336 | 9.5185 |
| seed_20260904/B_cost_aware | 5.2187 | 4.6085 | 3.2394 | -1.9793 | 0.1885 / 0.9219 | 8.5333 |
| seed_20260905/A_original_loss | 4.9157 | 4.8423 | 4.0146 | -0.9011 | 0.2354 / 1.3859 | 8.7952 |
| seed_20260905/B_cost_aware | 4.7978 | 3.8293 | 2.7490 | -2.0488 | 0.1671 / 0.7593 | 7.2355 |

목표: ADE 1s ≤0.5m, 3s ≤1m, 6.4s ≤2m; FDE 6.4s ≤4m. 반복 확인한 VAL은 개발용이다.

## 전체 학습 곡선

![Seed 20260903 fixed VAL curves](val_learning_curves_20260903.png)

![Seed 20260904 fixed VAL curves](val_learning_curves_20260904.png)

![Seed 20260905 fixed VAL curves](val_learning_curves_20260905.png)

![Changing-model TRAIN batch losses](train_batch_learning_curves.png)

TRAIN 곡선은 가중치가 변하는 batch 손실이다. A/B 목적함수가 달라 총 손실 높이를 직접 비교하지 않는다.

## 행동별 분석과 경로 그림

[모든 행동별 오차](behavior_metrics.md) · [정지·재가속 선택 진단](behavior_decisions.md). VAL 정지 유지 표본은 0개이며 정지 성능을 검증하지 못한다.

- [Seed 20260903: 고정 12장면 × A/B × epoch 10/20](galleries/seed_20260903.md)
- [Seed 20260904: 고정 12장면 × A/B × epoch 10/20](galleries/seed_20260904.md)
- [Seed 20260905: 고정 12장면 × A/B × epoch 10/20](galleries/seed_20260905.md)

[전체 곡선 원본](learning_curves.json) · [독립 실행 검산](verification.json).

이 결과만으로 모든 맵 일반화·VAD/METEOR 동급 성능·실제 차량 사용 가능성을 주장하지 않는다.
