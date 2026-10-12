# I 실패 원인 — geometry가 아니라 causal route intent가 빠졌다

<!-- HH_260906 - Separate post-run root-cause analysis from the sealed I training result. -->

## 결론

I1 실패는 optimizer 불안정이나 8,280-step 부족으로 설명되지 않는다. candidate XY·speed
geometry는 Town03 직진에서 유효했지만, 어느 route와 maneuver를 따라야 하는지 명시적으로
조건화하지 않아 fused context가 Town/episode shortcut으로 남았다.

Town03 straight의 oracle은 candidate 2가 `312/328`이다. F/I0는 candidate 2를 `65`, I1은
`105`만 선택했고 I1도 candidate 5를 `219` 선택했다. 반면 같은 15 km/h straight인 Town07
TRAIN에서는 I1과 oracle이 candidate 2를 `328/328` 선택한다. candidate 2−5 평균 logit은
Town07 `+18.011`, Town03 `-1.993`으로 약 20이 뒤집혔다.

## 학습량 부족이 아닌 근거

- I1 TRAIN DRIVE-oracle agreement: `91.40%`
- I1 DEV DRIVE-oracle agreement: `30.95%`
- I0 TRAIN / DEV: `90.42% / 27.74%`
- I1 optimization loss: epoch 0 `0.2005`, epoch 2 `0.1368`, epoch 19 `0.1304`
- nonfinite 0건, 최대 gradient norm `1.886`, clip 기준 5 초과 0건
- TRAIN 1,475개 중 최종 출력에서 폐기되는 F-STOP 243개에도 DRIVE loss가 적용되어 불필요한
  gradient 간섭이 있었다.

더 오래 같은 데이터를 반복하면 TRAIN agreement만 더 높이고 DEV domain flip을 유지할
가능성이 크다.

## 사후 ablation

아래는 구조 원인 확인용 사후 진단이며 승격 근거가 아니다.

| I1 입력 | 우회전 ADE/FDE/속도 | 직진 ADE/FDE/속도 |
| --- | --- | --- |
| full I1 | 3.321 / 9.201 / 1.406 | 6.272 / 15.383 / 2.453 |
| geometry 제거 | F와 exact 동일 | F와 exact 동일 |
| context 제거 | 4.924 / 15.098 / 2.332 | 0.867 / 2.368 / 0.656 |
| context·prior feature 제거 | 4.792 / 14.545 / 2.237 | 0.665 / 2.087 / 0.608 |

geometry만 사용하면 직진 후보는 알아보지만 우회전 route intent를 잃는다. fused context를
다시 넣으면 우회전은 일부 회복하는 대신 Town shortcut으로 직진이 다시 나빠진다.

## label-free causal-route probe

exact prepared corpus의 `samples.jsonl`과 H cache sample ID `2,140/2,140`을 일치시켜 다음
입력만 사용했다.

- `navigation.route_polyline_base_m`
- 현재 `ego.linear_velocity_base_mps` norm
- frozen `candidate_xy`
- 원래 F DRIVE/STOP mode

label, target, 미래 ego, image는 score 입력에 사용하지 않았다. `t=0.1…6.4 s`에서
`arc=current_speed×t`로 route를 보간하고 후보와 평균 L2가 가장 작은 DRIVE 후보를 골랐다.
threshold, fallback, DEV 계수 튜닝은 없었다.

| 진단 | 우회전 | 직진 | combined |
| --- | --- | --- | --- |
| constant-speed route matching | 3.150 / 8.348 / 1.314 | **0.537 / 1.675 / 0.518** | **1.861 / 5.057 / 0.922** |

직진은 candidate 2를 `319/328` 선택해 oracle ADE 수준까지 복구했지만 우회전 C gate는
실패했다. 즉 causal route 신호는 필요하고 유효하지만 이 단순 규칙 자체는 모델 후보가 아니다.
Town03 DEV를 이미 본 사후 진단이므로 독립 confirmation도 아니다.

## 다음 J 계약

1. exact raw sample과 H cache를 sample ID·manifest·fingerprint로 결합한 input-only sidecar를
   만든다.
2. model과 같은 방식의 causal `route_xy/route_mask`, 현재까지의 ego speed/acceleration trend,
   candidate XY/speed만 저장한다.
3. target, label, 미래 ego, image, fused, map/episode ID는 scorer 입력에서 금지한다.
4. original F logit은 ranking feature가 아니라 DRIVE/STOP mode와 DRIVE-group max 보존에만 쓴다.
5. F-original DRIVE 1,232행만 학습하고 STOP 243행은 loss에서 제외하며 byte-exact 보존한다.
6. candidate-index별 별도 parameter가 없는 작은 shared cost scorer와 비음수 가중치를 사용한다.
7. Town07 15/30 km/h 동일 route를 한 group으로 묶고 C-track left, Town01 right와 함께
   3-fold leave-one-route-out으로 구조·학습 budget을 고정한다.
8. Town03 DEV는 마지막 한 번만 평가하며 C-right 3 + D-straight 3 + D-combined 3을 모두
   통과하지 못하면 실패다.
9. J가 통과해도 exploratory mechanism signal일 뿐, 신규 물리 straight route와 독립
   stationary/seed 확인 전에는 승격하지 않는다.

[구조화된 수치](i_failure_root_cause.json) ·
[I 결과](../README.md)
