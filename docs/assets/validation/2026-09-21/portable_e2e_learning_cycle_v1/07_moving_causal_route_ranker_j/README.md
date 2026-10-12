# 07 — 성능 우선 J causal route ranker

<!-- HH_260906 - Record the failed causal-route recovery without model promotion or feature-head expansion. -->

## 결론

J 실험은 raw causal route를 candidate geometry에 직접 대응시키는 방향이 **직진 FDE와 속도에는
유효했지만, 고정된 9개 성능 기준을 동시에 회복하지 못했다.** 최종 판정은
`FAIL_CAUSAL_ROUTE_RANKER_RECOVERY_SIGNAL`, metric gate는 `4/9`다. geometry gate `3/3`과
TRAIN/DEV 보존 gate `2/2`는 통과했지만 성능 실패를 상쇄하지 않는다.

따라서 J scorer를 승격·runtime export하지 않는다. 신규 자율주행 기능 head, TEST, GPU 1,
closed-loop CARLA, Autoware 제어, 실차 적용도 시작하지 않았다.

## 무엇을 학습했나

J는 F의 후보 궤적과 DRIVE/STOP 결정을 고정한 채, 후보별로 같은 네 가지 cost만 계산하는
4-parameter 비음수 scorer다.

- ranker 입력: raw route polyline, 현재 ego speed/longitudinal acceleration, frozen candidate
  XY/speed
- 금지 입력: target, label, future ego, image, fused context, map/episode ID, 원래 F의 candidate
  ranking logits
- 학습 대상: F-original DRIVE `1,232`행만 사용, STOP `243`행은 loss에서 제외
- TRAIN 물리 경로 group: Town07 same-route `625`, C-track `289`, Town01 `318`
- 3-fold leave-one-route-group-out 각 256 step + all-TRAIN final 256 step
- 총 optimizer row exposure `946,176`, final fit exposure `315,392`
- shared feature: lateral, heading, speed, curvature; candidate-index parameter 없음
- seed `20260906`, PRO6000 물리 GPU 0만 사용

이는 작은 head를 여러 기능으로 확장한 실험이 아니다. 성능 저하 원인이 route intent 부재인지
최소 구조로 먼저 확인한 진단이다.

## 고정 DEV 결과

모두 6.4초 지평이며 낮을수록 좋다. 우회전은 C, 직진과 combined는 D의 사전 고정 상한을
그대로 사용했다.

| 범위 | 고정 상한 ADE / FDE / 속도 | J 결과 | 통과 |
| --- | --- | --- | --- |
| Town03 우회전 337 | 2.935314 / 7.793866 m / 1.107615 m/s | 2.994809 / 8.008273 / 1.111932 | 0/3 |
| Town03 직진 328 | 3.764740 / 11.027441 m / 1.787750 m/s | 4.148009 / **7.209370** / **1.475513** | 2/3 |
| combined 665 | 3.563246 / 10.353537 m / 1.635022 m/s | 3.563605 / **7.614228** / **1.291262** | 2/3 |

combined ADE는 상한보다 `0.000360 m` 높아 실패했다. 수치가 매우 가깝더라도 gate를 사후에
완화하지 않는다. 우회전은 ADE `+0.059495 m`, FDE `+0.214406 m`, 속도 MAE
`+0.004317 m/s`로 세 항목 모두 상한을 넘었다. 직진은 FDE와 속도를 크게 회복했지만 ADE가
`+0.383269 m` 높다.

## 보존·인과성 계약

- TRAIN/DEV 모두 DRIVE/STOP mode 변화 `0`, fallback logit mismatch `0`, reranked DRIVE group
  maximum mismatch `0`
- final TRAIN: fallback `694`, straight DRIVE rerank `781`
- final DEV: fallback `208`, straight DRIVE rerank `457`
- raw sample `2,140/2,140`의 dataset→episode→samples chain과 timestamp causality 검증
- raw route 유효 길이 `4…121`을 mask와 함께 처리하며 padding은 score에 사용하지 않음
- DEV는 final scorer를 고정한 뒤 한 평가 phase에서 combined 1회와 sealed subgroup 2회만 환산
- TEST read/use `0`, DEV 기반 중간 선택 `0`

turn command 또는 route turn evidence가 처음 나타난 시점부터는 sequence 끝까지 exact F
fallback을 유지한다. 단, 그 전의 인과적으로 구분 불가능한 straight prefix는 ranker가
재순위화할 수 있다. 이는 전체 회전 episode를 처음부터 보호한다는 계약이 아니다.

## LOO가 말하는 것과 말하지 않는 것

| hold-out TRAIN route group | fit DRIVE | hold DRIVE | policy 적용 hold DRIVE | scorer oracle agreement |
| --- | ---: | ---: | ---: | ---: |
| Town07 same physical route | 607 | 625 | 625 | 39.20% |
| C-track | 943 | 289 | 0 | 17.30% |
| Town01 | 914 | 318 | 156 | 69.81% |

C-track hold-out은 모든 `289`행이 turn fallback이라 실제 J policy가 적용된 행이 `0`이다.
따라서 3-fold 수치는 shared scorer 학습 절차의 TRAIN-only 진단이지, 세 경로에서 J policy가
일반화됐다는 증거가 아니다.

## 다음 성능 회복 순서

같은 DEV에 맞춰 threshold나 네 가중치를 다시 튜닝하지 않는다. 기능 head와 NuRec 도입도
계속 보류한다. 다음 단계는 사전에 승인된 create-only 15 km/h straight capture 계약을 사용해
Town03 DEV 및 Town04/Town15 TEST를 열지 않고 TRAIN 역할 Town의 **서로 다른 물리 경로**에서
독립 straight episode를 수집하는 것이다.

현재 capture plan 생성 수와 실제 신규 data 수는 모두 `0`이며 readiness는 `PENDING_QA`다.
후보 슬롯은 Town07, C-track, Town01의 straight route이며, C-track 후보 두 개는 corridor
section `7`개가 exact 중복되어 route-group independence가 아직 `NOT_CERTIFIED`다. 실제 수집
전 route QA로 중복을 제거하거나 한 group으로 묶어야 한다. 기존 약 210 m route는 30 km/h의
30초 anchor와 6.4초 future 계약에 필요한 `333.333 m`보다 짧으므로 30 km/h capture는 긴
경로가 따로 검증될 때까지 거부한다.

신규 데이터는 기존 F/J 구조와 C/D 9-metric gate를 고정한 A/B로 검증한다. 그 뒤 독립
stationary DEV, 새 seed, 새 물리 경로 확인을 모두 통과해야만 model promotion을 다시 검토한다.

## 증거와 제한

- [exact 실행 계획](evidence/plan.json)
- [exact 실행 요약](evidence/summary.json)
- [exact J report](metrics/J_causal_route_ranker/report.json)
- [원격→로컬 exact 전달 검증](evidence/exact_transfer_verification.json)
- [실패 원인과 다음 결정 경계](analysis/README.md)
- [exact artifact summary](../../../../../../storage/experiments/training/2026-09-21/moving_causal_route_ranker_j_v1/summary.json)
- [exact artifact checksum](../../../../../../storage/experiments/training/2026-09-21/moving_causal_route_ranker_j_v1/SHA256SUMS)

이 단계는 저장된 CARLA TRAIN/DEV candidate cache에 대한 single-seed offline open-loop
진단이다. 라이브 CARLA 주행, Autoware/RViz 화면, 모든 Town, 30/60 km/h closed-loop,
장애물 회피, ACC, 차선 변경 또는 실차 성능 검증이 아니다. 실행 중 새로운 화면이 없었으므로
PNG/GIF를 만들지 않았고 exact metric과 checksum만 보존했다.

[직전 I 단계](../06_moving_drive_mode_residual_recovery_i/README.md) ·
[cycle 목차](../README.md)
