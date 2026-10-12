# 06 — 성능 우선 I DRIVE-mode geometry residual

<!-- HH_260906 - Record the failed geometry-aware recovery without promoting a model or adding autonomous-function heads. -->

## 결론

I 실험도 **성능 회복에 실패했다.** 최종 판정은
`FAIL_DRIVE_MODE_RESIDUAL_RECOVERY_SIGNAL`이며 I0/I1 어느 쪽도 승격하지 않는다. 후보 궤적의
XY·속도 geometry를 직접 본 I1은 I0보다 Town03 직진 ADE/FDE/속도 MAE를 모두 개선했지만,
우회전 세 지표를 모두 악화시켰고 combined 속도 MAE도 `+0.039057 m/s` 퇴행했다. paired
비퇴행 gate는 `5/6`, I1 절대 metric gate는 `0/9`다.

따라서 현재 결과로 checkpoint export, learned closed-loop, 기능 head 추가, NuRec 혼합학습,
Autoware 제어 또는 실차 적용을 시작하지 않는다.

## 동등 A/B 계약

F에서 계산한 candidate와 H의 exact cache·학습 순서를 그대로 사용해, 같은 용량의 residual
scorer 두 개만 비교했다.

- I0: F prior/context + 192개 candidate geometry 입력을 모두 0으로 마스킹
- I1: F prior/context + candidate XY·speed geometry 사용
- 각 arm `449 → 256 → 1`, trainable parameter `115,457`
- 같은 seed `20260906`, zero final layer, AdamW, learning rate `1e-4`, weight decay `1e-4`
- 각 arm 20 epoch, 8,280 optimizer step, 33,120 exposure, batch 4
- H1의 token-bound cyclic-uniform schedule을 양 arm에 byte-exact하게 재사용
- 원래 F가 DRIVE로 고른 행에서만 후보 0..5를 재순위화
- 원래 F STOP 행의 K12 logits·선택을 exact 보존하고 DRIVE/STOP mode 변화 0건
- Town03 DEV의 label·cost·metric은 마지막 update 뒤에만 사용
- TEST·GPU1·resume·retry·새 기능 head·runtime export 사용 0

## 고정 DEV 결과

모두 6.4초 지평이며 낮을수록 좋다. 우회전은 C, 직진과 combined는 D의 사전 고정값이
상한이다.

| 범위 | 고정 상한 ADE / FDE / 속도 | I0 문맥-only | I1 geometry-aware | I1 절대 판정 |
| --- | --- | --- | --- | --- |
| Town03 우회전 337 | 2.935 / 7.794 m / 1.108 m/s | 2.928 / 7.350 / 1.019 | **3.321 / 9.201 / 1.406** | 0/3 FAIL |
| Town03 직진 328 | 3.765 / 11.027 m / 1.788 m/s | 7.210 / 17.484 / 2.772 | **6.272 / 15.383 / 2.453** | 0/3 FAIL |
| combined 665 | 3.563 / 10.354 m / 1.635 m/s | 5.040 / 12.349 / 1.883 | **4.777 / 12.251 / 1.922** | 0/3 FAIL |

I0는 우회전 `3/3`만 통과해 전체 `3/9`, I1은 `0/9`다. 두 arm 모두 candidate geometry
coverage `3/3`, TRAIN/DEV mode 보존 `2/2`를 통과했지만 성능 실패를 상쇄하지 않는다. 봉인된
stationary DEV episode가 없으므로 stop guard는
`NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE`다.

## I1 대 I0 paired 해석

| 범위 | 지표 | I0 | I1 | I1−I0 | 판정 |
| --- | --- | ---: | ---: | ---: | --- |
| 직진 | ADE (m) | 7.209749 | 6.272105 | -0.937644 | PASS |
| 직진 | FDE (m) | 17.484250 | 15.383233 | -2.101017 | PASS |
| 직진 | 속도 MAE (m/s) | 2.772033 | 2.453209 | -0.318825 | PASS |
| combined | ADE (m) | 5.040036 | 4.776527 | -0.263509 | PASS |
| combined | FDE (m) | 12.348766 | 12.250513 | -0.098253 | PASS |
| combined | 속도 MAE (m/s) | 1.883410 | 1.922467 | +0.039057 | FAIL |

geometry 입력에는 직진 선택을 고치는 신호가 분명히 있지만, 현재 한 개 Town07 straight
증강 episode와 우회전 replay 중심 TRAIN에서는 그 신호가 맵·행동 domain을 넘어 안정적으로
일반화되지 않았다. 후보 oracle ADE는 이미 충분히 낮으므로 후보가 없는 문제가 아니라 어떤
후보를 고르는가의 문제다. 같은 frame을 더 반복하거나 더 큰 head를 붙이는 것만으로 해결됐다고
볼 수 없다.

Town03 straight의 DRIVE oracle은 candidate 2가 `312/328`인데 I0는 candidate 2를 `65/328`,
I1은 `105/328`만 선택했고 candidate 5를 각각 `258/328`, `219/328` 선택했다. 반면 TRAIN의
유일한 straight 물리 경로인 Town07 p00의 15 km/h capture에서는 candidate 2가 oracle과 I1
선택 모두 `328/328`이다. candidate 2−5 logit 평균은 Town07에서 `+18.011`, Town03에서
`-1.993`으로 약 20 logit이 뒤집힌다. geometry 자체가 없어서가 아니라 desired route를
candidate와 인과적으로 연결하는 입력이 빠져 map/episode context가 그 역할을 대신한 결과다.

## 다음 성능 회복 순서

기능 head와 NuRec 도입은 계속 보류한다. 먼저 기존 exact corpus에서 label·미래 ego·이미지를
scorer 입력으로 쓰지 않고, raw causal route polyline과 현재까지의 ego speed/acceleration
history만 candidate geometry에 대응시키는 **J causal route ranker**를 진단한다. fused context와
original logit은 candidate ranking feature에서 제거하고, original logit은 DRIVE/STOP mode와
group maximum 보존에만 쓴다. F-original DRIVE TRAIN 1,232행만 loss에 포함하고 STOP 243행은
byte-exact 보존한다. 같은 Town07 p00의 15/30 km/h capture는 한 route group으로 묶어
Town07 straight / C-track left / Town01 right 3-fold route-group LOO로 구조와 계수를 고정한다.

J도 실패하면 우회전-only G가 아니라, Town03 DEV와 Town04/Town15 TEST를 열지 않은 채 TRAIN
역할 Town의 서로 다른 물리 경로에서 신규 15 km/h straight episode를 수집한다. 기존 약
210 m route는 30 km/h에서 unchanged 30초 anchor span과 6.4초 future를 함께 만족하는
`333.333 m`보다 짧으므로 30 km/h는 더 긴 route를 별도로 생성·검증한 뒤 진행한다.
J가 성공해도 이미 본 Town03 DEV를 사용한 exploratory mechanism signal일 뿐이며, 신규 물리
경로 확인 전에는 정식 일반화나 승격 근거가 아니다.

[NuRec 검토](../../../../../nurec-real2sim-feasibility-ko.md)는 별도 축이다. NuRec은 실차 로그를
real-to-sim 장면으로 복원하는 기술이지 현재 selector 붕괴를 고치는 도구가 아니다. 기존 CARLA
0.9.15 기준선은 보존하고, 성능 회복 후 별도 CARLA 0.9.16+ 환경에서 공식 scene 한 개부터
검증한다.

## 증거와 제한

- [전체 I summary](evidence/summary.json)
- [I 실패 원인과 J 계약](analysis/README.md)
- [구조화된 실패 수치](analysis/i_failure_root_cause.json)
- [I0 exact report](metrics/I0_prior_context_residual/report.json)
- [I1 exact report](metrics/I1_candidate_geometry_residual/report.json)
- [원격→로컬 exact 전달 검증](evidence/exact_transfer_verification.json)
- [exact artifact summary](../../../../../../storage/experiments/training/2026-09-21/moving_drive_mode_residual_recovery_i_v1/summary.json)
- [exact artifact checksum](../../../../../../storage/experiments/training/2026-09-21/moving_drive_mode_residual_recovery_i_v1/SHA256SUMS)

이 실험은 저장된 CARLA TRAIN/DEV candidate cache에 대한 single-seed open-loop 진단이다. 라이브
CARLA 주행, Autoware/RViz 화면, GIF/PNG, 모든 Town, 30/60 km/h closed-loop, 장애물 회피,
ACC, 차선 변경 또는 실차 성능 검증이 아니다. 이 단계가 PNG/GIF를 새로 만들지 않았으므로
화면 증거를 가장하지 않고 exact metric과 checksum만 보존했다.

[직전 H 단계](../05_moving_selector_sampling_recalibration_h/README.md) ·
[cycle 목차](../README.md)
