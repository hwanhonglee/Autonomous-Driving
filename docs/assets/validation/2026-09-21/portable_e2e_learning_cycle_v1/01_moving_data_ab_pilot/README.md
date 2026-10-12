# 01 — 이동 데이터 data-only A/B 단일-seed 학습 파일럿

<!-- HH_260906 - Report the paired data-only pilot without promoting either checkpoint or overstating offline evidence. -->

상태: **`COMPLETE_NOT_PROMOTED`**. PRO6000 물리 GPU 0에서 두 모델을 새로 학습하고,
동일한 Town03 VAL 665개를 전체·우회전·직진으로 나눠 평가했다. B는 새 직진 구간에서
좋아졌지만 기존 우회전에서 뚜렷하게 악화했으므로 **B checkpoint 채택을 거부한다.**
이 판정은 실행 실패가 아니라, 실행은 완료했으나 성능 퇴행 때문에 승격하지 않았다는 뜻이다.

- 실행 ID: `hh260921-moving-data-ab-seed20260906-v1`
- 실행 시각: 2026-09-21 02:58:57–03:29:16 UTC
  (KST 11:58:57–12:29:16)
- 장치: NVIDIA RTX PRO 6000 Blackwell Server Edition, 물리 GPU 0,
  `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`
- 두 arm 모두 fresh fit이며 resume·자동 retry·early stopping·best-checkpoint 선택은 없었다.

## 무엇만 바꿨나

이 실험은 **TRAIN 데이터만 바꾼 data-only A/B**다. 모델 구조, 초기화 seed, 손실,
optimizer step 수, batch 크기와 검증 데이터는 고정했다.

| 항목 | A — `A_baseline_train` | B — `B_town07_augmented_train` |
| --- | ---: | ---: |
| TRAIN | 기존 3 episode / **1,147개** | A + 신규 Town07 직진 1 episode 328개 = **1,475개** |
| 공통 VAL | Town03 우회전 337 + 신규 Town03 직진 328 = **665개** | 왼쪽과 byte/content 동일한 **665개** |
| seed | `20260906` | `20260906` |
| optimizer step | **5,740** | **5,740** |
| 실제 표본 노출 | **22,940** | **22,945** |
| 진행 위치 | 정확히 20 epoch | 15 epoch + 205 batch, 약 15.56 epoch |
| checkpoint SHA-256 | `05d4b7a31df5e3a611b7571e9123453da5eaeea1f8138e4c1b2b30e23e7d58e1` | `710989855448dfe7427f57e39010b62696db5ba68be4b65ff73c5bb769255605` |

공통 설정은 batch 4, learning rate `1e-4`, weight decay `1e-4`, gradient norm 5,
`uniform_without_replacement`, worker 0이다. STOPMIX의 candidate-score와 candidate-regret
가중치는 각각 0.1이다. 모델은 `portable_e2e.perspective_trajectory.physical_stopmix.v1`,
1,056,362 parameters로 동일하다. VAL content SHA-256은
`c65024d1f0b1bab30524c851bbd09342043d9ac57287dd39e9b5e7033becb229`, tensor fingerprint는
`9d293b60f0eefc61c924e48fa629ef6f4f43ce0b22171bc0ca78f9fe74c9d2ed`로 두 arm이 같다.
split 간 sample/source ID 중복과 image SHA 교차는 0이었다. TEST 참조나 payload는 포함하거나 열지 않았다.

## 전체·회전·직진 결과

아래는 최종 checkpoint의 같은 open-loop VAL 결과다. ADE/FDE는 m, 속도 MAE는 m/s이고,
`Δ = B - A`이므로 오차의 음수는 개선, 양수는 악화다. `loss`도 낮을수록 좋다.

| 평가 범위 | n | arm | loss | ADE 1.0 s | ADE 3.0 s | ADE 6.4 s | FDE 6.4 s | 속도 MAE 6.4 s |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 공통 VAL | 665 | A | 4.676649 | 0.241420 | 1.501736 | 5.449211 | 13.537088 | 2.053006 |
| 공통 VAL | 665 | B | 4.250228 | 0.174575 | 1.048191 | 4.259853 | 12.128328 | 1.911621 |
| 공통 VAL | 665 | **Δ** | **-0.426421** | **-0.066846** | **-0.453545** | **-1.189358** | **-1.408760** | **-0.141385** |
| Town03 우회전(기존 DEV) | 337 | A | 3.720582 | 0.223437 | 1.106932 | 3.422663 | 8.427078 | 1.186367 |
| Town03 우회전(기존 DEV) | 337 | B | 4.629201 | 0.232188 | 1.243243 | 4.355226 | 11.993512 | 1.876499 |
| Town03 우회전(기존 DEV) | 337 | **Δ** | **+0.908620** | **+0.008751** | **+0.136311** | **+0.932562** | **+3.566434** | **+0.690132** |
| Town03 직진(신규 DEV) | 328 | A | 5.658951 | 0.259897 | 1.907373 | 7.531365 | 18.787311 | 2.943425 |
| Town03 직진(신규 DEV) | 328 | B | 3.860857 | 0.115380 | 0.847786 | 4.161863 | 12.266843 | 1.947706 |
| Town03 직진(신규 DEV) | 328 | **Δ** | **-1.798094** | **-0.144517** | **-1.059587** | **-3.369503** | **-6.520468** | **-0.995719** |

B의 공통 VAL ADE 6.4초는 **21.83%**, 신규 직진은 **44.74%** 감소했다. 반면 기존
우회전 ADE 6.4초는 **27.25%**, FDE 6.4초는 **42.32%**, 속도 MAE는 **58.17%**
증가했다. 공통 평균 개선으로 기존 회전의 퇴행을 덮을 수 없다. 공통 candidate-score loss도
3.692476→3.787309(+0.094833)로 악화했다.

이 결과가 새 Town07 데이터 자체가 회전을 해친다는 인과 증명은 아니다. 같은-step 비교에서
A는 기존 1,147개를 20회 보았지만 B는 더 큰 혼합 데이터를 약 15.56 epoch만 보아,
기존 표본 반복 노출이 줄었다. 따라서 원인은 데이터 혼합 희석, 기존 장면 노출 부족,
단일 seed 변동 중 하나 또는 조합일 수 있다.

## 물리 gate와 화면

선택된 경로의 geometry gate는 두 arm 모두 **664/665 (99.8496%)** 통과했고, 각 1개는
speed 조건으로 거부됐다. 적어도 한 후보가 통과한 수도 두 arm 모두 664/665다. 다만 12개
후보가 모두 통과한 표본은 A 664/665, B 606/665였고, B candidate 3에는 extent 실패 58개가
추가로 있다. 선택 결과 통과만으로 B 전체 후보 품질이 같다고 판단하지 않는다.

compact docs에는 우회전/직진 × A/B × 각 6장, 총 **24개 PNG**가 있다. 청색 ego box를
화면 하단 중앙에 고정하고 회색 route, 녹색 expert, 굵은 색 selected trajectory가 함께 보이는
**오프라인 궤적 렌더링**이다. 실제 차량 사진, CARLA 라이브 GUI, RViz 또는 Autoware 실행
전체화면이 아니다. 따라서 이 그림을 모델 직접 주행 화면으로 사용하지 않는다.

| 기존 Town03 우회전 — A | 기존 Town03 우회전 — B |
| --- | --- |
| ![A right-turn offline trajectory](visuals/town03_right_turn_dev/A_baseline_train/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) | ![B right-turn offline trajectory](visuals/town03_right_turn_dev/B_town07_augmented_train/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) |
| 신규 Town03 직진 — A | 신규 Town03 직진 — B |
| ![A straight offline trajectory](visuals/town03_straight_dev/A_baseline_train/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) | ![B straight offline trajectory](visuals/town03_straight_dev/B_town07_augmented_train/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) |

[우회전 A 6장](visuals/town03_right_turn_dev/A_baseline_train/) ·
[우회전 B 6장](visuals/town03_right_turn_dev/B_town07_augmented_train/) ·
[직진 A 6장](visuals/town03_straight_dev/A_baseline_train/) ·
[직진 B 6장](visuals/town03_straight_dev/B_town07_augmented_train/)

정확한 artifact에는 위 subgroup 24장과 공통 VAL 렌더 24장을 합친 48장, 두 최종 checkpoint,
11,480개 학습 이력 행, 단계별 log/receipt, 고정 plan과 source snapshot이 보존돼 있다.

## 재현 근거와 검증

<!-- HH_260906 - Keep file hashes and test counts separate from model-quality and driving claims. -->

| 근거 | SHA-256 |
| --- | --- |
| 원격 source commit | `b478f02e42b94bf04bffec5c8170e05edc33b0f8` |
| 고정 실행 plan | `df2be25c367b167089cd2e483c38f79857ba70d96f742825396f9523124d21d2` |
| 실행 summary | `21340622c31bc2d7883f645bbd24de306116db901ce72ca00b63780023314111` |
| 전체 data-view report | `f111b147d8e873313db17a6e86b5d0df730c5fdb1e152f7b8eb0449c5288f5af` |
| model config 파일 / canonical | `29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602` / `c17b49ea680b5cae0d9fc35dc6c457af8cc32d1b7171863534f70e9b158cfb7a` |
| data-view builder | `cfb377a95e1f6c886560f1e5d3420bd859fc9c5cf7d4762fc25c0b50f76ca022` |
| 실행 runner | `a6852f9fec12c24201abf9ac45b8661aad6fe91f2f71d51fecb8f96f4106d70e` |
| subgroup evaluator | `eba4f7ef276f9a70a1b4857b4ccabccdc9889dea563ba898bb4713563f7a17b0` |
| 원격/로컬 inventory 파일 공통 digest | `4efe228339b4fc2f20cefe4e9f316834783bb3695fe82283a5886ec0ea8cfb04` |

신규 builder/runner/evaluator **47 passed**, 기존 학습 회귀 **280 passed**, 중복 없는 합계
**327 passed / 0 failed**다. `py_compile`, `git diff --check`, 원격 8단계 완료,
실행 후 source/data-view 재검사, 원격 repo clean, GPU 0 idle도 확인했다. 이는 코드·산출물
무결성 검사이며 모델 기능 합격 수가 아니다.

- [compact data-view 근거](evidence/data_view_summary.json)
- [원격 실행 summary](evidence/execution_summary.json)
- [최종 발행 검증](evidence/verification.json)
- [전송 receipt](evidence/transport_receipt.json)
- [A 전체/하위그룹 metrics](metrics/A_baseline_train/subgroup_summary.json)
- [B 전체/하위그룹 metrics](metrics/B_town07_augmented_train/subgroup_summary.json)
- [checkpoint·전체 log를 포함한 exact artifact](../../../../../../storage/experiments/training/2026-09-21/moving_data_ab_seed_pilot_v1/README.md)

## 판정 경계와 다음 실험

이번 결과는 단일 seed·CARLA 데이터·open-loop 궤적 예측 진단이다. **TEST 미사용**, export 없음,
Autoware/CARLA 폐루프 없음, 모든 Town 검사 없음, 30개 목표 기능 검증 없음, 실차 시험·제어 승인
없음이다. 이번 단계가 추가한 모델 직접 주행 기능 통과는 0개이며 기존 0/30 상태를 바꾸지 않는다.
VAD·VisionPilot·METEOR 또는 다른 E2E 모델과 동일 조건으로 비교하지 않았으므로 동급·우월하다고
주장하지 않는다. 연구용 데이터 뷰 검증은 일반 데이터셋 admission도 아니다.

다음 비교는 두 가지를 분리해 진행한다.

1. B를 **7,380 step = 혼합 데이터 20 full epoch = 총 29,500회 노출**로 학습해 기존
   1,147개 표본의 누적 노출도 A와 같은 22,940회가 되도록 맞추거나, balanced replay로
   기존/신규 표본 비율을 고정한다. 이번 5,740-step 결과와 같은 실험인 것처럼 합치지 않는다.
2. 사전에 고정한 최소 3개 seed에서 직진 개선과 우회전 비퇴행을 동시에 확인한다. 우회전
   퇴행이 남으면 B는 계속 거부하고, 회전 TRAIN 보강 또는 data-mix를 별도 A/B한다.
3. 이 offline 기준을 통과한 후보만 export 동등성 → 10 Hz Autoware shadow → fallback이 있는
   제한 CARLA 폐루프로 넘긴다. 모든 Town·정지·회피·ACC·차선 변경과 실차는 그 뒤의 별도 검증이다.

[현재 목차](../CURRENT.md) ·
[직전 Town03 직진 DEV 수집](../../../2026-09-16/portable_e2e_learning_cycle_v1/02_dev_route_and_capture/README.md)
