# 02 — 이동 데이터 고예산 C/D 단일-seed 학습 파일럿

<!-- HH_260906 - Report the preregistered high-budget C/D diagnostic without promoting a checkpoint or treating offline plots as driving evidence. -->

상태: **`COMPLETE_NOT_PROMOTED` / `FAIL_SINGLE_SEED_DIAGNOSTIC`**. PRO6000 물리
GPU 0에서 C와 D를 각각 7,380 optimizer step까지 fresh fit하고, 직전 5,740-step A/B와
함께 2×2로 비교했다. D는 Town03 직진에서 C보다 크게 좋아졌지만 기존 Town03 우회전의
ADE·FDE·속도 오차가 모두 사전 한도를 넘었다. 따라서 **D checkpoint를 승격하지 않는다.**

- 실행 ID: `hh260921-moving-data-high-budget-ab-seed20260906-v1`
- 실행 시각: 2026-09-21 06:29:03–07:06:38 UTC
  (KST 15:29:03–16:06:38, 약 37분 35초)
- 장치: NVIDIA RTX PRO 6000 Blackwell Server Edition, 물리 GPU 0,
  `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`
- 실행 결과: train/evaluate/audit/subgroup × 2 arm = **8/8 단계 완료**
- 두 arm 모두 resume·retry·early stopping·best-checkpoint 선택 없이 final checkpoint만 평가
- 공통 VAL: Town03 우회전 337 + Town03 직진 328 = **665개**, 네 arm 모두 동일
- TEST: 읽거나 사용하지 않음

원격 디렉터리는 기존 runner 호환성 때문에 `A_baseline_train`과
`B_town07_augmented_train`이지만, 이번 고예산 보고서에서는 각각 **C**와 **D**로 부른다.

## 2×2 실험의 의미

두 축은 TRAIN 구성과 optimizer budget이다. 모델 구조, seed `20260906`, batch 4,
learning rate `1e-4`, weight decay `1e-4`, gradient norm 5, 손실 가중치와 VAL은 고정했다.

| arm | TRAIN 구성 | step | 실제 표본 노출 | 학습 위치 | 역할 |
| --- | --- | ---: | ---: | --- | --- |
| A | 기존 3 episode / 1,147개 | 5,740 | 22,940 | 20 epoch | 저예산 기존 TRAIN |
| B | A + Town07 직진 328개 = 1,475개 | 5,740 | 22,945 | 15 epoch + 205 batch | 저예산 증강 TRAIN |
| C | 기존 3 episode / 1,147개 | 7,380 | 29,495 | 25 epoch + 205 batch | 고예산 기존 TRAIN |
| D | C + Town07 직진 328개 = 1,475개 | 7,380 | 29,500 | 20 full epoch | 고예산 증강 TRAIN |

핵심 비교는 같은 고예산의 `D - C`다. `B - A`는 직전 같은-step 데이터 효과,
`C - A`와 `D - B`는 각 데이터 구성에서 추가 1,640 step의 변화다. interaction
`(D-C)-(B-A)`는 budget이 데이터 효과를 얼마나 바꿨는지 보는 진단값이다. 단일 seed라서
인과 효과나 일반화 성능으로 확정하지 않는다.

고예산 run은 단순히 이전 checkpoint를 resume한 것이 아니라 동일 seed로 처음부터 다시
실행했다. 그럼에도 A와 C의 처음 **5,740개 history 행**, B와 D의 처음 **5,740개 history 행**이
각각 byte-identical이다. A↔C prefix SHA-256은
`992ac3e4ab97bc574266d4a98a5c8694d5ce31bcc709616a67b803bd37fc02e3`, B↔D는
`4d8ea621007e2f0a9674a2fd7ff9e8d370e99071655d6bcc1367a7a870760d57`다. 이는 이 고정
환경의 deterministic replay 근거이지, 다른 seed·장치에서 같은 성능이 나온다는 뜻은 아니다.

## C/D 고예산 결과

아래 값은 동일한 open-loop VAL의 final checkpoint 결과다. ADE/FDE는 m, 속도 MAE는 m/s이고
`Δ = D - C`다. 오차와 loss는 낮을수록 좋으므로 음수는 개선, 양수는 악화다.

| 평가 범위 | n | arm | loss | ADE 1.0 s | ADE 3.0 s | ADE 6.4 s | FDE 6.4 s | 속도 MAE 6.4 s |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 공통 VAL | 665 | C | 5.370674 | 0.231441 | 1.524760 | 5.478203 | 13.481566 | 2.065069 |
| 공통 VAL | 665 | D | 3.013610 | 0.132393 | 0.809049 | 3.563246 | 10.353537 | 1.635022 |
| 공통 VAL | 665 | **Δ** | **-2.357064** | **-0.099048** | **-0.715711** | **-1.914957** | **-3.128029** | **-0.430047** |
| Town03 우회전 | 337 | C | 3.612647 | 0.180321 | 0.853928 | 2.935314 | 7.793866 | 1.107615 |
| Town03 우회전 | 337 | D | 4.002404 | 0.195692 | 0.921545 | 3.367132 | 9.697630 | 1.486372 |
| Town03 우회전 | 337 | **Δ** | **+0.389757** | **+0.015371** | **+0.067617** | **+0.431818** | **+1.903764** | **+0.378757** |
| Town03 직진 | 328 | C | 7.176940 | 0.283964 | 2.213999 | 8.090865 | 19.325330 | 3.048795 |
| Town03 직진 | 328 | D | 1.997685 | 0.067357 | 0.693466 | 3.764740 | 11.027441 | 1.787750 |
| Town03 직진 | 328 | **Δ** | **-5.179255** | **-0.216607** | **-1.520533** | **-4.326125** | **-8.297889** | **-1.261045** |

D는 C 대비 공통 VAL ADE 6.4초를 **34.96%**, 직진 ADE 6.4초를 **53.47%**, 직진
FDE 6.4초를 **42.94%** 줄였다. 반면 우회전 ADE 6.4초는 **14.71%**, FDE는
**24.43%**, 속도 MAE는 **34.20%** 늘었다. 공통 평균 개선으로 회전 퇴행을 가리지 않는다.

## 저예산 A/B와 함께 본 해석

| 범위 | 지표 | A | B | C | D | 저예산 `B-A` | 고예산 `D-C` |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 공통 VAL | ADE 6.4 s | 5.449211 | 4.259853 | 5.478203 | 3.563246 | -1.189358 | -1.914957 |
| 공통 VAL | FDE 6.4 s | 13.537088 | 12.128328 | 13.481566 | 10.353537 | -1.408760 | -3.128029 |
| 우회전 | ADE 6.4 s | 3.422663 | 4.355226 | 2.935314 | 3.367132 | +0.932562 | +0.431818 |
| 우회전 | FDE 6.4 s | 8.427078 | 11.993512 | 7.793866 | 9.697630 | +3.566434 | +1.903764 |
| 직진 | ADE 6.4 s | 7.531365 | 4.161863 | 8.090865 | 3.764740 | -3.369503 | -4.326125 |
| 직진 | FDE 6.4 s | 18.787311 | 12.266843 | 19.325330 | 11.027441 | -6.520468 | -8.297889 |

고예산에서 우회전 퇴행 폭은 줄고 직진 개선 폭은 커졌다. ADE 6.4초 interaction은 우회전
`-0.500745 m`, 직진 `-0.956622 m`이고, FDE interaction은 각각 `-1.662670 m`,
`-1.777420 m`다. 즉 추가 budget이 D의 결과를 개선한 방향성은 보인다. 그러나 D는 C보다
여전히 우회전에서 나쁘므로, 이 관찰을 gate 통과나 데이터 채택으로 바꾸지 않는다.

## 사전 고정 gate와 판정

사전에 D가 C 대비 만족해야 할 조건은 다음과 같았다.

| slice | 조건 | 관측 D/C | 판정 |
| --- | --- | ---: | --- |
| 우회전 | ADE 6.4 s ≤ C의 100% | 114.71% | **FAIL** |
| 우회전 | FDE 6.4 s ≤ C의 100% | 124.43% | **FAIL** |
| 우회전 | 속도 MAE ≤ C의 105% | 134.20% | **FAIL** |
| 직진 | ADE 6.4 s ≤ C의 90% | 46.53% | PASS |
| 직진 | FDE 6.4 s ≤ C의 90% | 57.06% | PASS |

geometry 조건은 선택 경로, 하나 이상의 후보, 모든 후보 모두 C와 D 각각 **664/665
(99.8496%)**로 통과했다. 하지만 geometry 형식 적합성은 주행 성공이 아니며, 회전 metric
세 항목이 실패했으므로 전체 gate는 **FAIL**, `adoption_allowed=false`,
`automatic_promotion=false`다. 최소 3개 seed 확인 조건도 아직 충족하지 않았다.

## 오프라인 화면 자료

compact stage에는 우회전/직진 × C/D × 각 6장, 총 **24개 PNG**를 원본과 byte-identical로
보존했다. 청색 ego box는 하단 중앙에 있고 회색 route, 녹색 expert, 굵은 selected trajectory가
함께 보인다. 전체 route 범위를 그려 궤적이 화면 대비 작게 보일 수 있다.

| Town03 우회전 — C | Town03 우회전 — D |
| --- | --- |
| ![C right-turn offline trajectory](visuals/town03_right_turn_dev/C_old_train_7380_steps/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) | ![D right-turn offline trajectory](visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) |
| Town03 직진 — C | Town03 직진 — D |
| ![C straight offline trajectory](visuals/town03_straight_dev/C_old_train_7380_steps/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) | ![D straight offline trajectory](visuals/town03_straight_dev/D_augmented_train_7380_steps/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) |

[우회전 C/D 12장](galleries/town03_right_turn_dev.md) ·
[직진 C/D 12장](galleries/town03_straight_dev.md)

이 PNG들은 **offline fixed-DEV trajectory render**다. 새 CARLA 라이브 GUI, Autoware/RViz
전체화면, 모델 직접 폐루프 주행 또는 실차 촬영이 아니다. 이번 단계에서 GIF는 만들지 않았다.

## 정확한 근거

<!-- HH_260906 - Keep byte-integrity evidence distinct from model quality and closed-loop safety claims. -->

| 근거 | SHA-256 또는 결과 |
| --- | --- |
| 원격 source commit | `b478f02e42b94bf04bffec5c8170e05edc33b0f8` |
| 고정 plan | `dda31c7ff03b959575d8483bfed8506bbf4e2c6c5bb4597c6285517a3c991cdc` |
| 실행 summary | `12c34fde7c64bd47767191139a729b11e625538065f0e7fbb2d0230e4a46b369` |
| 2×2 comparison | `96c0e3cf94ce90420800c85678a23d341f9203248fb16166943c8c045977f561` |
| runner / evaluator / summarizer | `c4bf4fab…` / `eba4f7ef…` / `0e685ae8…` |
| C / D checkpoint | `01b6764d…` / `aba44cb5…` |
| C / D history | 각 7,380행, `0d0b2746…` / `7529937a…` |
| 원격↔로컬 transfer | 115 files / 55,944,319 bytes / inventory byte-identical |
| 관련 테스트 | 73 passed / 0 failed, 기존 327-pass regression evidence 유지 |

- [A/B/C/D 원값·contrast·gate](evidence/comparison_2x2.json)
- [고예산 실행 summary](evidence/execution_summary.json)
- [전송 검증](evidence/transfer_verification.json)
- [발행 검증](evidence/verification.json)
- [C 전체/하위그룹 metrics](metrics/C_old_train_7380_steps/subgroup_summary.json)
- [D 전체/하위그룹 metrics](metrics/D_augmented_train_7380_steps/subgroup_summary.json)
- [checkpoint·전체 history·log·source를 포함한 exact artifact](../../../../../../storage/experiments/training/2026-09-21/moving_data_high_budget_ab_seed_pilot_v1/README.md)
- [직전 5,740-step A/B 보고서](../01_moving_data_ab_pilot/README.md)

## 한계와 다음 단계

이번 증거는 **CARLA 수집 데이터의 single-seed open-loop fixed-DEV 진단**이다. 모델 export,
Autoware 10 Hz shadow, CARLA 폐루프, CARLA/Autoware live 화면, 모든 Town, 30개 목표 기능,
장애물 회피·ACC·차선 변경·정지 기능, 실차 검증·차량 제어 승인은 없다. 따라서 모델 직접 주행
기능 승인 수는 **0/30 유지**다. VAD·VisionPilot·METEOR와 동일 조건으로 비교하지도 않았다.

다음 학습은 현재 D를 승격하거나 실패한 C/D를 곧바로 3-seed 반복하는 작업이 아니다. 먼저
seed `20260906`에서 **회전 TRAIN 보강 또는 cohort-aware replay**를 별도 사전등록 A/B로 비교해
직진 개선을 유지하면서 회전 gate를 회복해야 한다. 이 개발 gate를 통과한 후보에만 학습에 쓰지
않은 최소 3개 seed로 확증한다. 그 확증까지 통과한 후보만 export 동등성 → 10 Hz Autoware
shadow → fallback이 있는 제한 CARLA 폐루프 순으로 올린다.

[현재 목차](../CURRENT.md)
