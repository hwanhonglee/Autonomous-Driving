# 기존 모델과 새 지도 학습 결과 비교

**기존 parent 대비 개선**과 **같은 추가 학습에서 OFF/ON 혼합이 준 영향**은 다른 비교입니다. 아래에서 둘을 따로 표시합니다.

parent는 Oct1의 동일 245,831-parameter 모델이며, control과 balanced는 그 가중치에서 각각 고정 10,000회 학습한 결과입니다.

Town01 DEV 118장과 Town02 p02 holdout 232장은 이미 확인한 자료입니다. 새 독립 평가 지역은 0개이며, 메인 0/9·서브 0/30 공식 승인은 유지됩니다.

이 수치는 RGB 영상의 7개 분류 결과입니다. 신호 상태·횡단보도·정지선·차선 연결·정밀 좌표를 갖춘 자동 정밀지도(AMG)나 자율주행 성공을 뜻하지 않습니다.

## dev_town01: 118장 전체 화면

| 분류 | 기존 IoU | control IoU | balanced IoU | control − 기존 | balanced − 기존 | balanced − control |
|---|---:|---:|---:|---:|---:|---:|
| other | 97.50% | 98.59% | 98.30% | +1.087 pp | +0.795 pp | -0.293 pp |
| road | 99.67% | 99.74% | 99.70% | +0.073 pp | +0.024 pp | -0.049 pp |
| sidewalk | 96.95% | 97.88% | 97.23% | +0.932 pp | +0.282 pp | -0.650 pp |
| lane_marking | 95.85% | 97.51% | 97.28% | +1.664 pp | +1.432 pp | -0.232 pp |
| traffic_light_fixture | 38.21% | 69.09% | 67.45% | +30.877 pp | +29.245 pp | -1.632 pp |
| traffic_sign | 6.77% | 6.44% | 6.58% | -0.336 pp | -0.189 pp | +0.147 pp |
| pole | 40.81% | 58.55% | 51.88% | +17.739 pp | +11.065 pp | -6.674 pp |

### 무엇을 놓치고, 잘못 표시했는지

정밀도(P)는 표시한 것 중 맞은 비율, 재현율(R)은 실제 대상 중 찾아낸 비율입니다. FN은 누락, FP는 오탐 픽셀 수입니다. 작은 물체는 도로 점수와 별도로 보아야 합니다.

| 분류 | 모델 | P | R | 정답 픽셀 | FN | FP |
|---|---|---:|---:|---:|---:|---:|
| other | parent | 99.15% | 98.32% | 11478501 | 192377 | 96589 |
| other | control | 99.48% | 99.10% | 11478501 | 103618 | 58945 |
| other | balanced | 99.43% | 98.86% | 11478501 | 130907 | 65548 |
| road | parent | 99.92% | 99.75% | 11638664 | 28661 | 9586 |
| road | control | 99.86% | 99.89% | 11638664 | 13119 | 16675 |
| road | balanced | 99.98% | 99.71% | 11638664 | 33315 | 2105 |
| sidewalk | parent | 98.48% | 98.42% | 3855877 | 60980 | 58491 |
| sidewalk | control | 98.99% | 98.87% | 3855877 | 43473 | 39087 |
| sidewalk | balanced | 98.15% | 99.04% | 3855877 | 36963 | 71839 |
| lane_marking | parent | 97.37% | 98.40% | 45917 | 736 | 1222 |
| lane_marking | control | 97.93% | 99.56% | 45917 | 203 | 964 |
| lane_marking | balanced | 97.63% | 99.63% | 45917 | 168 | 1112 |
| traffic_light_fixture | parent | 47.01% | 67.12% | 46214 | 15194 | 34971 |
| traffic_light_fixture | control | 78.02% | 85.78% | 46214 | 6573 | 11165 |
| traffic_light_fixture | balanced | 76.86% | 84.64% | 46214 | 7098 | 11775 |
| traffic_sign | parent | 7.11% | 58.66% | 2915 | 1205 | 22329 |
| traffic_sign | control | 6.68% | 63.81% | 2915 | 1055 | 25976 |
| traffic_sign | balanced | 6.89% | 60.14% | 2915 | 1162 | 23707 |
| pole | parent | 46.68% | 76.45% | 119112 | 28048 | 104013 |
| pole | control | 69.67% | 78.58% | 119112 | 25514 | 40743 |
| pole | balanced | 60.81% | 77.93% | 119112 | 26288 | 59815 |

<details>
<summary>모든 49칸 confusion 보기: 행=정답, 열=예측</summary>

### parent

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 11286124 | 5243 | 38926 | 523 | 34290 | 21291 | 92104 |
| road | 9911 | 11610003 | 16992 | 695 | 0 | 253 | 810 |
| sidewalk | 50267 | 4116 | 3794897 | 0 | 0 | 36 | 6561 |
| lane_marking | 125 | 25 | 1 | 45181 | 0 | 575 | 10 |
| traffic_light_fixture | 10691 | 0 | 0 | 0 | 31020 | 88 | 4415 |
| traffic_sign | 1076 | 0 | 12 | 4 | 0 | 1710 | 113 |
| pole | 24519 | 202 | 2560 | 0 | 681 | 86 | 91064 |

전체 픽셀 정확도: 98.80%.

### control

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 11374883 | 1446 | 30146 | 496 | 10372 | 25548 | 35610 |
| road | 6156 | 11625545 | 6064 | 468 | 0 | 120 | 311 |
| sidewalk | 25282 | 15085 | 3812404 | 0 | 0 | 56 | 3050 |
| lane_marking | 64 | 5 | 0 | 45714 | 0 | 133 | 1 |
| traffic_light_fixture | 4876 | 0 | 0 | 0 | 39641 | 23 | 1674 |
| traffic_sign | 953 | 0 | 4 | 0 | 1 | 1860 | 97 |
| pole | 21614 | 139 | 2873 | 0 | 792 | 96 | 93598 |

전체 픽셀 정확도: 99.29%.

### balanced

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 11347594 | 1239 | 42950 | 475 | 10917 | 23394 | 51932 |
| road | 6356 | 11605349 | 25629 | 637 | 0 | 97 | 596 |
| sidewalk | 31290 | 699 | 3818914 | 0 | 0 | 46 | 4928 |
| lane_marking | 71 | 26 | 0 | 45749 | 0 | 70 | 1 |
| traffic_light_fixture | 4797 | 0 | 0 | 0 | 39116 | 49 | 2252 |
| traffic_sign | 1047 | 0 | 1 | 0 | 8 | 1753 | 106 |
| pole | 21987 | 141 | 3259 | 0 | 850 | 51 | 92824 |

전체 픽셀 정확도: 99.13%.

</details>

## holdout_town02_p02: 232장 전체 화면

| 분류 | 기존 IoU | control IoU | balanced IoU | control − 기존 | balanced − 기존 | balanced − control |
|---|---:|---:|---:|---:|---:|---:|
| other | 95.09% | 95.75% | 95.14% | +0.661 pp | +0.050 pp | -0.611 pp |
| road | 98.66% | 99.13% | 98.14% | +0.469 pp | -0.522 pp | -0.992 pp |
| sidewalk | 92.62% | 94.92% | 92.15% | +2.292 pp | -0.472 pp | -2.763 pp |
| lane_marking | 98.10% | 98.89% | 98.85% | +0.798 pp | +0.757 pp | -0.041 pp |
| traffic_light_fixture | 39.23% | 39.44% | 37.75% | +0.212 pp | -1.481 pp | -1.693 pp |
| traffic_sign | 76.39% | 75.32% | 77.48% | -1.066 pp | +1.095 pp | +2.161 pp |
| pole | 43.82% | 46.72% | 44.87% | +2.898 pp | +1.041 pp | -1.857 pp |

### 무엇을 놓치고, 잘못 표시했는지

정밀도(P)는 표시한 것 중 맞은 비율, 재현율(R)은 실제 대상 중 찾아낸 비율입니다. FN은 누락, FP는 오탐 픽셀 수입니다. 작은 물체는 도로 점수와 별도로 보아야 합니다.

| 분류 | 모델 | P | R | 정답 픽셀 | FN | FP |
|---|---|---:|---:|---:|---:|---:|
| other | parent | 98.56% | 96.43% | 22157459 | 791293 | 312873 |
| other | control | 98.63% | 97.04% | 22157459 | 656344 | 298641 |
| other | balanced | 98.47% | 96.56% | 22157459 | 761244 | 332693 |
| road | parent | 99.59% | 99.06% | 21689357 | 204778 | 87531 |
| road | control | 99.63% | 99.50% | 21689357 | 109344 | 80664 |
| road | balanced | 99.72% | 98.40% | 21689357 | 346295 | 59230 |
| sidewalk | parent | 93.69% | 98.79% | 8773163 | 106302 | 583928 |
| sidewalk | control | 95.96% | 98.86% | 8773163 | 99887 | 364755 |
| sidewalk | balanced | 93.13% | 98.87% | 8773163 | 99078 | 639655 |
| lane_marking | parent | 98.23% | 99.86% | 108805 | 147 | 1963 |
| lane_marking | control | 98.99% | 99.90% | 108805 | 113 | 1104 |
| lane_marking | balanced | 98.97% | 99.88% | 108805 | 129 | 1133 |
| traffic_light_fixture | parent | 50.16% | 64.30% | 90724 | 32392 | 57962 |
| traffic_light_fixture | control | 46.82% | 71.46% | 90724 | 25897 | 73629 |
| traffic_light_fixture | balanced | 44.29% | 71.89% | 90724 | 25498 | 82055 |
| traffic_sign | parent | 81.53% | 92.38% | 58074 | 4428 | 12155 |
| traffic_sign | control | 78.56% | 94.80% | 58074 | 3017 | 15022 |
| traffic_sign | balanced | 81.85% | 93.55% | 58074 | 3744 | 12045 |
| pole | parent | 57.10% | 65.33% | 575218 | 199403 | 282331 |
| pole | control | 60.64% | 67.05% | 575218 | 189514 | 250301 |
| pole | balanced | 57.00% | 67.82% | 575218 | 185109 | 294286 |

<details>
<summary>모든 49칸 confusion 보기: 행=정답, 열=예측</summary>

### parent

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 21366166 | 82284 | 398022 | 24 | 52217 | 10588 | 248158 |
| road | 35532 | 21484579 | 156341 | 1802 | 2976 | 185 | 7942 |
| sidewalk | 78781 | 5080 | 8666861 | 1 | 0 | 136 | 22304 |
| lane_marking | 106 | 23 | 0 | 108658 | 0 | 18 | 0 |
| traffic_light_fixture | 28398 | 0 | 0 | 0 | 58332 | 209 | 3785 |
| traffic_sign | 3975 | 15 | 36 | 136 | 124 | 53646 | 142 |
| pole | 166081 | 129 | 29529 | 0 | 2645 | 1019 | 375815 |

전체 픽셀 정확도: 97.50%.

### control

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 21501115 | 73234 | 280773 | 51 | 68864 | 13856 | 219566 |
| road | 35460 | 21580013 | 60233 | 1020 | 1991 | 4 | 10636 |
| sidewalk | 75103 | 7192 | 8673276 | 0 | 0 | 31 | 17561 |
| lane_marking | 98 | 0 | 0 | 108692 | 0 | 15 | 0 |
| traffic_light_fixture | 23320 | 0 | 0 | 0 | 64827 | 116 | 2461 |
| traffic_sign | 2784 | 57 | 5 | 33 | 61 | 55057 | 77 |
| pole | 161876 | 181 | 23744 | 0 | 2713 | 1000 | 385704 |

전체 픽셀 정확도: 97.97%.

### balanced

| 정답 / 예측 | other | road | sidewalk | lane_marking | traffic_light_fixture | traffic_sign | pole |
|---|---:|---:|---:|---:|---:|---:|---:|
| other | 21396215 | 56722 | 361879 | 15 | 75966 | 11170 | 255492 |
| road | 75345 | 21343062 | 254700 | 1089 | 1821 | 4 | 13336 |
| sidewalk | 75103 | 2331 | 8674085 | 2 | 0 | 30 | 21612 |
| lane_marking | 105 | 7 | 0 | 108676 | 0 | 17 | 0 |
| traffic_light_fixture | 21805 | 0 | 0 | 0 | 65226 | 105 | 3588 |
| traffic_sign | 2908 | 106 | 11 | 27 | 434 | 54330 | 258 |
| pole | 157427 | 64 | 23065 | 0 | 3834 | 719 | 390109 |

전체 픽셀 정확도: 97.34%.

</details>

## 해석 범위와 다음 단계

정답이 없는 분류의 IoU·재현율은 미측정이며 오탐은 남깁니다. 예측이 없는 분류의 정밀도도 미측정입니다. 미측정을 0점이나 성공으로 바꾸지 않았고, 새 합격 기준을 만들지 않았습니다.

control − 기존은 같은 알려진 자료에서 추가 학습 후의 변화입니다. balanced − control은 장면·정답·학습 순서·crop이 같은 두 팔 사이에서 OFF/ON 혼합이 준 영향입니다. 양수를 일반화 성공으로, 일부 개선을 모든 분류 개선으로 확대하지 않습니다.

다음 지도 작업은 작은 물체의 거리·크기별 누락/오탐과 모든 480개 paired 장면의 appearance 영향 진단, 새 지역·날씨·rig의 고정 DEV 확보가 우선입니다. HD AMG에는 실제 보정·위치·깊이/삼각측량, 차선 vector·연결 topology와 정적 landmark의 다중 프레임 검증이 별도로 필요합니다.

새 RGB 960장은 같은 480개 물리 장면의 두 조건입니다. 독립 장소 960개가 아닙니다. [기존 결과](README.md)의 원자료 미승인·실패 기록·전체 결과를 변경하지 않았습니다.

## 원본 추적

모델 추론·Torch·pickle·NPZ/PT 역직렬화 없이 저장된 confusion과 원본 바이트 해시만 읽었습니다. 비교표는 고정 source/caller pins를 확인한 연구 진단입니다.

- `/home/a/autoware_e2e/docs/assets/validation/2026-10-02/map_paired_balanced_results_v1/results.json` — `29019d4592c8dd8c497d17f4fe0273002fa13b41efeec955fbe4f13d545b67b1`
- `/home/a/autoware_e2e/tools/e2e/publish_map_paired_balanced_results_20261002.py` — `33c717ddf51c1dc6aa2e56c15c8e131d5866983c97fc19b0421a5c0bf9ba1808`
- `/home/a/autoware_e2e/docs/assets/validation/2026-10-02/map_paired_balanced_results_v1/source/publish_map_paired_balanced_results_20261002.py` — `33c717ddf51c1dc6aa2e56c15c8e131d5866983c97fc19b0421a5c0bf9ba1808`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1/summary.json` — `fa9749d67b27eb8169473350289ba61530f324736c61bb3a269adc6b070027d0`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1/control/weights.npz` — `2284511d2b28c273ab731fcbc7b1326c8e040f4c1d84d57faacf2df86afd8310`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1/control/checkpoint.pt` — `da56f5113b60ec1188ab6a53b1e78f7bacdde70b140021bfd50f2507c873617f`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1/balanced/weights.npz` — `a2eca2f5becc316f1b4ae34799d7def899e6d5426a2b33286357833c3703a57c`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-02/map_paired_balanced_completed_run_v1/balanced/checkpoint.pt` — `ddbc4de4a4eb9569527db8da1e5b927c7c5512fb17e3be3b4a2ff99f36854f66`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1/B/metrics.json` — `0cc82311b89d8dc37697ff65da6e1d8e38f58f8cb225887af345cdfdc4511171`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1/summary.json` — `127c15564bd2180eee4061fd6993f52f00bac0dcbc65aecc6505b5dae425b606`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1/B/weights.npz` — `ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e`
- `/home/a/autoware_e2e/storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1/B/checkpoint.pt` — `410358363d1c9cdc5551a9cdc3f37fa1bd336a1c3fd17ea64e9a71cd11d74abf`
