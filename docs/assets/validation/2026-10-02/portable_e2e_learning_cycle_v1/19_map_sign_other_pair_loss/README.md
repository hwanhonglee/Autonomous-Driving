# 표지↔배경 손실 교정: 실제 결과와 원본 화면

전체 기존 평가 350장·27개 판정: **21/27, FAIL / 미채택**.

같은 10,000회 시작 모델에서 대조군·후보군을 각각 4,000회 추가 학습했습니다. 양쪽 실제 optimizer 노출은 각각 32,000회이며 CUDA 사전검사 8회 노출씩은 별도입니다. 영상·정답·crop·순서·FALSE 영상·원 CE는 같고 후보만 표지↔other 조건부 pair margin 손실 0.10을 더했습니다. 직접 보조 logit gradient는 두 분류뿐이지만 공유 특징을 통한 차선·재현율 퇴행은 가능합니다. 이전 표지 손실 실패를 덮거나 새로운 head·문턱값을 추가한 것이 아닙니다.

Town01 118장·Town02 232장 전체를 검산했습니다. 이미 사용한 장소에서 만든 적응 가설의 진단이며 신규 독립 장소는 0개입니다. 아래 10장은 결과 전에 고정한 인덱스입니다. 7분류 인식은 신호 상태·좌표 누적·차선 연결을 가진 정밀지도 AMG가 아닙니다. 메인 0/9·하위 0/30, 자동 채택 금지입니다.

미달 판정: Town01_sign_recall_vs_parent, dev_town01_lane_marking_retention_vs_parent, Town01_sign_recall_vs_control, dev_town01_lane_marking_retention_vs_control, ADDED_Town01_sign_recall_strict_gain_vs_parent, ADDED_Town01_sign_recall_strict_gain_vs_control

## dev_town01: 전체 분류 IoU

| 분류 | 부모228 | 대조군 | 손실 교정 후보 |
|---|---:|---:|---:|
| other | 98.59% | 98.67% | 98.73% |
| road | 99.74% | 99.83% | 99.77% |
| sidewalk | 97.88% | 98.28% | 98.15% |
| lane_marking | 97.51% | 98.41% | 96.42% |
| traffic_light_fixture | 69.09% | 66.51% | 69.69% |
| traffic_sign | 6.44% | 5.95% | 9.38% |
| pole | 58.55% | 61.91% | 57.72% |

| 표지 지표 | 부모228 | 대조군 | 후보 |
|---|---:|---:|---:|
| 정밀도 | 6.68% | 6.19% | 10.95% |
| 재현율 | 63.81% | 60.48% | 39.59% |
| other→표지 오탐 픽셀 | 25548 | 26283 | 9277 |

## holdout_town02_p02: 전체 분류 IoU

| 분류 | 부모228 | 대조군 | 손실 교정 후보 |
|---|---:|---:|---:|
| other | 95.75% | 95.58% | 95.64% |
| road | 99.13% | 99.16% | 98.93% |
| sidewalk | 94.92% | 95.41% | 93.76% |
| lane_marking | 98.89% | 99.21% | 99.23% |
| traffic_light_fixture | 39.44% | 38.08% | 40.12% |
| traffic_sign | 75.32% | 71.47% | 81.54% |
| pole | 46.72% | 45.87% | 46.58% |

| 표지 지표 | 부모228 | 대조군 | 후보 |
|---|---:|---:|---:|
| 정밀도 | 78.56% | 74.64% | 95.83% |
| 재현율 | 94.80% | 94.40% | 84.54% |
| other→표지 오탐 픽셀 | 13856 | 17324 | 2108 |

## 평가 전에 고정한 전체 화각 화면

위는 실제 RGB·정답·시작 모델, 아래는 대조군·손실 교정 후보입니다. 정답은 모델 입력이나 모델이 만든 지도가 아닙니다.

![dev_town01_0000.png](dev_town01_0000.png)

![dev_town01_0029.png](dev_town01_0029.png)

![dev_town01_0058.png](dev_town01_0058.png)

![dev_town01_0088.png](dev_town01_0088.png)

![dev_town01_0117.png](dev_town01_0117.png)

![holdout_town02_p02_0000.png](holdout_town02_p02_0000.png)

![holdout_town02_p02_0057.png](holdout_town02_p02_0057.png)

![holdout_town02_p02_0115.png](holdout_town02_p02_0115.png)

![holdout_town02_p02_0174.png](holdout_town02_p02_0174.png)

![holdout_town02_p02_0231.png](holdout_town02_p02_0231.png)

[results.json](results.json)에 전체 49칸 confusion, 7분류 FN/FP·정밀도·재현율, 27개 판정과 전체 가중치 SHA를 남겼습니다. 저장 예측만 읽었으며 새 추론·Torch·pickle·GPU를 사용하지 않았습니다.
