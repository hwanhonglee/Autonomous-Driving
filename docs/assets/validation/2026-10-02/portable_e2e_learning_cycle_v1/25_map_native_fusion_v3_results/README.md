# 지도 인식 구조 변경: 좋아진 부분과 남은 표지 누락

**기존 진단 전체 23/27 통과, 4개 미달. 모델 자동 채택·기능 승인은 하지 않습니다.**

같은 10,000회 부모에서 기존 구조와 세밀한 영상·주변 맥락을 결합한 후보를 각각 20,000회 추가 학습했습니다. 각 160,000회 입력 노출이며 자료·순서·crop·원래 CE 손실은 같습니다. 반복 노출은 새로운 장소의 증가가 아닙니다.

후보가 표지 오탐을 줄였더라도 재현율(실제 표지를 찾아낸 비율)의 4개 기준은 미달입니다. 아래는 부모·대조군·후보의 전체 수치와 모든 실패입니다. 이미 본 Town01 118장·Town02 232장 전체 350장을 검산했으며 신규 독립 장소는 0개입니다.

RGB 7종 인식 연구이지 신호 상태·좌표 누적·차선 연결을 갖춘 정밀지도 AMG 완성품이 아닙니다. 실제 Autoware/RViz 화면이 아닌 기록 기반 진단 그림입니다. 주행 메인 0/9·하위 0/30, 실시간 100ms 성능은 미측정입니다.

## 기존 기준 27개를 빠짐없이 확인

| 기준 | 결과 |
|---|---|
| Town01_sign_precision_vs_parent | 통과 |
| Town01_sign_recall_vs_parent | 미달 |
| Town01_other_to_sign_FP_reduction_vs_parent | 통과 |
| dev_town01_road_retention_vs_parent | 통과 |
| dev_town01_lane_marking_retention_vs_parent | 통과 |
| dev_town01_traffic_light_fixture_retention_vs_parent | 통과 |
| holdout_town02_p02_road_retention_vs_parent | 통과 |
| holdout_town02_p02_lane_marking_retention_vs_parent | 통과 |
| holdout_town02_p02_traffic_light_fixture_retention_vs_parent | 통과 |
| Town02_sign_IoU_retention_vs_parent | 통과 |
| Town01_sign_precision_vs_control | 통과 |
| Town01_sign_recall_vs_control | 미달 |
| Town01_other_to_sign_FP_reduction_vs_control | 통과 |
| dev_town01_road_retention_vs_control | 통과 |
| dev_town01_lane_marking_retention_vs_control | 통과 |
| dev_town01_traffic_light_fixture_retention_vs_control | 통과 |
| holdout_town02_p02_road_retention_vs_control | 통과 |
| holdout_town02_p02_lane_marking_retention_vs_control | 통과 |
| holdout_town02_p02_traffic_light_fixture_retention_vs_control | 통과 |
| Town02_sign_IoU_retention_vs_control | 통과 |
| ADDED_Town01_sign_precision_strict_gain_vs_parent | 통과 |
| ADDED_Town01_sign_recall_strict_gain_vs_parent | 미달 |
| ADDED_Town01_sign_IoU_strict_gain_vs_parent | 통과 |
| ADDED_Town01_sign_precision_strict_gain_vs_control | 통과 |
| ADDED_Town01_sign_recall_strict_gain_vs_control | 미달 |
| ADDED_Town01_sign_IoU_strict_gain_vs_control | 통과 |
| ADDED_Town01_other_to_sign_FP_at_most_90pct_parent | 통과 |

## dev_town01: 전체 분류 겹침(IoU)

| 분류 | 부모 | 대조군 | 후보 | 후보−부모 | 후보−대조군 |
|---|---:|---:|---:|---:|---:|
| other | 98.59% | 98.93% | 99.03% | +0.44%p | +0.10%p |
| road | 99.74% | 99.85% | 99.87% | +0.12%p | +0.02%p |
| sidewalk | 97.88% | 98.65% | 98.76% | +0.88%p | +0.12%p |
| lane_marking | 97.51% | 99.49% | 99.33% | +1.82%p | -0.16%p |
| traffic_light_fixture | 69.09% | 73.75% | 76.48% | +7.39%p | +2.73%p |
| traffic_sign | 6.44% | 6.45% | 6.93% | +0.50%p | +0.48%p |
| pole | 58.55% | 64.94% | 68.22% | +9.67%p | +3.28%p |

재현율은 놓친 대상을, FP는 잘못 표시한 픽셀을 보여 줍니다. 증가·감소가 섞이므로 IoU만 보고 개선으로 판단하지 않습니다.

| 분류 | 부모 재현율 / FP | 대조군 재현율 / FP | 후보 재현율 / FP | 후보 재현율−대조군 | 후보 FP−대조군 |
|---|---:|---:|---:|---:|---:|
| other | 99.10% / 58945 | 99.38% / 53030 | 99.50% / 55457 | +0.12%p | +2427 |
| road | 99.89% / 16675 | 99.86% / 1616 | 99.88% / 1075 | +0.02%p | -541 |
| sidewalk | 98.87% / 39087 | 99.48% / 32707 | 99.49% / 28465 | +0.01%p | -4242 |
| lane_marking | 99.56% / 964 | 99.83% / 158 | 99.86% / 248 | +0.03%p | +90 |
| traffic_light_fixture | 85.78% / 11165 | 83.22% / 5935 | 83.69% / 4357 | +0.47%p | -1578 |
| traffic_sign | 63.81% / 25976 | 52.69% / 20884 | 46.79% / 16758 | -5.90%p | -4126 |
| pole | 78.58% / 40743 | 79.32% / 26368 | 78.91% / 18677 | -0.40%p | -7691 |

| 표지 지표 | 부모 | 대조군 | 후보 |
|---|---:|---:|---:|
| 정밀도(표시 중 진짜인 비율) | 6.68% | 6.85% | 7.53% |
| 재현율(진짜 중 찾은 비율) | 63.81% | 52.69% | 46.79% |
| other→표지 오탐 픽셀 | 25548 | 20791 | 16702 |

## holdout_town02_p02: 전체 분류 겹침(IoU)

| 분류 | 부모 | 대조군 | 후보 | 후보−부모 | 후보−대조군 |
|---|---:|---:|---:|---:|---:|
| other | 95.75% | 95.76% | 96.18% | +0.44%p | +0.42%p |
| road | 99.13% | 99.25% | 99.43% | +0.31%p | +0.18%p |
| sidewalk | 94.92% | 94.53% | 95.41% | +0.49%p | +0.88%p |
| lane_marking | 98.89% | 99.61% | 99.77% | +0.88%p | +0.17%p |
| traffic_light_fixture | 39.44% | 42.70% | 44.78% | +5.34%p | +2.09%p |
| traffic_sign | 75.32% | 78.30% | 79.82% | +4.50%p | +1.52%p |
| pole | 46.72% | 49.40% | 52.10% | +5.38%p | +2.70%p |

재현율은 놓친 대상을, FP는 잘못 표시한 픽셀을 보여 줍니다. 증가·감소가 섞이므로 IoU만 보고 개선으로 판단하지 않습니다.

| 분류 | 부모 재현율 / FP | 대조군 재현율 / FP | 후보 재현율 / FP | 후보 재현율−대조군 | 후보 FP−대조군 |
|---|---:|---:|---:|---:|---:|
| other | 97.04% / 298641 | 97.10% / 310755 | 97.53% / 310504 | +0.43%p | -251 |
| road | 99.50% / 80664 | 99.56% / 66577 | 99.69% / 55373 | +0.13%p | -11204 |
| sidewalk | 98.86% / 364755 | 98.99% / 414496 | 99.09% / 338782 | +0.10%p | -75714 |
| lane_marking | 99.90% / 1104 | 99.91% / 333 | 99.91% / 152 | +0.00%p | -181 |
| traffic_light_fixture | 71.46% / 73629 | 61.10% / 39105 | 61.76% / 34402 | +0.66%p | -4703 |
| traffic_sign | 94.80% / 15022 | 91.29% / 9637 | 90.92% / 8072 | -0.38%p | -1565 |
| pole | 67.05% / 250301 | 67.58% / 211705 | 67.72% / 172410 | +0.14%p | -39295 |

| 표지 지표 | 부모 | 대조군 | 후보 |
|---|---:|---:|---:|
| 정밀도(표시 중 진짜인 비율) | 78.56% | 84.62% | 86.74% |
| 재현율(진짜 중 찾은 비율) | 94.80% | 91.29% | 90.92% |
| other→표지 오탐 픽셀 | 13856 | 8798 | 7593 |

## 이전부터 고정한 동일 10개 전체 화각 장면

위: 실제 RGB·정답·부모. 아래: 기존 구조·새 구조. 정답은 모델 입력이나 생성 지도가 아니며 성공 장면을 다시 선택하지 않았습니다.

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

[results.json](results.json)에 49칸 confusion·모든 FN/FP·27개 판정과 전체 가중치/52개 원본 SHA를 보존했습니다. 저장 예측만 읽고 새 추론·Torch·pickle·GPU는 실행하지 않았습니다. 최종 숫자 가중치와 전량 학습 기록 검증은 고정 ROOT 완료 영수증의 검증이며 이 publisher가 다시 학습 기록을 해석한 것은 아닙니다.
