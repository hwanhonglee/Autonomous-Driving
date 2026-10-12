# 지도 인식이 무너지는 조건을 같은 장면에서 재현했다

<!-- HH_261001 - Explain the same-frame appearance failure without claiming independent generalization. -->

**카메라 배치가 아니라 영상 후처리 조건만 달라도 인식이 크게 무너졌다.** 기존 TRAIN에 쓰인 두 경로에서 같은 순간·위치·화각의 RGB를 OFF/ON 두 개로 동시에 촬영하고, 같은 원본 semantic 정답으로 비교했다. 이것이 C-track 실패의 유일한 원인이라고 단정하지는 않는다.

각 경로 240프레임, 총 **480쌍의 RGB + 같은 시점의 semantic/depth**를 확보했다. 초기 실패는 비-RGB 센서에 지원되지 않는 속성을 요구한 것이었으며, 해당 실패와 코드는 보존했다. v2는 RGB에만 OFF/ON을 필수로 요구하고, depth/semantic은 실제 지원 속성 그대로 기록한다.

## 현재 지도 B 모델의 실제 수치

IoU는 예측 영역과 정답 영역이 얼마나 겹치는지 보는 값이며 높을수록 좋다. 아래 경로는 익숙한 TRAIN 경로이므로 높은 OFF 점수를 미학습 맵 성능으로 보면 안 된다.

| 경로 | RGB 조건 | 도로 IoU | 차선 표시 IoU | 보도 IoU |
| --- | --- | ---: | ---: | ---: |
| Town01 우회전 p01 | OFF — 기존 학습 조건 | 99.90% | 98.52% | 98.97% |
| 같은 경로·같은 순간 | ON | 16.13% | 9.11% | 0.77% |
| Town02 직진 p00 | OFF — 기존 학습 조건 | 99.89% | 99.07% | 98.72% |
| 같은 경로·같은 순간 | ON | 29.48% | 31.03% | 0.48% |

A도 두 경로의 7개 클래스 모두 ON에서 하락했다. 모든 프레임을 평가했고 좋은 장면만 골라내지 않았다. 총 **1,920개 전체 화면 예측, 442,368,000개 픽셀 예측**을 별도 코드로 원본 semantic에서 다시 집계해 모든 confusion matrix가 정확히 일치했다.

## 화면에서 확인한 것

사전에 고정한 첫/중간/마지막 프레임(0/119/239)의 OFF/ON 원본 12장을 모두 확인했다. OFF 영상은 하늘이 희고 건물·나무가 어둡게 보이며, ON은 조명·그림자가 복원되지만 일부 밝은 노면에는 강한 눈부심과 뿌연 하이라이트가 있다. 두 조건 모두 원본 그대로 보존했다. 후처리 스위치는 여러 렌더 효과를 함께 바꾸므로 감마 하나의 영향만 분리한 실험은 아니다.

- [Town01 첫 장면 전체 패널](../../../../../storage/experiments/validation/2026-10-01/map_paired_town01_p01_fixed_models_20261001_v2/town01_p01_000_paired_full_fov.png), [중간](../../../../../storage/experiments/validation/2026-10-01/map_paired_town01_p01_fixed_models_20261001_v2/town01_p01_119_paired_full_fov.png), [마지막](../../../../../storage/experiments/validation/2026-10-01/map_paired_town01_p01_fixed_models_20261001_v2/town01_p01_239_paired_full_fov.png).
- [Town02 첫 장면 전체 패널](../../../../../storage/experiments/validation/2026-10-01/map_paired_town02_p00_fixed_models_20261001_v2/town02_p00_000_paired_full_fov.png), [중간](../../../../../storage/experiments/validation/2026-10-01/map_paired_town02_p00_fixed_models_20261001_v2/town02_p00_119_paired_full_fov.png), [마지막](../../../../../storage/experiments/validation/2026-10-01/map_paired_town02_p00_fixed_models_20261001_v2/town02_p00_239_paired_full_fov.png).
- [Town01 수치 원본](../../../../../storage/experiments/validation/2026-10-01/map_paired_town01_p01_fixed_models_20261001_v2/summary.json), [Town02 수치 원본](../../../../../storage/experiments/validation/2026-10-01/map_paired_town02_p00_fixed_models_20261001_v2/summary.json), [전체 픽셀 독립 재집계](independent_pixel_recount_v2.json).

## 다음 학습

기존 지도 B 구조·시작 가중치는 두 학습 팔에서 동일하게 둔다. 기존 729장에 새 480장을 추가하되, 한쪽은 OFF 영상, 다른 쪽은 같은 프레임의 ON 영상을 쓴다. 각 팔 **1,209장·10,000 step**, 같은 표본 순서·잘라내기 조건·정답·optimizer로 맞춘다. 기존 조건 유지와 조명 조건 확장의 효과를 분리하기 위한 비교다.

새 자료는 두 기존 TRAIN 경로만 사용한다. C-track은 계속 평가용이며 학습에 섞지 않는다. 이미 관찰한 C-track 점수를 다시 재더라도 ‘처음 보는 독립 시험’이라고 부르지 않는다. 현재는 원본 수집·품질 검사·고정 모델 진단을 완료했고, 비교 학습용 등록/패키지와 실행 검사를 준비 중이다.

이 결과는 RGB 분할 모델의 진단이다. 신호등의 현재 색 판단, 신호와 차선의 연결 관계, 검증된 전역 HD 지도 자동 완성, 실차 운용은 별도 과제이며 완료했다고 세지 않는다.
