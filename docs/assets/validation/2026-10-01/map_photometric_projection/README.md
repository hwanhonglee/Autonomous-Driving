# 고정 프레임의 학습 예측 → BEV 증거 누적

<!-- HH_261001 - Report mixed fixed-recipe mapping results without changing thresholds or claiming HD-map quality. -->

36회 CPU 추론으로 confidence만 복구했고, **기존 저장 마스크와 모든 argmax 픽셀이 일치**했습니다. 이전 투영/누적 코드를 바꾸지 않고 각 수집의 **0/119/239만** 깊이·위치와 결합했습니다. 독립 재생은 추가 추론 없이 **36개 학습 관측, 최종 격자16개, PNG12개의 모든 픽셀**을 검산해 PASS입니다. 산출물 전체는 **3,556,971바이트(약3.39MiB)**로10MiB 제한 이내입니다. 테스트6개, 기존 endpoint 테스트 포함22개가 통과했습니다.

깊이와 위치는 **CARLA oracle**이며, semantic 정답은 별도 기준 지도에만 사용했습니다. confidence≥0.5, stride3, 깊이0.1–40m, 격자0.2m, 프레임별 cell/class 최대 confidence 투표를 그대로 유지했습니다. 높이 필터·빈 공간 보간·누락 프레임 채우기는 없습니다. 어두운 곳은 계속 **미관측/unknown**입니다.

## 결과: 도로 개선은 일부 이어지지만 차선 지도 개선은 보장되지 않음

아래는 **세 프레임을 누적한 격자의 dominant-label IoU**입니다. 전체240프레임 RGB 픽셀 IoU와 다른 지표이며, confidence 필터와 한 cell 안의 라벨 경쟁 영향을 함께 받습니다.

| CTrack | 도로 cell IoU | 차선 cell IoU | 보도 cell IoU |
| --- | ---: | ---: | ---: |
| LEFT | 77.065 → 83.841% | 9.690 → 6.641% | 27.546 → 45.123% |
| STRAIGHT | 41.075 → 48.251% | 11.074 → 11.268% | 11.953 → 14.471% |

LEFT 차선은 oracle dominant cell242개 중 일치 cell이 **25→17개로 감소**했습니다. STRAIGHT는 공통 관측 cell의 라벨 일치율이50.97→65.17%로 올랐지만, 관측 cell 겹침은76.09→68.25%로 내려갔습니다. 따라서 픽셀 분류 개선을 차선 지도나 관측 범위의 일괄 개선으로 주장할 수 없습니다. 불리한 결과도 그대로 보존했습니다.

TRAIN 재사용 비교에서도 TRUE 조건의 도로 cell IoU는 Town01 97.549→97.126%, Town02 98.026→97.379%로 소폭 하락했습니다. 새 대조군은 이전 Appearance 모델과 동일한 TRUE 학습 기준입니다. 이전 FALSE 학습 대조군과 혼동하지 않습니다.

## 고정 PNG 검토

12개 모두 확인했습니다. LEFT에서는 도로에 생기던 잘못된 보도 띠가 줄지만 노란 차선 증거가 부족합니다. STRAIGHT000/119에서는 여전히 도로의 큰 부분이 보도로 분류되며 candidate의 confidence 필터 이후 비어 있는 부분도 더 보입니다. TRAIN에서는 도로/보도 윤곽이 대체로 oracle과 비슷하지만 얇은 차선과 기둥/시설물 오탐은 남습니다.

BEV의 먼 곳에 보이는 수평 줄무늬는 stride3 깊이 투영의 성긴 표본입니다. 정지선이나 횡단보도가 아닙니다. 세 관측 사이의 큰 빈 간격도 실제237개 프레임을 누적하지 않았기 때문에 그대로 남겼습니다. 화면 밖의 cell은 격자 파일에 보존되지만 미관측 공간을 채우지 않습니다.

| 수집 | 고정000 | 고정119 | 고정239 |
| --- | --- | --- | --- |
| CTrack LEFT | [PNG](run_v1/ctrack_left_000_bev.png) | [PNG](run_v1/ctrack_left_119_bev.png) | [PNG](run_v1/ctrack_left_239_bev.png) |
| CTrack STRAIGHT | [PNG](run_v1/ctrack_straight_000_bev.png) | [PNG](run_v1/ctrack_straight_119_bev.png) | [PNG](run_v1/ctrack_straight_239_bev.png) |
| Town01 p01 TRAIN | [PNG](run_v1/train_town01_p01_000_bev.png) | [PNG](run_v1/train_town01_p01_119_bev.png) | [PNG](run_v1/train_town01_p01_239_bev.png) |
| Town02 p00 TRAIN | [PNG](run_v1/train_town02_p00_000_bev.png) | [PNG](run_v1/train_town02_p00_119_bev.png) | [PNG](run_v1/train_town02_p00_239_bev.png) |

원본 RGB와 의미분할 비교는 [이전 전체시야 패널](../map_photometric_endpoints/RESULTS.md)에 있습니다. 새 BEV 파일에는 RGB를 중복 저장하지 않았습니다.

## 증거와 한계

- [동결 계획](PLAN.md) / [사전등록](preregistration.json): SHA256 `f796b6e82486f20449c94805f121ee91f329b7793c1c4fd2cd2c940ea9b56331`
- [전체 투영·격자 통계와64개 artifact 해시](run_v1/summary.json): SHA256 `ab789f2b0b7bfb27b8f3efaf4a348421e1ba34905d8a490da8b56b4cf91504d1`
- [독립 재생 PASS](run_v1/independent_replay.json): 추가 추론0, 학습0, GPU·CARLA 실행0

CTrack은 계속 노출된 외부 진단 EVAL이며 TRAIN에 넣지 않았습니다. LEFT의 원래 owner FAIL과 당시 live-map 일치 확인 불가도 유지합니다. 이 결과는 oracle-assisted XY 의미 증거 배선입니다. 실제 센서 깊이·측위 검증, 독립 일반화, 전체 경로 커버리지, 차선 연결, 지면 추정, HD-map 완성 또는 주행 성공을 뜻하지 않습니다. 모델 승격이나 임계값 변경은 없습니다.
