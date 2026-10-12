# 학습 지도 예측을 위치 지도에 넣는 첫 연결 검사

<!-- HH_260929 - Report a CPU-only three-frame RGB-prediction projection without claiming HD-map quality. -->

**결론:** 기존 AI v2 RGB 의미분할 모델의 예측을 기존 0.2m `SemanticEvidenceMap`에 실제로 누적했다. CARLA 정답 의미분할과 모델 예측은 서로 다른 출처의 [JSON 격자](run_v2/learned_evidence.json)·[기준 JSON](run_v2/oracle_evidence.json)으로 저장했고, 원본 3프레임과 예측 마스크에서 독립 재생 검증이 통과했다. 이는 이전의 *정답 센서만 지도에 넣는 배선*에서 한 단계 나아간 것이다. **지도 모델을 새로 학습하거나 성능을 승인한 것은 아니다.**

| 구분 | 이번에 실제 사용한 값 |
| --- | --- |
| 모델 입력 | Town01 BasicAgent 수집의 기존 RGB 640×360, 프레임 **0000/0090/0179**만 사용 |
| 모델 | 기존 AI v2 checkpoint SHA-256 `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380`; 사진측광 실패 후보는 미사용 |
| 투영 보조 | **CARLA 정답 깊이와 위치**. 모델에는 정답 semantic을 공급하지 않았고, 기준 지도를 만들 때만 사용 |
| 누적 | 3픽셀 간격, 깊이 0.1–40m, 모델 confidence ≥0.5, `other` 제외; 0.2m 격자 |
| 환경 | **로컬 CPU 2스레드만**. GPU·CARLA 서버·Autoware·SSH·패키지 설치 없음 |
| 승인 상태 | 독립 DEV 0개, TEST 미사용, 실차/폐루프/10Hz 실시간/HD-map 미승인 |

## 눈으로 확인하기

세 그림에서 차량(주황색)은 양쪽 지도 패널 **정중앙**에 고정된다. 위쪽은 같은 시각의 RGB와 실제 모델 예측 색상, 아래쪽은 CARLA semantic 기준과 모델 예측을 각각 같은 oracle 깊이·위치로 투영한 2D 누적 지도다. 어두운 영역은 관측되지 않은 곳이다. 3프레임 간의 나머지 **177장은 누적하지 않았으므로** 빈 곳을 주행해 매핑한 것처럼 해석하면 안 된다.

- [첫 프레임 전체 시야와 지도](run_v2/frame_0000_ego_centered.png)
- [중간 프레임 전체 시야와 지도](run_v2/frame_0090_ego_centered.png)
- [마지막 프레임 전체 시야와 지도](run_v2/frame_0179_ego_centered.png)

![중간 프레임 RGB, 예측, 차량 중앙 기준 지도 두 장](run_v2/frame_0090_ego_centered.png)

## 측정 결과

| 표본 프레임 | RGB 픽셀 도로 IoU | RGB 픽셀 차선 IoU | 기준 누적 cell | 모델 누적 cell | CPU 추론만 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0000 | 99.69% | 96.39% | 3,173 | 3,229 | 0.118s |
| 0090 | 98.82% | 96.92% | 7,088 | 7,143 | 0.102s |
| 0179 | 95.93% | 85.71% | 10,541 | 10,640 | 0.090s |

세 표본 누적 지도에서 관측 cell의 Jaccard는 **96.87%**(공통 10,422/합집합 10,759), 공통 cell 중 가장 큰 증거 라벨의 일치율은 **97.99%**였다. [AI v2 학습 manifest](../../../../../storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json) (SHA-256 `e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c`)의 `train.source_records`와 원본 frame·ego 좌표를 대조하면 **0000과 0090은 정확히 학습에 포함된 프레임**이다. 0179도 같은 수집 경로다. 즉 위 IoU는 독립 평가가 아니라 **학습 노출된 경로에서의 배선 검산**이며, 양쪽 지도가 정답 깊이·위치를 공유한다. 다른 Town 일반화나 독립 지도 정확도로 해석할 수 없다. 세 프레임 CPU *모델 추론 구간만* 0.09–0.12초였고, I/O·깊이 처리·누적·ROS/CARLA를 포함한 10Hz 측정은 아니다. 실제 주행 모델 승인 기능은 여전히 **9개 상위/30개 하위 중 0/30**이다.

작은 객체는 이미 이 세 장에서도 불안정하다. 첫 프레임의 신호등 기구 IoU는 **0%**, 표지판 **6.57%**, 기둥 **22.12%**였다. 도로·차선 숫자가 좋아도 사용자에게 필요한 신호/표지/규칙 인식을 입증하지 못한다.

## 재현·증거 경계

- 실행 전 [고정 계약](PLAN.md)에는 프레임 순서, 입력·모델·소스 해시, 임계값, 실패 시 미승격이 기록돼 있다. 최초 [v1 요약](run_v1/summary.json)은 그림과 셀 개수만 남긴 결과로 보존했다. [v2 요약](run_v2/summary.json)은 같은 입력/임계값으로 모델 [예측+확률 3개](run_v2/frame_0090_prediction.npz), 두 격자 JSON, PNG 3개와 각각의 SHA-256을 추가했다. v1을 덮어쓰지 않았다.
- `python3 tools/workspace_python.py tools/e2e/verify_learned_mapping_bridge.py`가 Town01의 TRAIN 노출 역할과 **8개 산출물 SHA, 원본/코드 해시, 3개 픽셀 혼동행렬, 두 지도 스냅샷, 차량 중심 좌표**를 예측 저장값과 원본 깊이/위치에서 재계산해 `VERIFIED_CPU_ONLY_THREE_FRAME_PLUMBING`으로 통과했다. 재검증은 추론하지 않는다.
- 이 출력은 차선 연결, 횡단보도/정지선 규칙, 신호 상태, 지도 좌표 정합, 정밀지도 자동 생성 결과가 아니다. 특히 CARLA 정답 깊이/위치 대신 사용할 실제 센서·측위 체인이 아직 검증되지 않았다. 모델의 Town05 어두운 도로·회색 차선 성능도 [별도 시험](../map_photometric_pilot/README.md)에서 미달이다.

**다음 우선순위:** [새 자료 출처 목록](../map_next_source_inventory/README.md)에 따라 C-track 또는 Town12에서 RGB/semantic/depth/pose를 **새 TRAIN 목적**으로 안전하게 수집할 수 있는지 먼저 확인한다. 동시에 새 미사용 독립 DEV를 보호·선언해야 한다. 그 전에는 이 세 장의 좋은 점수로 가중치를 업데이트하거나 HD-map/실차 성능을 승격하지 않는다.
