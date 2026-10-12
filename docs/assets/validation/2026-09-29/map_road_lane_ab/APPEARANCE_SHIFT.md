# 지도 차선 퇴행: 원본 색·양 분포 차이

<!-- HH_260929 - Treat repeatedly viewed Town05 DEV as diagnosis, never a new independent selection set. -->

**관측:** 기존 TRAIN과 반복해서 사용한 Town05_Opt DEV는 차선의 **양과 색**, 도로 밝기가 모두 크게 다르다. [고정 manifest](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2/manifest.json)의 RGB와 CARLA 7클래스 라벨에서 각 픽셀을 전수 집계했다. [계산 결과 JSON](appearance_diagnostic.json) SHA-256 `8e20d2b8f4f7d3509805e05049480c1603d4e3af3d2dd73b4a63b4305676bdb9`; [읽기 전용 계산기](../../../../../tools/e2e/audit_map_appearance_shift.py)는 모델 가중치·예측·TEST를 읽거나 새 학습을 하지 않는다.

| 픽셀 라벨 | Town01 TRAIN 497장 | Town02_Opt TRAIN 232장 | Town05_Opt 반복 DEV 232장 |
| --- | ---: | ---: | ---: |
| 도로 평균 RGB | (51.09, 51.28, 51.41) | (48.88, 49.07, 49.17) | **(17.18, 16.01, 13.98)** |
| 도로 명도 0–63 비율 | 80.11% | 81.64% | **99.99% 이상** |
| 차선 평균 RGB | (203.77, 128.14, 0.00) | (188.14, 111.95, 0.00) | **(100.44, 100.04, 91.10)** |
| 차선 명도 128–255 비율 | 74.07% | 54.48% | **25.93%** |
| 차선 픽셀 수 | 287,660 | 108,805 | **1,289,336** |
| 차선 픽셀/전체 화면 | 0.251% | 0.204% | **2.412%** |

명도는 `floor((77R+150G+29B)/256)`이고 `0–63`·`128–255`는 사전 고정한 단순 구간이다. 보관된 [Town01 수집기](../../../../../storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_town01_p03/provenance/scripts/e2e/collect_carla_mapping_probe.py)와 [Town05 수집기](../../../../../storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/provenance/scripts/e2e/collect_carla_mapping_probe.py)는 모두 `ClearNoon`, RGB 640×360, FOV 90°를 설정했고 실행 계획의 품질도 Epic이다. 그러므로 문서에 기록된 **설정값 자체**가 다른 것이 이 차이를 바로 설명하지는 않는다. 다만 collector 버전은 달랐고 표지 형태·페인트·조명·카메라 자동 노출의 각 원인을 분리하지는 못한다. 차선 라벨 수가 Town05에서 약 10배인 것도 **같은 물리 차선을 더 많이 그렸다**는 뜻은 아니다. 교차로의 횡단 정지선 등 CARLA `lane_marking` 태그 구성·촬영 경로가 다를 수 있다. 원본 전체 화각과 기존 D0/D1의 오류는 [대표 PNG](visuals_town05_dev_v1/town05_dev_115_road_lane_ab.png)에서 함께 볼 수 있다. 이 PNG는 Autoware 주행 화면이나 생성된 정밀지도가 아니다.

**해석:** 단순히 희소한 TRAIN 차선의 손실 계수를 2배로 만드는 것만으로 이 강한 외관·밀도 이동이 해결된다고 기대하기 어렵다. 이는 근거 있는 다음 *가설*이지 인과 판정은 아니다. [경로 보류 2배 A/B](../map_lane_weight_pilot/PLAN.md)의 첫 기준 팔은 Town01 같은 지도 경로에서 차선 IoU가 95%를 넘었고, 해당 보류 경로는 Town05 교차 사이트 문제를 그대로 재현하지 않는다. 그러므로 Town01 holdout의 작은 개선폭을 Town05 성능 회복/악화로 바꾸어 말하지 않는다.

## 다음 진척 관문

1. Town05를 TRAIN에 옮기거나 여러 배율을 같은 반복 DEV 점수로 고르지 않는다. 신규 TRAIN 후보는 주행·지도 공통 [분할 원장](../site_split_guard/README.md)에 먼저 등록하고, 원본 RGB/semantic·차선 객체 구성·차체/주행 자료 역할을 검수한다. 목표는 **어두운 노면과 무채색 차선**을 독립된 허용 TRAIN site에서 실제로 관측하는 것이다. 기존 Town01/Town02 프레임의 단순 복제는 새로운 도메인이 아니다.
2. 새로운 미사용 DEV는 사이트·경로·원본 해시·카메라 품질과 평가 기준을 모델 점수 보기 전에 잠근다. Town13은 현재 skydome/타일·카메라 crash 관문 때문에 사용할 수 없다. 기존 Town05를 다시 평가하면 탐색용 진단으로만 표시한다.
3. 다음 모델 A/B는 자료 추가 *또는* 명시적 photometric 변형 중 한 요인만 바꾸고, 다른 클래스 비퇴행·차선/도로의 **달성 가능한** 기준과 세 seed를 사전 등록한다. 내부 경로 보류에서 기준 차선 IoU가 이미 95%이면 무조건 +5%p처럼 수학적으로 불가능한 조건을 반복하지 않는다. 현재 자료만으로 새 모델의 교차-Town 개선이나 정밀지도 자동 생성은 주장하지 않는다.

재실행: `python3 tools/workspace_python.py tools/e2e/audit_map_appearance_shift.py --output /tmp/map_appearance_new.json` (새 경로만 사용). 집중 수학·마스크 시험 `python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_audit_map_appearance_shift.py`는 2개 통과했다.
