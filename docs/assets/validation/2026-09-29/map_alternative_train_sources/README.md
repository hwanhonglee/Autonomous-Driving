# 지도 도로·차선 학습의 기존 원본 후보 감사 — 2026-09-29

<!-- HH_260929 - Keep prospective map TRAIN sources separate from dataset admission and independent validation. -->

**결론:** 로컬에 이미 촬영됐지만 D0/D1에 사용하지 않은, RGB·CARLA semantic·깊이·카메라 자세가 짝지어진 240프레임 수집본은 **3개**다. 도로 `Road=1`과 차선 `RoadLines=24`가 있는 첫 후보는 **Town02_Opt p00**이다. 다만 이는 *후보 발견*이며 새 데이터 편입·학습·성능 개선이 아니다. 현재 [사이트 분할 V1](../../../../../learning/config/site_split_guard_20260929.json)의 지도 사용 권한은 Town01/Town02_Opt의 **기존 TRAIN 재사용**뿐이고 `independent_dev_sites=[]`다. 새 에피소드의 QA와 별도 편입 결정을 통과하기 전 `train_eligible=false`; 승인된 독립 DEV는 **0곳**이다.

후속 [p00 시작·중간·끝 전체 화각 시각 QA](visual_qa/README.md)는 RGB 도로·황색선과 oracle 태그 1·24의 정합을 **조건부 PASS**로 관찰했다. 232장 전체의 시각 승인이나 새 TRAIN 편입 판정은 아니다.

<!-- HH_260929 - Record the later CPU research package without revising the original source-audit verdict. -->
후속 [좁은 p00 연구 계약](../../../../../learning/config/map_town02_p00_research_scope_20260929.json)(SHA-256 `c4870d042a074f559be7ed841ba0351be007d2d562d9a0b7567db2ac95b719b0`)은 **한 번의 탐색 A/B 학습만 별도 허용**했다. 이에 따라 원본·D0/D1을 바꾸지 않고 [새 CPU 데이터 패키지](../../../../../storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1/manifest.json)를 만들었다. manifest SHA-256은 `f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834`, [원본→저장본 재검증 영수증](../../../../../storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1/verification.json)은 SHA-256 `625eb5e700385d752a35f1e49d74aa6e127e2dfac3f3809a5e52a492f9c82fcc`다. A 497, B 729, p02 경로 보류 232, Town01 탐색 DEV 118장이다. 이 문단의 패키지 생성 당시에는 아직 학습하지 않았으며 정식 데이터 편입도 아니다.

<!-- HH_260929 - Append the completed research result without rewriting the prior source-inventory audit. -->
이후 실제 GPU 0 [탐색 A/B와 독립 결과 감사](../map_town02_p00_route_holdout/AUDIT.md)를 완료했다. p02 차선은 **+3.31%p**였지만 Town01 차선은 **−2.50%p**로 퇴행해 사전 관문에 실패했다. [전체 화각 PNG/GIF](../map_town02_p00_route_holdout/VISUALS.md)도 보관했다. **새 모델 채택·정식 TRAIN 편입·독립 DEV 승인 모두 없음**이다. 아래의 원본 후보 표와 당시 조사 범위는 학습 전 증거로 유지한다.

## 무엇을 대조했나

[D0/D1 패키지 manifest](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2/manifest.json) SHA-256 `313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6`와 [Town01 부모 manifest](../../../../../storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json) `e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c`를 고정했다. D0는 Town01 직진 114장·p03 187장·p01 196장, 총 **497장**이다. D1은 그 497장을 그대로 두고 Town02_Opt p02의 위치 보존 **232장**을 붙인 **729장**이다. 두 배열의 SHA-256은 각각 `69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace`, `4e2f7df61814a7e8aade88877482506a30e41c9fff0b5e6b7c3182193ff7d8f8`이며, [기존 전수 감사](../map_lane_train_audit/README.md)가 원본과 배열을 대조했다.

로컬 `storage/experiments/{training,validation}`의 `mapping/summary.json`에서 Town01·Town02_Opt를 전수 열거하면 **11개**다. 그중 D0의 3경로와 D1의 p02는 위 사용 이력에 속한다. Town01 기존 DEV와 같은 경로의 재촬영·짧은 정지 프로브는 새 TRAIN 후보로 세지 않았다. 남은 아래 3개에 대해 원본 240쌍의 센서 배열 크기(`semantic/depth 360×640×4`, `world_from_camera 4×4`)와 유한 자세를 확인하고, 보존 프레임의 원본 semantic 배열 채널 2에서 태그 1·24 픽셀을 직접 셌다. D0/D1 사용 RGB PNG **729개**의 파일 SHA-256 집합과 후보별 원본 **240개**를 비교한 정확한 파일 중복은 각 후보 **0개**였다. 다른 파일이라는 사실만으로 장면 독립성을 주장하지 않고 아래 차량 중심 위치도 비교했다.

| 우선순위 / 원본 | 보존 프레임 | Road=1 양성 / 픽셀 합 | RoadLines=24 양성 / 픽셀 합 | 경로·분리와 제한 |
| --- | ---: | ---: | ---: | --- |
| 1. [Town02_Opt p00](../../../../../storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1/mapping/summary.json) | 232/240 | 232/232 · 21,432,799 | 232/232 · 145,835 | D1의 Town02 p02와 최단 차량 중심 거리 **197.040m**. BasicAgent 이동 94.30m, 충돌 0, 재생 일치; 목적지 미도착. [기존 감사](../../2026-09-27/map_town02_train_scout/README.md)의 소형 표지판 조건은 **3/232**로 실패. |
| 2. [Town01 p02 우회전](../../../../../storage/experiments/training/2026-09-27/map_sign_native_p02_probe/mapping/summary.json) | 197/240 | 197/197 · 19,209,421 | 197/197 · 108,667 | D0 TRAIN과 최단 **73.515m**. 도로·차선은 있으나 같은 Town 내부 확장이다. 기존 소형 표지판 조건 0/197 실패. |
| 3. [Town01 p04 좌회전](../../../../../storage/experiments/training/2026-09-27/map_sign_native_p04_probe/mapping/summary.json) | 207/240 | 207/207 · 19,647,005 | 175/207 · 100,287 | D0 TRAIN과 최단 **73.456m**. p02와 일부 구간이 겹친다. 기존 소형 표지판 조건 0/207 실패. |

Town01 p02와 p04의 차량 중심 최소 거리는 **0.017m**다. 더 구체적으로 p02 보존 197위치 중 49개, p04 보존 207위치 중 50개가 서로의 경로에서 **1m 이내**다. 이는 일부 물리 구간의 공유를 뜻하며 두 전 경로가 같다는 뜻은 아니다. 두 수집은 다른 회전 방향을 보여 주지만 독립 *사이트* 두 곳으로 셀 수 없다. Town02 p00과 D1 p02는 같은 Town에서도 차량 중심이 197m 이상 떨어진 구간이다. 시야·표지 객체의 완전한 비중복은 차량 중심 거리만으로 증명되지 않는다.

위 세 후보의 원본 요약은 모두 `COMPLETE_ORACLE_PLUMBING_ONLY`, Epic 화질, CARLA BasicAgent, 240 RGB PNG/240 센서 NPZ/240 자세 기록, 충돌 0, `live_replay_exact_match=true`다. [Town02 p00 감사 JSON](../../2026-09-27/map_town02_train_scout/summary.json) SHA-256 `8467dcf470e5e4cef2f9aee884736127989c13b19f03254d43a6448824f2ac47`와 [Town01 p02/p04 감사 JSON](../../2026-09-27/map_sign_probe/summary.json) `a6de99f1ac6e09187cd01fba1a33df9f727240b6bd856e1c64d0237f667f3c43`가 소유 실행·경로·기존 표지판 판정을 보존한다. 이 수집은 oracle 라벨과 깊이·자세를 기록했지만 지도 모델은 현재 **RGB만 입력**한다. 태그의 픽셀 존재는 RGB에서 차선 도색을 정확히 식별할 수 있음을 보증하지 않는다.

## 다음 실험과 통과 조건

1. Town02 p00의 **고정 시작·중간·끝 전체 화각**에서 RGB와 태그 1/24의 경계, 가림·횡단선·미도색 구간을 원본 해시와 함께 육안 점검한다. 원본 기록 SHA-256은 `9c74b16aeafe4c4e98a7f3f95ef0d98d667e835eb8bdd28cee86d39a65be0985`이고 원본 수집 경로 SHA-256은 `520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b`다. 소형 표지판 실패는 그대로 유지하고 이번 질문을 도로·차선 일반화로 사전 등록한다.
2. 그 점검을 통과하면 같은 모델·2000 step·seed·손실·Town01 기준 클래스 가중치로 **A=Town01 기존 497장**, **B=같은 497장+Town02 p00 232장**의 짝 실험을 별도 manifest/체크포인트로 준비한다. 기존 D1에 들어간 **Town02 p02 전체 경로**를 두 팔에서 제외한 *내부 경로 보류*로 평가하면, 새 p00의 학습 효과를 p02에서 볼 수 있다. p02는 과거 D1 학습과 설계 과정에 노출됐으므로 이를 새 독립 DEV나 봉인 TEST로 부르지 않는다. 정확한 지표·허용 퇴행·seed 수는 첫 학습 **전에** 등록해야 한다.
3. 내부 실험이 좋아도 독립 DEV는 현재 **0곳**이다. 사이트 역할을 미리 확정한 새 촬영·원본 QA·미사용 DEV를 확보하기 전에는 Town05 반복 DEV로 추가 후보를 고르거나 모델을 승격하지 않는다. HD map, Autoware 주행, 실차·10Hz 승인은 모두 미확인이다.

이 문서는 로컬 보관 원본·manifest의 읽기 전용 조사다. CARLA, GPU, SSH, 재학습, 데이터 편입, DEV/TEST 예측은 실행하지 않았다.
