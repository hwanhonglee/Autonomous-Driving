# Town02_Opt p00 도로·차선 원본 시각 QA

<!-- HH_260929 - Report fixed full-FOV source inspection without promoting the captured dataset. -->

**판정: 고정 화면과 원본 태그의 도로·차선 정합은 조건부 PASS.** 시작 `0000`, 중간 `0120`, 끝 `0239`를 원본 지표를 보기 전에 고정하고 [전체 화각 3행 비교](town02_p00_start_middle_end_full_fov.png)를 만들었다. 각 행의 왼쪽은 **원본 RGB 640×360**, 가운데는 CARLA oracle 도로=파랑·차선=노랑, 오른쪽은 원본 위 oracle 중첩이다. 패널은 원본 화각을 자르거나 차량 위치를 바꾸지 않았다. [0039의 어두운 도색 확인](town02_p00_0039_full_fov.png)은 전수 숫자 감사에서 발견한 뒤 추가한 *사후 진단*이며 고정 세 프레임 판정을 대체하지 않는다. 이 화면은 학습 모델 예측이나 Autoware 주행 화면이 아니다.

| 프레임 | Road=1 픽셀 | RoadLines=24 픽셀 | 육안 관찰 |
| --- | ---: | ---: | --- |
| [시작 0000](town02_p00_0000_full_fov.png) | 89,885 | 964 | 도로 면과 중앙 황색선의 oracle 위치가 RGB와 대응한다. |
| [중간 0120](town02_p00_0120_full_fov.png) | 91,400 | 209 | 멀어진 황색선도 태그 24에 표시되며 보도는 도로 마스크 밖에 있다. |
| [끝 0239](town02_p00_0239_full_fov.png) | 126,719 | **29** | 넓어진 교차부 도로가 태그 1에 들어가고 멀리 보이는 소수의 황색선만 태그 24다. 마지막 화면의 차선 양성량은 작다. |

원본 **240쌍**의 RGB/semantic/depth/pose 배열 모양을 다시 확인했고 위치 보존 **232장** 모두에 태그 1과 24가 있다. [기계 판독 결과](result.json)의 태그 1 합은 **21,432,799픽셀**, 태그 24 합은 **145,835픽셀**로 [기존 원본 감사](../../../2026-09-27/map_town02_train_scout/summary.json)에서 고정한 232장과 일치한다. `R≥150,G≥80,B≤30,R≥G+20`, 화면 `y≥90`이라는 단순 RGB 황색 단서는 태그 24의 **142,324/145,835 = 97.59%**와 겹치고, 황색 단서의 **142,324/142,462 = 99.90%**가 태그 24다. 이 색 조건은 원본을 본 뒤 도입한 **설명용 진단**이므로 정답 정확도나 독립 성능 지표가 아니다. 13/232장에서는 태그 24 픽셀 중 이 밝은 황색 조건을 만족한 비율이 90% 미만이다. 0039의 RGB와 중첩 화면을 보면 어두운 황색/주황 도색도 태그 24를 따라가므로, 그 13장을 라벨 오류로 자동 판정하지 않는다.

재현: 저장소 루트에서 `python3 tools/e2e/audit_map_town02_p00_road_lane.py`를 실행한다. 이 감사기는 V1 사이트 정책, 기존 p00 감사·소유 계획/결과·수집 요약/기록·경로의 SHA를 먼저 대조한 후 보존 원본을 읽고 **이 폴더의 PNG/JSON만** 생성한다. [결과 JSON](result.json) SHA-256 `62f354445943ab8931c0fbf3a14530e29d72ad22f19a5f4f214ab10579bb8396`, [전체 화각 묶음](town02_p00_start_middle_end_full_fov.png) SHA-256 `0e8b730ae1d7e358f4980cb19ea4c1bb674e16d9d96e7649cee771fb5482497a`다. 개별 원본 RGB·센서 해시는 결과 JSON의 `selected_frame_metrics`에, 그려진 각 PNG 해시는 `visual_sha256`에 있다. 원본·D0/D1·DEV/TEST·모델 체크포인트는 수정하지 않았다.

이 QA의 **불합격 기준**은 소유 경로·원본 해시 불일치, 240 원본 쌍이나 232 보존 프레임 누락, 동기화 배열/자세 오류, 보존 프레임의 Road/RoadLines 태그 결손, 고정 화면에서 RGB 도로·차선과 oracle이 명백히 다른 표면을 가리키는 경우다. 현재 고정 화면에서는 그런 결함을 보지 못했다. 그러나 3장+사후 1장의 육안 확인만으로 232장 전체의 가림·도색·교차로 경계를 승인할 수 없다. p00은 목적지에 도착하지 않은 **부분 경로**이고 과거 소형 표지판 게이트도 **3/232 FAIL**이다. 따라서 기존 QA 판정 `dataset_admitted=false`, `train_eligible=false`, `independent_dev_approved=false`를 유지한다. 여기서 `train_eligible=false`는 **일반·정식 TRAIN 편입 불가**를 뜻한다.

<!-- HH_260929 - Distinguish the later single research authorization from formal dataset admission. -->
후속 [p00 전용 연구 범위 계약](../../../../../../learning/config/map_town02_p00_research_scope_20260929.json)의 `exploratory_ab_train_authorized=true`는 이 QA 이후 [사전 등록한 한 번의 p00 경로 보류 A/B](../../map_town02_p00_route_holdout/PLAN.md)만 별도로 허용한다. 이는 기존 QA를 PASS로 바꾸거나 정식 데이터셋 편입·독립 DEV·자동 모델 승격을 허용하는 것이 아니다.
