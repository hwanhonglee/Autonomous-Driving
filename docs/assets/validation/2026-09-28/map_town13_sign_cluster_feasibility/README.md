# Town13 표지판 3개 구간: XODR 거리·연결성 사전 조사

<!-- HH_260928 - Keep this source-only screen separate from live route catalogs and camera evidence. -->

**판정: 기존 240프레임 Town13 정찰로 표지판 3개 DEV 목표를 진행하지 않는다.** 소스 `Town13.xodr`의 SHA256 `5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f`를 확인하고 읽기 전용으로 분석했다. CARLA/UE4Editor, 카메라, GPU 0/1, SSH, 패키지 설치, 미사용 TEST 맵은 사용하지 않았다. 결과는 [XY 필요조건](report.json)과 [방향성 도로 연결 후보](directed_topology.json)에 고정했다. 기존 v2 카탈로그·촬영 결과를 수정하지 않았다.

| 질문 | 이번 결과 | 의미 |
| --- | ---: | --- |
| 전체 정지/양보 XODR 기록 | 1,084개: `Sign_Stop` 1,062, `Sign_Yield` 22 | 서로 다른 `road_id:signal_id` 기록 수. RGB에서 서로 다른 물체 수가 아니다. |
| 3개 표지판이 있는 같은 road ID | 0곳 | 세 표지판을 보려면 적어도 도로 간 연결이 필요하다. |
| 250m 이내를 *배제할 수 없는* XY 대표 3개 조합 | 623개 | 각 가능한 가운데 표지판에서 가장 가까운 두 기록을 택한 Euclidean 거리 **필요조건**. 모든 가능한 3개 조합의 총수가 아니다. 교차로의 반대 진입 방향도 포함한다. |
| XODR road/junction 연결과 `+`/`-` 방향을 따른 3개 기록 후보 | 219개 | 250m 구간에서 양끝 각 20m를 허용하는 **후보**. 차로 수준 연결, CARLA 경로 생성, 방향별 도로 법규는 미검증이다. |
| 가장 짧은 방향성 후보 | 첫째 `22:20022` → `6233:20041` → `607:20040`, 표지판 간 108.309m, 양끝 각 20m 포함 최소 148.309m | 연결 경로가 실제 차량 차로에서 가능한지, 표지판 세 물체가 화면에 보이는지 아직 모른다. |
| 기존 240장·10Hz = 24초 거리 기준 | 9km/h 약 60m, 15km/h 약 100m | 9/15km/h 일정 속도일 때의 참고값이며 정차·감속은 거리를 줄인다. 방향성 후보 중 양끝 포함 100m 이하 **0개**. 더 강하게, 최단 표지판 간 108.309m 자체도 100m를 초과한다. |

XY 계산은 각 표지판이 미지의 경로 중심선에서 최대 8m 떨어질 수 있다고 허용해 `20+20+max(0,d₁−16)+max(0,d₂−16)`을 하한으로 둔다. 이 조건을 통과해도 도로가 같은 방향으로 연결된다는 결론은 나오지 않는다. 두 번째 계산은 XODR road의 `successor`/`predecessor`, junction의 `incomingRoad`/`connectingRoad`, 표지판 `orientation`을 사용한다. **차로별 laneLink 연속성과 물리적 경로 생성은 증명하지 않으므로** 219개를 실행 가능한 주행 경로 수라고 부르지 않는다. 상위 후보 중 표지판은 교차로와 도로 시작/끝에 가까워 별개의 RGB 객체인지도 확인해야 한다.

## 다음 단계의 사전 등록 게이트

지금의 무작위 60–140m·32개 v2 카탈로그를 단순히 120–250m로 늘려 재실행하는 것은 **표지판 3개를 겨냥한 탐색이 아니다**. v3 소유 카탈로그 실행을 아직 승인하지 않는다. 다음 구현은 다음 순서다.

1. 고정된 후보 2개 — `22:20022 → 6233:20041 → 607:20040`(108.309m)와 `183:20019 → 1323:20017 → 957:20013`(108.501m) — 에 대해 XODR `laneLink`, CARLA 주행 차로 및 시작·끝 각각 20m 여유를 검증한다. 경로가 120–250m 안에 들어오고 **서로 다른 신호 ID 세 개를 같은 방향으로 밟는지**를 확인한다. 이것도 RGB 증명은 아니다.
2. 위 검증을 통과한 경우에만, 정확한 `town13`, `validation`, `source_editor_0915_4ws`, `/Game/Carla/Maps/Town13/Town13`, 시나리오·seed·쌍 수·120–250m/선호 길이·endpoint 생성/검색 정책·required road/sign 순서·최대 trace·새 출력 경로를 코드와 문서에 **실행 전에 고정**한다. 현재 카탈로그 도구에는 required-road/sign 필터가 없으므로 이를 먼저 구현·단위 테스트해야 한다. `--allow-map-load`는 금지하고, 기존 v2 출력은 덮어쓰지 않는다. GPU/포트/프로세스/lock은 실제 owner 실행 직전 확인한다. 이 단계는 카메라 없는 카탈로그만 만든다.
3. 카탈로그가 실제 경로를 만들면 해당 route의 정확한 SHA, 차량 속도와 카메라 프레임 수를 수집 전에 잠근다. 240프레임 조건은 15km/h에서 최단 표지판 간 거리에도 부족하므로, 안전한 속도 범위에서 **실제 경로 거리와 정차 시간을 기준으로** 더 긴 촬영 프레임 수를 산정해야 한다. 그 뒤 별도 source-editor camera owner와 Town13 전용 mapping-probe scope를 만들고, 동일 frame ID의 전체 화각 RGB·semantic·depth에서 **서로 다른 실제 표지판 ≥3개**를 확인한다.

현재는 어느 단계도 카메라 촬영, 독립 DEV 자격, 지도 모델 학습/승격 또는 Autoware 주행 성공을 인정하지 않는다. **이 오프라인 필터에서 100m 이하 후보가 0개라는 결과는 다른 속도·더 긴 촬영·렌더 객체까지 포함한 전면적인 불가능성 증명이 아니다.** 기존 Town13 v2의 여섯 1표지판 경로와 [촬영 전 게이트](../../2026-09-27/map_town13_scout_prep/PLAN.md)도 그대로 유지한다.

재현 명령은 다음과 같다. 두 결과 파일은 새 파일로 한 번만 작성하고 재실행 시 덮어쓰기를 거부한다.

```bash
python3 tools/workspace_python.py -m scripts.e2e.audit_town13_sign_cluster_feasibility
python3 tools/workspace_python.py -m scripts.e2e.audit_town13_sign_directed_topology
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_audit_town13_sign_cluster_feasibility.py \
  learning/tests/test_audit_town13_sign_directed_topology.py
```
