# 다음 우회전 site 탐색의 최소 확대 범위

<!-- HH_260929 - Propose only a split-safe next static search; do not promote a route or alter gates. -->

**제안 상태: 계획만 작성, 새 route 계산·수집 없음.** [이번 3-map 감사](README.md)의 30개 endpoint 조합에서는 물리 형상 PASS 62개 중 명목 10cm 차체 PASS가 0개였다. 같은 junction의 시작·끝 거리만 바꿔 이 침범을 해결했다고 간주하지 않는다. 다음 한 번의 정적 검색은 **새 비-TEST 지도 한 개**를 먼저 추가하고, 결과를 본 뒤에만 범위를 넓힌다.

| 순서 | 지도·endpoint 범위 | 분할과 중단 조건 |
| --- | --- | --- |
| 1 — TRAIN 정적 탐색 | `Town12` source OpenDRIVE에서 오른쪽 driving connector를 대상으로 `previous(u) × next(d)`, `u={60,70,80}m`, `d={100,120}m`의 **6개 scope**만 사전 고정한다. | [expert suite](../../../../../tools/e2e/carla_expert_suite.yaml)의 `town12: train, source_editor_required`를 *TRAIN 후보*로만 따른다. 기존 통합 분할에 Town12의 모델 노출이 잠기지 않았으므로 편입은 보류한다. 여섯 scope가 실패하면 같은 지도 전체 30-scope 재검색은 별도 계획으로 판단한다. |
| 2 — DEV는 조건부 | `Town13`은 suite의 `validation` 지도다. [기존 Town13 우회전 카탈로그](../../2026-09-27/map_town13_static_sign_rank/rank_v2.json)의 8개 경로가 모두 9km/h의 120m+6.4초 예산보다 짧다. 렌더 소유 문제가 해결되고 site 노출이 잠긴 후, 기존 catalog의 **한 물리 우회전 connector를 먼저 고정**하여 위와 같은 6개 endpoint scope로 연장 가능성만 정적 조사한다. | [240/240장 skydome 경고와 후속 editor 충돌](../../2026-09-28/README.md)이 남아 있어 현재 주행 수집·독립 DEV 인정은 중단한다. route ID/seed만 바꾸거나 8개를 독립 site로 세지 않는다. |

두 탐색 모두 packaged/source XODR 바이트와 검색 코드·선택 connector·새 출력 경로를 실행 **전에** 고정해야 한다. 원래 1m planner, 단일 RIGHT block, `speed_30kph` 물리 **형상** profile, 30–120m 창, 9km/h 기준 최소 146m(6.4초 미래 16m와 guard 10m 포함), 신호 진입부, 원래 Prius 명목 차체/10cm inset sweep을 유지한다. 정적 PASS여도 live XODR·실제 bbox 일치, 실제 `>0.5m/s` 연속 30초, 각 anchor의 64점/6.4초 미래, 원래 아홉 운동 한계, 충돌·실차체 차로·목표 ≤2.5m의 소유 원본 QA 전에는 수집·편입·학습을 승인하지 않는다.

**DEV 오염 경계:** [주행 연구 manifest](../../../../../storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json)는 Town03 우회전을 DEV로 사용했으나, 과거 지도 B/C가 Town03을 TRAIN에 노출했다. Town05_Opt는 [옛 suite TRAIN과 새 DEV 제안이 충돌](../../2026-09-27/drive_long_route_town05_dev_prep/PLAN.md)한다. 따라서 두 Town의 새 seed/endpoint는 독립 통합 DEV를 만들지 않는다. Town12를 탐색 후 성능에 따라 DEV로 바꾸거나 Town13을 TRAIN에 섞지 않는다. [최근 통합 분할 계획](../../2026-09-27/map_next_training_design/README.md)의 Town01·Town02 TRAIN / Town05 DEV 구상과 주행 manifest를 함께 대조해, 지도 family·route·source session·checkpoint 노출을 기계 판독형 split으로 잠그기 전에는 **독립 DEV 0개**로 표시한다.

Town02/Town05 표준판은 렌더 실패, Town11은 시각 학습 제외 POC, Town08/09는 로컬 자산 없음이다. Town04/Town15와 예약·held-out Town06/Town07/Town10HD_Opt는 이번 DEV 탐색에 사용하지 않는다. `Town01_Opt`는 확인된 통합 split 역할이 없어 위의 최소 확대에 넣지 않는다. 이 제외 근거는 [suite](../../../../../tools/e2e/carla_expert_suite.yaml), [비-TEST site 검토](../../2026-09-27/map_sign_dev_feasibility/README.md), [최근 분할 계획](../../2026-09-27/map_next_training_design/README.md)이다.
