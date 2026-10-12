# Town12 junction 3742: 신호를 포함한 우회전 수집 전 점검

<!-- HH_260929 - Bind the proposed signal-aware episode to the pinned static evidence and keep admission closed. -->

**상태: 설계 점검만 완료.** [Junction 3742 파생 결과](junction3742_derivative.json)는 전체 628개 junction 탐색의 완료 보고서가 아니다. [선택 경로 그림](junction3742_supported_route_overview.png)의 경로 SHA256 `629f4bfc07758e5fe787f51fcd01cef6315bce69f36fa0f3530e058dcac0eb31`은 junction 3742, connector `(3745, section 0, lane -1)`, 상류 80m/하류 120m, 길이 221.57m, RIGHT 구간 80.32–100.94m다. 정적 명목 Prius 차체와 10cm inset sweep은 PASS이며 최소 inset 여유는 0.01959m, `production_lane_gate_supported=true`다. 해당 junction의 명목 PASS 8개는 endpoint 변형이며 독립 주행 장소 8곳이 아니다. 그중 이 connector의 4개만 production 차로 검사에 지원된다.

## 확인된 신호 자료와 그 한계

[원본 XML 분류](junction3742_approach_signal_classification.json)는 SHA256 `26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f`인 source Town12 OpenDRIVE에서 road 1062의 비동적 `StopLine2` (ID 35843, type -1), connector 3745 lane -1의 `turnRelation=Right` 신호 참조 (ID 21131), road 3743의 동적 `Signal_3Light_Post01` 정의를 확인했다. 이름이 `StopLine2`라는 사실은 정지 표지나 언제나 정지해야 한다는 증명이 아니다. XML의 신호 참조도 실제 TrafficLight actor ID, 실시간 색상, 해당 주행에 적용되는 제어 상태를 증명하지 않는다.

source CARLA `PythonAPI/carla/agents/navigation/basic_agent.py`에서 BasicAgent는 기본적으로 traffic light 무시가 꺼져 있고, `run_step()`이 차량과 **빨간 신호** 위험을 본다. 신호 위험 판단은 trigger waypoint와 ego waypoint의 road ID·방향·거리 등을 조건으로 한다. stop-line 기록과 connector 참조만으로 이 로직이 junction 3742에서 올바른 신호 actor를 고른다고 단정할 수 없다.

기존 [expert 수집기](../../../../../tools/e2e/collect_carla_vad_expert.py)는 `--observe-traffic-lights` 사용 시 같은 physics frame의 `Vehicle.is_at_traffic_light`, 연결된 신호 actor ID/상태, `BasicAgent._last_traffic_light` ID/상태를 `states.jsonl`에 읽기 전용으로 남긴다. 원래 기록에도 `basic_agent_stop_reason_proven=false`가 명시된다. 관련 actor와 XML ID의 대응, 차량 앞 범퍼와 정지선의 부호 있는 거리, 신호 변경과 진입 시각, 신호로 인해 제동했다는 원인은 현재 기록하지 않는다. 따라서 이 옵션만으로 신호 준수를 통과 처리할 수 없다.

## 실제 수집을 막는 현재 연결 문제

1. [통합 split 원장](../../../../../learning/config/site_split_guard_20260929.json)은 Town12 주행에 `offline_train_scout`만 허용한다. suite의 `train/source_editor_required` 표시는 수집 허가가 아니다. 현 상태는 `capture_authorized=false`, `dataset_admitted=false`다.
2. [기존 소유 실행기](../../../../../tools/e2e/run_owned_carla_expert_trial.sh)는 서버 시작 시 [packaged CARLA 실행기](../../../../../tools/e2e/run_carla_map.sh)를 고정 호출한다. [suite](../../../../../tools/e2e/carla_expert_suite.yaml)의 Town12는 `source_editor_0915_4ws` 및 `/Game/Carla/Maps/Town12/Town12`가 필요하므로 이 실행기로 Town12를 시작할 수 없다. [Town12 전용 source-editor 소유 프로브](OWNER_PROBE.md)는 고정 에셋 사전 점검이 통과했고 독점 시작·전체 맵/XODR 식별·종료 경로가 구현됐지만, 실제 UE4Editor 실행과 수집은 아직 하지 않았다. 이 프로브에는 driving collector가 없다.
3. 파생 JSON의 `route_sha256`은 정적 탐색 경로의 식별자다. expert 수집기가 요구하는 `schema_version=1` route JSON 파일과 그 **파일 SHA256**은 아직 없다. 수집기는 start/goal transform과 route points를 받아 live map에서 GlobalRoutePlanner를 다시 실행하므로, 선택한 connector·RIGHT 구간·route sequence의 동일성을 별도로 확인해야 한다.
4. 수집기는 `--expected-live-xodr-sha256`로 actor 생성 전에 live `carla.Map.to_opendrive()` 바이트를 검사하고 같은 바이트를 보관할 수 있다. Town12 실제 UE4Editor에서 이 값이 source XODR 해시와 일치하는지는 아직 측정되지 않았다. 일치하지 않으면 원인을 조사하고 수집을 중단해야 한다.

## 가장 작은 후속 구현 범위

1. **정적 최종 판정:** 실행 중인 V4가 628개 junction과 모든 물리 PASS 경로 판정을 마치고 `report_v4.json`을 낼 때, 선택 경로의 route SHA·명목 차체·signal 목록을 파생 결과와 대조한다. 현재 파생 결과를 전체 탐색 결론으로 승격하지 않는다.
2. **Town12 경로와 수집 연결:** [소유 프로브](OWNER_PROBE.md)의 `--execute`를 다른 CARLA 작업이 없는 시간에 한 번 실행하여 실제 source editor 시작·전체 맵/XODR·종료 영수증을 확인한다. 그 뒤 versioned route JSON과 사전 등록을 만들고 프로브의 고정 소스·잠금·PID 정리 패턴을 별도 수집 소유자에 연결한다. actor 생성 전에 live XODR을 확인하고, 첫 주행 명령 전 live planner가 connector 3745 lane -1과 고정 RIGHT 구간을 재현하는지 검사한다. 불일치 시 원본을 보존하고 실패로 기록한다. 기존 Town13 및 packaged 실행기는 수정 대상이 아니다.
3. **신호 계측 및 판정:** route 상의 StopLine2 위치를 차량 진행 방향의 거리로 고정하고, live TrafficLight actor와 XML ID 21131의 관계를 고유하게 확인한다. 각 20Hz physics frame에 actor ID/상태, ego road/lane, 앞 범퍼와 정지선 거리, 실제 속도, BasicAgent 판단 관찰치와 적용된 제어를 시각과 함께 기록한다. 빨간 신호에서 정지선 침범이 없고 정지선 전에 정지했는지 확인한다. 노랑·Off·Unknown 또는 actor 대응 실패는 별도 사전 정의 정책이 없으면 판정 불가로 남긴다. 녹색 통과도 연결 actor가 확인된 경우에만 판정한다. 이 검사는 CARLA 에피소드의 신호 행동 점검이며 실제 도로의 법적 안전 인증은 아니다.
4. **주행 QA:** 선택 구간을 사전 고정하고 우회전 전·중·후의 실제 camera/state를 10Hz/20Hz로 보존한다. 하나의 연속 구간에서 속도 `>0.5m/s`가 30초 이상인지, 선택한 **모든** anchor마다 실제 미래 64점/6.4초가 있는지, 기존 9개 운동 경계가 모두 통과하는지 확인한다. 신호 정차 때문에 끊긴 시간을 합쳐 30초로 계산하지 않는다. 같은 live XODR과 실제 actor bounding box로 차체·sweep의 의도 차로 침범을 검증하고, collision·lane-invasion·목표 `≤2.5m` 및 화면의 차량·궤적 가시성을 기록한다. [기존 export](../../../../../tools/e2e/export_carla_vad_expert.py)는 끝부분의 불완전한 미래 anchor를 버릴 수 있으므로 export 성공만으로 이 조건을 충족했다고 볼 수 없다. [기존 moving QA](../../../../../tools/e2e/audit_carla_moving_capture.py)는 고정된 다른 straight task용이므로 Town12 경로와 신호 조건을 가진 별도 QA가 필요하다.

새 에피소드가 이 절차를 통과하더라도 먼저 `PENDING_QA`로 취급한다. TRAIN 편입은 출처·품질 검증과 split 원장 변경을 거친 별도 결정이며, 현재 학습·Autoware closed-loop 주행·독립 DEV 인정 수는 증가하지 않는다. 현재 정적 XML 분류에는 이미 [고정 바이트 검증 테스트](../../../../../learning/tests/test_classify_town12_junction3742_signals_20260929.py)가 있어 같은 내용을 다시 검사하는 도구는 추가하지 않았다. 남은 핵심 증거는 live 상태와 실제 주행에서만 얻을 수 있다.
