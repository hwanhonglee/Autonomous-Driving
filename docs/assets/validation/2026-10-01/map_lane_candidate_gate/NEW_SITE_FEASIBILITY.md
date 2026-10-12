# 설치된 CARLA 지도에서 새 독립 검증 장소를 지정할 수 있는가

<!-- HH_261001 - Screen only OpenDRIVE/catalogue metadata and frozen roles, never protected TEST pixels. -->

**현재 확정 가능한 새 독립 지도 DEV/TEST 경로: 0/2.** 맵이나 route ID가 존재한다는 사실은, 그 장소를 성능을 보며 새 DEV로 바꿀 권한이나 품질 합격을 뜻하지 않는다. 이번 감사는 설치된 OpenDRIVE·경로 카탈로그의 **파일/해시 metadata**와 [기존 통합 역할](../../2026-09-27/map_next_training_design/README.md)·[DEV 적격성](../../2026-09-29/map_new_dev_feasibility/README.md)·[CARLA suite](../../../../../tools/e2e/carla_expert_suite.yaml)만 읽었다. 후속 문서들은 기계 판독형 통합 분할 원장이 아직 필요하다고 명시한다. 여기서는 기존 suite의 오래된 `train` 필드와 역할이 충돌하면 더 엄격한 후속 보호 판정을 따랐다. CARLA/UE4·GPU·새 수집·학습·모델 추론, 보호 TEST의 RGB/semantic/예측 열람은 없었다.

| 설치/카탈로그 구역 | 현재 역할·이력 | 신규 독립 지도 검증 후보인가 |
| --- | --- | --- |
| Town01, Town02_Opt, Town03, Town05_Opt, CTrack | 지도 TRAIN/반복 DEV/외부 진단에 이미 노출. Town03는 과거 지도 C TRAIN·주행 DEV 충돌. CTrack은 지도 EVAL 반복 노출 | **아니오** |
| Town04, Town07, Town15 | 후속 계획의 held-out TEST; Town04 common10 episode도 `split=test` | **아니오 — 보호 TEST** |
| Town06, Town10HD_Opt | 예약 TEST. 예전 expert suite의 `train` 표기가 더 최근의 보호 역할을 덮지 못함 | **아니오 — 예약 TEST** |
| Town12 | source-editor 설치 자산 있음, suite는 `train` 후보. 주행 정적 새 경로 탐색 제안도 TRAIN으로만 제한 | **아니오 — 결과를 보기 전 고정된 TRAIN 역할** |
| Town11 | OpenDRIVE/level은 있지만 장식 없는 POC, suite `excluded`; 시각 학습·검증용 아님 | **아니오 — 품질/역할 부적격** |
| **Town13** | suite `validation` source-editor 지도이며 정적 카탈로그32경로. 그러나 선택 경로 `town13_right_s0001_p00`은 이미 카메라240장 정찰에 쓰였고 전량 skydome 경고. 다른 위치 A→B→A는 엔진 충돌, 현재 [tile/camera 안전 관문](../../2026-09-29/map_town13_tile_residency_gate/README.md)이 **STOP** | **장래 조건부 site 하나뿐, 지금은 0개.** 기존240장이나 route ID만 바꿔 새 독립 검증으로 만들지 않는다. 렌더/소유 프로세스 안전성·새 경로 분리·새 센서 QA가 먼저 필요하다. |
| 표준 Town02/Town05, Town08/09, 표준 Town10HD | Town02/Town05 표준 렌더 실패, Town08/09 배포 자산 없음, 표준 Town10HD는 미검증이고 Opt가 보호 TEST | **아니오** |
| Town01_Opt 등 `*_Opt` 별칭 | source OpenDRIVE `Town01=Town01_Opt` SHA `97a7f6ac...`, `Town02=Town02_Opt` SHA `953c05f1...`, `Town03=Town03_Opt` SHA `abf177bc...`, `Town05=Town05_Opt` SHA `7edc53b0...` | **아니오 — 같은 도로 기하를 새 site로 세지 않음** |
| Woraksan custom | suite `train`, 이미 expert 주행 QA·export 사용; CTrack custom은 위 EVAL 노출 | **아니오 — 기존 TRAIN/노출 역할** |

Packaged [2026-09-01 경로 카탈로그](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps)는 Town01/02_Opt/03/04/05_Opt/06/07/10HD_Opt/CTrack **9개 map family**의 straight/turn route JSON을 가진다. 이 가운데 현재 역할·노출·보호를 모두 통과하는 새 site는 0개다. source-editor Town11/12/13/15와 custom map까지 suite로 대조해도 **당장 사용 가능한 독립 두 site를 구성할 수 없다.** 특히 한 Town의 두 seed·두 connector를 서로 독립한 *site* 둘로 부풀리지 않는다.

### 실제 다음 조건

1. Town13을 후보로 남기려면 [소스 안전 관문](../../2026-09-29/map_town13_tile_residency_gate/README.md)의 read-only tile-state RPC·moved-stream fix·외부 전송 containment·무센서 확인부터 해결하고, 기존 문제 경로가 아닌 새 물리 구간과 렌더/라벨 조건을 **촬영 전** 봉인한다. 기존 warning-marked240장은 DEV가 아니다.
2. 두 번째 독립 site는 현재 설치/분할에서 지정할 수 없다. 새 합법적 자산·별도 합의된 통합 역할이 생기기 전에는 보호 TEST를 DEV로 전용하거나 Town12 TRAIN을 임의 전환하지 않는다.
3. 두 장소와 같은 틱 RGB/semantic/depth/pose QA가 실제로 확보되기 전에는 [차선 가중치 후보](README.md)의 GPU fit/승격을 NO-GO로 유지한다.
