<!-- HH_260810 - Defined the safe PC4 scope for the next actual four-PC test session. -->
# PC4 tomorrow test decision and checklist

<!-- HH_260810 - Limited the immediately executable scope to a PC4-absent physical baseline. -->
Status: **Stage 1.5 with PC4 fully off is the only currently ready next-session scope. Actual-LAN
Stage 2 is NOT READY and remains conditional on the gates in this document.**

<!-- HH_260810 - Prevented this checklist from authorizing vehicle actuation. -->
이 문서는 PC4 관점의 준비·관측·중단 순서를 정의한다. Stage 1.5–2 전 구간에서 차량은
`MANUAL/PARK`·정지 상태여야 하며 CAN TX, engage/autonomous 호출, actuation, PC4 canonical object
publication은 허용되지 않는다. PC1 historical `run_bridge`도 이 시험에서 사용하지 않는다.

<!-- HH_260810 - Separated measured same-host evidence from unmeasured cross-PC behavior. -->
## 현재 evidence가 말하는 범위

<!-- HH_260810 - Recorded the bounded same-host application-message result without promoting it to a LAN result. -->
- 동일 PC의 localhost-only Domain 10→5 surrogate에서는 live CARLA LiDAR→detection→tracking→adapter→
  Planning 경로와 ordered CDR `203/203` 일치가 관측되었다.
<!-- HH_260810 - Recorded the bounded route and Planning response result without claiming real dynamics. -->
- 한 captured start/goal에 대해 23개 ordered lane segment가 일치했고 fresh Planning process의
  object-present/absent/present 조건에서 stop/move/stop response class가 관측되었다.
<!-- HH_260810 - Preserved the exact limitations of the same-host result. -->
- 위 결과는 loopback application-message continuity와 surrogate Planning characterization이다. 물리 LAN
  packet PDR, cross-PC MDR/latency/jitter, Chrony acceptance, PC3 map authority, 실제 차량 dynamics 또는
  vehicle shadow PASS가 아니다.
<!-- HH_260810 - Kept the archived replay baseline separate from live Stage 1 acceptance. -->
- archived TrackedObjects `2,341/2,341` replay는 isolated software-integrity baseline일 뿐이며 full
  live/map-correct Stage 1이나 recorded-real-state `ego_actual` acceptance를 대체하지 않는다.

<!-- HH_260810 - Defined the domain ownership used by the actual and surrogate topologies. -->
## Domain 판정

<!-- HH_260810 - Added one bounded ownership statement to each domain row. -->
| Change comment | Domain | 용도 | 현재 판정 |
|---|---:|---|---|
| HH_260810 - Kept the actual simulator source off the vehicle domain. | `42` | actual PC4 private source 후보 | 실제 LAN에서 미사용·승인 여부를 다시 확인해야 하는 candidate |
| HH_260810 - Preserved the existing vehicle-domain authority. | `10` | PC1–PC3 vehicle domain | PC4는 approved gateway route 외 authority를 갖지 않음 |
| HH_260810 - Confined the paper-domain value to the software surrogate. | `5` | same-host Planning Simulator surrogate | 실제 4-PC topology에서 사용 금지 |

<!-- HH_260810 - Kept Domain 5 out of every actual four-PC phase. -->
Domain 5는 same-host Planning Simulator surrogate 전용이며 actual Stage 1.5, 2A 또는 2B에 사용하지 않는다.

<!-- HH_260810 - Prevented historical aliases and local gateways from becoming deployment profiles. -->
현재 Domain 10→5 및 Domain 42→43 Planning gateway는 localhost-only surrogate용이다. 현재
`vehicle_gateway.disabled.yaml`도 `enabled:false`, vehicle domain `null`, 실행 runner 없음 상태이며
replay diagnostic set을 담은 정적 계약이다. 어느 것도 실제 LAN용 Domain 42↔10 gateway로 실행하거나
복사해 사용하지 않는다.

<!-- HH_260810 - Declared the missing LAN profile without inventing an address or interface. -->
실제 Stage 2 전에는 별도의 reviewed `vehicle_gateway_42_to_10.disabled.yaml` 계열 profile과 runner,
acceptance artifact, rollback이 필요하다. profile은 PC4-side Domain 42 participant와 vehicle-side Domain 10
participant의 DDS interface confinement를 분리해 증명해야 한다. approved topology manifest가 제공하기
전에는 NIC, IP, route, MTU, multicast, firewall 값을 이 문서에서 정하거나 추정하지 않는다.

<!-- HH_260810 - Defined the only candidate live egress routes for actual Stage 2. -->
## Exact PC4 live egress candidate

<!-- HH_260810 - Distinguished the reviewed live candidate from the older replay diagnostic set. -->
| Change comment | PC4 source topic | ROS type and contract | Domain 10 소비자 | 상태 |
|---|---|---|---|---|
| HH_260810 - Kept virtual objects namespaced and noncanonical. | `/perception/pc4/virtual_obstacles/tracked_objects` | `autoware_perception_msgs/msg/TrackedObjects`; `frame_id=map`; reliable/volatile/keep-last/depth 1; full snapshot | PC2 raw shadow recorder only | candidate; actual gateway not implemented or accepted |
| HH_260810 - Selected one live source-health diagnostic revision. | `/diagnostics/pc4/object_adapter` | `diagnostic_msgs/msg/DiagnosticArray`; reliable/volatile/keep-last/depth 1 | PC2 raw shadow recorder only | proposed revision requiring owner approval |

<!-- HH_260810 - Excluded replay diagnostics from the live vehicle-facing profile. -->
Live profile에서는 `/diagnostics/pc4/replay_adapter`와 `/diagnostics/pc4/tx_event_logger`를 route하지 않는다.
그 둘은 archived replay evidence용 기존 set이다. live source의 independent TX event JSONL/logger는 PC4
private evidence로 남길 수 있지만, 별도 승인이 없는 diagnostic topic을 Domain 10에 만들지 않는다.

<!-- HH_260810 - Kept archived cross-PC transport outside the exact live profile in this runbook. -->
archived-source LAN rehearsal은 이 live profile에 source나 diagnostic만 바꿔 끼워 실행하지 않는다.
필요하면 replay diagnostic set, source identity와 claim scope를 고정한 별도 reviewed profile/runbook을 만든다.

<!-- HH_260810 - Prohibited every authority-bearing or unlisted gateway route. -->
PC4→Domain 10에서 `/clock`, `/tf`, `/tf_static`, canonical objects, `/localization/**`, `/vehicle/**`,
`/planning/**`, `/control/**`, `/system/**`, CAN namespaces, `/to_can_bus`, `/from_can_bus`, services,
actions, parameters, wildcard 및 모든 unlisted route는 금지한다.

<!-- HH_260810 - Separated raw-transport readiness blockers from aligned-shadow acceptance blockers. -->
## Readiness and acceptance blockers by phase

<!-- HH_260810 - Required isolated validity of the live source selected for the raw transport rehearsal. -->
1. 이 runbook의 Stage 2A source는 live mode로 고정하고 isolated content/lifecycle, wall pacing,
   nonfuture stamp와 actor lineage를 먼저 통과해야 한다. 현재 archived replay만 bounded PASS이며 full
   live/map-correct Stage 1은 PASS가 아니다.
<!-- HH_260810 - Kept recorded real-state acceptance mandatory before aligned shadow rather than raw transport. -->
2. immutable real-state `ego_actual` replay가 아직 PASS가 아니므로 Stage 2B aligned shadow와 full
   architecture Stage 2 acceptance는 막혀 있다.
<!-- HH_260810 - Required a PC3-authoritative spatial contract before aligned shadow. -->
3. PC3 active Lanelet2/PCD/projector/config hashes, approved numeric simulator-world→map transform,
   3개 이상 non-collinear control points 및 live ego overlay가 없어 Stage 2B를 시작할 수 없다.
<!-- HH_260810 - Required fresh temporal validity for any live source selected in Stage 2A. -->
4. 이전 live run의 약 `1.824x` simulator pacing과 약 `275.35 s` future stamp failure 뒤 wall-rate,
   nonfuture, age acceptance를 fresh run으로 다시 통과하지 않았다. 이 gap은 live-source Stage 2A도 막는다.
<!-- HH_260810 - Required one common four-host time source before latency or aligned acceptance. -->
5. four-host Chrony source/offset/jitter/step-free acceptance가 없으므로 cross-PC one-way latency와
   Stage 2B time-aligned claim을 계산하거나 주장할 수 없다.
<!-- HH_260810 - Required an approved physical topology and LAN-capable gateway implementation for both subphases. -->
6. vehicle LAN NIC/IP/route/firewall/multicast가 승인되지 않았고 per-domain DDS binding을 갖춘 actual
   gateway config/runner/runtime audit가 없다. 따라서 현재 Stage 2A와 2B 모두 NOT READY이다.
<!-- HH_260810 - Required exact live actor and adapter provenance. -->
7. respawn lineage에서 adapter metadata actor `40`과 current actor `41`이 어긋난 이력이 있으므로 live
   source session은 actor ID/UUID/spawn/destroy lineage를 atomic하게 고정해야 한다.
<!-- HH_260810 - Preserved the independent traffic-light defect as a scope limitation. -->
8. camera 6/7 traffic-light TensorRT containers의 `-11` defect가 남아 있어 해결 전에는 LiDAR-object-only
   scope로만 기록하고 full-stack PASS를 금지한다.
<!-- HH_260810 - Required the vehicle-side no-actuation and raw-recorder prerequisites. -->
9. PC1 approved receive-only vehicle-status path, PC2 raw shadow recorder, all-host no-CAN/no-engage
   evidence 및 phase-specific written authorization이 아직 runtime evidence로 닫히지 않았다.

<!-- HH_260810 - Defined what PC4 must do during the next Stage 1.5 baseline. -->
## Tomorrow Stage 1.5 PC4 duties

<!-- HH_260810 - Kept every PC4 source and gateway absent during the physical baseline. -->
1. CARLA, Autoware simulation source, object adapter, TX logger, domain gateway 및 PC4-owned actors/sensors를
   시작하지 않는다.
<!-- HH_260810 - Proved absence without publishing a probe topic. -->
2. baseline 전·중·후에 PC4-owned process count, CARLA-owned actor count, gateway session count와 Domain 10
   PC4/gateway-owned endpoint count가 각각 0인지 read-only snapshot으로 기록한다.
<!-- HH_260810 - Preserved the host and network state without deploying an unapproved LAN configuration. -->
3. host identity, network inventory, route, link, MTU, firewall-read status, DDS config와 clock sample을
   비변경 evidence로 저장하며 차량용 IP를 임의 설정하지 않는다.
<!-- HH_260810 - Required an explicit stop decision after the baseline. -->
4. PC1–PC3 10분 physical baseline이 PASS하더라도 위 Stage 2 blocker가 하나라도 남으면 PC4는 off 상태를
   유지하고 세션을 종료한다.

<!-- HH_260810 - Distinguished transport handling from spatial and temporal acceptance. -->
## Raw shadow와 aligned shadow의 의미

<!-- HH_260810 - Added one explicit claim boundary to each Stage 2 label. -->
| Change comment | Label | PC2 data path | 허용되는 주장 | 금지되는 주장 |
|---|---|---|---|---|
| HH_260810 - Defined raw shadow as transport-only noncanonical observation. | transport-only raw shadow | namespaced object와 live diagnostic을 raw recorder만 구독; validator/accepted status/canonical consumer 0 | exact route set, endpoint ownership, ordered payload continuity, receiver gaps | object가 공간적으로 맞음, 안전하게 accepted됨, Planning 영향, packet PDR, one-way latency |
| HH_260810 - Defined aligned shadow as stronger evidence without changing canonical ownership. | aligned shadow | raw recorder-only 상태를 유지하면서 approved map/transform와 live ego overlay를 함께 기록 | raw continuity와 approved spatial overlay; common Chrony PASS 시에만 cross-PC application latency | canonical fusion, vehicle response, CAN/actuation, real dynamics |

<!-- HH_260810 - Prevented the raw-shadow label from weakening transport safety prerequisites. -->
`transport-only`는 consumer와 claim의 범위를 뜻하며 actual gateway review, no-actuation, exact allowlist를
면제하지 않는다. 다만 map/time/live-ego gap이 남은 별도 승인 capture는 Stage 2A
`TRANSPORT REHEARSAL / NOT STAGE 2 ACCEPTANCE`로만 보존할 수 있다. 현재 actual LAN profile이 없으므로
그 Stage 2A도 실행 준비가 되지 않았다.

<!-- HH_260810 - Defined the conditional raw-transport start order after transport gates are reviewed. -->
## Conditional Stage 2A raw-transport start order

<!-- HH_260810 - Started independent evidence before the component under test. -->
1. 네 host의 run manifest, graph/process/network/time/CAN counters, PC2 raw recorder를 먼저 시작한다.
<!-- HH_260810 - Started the private source with every vehicle-facing route hard disabled. -->
2. PC4 Domain 42 source를 시작하되 gateway와 PC4→Domain 10 object/diagnostic egress는 hard-disabled로 둔다.
<!-- HH_260810 - Required isolated provenance for the selected live source. -->
3. actor→pointcloud→detection→tracking→adapter, content/lifecycle, session identity, one writer,
   full-snapshot semantics, wall pacing/nonfuture stamp와 actor lineage가 fresh PASS인지 확인한다.
<!-- HH_260810 - Opened an egress-only transport profile without reverse telemetry. -->
4. reverse route 0, exact live object/diagnostic route 2개만 가진 reviewed Stage 2A profile로 gateway
   process 하나를 시작한다.
<!-- HH_260810 - Kept Stage 2A data outside canonical perception. -->
5. PC2는 exact namespaced object와 live diagnostic만 raw 기록한다. validator, accepted status,
   selector, fuser 및 canonical subscriber는 시작하지 않는다.
<!-- HH_260810 - Measured application and packet evidence separately. -->
6. PC4 TX event와 PC2 RX event/bag의 count·ordered CDR hash·gap을 대조하고, 물리 packet evidence는
   양쪽 pcap/NIC counter로 별도 기록한다. Chrony PASS 전에는 one-way latency 열을 비워 둔다.

<!-- HH_260810 - Defined a separate promotion sequence for map and ego aligned shadow. -->
## Conditional Stage 2B aligned-shadow promotion

<!-- HH_260810 - Required all spatial time and recorded-ego gates before promotion. -->
1. recorded-real-state `ego_actual`, PC3 map/transform/control points와 four-host time gates를 먼저 닫는다.
<!-- HH_260810 - Required an endpoint-zero boundary before introducing reverse routes. -->
2. Stage 2A gateway를 종료하고 양 domain gateway endpoint 0을 증명한다.
<!-- HH_260810 - Opened read-only reverse telemetry before the aligned egress profile. -->
3. reviewed reverse-only Domain 10→42 route 9개로 gateway 하나를 시작하고 PC3 odom/accel 기반
   `ego_actual` live overlay residual을 승인한다. PC4 egress는 계속 0이어야 한다.
<!-- HH_260810 - Required another clean transition before the aligned profile. -->
4. reverse-only gateway를 종료하고 endpoint 0을 다시 증명한 뒤 reverse 9개와 egress 2개만 포함한
   aligned-profile hash로 gateway 하나를 시작한다.
<!-- HH_260810 - Kept aligned shadow noncanonical. -->
5. PC2는 계속 raw recorder만 사용하며 validator, accepted status, selector, fuser와 canonical subscriber를
   시작하지 않는다.

<!-- HH_260810 - Defined immediate rollback without waiting for a nonexistent Stage 2 selector. -->
## Stop and rollback

<!-- HH_260810 - Closed transport first on any Stage 2 anomaly. -->
1. unexpected route/writer, map/time drift, stale/future/replay, source loss, actor-lineage mismatch, physical
   baseline 변화 또는 no-actuation invariant 위반 시 selector/cache를 기다리지 않고 gateway를 즉시 종료한다.
<!-- HH_260810 - Proved vehicle-domain removal before stopping the source. -->
2. Domain 10의 PC4/gateway-owned object, diagnostic 및 forbidden endpoint가 0인지 확인한다.
<!-- HH_260810 - Preserved the physical baseline while virtual transport is removed. -->
3. PC2 canonical physical objects, PC3 localization/TF, PC1 trajectory와 CAN TX counter가 Stage 1.5 baseline을
   유지하는지 확인한다.
<!-- HH_260810 - Stopped only owned PC4 components in dependency order. -->
4. adapter/TX evidence logger를 clean-stop한 뒤 current-session actors/sensors를 destroy하고 CARLA를 마지막에
   종료한다.
<!-- HH_260810 - Required zero residue and immutable failed evidence. -->
5. PC4 process/session/ports `2000–2002`/locks/actors/ROS endpoints residue 0을 기록하고 실패 artifact를
   덮어쓰지 않은 채 별도 checksum manifest로 봉인한다.
<!-- HH_260810 - Prevented restart from silently rearming the gateway. -->
6. cleanup PASS 후에도 자동 reconnect/re-arm하지 않는다. 다음 시도는 새 `run_id`, fresh process,
   Stage 1.5 재확인과 manual authorization으로 시작한다.

<!-- HH_260810 - Defined the only honest result expected from the next session at the current readiness level. -->
## Tomorrow decision output

<!-- HH_260810 - Required a bounded conclusion rather than a forced Stage 2 attempt. -->
현재 blocker가 유지되면 내일 결과는 `STAGE_1_5_PHYSICAL_BASELINE_<PASS|FAIL>; PC4_ABSENT_PASS;
STAGE_2_NOT_READY`로 기록한다. actual LAN gateway를 열지 않은 것은 실패가 아니라 현재 safety contract를
지킨 결과다.
