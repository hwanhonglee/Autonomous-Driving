<!-- HH_260810 - Defined the gated Stage 1.5 and Stage 2 operator sequence for the actual four-PC preparation. -->
# Stage 1.5 and Stage 2 no-actuation runbook

<!-- HH_260810 - Prevented this conditional runbook from authorizing an unimplemented gateway. -->
Status: **Stage 1.5 is executable after the listed physical preflight. Stage 2 is a conditional procedure only;
the current PC4 localhost-only adapter and surrogate gateways are not an actual-LAN deployment path.**

<!-- HH_260810 - Fixed the safety invariant for every step in this runbook. -->
차량은 전 구간 `MANUAL/PARK`·정지 상태를 유지한다. PC1 historical `run_bridge`, CAN sender/control
adapter, `/to_can_bus`, engage/autonomous call 및 actuation-capable sink는 실행하지 않는다. 독립 CAN TX
counter observer는 test process보다 먼저 시작하고 마지막에 종료한다.

<!-- HH_260810 - Assigned the stage boundaries without changing existing vehicle authority. -->
## Stage ownership summary

<!-- HH_260810 - Added one exact authority boundary to each host row. -->
| Change comment | Host | Stage 1.5 | Stage 2 |
|---|---|---|---|
| HH_260810 - Kept Planning and command observation sink-free. | PC1 | approved receive-only vehicle status; Planning/Control evidence; no sender | unchanged physical path and read-only response evidence only |
| HH_260810 - Preserved physical canonical perception. | PC2 | PC3 LiDAR-derived physical perception and exactly one canonical predictor | physical canonical unchanged; PC4 raw namespaced recorder only |
| HH_260810 - Preserved real map localization and TF authority. | PC3 | LiDAR, GNSS/INS, odom/accel, NDT/EKF, map→base_link | same authority; exact reverse telemetry sources only |
| HH_260810 - Kept the simulator absent before introducing shadow transport. | PC4 | all source/gateway/owned actors off | candidate private Domain 42 source and one reviewed Domain 42↔10 gateway only after gates pass |

<!-- HH_260810 - Kept Domain 5 exclusive to the same-host Planning surrogate. -->
Domain 5는 same-host Planning Simulator surrogate 전용이며 actual four-PC Stage 1.5, 2A 또는 2B에
participant나 dependency로 포함하지 않는다.

<!-- HH_260810 - Required one immutable evidence context before starting any stack. -->
## Common preflight

<!-- HH_260810 - Required one run identity and non-overwriting evidence roots. -->
1. 네 host가 동일한 `run_id`, UTC/KST window, scenario, stage, map/transform revision을 사용하고 각 host에
   새 owner-only evidence directory를 만든다.
<!-- HH_260810 - Bound every executable and configuration to recorded bytes. -->
2. source commit, dirty state, installed executable, launch, config, DDS XML, map 및 transform의 resolved path와
   SHA-256를 기록한다.
<!-- HH_260810 - Required independent physical safety evidence instead of software inference. -->
3. safety operator가 물리적으로 MANUAL/PARK·정지를 확인하고 sender/adapter process 0, `/to_can_bus`
   publisher와 actuator subscriber 0, engage/autonomous call 0, CAN TX baseline counter를 기록한다.
<!-- HH_260810 - Required exact live network inventory without prescribing an address. -->
4. 각 host의 address, route, link, MTU, multicast, firewall-read status, DDS interface와 sockets를 기록한다.
   approved topology manifest가 없는 NIC/IP/route 변경은 하지 않는다.
<!-- HH_260810 - Kept latency claims blocked until all hosts share an accepted clock. -->
5. four-host Chrony source, reach, stratum, offset, jitter와 step-free window를 확인한다. 실패 시 continuity만
   측정하며 one-way latency와 moving claim을 금지한다.
<!-- HH_260810 - Started independent evidence before any component under test. -->
6. graph/process/network/time/map/CAN recorders를 vehicle stack, PC4 source 또는 gateway보다 먼저 시작한다.

<!-- HH_260810 - Defined the PC4-absent physical baseline. -->
## Stage 1.5 start sequence

<!-- HH_260810 - Proved that PC4 contributes no source or endpoint during baseline. -->
1. PC4 CARLA, Autoware simulator source, adapter, TX logger, gateway와 PC4-owned actors/sensors가 모두 off이며
   Domain 10의 PC4/gateway-owned endpoint가 0인지 확인한다.
<!-- HH_260810 - Started a separately approved receive-only vehicle-status source on PC1. -->
2. PC1은 approved receive-only vehicle-status profile 또는 동등한 non-actuating source만 시작한다.
   physical CAN TX를 포함하는 historical `run_bridge`는 사용하지 않는다.
<!-- HH_260810 - Started PC3 physical sensing and localization authority before downstream consumers. -->
3. PC3 LiDAR/GNSS-INS/map/localization을 정상 operator 순서로 시작하고 odom, accel, NDT/EKF health,
   `map→base_link`, map loader, MRM baseline의 owner/GID/type/QoS/rate를 기록한다.
<!-- HH_260810 - Preserved the current PC3 relay consumed by the hardware-free PC2. -->
4. PC3 `/sensing/lidar/concatenated/pointcloud`→PC3-owned relay→
   `/sensing/lidar/top/pointcloud_before_sync`→Domain 10→PC2 path를 그대로 사용한다. PC2에는 LiDAR hardware,
   driver 또는 relay가 없어야 한다.
<!-- HH_260810 - Started PC2 physical perception without any PC4 consumer. -->
5. PC2 physical tracker, obstacle pointcloud와 기존 map-based predictor를 시작하고 canonical tracked/predicted
   writer가 승인된 하나씩인지 확인한다. PC4 recorder/validator/selector/fuser는 아직 시작하지 않는다.
<!-- HH_260810 - Started PC1 Planning and Control without a physical command sink. -->
6. PC1 Planning/Control을 시작하고 route, stable non-zero trajectory, physical objects/occupancy와 status
   continuity를 확인한다. CAN sender/control adapter는 계속 0이어야 한다.
<!-- HH_260810 - Required one uninterrupted baseline window. -->
7. 최소 10분 동안 physical object/pointcloud, odom/accel/TF, MRM, route/trajectory/control, graph/process,
   Chrony와 CAN counters를 동시 기록한다.

<!-- HH_260810 - Defined an all-or-nothing Stage 1.5 pass gate. -->
Stage 1.5 PASS는 PC4 absence, continuous physical perception/localization, one canonical writer, duplicate critical
writer 0, stable non-zero trajectory, approved common time 및 CAN TX delta 0이 모두 성립할 때만 선언한다.
하나라도 실패하면 PC4를 시작하지 않고 rollback한다.

<!-- HH_260810 - Listed the transport and safety prerequisites for a raw rehearsal. -->
## Stage 2A raw-transport entry gates

<!-- HH_260810 - Required isolated validity of the live source used by this raw transport rehearsal. -->
1. PC4 live source bytes를 manifest에 고정하고 isolated content/lifecycle, fresh pacing/nonfuture stamp,
   actor lineage와 zero-residue를 먼저 통과한다. archived source는 이 live runbook의 대체 입력이 아니다.
<!-- HH_260810 - Required a LAN-capable gateway rather than a localhost surrogate. -->
2. candidate Domain 42의 fleet-wide nonuse approval, actual Domain 42↔10 deny-by-default config/runner,
   per-domain DDS NIC confinement, static lint, runtime endpoint audit와 rollback rehearsal가 있어야 한다.
<!-- HH_260810 - Required exact receiver ownership before any object leaves PC4. -->
3. PC2 raw shadow recorder의 node/GID/type/QoS/evidence path가 승인되어야 하며 canonical subscriber,
   validator, accepted status, selector 또는 fuser가 없어야 한다.
<!-- HH_260810 - Required renewed physical safety evidence immediately before opening a gateway. -->
4. Stage 1.5 PASS가 유지되고 MANUAL/PARK·정지, no sender/adapter, `/to_can_bus` endpoints 0,
   no engage/autonomous call, CAN TX delta 0을 다시 확인한다.
<!-- HH_260810 - Required explicit authorization for a nonacceptance transport rehearsal. -->
5. exact egress-only profile SHA, 시간 창과 `TRANSPORT REHEARSAL / NOT STAGE 2 ACCEPTANCE` label을 지정한
   written authorization이 있어야 한다.

<!-- HH_260810 - Listed the stronger gates required before aligned-shadow promotion. -->
## Stage 2B aligned-shadow promotion gates

<!-- HH_260810 - Required recorded real-state acceptance before live ego alignment. -->
1. immutable real-state `ego_actual` replay를 포함한 isolated Stage 1 acceptance가 PASS여야 한다.
<!-- HH_260810 - Required approved spatial identity before a live ego or object overlay. -->
2. PC3 active Lanelet2/PCD/projector/config manifest, approved numeric simulator-world→map transform,
   3개 이상 non-collinear control points와 residual threshold가 있어야 한다.
<!-- HH_260810 - Required an accepted common clock for aligned cross-host timing. -->
3. four-host Chrony source/offset/jitter/step-free window가 승인되어야 한다.
<!-- HH_260810 - Required an endpoint-zero transition and a separately hashed reverse profile. -->
4. Stage 2A egress gateway를 완전히 닫은 뒤 reverse-only와 aligned combined profile 각각의 exact hash,
   ownership 및 rollback이 승인되어야 한다.

<!-- HH_260810 - Blocked use of the current local profiles for either actual LAN subphase. -->
현재 Stage 2A와 2B gate는 닫히지 않았다. `vehicle_gateway.disabled.yaml`은 vehicle domain `null`인 inert
contract이고 Domain 10→5 및 Domain 42→43 profiles는 localhost surrogate이다. 별도 reviewed actual
profile 없이 아래 절차를 실행하지 않는다.

<!-- HH_260810 - Defined the exact read-only reverse routes used only for aligned-shadow promotion. -->
## Stage 2B reverse-only route set

<!-- HH_260810 - Required every vehicle input to be remapped into a private PC4 namespace. -->
| Change comment | Domain 10 source | PC4 private destination | ROS type |
|---|---|---|---|
| HH_260810 - Mirrored authoritative PC3 odometry read-only. | `/localization/kinematic_state` | `/vils/in/localization/kinematic_state` | `nav_msgs/msg/Odometry` |
| HH_260810 - Mirrored authoritative PC3 acceleration read-only. | `/localization/acceleration` | `/vils/in/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` |
| HH_260810 - Mirrored vehicle velocity for comparison only. | `/vehicle/status/velocity_status` | `/vils/in/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` |
| HH_260810 - Mirrored vehicle steering for comparison only. | `/vehicle/status/steering_status` | `/vils/in/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` |
| HH_260810 - Mirrored vehicle gear for parked-state evidence. | `/vehicle/status/gear_status` | `/vils/in/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` |
| HH_260810 - Mirrored control mode without treating it as physical authority. | `/vehicle/status/control_mode` | `/vils/in/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` |
| HH_260810 - Mirrored final control command without a PC4 vehicle writer. | `/control/command/control_cmd` | `/vils/in/control/command/control_cmd` | `autoware_control_msgs/msg/Control` |
| HH_260810 - Mirrored the vehicle Planning trajectory read-only. | `/planning/scenario_planning/trajectory` | `/vils/in/planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` |
| HH_260810 - Mirrored the mission route read-only. | `/planning/mission_planning/route` | `/vils/in/planning/route` | `autoware_planning_msgs/msg/LaneletRoute` |

<!-- HH_260810 - Kept the reverse-only transition free of PC4 vehicle-facing writers. -->
reverse-only profile에는 위 9개 exact routes만 있어야 한다. PC4 object/diagnostic route와 Domain 10의
PC4-owned writer는 0이어야 하며 services/actions/parameters와 wildcard는 empty여야 한다.

<!-- HH_260810 - Defined the egress route set shared by raw and aligned shadow profiles. -->
## Stage 2A and Stage 2B live egress route set

<!-- HH_260810 - Selected only one namespaced object route and one versioned live diagnostic. -->
| Change comment | Domain 42 source and Domain 10 destination | ROS type | QoS | Consumer |
|---|---|---|---|---|
| HH_260810 - Routed a full-snapshot virtual object stream without canonical ownership. | `/perception/pc4/virtual_obstacles/tracked_objects` | `autoware_perception_msgs/msg/TrackedObjects` | reliable; volatile; keep-last; depth 1 | PC2 raw recorder only |
| HH_260810 - Routed one owner-approved live health revision. | `/diagnostics/pc4/object_adapter` | `diagnostic_msgs/msg/DiagnosticArray` | reliable; volatile; keep-last; depth 1 | PC2 raw recorder only |

<!-- HH_260810 - Prevented the live profile from combining incompatible diagnostic generations. -->
`/diagnostics/pc4/replay_adapter`와 `/diagnostics/pc4/tx_event_logger`는 live egress에 포함하지 않는다.
live diagnostic revision이 owner approval을 받지 못하면 diagnostic을 임의 대체하지 않고 egress profile
activation을 중단한다.

<!-- HH_260810 - Required a separate reviewed contract for any archived cross-PC rehearsal. -->
archived-source transport가 필요하면 replay diagnostic set과 source identity를 고정한 별도 profile/runbook을
review한다. 이 live profile의 object adapter와 replay diagnostics를 혼합하지 않는다.

<!-- HH_260810 - Defined the egress-only Stage 2A transport sequence. -->
## Conditional Stage 2A start sequence

<!-- HH_260810 - Continued all Stage 1.5 evidence during Stage 2. -->
1. Stage 1.5 stacks와 independent recorders를 유지하고 new gateway/source-specific pre-snapshots를 기록한다.
<!-- HH_260810 - Started the live source privately before exposing a transport endpoint. -->
2. Domain 42에서 manifest-selected live source와 adapter를 시작하고 egress를 hard-disable한다.
   actor/sensor inventory, pointcloud→detection→tracking chain, content/lifecycle, one-writer ownership,
   actor lineage와 wall pacing/nonfuture stamp를 확인한다.
<!-- HH_260810 - Opened an egress-only gateway for raw transport. -->
3. reverse route 0과 live egress 2개만 포함한 exact Stage 2A profile로 gateway process 하나를 시작하고
   양 domain node/GID/type/QoS, writer cardinality와 forbidden route 0을 확인한다.
<!-- HH_260810 - Kept every PC4 payload outside validation and canonical publication. -->
4. PC2 raw recorder만 PC4 object와 live diagnostic을 구독한다. PC2 physical canonical tracker/predictor 및
   PC1 Planning input은 Stage 1.5와 동일해야 한다.
<!-- HH_260810 - Audited ownership repeatedly instead of relying on startup state. -->
5. endpoint audit를 start 직후, 주기적, stop 직전에 반복하고 writer/session/source drift 시 즉시 rollback한다.

<!-- HH_260810 - Defined a separate aligned-shadow promotion after Stage 2A transport evidence. -->
## Conditional Stage 2B promotion sequence

<!-- HH_260810 - Required every stronger alignment gate before changing gateway directionality. -->
1. Stage 2A evidence를 봉인하고 recorded-state, map/transform/control-point와 four-host time gates를 확인한다.
<!-- HH_260810 - Required an endpoint-zero transition before reverse telemetry. -->
2. Stage 2A gateway를 종료하고 Domain 42/10 gateway endpoints 0을 확인한다.
<!-- HH_260810 - Opened one reverse-only gateway with zero PC4 egress. -->
3. exact hash의 reverse-only profile로 gateway process를 하나만 시작하고 exact 9 routes,
   node/GID/type/QoS와 forbidden route 0을 확인한다.
<!-- HH_260810 - Validated the live ego before opening aligned egress. -->
4. PC3 odom/accel로 read-only `ego_actual`을 만들고 approved map/transform 위 live ego overlay residual을
   확인한다. 실패하면 gateway를 닫고 다음 단계로 진행하지 않는다.
<!-- HH_260810 - Required another clean endpoint-zero boundary before the aligned profile. -->
5. reverse-only gateway를 종료하고 Domain 42/10 gateway endpoints 0을 다시 확인한다.
<!-- HH_260810 - Opened one aligned profile only after exact route review. -->
6. reverse 9개와 live egress 2개만 포함한 aligned profile hash로 gateway process 하나를 시작하고
   route/GID/type/QoS/writer cardinality와 forbidden endpoint 0을 다시 확인한다.
<!-- HH_260810 - Preserved raw-recorder-only ownership after alignment. -->
7. PC2는 계속 raw recorder만 사용하며 validator, accepted status, selector, fuser 및 canonical subscriber는
   시작하지 않는다.

<!-- HH_260810 - Distinguished the Stage 2 transport role from stronger alignment evidence. -->
## Raw shadow and aligned shadow evidence labels

<!-- HH_260810 - Added an explicit acceptance boundary to every evidence label. -->
| Change comment | Evidence label | 필수 evidence | 해석 |
|---|---|---|---|
| HH_260810 - Limited raw shadow to noncanonical transport observation. | `STAGE2_RAW_TRANSPORT_ONLY` | exact route/profile hash; endpoint/GID/QoS; PC4 TX and PC2 RX count/ordered CDR/gaps; physical canonical invariant | transport observation only; no accepted virtual object or spatial claim |
| HH_260810 - Added spatial evidence without creating a selector. | `STAGE2_ALIGNED_SHADOW` | raw evidence plus PC3 map hashes, numeric transform hash, 3+ control points, live ego overlay residual | aligned raw shadow only; PC2 still does not validate/select/fuse |
| HH_260810 - Required common time before distributed latency evidence. | `STAGE2_TIME_ACCEPTED_SHADOW` | aligned evidence plus four-host Chrony acceptance and two-sided timestamps | cross-PC application latency may be reported with method; packet metrics remain separate |

<!-- HH_260810 - Prevented an incomplete sublabel from becoming a Stage 2 pass. -->
map/transform, live ego 또는 time gate가 빠진 capture는 해당 상위 label을 사용할 수 없다. raw transport
label도 current LAN gateway readiness를 우회하지 않으며, full Stage 2 PASS는 aligned/time/transport evidence와
physical no-change/no-actuation invariant를 모두 요구한다.

<!-- HH_260810 - Defined exact measurement artifacts without conflating ROS messages and packets. -->
## Measurement and evidence

<!-- HH_260810 - Required source-side payload and lifecycle provenance. -->
1. PC4는 CARLA build/world/settings, actor/sensor spawn-destroy inventory, adapter session/sequence/UUID,
   source/output stamp, object count, output CDR SHA, diagnostic and TX event JSONL을 기록한다.
<!-- HH_260810 - Required receiver-side raw evidence without a canonical consumer. -->
2. PC2는 raw object와 live diagnostic bag, receive wall/monotonic time, writer GID, type/QoS, message count,
   ordered CDR hashes와 inter-message gaps를 기록한다.
<!-- HH_260810 - Required two-sided application reconciliation. -->
3. PC4 TX interval과 PC2 RX sequence를 count/order/CDR로 reconcile하여 application MDR, duplicate,
   reorder와 mutation을 계산한다.
<!-- HH_260810 - Kept network packet metrics independent from ROS message delivery. -->
4. approved vehicle NIC 양쪽의 pcap, interface counters와 gateway sockets를 별도 artifact로 저장하며
   application MDR을 packet PDR로 부르지 않는다.
<!-- HH_260810 - Restricted latency analysis to accepted clocks. -->
5. four-host Chrony PASS 후에만 one-way object/diagnostic latency와 jitter를 계산하고 source, receive,
   wall, monotonic timestamp의 정의를 보고서에 고정한다.
<!-- HH_260810 - Required continuous proof that physical outputs were unaffected. -->
6. PC2 canonical objects, PC3 odom/TF, PC1 trajectory/control과 CAN counters를 전 구간 기록하여 PC4
   stop/restart 또는 gateway stop 전후 불변을 확인한다.

<!-- HH_260810 - Defined the exact Stage 2 pass and abort conditions. -->
## Pass and abort gates

<!-- HH_260810 - Required complete transport spatial time and safety evidence for Stage 2 pass. -->
Stage 2 PASS는 PC4 object/live health raw receive, ordered application reconciliation, approved map/transform,
live ego overlay, common time, exact gateway ownership과 physical canonical/Planning/CAN invariance가 모두
성립할 때만 선언한다. PC4 data는 끝까지 noncanonical이다.

<!-- HH_260810 - Closed the gateway on every authority or provenance violation. -->
unexpected route, duplicate gateway/writer, canonical PC4 subscriber, accepted status, wrong frame/map,
stale/future/replay/nonmonotonic stamp, source/session/actor drift, unexplained loss, physical stream change,
CAN TX delta 또는 engage/autonomous call이 하나라도 발생하면 즉시 FAIL 및 rollback한다.

<!-- HH_260810 - Defined rollback for the PC4-absent physical baseline. -->
## Stage 1.5 rollback

<!-- HH_260810 - Kept PC4 absent while the physical baseline is repaired. -->
1. PC4와 모든 gateway를 계속 off 상태로 둔다.
<!-- HH_260810 - Preserved independent evidence while physical owners stop their stacks. -->
2. 각 owner의 승인된 역순으로 PC1–PC3 stack을 종료하되 graph/process/CAN recorders는 post-stop snapshot과
   CAN TX delta 확인이 끝날 때까지 유지한다.
<!-- HH_260810 - Required a fresh baseline after every correction. -->
3. 실패 artifact를 보존하고 수정 후 새 `run_id`로 전체 10분 baseline을 다시 수행한다.

<!-- HH_260810 - Defined transport-first rollback for Stage 2 without selector assumptions. -->
## Stage 2 stop and rollback

<!-- HH_260810 - Closed the gateway immediately because Stage 2 has no selector or accepted cache. -->
1. 차량 정지와 MANUAL/PARK를 확인하고 PC2 selector/cache를 기다리거나 조작하지 않은 채 gateway를
   즉시 종료한다.
<!-- HH_260810 - Proved removal of every PC4 vehicle-domain endpoint. -->
2. Domain 10에서 PC4/gateway-owned object, diagnostic 및 forbidden endpoint가 0인지 확인한다.
<!-- HH_260810 - Reproved the untouched physical baseline before removing the PC4 source. -->
3. PC2 physical canonical writer/content, PC3 localization/TF, PC1 trajectory와 CAN TX delta가 Stage 1.5와
   동일한지 확인한다.
<!-- HH_260810 - Stopped private PC4 components after the boundary was closed. -->
4. PC4 adapter/TX logger를 clean-stop하고 current-session actors/sensors를 destroy한 뒤 CARLA를 마지막에
   종료한다.
<!-- HH_260810 - Required zero residue across processes actors ports and endpoints. -->
5. four-host process/endpoint/lock, PC4 actor/sensor와 ports `2000–2002`, gateway sessions, CAN counters를
   post-stop snapshot으로 기록한다.
<!-- HH_260810 - Stopped independent evidence only after cleanup proof completed. -->
6. checksum과 rollback timeline을 fsync한 뒤 independent recorders를 마지막에 종료한다.
<!-- HH_260810 - Prevented automatic continuation after cleanup. -->
7. reconnect/restart는 gateway를 자동 arm하지 않는다. 다음 실행은 새 `run_id`, Stage 1.5 재확인,
   exact profile review와 manual authorization을 다시 요구한다.

<!-- HH_260810 - Defined the honest report wording when only Stage 1.5 can run. -->
## Current expected report outcome

<!-- HH_260810 - Kept a blocked Stage 2 from being reported as a failed transport experiment. -->
현재 상태에서 정상적인 다음-session 결론은 Stage 1.5 결과와 `PC4_ABSENT` evidence를 먼저 남기고,
actual LAN profile·map/transform·live ego·time·Stage 1 acceptance 중 하나라도 미완료이면
`STAGE_2_NOT_READY_NO_GATEWAY_STARTED`로 종료하는 것이다. 동일-PC Domain 10→5 evidence를 이 결과에
합산하거나 cross-PC 성능으로 재표현하지 않는다.
