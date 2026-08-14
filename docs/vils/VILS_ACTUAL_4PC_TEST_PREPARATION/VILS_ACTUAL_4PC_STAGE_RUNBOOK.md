<!-- HH_260810 - Defined the ordered and gated runbook for actual four-PC VILS preparation. -->
# VILS actual four-PC stage runbook

<!-- HH_260810 - Prevented this preparation runbook from authorizing physical actuation. -->
Status: **STAGES 1.5–4 ARE PREPARATION PROCEDURES; STAGE 5 REMAINS BLOCKED WITHOUT SEPARATE APPROVAL**

<!-- HH_260810 - Anchored the runbook to the owner-authored validation sequence. -->
이 순서는 [shared Stage 1.5–5 contract](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_SHARED_ARCHITECTURE.md#L615-L666)을
현재 PC4 evidence와 PC별 v2 audit에 맞게 실제 operator 절차로 풀어쓴 것이다.

<!-- HH_260810 - Defined conditions common to every stage before any process starts. -->
## 공통 사전조건

<!-- HH_260810 - Required immutable releases and named integration branches. -->
1. PC1–PC3 v2 commit은 immutable rollback target이며 변경은 reviewed post-v2 branch에서만 한다.
<!-- HH_260810 - Required one shared run identity and evidence clock window. -->
2. 네 PC가 동일한 `run_id`, UTC/KST start/end, scenario, mode, map/transform manifest를 사용한다.
<!-- HH_260810 - Required a stable common clock before distributed latency or moving claims. -->
3. 네 PC의 Chrony source/offset/jitter/step-free window가 승인되지 않으면 continuity만 측정하고
   one-way latency 및 moving test를 금지한다.
<!-- HH_260810 - Required physical no-actuation conditions before any vehicle-domain gateway. -->
4. Stage 1.5–4는 차량 MANUAL/PARK·정지, sender/command adapter 0, `/to_can_bus` endpoint 0,
   engage/autonomous 호출 0, CAN TX counter delta 0을 유지한다.
<!-- HH_260810 - Required recorders to start before the component under test. -->
5. graph/process/map/time/network/CAN baseline recorder를 gateway나 fault injector보다 먼저 시작한다.
<!-- HH_260810 - Required non-overwriting evidence paths and exact source provenance. -->
6. 모든 host는 새 owner-only evidence directory와 source/install/config/binary SHA manifest를 사용한다.

<!-- HH_260810 - Defined a consistent evidence directory layout across all hosts. -->
## Run directory 구조

<!-- HH_260810 - Standardized the artifact layout without requiring identical host paths. -->
```text
<evidence_root>/<UTC>-<run_id>/
  manifest/
    host.json
    git_and_install.json
    hashes.sha256
  graph/
    before.json
    during_<n>.json
    after.json
  time/
  network/
  rosbag/
  events/
  metrics/
  map_transform/
  can_no_actuation/
  process_lifecycle/
  checksums.txt
```

<!-- HH_260810 - Required an isolated PC4 source gate before introducing distributed variables. -->
## Stage 1 — PC4 private-domain prerequisite

<!-- HH_260810 - Kept the source in a private domain and prohibited the vehicle gateway. -->
1. PC4 private domain에서 CARLA와 Autoware source chain을 시작하고 vehicle gateway는 disabled로 둔다.
<!-- HH_260810 - Required live object content rather than graph discovery alone. -->
2. CARLA actor create→nonempty detection→stable tracker UUID→adapter output을 bag/event log로 검증한다.
<!-- HH_260810 - Required deterministic deletion and restart semantics. -->
3. actor delete/out-of-range→fresh snapshot absence, adapter restart→새 session/UUID namespace를 검증한다.
<!-- HH_260810 - Required recorded real-state replay into the isolated digital ego before any distributed stage. -->
4. immutable real localization bag과 contract를 고정하고 Simple Planning Simulator 및 vehicle gateway를
   끈 상태에서 recorded real pose/twist/acceleration을 PC4 `ego_actual`에 replay한다. source stamp와
   wall/monotonic receive time, frame, map manifest, pose/twist/accel digest, start/end count를 기록하고
   `ego_actual`이 CARLA `ego_model`이나 synthetic hero와 분리된 read-only mirror인지 검증한다.
<!-- HH_260810 - Scoped forbidden-interface evidence to gateway crossings while allowing normal private-stack topics. -->
5. gateway가 닫힌 상태에서 Domain 10의 PC4-owned endpoint 0을 확인하고, 향후 gateway route table에
   forbidden route/service/action/parameter가 0인지 static audit한다. full PC4 private graph 내부의
   `/clock`, `/tf*`, localization, planning topics는 source stack에 필요할 수 있으므로 publisher 0을
   요구하지 않는다. stop 후 process/actor/port/lock residue는 0이어야 한다.
<!-- HH_260810 - Kept map-correct status blocked until PC3 evidence is available. -->
6. PC3 map/transform이 아직 없으면 `LIVE_DATA_PATH_PASS_MAP_TRANSFORM_BLOCKED`처럼 data path와
   map acceptance를 분리해 기록한다.

<!-- HH_260810 - Blocked Stage 2 until both live-source and recorded-ego prerequisites were complete. -->
Stage 1 isolated PASS는 live actor lifecycle/object chain과 archived-real-state `ego_actual` replay,
forbidden-crossing/residue proof가 모두 PASS일 때만 선언한다. 현재 `ego_actual` replay가 runtime PASS가
아니므로 Stage 1은 **NOT PASSED**이며 Stage 2를 시작할 수 없다. 실제 Stage 2 진입에는 이 isolated
PASS와 별도로 PC3 active map/static transform acceptance도 필요하다.

<!-- HH_260810 - Defined the mandatory physical baseline without any PC4 process. -->
## Stage 1.5 — PC1–PC3 baseline without PC4

<!-- HH_260810 - Required PC4 and all gateways to remain absent during baseline. -->
1. PC4 CARLA·adapter·gateway가 꺼져 있고 Domain 10에 PC4 endpoint가 0인지 확인한다.
<!-- HH_260810 - Required the safe PC1 receive path and zero transmit capability. -->
2. 승인된 별도 receive-only profile 또는 동등한 non-actuating vehicle-status 공급원을 필수로 실행한다;
   historical `run_bridge`는 금지한다. 해당 공급원 없이 Stage 1.5를 시작하지 않는다.
<!-- HH_260810 - Required core PC3 authority before starting the timed window. -->
3. PC3 odom·accel·`map→base_link`·LiDAR·NDT/EKF health와 MRM owner를 확인한다.
<!-- HH_260810 - Resolved the PC2 subscription to pointcloud data published by the PC3-owned LiDAR. -->
4. PC2에는 LiDAR 하드웨어·driver·relay가 없음을 manifest에 기록한다. 현재 path는 PC3
   `/sensing/lidar/concatenated/pointcloud` → PC3-owned legacy relay →
   `/sensing/lidar/top/pointcloud_before_sync` → Vehicle Domain 10 → PC2 subscriber이다. 이 existing path를
   Stage 1.5 baseline으로 유지하고 source/relay/consumer GID·type·QoS·frame·rate를 증명한다. PC3
   `/sensing/lidar/concatenated/pointcloud`를 PC2가 직접 구독하는 outer-launch remap은 Stage 1.5에 필요하지
   않으며, relay 제거가 승인될 때만 별도 post-v2 migration으로 A/B 검증한다.
<!-- HH_260810 - Required stable PC2 perception after resolving the PC3-to-PC2 subscription seam. -->
5. 선택한 PC3→PC2 pointcloud subscription으로 PC2 physical object/obstacle pointcloud가 fresh하고 canonical
   PredictedObjects publisher가 정확히 1개인지 확인한다.
<!-- HH_260810 - Required a stable route and moving planning baseline. -->
6. route를 설정하고 trajectory가 stable하며 baseline 전체가 zero-velocity stop path가 아님을 확인한다.
<!-- HH_260810 - Required a continuous ten-minute baseline and shared evidence window. -->
7. 최소 10분 동안 모든 critical topics·graph·metrics·Chrony·CAN counters를 동시에 기록한다.

<!-- HH_260810 - Defined the exact Stage 1.5 pass decision. -->
PASS는 continuous physical objects/obstacle pointcloud, continuous PC3 state/TF, approved MRM baseline,
canonical writer 1, duplicate critical writer 0, stable non-zero trajectory, common clock, CAN TX delta 0을
모두 만족할 때만 선언한다. 하나라도 실패하면 PC4를 연결하지 않고 physical baseline부터 수정한다.

<!-- HH_260810 - Defined four-PC shadow while preserving every physical source. -->
## Stage 2 — four-PC shadow and no actuation

<!-- HH_260810 - Required evidence capture before opening the bridge. -->
1. Stage 1.5 recorders와 no-actuation counters를 계속 유지한 상태에서 PC4 private-domain source를 시작한다.
<!-- HH_260810 - Required static map identity and surveyed geometry before opening even a read-only route. -->
2. 실제 PC3 active map hash와 numeric simulator transform, 3+ static control points를 먼저 검증한다.
<!-- HH_260810 - Opened a reverse-only profile so ego alignment could be tested without vehicle-facing objects. -->
3. PC4 object/diagnostic publisher와 PC4→Domain 10 route가 hard-disabled인지 확인한 뒤, 정확히 한 owned
   gateway process의 reverse-only profile을 연다. Domain 10→PC4 read-only allowlist는 다음 9개로
   제한하고 source writer/type/QoS와 private
   remap destination의 one-writer/one-subscriber ownership 및 loopback absence를 검증한다:
   `/localization/kinematic_state`→`/vils/in/localization/kinematic_state`,
   `/localization/acceleration`→`/vils/in/localization/acceleration`,
   `/vehicle/status/velocity_status`→`/vils/in/vehicle/status/velocity_status`,
   `/vehicle/status/steering_status`→`/vils/in/vehicle/status/steering_status`,
   `/vehicle/status/gear_status`→`/vils/in/vehicle/status/gear_status`,
   `/vehicle/status/control_mode`→`/vils/in/vehicle/status/control_mode`,
   `/control/command/control_cmd`→`/vils/in/control/command/control_cmd`,
   `/planning/scenario_planning/trajectory`→`/vils/in/planning/trajectory`,
   `/planning/mission_planning/route`→`/vils/in/planning/route`.
<!-- HH_260810 - Used the reverse-only state to validate the otherwise unavailable live ego overlay. -->
4. reverse-only profile에서 PC3 odom/accel로 `ego_actual`을 만들고 live ego overlay residual을 승인한다.
   실패하면 gateway를 닫고 object egress를 절대 열지 않는다.
<!-- HH_260810 - Required a zero-endpoint transition before enabling the object-egress profile. -->
5. reverse-only gateway를 종료하고 양 domain gateway endpoint가 0인지 확인한다. 승인된 reverse 9개에
   PC4→Domain 10 one `TrackedObjects` route와 owner-approved live diagnostic revision만 더한 새 profile
   hash로 정확히 한 gateway process를 재시작한다. replay diagnostics는 live profile에서 reject한다.
<!-- HH_260810 - Kept PC4 objects out of the canonical pipeline. -->
6. PC2에서는 raw PC4 stream을 read-only recorder로만 기록한다. Stage 2에는 validator/accepted status가
   없으며 physical canonical path와 연결하지 않는다.
<!-- HH_260810 - Required repeated endpoint and forbidden-route audits during the run. -->
7. 양 domain의 node/GID/type/QoS/cardinality와 forbidden route를 시작 직후·주기적·종료 직전에 검사한다.
<!-- HH_260810 - Required shadow failure observations without changing physical output. -->
8. adapter stop/restart와 gateway stop을 각각 수행하여 physical canonical·trajectory가 불변인지 확인한다.
<!-- HH_260810 - Required gateway-first shutdown for a clean boundary. -->
9. gateway를 먼저 종료한 뒤 PC4 adapter/actors/CARLA를 종료하고 네 host residue를 검사한다.

<!-- HH_260810 - Defined the exact Stage 2 pass decision. -->
PASS는 PC4 object/health가 PC2에서 fresh하게 관측되고 map/transform/time/transport evidence가 완전하며,
canonical Planning path·physical perception·CAN counters에 변화가 없고 no-actuation invariant를 전 구간
유지할 때만 선언한다.

<!-- HH_260810 - Defined candidate fusion without changing canonical Planning. -->
## Stage 3 — PC2 candidate validation and fusion

<!-- HH_260810 - Required the reviewed PC2 post-v2 implementation before the stage. -->
1. PC2 integration branch의 validator·TTL·source manager·physical-main fuser·accepted status tests를 먼저 통과한다.
<!-- HH_260810 - Kept the selector in shadow and canonical physical-only at startup. -->
2. PC2 mode를 `shadow`로 시작하고 canonical은 physical-only로 유지한다.
<!-- HH_260810 - Routed fused data only to a debug candidate topic. -->
3. validated PC4와 physical objects를 candidate/debug topic에서 association·dedup하며 PC1은 이를 구독하지 않는다.
<!-- HH_260810 - Required explicit physical priority and full provenance. -->
4. overlap pair는 physical 우선, unmatched accepted virtual은 1회 추가, 모든 결과에 source count/ID를 기록한다.
<!-- HH_260810 - Required deterministic application-level fault injection. -->
5. random/burst omission, delay, jitter, stale, replay, future/non-monotonic stamp, wrong map/frame,
   writer/session change를 seed와 schedule로 주입한다.
<!-- HH_260810 - Required process loss and permitted link loss without risking shared physical traffic. -->
6. 먼저 PC4 kill/restart와 gateway process stop을 수행한다. cable removal은 dedicated PC4-only
   link/NIC와 switch port가 topology manifest로 증명되고 independent observer가 PC1–PC3 physical
   traffic 비영향을 확인한 경우에만 그 exact link에 수행한다; shared vehicle-LAN cable은 제거하지 않는다.
<!-- HH_260810 - Preserved all Stage 2 no-actuation gates. -->
7. Stage 2의 MANUAL/PARK·no engage·no CAN·one canonical writer 조건을 계속 유지한다.

<!-- HH_260810 - Defined the exact Stage 3 pass decision. -->
PASS는 모든 invalid sample이 reject되고 stale virtual이 승인 TTL 내 제거되며, source sequence가
있을 때 missing physical sample 0, inter-message maximum gap이 승인 threshold 이하, duplicate/ghost 0,
candidate provenance 완전, canonical output은 physical baseline과 동일할 때만 선언한다.

<!-- HH_260810 - Defined canonical Planning observation without a physical command sink. -->
## Stage 4 — PC2 canonical fusion and Planning observation

<!-- HH_260810 - Required successful candidate evidence before canonical selection. -->
1. Stage 3의 동일 build/config/hash가 세 번 이상의 독립 run에서 통과한 뒤에만 진행한다.
<!-- HH_260810 - Required stationary manual selection and one explicit mode transition. -->
2. MANUAL/PARK·정지 상태에서 operator가 `hybrid_optional`을 명시적으로 선택한다.
<!-- HH_260810 - Preserved one tracked and one predicted canonical owner. -->
3. PC2 fuser/selector만 canonical tracked를 publish하고 기존 PC2 predictor 하나만 canonical PredictedObjects를 publish한다.
<!-- HH_260810 - Measured PC1 Planning and Control while keeping every physical sink absent. -->
4. PC1 object→trajectory→control response를 기록하되 sender/adapter/CAN sink를 실행하지 않는다.
<!-- HH_260810 - Required optional-mode loss behavior before required-mode evaluation. -->
5. PC4 loss에서 virtual object가 TTL 내 제거되고 physical-only Planning이 지속되는지 먼저 확인한다.
<!-- HH_260810 - Required implementation of the proposed vehicle-side availability chain before testing it. -->
6. `hybrid_required` 전에 proposed PC3 post-v2 availability/MRM component와 PC1 existing gate
   integration의 exact type/topic/node/owner, unit tests와 one-writer proof를 owner review한다.
<!-- HH_260810 - Evaluated the implemented required-mode availability only while parked and sink-free. -->
7. 구현·review된 경우에만 parked/no-sink에서 accepted status→PC3 availability/MRM→PC1 gate를
   검증하며 물리 명령을 보내지 않는다.
<!-- HH_260810 - Prevented automatic mode recovery after any source identity change. -->
8. restart/reconnect/session/GID/map 변경 후 mode가 `shadow`로 복귀하고 수동 re-arm을 요구하는지 검증한다.

<!-- HH_260810 - Defined the exact Stage 4 pass decision. -->
PASS는 canonical publisher 1개, accepted status와 selected snapshot의 정확한 일치, optional fail-open,
required unavailable routing, no clear-scene substitution, PC1 Planning response, 전 구간 CAN TX delta 0을
모두 충족할 때만 선언한다.

<!-- HH_260810 - Kept physical actuation blocked behind separate safety approval. -->
## Stage 5 — closed-course low-speed test

<!-- HH_260810 - Listed the blockers that prevent this document from authorizing execution. -->
현재 Stage 5는 **BLOCKED**이다. PC1 `control_cmd`와 cruise/control-request source의 독립
monotonic-age/validity watchdog·request-bit fail-closed clear·exclusive CAN owner,
실제 IONIQ geometry, `0.833 m/s` enforced planner/controller config, PC2 장기 fusion 안정성,
PC3 post-power localization, live-audited vehicle MRM owner and proposed post-v2 loss-policy implementation,
four-host Chrony, required-source loss chain, safety driver/E-stop/spotter
승인이 모두 끝나지 않았다.

<!-- HH_260810 - Defined the first future condition without giving an executable actuation command. -->
별도 written approval 이후에도 첫 조건은 static virtual obstacle 1개, 3 km/h 이하, one-way progression,
사전 승인 failure만 허용하며 detection→plan, plan→command, command→vehicle response를 각각 측정한다.
이 문서에는 CAN/engage/actuation 실행 명령을 넣지 않는다.

<!-- HH_260810 - Continued the surrogate experiments without conflating them with vehicle acceptance. -->
## Same-PC surrogate 후속 실험

<!-- HH_260810 - Kept every surrogate result supplementary to the real four-PC stages. -->
아래 실험은 PC4+Planning Simulator에서 계속할 수 있지만 `supplementary SIL characterization`이며
Stage 2–5의 vehicle LAN/map/time/PC2/real-dynamics acceptance를 대체하지 않는다.

<!-- HH_260810 - Added one exact interpretation to each surrogate experiment row. -->
| Change comment | 시험 | 설계 | 허용 주장 |
|---|---|---|---|
| HH_260810 - Repeated the live response with a fresh Planning process each time. | fresh-process A/B/A | object present→absent→present; same fixed start/goal; new process per replicate | response-class repeatability and latency |
| HH_260810 - Kept the software vehicle-state loop distinct from CARLA sensor viewpoint. | simulated-state closed loop | Planning→Control→Simple Planning Simulator state; PC4 objects exogenous | vehicle-state closed-loop SIL only |
| HH_260810 - Preserved replay causality limits when commands cannot alter future recorded poses. | real-state replay open loop | recorded state sole publisher; Simple Planning Simulator off | open-loop response and repeatability only |
| HH_260810 - Injected deterministic faults at the application-message boundary. | omission/delay/jitter/stale/replay | exact seed and schedule; conditioner before prediction | controlled message-fault Planning characterization |
| HH_260810 - Measured processing trends without claiming physical network load. | 1x/2x/4x | object count and publish rate varied independently | same-host CPU/GPU/RSS/latency trend |

<!-- HH_260810 - Defined the surrogate-only two-by-two causal matrix without disabling real safety inputs. -->
### 2×2 object-versus-occupancy matrix

<!-- HH_260810 - Added one bounded causal question to each surrogate matrix row. -->
| Change comment | PC4 object | Surrogate occupancy support | 질문 |
|---|---|---|---|
| HH_260810 - Established a no-object baseline with Planning prerequisites present. | absent | present | baseline route/trajectory/control |
| HH_260810 - Isolated the PC4 object effect while holding occupancy support present. | present | present | object-dependent predicted/trajectory/control change |
| HH_260810 - Diagnosed the known Planning dependency without attributing it to PC4. | absent | absent | Planning readiness loss caused by missing occupancy |
| HH_260810 - Detected confounding between object input and missing occupancy support. | present | absent | object cannot be credited if Planning remains occupancy-blocked |

<!-- HH_260810 - Prevented the surrogate matrix from disabling the real physical safety overlay. -->
이 2×2는 동일-PC surrogate diagnosis에만 사용한다. 실제 Stage 2–5에서 PC2 physical obstacle
pointcloud/occupancy를 고의로 제거하는 시험은 별도 AEB/sensor safety 설계 없이는 금지한다.

<!-- HH_260810 - Defined mode-specific expected behavior for the actual PC2 implementation. -->
## Mode and failure acceptance matrix

<!-- HH_260810 - Added one deterministic expected response to each mode and fault row. -->
| Change comment | Mode | Fault | Expected result |
|---|---|---|---|
| HH_260810 - Proved PC4 independence in the rollback mode. | `real_only` | PC4 any state | no canonical effect; physical continuous |
| HH_260810 - Kept validation evidence outside canonical selection. | `shadow` | valid PC4 | accepted/debug only; canonical physical |
| HH_260810 - Disarmed shadow on source identity change. | `shadow` | writer/session/map change | reject; cache clear; remain shadow |
| HH_260810 - Kept optional hybrid physical fail-open. | `hybrid_optional` | stall/kill/cable | virtual removed within TTL; physical continues |
| HH_260810 - Removed duplicates while preserving physical identity. | `hybrid_optional` | overlapping object | one physical-priority object |
| HH_260810 - Prevented required-source loss from becoming a clear scene. | `hybrid_required` | stall/kill/cable | unavailable; inhibit or approved MRM; no clear snapshot |
| HH_260810 - Prevented silent physical fallback in virtual-only required mode. | `virtual_only_required` | PC4 loss | unavailable; no automatic physical selection |
| HH_260810 - Rejected invalid content consistently in every armed mode. | any PC4-using mode | stale/future/replay/wrong map/frame/malformed | reject and record exact reason; no stale canonical object |

<!-- HH_260810 - Required independent repetitions and honest statistics. -->
## 반복과 통계

<!-- HH_260810 - Required fresh processes to avoid hidden planner and tracker state. -->
- 조건별 최소 3회 independent fresh-process run은 exploratory repeatability floor로 사용한다.
<!-- HH_260810 - Required deterministic fault seeds and exact condition manifests. -->
- fault schedule/seed, actor set, publish rate, map/route, source session을 manifest에 고정한다.
<!-- HH_260810 - Limited three-run results to descriptive claims. -->
- independent run `n=3`만 있으면 run-level values와 exploratory mean/range를 보고하며 formal
  confidence interval 또는 run-level p95/p99 claim을 하지 않는다.
<!-- HH_260810 - Required a prespecified adequate analysis plan before inferential claims. -->
- p95/p99 또는 95% confidence interval을 보고하려면 분석 단위(run 또는 message), within-run
  autocorrelation 처리, bootstrap/parametric method, target precision/power와 필요한 independent run 수를
  사전에 고정한다. message-level percentile은 run-level repeatability와 분리한다.
<!-- HH_260810 - Required loss and timing metrics to remain at their measured abstraction layers. -->
- application MDR, DDS receive timing, packet PDR, Chrony offset을 서로 다른 지표로 유지한다.
<!-- HH_260810 - Required failed and partial runs to remain visible. -->
- 실패/중단/cleanup-fail run도 삭제하지 않고 acceptance 집계에서 이유와 함께 분리한다.

<!-- HH_260810 - Defined hard abort triggers shared by every stage. -->
## 즉시 중단 조건

<!-- HH_260810 - Aborted on any actuator-capable path before Stage 5 authorization. -->
- Stage 1.5–4에서 CAN TX 증가, sender/adapter, `/to_can_bus` endpoint, engage/autonomous call 발견.
<!-- HH_260810 - Aborted on duplicate canonical or authority writers. -->
- canonical PredictedObjects 또는 PC3 `map→base_link`의 zero/duplicate/unapproved writer.
<!-- HH_260810 - Aborted on coordinate or time identity drift. -->
- map/transform hash mismatch, unapproved control-point residual, clock source change/step.
<!-- HH_260810 - Aborted on stale or replayed virtual state reaching selection. -->
- validator가 reject해야 할 sample이 candidate/canonical에 등장.
<!-- HH_260810 - Aborted on a physical perception or localization interruption. -->
- PC4 fault 중 physical objects/pointcloud/odom/TF/MRM/trajectory의 unapproved gap.
<!-- HH_260810 - Aborted on ownership or lifecycle residue. -->
- unexpected gateway/node/GID/process/actor/port/lock 또는 cleanup residue.
