<!-- HH_260810 - Defined the PC2 integration implementation and evidence required for actual VILS testing. -->
# PC2 actual-test changes and data

<!-- HH_260810 - Identified PC2 as the mandatory implementation host for candidate and canonical fusion. -->
Status: **NO LIDAR HARDWARE ON PC2; RETAIN AND VERIFY THE PC3 RELAY BASELINE; POST-v2 VALIDATOR/FUSER/SELECTOR REQUIRED**

<!-- HH_260810 - Recorded exact PC2 rollback identities. -->
## 고정 기준

<!-- HH_260810 - Bound PC2 source work to the reviewed Autoware release. -->
- Autoware v2.0.0: `44f79b408ccbffb4cad6f56cbee68de841ac38ac`
<!-- HH_260810 - Preserved the exact PC2 ROS workspace release for rollback. -->
- ros2_ws v2.0.0: `65515a2fc13266c95a812b11c1df7f7b7f5f184c`

<!-- HH_260810 - Recorded the current PC2 pipeline before proposing any integration seam. -->
현재 PC2 physical tracker는 `/perception/object_recognition/tracking/objects`를 publish하고,
기존 map-based predictor가 이를 구독하여 canonical `/perception/object_recognition/objects`를
publish한다. v2에는 PC4 subscriber, writer/session/sequence validator, map gate, TTL cache,
physical-main fuser, source selector, accepted-source status가 없다.

<!-- HH_260810 - Distinguished PC3 LiDAR ownership from the unresolved PC2 pointcloud subscription. -->
PC2에는 LiDAR 하드웨어가 없다. 실제 LiDAR와 sensing authority는 PC3에 있고, PC3가 Domain 10에
`sensor_msgs/msg/PointCloud2`를 publish하면 PC2는 이를 구독하여 perception만 수행한다. PC2 v2 active
`perception.launch.xml`의 default subscribed topic은 `/sensing/lidar/top/pointcloud_before_sync`이며,
PC3의 canonical physical output 후보
`/sensing/lidar/concatenated/pointcloud`와 다르다. shared history는 `before_sync` relay가 temporary였음을
명시한다. 현재 실제시험 baseline은 PC3-owned relay를 유지하고 PC2가 `pointcloud_before_sync`를 구독하는
경로다. 이 경로의 live endpoint/type/QoS/frame/rate를 먼저 검증한다. PC2 outer launch에서 PC3 canonical
`concatenated/pointcloud`를 직접 구독하는 remap은 relay 제거가 필요해질 때 별도 post-v2 migration으로만
검토하며 Stage 1.5 prerequisite가 아니다. LiDAR driver와 relay는 PC2에서 실행하지 않는다.

<!-- HH_260810 - Linked the implementation gap and approved seam to immutable source. -->
근거는 [PC2 work-history implementation gap](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC2_WORK_HISTORY.md#L537-L570),
[tracking output argument](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/object_recognition/tracking/tracking.launch.xml#L19-L44),
[prediction input argument](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/object_recognition/prediction/prediction.launch.xml#L3-L12)이다.

<!-- HH_260810 - Linked the active PC2 pointcloud subscription that must be resolved before Stage 1.5. -->
PC2의 current default는 [perception input lines 48–49](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/perception.launch.xml#L48-L49)이고,
temporary relay limitation은 [shared work-history lines 515–525](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC2_WORK_HISTORY.md#L515-L525)에 남아 있다.
PC3 v2가 canonical concatenated output과 `before_sync` relay를 함께 publish하는 근거는
[PC3 lidar launch lines 43–57](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml#L43-L57)이다.

<!-- HH_260810 - Classified every PC2 stage before describing the new package. -->
## 변경 항목

<!-- HH_260810 - Added one auditable stage decision to each PC2 change row. -->
| Change comment | Stage | 변경 여부 | 결과 path |
|---|---|---|---|
| HH_260810 - Preserved the released PC3-relay-to-PC2 physical pipeline as the actual-test baseline. | 1.5 | PC2 LiDAR hardware 없음; existing relay subscription `RETAIN AND VERIFY` | PC3 LiDAR/driver→PC3 `concatenated/pointcloud`→PC3 relay→`pointcloud_before_sync`→Vehicle Domain 10→PC2 subscriber→PC2 detection/tracking→existing predictor |
| HH_260810 - Kept PC4 data outside canonical perception during raw shadow. | 2 | Core `NO SOURCE CHANGE` | raw PC4 RX/transport recorder만 사용; validator와 accepted status는 아직 없음 |
| HH_260810 - Added validation and candidate fusion without changing canonical output. | 3 | `NEW SOURCE REQUIRED` | physical canonical 유지; validated/fused candidate는 debug topic |
| HH_260810 - Made PC2 the only selected tracked and predicted owner. | 4 | `NEW SOURCE REQUIRED` | fuser/selector→canonical tracking→existing one predictor |
| HH_260810 - Deferred physical actuation until long-duration and loss-routing acceptance. | 5 | 추가 승인 필요 | Stage 4 code 재사용 가능하나 safety gate 별도 |

<!-- HH_260810 - Proposed one isolated package boundary without pretending it already exists. -->
## 제안 package 설계

<!-- HH_260810 - Marked the package name as a reviewable candidate rather than an existing component. -->
제안 package 이름은 `autoware_vils_object_integration`이며 **PROPOSED**이다. 실제 구현은 PC2
v2 commit에서 만든 별도 integration worktree/branch에서만 진행한다.

<!-- HH_260810 - Added one precise responsibility to each proposed PC2 component. -->
| Change comment | Component | 책임 |
|---|---|---|
| HH_260810 - Isolated raw transport validation from fusion policy. | `pc4_input_validator` | exact topic/type/QoS/writer GID/session/seq/frame/stamp/map/finite/shape/count 검사 및 TTL cache |
| HH_260810 - Preserved physical output when PC4 is absent or delayed. | `physical_main_tracked_object_fuser` | physical-triggered fail-open pass-through·association·dedup·physical priority·unmatched virtual addition |
| HH_260810 - Centralized mode transitions and manual re-arm behavior. | `vils_object_source_manager` | `real_only\|shadow\|hybrid_optional\|hybrid_required\|virtual_only_required` state machine |
| HH_260810 - Exposed accepted state without giving PC4 an arming authority. | `accepted_pc4_status_publisher` | accepted/rejected reason·writer/session/seq/map/TTL/object snapshot identity를 typed status로 publish |
| HH_260810 - Preserved source attribution outside canonical message contents. | `vils_object_provenance_logger` | physical/virtual/matched/unmatched/rejected counts와 CDR/session evidence |

<!-- HH_260810 - Proposed a reviewable source layout for implementation and rollback. -->
```text
src/universe/autoware.universe/perception/autoware_vils_object_integration/
  include/autoware_vils_object_integration/
  src/pc4_input_validator.cpp
  src/physical_main_tracked_object_fuser.cpp
  src/vils_object_source_manager.cpp
  config/validator.param.yaml
  config/fuser.param.yaml
  config/source_modes.param.yaml
  launch/vils_object_integration.launch.xml
  test/
```

<!-- HH_260810 - Kept the accepted-status schema explicit but provisional until PC owners review it. -->
accepted status는 `PROPOSED vils_interfaces/msg/AcceptedPc4Status` 같은 typed interface가 필요하다.
최소 필드는 mode, armed 여부, accepted 여부/reason, source writer GID, PC4 session ID,
sequence, source stamp, receive age, TTL deadline, map/transform digest, object count, snapshot digest,
last transition time이다. `DiagnosticArray` raw health만으로 required-mode safety state를 결정하지 않는다.

<!-- HH_260810 - Required a fail-closed join because standard TrackedObjects has no session sequence or map digest fields. -->
표준 `TrackedObjects`에는 session ID, sequence, map/transform digest가 없다. PC2 validator는 object
snapshot을 즉시 통과시키지 않고 bounded pending cache에 보관한 뒤, PC4 diagnostic 또는 별도 typed
envelope의 `output_cdr_sha256`, output stamp, object count, session, sequence, map/transform digest와
정확히 join해야 한다. matching metadata의 missing/timeout/reorder/duplicate/conflict는 reject하고 cache를
clear한다. 장기적으로는 이 결합을 하나의 typed envelope로 바꾸는 안을 owner review한다.

<!-- HH_260810 - Defined the exact outer-launch seams while preserving generic launch files. -->
## Launch 연결

<!-- HH_260810 - Preserved the exact canonical writer and GID boundary during candidate-only Stage 3. -->
1. Stage 3에서는 existing tracker→predictor canonical path를 그대로 유지한다. candidate fuser는
   existing physical tracked topic을 tee하여 구독하고 debug candidate topic만 publish한다.
<!-- HH_260810 - Deferred physical tracker output remapping until canonical fusion is explicitly authorized. -->
2. Stage 4 VILS profile에서만 physical tracker `output/objects`를
   `/perception/object_recognition/tracking/pc2_physical_objects`로 설정한다.
<!-- HH_260810 - Routed the Stage 4 fuser or selector output to the existing canonical tracked boundary. -->
3. Stage 4에서는 VILS integration node만 `/perception/object_recognition/tracking/objects`를 publish한다.
<!-- HH_260810 - Reused the existing predictor input argument and retained one predictor. -->
4. 기존 prediction `input/objects`는 canonical tracked topic을 유지하고 map-based predictor는
   정확히 한 인스턴스만 실행한다.
<!-- HH_260810 - Preserved no-argument behavior and rollback on the released physical pipeline. -->
5. top-level PC2 launch의 no-argument default는 `real_only`와 기존 v2 path로 유지한다.
<!-- HH_260810 - Prevented simultaneous Boolean profiles from creating ambiguous source ownership. -->
6. 독립 Boolean 대신 단일 `vils_object_mode` enum만 사용하고 invalid combination은 launch 전에 거부한다.

<!-- HH_260810 - Defined every source mode and its loss behavior. -->
## Mode 계약

<!-- HH_260810 - Applied the following enum only after the post-v2 integration package exists. -->
아래 enum은 Stage 3부터 post-v2 integration package가 실행될 때 적용한다. Stage 1.5의 immutable
physical baseline과 Stage 2 raw recorder에는 source manager, cache 또는 accepted status가 없다.

<!-- HH_260810 - Added one deterministic source and failure rule to each PC2 mode. -->
| Change comment | Mode | Canonical source | PC4 loss behavior | 허용 stage |
|---|---|---|---|---|
| HH_260810 - Preserved the physical-only rollback state. | `real_only` | physical | 영향 없음; PC4 cache disconnected | baseline/rollback |
| HH_260810 - Validated PC4 without changing canonical perception after the validator exists. | `shadow` | physical | diagnostic·cache clear·disarm | Stage 3 and post-v2 rollback |
| HH_260810 - Kept physical perception fail-open in the optional hybrid. | `hybrid_optional` | physical-main fused | TTL 내 virtual 제거 후 physical 지속 | no-actuation 먼저 |
| HH_260810 - Prevented a missing required source from becoming a clear road. | `hybrid_required` | physical-main fused with required PC4 | unavailable; engagement inhibit 또는 승인된 MRM | Stage 4 parked validation then Stage 5 review |
| HH_260810 - Prevented automatic physical fallback in required virtual-only mode. | `virtual_only_required` | accepted PC4 through selector | unavailable; clear scene/physical fallback 금지 | stationary/no-actuation only until separate AEB approval |

<!-- HH_260810 - Required conservative transitions across writers sessions and maps. -->
mode 변경, writer GID 변경, PC4 session 변경, map digest 변경, reconnect 또는 process restart가
발생하면 accepted snapshot을 즉시 clear하고 `shadow`로 돌아간다. 재-arm은 MANUAL/PARK·정지 상태의
명시적 operator action만 허용한다.

<!-- HH_260810 - Defined validator behavior before accepting any virtual object. -->
## Validator와 TTL 계약

<!-- HH_260810 - Required exact graph ownership before payload acceptance. -->
- input topic/type/QoS와 publisher count `1`, approved node identity/current GID를 검사한다.
<!-- HH_260810 - Bound each object snapshot to its separate metadata before acceptance. -->
- object CDR digest·output stamp·object count와 diagnostic/envelope의 session·sequence·map/transform을
  bounded timeout 내 exact join한다.
<!-- HH_260810 - Required session and sequence continuity independent of DDS delivery. -->
- session ID, monotonically increasing sequence, duplicate/out-of-order/replay를 검사한다.
<!-- HH_260810 - Required synchronized time and bounded age rather than bridge receive stamping. -->
- `frame_id=map`, source stamp의 old/future/non-monotonic 여부, synchronized receive age를 검사한다.
<!-- HH_260810 - Required exact map identity and transform acceptance. -->
- PC3-approved map bundle digest와 PC4 transform digest가 selected manifest와 일치해야 한다.
<!-- HH_260810 - Required bounded and finite object payloads. -->
- pose/twist/covariance/classification/existence/shape/footprint의 finite·range·size·count를 검사한다.
<!-- HH_260810 - Preserved full-snapshot deletion and distinguished missing from empty. -->
- 각 message는 full snapshot이며 fresh empty array와 missing/stale source를 구분한다.
<!-- HH_260810 - Derived the final TTL from measured distributed latency and stopping constraints. -->
- TTL은 hard-coded 추측이 아니라 실제 LAN/processing delay와 승인 speed envelope 측정 후 고정한다.

<!-- HH_260810 - Defined physical-main fusion semantics that generic mergers do not provide. -->
## Association과 fusion

<!-- HH_260810 - Triggered optional hybrid output only from the PC2 stream derived from PC3 pointcloud data. -->
- `hybrid_optional`과 `hybrid_required`의 fuser publish trigger는 PC2가 PC3-published PointCloud2로 생성한
  physical `TrackedObjects` stream이다.
<!-- HH_260810 - Prevented delayed PC4 input from starving physical perception. -->
- PC4가 never-started/delayed/disconnected/restarted여도 physical objects는 항상 통과한다.
<!-- HH_260810 - Preferred physical identity for associated overlap pairs. -->
- overlap association pair는 physical object를 채택하고 PC4 duplicate는 제거한다.
<!-- HH_260810 - Preserved unique non-overlapping virtual actors with provenance. -->
- physical과 겹치지 않는 accepted virtual actor만 추가하고 source provenance count를 남긴다.
<!-- HH_260810 - Prevented stale republishing and transient-local ghost objects. -->
- TTL 만료 즉시 virtual actor를 제거하며 stale sample을 heartbeat 목적으로 재publish하지 않는다.

<!-- HH_260810 - Listed the exact PC2 data needed to validate physical and virtual ownership. -->
## PC2에서 취득할 데이터

<!-- HH_260810 - Added one auditable purpose to each PC2 data stream row. -->
| Change comment | Topic or artifact | 목적 |
|---|---|---|
| HH_260810 - Preserved the unfused physical truth path for fail-open analysis. | Stage 3 existing canonical tracked input; Stage 4 `/perception/object_recognition/tracking/pc2_physical_objects` | physical count·UUID·rate·inter-message gap·CDR·freshness |
| HH_260810 - Captured exactly what arrived from PC4 before validation. | `/perception/pc4/virtual_obstacles/tracked_objects` | Stage 2 raw transport only; Stage 3+ pending snapshot joined by CDR/stamp/count to metadata |
| HH_260810 - Recorded every validation decision and deadline after validator implementation. | Stage 3+ accepted/rejected status and event log | reason·TTL·GID·session·map digest·age·joined snapshot digest |
| HH_260810 - Kept candidate fusion separate from canonical Planning during Stage 3. | proposed `/perception/pc2/vils/candidate_tracked_objects` | association/dedup/physical priority inspection |
| HH_260810 - Verified the single selected tracked-object boundary in Stage 4. | `/perception/object_recognition/tracking/objects` | exact one writer·selected snapshot digest |
| HH_260810 - Verified the sole canonical predictor output consumed by PC1. | `/perception/object_recognition/objects` | exact one predictor writer·PredictedObjects content |
| HH_260810 - Kept physical safety overlay freshness independently observable. | `/perception/obstacle_segmentation/pointcloud` | point count·frame·rate·stamp·publisher continuity |
| HH_260810 - Measured the retained PC3 relay path and kept direct canonical subscription as an optional migration. | Current PC3-published `/sensing/lidar/top/pointcloud_before_sync`; optional post-v2 `/sensing/lidar/concatenated/pointcloud` | PC3 publisher·relay와 PC2 subscriber owner·QoS·frame·rate·age; current relay path only during baseline |
| HH_260810 - Preserved per-message source attribution outside canonical messages. | provenance JSONL/CSV | physical/virtual/matched/unmatched/rejected counts and hashes |

<!-- HH_260810 - Required resource and process evidence around every PC2 run. -->
추가로 graph endpoint node/GID/QoS, process/session tree, CPU/GPU/RSS, DDS queue/drop counters,
network socket/NIC counters, exact source/install/config hashes를 같은 run manifest에 저장한다.

<!-- HH_260810 - Defined the minimum PC2 unit and integration test matrix. -->
## 구현 시험 matrix

<!-- HH_260810 - Added one expected behavior to each PC2 fault and mode row. -->
| Change comment | 조건 | 기대 결과 |
|---|---|---|
| HH_260810 - Proved the rollback path without any PC4 process. | PC4 never starts in `real_only/shadow/hybrid_optional` | physical canonical 연속; virtual count 0 |
| HH_260810 - Distinguished a valid empty virtual scene from source loss. | fresh empty full snapshot | accepted empty only when scenario allows zero actors |
| HH_260810 - Removed stale objects within the measured deadline. | message stall beyond TTL | virtual clear; optional physical continues |
| HH_260810 - Rejected an unapproved source identity. | second writer or GID change | reject·cache clear·shadow/disarm |
| HH_260810 - Rejected cross-session replay and sequence faults. | duplicate/out-of-order/replay/session reuse | reject with exact counter/reason |
| HH_260810 - Rejected coordinate and time contract failures. | wrong frame/map/transform/old/future stamp | reject; canonical physical unchanged |
| HH_260810 - Preserved physical priority for overlapping observations. | physical and virtual overlap | one physical object; no duplicate ghost |
| HH_260810 - Preserved unique virtual actors only when validated. | unmatched accepted virtual | add once with source provenance |
| HH_260810 - Proved optional fail-open behavior during approved process and link loss. | PC4 kill/restart; gateway process stop; dedicated PC4-only link removal only when topology-approved | TTL removal; zero missing physical sequence samples where sequence exists; maximum inter-message gap within approved threshold |
| HH_260810 - Prevented required-source loss from becoming a clear road. | required mode PC4 loss | accepted unavailable; inhibit/MRM path; no clear snapshot |
| HH_260810 - Measured bounded resource scaling. | 1x/2x/4x objects and rates | latency/gap/CPU/RSS trend and no ownership drift |

<!-- HH_260810 - Defined PC2 acceptance before canonical fusion can reach PC1. -->
## PASS와 중단 기준

<!-- HH_260810 - Required sole canonical ownership in every selected mode. -->
- `/perception/object_recognition/objects`의 publisher는 PC2 predictor 정확히 1개이다.
<!-- HH_260810 - Required optional physical continuity under every PC4 failure. -->
- `real_only`, `shadow`, `hybrid_optional`에서 PC4 never-start/stall/kill/restart/disconnect 동안
  source sequence가 있으면 missing physical sample은 0이고, inter-message maximum gap은 승인 한도를
  넘지 않는다. sequence가 없으면 expected count/rate와 gap을 함께 보고한다.
<!-- HH_260810 - Required TTL removal and no ghost reappearance. -->
- stale virtual object가 승인 TTL 내 제거되고 transient/replay로 재등장하지 않는다.
<!-- HH_260810 - Required exact rejection of every malformed or misaligned sample. -->
- stale/future/replayed/map-mismatch/frame-mismatch/malformed sample은 canonical에 들어가지 않는다.
<!-- HH_260810 - Required explicit accepted-source state before any required mode. -->
- required mode는 accepted status와 owner-reviewed proposed PC3 availability/MRM→PC1 gate 구현이
  일치하지 않으면 arm할 수 없다.

<!-- HH_260810 - Defined immediate physical-only PC2 rollback. -->
## PC2 rollback

<!-- HH_260810 - Removed raw Stage 2 transport without waiting for a nonexistent source manager. -->
1. Stage 2에서는 gateway와 raw recorder를 종료하고 physical canonical 경로를 변경하지 않는다.
<!-- HH_260810 - Returned the implemented selector to the immutable physical-only state from Stage 3 onward. -->
2. Stage 3 이상에서는 stationary MANUAL/PARK에서 mode를 `real_only`로 전환한다.
<!-- HH_260810 - Cleared every accepted virtual state and prevented automatic re-arm. -->
3. Stage 3 이상에서만 accepted snapshot·TTL cache·session·armed state를 clear하고 자동 re-arm을 차단한다.
<!-- HH_260810 - Removed the upstream source after the local selector is safe. -->
4. PC4 gateway를 종료한다. Stage 3 이상에서 source de-selection이 bounded timeout 내 실패하면
   기다리지 말고 gateway를 즉시 종료하여 fault를 격리한다.
<!-- HH_260810 - Reproved physical input and canonical output ownership. -->
5. physical tracker→existing predictor 경로와 canonical publisher 1개를 확인한다.
<!-- HH_260810 - Restored the immutable package profile if the integration branch is unhealthy. -->
6. VILS include/profile을 제거하고 PC2 v2 commit의 exact launch/config를 atomic 복원한다.
<!-- HH_260810 - Preserved fault artifacts for root-cause analysis. -->
7. failed run bag/log/hash/reject reason은 삭제하거나 덮어쓰지 않는다.
