<!-- HH_260810 - Defined the PC2 operator plan for tomorrow's physical baseline and raw PC4 shadow recording. -->
# PC2 tomorrow test — Stage 1.5 physical perception and Stage 2A raw transport rehearsal

<!-- HH_260810 - Kept the released PC2 perception pipeline immutable while separating raw shadow evidence. -->
Status: **PC2 v2 IMMUTABLE; PHYSICAL CANONICAL ONLY; RAW PC4 RECORDER ONLY IN STAGE 2**

<!-- HH_260810 - Pinned the exact PC2 source identities that must remain unchanged tomorrow. -->
## Immutable PC2 baseline

<!-- HH_260810 - Bound tomorrow's PC2 Autoware execution to the audited v2 commit. -->
- PC2 Autoware v2.0.0: `44f79b408ccbffb4cad6f56cbee68de841ac38ac`.
<!-- HH_260810 - Bound tomorrow's PC2 auxiliary workspace to the corrected audited v2 commit. -->
- PC2 `ros2_ws` v2.0.0: `65515a2fc13266c95a812b11c1df7f7b7f5f184c`.
<!-- HH_260810 - Required runtime and installation provenance for the exact bytes used in the test. -->
- Record source commit/dirty state, installed prefix, executable/library/config path and SHA-256, launch
  arguments, ROS domain, RMW, DDS configuration, and process tree in the shared manifest.
<!-- HH_260810 - Prevented tomorrow's baseline from silently becoming a new perception deployment. -->
- Do not edit or rebuild v2, change the no-argument operator flow, change the active launch filename, modify
  generic `tracking.launch.xml`, or introduce an outer remap tomorrow.

<!-- HH_260810 - Defined the exact released physical perception path that remains authoritative tomorrow. -->
## PC2 ownership and unchanged physical path

<!-- HH_260810 - Preserved the current PC3 legacy relay because it is the audited PC2 v2 input. -->
```text
PC3 Pandar64 and Nebula
  -> /sensing/lidar/concatenated/pointcloud
  -> PC3-owned legacy relay
  -> /sensing/lidar/top/pointcloud_before_sync
  -> PC2 physical perception
  -> detection
  -> /perception/object_recognition/tracking/objects
  -> one existing PC2 map-based predictor
  -> /perception/object_recognition/objects
  -> PC1 Planning
```

<!-- HH_260810 - Kept all LiDAR acquisition and compatibility relay ownership off PC2. -->
PC2 must not run a Hesai/Pandar/Nebula driver, a pointcloud source, or `pc3_lidar_legacy_relay`. The selected
input tomorrow remains `/sensing/lidar/top/pointcloud_before_sync`. Do not switch to direct
`/sensing/lidar/concatenated/pointcloud`, remove the PC3 relay, or run both inputs.

<!-- HH_260810 - Preserved the exact canonical publisher ownership in Stage 1.5 and Stage 2. -->
The existing PC2 multi-object tracker remains the sole writer of
`/perception/object_recognition/tracking/objects`, and the existing PC2 map-based predictor remains the sole
writer of `/perception/object_recognition/objects`. PC4 and the raw recorder never write either topic.

<!-- HH_260810 - Defined the only additional PC2 role allowed in Stage 2. -->
In Stage 2, PC2 adds only a read-only evidence recorder for
`/perception/pc4/virtual_obstacles/tracked_objects` and the exact live diagnostic
`/diagnostics/pc4/object_adapter`. The recorder has no output into detection, tracking, prediction, Planning,
Control, localization, vehicle, system, TF, clock, service/action/parameter, or CAN interfaces.

<!-- HH_260810 - Prohibited unimplemented post-v2 integration components during tomorrow's raw shadow. -->
## Absolute prohibitions

<!-- HH_260810 - Prohibited a PC2-local physical sensor or compatibility relay duplicate. -->
- Do not start a Hesai/Pandar/Nebula driver, a pointcloud generator, or a PC3 relay process on PC2.
<!-- HH_260810 - Prohibited the unimplemented validation and selection stack in Stage 2. -->
- Do not start a PC4 validator, TTL cache, source manager, fuser, selector, accepted-status publisher, or
  fault-policy consumer.
<!-- HH_260810 - Prohibited canonical remapping and additional prediction ownership. -->
- Do not remap the physical tracker output, publish PC4 data to a canonical topic, or start a second
  map-based predictor.
<!-- HH_260810 - Prohibited unrelated ownership on the perception host. -->
- Do not start Planning, Control, localization authority, CAN, engage, vehicle command, or PC4 simulator
  processes on PC2.
<!-- HH_260810 - Prevented a raw subscriber from being mistaken for source acceptance. -->
- Do not label raw receipt as validated, accepted, fused, selected, canonical, Planning-visible, or Stage 3.

<!-- HH_260810 - Limited all command examples to read-only graph host timing and evidence observation. -->
## Command safety boundary

<!-- HH_260810 - Kept PC2 startup and shutdown under its audited owner procedure. -->
The commands below inspect or record state only. They do not launch Autoware, publish ROS messages, call a
service/action, change a parameter or interface, send CAN, kill a process, or alter a topic route. The PC2
owner starts and stops the unchanged v2 stack through the existing audited no-argument operator flow.

<!-- HH_260810 - Required an uncontaminated vehicle-domain observation shell. -->
Run ROS observations from the already sourced PC2 vehicle-domain environment with `ROS_DOMAIN_ID=10` and
record its RMW/DDS/NIC provenance. Do not source PC3, PC4, or Planning-Simulator build overlays into it.

<!-- HH_260810 - Provided a read-only preflight for forbidden local sensor and relay processes. -->
### Read-only process and host preflight

<!-- HH_260810 - Checked that PC2 does not duplicate the PC3 physical sensor path. -->
```bash
date --iso-8601=ns
pgrep -af '[h]esai|[p]andar|[n]ebula|[p]c3_lidar_legacy_relay|[t]opic_tools.*relay'
pgrep -af '[p]c4_input_validator|[v]ils_object_source_manager|[p]hysical_main_tracked_object_fuser|[a]ccepted_pc4_status'
ps -eo pid,ppid,lstart,etime,args
nvidia-smi
free -h
df -h <PC2_EVIDENCE_FILESYSTEM>
```

<!-- HH_260810 - Defined the required interpretation of process and storage checks. -->
Both `pgrep` results must be empty on PC2. Record the complete process output and verify enough storage for
the pointcloud and object bags before the run; an unreadable or nearly full evidence filesystem is NO-GO.

<!-- HH_260810 - Provided read-only network and time observations for actual cross-PC evidence. -->
```bash
chronyc tracking
chronyc sources -v
chronyc sourcestats -v
ip -br address
ip route show
ip -s link show
ss -lntup
```

<!-- HH_260810 - Kept timing claims at the abstraction layer supported by clock evidence. -->
Without a shared approved four-host Chrony source, bounded offset/jitter, and no clock step, report local
receive order, inter-arrival, gap, and continuity only. Do not report cross-host one-way latency.

<!-- HH_260810 - Listed the exact Stage 1.5 PC2 physical perception topics and message types. -->
## Exact Stage 1.5 collection topics

<!-- HH_260810 - Recorded the current PC2 v2 input provided by the PC3-owned relay. -->
- `/sensing/lidar/top/pointcloud_before_sync` — `sensor_msgs/msg/PointCloud2`; selected PC2 input.
<!-- HH_260810 - Recorded the PC3 canonical pointcloud for relay comparison without selecting it on PC2. -->
- `/sensing/lidar/concatenated/pointcloud` — `sensor_msgs/msg/PointCloud2`; comparison only.
<!-- HH_260810 - Recorded CenterPoint detections at their model-specific boundary. -->
- `/perception/object_recognition/detection/centerpoint/objects` —
  `autoware_perception_msgs/msg/DetectedObjects`.
<!-- HH_260810 - Recorded the selected detection boundary feeding the released tracker. -->
- `/perception/object_recognition/detection/objects` — `autoware_perception_msgs/msg/DetectedObjects`.
<!-- HH_260810 - Recorded the existing physical tracked-object canonical boundary. -->
- `/perception/object_recognition/tracking/objects` — `autoware_perception_msgs/msg/TrackedObjects`.
<!-- HH_260810 - Recorded the sole predicted-object boundary consumed by PC1. -->
- `/perception/object_recognition/objects` — `autoware_perception_msgs/msg/PredictedObjects`.
<!-- HH_260810 - Recorded the physical obstacle pointcloud independently of virtual objects. -->
- `/perception/obstacle_segmentation/pointcloud` — `sensor_msgs/msg/PointCloud2`.
<!-- HH_260810 - Recorded occupancy output when it is present in the resolved v2 profile. -->
- `/perception/occupancy_grid_map/map` — `nav_msgs/msg/OccupancyGrid`; record presence/absence and exact
  runtime owner rather than fabricating a required publisher.
<!-- HH_260810 - Recorded system-wide component health without using diagnostics as source acceptance. -->
- `/diagnostics` — `diagnostic_msgs/msg/DiagnosticArray`.

<!-- HH_260810 - Defined the exact Stage 2A raw PC4 topics without adding canonical consumers. -->
## Exact Stage 2A raw-transport topics

<!-- HH_260810 - Recorded the only virtual-object payload permitted from PC4 to the vehicle domain. -->
- `/perception/pc4/virtual_obstacles/tracked_objects` —
  `autoware_perception_msgs/msg/TrackedObjects`; PC2 raw recorder only.
<!-- HH_260810 - Recorded the single owner-approved live diagnostic contract separately from the payload. -->
- `/diagnostics/pc4/object_adapter` — `diagnostic_msgs/msg/DiagnosticArray`; PC2 raw recorder only.

<!-- HH_260810 - Prevented replay diagnostics from coexisting with the selected live diagnostic revision. -->
The live gateway profile must not also route `/diagnostics/pc4/replay_adapter` or
`/diagnostics/pc4/tx_event_logger`. If the diagnostic revision has not been approved and frozen to the exact
topic/type/QoS, Stage 2A is NO-GO.

<!-- HH_260810 - Required raw receipt metadata that a payload-only bag cannot prove by itself. -->
The PC2 raw observer must preserve received serialized CDR, CDR SHA-256, source stamp, object count, observed
publisher GID, PC2 receive wall time, PC2 receive monotonic time, and the correlated diagnostic
session/sequence/map/transform/output-digest fields. It records all payloads and makes no accept/reject or
source-selection decision.

<!-- HH_260810 - Blocked full Stage 2 acceptance when only a basic bag is available. -->
If only a standard rosbag is available and no approved raw RX event observer can provide monotonic receive
time and CDR/diagnostic correlation, a bag-only communication rehearsal may be retained as incomplete
evidence, but **Stage 2A evidence-complete or full Stage 2 acceptance must not be declared**.

<!-- HH_260810 - Provided read-only graph checks for the unchanged physical pipeline. -->
## Read-only graph and rate observations

<!-- HH_260810 - Captured exact PC3 relay input ownership and the absence of a direct-input migration. -->
```bash
ROS_DOMAIN_ID=10 ros2 topic info --verbose /sensing/lidar/top/pointcloud_before_sync
ROS_DOMAIN_ID=10 ros2 topic info --verbose /sensing/lidar/concatenated/pointcloud
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/detection/centerpoint/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/detection/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/tracking/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/obstacle_segmentation/pointcloud
```

<!-- HH_260810 - Required complete endpoint provenance rather than publisher counts alone. -->
Save node name, namespace, host/process correlation, GID, type, QoS, publisher/subscriber cardinality, and
resolved input arguments. The selected legacy pointcloud has one PC3 relay writer and a PC2 perception
subscriber. The canonical concatenated pointcloud may be visible but must not also feed the PC2 v2 stack.

<!-- HH_260810 - Provided bounded read-only rate observations for each physical pipeline boundary. -->
```bash
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /sensing/lidar/top/pointcloud_before_sync
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /sensing/lidar/concatenated/pointcloud
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /perception/object_recognition/detection/centerpoint/objects
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /perception/object_recognition/tracking/objects
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /perception/object_recognition/objects
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /perception/obstacle_segmentation/pointcloud
```

<!-- HH_260810 - Explained that short rate probes complement but do not replace the complete evidence window. -->
The intentional timeout exit is expected. Compare source and relay counts, rate, maximum inter-message gap,
header stamp, frame, `width`, `height`, `point_step`, `row_step`, and serialized payload hashes offline over
the complete bag; do not infer relay integrity from mean Hz alone.

<!-- HH_260810 - Provided a subscriber-only Stage 1.5 recorder for a unique evidence directory. -->
### Stage 1.5 physical perception recorder

<!-- HH_260810 - Required a non-overwriting owner-prepared path before recording high-volume LiDAR data. -->
Use a pre-created owner-only `<PC2_BASELINE_EVIDENCE_DIR>` with confirmed capacity. Stop with `Ctrl-C` and
freeze bag metadata and checksums; never reuse the directory.

<!-- HH_260810 - Recorded both relay sides and every physical perception boundary without changing publishers. -->
```bash
ROS_DOMAIN_ID=10 ros2 bag record \
  --output <PC2_BASELINE_EVIDENCE_DIR>/rosbag/pc2_physical \
  /sensing/lidar/concatenated/pointcloud \
  /sensing/lidar/top/pointcloud_before_sync \
  /perception/object_recognition/detection/centerpoint/objects \
  /perception/object_recognition/detection/objects \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects \
  /perception/obstacle_segmentation/pointcloud \
  /perception/occupancy_grid_map/map \
  /diagnostics
```

<!-- HH_260810 - Ordered the PC2 portion of the three-PC baseline after PC3 becomes authoritative. -->
## Stage 1.5 — PC2 procedure

<!-- HH_260810 - Required PC3 physical-source readiness before PC2 starts. -->
1. Wait for the PC3 owner to prove one Pandar64/Nebula source, one canonical pointcloud writer, one PC3 legacy
   relay writer, fresh localization/TF, and its approved MRM baseline; confirm PC4 and gateways are absent.
<!-- HH_260810 - Reproved that no sensor or relay source runs locally on PC2. -->
2. Capture the PC2 process preflight and reject any local Hesai/Pandar/Nebula/relay process.
<!-- HH_260810 - Preserved the existing v2 PC2 operator flow and selected input. -->
3. Have the PC2 owner start the unchanged v2 perception stack with its current no-argument operator flow and
   `/sensing/lidar/top/pointcloud_before_sync` input.
<!-- HH_260810 - Required continuous causality through all physical perception boundaries. -->
4. Verify input PointCloud2, CenterPoint detections, selected detections, tracked objects, predicted objects,
   and physical obstacle pointcloud are fresh and continuous.
<!-- HH_260810 - Required one writer at each canonical boundary before PC1 starts. -->
5. Verify exactly one existing PC2 writer for canonical TrackedObjects and exactly one existing PC2
   map-based predictor writer for canonical PredictedObjects; record their GIDs and process owners.
<!-- HH_260810 - Required full-window resource and process evidence rather than one startup snapshot. -->
6. Start the physical bag, endpoint watcher, GPU/CPU/RSS/NIC recorder, and process supervisor before PC1 starts;
   use the shared `BASELINE_3PC_R01` run ID for at least ten continuous minutes.
<!-- HH_260810 - Required a clean final snapshot and immutable failed evidence. -->
7. Capture the same topics, graph, process tree, GPU state, and storage state after the interval and freeze
   checksums even when a component failed.

<!-- HH_260810 - Defined the complete PC2 physical baseline pass gate. -->
### Stage 1.5 PC2 GO

<!-- HH_260810 - Required exactly one selected PC3 relay input without a PC2 source duplicate. -->
- `/sensing/lidar/top/pointcloud_before_sync` has one approved PC3 relay writer, is fresh for the complete
  window, and is the only selected PC2 LiDAR input.
<!-- HH_260810 - Required relay continuity and structural payload evidence. -->
- The PC3 canonical and legacy relay streams have an explained count/rate relationship, preserved stamp/frame
  and PointCloud2 structure, and no unexplained payload mutation or excessive gap.
<!-- HH_260810 - Required continuous physical detection tracking and prediction. -->
- Detection, tracking, prediction, and physical obstacle pointcloud remain fresh with no process dropout,
  unexplained queue stall, GPU failure, or crash.
<!-- HH_260810 - Required exact sole canonical ownership. -->
- `/perception/object_recognition/tracking/objects` has one existing PC2 tracker writer and
  `/perception/object_recognition/objects` has one existing PC2 predictor writer with stable GIDs.
<!-- HH_260810 - Required absence of post-v2 and unrelated host roles. -->
- PC2 has zero LiDAR driver/relay, PC4 validator/fuser/selector/accepted-status, Planning, Control, CAN, engage,
  and localization-authority process.

<!-- HH_260810 - Provided a subscriber-only Stage 2 recorder that leaves canonical perception untouched. -->
## Stage 2 raw recorder

<!-- HH_260810 - Required one unique evidence path and recorder identity per shadow condition. -->
Use one pre-created `<PC2_SHADOW_EVIDENCE_DIR>` for each run ID. The approved raw RX event observer starts
before the gateway and is the only PC2 application consumer of the two PC4 namespaced topics.

<!-- HH_260810 - Recorded raw PC4 transport and physical canonical invariance in one evidence window. -->
```bash
ROS_DOMAIN_ID=10 ros2 bag record \
  --output <PC2_SHADOW_EVIDENCE_DIR>/rosbag/pc2_raw_shadow \
  /perception/pc4/virtual_obstacles/tracked_objects \
  /diagnostics/pc4/object_adapter \
  /sensing/lidar/top/pointcloud_before_sync \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects \
  /perception/obstacle_segmentation/pointcloud \
  /diagnostics
```

<!-- HH_260810 - Clarified that evidence subscription does not implement validation or source selection. -->
The bag recorder and raw event observer may record physical/canonical topics for comparison, but neither may
publish, remap, republish, filter into, validate for, or call the canonical pipeline. Record their exact node,
PID, endpoint GID, command line, start/stop time, and binary/config hashes.

<!-- HH_260810 - Provided read-only graph checks for raw-only Stage 2 ownership. -->
```bash
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/pc4/virtual_obstacles/tracked_objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /diagnostics/pc4/object_adapter
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/tracking/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/pc2/vils/accepted_pc4_status
```

<!-- HH_260810 - Defined the expected Stage 2 endpoint state without confusing a missing topic with failure. -->
The PC4 object and diagnostic each have the one approved vehicle-domain gateway writer and only approved raw
evidence subscribers. Canonical writers remain the existing PC2 tracker and predictor. The proposed accepted
status topic is expected to be absent; any validator, source-manager, accepted-status, candidate, or canonical
PC4 consumer is NO-GO.

<!-- HH_260810 - Ordered the PC2 raw-shadow procedure after the physical baseline is accepted. -->
## Stage 2A — PC2 raw-transport procedure

<!-- HH_260810 - Required the physical baseline and transport-specific gates before a raw rehearsal. -->
1. Enter only after signed Stage 1.5 GO, PC4 isolated live-source content/lifecycle/wall-pacing and
   actor-lineage PASS, an approved private-domain/LAN egress-only gateway profile, an approved PC2 raw
   recorder, renewed no-actuation evidence, written authorization, and the shared four-host run manifest.
   Archived-real-state `ego_actual`, PC3 map/transform/control points, live overlay, and common-time gates
   remain mandatory for aligned Stage 2B and full Stage 2 acceptance, not for this transport-only capture.
<!-- HH_260810 - Required physical and canonical recorders before the virtual transport. -->
2. Keep the unchanged PC2 physical stack running and start the raw RX observer, shadow bag, endpoint watcher,
   resource recorder, and PC1/PC3 independent recorders before the gateway.
<!-- HH_260810 - Required exact raw-only ownership before accepting the first PC4 payload. -->
3. When the gateway owner opens the reviewed allowlist, verify one PC4 object writer, one live diagnostic
   writer, only evidence subscribers, no wildcard/replay diagnostic route, and no canonical connection.
<!-- HH_260810 - Required separate non-overwriting runs for archive live loss and restart behavior. -->
4. Record `TRANSPORT_LIVE_R01`, `TRANSPORT_PC4_STOP_R01`, and `TRANSPORT_PC4_RESTART_R01` as separate
   evidence directories with their exact PC4 session, scenario, source hash, actor inventory, gateway hash,
   and event-marker timeline. An archived-source rehearsal requires a separate reviewed replay profile.
<!-- HH_260810 - Required per-message reconciliation without granting source acceptance. -->
5. Reconcile PC4 TX and PC2 raw RX CDR hashes, session/sequence, object count, missing runs, duplicates,
   reorder, mutation, inter-arrival, and gaps. Treat fresh empty and missing/stale source as different states.
<!-- HH_260810 - Required the physical pipeline to remain invariant under every PC4 event. -->
6. Across PC4 never-start/start/stop/restart and gateway stop, verify physical pointcloud, detection, tracking,
   prediction, obstacle pointcloud, canonical writer GIDs, and PC1 Planning continuity have no event-correlated
   source switch, dropout, reset, or ghost.
<!-- HH_260810 - Required gateway-first shutdown because Stage 2 has no local selector to disarm. -->
7. The gateway owner closes the gateway first. Verify PC4/gateway-owned vehicle-domain writers are zero, then
   stop the PC2 raw observer and bag after their final graph/event snapshot; leave the physical v2 path
   unchanged until the shared after-state is complete.

<!-- HH_260810 - Defined the complete PC2 raw-shadow pass gate. -->
### Stage 2A PC2 evidence complete

<!-- HH_260810 - Required complete raw transport evidence while keeping acceptance semantics absent. -->
- Every observed PC4 payload and diagnostic is preserved with exact source/RX correlation; missing,
  duplicates, reorder, mutations, and gaps are explicitly reported rather than silently dropped.
<!-- HH_260810 - Required the namespaced stream to remain raw and noncanonical. -->
- Only approved raw evidence subscribers consume PC4 topics; validator, accepted status, TTL cache, fuser,
  selector, candidate output, and canonical PC4 connection are absent.
<!-- HH_260810 - Required exact canonical ownership and physical continuity through all PC4 failures. -->
- Existing PC2 tracker and predictor remain the sole canonical writers with stable GIDs, and the physical
  input/detection/tracking/prediction/obstacle streams show no PC4-event-correlated interruption.
<!-- HH_260810 - Required no forbidden PC4 ownership in the vehicle domain. -->
- PC4/gateway has zero writer for `/clock`, `/tf*`, localization, vehicle, Planning, Control, system/MRM,
  canonical perception, CAN, service/action, or parameter interfaces.
<!-- HH_260810 - Kept packet and one-way timing claims behind their independent evidence gates. -->
- Application MDR/continuity is separated from packet PDR; packet PDR requires two-sided packet capture, and
  cross-host one-way latency requires accepted four-host Chrony evidence.

<!-- HH_260810 - Defined conditions that immediately block or abort tomorrow's PC2 work. -->
## Immediate NO-GO and abort conditions

<!-- HH_260810 - Blocked the baseline on duplicate physical acquisition ownership. -->
- Any Hesai/Pandar/Nebula driver or PC3 relay process runs on PC2, or PC2 selects both legacy and canonical
  pointcloud inputs.
<!-- HH_260810 - Blocked the baseline on unresolved physical input or canonical ownership. -->
- Missing/stale PC3 relay input, unresolved frame/QoS, zero/duplicate canonical tracked or predicted writer,
  GID drift, process dropout, GPU failure, or stale physical obstacle pointcloud.
<!-- HH_260810 - Blocked raw shadow on any post-v2 source-selection component. -->
- Validator, accepted status, TTL cache, fuser, selector, candidate publisher, canonical PC4 consumer, or a
  second predictor exists in Stage 2.
<!-- HH_260810 - Aborted on a forbidden PC4 route or ownership leak. -->
- PC4/gateway publishes or routes canonical perception, clock, TF, localization, vehicle, Planning, Control,
  system/MRM, CAN, services/actions, parameters, wildcard topics, or both live and replay diagnostics.
<!-- HH_260810 - Aborted on any PC4-correlated disturbance to the physical stack. -->
- Physical pointcloud, detection, tracking, prediction, obstacle pointcloud, canonical ownership, PC1
  trajectory, localization/TF, or MRM shows an unexplained gap/reset during a PC4 event.
<!-- HH_260810 - Prevented incomplete evidence from becoming a transport acceptance claim. -->
- Missing raw RX CDR/monotonic/session evidence, storage exhaustion, clock step, unknown gateway hash, or
  residue prevents Stage 2A evidence completion and full Stage 2 acceptance even if ROS discovery showed
  messages.

<!-- HH_260810 - Defined evidence-preserving PC2 rollback without invoking nonexistent Stage 3 state. -->
## Rollback

<!-- HH_260810 - Kept Stage 1.5 rollback on the unchanged physical profile with PC4 absent. -->
For a Stage 1.5 failure, keep PC4 and all gateways off, preserve the complete failed physical bag and host
evidence, and have the PC2 owner stop the v2 stack only through the approved operator procedure after the
independent after-state recorder has captured process, endpoint, GPU, network, and storage state.

<!-- HH_260810 - Removed Stage 2 transport immediately without waiting for a validator selector or cache. -->
For a Stage 2 failure, the gateway owner closes the PC4 gateway immediately. Do **not** set `real_only`, wait
for accepted status, or clear a candidate/TTL/session cache: none exists in Stage 2. Confirm namespaced PC4
vehicle-domain writers are zero while the physical tracker and predictor remain the sole canonical writers.

<!-- HH_260810 - Preserved raw evidence until gateway removal and canonical recovery are both observed. -->
Keep the raw observer, physical/canonical bag, and endpoint recorder active through the gateway-zero and
canonical-health snapshots, then stop the raw observer without touching the physical input or canonical
launch. If the PC2 stack itself is unhealthy, use only the owner-approved v2 profile restoration procedure;
no ad-hoc file edit, remap, or Git command is part of tomorrow's rollback.

<!-- HH_260810 - Prevented failed and rollback evidence from being rewritten. -->
Freeze failed and recovered artifacts as separate checksum manifests. A retry starts from the full Stage 1.5
gate with a new run ID, new processes, and a new non-overwriting evidence directory.

<!-- HH_260810 - Separated tomorrow's raw recorder from the proposed post-v2 integration package. -->
## Explicitly deferred Stage 3 and later work

<!-- HH_260810 - Listed the proposed components that remain absent from the released PC2 baseline. -->
Stage 3+ requires a separately reviewed post-v2 `autoware_vils_object_integration` implementation containing
an exact topic/type/QoS/GID/session/sequence/frame/stamp/map/content validator, bounded pending metadata join,
TTL cache, physical-main association/dedup fuser, single mode state machine, typed accepted-PC4 status, and
complete provenance logger. These are proposals, not v2 features.

<!-- HH_260810 - Kept candidate validation outside Planning until Stage 3 acceptance is complete. -->
Stage 3 would tee the existing physical tracked topic read-only and publish a noncanonical candidate/debug
topic only. The current tracker-to-predictor canonical path remains physical-only.

<!-- HH_260810 - Deferred canonical remapping and one-predictor selection until Stage 4 review. -->
Only a later Stage 4 VILS profile may remap the physical tracker to an internal PC2 physical topic and let one
reviewed fuser/selector write canonical TrackedObjects while retaining exactly one existing map-based
predictor. No such remap or mode is part of tomorrow's work.

<!-- HH_260810 - Bounded the claims that PC2 evidence may support after tomorrow's run. -->
## Allowed result statement

<!-- HH_260810 - Limited a successful PC2 result to physical stability and raw namespaced transport. -->
A successful result may state that the immutable PC2 v2 pipeline continuously converted the existing PC3
legacy-relayed Pandar64 input into one physical canonical TrackedObjects and one canonical PredictedObjects
stream, while a separate PC2 evidence recorder observed PC4 namespaced raw objects without altering that
pipeline during source loss and restart cases.

<!-- HH_260810 - Forbade unsupported fusion Planning packet and actuation claims. -->
It must not claim PC4 validation, TTL safety, accepted-source state, physical/virtual fusion, canonical or
Planning effect, required-source loss handling, packet PDR without packet capture, one-way latency without
clock acceptance, CAN/engage/vehicle response, AEB coverage, moving VILS, or Stage 3–5 PASS.
