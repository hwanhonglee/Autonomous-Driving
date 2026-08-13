<!-- HH_260810 - Defined the PC1 operator plan for tomorrow's non-actuating baseline and raw-shadow tests. -->
# PC1 tomorrow test — Stage 1.5 baseline and Stage 2A raw transport rehearsal

<!-- HH_260810 - Kept tomorrow's PC1 scope on evidence collection without modifying the released vehicle stack. -->
Status: **PC1 v2 IMMUTABLE; STAGE 1.5 AND CONDITIONAL STAGE 2A ONLY; NO ACTUATION AUTHORIZED**

<!-- HH_260810 - Pinned the exact PC1 source identities that must remain unchanged tomorrow. -->
## Immutable PC1 baseline

<!-- HH_260810 - Bound tomorrow's PC1 Autoware execution to the audited v2 commit. -->
- PC1 Autoware v2.0.0: `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8`.
<!-- HH_260810 - Bound tomorrow's PC1 vehicle workspace to the audited v2 commit. -->
- PC1 `ros2_ws` v2.0.0: `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34`.
<!-- HH_260810 - Required runtime provenance instead of assuming that a source checkout is the active installation. -->
- Before the timed window, record the resolved source commit, dirty state, installed package prefixes,
  executable/config paths, and SHA-256 values in the shared run manifest.
<!-- HH_260810 - Prevented tomorrow's test from becoming an unreviewed source or launch migration. -->
- Do not edit, rebuild, cherry-pick, remap the canonical object input, or change the existing no-argument
  operator command or active launch filename.

<!-- HH_260810 - Defined the exact PC1 ownership boundary for both approved stages. -->
## PC1 ownership tomorrow

<!-- HH_260810 - Preserved the only object boundary that PC1 Planning is allowed to consume. -->
PC1 consumes only PC2's canonical `/perception/object_recognition/objects`
(`autoware_perception_msgs/msg/PredictedObjects`) and continues to own Planning, Control computation,
the vehicle-command boundary, and CAN policy.

<!-- HH_260810 - Corrected the earlier draft by keeping the raw PC4 shadow subscriber off PC1. -->
The Stage 2 PC4 stream is recorded by the **PC2 raw shadow recorder**, not by PC1. PC1 must not add an
application subscriber to `/perception/pc4/virtual_obstacles/tracked_objects`, must not subscribe directly
to a PC4 candidate for Planning, and must never publish either canonical object topic.

<!-- HH_260810 - Illustrated the unchanged PC1 data path and the isolated Stage 2 observer. -->
```text
PC2 sole canonical PredictedObjects writer
  -> /perception/object_recognition/objects
  -> PC1 Planning
  -> trajectory
  -> Control computation
  -> final control_cmd evidence only
  X  no command adapter, no SocketCAN sender, no physical CAN sink

PC4 namespaced TrackedObjects
  -> vehicle Domain 10
  -> PC2 raw recorder only
  X  no PC1 subscriber and no canonical connection
```

<!-- HH_260810 - Forbade every actuation-capable PC1 path during tomorrow's work. -->
## Absolute prohibitions

<!-- HH_260810 - Prohibited the historical bridge because it launches receive and transmit components together. -->
- Never run `run_bridge`; it is not a DDS-only bridge or a receive-only logger.
<!-- HH_260810 - Prohibited the proven CAN sender and control-to-CAN adapter processes. -->
- `socket_can_sender_node_exe`, any equivalent SocketCAN sender, and
  `twistController2VCU2EPS2ACC_node` must remain absent.
<!-- HH_260810 - Prohibited ROS and native CAN transmit tools in the no-actuation stages. -->
- Do not run `cansend`, `cangen`, `canplayer`, a CAN replay tool, or any process capable of writing a
  vehicle CAN frame.
<!-- HH_260810 - Prohibited commands that could change vehicle operation state. -->
- Do not publish a control command, call engage or operation-mode services/actions, change parameters,
  or invoke `/vehicle/engage`, `/autoware/engage`, or any autonomous-mode transition.
<!-- HH_260810 - Required the physical vehicle to remain operator controlled and stationary. -->
- The vehicle remains independently verified **MANUAL, PARK, stationary, no engage, no CAN TX** for the
  complete Stage 1.5 and Stage 2 windows.
<!-- HH_260810 - Prohibited direct virtual-object integration before the PC2 Stage 3 implementation exists. -->
- Do not remap PC4 objects to `/perception/object_recognition/objects` or
  `/perception/object_recognition/tracking/objects` and do not modify PC1 Planning to consume them.

<!-- HH_260810 - Made non-actuating vehicle-status acquisition a hard prerequisite rather than an optional convenience. -->
## Required receive-only status source

<!-- HH_260810 - Required one reviewed status source before the physical baseline begins. -->
Before Stage 1.5, the PC1 vehicle owner must provide either a separately named and reviewed receive-only
profile such as `run_vehicle_status_rx_only`, or a documented equivalent non-actuating status source.
It must contain only the CAN receiver/decoder and read-only evidence logger.

<!-- HH_260810 - Rejected any receive profile that can instantiate the historical transmit path. -->
The profile is rejected if its resolved launch tree can start `socket_can_sender_node_exe`,
`twistController2VCU2EPS2ACC_node`, `/to_can_bus`, a command adapter, or any native CAN writer.

<!-- HH_260810 - Required graph process and kernel evidence around the receive-only profile. -->
Acceptance requires, before/during/after the run, sender and adapter process count zero, `/to_can_bus`
publisher and actuator-subscriber count zero, and PC1 `can0` TX packet/byte delta zero. If this approved
source is unavailable, **Stage 1.5 is NO-GO and Stage 2 must not start**.

<!-- HH_260810 - Prevented the local control-mode toggle from being treated as physical vehicle authority. -->
`/vehicle/status/control_mode` is an unvalidated local-toggle diagnostic in the audited PC1 receiver.
It may be recorded for comparison, but MANUAL/PARK must be proven independently using the approved
vehicle/ECU indication and timestamped safety-operator record.

<!-- HH_260810 - Limited all command examples to read-only graph host timing and evidence observation. -->
## Command safety boundary

<!-- HH_260810 - Kept stack startup and shutdown under the existing owner-approved operator procedure. -->
The commands below only inspect or record state. They do not launch Autoware, publish ROS messages, call
services/actions, change parameters, configure an interface, send CAN, engage, or stop a process. Start and
stop PC1 only with the existing audited v2 operator procedure; that command is intentionally not reproduced
here.

<!-- HH_260810 - Required the observer to use the already approved vehicle-domain environment. -->
Run ROS observations from the already sourced PC1 vehicle-domain shell with `ROS_DOMAIN_ID=10`. Record the
resolved `RMW_IMPLEMENTATION`, DDS configuration, NIC, and environment provenance before accepting output.
Do not source a PC4 or Planning-Simulator overlay into this shell.

<!-- HH_260810 - Defined safe preflight process and CAN observations. -->
### Read-only process and CAN preflight

<!-- HH_260810 - Provided a read-only process inventory that avoids matching the observation command itself. -->
```bash
date --iso-8601=ns
pgrep -af '[r]un_bridge|[s]ocket_can_sender|[t]wistController2VCU2EPS2ACC|[c]ansend|[c]angen|[c]anplayer'
ps -eo pid,ppid,lstart,etime,args
ip -s -details link show dev can0
```

<!-- HH_260810 - Defined the required interpretation of an empty process query and a stable CAN counter. -->
The `pgrep` result must be empty. Preserve the complete `ip -s` snapshot with a timestamp; compare TX
packets, bytes, dropped packets, and errors with the during-run and after-run snapshots. Interface absence,
permission failure, or an unreadable counter is **indeterminate**, not zero and not PASS.

<!-- HH_260810 - Provided read-only checks for the forbidden ROS CAN boundary and critical PC1 ownership. -->
```bash
ROS_DOMAIN_ID=10 ros2 topic list -t
ROS_DOMAIN_ID=10 ros2 topic info --verbose /to_can_bus
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /planning/scenario_planning/trajectory
ROS_DOMAIN_ID=10 ros2 topic info --verbose /control/command/control_cmd
ROS_DOMAIN_ID=10 ros2 topic info --verbose /system/fail_safe/mrm_state
```

<!-- HH_260810 - Defined how to interpret a missing forbidden topic without hiding unexpected endpoints. -->
`/to_can_bus` being absent is acceptable. If it exists, both publisher count and actuator-capable subscriber
count must be zero. Preserve verbose node name, namespace, GID, type, and QoS output for every critical topic.

<!-- HH_260810 - Provided read-only clock network and host observations for distributed evidence. -->
```bash
chronyc tracking
chronyc sources -v
chronyc sourcestats -v
ip -br address
ip route show
ip -s link show
ss -lntup
free -h
```

<!-- HH_260810 - Prevented unsynchronized hosts from producing a false one-way-latency claim. -->
If all four hosts do not show the same approved time source, bounded offset/jitter, and a step-free test
window, record continuity and inter-arrival only. Do not calculate or report cross-host one-way latency.

<!-- HH_260810 - Listed the exact PC1 ROS evidence contract for both stages. -->
## Exact PC1 collection topics

<!-- HH_260810 - Defined the canonical object input and its exact released type. -->
- `/perception/object_recognition/objects` — `autoware_perception_msgs/msg/PredictedObjects`.
<!-- HH_260810 - Defined route evidence needed to interpret every trajectory. -->
- `/planning/mission_planning/route` — `autoware_planning_msgs/msg/LaneletRoute`.
<!-- HH_260810 - Defined the final Planning trajectory and its exact released type. -->
- `/planning/scenario_planning/trajectory` — `autoware_planning_msgs/msg/Trajectory`.
<!-- HH_260810 - Defined controller-output evidence before the final vehicle command gate. -->
- `/control/trajectory_follower/control_cmd` — `autoware_control_msgs/msg/Control`.
<!-- HH_260810 - Defined final-command evidence while keeping every physical sink absent. -->
- `/control/command/control_cmd` — `autoware_control_msgs/msg/Control`.
<!-- HH_260810 - Defined the decoded longitudinal comparison report from the approved RX-only source. -->
- `/vehicle/status/velocity_status` — `autoware_vehicle_msgs/msg/VelocityReport`.
<!-- HH_260810 - Defined the decoded steering comparison report from the approved RX-only source. -->
- `/vehicle/status/steering_status` — `autoware_vehicle_msgs/msg/SteeringReport`.
<!-- HH_260810 - Defined the decoded gear comparison report while requiring independent PARK evidence. -->
- `/vehicle/status/gear_status` — `autoware_vehicle_msgs/msg/GearReport`.
<!-- HH_260810 - Defined the local-toggle diagnostic without granting it physical mode authority. -->
- `/vehicle/status/control_mode` — `autoware_vehicle_msgs/msg/ControlModeReport`.
<!-- HH_260810 - Defined the live-audited vehicle safety state and its exact released type. -->
- `/system/fail_safe/mrm_state` — `autoware_adapi_v1_msgs/msg/MrmState`.
<!-- HH_260810 - Required raw receive evidence when the approved RX-only source exposes it. -->
- `/from_can_bus` — `can_msgs/msg/Frame`, only when produced by the approved receive-only path; otherwise
  use an owner-approved timestamped read-only SocketCAN capture.

<!-- HH_260810 - Explicitly excluded the raw PC4 payload from PC1 collection. -->
Do **not** add `/perception/pc4/virtual_obstacles/tracked_objects` or
`/diagnostics/pc4/object_adapter` to the PC1 bag. PC2 owns that raw Stage 2 evidence. PC1 only performs a
graph audit to prove that no PC1 application node subscribes to the PC4 namespaced stream.

<!-- HH_260810 - Provided a subscriber-only bag recording command for a pre-created owner-approved evidence directory. -->
### Subscriber-only PC1 bag recorder

<!-- HH_260810 - Required a unique non-overwriting output path prepared by the evidence owner. -->
Use a pre-created owner-only `<PC1_EVIDENCE_DIR>` for exactly one run ID. Stop the recorder with `Ctrl-C`;
never reuse or overwrite the output directory.

<!-- HH_260810 - Recorded Planning Control safety and decoded status without creating a command publisher. -->
```bash
ROS_DOMAIN_ID=10 ros2 bag record \
  --output <PC1_EVIDENCE_DIR>/rosbag/pc1_core \
  /perception/object_recognition/objects \
  /planning/mission_planning/route \
  /planning/scenario_planning/trajectory \
  /control/trajectory_follower/control_cmd \
  /control/command/control_cmd \
  /vehicle/status/velocity_status \
  /vehicle/status/steering_status \
  /vehicle/status/gear_status \
  /vehicle/status/control_mode \
  /system/fail_safe/mrm_state \
  /from_can_bus
```

<!-- HH_260810 - Limited the recorder role to evidence acquisition. -->
The recorder must appear only as a subscriber to these topics. Record its PID, command line, start/stop
timestamps, storage plugin, bag metadata, and final checksums. A recorder failure or storage exhaustion makes
the run incomplete; it does not justify repeating commands into the same directory.

<!-- HH_260810 - Provided bounded read-only rate observations that do not alter the ROS graph after completion. -->
```bash
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /perception/object_recognition/objects
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /planning/scenario_planning/trajectory
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /control/command/control_cmd
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /vehicle/status/velocity_status
timeout 30s env ROS_DOMAIN_ID=10 ros2 topic hz /system/fail_safe/mrm_state
```

<!-- HH_260810 - Explained the expected timeout exit without treating it as a component failure. -->
An exit caused only by the intentional 30-second `timeout` is expected. Preserve the complete rate and gap
summary; do not use a short `hz` sample as a substitute for the full bag window.

<!-- HH_260810 - Ordered the PC1 portion of the three-PC physical baseline. -->
## Stage 1.5 — PC1 procedure

<!-- HH_260810 - Required PC4 absence before PC1 joins the physical baseline. -->
1. Confirm with the shared manifest that PC4 CARLA, adapter, gateway, and vehicle-domain PC4-owned endpoints
   are absent and that PC3 then PC2 have passed their startup gates.
<!-- HH_260810 - Required the non-actuating status path before starting the timed baseline. -->
2. Start only the separately approved receive-only PC1 status source through the vehicle owner's procedure;
   repeat the sender/adapter, `/to_can_bus`, and CAN TX preflight.
<!-- HH_260810 - Preserved the existing PC1 v2 launch and operator interface. -->
3. Have the PC1 owner start the unchanged v2 stack using the audited no-argument operator flow.
<!-- HH_260810 - Required a route without authorizing engagement or vehicle motion. -->
4. Set the approved fixed route/start/goal using the existing operator UI while remaining MANUAL/PARK and
   do not engage. Save the exact route UUID and ordered Lanelet segments.
<!-- HH_260810 - Blocked causal testing when Planning is already stopped for an unrelated reason. -->
5. Confirm a stable, non-empty canonical object input and a stable trajectory that is not an all-zero-velocity
   path caused by start-planner, missing localization, MRM, or another baseline fault.
<!-- HH_260810 - Required recorders to cover the complete acceptance interval. -->
6. Start the PC1 bag and independent graph/process/CAN/time recorders before the timed interval and collect at
   least ten continuous minutes with the common `BASELINE_3PC_R01` run ID.
<!-- HH_260810 - Required post-window ownership and zero-actuation evidence before declaring success. -->
7. Capture the same verbose endpoints, process inventory, CAN counters, Chrony, and host state immediately
   after the interval; freeze checksums without editing failed evidence.

<!-- HH_260810 - Defined the complete PC1 baseline pass gate. -->
### Stage 1.5 PC1 GO

<!-- HH_260810 - Required one approved upstream canonical source and stable Planning output. -->
- `/perception/object_recognition/objects` has exactly one approved PC2 predictor writer, is fresh, and has a
  stable GID for the complete interval.
<!-- HH_260810 - Required an interpretable route and trajectory baseline. -->
- Route is set, trajectory has exactly one approved PC1 Planning writer, is fresh, and is not an unrelated
  all-zero stop path.
<!-- HH_260810 - Required final command evidence while maintaining sink absence. -->
- Controller and final `control_cmd` each have one approved PC1 writer and fresh output, but no command
  adapter, CAN sender, or actuator-capable subscriber exists.
<!-- HH_260810 - Required an independently proven stationary physical state. -->
- The approved RX-only reports are fresh and their decoder provenance is recorded; MANUAL and PARK are also
  independently proven and are not inferred from the local-toggle control-mode topic alone.
<!-- HH_260810 - Required one fresh audited MRM owner before introducing PC4. -->
- `/system/fail_safe/mrm_state` has one live-audited owner and remains in the owner-approved NORMAL/NONE
  baseline without heartbeat loss.
<!-- HH_260810 - Required electrical and API silence over the full window. -->
- `run_bridge`, sender, adapter, `/to_can_bus`, engage/autonomous calls, and CAN TX delta all remain zero.

<!-- HH_260810 - Defined the PC1 role during raw four-PC shadow without changing Planning. -->
## Stage 2A — PC1 raw-transport procedure

<!-- HH_260810 - Required the physical baseline and transport-specific gates before a raw rehearsal. -->
1. Do not enter Stage 2A unless Stage 1.5 was signed GO by PC1, PC2, and PC3 owners, the PC4 private
   live source has passed isolated content, lifecycle, wall-pacing and actor-lineage checks, and the exact
   LAN gateway, PC2 raw-recorder, no-actuation and written-authorization gates are approved. Missing
   map, transform, `ego_actual`, or common-time evidence blocks aligned Stage 2B and full Stage 2
   acceptance, but it must be reported separately from this transport-only gate.
<!-- HH_260810 - Preserved PC1 processes configuration and object subscriptions across the stage transition. -->
2. Keep the same unchanged PC1 v2 stack, RX-only status source, route, physical source, and no-actuation
   invariant. Do not restart PC1 merely because PC4 is introduced.
<!-- HH_260810 - Required a fresh evidence directory for each shadow condition. -->
3. Start a new PC1 bag and independent evidence window before the gateway for each approved live-source
   run ID: `TRANSPORT_LIVE_R01`, `TRANSPORT_PC4_STOP_R01`, and `TRANSPORT_PC4_RESTART_R01`.
   An archived-source rehearsal requires a separate reviewed replay profile and runbook.
<!-- HH_260810 - Assigned raw PC4 evidence to PC2 while PC1 watches only its canonical boundary. -->
4. Wait for the PC2 owner to confirm that the namespaced PC4 stream is connected only to the PC2 raw
   recorder. Audit the Domain 10 graph and reject any PC1 application subscriber to that stream.
<!-- HH_260810 - Measured PC4 event noninterference at the existing PC1 boundary. -->
5. During PC4 start, adapter stop, gateway stop, and restart markers, continue recording canonical
   PredictedObjects, trajectory, controller output, final command, MRM, status, process state, and CAN counters.
<!-- HH_260810 - Avoided an invalid byte-identical claim for a naturally changing physical scene. -->
6. Require writer/GID/topology continuity and no event-correlated gap, reset, all-zero substitution, or module
   transition attributable to PC4; do not require naturally changing physical payloads to be byte-identical.
<!-- HH_260810 - Required gateway-first isolation without introducing a nonexistent PC2 selector operation. -->
7. After the gateway owner reports the Stage 2 gateway closed, confirm the PC4 vehicle-domain writers are
   gone while PC2 canonical input and PC1 Planning remain healthy; then close the PC1 evidence window.

<!-- HH_260810 - Defined the exact graph audit for the raw shadow boundary without subscribing to its payload. -->
```bash
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/pc4/virtual_obstacles/tracked_objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /diagnostics/pc4/object_adapter
ROS_DOMAIN_ID=10 ros2 topic info --verbose /perception/object_recognition/objects
ROS_DOMAIN_ID=10 ros2 topic info --verbose /planning/scenario_planning/trajectory
```

<!-- HH_260810 - Defined the expected Stage 2 ownership visible from PC1. -->
The namespaced PC4 topics may show the approved gateway writer and the PC2 raw recorder subscriber. They must
show no PC1 Planning, Control, or application logger subscriber. Canonical PredictedObjects must still show
exactly the existing PC2 predictor writer.

<!-- HH_260810 - Defined the complete PC1 raw-shadow pass gate. -->
### Stage 2A PC1 evidence complete

<!-- HH_260810 - Required canonical source ownership to remain unchanged by PC4 transport. -->
- One PC2 canonical PredictedObjects writer and one PC1 trajectory writer persist without GID drift or an
  event-correlated gap during all PC4 cases.
<!-- HH_260810 - Required PC4 to remain outside the PC1 application graph. -->
- No PC1 application subscribes to the namespaced PC4 object or diagnostic, and no PC4/gateway process writes
  a canonical, Planning, Control, vehicle, system, TF, clock, or CAN topic.
<!-- HH_260810 - Required PC4 failure to have no causal effect on physical Planning in raw shadow. -->
- PC4 never-start, start, stop, kill marker, gateway stop, and restart cause no PC1 module reset, canonical
  source switch, unexplained trajectory transition, or MRM transition.
<!-- HH_260810 - Repeated the zero-actuation invariant for the entire distributed window. -->
- Vehicle remains independently MANUAL/PARK/stationary; sender, adapter, `/to_can_bus`, engage calls, and CAN
  TX delta remain zero.
<!-- HH_260810 - Kept latency claims behind the four-host clock gate. -->
- PC1 reports only continuity unless the four-host Chrony acceptance is signed; it does not turn a same-DDS
  timestamp difference into an unqualified one-way-latency result.

<!-- HH_260810 - Defined conditions that immediately block or abort tomorrow's PC1 work. -->
## Immediate NO-GO and abort conditions

<!-- HH_260810 - Blocked the test when the safe status prerequisite is missing. -->
- Approved receive-only status source absent, unresolved, or capable of starting a sender/adapter.
<!-- HH_260810 - Aborted on any sign of a physical command path. -->
- `run_bridge`, SocketCAN sender, command adapter, `/to_can_bus` endpoint, CAN writer FD, engage/autonomous
  invocation, or any increase in the PC1 CAN TX counter.
<!-- HH_260810 - Blocked attribution when the physical Planning baseline is already invalid. -->
- Missing/stale canonical objects, unset route, duplicate writer, stale MRM, localization dependency failure,
  or a baseline trajectory already stopped for an unrelated reason.
<!-- HH_260810 - Aborted on direct PC4 ownership leakage into PC1. -->
- A PC1 application subscribes directly to PC4 objects, or PC4/gateway publishes canonical objects,
  Planning, Control, vehicle, system/MRM, TF, clock, service/action/parameter, or CAN interfaces.
<!-- HH_260810 - Prevented unsafe timing and coordinate claims. -->
- Different/unapproved Chrony sources while one-way latency is being calculated, or an unapproved map/
  transform while claiming live ego or object alignment.

<!-- HH_260810 - Defined evidence-preserving PC1 rollback without using actuation or source-control commands. -->
## Rollback

<!-- HH_260810 - Kept Stage 1.5 rollback independent of the absent PC4 source. -->
For a Stage 1.5 failure, keep PC4 and every gateway off, preserve the failed bag and recorder output, keep
independent CAN/graph recorders active through the final snapshot, and have the PC owners stop their stacks
using the approved operator order. Do not start `run_bridge` or bypass Planning/MRM to repair a baseline.

<!-- HH_260810 - Removed raw Stage 2 transport without waiting for a nonexistent selector or cache. -->
For a Stage 2 failure, the gateway owner closes the PC4 gateway immediately. PC1 does **not** request
`real_only`, clear a PC2 accepted cache, or wait for accepted status because none of those Stage 3 components
exists tomorrow. Confirm PC4/gateway-owned vehicle-domain endpoints are zero and the physical canonical path
is still the v2 baseline.

<!-- HH_260810 - Preserved independent safety evidence until all residue checks are complete. -->
After gateway isolation, retain the PC1 bag, process audit, engage-call record, and CAN counter observer until
the after-state is captured. Then stop PC1 and the RX-only profile only through their approved owner procedure,
record zero sender/adapter/`/to_can_bus` residue and zero CAN TX delta, and freeze a new checksum manifest.

<!-- HH_260810 - Prevented failed evidence from being overwritten by a retry. -->
Every retry uses a new run ID and new evidence directory. Never delete, edit, append a repair into, or reuse a
failed bag, graph snapshot, process log, CAN snapshot, or checksum manifest.

<!-- HH_260810 - Separated tomorrow's raw-shadow work from the unimplemented canonical integration stages. -->
## Explicitly deferred Stage 3 and later work

<!-- HH_260810 - Kept PC2 integration components out of tomorrow's PC1 test. -->
Tomorrow does not implement or claim a PC2 `pc4_input_validator`, TTL cache, session/map gate,
physical-main fuser, source selector, accepted-PC4 status, availability/MRM consumer, or canonical fusion.

<!-- HH_260810 - Kept later CAN safety changes blocked behind separate implementation and review. -->
Stage 5 additionally requires post-v2 PC1 source work: independent monotonic-age and validity watchdogs for
both `control_cmd` and the active cruise/control-request source, fail-closed request-bit clearing when either
input is stale/invalid, exclusive CAN writer and ID ownership, reviewed safe fallback, actual IONIQ geometry,
and enforced `0.833 m/s` limits. None is authorized or tested tomorrow.

<!-- HH_260810 - Bounded the claims that PC1 evidence may support after tomorrow's run. -->
## Allowed result statement

<!-- HH_260810 - Limited a successful PC1 result to baseline stability and raw-shadow noninterference. -->
A successful result may state that the unchanged PC1 v2 Planning/Control computation consumed one PC2
canonical PredictedObjects source during a stable three-PC baseline, and remained non-actuating and
unaffected at its canonical boundary while PC4 namespaced data was transported to a PC2 raw recorder.

<!-- HH_260810 - Forbade unsupported Planning fusion network safety and vehicle claims. -->
It must not claim that PC4 objects changed real Planning, that PC2 fused physical and virtual perception,
that AEB consumed a PC4 object, that packet PDR was proven without packet evidence, that one-way latency was
valid without clock acceptance, or that CAN, engage, vehicle response, moving VILS, or Stage 3–5 passed.
