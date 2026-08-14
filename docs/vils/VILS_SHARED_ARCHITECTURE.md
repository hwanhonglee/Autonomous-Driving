<!-- HH_260812 - Define the cross-PC vehicle-in-the-loop digital-twin integration contract. -->

# VILS shared architecture

Status: **PROPOSAL — cross-PC review required before implementation**

Date: 2026-08-12

Applies to: IONIQ EV PC1, PC2, PC3, and the proposed PC4 simulator host

## Companion implementation records

This file defines the shared VILS architecture. Work performed on each vehicle PC is recorded in
a separate file so implemented evidence is not confused with the proposed cross-PC design:

- `docs/vils/VILS_PC1_WORK_HISTORY.md` — PC1 command/change/incident/VILS review;
- `docs/vils/VILS_PC2_WORK_HISTORY.md` — PC2 audit, implementation, validation, release, limitations,
  and proposed VILS responsibility;
- `docs/vils/VILS_PC3_WORK_HISTORY.md` — PC3 sensing/localization/system release and
  VILS handoff.

The documentation set contains this architecture file and exactly three owner-authored PC records.

## 1. Purpose

This document defines how a PC4 virtual environment can participate in a real-vehicle test
without becoming a second localization, planning, control, or actuation authority.

The target is a vehicle-in-the-loop digital twin:

- the real vehicle sends its measured motion, final commanded control, and measured response to
  PC4;
- PC4 aligns a digital ego vehicle to that measured state;
- PC4 creates virtual actors in the same Lanelet2/PCD map coordinate system;
- PC4 sends only validated virtual-object information back to the vehicle stack;
- PC2 fuses physical and virtual objects or explicitly selects the required source by mode;
- PC1 plans and controls the real vehicle using one canonical perception output;
- PC3 remains the real map, localization, and TF authority. A common four-PC time source must be
  selected and verified before timestamp-sensitive tests.

This architecture tests planning and control against virtual obstacles while the real vehicle
moves on the corresponding physical track. It does **not** validate that real sensors could
detect those virtual actors. Synthetic LiDAR/camera injection is a separate sensor-in-the-loop
test mode.

## 2. Decision summary

The intended **hybrid-mode** closed loop is:

```text
PC3 real localization --------------------------+
PC1 report telemetry/final command/trajectory --+--> PC4 digital twin
                                                  |       |
                                                  |       | virtual tracked objects
                                                  |       v
PC2 physical perception ----------------------> PC2 fail-open fusion
                                                          |
                                                          v
                                    /perception/object_recognition/objects
                                                          |
                                                          v
                                             PC1 planning and control
                                                          |
                                                          v
                                                   physical vehicle
                                                          |
                                                          +----> measured result back to PC4
```

The following rules are mandatory:

1. PC1 is the only planning/control authority and the only physical actuation owner.
2. PC2 is the only publisher of the canonical final `PredictedObjects` topic in every armed
   injection mode.
3. PC3 is the only real `map -> base_link` localization authority.
4. PC4 publishes only namespaced virtual actors, health, and scenario metadata toward the
   vehicle domain.
5. PC4 must not publish vehicle `/clock`, `/tf*`, localization, vehicle status, planning,
   control, or system-command topics.
6. A PC4 failure must never stop the PC2 physical-perception stream or leave stale virtual
   objects active.

## 3. Source-of-truth ownership

| Data or function | Authoritative owner | Notes |
|---|---|---|
| Lanelet2/PCD release and map hash | PC3 deployment | PC4 must load the exact approved map bundle. |
| Real ego pose and twist | PC3 localization | `/localization/kinematic_state` is the digital-twin ego reference. |
| Real ego acceleration | PC3 localization | Used for measured-response comparison. |
| Real vehicle speed, steering, gear, and mode | PC1 vehicle interface | Diagnostic/comparison reports until the custom CAN decode is independently validated; they do not replace PC3 odometry as `ego_actual`. |
| Route, trajectory, and control command | PC1 | Read-only input to PC4 for visualization/model comparison. |
| Physical obstacles | PC2 perception | Must continue if PC4 disappears. |
| Virtual actors and scenario lifecycle | PC4 | Must use a dedicated namespace and stable session identity. |
| Physical/virtual object fusion | PC2 | PC2 remains the final perception owner. |
| Canonical predicted objects | PC2 | Exactly one publisher. |
| Physical CAN transmission | PC1 only | PC4 and PC3 must not obtain this authority. |

## 4. Supported operating modes

### 4.1 Shadow mode — mandatory initial/default mode

PC4 receives real state and runs the virtual world, but its actors are not connected to the
canonical perception pipeline. PC4 publishes a candidate/debug stream only.

Use this mode to validate:

- map alignment;
- timestamp and latency;
- actor identity and lifetime;
- real-ego mirroring;
- predicted physical-versus-model vehicle motion;
- disconnect/restart behavior.

Boot and reconnect defaults must always return to shadow/off. Enabling hybrid mode requires an
explicit operator action while the vehicle is in MANUAL and stationary.

### 4.2 Hybrid additive mode — target vehicle-in-the-loop mode

PC2 physical objects and PC4 virtual objects are combined. PC2 remains the single final
publisher. This mode exercises real planning and control against both physical and virtual
obstacles.

### 4.3 Virtual-only mode — optional, separate test profile

PC4 replaces the PC2 physical-object **source** for a controlled simulation test. The selector
boundary must match the PC4 message type: select physical versus virtual at the tracked-object
boundary when PC4 provides stable `TrackedObjects`, or at the detected-object boundary when PC2
must track PC4 `DetectedObjects`. The existing PC2 map-based predictor remains the only publisher
of `/perception/object_recognition/objects` in every injection mode. The PC2 physical stream is
preserved under a shadow/debug topic rather than deleted. PC4 never publishes the canonical topic
directly.

This selection controls object recognition only. Selected PC1 planning modules and AEB also consume
`/perception/obstacle_segmentation/pointcloud`, and planners consume
`/perception/occupancy_grid_map/map`. PC1's audited AEB configuration uses physical pointcloud and
does not use predicted objects. A test must therefore declare whether it is:

- object-virtual-only with the physical pointcloud retained as an independent safety overlay; or
- full obstacle virtual-only, which also replaces pointcloud/occupancy inputs and is limited to
  stationary/no-actuation until a separate AEB/sensor-in-the-loop safety design is approved. Low
  speed alone does not make removal of the physical obstacle safety channel acceptable.

Virtual-only mode is not the recommended default for real-vehicle digital-twin testing.

### 4.4 Explicit source-selection state machine

Implementation must use one explicit state machine rather than independent Boolean launch flags.
The minimum states and loss behavior are:

| Mode | PC2 physical objects | PC4 objects | Canonical trigger | PC4 loss | Permitted stage |
|---|---|---|---|---|---|
| `real_only` | canonical source | disconnected | physical | no effect | Normal baseline and rollback |
| `shadow` | canonical source | debug only | physical | diagnostic; disarm PC4 session | Stages 1-3 |
| `hybrid_optional` | main source | accepted additive | physical-main fuser | discard virtual within TTL; continue physical | No-actuation first; moving use needs separate approval |
| `hybrid_required` | main source | required additive | physical-main fuser | unavailable; inhibit engage or separately validated MRM path | Stage 5 only after loss routing is proven |
| `virtual_only_required` | shadow/log only | selected source | accepted PC4 snapshot through PC2 selector/predictor | no automatic physical failover; inhibit engage or separately validated MRM path | Stationary/no-actuation; moving use needs separate AEB/sensor safety approval |

Mode/session changes clear the accepted PC4 snapshot and return to `shadow`. Only a stationary,
manual operator action may arm a required mode.

## 5. Real vehicle to PC4 contract

The bridge direction is read-only from the vehicle domain into PC4.

| Topic | ROS type | Owner | Suggested use |
|---|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | PC3 | Authoritative real ego pose/twist in `map`. |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | PC3 | Measured ego acceleration. |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | PC1 | Comparison telemetry; DBC/unit validation remains open and it must not override PC3 odometry twist. |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | PC1 | Measured-response comparison telemetry. |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | PC1 | Comparison telemetry; raw mapping proof remains open. |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | PC1 | Reported MANUAL/AUTONOMOUS comparison state. |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | PC1 | Final command for shadow vehicle-model comparison only. |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | PC1 | Planned path visualization and comparison. |
| `/planning/mission_planning/route` | `autoware_planning_msgs/msg/LaneletRoute` | PC1 | Route-aware scenario placement. |

Map files should normally be deployed to PC4 as an approved immutable bundle rather than sent
continuously over DDS. The deployment must record SHA-256 hashes for:

- `lanelet2_map.osm`;
- `pointcloud_map.pcd`;
- `map_projector_info.yaml`;
- any simulator-world-to-map transform configuration.

## 6. PC4 to vehicle contract

### 6.1 Preferred virtual-object interface

```text
Topic: /perception/pc4/virtual_obstacles/tracked_objects
Type:  autoware_perception_msgs/msg/TrackedObjects
Frame: map
QoS:   reliable, volatile, keep-last depth 1
Rate:  initial target 10-20 Hz; validate with measured end-to-end latency
```

Each virtual actor must provide:

- a UUID stable for that actor's lifetime and unique to the PC4 session;
- classification and probability;
- existence probability;
- pose, orientation, and covariance;
- twist, angular rate where applicable, and covariance;
- shape and dimensions;
- a real synchronized timestamp;
- deterministic creation, modification, and deletion behavior.

If the simulator cannot provide stable actor identity, PC4 may publish a dedicated
`autoware_perception_msgs/msg/DetectedObjects` stream instead. In hybrid modes, PC2 needs a
physical-main, fail-open association gate before tracking and must not use a two-input synchronizer
that blocks physical perception when PC4 is absent. In `virtual_only_required`, accepted PC4 data
drives the selected path and missing PC4 is a fault; it must not wait for a physical trigger.

### 6.2 Health and scenario metadata

PC4 must expose diagnostics containing at least:

- session ID;
- monotonically increasing sequence number;
- scenario name/version;
- map hashes;
- source timestamp and vehicle-receive age;
- actor count;
- bridge state;
- scenario readiness and requested profile;
- stale, rejected, malformed, replay, and transform-failure counters.

Diagnostics can use `diagnostic_msgs/msg/DiagnosticArray` under a stable `pc4/digital_twin`
hardware ID. A fresh empty object array means “no virtual actors.” Missing messages mean a
source fault and must not be interpreted as an empty scene.

PC4 does not own the vehicle-selected mode or armed state. The vehicle-side operator/supervisor
owns those states. The PC4 raw heartbeat is evidence, not direct permission to arm or an MRM
authority. The required accepted-state chain must be:

```text
PC4 raw health/session/object snapshot
  -> PC2 validation, TTL, map/frame, writer, and source-selection gate
  -> PC2 accepted-PC4 status
  -> PC3 authoritative diagnostic/availability and MRM policy
  -> PC1 operation-mode and vehicle-command gate
```

PC2 validates writer identity, session, sequence, frame, stamp, map hash, TTL, and the object
snapshot before publishing its machine-readable accepted-input status. PC4 must never publish
operation-mode availability, MRM state, emergency heartbeat, or a vehicle armed state. Raw-health
and accepted-status rates/timeouts must be measured and selected so multiple updates fit inside the
required loss deadline; a 1 Hz diagnostic alone cannot satisfy a sub-second source-loss gate.

## 7. PC2 fusion contract

The recommended hybrid pipeline is:

```text
PC2 physical DetectedObjects
  -> PC2 multi-object tracker
  -> /perception/object_recognition/tracking/pc2_physical_objects

PC4 virtual TrackedObjects
  -> validation and TTL gate
  -> /perception/pc4/virtual_obstacles/tracked_objects

PC2 physical-main fail-open tracked-object fuser
  -> /perception/object_recognition/tracking/objects

PC2 map-based prediction
  -> /perception/object_recognition/objects
```

In `hybrid_optional` and `hybrid_required`, the fuser is triggered by the PC2 physical stream
and must always pass physical objects even when PC4 has never connected, is delayed, restarts, or
disconnects. This physical-trigger rule does not apply to `virtual_only_required`; that mode uses
the type-matched selector described in Section 4.3 and treats missing accepted PC4 data as a fault,
not a clear scene or an automatic physical failover.

For overlapping objects:

- use association and deduplication;
- prefer the PC2 physical object for an associated pair;
- preserve unique PC4 actors that do not overlap physical detections;
- publish source-specific debug counts;
- remove a virtual object when its TTL expires;
- never republish a stale PC4 sample merely to keep a topic alive.

The existing generic mergers are not drop-in safety solutions:

- a two-input approximate-time merger can stop producing output when either input disappears;
- a simple concatenation merger does not deduplicate overlapping physical/virtual objects;
- directly adding PC4 as another tracker input can make a delayed PC4 stream the processing
  trigger and can starve tracking after PC4 disconnects.

A main-driven tracked-object merger is the closest existing pattern, but it needs explicit PC4
semantics, physical-object priority, measured TTL, source diagnostics, and disconnect tests.

## 8. Digital-twin ego model

PC4 should maintain two different ego representations.

### `ego_actual`

- pose and twist are slaved to PC3 `/localization/kinematic_state`;
- vehicle reports from PC1 provide diagnostic/comparison speed, steering, gear, and mode until the
  custom CAN decode is validated; they do not replace PC3 odometry;
- this ego anchors the real vehicle in the virtual scene;
- this state determines which virtual actors are relevant to the real vehicle.

### `ego_model`

- driven by the same final PC1 control command inside the simulator vehicle model;
- used only to compare expected and actual vehicle response;
- reports position, yaw, velocity, steering, acceleration, and latency error;
- remains under a PC4 debug namespace;
- must never publish localization or vehicle status into the vehicle domain.

PC4 must not integrate control commands to create the authoritative real ego pose. Doing so will
accumulate model drift and can misplace virtual actors relative to the physical vehicle.

## 9. Map and coordinate contract

Using the same Lanelet2 and PCD bundle on the real and virtual systems makes a shared local
`map` frame possible, but identical filenames alone are insufficient.

Before enabling any injection mode, verify:

- byte-identical map hashes;
- identical map origin and projection parameters;
- metre/radian units;
- X/Y/Z axes and handedness;
- yaw-zero definition and sign;
- ground height convention;
- at least three non-collinear control points;
- real `base_link` and digital `ego_actual` overlay within an approved tolerance.

The operator identifies the real C-track reference as `52SCF60`, while the active historical
PC3 external map configuration has used an origin label `52SCF0` plus map-specific GNSS poser
subtractions. That naming/configuration mismatch must be resolved explicitly rather than inferred.

The current PC3 GNSS-to-local-map compatibility correction is approximately:

```text
x -= 60966.4679793288
y -= 65973.64540576655
z -= 15.816125382
```

If PC4 loads the same local Lanelet2/PCD coordinates and places actors directly in `map`, PC4
must **not** apply this GNSS correction again. It is relevant only when converting a geodetic
GNSS result into the current local-map convention.

Long term, the GNSS/map correction should be moved from duplicated constants into one versioned
shared projection/transform configuration. PC4 must never independently copy and retune these
numbers.

## 10. Time contract

The real vehicle stack uses system/wall-clock time. Therefore:

- PC4 must synchronize to one approved four-PC source. PC3 Chrony is the proposed vehicle-LAN
  source, but it is not yet an accepted contract: a 2026-08-12 PC1 run selected a public NTP
  server instead of PC3;
- PC4 vehicle-facing ROS nodes must use `use_sim_time=false`;
- `/clock` must not cross into the vehicle domain;
- simulator tick time must not be copied directly into vehicle-facing message stamps;
- vehicle-facing message stamps must represent the source snapshot time mapped to synchronized
  wall clock, not the bridge receive time;
- old, future, non-monotonic, and replayed samples must be rejected;
- receive age and end-to-end latency must be monitored independently.

Initial rate targets for measurement, not final acceptance thresholds:

- real ego state: 30-50 Hz;
- vehicle reports: approximately 50 Hz where available;
- virtual tracked objects: 10-20 Hz;
- PC4 raw-health and accepted-status rates: derived from the accepted loss timeout and measured so
  that multiple updates fit within that timeout.

Final TTL and latency limits must be calculated from measured network/processing delay and the
approved test speed/stopping envelope.

Until all four hosts show the same selected source and a recorded acceptable offset/jitter window,
timestamp-sensitive VILS data collection and every moving stage remain blocked.

## 11. DDS and bridge isolation

PC4 should run in a simulator ROS domain distinct from the vehicle domain. PC1-PC3 have been
verified on ROS domain 10 with CycloneDDS; the separate PC4 domain and bridge are still proposed
and unverified. Use one explicit domain bridge with a deny-by-default allowlist.

Allowed vehicle-to-PC4 direction:

- the read-only topics listed in Section 5;
- minimal map metadata/hash and approved static configuration where required.

Allowed PC4-to-vehicle direction:

- the dedicated virtual-object topic;
- PC4 health/scenario diagnostics.

Forbidden PC4-to-vehicle direction:

```text
/clock
/tf
/tf_static
/localization/**
/vehicle/**
/planning/**
/control/**
/system/**
/to_can_bus
/pc1/can/**
/external/**
/api/autoware/set/**
/autoware/engage
/from_can_bus
all services, actions, and parameter interfaces
```

The bridge must not create a physical CAN path. Firewall/interface allowlists and a capture log
should be retained for every vehicle test. Vehicle-to-PC4 inputs must be remapped under a private
`/vils/in/**` namespace in the PC4 domain so a same-name simulator publisher cannot loop back to
the canonical vehicle topic.

## 12. Failure policy

### Shadow/optional mode

- PC4 failure raises diagnostics;
- physical perception continues;
- stale virtual objects are discarded;
- no automatic switch to a second final-object publisher occurs.

### Virtual-required test mode

When the test scenario requires virtual obstacles, PC4 loss must not silently turn the road
clear. The required-mode fault must:

1. discard stale virtual objects;
2. mark PC4/scenario availability false;
3. inhibit a new autonomous engagement;
4. request the controlled-stop/MRM path if already autonomous, but only after that path is
   separately implemented and validated;
5. require a stationary manual re-arm after recovery.

Do not hold stale obstacles indefinitely and do not allow an automatic reconnect to re-arm the
scenario.

This loss-to-stop path is a required design, not an existing verified feature. PC4 must never
publish `/system/fail_safe/mrm_state` or directly call the vehicle command gate. PC2 must publish
accepted-source status, the authoritative vehicle-side diagnostic/system owner must apply the
approved availability/MRM policy, and PC1 must only consume that validated result. Moving
`hybrid_required` and `virtual_only_required` modes are prohibited until this chain is tested while
parked with no physical command sink.

## 13. PC-specific implementation checklist

### PC4

- [ ] Identify the simulator's source message, frame, time base, actor ID, and update rate.
- [ ] Implement the simulator-to-Autoware object adapter.
- [ ] Implement `ego_actual` from PC3 real localization; use PC1 vehicle reports only for
      measured-response comparison until their decode is validated.
- [ ] Implement the isolated `ego_model` comparison path.
- [ ] Load and verify the exact map bundle hashes.
- [ ] Calibrate simulator-world to Autoware-`map` axes and transform.
- [ ] Add session, sequence, heartbeat, validation, and replay rejection.
- [ ] Configure a separate ROS domain and allowlisted bridge.
- [ ] Prove zero PC4 vehicle-command, TF, clock, and localization publishers.

### PC2

- [ ] Keep the physical tracker output on an internal physical-only topic.
- [ ] Validate PC4 frame, stamp, UUID, dimensions, velocity, covariance, and map/session metadata.
- [ ] Add a physical-main, fail-open tracked-object fuser.
- [ ] Add association/deduplication with physical priority.
- [ ] Add PC4 TTL and source-specific diagnostics.
- [ ] Keep the canonical `PredictedObjects` publisher count exactly one.
- [ ] Prove physical output continues through PC4 process kill and cable disconnect.

#### PC2 owner review and implementation decision

PC2 agrees with object-level injection only under the following clarified contract:

1. PC4 sends only PC4-native scenario actors. Real objects optionally mirrored from PC2 into PC4
   for visualization or evaluation must carry a separate source registry and must never return in
   the PC4 virtual-actor snapshot. This prevents a physical-object-to-PC4-to-PC2 feedback loop.
2. The PC4 `TrackedObjects` message is a full snapshot. A UUID absent from a new valid snapshot is
   deleted. A fresh empty snapshot means no scenario actors; a missing or stale snapshot means a
   PC4 fault.
3. In hybrid modes, PC2 stores only the latest valid PC4 snapshot, aligns or predicts it to the
   triggering physical timestamp within a measured TTL, and never creates an independent virtual
   tracklet or holds an actor beyond that TTL.
4. In hybrid modes, fusion is triggered by the physical PC2 stream. When physical and virtual
   actors associate, the physical UUID, pose, twist, classification, and shape win; only unmatched
   virtual actors are added. PC2 records provenance in a debug sidecar because the canonical
   message has no source field.
5. The PC4 status, not a launch argument, owns session ID and sequence. A session change clears the
   snapshot, disarms injection, and returns PC2 to shadow mode. Reconnection never re-arms it.
6. PC4 converts simulator-world coordinates into the approved `map` frame before publication.
   PC2 rejects all other frames, so PC4 TF does not enter the vehicle domain.
7. PC3 remains the physical LiDAR owner. Its canonical pointcloud is
   `/sensing/lidar/concatenated/pointcloud`. PC2 historically expected
   `/sensing/lidar/top/pointcloud_before_sync`; one runtime relay proved that mismatch but was
   temporary and is not a current or persistent ownership contract.
8. Physical-triggered alignment/fusion applies only to hybrid modes. In
   `virtual_only_required`, PC2 processes the accepted PC4 snapshot without waiting for a
   physical message and treats loss as a required-source fault.
9. PC2 will implement the Section 4.4 state machine: `real_only`, `shadow`,
   `hybrid_optional`, `hybrid_required`, and `virtual_only_required`. The old
   `inject_stationary`/`closed_loop_vils` names are superseded so source ownership and loss policy
   are explicit. No injection code is enabled by this architecture-only branch.

Before implementation, PC2 additionally requires the bridge host/IP, simulator source message,
PC4 status contract, map/projector/transform manifest, PC3 Chrony offset measurements, and the
exact PC1 loss-to-MRM behavior. DDS writers are bound to the approved node identity and current GID
at session arm time; a writer or session change is a fault rather than a static forever-GID rule.

### PC1

- [ ] Continue consuming only `/perception/object_recognition/objects`.
- [ ] Add/verify PC4-required availability and engagement gating.
- [ ] Verify diagnostics-to-MRM-to-vehicle-command-gate behavior while parked.
- [ ] Confirm whether virtual objects are intended to affect AEB. The audited PC1 configuration
      had predicted-object AEB input disabled and pointcloud input enabled; changing that is a
      separate safety decision.
- [ ] Keep physical CAN authority unique to PC1.

#### PC1 owner review and implementation decision

PC1 accepts the VILS architecture only with the following audited constraints. The detailed PC1
change and incident history is recorded in `VILS_PC1_WORK_HISTORY.md`.

1. PC1 continues to consume one canonical `autoware_perception_msgs/msg/PredictedObjects` stream
   on `/perception/object_recognition/objects`. In hybrid and virtual-only modes, PC2 remains the
   sole final map-based-prediction publisher. A PC4 direct canonical publisher is never permitted
   by this contract.
2. The current AEB settings are `use_pointcloud_data: true`,
   `use_predicted_object_data: false`, and `aeb_hz: 10.0`. Consequently, virtual predicted objects
   accepted into the canonical stream would be eligible to influence selected object-based
   planning, but they do not become AEB obstacles. Changing that is a separate safety design and
   validation task.
3. Retaining PC2/PC3 physical obstacle pointcloud for AEB is the preferred first real-vehicle
   overlay. It is not a pure virtual-only causality experiment. Removing the physical pointcloud
   or occupancy path is limited to stationary/no-actuation until a separate AEB/sensor-in-the-loop
   safety design is approved; a low-speed limit by itself is insufficient.
4. PC4-required availability and loss-to-MRM routing do not currently exist as a validated PC1
   feature. PC4 must publish health/session data only; it must not publish MRM state. The approved
   system owner must convert a required-PC4 fault into engagement inhibition or the validated
   controlled-stop request.
5. PC1's vehicle command gate depends on `/system/fail_safe/mrm_state`. Its audited heartbeat
   timeout is 0.5 s. A prior PC3 MRM heartbeat loss caused the gate to replace a positive follower
   command with the configured `-2.4 m/s²` emergency command. PC3 MRM continuity must therefore be
   proven before adding PC4-required loss behavior.
6. Historical `run_bridge` is not receive-only: it launches the physical CAN receiver, sender, and
   `twistController2VCU2EPS2ACC_node`. The adapter republishes its cached acceleration and steering
   in CAN ID `0x630` every 30 ms and has no command-age watchdog. Stages 2 through 4 must keep the
   sender and adapter absent, `/to_can_bus` absent, and CAN TX unchanged.
7. Stage 5 remains blocked until PC1 has a bench-proven command-freshness timeout and request-bit
   clear, exclusive `/to_can_bus`/CAN-ID ownership, verified IONIQ EV geometry, stable PC2
   perception, stable PC3 localization/MRM, and an approved common Chrony source.
8. PC4 may read PC1 vehicle reports, trajectory, and final control command for comparison. These
   are telemetry only. PC1 CAN-derived reports remain diagnostic until the custom decode is
   independently validated; PC3 odometry remains the `ego_actual` motion authority. No PC4 output
   can feed `/control/**`, `/vehicle/**`, `/system/**`, or the physical CAN path.

PC1 requires every test record to include the exact Git/map/config hashes, publisher identities,
QoS, topic rates and age, PC4 session/sequence, localization and MRM continuity, CAN statistics,
and start/stop logs in one common time window. Restart or writer/session change returns the
integration to shadow/off and requires a stationary manual re-arm.

### PC3

- [ ] Publish healthy localization, `map -> base_link`, and required map/static data.
- [ ] Confirm Chrony service and measure PC4 offset/stability.
- [ ] Publish/version the authoritative map hashes and coordinate convention.
- [ ] Resolve the `52SCF60` operator reference versus historical `52SCF0` configuration naming.
- [ ] Do not add PC4 simulation localization or TF as a second authority.

#### PC3 owner handoff reflected in the shared contract

PC3's owner-authored completion record accepts the VILS direction with these boundaries:

1. PC3 supplies the real localization, `map -> base_link`, map identity/projection convention,
   physical LiDAR cloud, timestamp/TF context, and GNSS/INS/localization diagnostics.
2. PC4 must not publish competing `/tf`, `/tf_static`, `/clock`, localization, control, vehicle,
   or system-authority topics into the vehicle domain.
3. `map_projector_info.yaml` alone is not a complete description of the legacy C-track correction.
   The simulator-world-to-Autoware-`map` transform must be reviewed once and verified against map
   hashes, non-collinear control points, and the live ego pose; the historical correction must not
   be applied twice.
4. PC2 retains physical/virtual fusion and sole canonical-object ownership, while PC1 retains
   planning/control/CAN policy.
5. PC3's controlled post-power restart, sustained localization, MRM ownership/continuity, and
   common-clock acceptance are still open gates; its published source/offline tests do not clear a
   moving VILS stage.

## 14. Validation sequence

### Stage 0 — offline contract and unit tests

- validate malformed, NaN, infinite, oversized, out-of-range, old, future, and non-monotonic
  objects;
- validate UUID stability and deletion;
- validate map hash and transform rejection;
- validate physical-priority deduplication;
- validate PC4 startup, restart, and missing-input behavior.

### Stage 1 — isolated PC4

- run PC4 in its private domain;
- verify actor creation/modification/deletion;
- verify `ego_actual` replay from a recorded real localization stream;
- verify no forbidden publishers.

### Stage 1.5 — three-PC baseline without PC4

- run PC1-PC3 for at least ten continuous minutes before introducing PC4;
- require exactly one fresh PC2 canonical object publisher plus continuous obstacle-segmentation
  pointcloud;
- require continuous PC3 odometry, acceleration, and `map -> base_link`, with no NDT
  jump/reinitialization beyond the accepted bound;
- require exactly one MRM-state publisher at the expected rate and `NORMAL/NONE`;
- require one approved common clock source and no duplicate critical publishers;
- after selecting a route, require a stable non-zero baseline trajectory. A PC4 obstacle response
  is not attributable while the baseline is already a zero-velocity start-planner stop path.

### Stage 2 — four-PC shadow, MANUAL/PARK

- do not run the historical `run_bridge`, because it starts the receiver, SocketCAN sender, and
  control-to-CAN adapter together;
- physically secure the vehicle in MANUAL/PARK and block CAN TX/actuation: command adapter count 0,
  SocketCAN sender count 0, `/to_can_bus` publisher count 0 and actuator-capable subscriber count 0,
  no control-request bit on CAN, and `can0` TX counter delta exactly 0 for the full window;
- start logging before the allowlisted DDS/domain bridge;
- verify one DDS/domain bridge and one PC4 virtual-object writer;
- verify map control points and ego overlay;
- measure QoS, rate, timestamp age, and latency;
- keep virtual objects out of the canonical planning topic.

### Stage 3 — candidate fusion, still no actuation

- publish to a candidate fusion/debug topic;
- verify real/virtual association and physical priority;
- test PC4 process kill, reboot, cable removal, packet delay, and message replay;
- require physical perception to continue with no stale virtual objects.
- preserve every Stage 2 no-actuation invariant.

### Stage 4 — canonical fusion, no actuation

- make PC2 the sole canonical publisher;
- observe PC1 planning/stop trajectory response;
- prove availability and MRM fault routing without a physical command sink;
- verify object publisher count remains exactly one.
- preserve every Stage 2 no-actuation invariant.

### Stage 5 — closed-course low-speed test

- safety driver, E-stop, spotters, and rollback ready;
- begin with one static virtual obstacle;
- enforce the initial speed ceiling in the approved planner/controller configuration at
  `0.833 m/s` (3 km/h), verify the resulting trajectory and final command limit, and do not rely on
  an operator promise. The currently preserved `max_vel: 36.0` and `sample_vehicle` geometry are
  not accepted for this gate;
- measure detection-to-plan, plan-to-command, and command-to-vehicle response;
- test only pre-approved failures with the expected controlled-stop behavior;
- expand to moving and multiple actors only after acceptance.

## 15. Minimum acceptance criteria

- [ ] PC2 is the sole canonical final predicted-object publisher in every armed injection mode.
- [ ] PC2 accepted-input status matches the selected mode, PC4 session, writer GID, map hash,
      sequence, and fresh accepted object snapshot.
- [ ] PC4 publishes no vehicle command, TF, clock, localization, or vehicle-status topics.
- [ ] In `real_only`, `shadow`, and `hybrid_optional`, physical perception continues when PC4 has
      never started, stalls, disconnects, or restarts.
- [ ] In `hybrid_optional`, PC4 loss removes virtual objects within the accepted TTL while physical
      perception continues.
- [ ] Stale/future/replayed/misaligned/malformed virtual objects are rejected.
- [ ] No duplicate or ghost objects remain after association within the approved test set.
- [ ] A PC4 actor is deleted within the approved TTL without reappearing from transient history.
- [ ] Map hashes, axes, yaw, height, and three control points match.
- [ ] `ego_actual` tracks the real vehicle without simulator-model drift.
- [ ] `ego_model` remains debug-only and cannot become a vehicle state authority.
- [ ] In a required mode, PC4 loss inhibits engagement or produces the approved controlled stop;
      it never automatically substitutes a clear canonical scene.
- [ ] In `virtual_only_required`, PC4 loss never automatically switches the canonical source to
      physical objects.
- [ ] Restart returns the integration to shadow/off and requires manual re-arm.
- [ ] Rollback to physical-perception-only operation is manual, performed while stationary in
      MANUAL/PARK, and documented. Automatic rollback while moving is allowed only for the
      separately approved `hybrid_optional` policy.
- [ ] A fresh empty PC4 array is accepted as an empty scene only in a scenario where zero virtual
      actors is valid; in an obstacle-required scenario it is a scenario fault.
- [ ] The initial real-vehicle profile keeps the physical AEB pointcloud fresh and does not claim
      that PC4 PredictedObjects are AEB inputs.
- [ ] Stage 5 additionally closes the command-watchdog, exclusive `/to_can_bus` ownership, actual
      IONIQ EV geometry, enforced `0.833 m/s` limit, three-PC stability, localization/MRM
      continuity, and common-clock gates.

## 16. Review questions for PC1, PC2, PC3, and PC4 owners

1. What is the exact PC4 simulator source message, frame, actor-ID model, and publish rate?
2. Can PC4 publish stable `TrackedObjects`, or must PC2 track PC4 `DetectedObjects`?
3. What TTL follows from measured latency and the approved 3 km/h stopping envelope?
4. Which source-selection state is requested, at which detected/tracked boundary is it applied, and
   what is that state's loss policy?
5. Should virtual actors affect planning only, or must AEB also consume them?
6. What exact map files/hashes and simulator-world transform define the C-track release?
7. Which host owns the domain bridge and its allowlist configuration?
8. How will PC4-required diagnostics feed PC1 availability and the validated MRM path?
9. Which logs and metrics constitute a reproducible test record?
10. Who has authority to arm hybrid mode and approve each validation stage?

Implementation must not begin on the physical command path until these questions and the
minimum contract are reviewed by the PC1, PC2, PC3, and PC4 owners.
