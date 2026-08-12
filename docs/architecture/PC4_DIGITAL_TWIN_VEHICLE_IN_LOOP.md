<!-- HH_260812 - Define the cross-PC vehicle-in-the-loop digital-twin integration contract. -->

# PC4 vehicle-in-the-loop digital twin integration

Status: **PROPOSAL — cross-PC review required before implementation**

Date: 2026-08-12

Applies to: IONIQ EV PC1, PC2, PC3, and the proposed PC4 simulator host

## 1. Purpose

This document defines how a PC4 virtual environment can participate in a real-vehicle test
without becoming a second localization, planning, control, or actuation authority.

The target is a vehicle-in-the-loop digital twin:

- the real vehicle sends its measured motion and actual control result to PC4;
- PC4 aligns a digital ego vehicle to that measured state;
- PC4 creates virtual actors in the same Lanelet2/PCD map coordinate system;
- PC4 sends only validated virtual-object information back to the vehicle stack;
- PC2 combines physical and virtual objects;
- PC1 plans and controls the real vehicle using one canonical perception output;
- PC3 remains the real map, localization, TF, and time reference.

This architecture tests planning and control against virtual obstacles while the real vehicle
moves on the corresponding physical track. It does **not** validate that real sensors could
detect those virtual actors. Synthetic LiDAR/camera injection is a separate sensor-in-the-loop
test mode.

## 2. Decision summary

The intended closed loop is:

```text
PC3 real localization --------------------------+
PC1 actual vehicle reports/control/trajectory --+--> PC4 digital twin
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
2. PC2 is the only publisher of the canonical final `PredictedObjects` topic in hybrid mode.
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
| Real vehicle speed, steering, gear, and mode | PC1 vehicle interface | Measured reports, not commands. |
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

PC4 replaces PC2 objects for a controlled simulation test. This must use an explicit selector
or arbiter so that only one source reaches the canonical topic. PC2 and PC4 must never publish
the canonical topic concurrently.

Virtual-only mode is not the recommended default for real-vehicle digital-twin testing.

## 5. Real vehicle to PC4 contract

The bridge direction is read-only from the vehicle domain into PC4.

| Topic | ROS type | Owner | Suggested use |
|---|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | PC3 | Authoritative real ego pose/twist in `map`. |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | PC3 | Measured ego acceleration. |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | PC1 | Actual longitudinal/lateral speed and heading rate. |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | PC1 | Actual steering response. |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | PC1 | Actual gear state. |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | PC1 | MANUAL/AUTONOMOUS state. |
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
`autoware_perception_msgs/msg/DetectedObjects` stream instead. In that case PC2 needs a
main-triggered, fail-open association gate before tracking. It must not use a two-input
synchronizer that can block physical perception when PC4 is absent.

### 6.2 Health and scenario metadata

PC4 must expose diagnostics containing at least:

- session ID;
- monotonically increasing sequence number;
- scenario name/version;
- map hashes;
- source timestamp and vehicle-receive age;
- actor count;
- bridge state;
- armed/shadow/required mode;
- stale, rejected, malformed, replay, and transform-failure counters.

Diagnostics can use `diagnostic_msgs/msg/DiagnosticArray` under a stable `pc4/digital_twin`
hardware ID. A fresh empty object array means “no virtual actors.” Missing messages mean a
source fault and must not be interpreted as an empty scene.

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

The fuser must be triggered by the PC2 physical stream. It must always pass physical objects
even when PC4 has never connected, is delayed, restarts, or disconnects.

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
- vehicle reports from PC1 update measured speed, steering, gear, and mode;
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

Before enabling hybrid mode, verify:

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

- PC4 must synchronize to the approved vehicle time source, currently expected to be PC3
  Chrony on the vehicle LAN;
- PC4 vehicle-facing ROS nodes must use `use_sim_time=false`;
- `/clock` must not cross into the vehicle domain;
- simulator tick time must not be copied directly into vehicle-facing message stamps;
- old, future, non-monotonic, and replayed samples must be rejected;
- receive age and end-to-end latency must be monitored independently.

Initial rate targets for measurement, not final acceptance thresholds:

- real ego state: 30-50 Hz;
- vehicle reports: approximately 50 Hz where available;
- virtual tracked objects: 10-20 Hz;
- PC4 health heartbeat: 1-5 Hz.

Final TTL and latency limits must be calculated from measured network/processing delay and the
approved test speed/stopping envelope.

## 11. DDS and bridge isolation

PC4 should run in a simulator ROS domain distinct from the vehicle domain. The current vehicle
domain is expected to be ROS domain 10 with CycloneDDS. Use one explicit domain bridge with a
deny-by-default allowlist.

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
all services, actions, and parameter interfaces unless individually reviewed
```

The bridge must not create a physical CAN path. Firewall/interface allowlists and a capture log
should be retained for every vehicle test.

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
4. request the previously validated controlled-stop/MRM path if already autonomous;
5. require a stationary manual re-arm after recovery.

Do not hold stale obstacles indefinitely and do not allow an automatic reconnect to re-arm the
scenario.

## 13. PC-specific implementation checklist

### PC4

- [ ] Identify the simulator's source message, frame, time base, actor ID, and update rate.
- [ ] Implement the simulator-to-Autoware object adapter.
- [ ] Implement `ego_actual` from real localization and vehicle reports.
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
3. PC2 stores only the latest valid PC4 snapshot, aligns or predicts it to the triggering physical
   timestamp within a measured TTL, and never creates an independent virtual tracklet or holds an
   actor beyond that TTL.
4. Fusion is triggered by the physical PC2 stream. When physical and virtual actors associate,
   the physical UUID, pose, twist, classification, and shape win; only unmatched virtual actors are
   added. PC2 records provenance in a debug sidecar because the canonical message has no source
   field.
5. The PC4 status, not a launch argument, owns session ID and sequence. A session change clears the
   snapshot, disarms injection, and returns PC2 to shadow mode. Reconnection never re-arms it.
6. PC4 converts simulator-world coordinates into the approved `map` frame before publication.
   PC2 rejects all other frames, so PC4 TF does not enter the vehicle domain.
7. PC3 remains the physical LiDAR owner. Its canonical pointcloud is
   `/sensing/lidar/concatenated/pointcloud`; the current
   `/sensing/lidar/top/pointcloud_before_sync` input is a temporary compatibility relay, not PC2
   sensor ownership.
8. PC2 will implement `real_only`, `shadow`, `inject_stationary`, and supervised
   `closed_loop_vils` modes in that order. No injection code is enabled by this architecture-only
   branch.

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

### PC3

- [ ] Publish healthy localization, `map -> base_link`, and required map/static data.
- [ ] Confirm Chrony service and measure PC4 offset/stability.
- [ ] Publish/version the authoritative map hashes and coordinate convention.
- [ ] Resolve the `52SCF60` operator reference versus historical `52SCF0` configuration naming.
- [ ] Do not add PC4 simulation localization or TF as a second authority.

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

### Stage 2 — four-PC shadow, MANUAL/PARK

- physically secure the vehicle and block CAN TX/actuation;
- start logging before the bridge;
- verify one bridge and one PC4 source;
- verify map control points and ego overlay;
- measure QoS, rate, timestamp age, and latency;
- keep virtual objects out of the canonical planning topic.

### Stage 3 — candidate fusion, still no actuation

- publish to a candidate fusion/debug topic;
- verify real/virtual association and physical priority;
- test PC4 process kill, reboot, cable removal, packet delay, and message replay;
- require physical perception to continue with no stale virtual objects.

### Stage 4 — canonical fusion, no actuation

- make PC2 the sole canonical publisher;
- observe PC1 planning/stop trajectory response;
- prove availability and MRM fault routing without a physical command sink;
- verify object publisher count remains exactly one.

### Stage 5 — closed-course low-speed test

- safety driver, E-stop, spotters, and rollback ready;
- begin with one static virtual obstacle;
- cap the initial vehicle speed at 3 km/h;
- measure detection-to-plan, plan-to-command, and command-to-vehicle response;
- test only pre-approved failures with the expected controlled-stop behavior;
- expand to moving and multiple actors only after acceptance.

## 15. Minimum acceptance criteria

- [ ] Canonical predicted-object publisher count is exactly one and owned by PC2.
- [ ] PC4 publishes no vehicle command, TF, clock, localization, or vehicle-status topics.
- [ ] Physical perception continues when PC4 has never started, stalls, disconnects, or restarts.
- [ ] Stale/future/replayed/misaligned/malformed virtual objects are rejected.
- [ ] No duplicate or ghost objects remain after association within the approved test set.
- [ ] A PC4 actor is deleted within the approved TTL without reappearing from transient history.
- [ ] Map hashes, axes, yaw, height, and three control points match.
- [ ] `ego_actual` tracks the real vehicle without simulator-model drift.
- [ ] `ego_model` remains debug-only and cannot become a vehicle state authority.
- [ ] PC4-required loss inhibits engagement or produces the approved controlled stop.
- [ ] Restart returns the integration to shadow/off and requires manual re-arm.
- [ ] Rollback to physical-perception-only operation is immediate and documented.

## 16. Review questions for PC1, PC2, PC3, and PC4 owners

1. What is the exact PC4 simulator source message, frame, actor-ID model, and publish rate?
2. Can PC4 publish stable `TrackedObjects`, or must PC2 track PC4 `DetectedObjects`?
3. What TTL follows from measured latency and the approved 3 km/h stopping envelope?
4. Is hybrid mode supplemental or mandatory for the scenario, and what is the loss policy?
5. Should virtual actors affect planning only, or must AEB also consume them?
6. What exact map files/hashes and simulator-world transform define the C-track release?
7. Which host owns the domain bridge and its allowlist configuration?
8. How will PC4-required diagnostics feed PC1 availability and the validated MRM path?
9. Which logs and metrics constitute a reproducible test record?
10. Who has authority to arm hybrid mode and approve each validation stage?

Implementation must not begin on the physical command path until these questions and the
minimum contract are reviewed by the PC1, PC2, PC3, and PC4 owners.
