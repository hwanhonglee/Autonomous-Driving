<!-- HH_260810 - Defined one evidence and decision contract for all four hosts. -->
# Evidence and Go/No-Go contract

Status: **MANDATORY RECORD FOR EVERY RUN**

<!-- HH_260810 - Required one immutable manifest before any process starts. -->
## Run manifest

Every host must use the same run ID and record:

- UTC and KST start time;
- host name and owner;
- exact source branch and 40-character commit SHA;
- installed package prefix and relevant binary/config hashes;
- map, projector, simulator-transform, DDS, and gateway SHA-256 values;
- `ROS_DOMAIN_ID`, RMW implementation, CycloneDDS URI/hash, interface, IP, route, and MTU;
- scenario name, PC4 session ID, CARLA actor IDs, and requested stage;
- explicit statement that CAN, engage, and actuation are disabled.

<!-- HH_260810 - Established stable run identifiers for cross-host correlation. -->
Recommended run IDs:

```text
BASELINE_3PC_R01
TRANSPORT_LIVE_R01
TRANSPORT_PC4_STOP_R01
TRANSPORT_PC4_RESTART_R01
OPTIONAL_TRANSPORT_ARCHIVE_R01
```

<!-- HH_260810 - Kept archived transport behind a source-specific reviewed contract. -->
Use `OPTIONAL_TRANSPORT_ARCHIVE_R01` only with a separately reviewed replay diagnostic profile and
runbook; never substitute archived input or replay diagnostics into the live Stage 2A profile.

<!-- HH_260810 - Required endpoint evidence instead of relying on topic names alone. -->
## ROS graph and payload evidence

Capture before, during, and after each run:

- node and topic inventories;
- topic type and endpoint details;
- publisher/subscriber node identity, namespace, GID, QoS, and count;
- header stamp, receive wall time, monotonic time, sequence/session, object count and UUIDs;
- message size and CDR or stable payload hash where supported;
- rate, inter-arrival distribution, missing runs, duplicates, reorder, and stale age;
- exact canonical `TrackedObjects` and `PredictedObjects` writer cardinality;
- PC4 gateway route hash and forbidden-route count.

<!-- HH_260810 - Required physical sensing lineage from PC3 through PC2. -->
## PC3-to-PC2 LiDAR evidence

Record the same time window for:

```text
PC3 /sensing/lidar/concatenated/pointcloud
  -> PC3 pc3_lidar_legacy_relay
  -> /sensing/lidar/top/pointcloud_before_sync
  -> PC2 perception subscriber
  -> PC2 detection/tracking/prediction outputs
```

PASS requires one expected PC3 source writer, one expected relay subscription and legacy writer,
one PC2 subscriber, continuous fresh payloads, compatible QoS, expected frame, and no unexplained
loss. PC2 LiDAR hardware and driver count must be zero. Git configuration does not replace this
runtime proof.

<!-- HH_260810 - Required independent localization map and transform evidence. -->
## Map and localization evidence

- active PC3 Lanelet2, PCD, projector, and GNSS-offset configuration hashes;
- PC4 candidate map and transform hashes;
- current `map -> base_link` writer node/GID and rate;
- odometry/acceleration continuity, NDT score/skip/reinit, pose jumps;
- three or more reviewed non-collinear control points and residuals before aligned-shadow claims;
- `ego_actual` versus real odometry overlay time series before aligned-shadow claims.

<!-- HH_260810 - Prevented unsynchronized clocks from becoming false latency evidence. -->
## Time evidence

On all four hosts, capture `chronyc tracking` and `chronyc sources -v` before, during, and after the
window. Record selected source, stratum, reach, offset, RMS offset, jitter, and any clock step.
Without one accepted source and bounded offset/jitter, report continuity and receive-time
inter-arrival only; do not report cross-host one-way latency.

<!-- HH_260810 - Required wire-level evidence for the actual LAN transport rehearsal. -->
## Network evidence

- `ip addr`, `ip route`, `ip link`, NIC/MTU/offload, firewall, and DDS interface binding;
- source and destination endpoint GIDs and UDP socket inventory;
- packet captures on both relevant hosts when approved;
- NIC packet/drop counters before and after;
- proof that PC4 forbidden routes and PC4-owned forbidden Domain 10 writers are zero.

Do not require the full private PC4 Autoware graph to contain zero `/tf`, localization, planning,
or control publishers; those may exist internally. The prohibition applies to gateway routes and
PC4-owned writers that cross into vehicle Domain 10.

<!-- HH_260810 - Required positive proof that no physical command path existed. -->
## No-actuation evidence

Record vehicle MANUAL/PARK/stationary state independently, plus:

- `run_bridge`, SocketCAN sender, command adapter, and CAN-writer process count;
- `/to_can_bus` publisher/subscriber endpoints;
- CAN interface and TX counters before/during/after;
- engage, autonomous-mode, control-mode, service, and action call log;
- scoped process file descriptors for CAN/serial devices.

PASS requires zero actuation-capable process, zero `/to_can_bus` writer, zero CAN TX delta, and zero
engage/autonomous call. Topic silence alone is insufficient.

<!-- HH_260810 - Defined the three-PC baseline gate that must pass before PC4 is connected. -->
## Stage 1.5 Go criteria

- approved receive-only PC1 vehicle-status source is present without a command/CAN sink;
- PC3 LiDAR, relay, map, localization, TF, and current system diagnostic owners are healthy;
- PC2 detection, tracking, occupancy, and prediction remain fresh;
- canonical final `PredictedObjects` publisher count is exactly one;
- PC1 has a valid route and stable nontrivial trajectory baseline;
- no duplicate critical writer, localization jump/reinit, perception dropout, or CAN activity;
- at least ten continuous minutes are recorded on all three hosts.

Any failure means **NO-GO for connecting PC4**.

<!-- HH_260810 - Defined the narrower raw-shadow transport rehearsal gate. -->
## Stage 2A transport-rehearsal Go criteria

- Stage 1.5 passed in the same configuration;
- PC4 source topic and diagnostics have exactly one approved writer each;
- one reviewed, deny-by-default gateway exposes only exact approved routes;
- one vehicle-domain logger consumes the PC4 object stream; PC2 canonical and PC1 Planning do not;
- PC4 process kill/restart does not change PC2 canonical output or PC1 trajectory ownership;
- missing, duplicate, reorder, mutation, and restart-session behavior can be calculated;
- no forbidden Domain 10 writer, CAN path, or actuation call appears.

This is not aligned-shadow acceptance while map/time/`ego_actual` gates remain open.

<!-- HH_260810 - Defined immediate stop conditions for every host. -->
## Immediate No-Go and stop conditions

- duplicate canonical object, trajectory, control, localization, or MRM writer;
- stale PC4 object persists after gateway/source stop;
- PC3 localization jump, reinitialization, or loss of `map -> base_link`;
- PC2 perception process dropout or loss of physical output;
- unexpected PC4 route or PC4-owned forbidden Domain 10 endpoint;
- `/to_can_bus`, CAN writer, CAN TX increment, engage, or autonomous call;
- different map hashes while an aligned claim is attempted;
- different clock sources while one-way latency is being calculated;
- unknown process, route, or writer ownership.

<!-- HH_260810 - Preserved independent recorders until cleanup evidence is complete. -->
## Evidence finalization and rollback

1. Keep independent recorders running while application processes are shut down.
2. For Stage 2A, stop the PC4-to-vehicle gateway first, then verify vehicle-domain PC4 endpoints
   are zero.
3. Stop the PC4 adapter/source and remove only session-owned actors.
4. Verify PC2 physical canonical output and PC1 baseline remain unchanged.
5. Stop PC1, then PC2, then PC3 according to the approved host procedures.
6. Capture final process, node, endpoint, port, actor, CAN, and lock residue.
7. Stop recorders last, hash every artifact, and write an immutable manifest.

<!-- HH_260810 - Bounded every publication claim to the evidence that tomorrow can actually collect. -->
## Permitted conclusions

If successful, tomorrow may establish three-PC baseline stability, actual PC3-to-PC2 physical
pointcloud continuity, PC4 namespaced-object LAN continuity, application-level MDR/inter-arrival,
restart behavior, and lack of influence on the canonical path.

It cannot establish PC4-driven Planning response, PC2 physical/virtual fusion, required-source
loss handling, AEB coverage, vehicle actuation response, cross-PC one-way latency without accepted
time, or moving VILS safety.
