# PC2 post-v2 VILS integration implementation and verification record

Date: 2026-08-14 KST

Scope: PC2 Autoware source only

Status: local fail-closed prototype and isolated tests complete; distributed Stage 2/3/4 acceptance not run

## 1. Request and source of truth

This work was requested after reviewing the PC4 field-test preparation material in:

- repository: `hwanhonglee/Autonomous-Driving`;
- branch: `agent/pc4-digital-twin-integration`;
- reviewed branch tip: `ac12565e03e166138a684e5e95d03b59a3cbba50`;
- directory: `docs/vils/VILS_ACTUAL_4PC_TEST_PREPARATION`.

The request was to determine what PC2 must change and test from the PC4 evidence, preserve a
current-byte `copy_org` before editing an existing file, and continue only as far as the available
safety evidence permits.

The preparation documents define the following PC2 progression:

1. Stage 1.5 preserves the physical perception baseline. PC2 does not start a LiDAR driver. It
   consumes the PC3-owned legacy relay at
   `/sensing/lidar/top/pointcloud_before_sync`.
2. Stage 2 is receive-only raw evidence capture. It must not publish a candidate, accepted status,
   or canonical object topic.
3. Stage 3 adds validation, source identity, manual arm, TTL, physical-main association, provenance,
   and a non-canonical candidate.
4. Stage 4 may transfer the canonical tracked-object writer only after repeated Stage 3 acceptance.
   The existing map-based predictor remains the only canonical `PredictedObjects` writer.
5. Required virtual modes remain blocked until the PC1/PC3 MRM availability and command-path policy
   are implemented and reviewed.

## 2. Distributed-test boundary and later bounded field observation

The reviewed PC4 preparation state explicitly does not authorize Stage 2 or later distributed
acceptance. At the reviewed commit:

- PC4 Stage 1 real-ego replay was not passed;
- PC3 and PC4 map/transform digests were not approved;
- four-host time synchronization was not accepted;
- the approved PC4 object writer, diagnostic writer, metadata schema, TTL, join timeout, and
  association threshold were not finalized;
- the PC1 receive-only profile and the PC1/PC3 required-mode MRM path were not accepted.

The initial implementation therefore stopped at isolated fail-closed tests. On 2026-08-14 a later
bounded field observation ran PC2 `run_autoware` in `real_only` mode while PC1 and PC3 were already
active. PC2 still launched no vehicle, planning, control, API, or CAN component and did not publish
a test object. This observation was interrupted by vehicle power loss before the coordinated
ten-minute rosbag and four-host Chrony capture began, so it is not a Stage 2/3/4 or PG-VILS PASS.

## 3. Git and workspace boundary

The implementation was created on a new local branch:

```text
agent/vils-pc2-integration
```

Its immutable base is the current PC2 Autoware v2 branch tip:

```text
IONIQ_EV/PC2/autoware_universe/v2.0.0
44f79b408ccbffb4cad6f56cbee68de841ac38ac
```

The VILS implementation itself did not modify:

- `IONIQ_EV/PC2/autoware_universe/v2.0.0` itself;
- PC1 or PC3 source/configuration;
- host NetworkManager profiles and CAN configuration.

A companion ROS 2 workspace PR now carries the later, reversible Windshield transport changes:
JPEG transport plus 2x2 binning and the matching transport-only CameraInfo. The guarded runner also
seeds the known PC1/PC3/PC4 CycloneDDS peer addresses while retaining multicast.

Commit, push, and PR identities are recorded by GitHub and are not predeclared in this source file.

## 4. Exact pre-edit backups

Every existing file modified for this prototype first received a new, non-overwriting backup made
immediately before its corresponding edit. The four initial VILS-seam backups were also checked
against `git show HEAD:<path>`. Backups are archived outside package `launch/`, `config/`, and
`param/` install directories so rollback evidence is retained without creating executable or
loadable duplicate resources.

| Active file | Backup | Pre-edit SHA-256 |
|---|---|---|
| `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | `migration_work/backups/PC2/vils_runtime_20260814/autoware_launch/autoware.launch.copy_org_vils_pc2_260814.xml` | `84687435b978fce34460d42684e9b9f8284063a2018ba2704c56dc17dd78a739` |
| `launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml` | `migration_work/backups/PC2/vils_runtime_20260814/autoware_launch/tier4_perception_component.launch.copy_org_vils_pc2_260814.xml` | `55b76a70880aa949be4787f1025de2f793f89e2b69a76380b238112b2de59e72` |
| `launcher/autoware_launch/autoware_launch/package.xml` | `migration_work/backups/PC2/vils_runtime_20260814/autoware_launch/package.copy_org_vils_pc2_260814.xml` | `89a6b2fb4f16f93b54ce8123eb3732dde64a412da4c8ecb09cf25f6813d3d497` |
| `universe/autoware.universe/launch/tier4_perception_launch/launch/perception.launch.xml` | `migration_work/backups/PC2/vils_runtime_20260814/perception/perception.launch.copy_org_vils_pc2_260814.xml` | `2378cd8dca9580d193586c1047211fab18dd6d5273b90519083702f16a3a0141` |
| `migration_work/scripts/probe_yolox_contract.py` | `migration_work/backups/PC2/vils_runtime_20260814/scripts/probe_yolox_contract.copy_org_yolo_transport_260814.py` | `4c35827617988dde2b2bbe302fbe116d9f520bb789c37cae2a340f62007b355c` |
| `migration_work/scripts/run_pc2_autoware.sh` | `migration_work/backups/PC2/vils_runtime_20260814/scripts/run_pc2_autoware.copy_org_dds_peers_260814.sh` | `919ec9f06ddb0e3150a658cdb88971d388fd59d44e1a41d2f5394202c47bd6e9` |
| `migration_work/scripts/validate_pc2_cycles.sh` | `migration_work/backups/PC2/vils_runtime_20260814/scripts/validate_pc2_cycles.copy_org_binning_260814.sh` | `69f15bbc6294b1cec94ed87549f7e6859651a6e82f7afa69f8cb9b9137fca4d8` |
| `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/camera.launch.xml` | `migration_work/backups/PC2/vils_runtime_20260814/sensing/camera.launch.copy_org_yolo_transport_260814.xml` | `6c738832ea9fb0862b149574439587635ac46c5e27f07934df12692b574e78da` |
| `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` before compressed transport | `migration_work/backups/PC2/vils_runtime_20260814/autoware_launch/autoware.launch.copy_org_yolo_transport_260814.xml` | `5bab8965f750d29a6424b728ea22a238a52e5aedd3da752d93519d6b99bdb310` |

Older `copy_org` files were not reused because they are not guaranteed to match the current v2
bytes.

## 5. Existing launch files changed

### 5.1 Top PC2 launch

`launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` now exposes:

- `vils_object_mode`, default `real_only`;
- `vils_contract_approved`, default `false`;
- `vils_enable_canonical_selection`, default `false`;
- `vils_enable_required_modes`, default `false`;
- the reviewed parameter-file path;
- an explicit append-only provenance-log path.

The default parameter path is the actually installed file:

```text
$(find-pkg-share autoware_vils_object_integration)/config/vils_object_integration.param.yaml
```

The default `real_only` topology is unchanged:

```text
physical tracker
  -> /perception/object_recognition/tracking/objects
  -> existing map-based predictor
  -> /perception/object_recognition/objects
```

The dormant Stage 4 seam is:

```text
physical tracker
  -> /perception/object_recognition/tracking/pc2_physical_objects
  -> PC2 VILS validator/fuser
  -> /perception/object_recognition/tracking/objects
  -> existing map-based predictor
```

That seam is represented only as dormant Stage 4 wiring. The integration launch and executable
currently reject every canonical mode before node creation because Stage 3 evidence and READY/MRM
supervision do not exist. Required modes are likewise unavailable.

### 5.2 Perception launch plumbing

The two existing perception wrappers now expose and forward default-preserving seams:

- `tracking/output/objects`;
- `prediction/input/objects`.

The lower tracker and predictor launch files were not modified. Their original canonical defaults
remain in effect unless the upper PC2 launch deliberately selects the Stage 4 seam.

### 5.3 Package dependency

`autoware_launch/package.xml` now declares the new integration package as an execution dependency.

## 6. New typed interface package

The new `core/autoware_msgs/vils_interfaces` package contains:

- `AcceptedPc4Status.msg`;
- `ArmPc4Source.srv`;
- ROSIDL build and package metadata.

The status reports the reviewed object and diagnostic writer identities, session, sequence, source
stamp, TTL, map/transform/serialization contracts, object count, state transition, and acceptance/
rejection counters.

The arm request binds all of the following in one explicit operator decision:

- source session;
- object writer GID;
- diagnostic writer GID;
- map digest;
- transform digest.

## 7. New Stage 2 receive-only raw recorder

`migration_work/scripts/record_pc2_vils_stage2_raw.py` implements the Stage 2 boundary separately
from the validator. In steady state it has no publisher, service, action, client, or parameter
mutation. Its only retained ROS entities are two Reliable/Volatile/KeepLast(1) subscriptions:

- `/perception/pc4/virtual_obstacles/tracked_objects`;
- `/diagnostics/pc4/object_adapter`.

Humble rclpy constructs an implicit `/parameter_events` publisher inside `Node.__init__`; the
recorder removes it immediately after construction and before its first graph snapshot. Therefore
this is a measured steady-state no-output contract, not a claim that rclpy never instantiated an
internal publisher at any point in process initialization.

The recorder writes a new owner-private run directory and refuses to reuse an existing output path.
It records:

- exact SHA-256 of the serialized byte sequence delivered to each rclpy raw callback;
- the same callback byte sequences in a `rosbag2` sqlite3 stream under `raw_cdr`, with CDR topic
  metadata and a strictly increasing, wall-time-derived bag index;
- receive wall and monotonic time plus global and per-source receive order;
- typed frame, source stamp, object count, and explicit empty-snapshot status;
- bounded diagnostic keys, session, sequence, source hash/stamp/count, map, transform, and named
  serialization fields;
- start, periodic, and end graph snapshots with input publishers/subscribers, recorder entities,
  protected candidate/accepted/canonical outputs, FQN, endpoint GID, type, QoS, and cardinality;
- middleware loss notifications and separately labelled diagnostic-sequence gap inference;
- thresholded no-message intervals, which are never converted into empty actor arrays;
- script/config hashes, environment, start/stop reason, counts, gaps, and clean-shutdown summary.

The run directory and `raw_cdr` directory are mode `0700`; `manifest.json`, `events.jsonl`, and
`summary.json` are created with `O_EXCL`/`O_NOFOLLOW` and mode `0600`. After the rosbag writer is
closed, every sqlite3/metadata file is checked as an owner-owned, non-symlink regular file, changed
to mode `0600`, hashed, and synchronized. `events.jsonl` uses `O_APPEND`; after all streams close,
`checksums.json` covers the manifest, events, summary, and every bag file. A run is complete only
when the final owner-only `COMPLETE.json` marker exists and its checksum-of-checksums matches.
Here, complete means only that the local artifact set was closed and sealed. It is not a Stage 2
PASS or PC4-source acceptance marker: a zero-message or zero-publisher observation can still be
sealed and must be rejected later by the coordinated acceptance analysis.

Every graph snapshot must show exactly one raw subscriber (the recorder), zero candidate and
accepted-status publishers, no VILS integration node, no VILS-owned canonical tracked publisher,
and no steady-state recorder publisher/service/client. A violation is recorded, terminates the run
with a nonzero result, and prevents creation of `COMPLETE.json`.

Humble rclpy does not expose the callback `MessageInfo` GID through its Python executor. Therefore
each raw event explicitly records `callback_gid_available: false` and
`callback_publication_gid: null`; graph endpoint GIDs remain periodic observations and are not
misrepresented as per-message attribution.

The companion local-only test is:

```text
migration_work/scripts/test_record_pc2_vils_stage2_raw.py
```

It uses only isolated test data topics and checks nonempty and empty object arrays, diagnostics,
sequence-gap evidence, graph identity/QoS, no recorder publishers/services/clients, no overwrite,
private permissions, duration completion, SIGINT sealing, and rejection of non-finite timing inputs
before output creation. It also reopens the sealed rosbag, requires four stored messages, and matches
every bag-payload hash, timestamp, and order against its JSONL callback event. A separate negative
case creates a candidate publisher and a second raw subscriber and requires the recorder to stop
without a `COMPLETE.json` marker.

The raw receive component is ready for a future Stage 2 capture, but it is not by itself the complete
four-PC Stage 2 evidence package. The common run manifest still needs approved scenario/session,
map/transform, four-host time, NIC/DDS, and cross-host source reconciliation records. No live PC4
Stage 2 evidence was recorded in this work because the remote prerequisites remain blocked.

## 8. New PC2 VILS integration package

The new package is:

```text
universe/autoware.universe/perception/autoware_vils_object_integration
```

It provides:

- exact source-mode parsing;
- a fail-closed installed YAML profile;
- one C++ validation/fusion node;
- typed accepted status and manual-arm service;
- pure contract/fusion GTests;
- an isolated DDS launch test;
- an operator README.

### 8.1 Source modes

| Mode | Current behavior |
|---|---|
| `real_only` | Launch creates no VILS process. Existing physical path remains unchanged. |
| `shadow` | Candidate/debug path only. An unapproved contract accepts nothing. |
| `hybrid_optional` | Dormant seam implemented, but intentionally rejected until Stage 3 and READY/MRM review. |
| `hybrid_required` | Intentionally rejected until the PC1/PC3 MRM availability path exists. |
| `virtual_only_required` | Intentionally rejected until the PC1/PC3 MRM availability path exists. |

### 8.2 Fail-closed installed profile

The installed YAML contains:

- `contract_approved: false`;
- `__UNAPPROVED__` identities and digests;
- zero timing, content, covariance, pending-capacity, and association bounds;
- `allow_empty_snapshot: false`;
- safe-state arming enabled;
- no evidence path.

Therefore copying only a Boolean gate cannot create an accepted deployment profile.

### 8.3 Endpoint and identity checks

For both PC4 object and diagnostic topics the node requires:

- exactly one graph publisher;
- the approved node FQN;
- the exact ROS type;
- Reliable, Volatile, KeepLast, depth 1 QoS;
- the current graph endpoint GID;
- a separate object-writer and diagnostic-writer identity;
- manual re-arm after a writer or session transition.

The isolated CycloneDDS test showed that the callback `MessageInfo` publication GID and graph
endpoint GID are not interchangeable on this Humble host. The implementation therefore treats the
single approved graph endpoint as the current identity and does not claim callback-GID equivalence.

### 8.4 Metadata, timing, and payload validation

The current prototype validates:

- exact object/diagnostic digest join in either arrival order;
- strict decimal parsing for sequence, source stamp, and object count;
- session syntax, session reuse, duplicate/replay/out-of-order sequence, and writer transition;
- source-ready and full-snapshot flags;
- frame, source stamp field range, age, future skew, and monotonicity;
- map, transform, and named serialization contract;
- finite position, orientation, twist, acceleration, covariance, probability, shape, and footprint;
- configured value bounds, object-count limits, classification validity, nonzero unique UUIDs, and
  bounded pending joins;
- explicit scenario approval before a fresh empty actor snapshot is accepted.

The current digest is a bounded receiver-side ROS reserialization digest. It is not represented as
proof of the exact original wire CDR bytes. That wire-level contract remains an owner-review item.

### 8.5 Manual arm and loss behavior

Manual arm additionally requires:

- a recently reviewed source identity;
- both current graph endpoints still present with the reviewed GIDs;
- exact request session/object-GID/diagnostic-GID/map/transform values;
- fresh finite vehicle reports;
- MANUAL control mode;
- PARK gear;
- stationary longitudinal and lateral velocity.
- an owner-reviewed stationary yaw-rate bound.

TTL, join timeout, endpoint change, session change, replay, contract mismatch, evidence-write fault,
and content failure clear the virtual cache and disarm the source. A fresh explicit empty snapshot is
different from an absent or stale source.

### 8.6 Physical-main fusion behavior

The physical tracked-object array is always copied first. A virtual actor is suppressed when:

- its UUID collides with a physical UUID; or
- it meets the approved planar association threshold and, when configured, the primary class rule.

Only unmatched virtual actors are appended. The physical UUID, pose, velocity, class, shape, and
other physical fields are retained. The result is rejected if it exceeds the configured total object
limit.

This is an object-level planning/control test seam. It bypasses camera/LiDAR detection and therefore
does not validate camera, YOLOX, TLR, calibration, sensor fusion, or general perception accuracy.

### 8.7 Evidence handling

The provenance destination is opened with no symlink following, checked as a regular file, forced to
mode `0600`, written with partial-write/EINTR handling, and synchronized. An approved contract cannot
publish selected output after an evidence write failure.

This synchronous evidence implementation remains a prototype limitation because filesystem `fsync`
and graph queries occur in callback paths. Its latency must be measured before any real-time use.

## 9. Local verification completed

All ROS work was isolated with `ROS_LOCALHOST_ONLY=1` and a non-vehicle ROS domain. No external PC or
vehicle topic was changed.

### 9.1 Static checks

- `git diff --check`: PASS;
- XML/package parsing: PASS;
- Python AST and YAML parsing: PASS;
- `ament_flake8`: two Python files checked, PASS;
- `ament_uncrustify`: new C++ files formatted, final PASS;
- top Autoware `--show-args`: PASS and correct installed VILS YAML path shown.

The Stage 2 recorder and its standalone test were also checked with `ament_flake8`: two files,
zero problems.

### 9.2 Build

```bash
colcon build --symlink-install \
  --packages-select vils_interfaces autoware_vils_object_integration
```

Result: both packages built successfully.

The two existing launch packages changed by the seam were then rebuilt separately:

```bash
colcon build --symlink-install --packages-select \
  autoware_launch tier4_perception_launch
```

Result: both launch packages built and installed successfully. The current-byte `copy_org`
resources were moved to the non-installed `migration_work/backups/PC2/vils_runtime_20260814/`
archive before publication.

### 9.3 Package tests

```bash
ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 \
colcon test --packages-select \
  vils_interfaces autoware_vils_object_integration
```

Results:

- `vils_interfaces`: 3/3 CTest targets passed;
- `autoware_vils_object_integration`: 6/6 CTest targets passed;
- contract/fusion GTest: 10/10 passed;
- DDS launch test: test body and clean-process-exit checks passed;
- `colcon test-result` for the two package build directories: 0 errors, 0 failures;
- `autoware_launch` and `tier4_perception_launch`: 3/3 CTest targets passed for each package, and
  each package's expanded xUnit result reported 35 tests with 0 errors and 0 failures;
- aggregate current Autoware build-tree result after the rerun: 210 tests, 0 errors, 0 failures,
  46 skipped;
- four cppcheck records were skipped because the installed cppcheck version is disabled by the
  ROS lint wrapper; they are not represented as executed static-analysis passes.

The isolated DDS test exercises:

- metadata-first and object-first join;
- exact current object/diagnostic graph identities;
- explicit manual arm;
- physical-priority overlap suppression;
- one unmatched virtual actor append;
- TTL physical-only fallback;
- an explicitly allowed fresh empty snapshot;
- replay rejection and disarm;
- clean SIGINT and provenance events.

The separate Stage 2 recorder synthetic test was rerun as:

```bash
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
python3 migration_work/scripts/test_record_pc2_vils_stage2_raw.py
```

It ran four cases in the isolated local domain:

- bounded recording with nonempty and explicit empty objects, two diagnostic samples, graph/QoS
  evidence, sequence-gap inference, source-absence evidence, private file modes, and overwrite
  refusal; the four callback payloads were written to rosbag2/sqlite3, reopened, counted, and matched
  by SHA-256 to the append-only JSONL events;
- unbounded recorder mode followed by SIGINT and clean sealed summary;
- non-finite duration/graph/absence timing arguments rejected before creating output;
- candidate-output and second-raw-subscriber conflicts detected, with a non-clean summary and no
  completion marker.

Final result: 4/4 tests passed. An initial test exposed that Humble presents
`DiagnosticStatus.level` as a one-byte octet; the recorder now handles both the octet and integer
representations, and the test was rerun successfully.

### 9.4 Launch gates

- integration `--show-args`: PASS;
- `mode:=real_only`: returns successfully and creates no VILS node;
- invalid mode: rejected before process creation;
- canonical optional and required modes with all Boolean gates: still rejected before process creation;
- rejected canonical launch created no provenance file and left no VILS process.

### 9.5 Bounded four-host field observation and power interruption

The later `real_only` field session provided the following bounded PC2-side observations before
power loss:

- explicit CycloneDDS peers restored the PC1 and PC3 endpoints that multicast-only discovery had
  intermittently missed;
- one persistent ten-second reader measured PC1 vehicle status at 50.099 Hz, trajectory at
  10.100 Hz, and control command at 32.899 Hz;
- it measured PC3 localization at 49.299 Hz, but the legacy LiDAR relay only at 0.500 Hz, which is
  degraded and not accepted;
- the PC2 canonical `PredictedObjects` topic was observed at 9.000 Hz;
- after 2x2 Windshield binning and JPEG transport, the bounded YOLOX contract probe passed with
  10/10 samples at 6.574 Hz and zero invalid ROIs;
- TLR judged and final topics transported fresh empty arrays at approximately 14.7 Hz; this proves
  transport/lifecycle only, not semantic signal recognition;
- PC4 remained ICMP reachable in the preceding check, but the expected PC4 object and diagnostic
  publishers were both zero.

The final launch log then recorded dynamic TF ceasing to advance while later LiDAR stamps
continued, followed by an abrupt log end without the normal PC2 shutdown sequence. No coordinated
rosbag, common four-host manifest, same-window Chrony set, or clean completion marker exists. The
small logs and measurement summary are published separately under the PC4 integration branch's
PC2 result directory and are explicitly labelled partial evidence.

## 10. What is implemented versus what is accepted

| Item | Status |
|---|---|
| Default real-only topology | Implemented and locally checked |
| Dormant tracker/predictor Stage 4 seam | Implemented and statically checked |
| Typed status/arm interface | Implemented and built |
| Stage 2 receive-only raw recorder | Implemented and synthetic 4/4 PASS; no live PC4 run |
| Fail-closed validator and candidate fuser | Implemented as Stage 3 prototype |
| Local contract/fusion tests | Passed |
| Isolated DDS validation/arm/fusion/TTL test | Passed |
| Stage 2 PC4 live receive evidence | BLOCKED; expected PC4 publishers were zero |
| Stage 3 four-PC shadow test | BLOCKED, not run |
| Canonical Stage 4 vehicle test | NOT AUTHORIZED, not run |
| Required virtual modes | Intentionally unavailable |
| Exact original network wire-CDR proof | Not implemented; callback CDR is preserved |
| Camera/YOLOX/TLR validation through object injection | Out of scope; injection bypasses perception |

## 11. Known gaps before Stage 3 review

The following remain open and must not be hidden by the local PASS:

1. The PC4 diagnostic schema and named serialization/hash contract need owner approval.
2. The Stage 3 validator's receiver-side reserialization is not the original network wire CDR.
   Stage 2 preserves the exact bytes delivered to the rclpy callback, but does not claim packet-level
   identity before middleware handling.
3. The final map/transform digests, PC4 object/diagnostic FQNs, TTL, join timeout, clock bounds,
   payload bounds, and association threshold are unapproved.
4. The provenance schema does not yet contain every evidence-matrix field, including complete
   physical/virtual/matched/unmatched identity lists.
5. Source failure disarms and removes virtual data, but the immutable configured mode name remains
   visible as the requested mode. Owner review must decide whether the status requires a separate
   effective `shadow` state before Stage 4.
6. Required-mode MRM availability is absent and both required modes remain hard blocked.
7. Bounded PC1/PC3 topic rates were observed, but no accepted four-host time, map/transform,
   LiDAR-continuity, PC1 receive-only, or CAN-TX invariant run was completed.
8. No PC4 live source publisher was present, so no real PC4 writer GID, session, rate, delay, jitter, drop, or
   reconnect result exists.
9. Synchronous provenance `fsync`, graph queries, and reliable publication need latency/load tests.
10. Object-level injection can test downstream scenario response only; it cannot establish sensor
    perception or overall ADS safety.
11. The Stage 2 recorder component manifest does not replace the shared four-PC run manifest; PC2
    Git/dirty state, DDS/NIC hashes, scenario/session, PC3/PC4 map/transform, and time evidence must
    be captured by the coordinated test procedure.
12. The recorder has not yet passed 1x/2x/4x burst, disk-full/fsync fault injection, source-writer
    restart, second-writer, malformed-CDR, or long-duration drop-accounting tests.
13. Stage 2 acceptance still needs an external minimum observation window and discovery-convergence
    rule plus approved input publisher FQN/GID/type/QoS/cardinality, nonzero continuity/rate/gap
    checks, and PC4 transmit-to-PC2 receive reconciliation. `COMPLETE.json` proves none of those.

## 12. Required next test sequence

### Stage 1 and Stage 1.5 prerequisites

- complete PC4 real-ego replay;
- approve PC3/PC4 map and transform manifests;
- measure four-host clock offset and stability;
- prove PC3 canonical LiDAR to legacy relay continuity and PC2 physical perception continuity;
- use a reviewed PC1 receive-only procedure, not the actuator-capable historical bridge command;
- prove MANUAL/PARK/stationary and zero CAN-TX delta.

### Stage 2 receive-only

- run only the dedicated raw recorder;
- record both raw object and diagnostic samples, graph endpoints, type/QoS, timestamps, rate, gaps,
  empty snapshots, source absence, and map/transform/session metadata;
- keep validator, accepted-status, candidate, and canonical VILS publishers at zero;
- reconcile receiver evidence with the PC4 source manifest without accepting a source.

### Stage 3 shadow

- approve a measured YAML contract and provenance destination;
- leave the physical tracker/predictor canonical path unchanged;
- run validator and candidate output only;
- exercise writer restart, session rollover, replay, join timeout, stale/future data, malformed data,
  map/transform mismatch, clock offset, fresh empty, source absence, TTL, overlap, and PC4 loss;
- complete at least three independent stationary runs before considering Stage 4.

### Stage 4 canonical optional

- verify tracker output is private, the VILS node is the sole canonical tracked writer, and the
  existing predictor is the sole canonical predicted-object writer;
- verify fail-open physical-only behavior on every optional-source fault;
- retain PC1/PC3 safety ownership and zero new PC2 control/vehicle/CAN writers;
- begin with stationary MANUAL/PARK evidence before any separately approved closed-course movement.

## 13. Rollback

The safest rollback is to run with:

```text
vils_object_mode:=real_only
vils_contract_approved:=false
vils_enable_canonical_selection:=false
vils_enable_required_modes:=false
```

This creates no VILS process and leaves the existing physical tracker-to-predictor topology intact.
For a source rollback, restore the active files from the exact
`migration_work/backups/PC2/vils_runtime_20260814/` archive and remove the two new packages after
preserving any review evidence.
