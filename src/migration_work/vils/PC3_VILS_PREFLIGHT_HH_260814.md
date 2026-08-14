<!-- HH_260814 - Record the PC3 implementation boundary, four-PC handoff, and preflight evidence for staged VILS testing. -->
# PC3 VILS preflight and cross-PC handoff

Status: **POST-v2 PROTOTYPE; PREPARATION ONLY; NO FOUR-PC VILS PASS**

This document records the PC3 decision made after reviewing the preparation set at
`agent/pc4-digital-twin-integration` commit
`ac12565e03e166138a684e5e95d03b59a3cbba50`. The same-host PC4 surrogate result
does not prove the distributed vehicle path. In particular, the real-state
`ego_actual` replay was not run, so Stage 1 is not accepted and Stage 2 must not
start from this document alone.

## Decision

PC3 does not subscribe to PC4 objects and does not merge, predict, or publish the
canonical perception result. PC3 remains the authority for:

- the physical Hesai LiDAR and its canonical plus legacy-relay point clouds;
- NovAtel/u-blox GNSS, INS orientation, and RTCM ingress;
- the deployed Lanelet2, PCD, and projector bundle;
- localization, including the single `map -> base_link` dynamic TF edge; and
- its reviewed MRM/system graph only after a live one-writer ownership audit.

The only implementation added in this candidate is read-only evidence tooling and
an opt-in DDS environment. It creates no control, engage, CAN, TF, localization,
or perception publisher.

## PC1 handoff

PC1 must not subscribe directly to PC4. It continues to consume the one canonical
`/perception/object_recognition/objects` publisher owned by PC2. Before Stage 2,
PC1 must provide a separately named receive-only status profile for the seven
approved vehicle-state streams. Through Stages 1.5 to 4 it must prove:

- MANUAL and PARK before every topology or fault-injection change;
- no socket-CAN sender, vehicle command adapter, engage request, `/to_can_bus`
  endpoint, or CAN TX delta;
- no historical bidirectional bridge profile; and
- fresh velocity, steering, control-mode, gear, turn-indicator, hazard-light, and
  operation-mode state with recorded type, QoS, rate, timestamp, and source GID.

Moving Stage 5 remains blocked until PC1 has independent command-freshness and
request-freshness watchdogs, request-bit clearing, an exclusive CAN writer and CAN
ID allow-list, accepted vehicle geometry, and the 0.833 m/s limit. None of those
requirements is bypassed by PC4 object availability.

## PC2 handoff

PC2 owns all PC4 object validation and fusion. It must retain the PC3 legacy LiDAR
relay as the current physical-perception baseline and must not launch another
physical LiDAR driver. The staged PC2 work is:

1. Stage 2: record the PC4 raw object stream without changing canonical output.
2. Stage 3: validate run ID, session/sequence, source timestamp, frame, map hash,
   finite values, dimensions, velocity, classification, geofence, TTL, and reset
   behavior; join bounded metadata to CDR; publish only a debug candidate.
3. Stage 4: use physical perception as the main trigger, merge only fresh accepted
   PC4 objects, preserve physical-only fail-open behavior where the approved mode
   allows it, and keep exactly one predictor and one canonical publisher.
4. Publish the accepted-PC4 status only after its exact package, message type, QoS,
   timeout, mode semantics, and owner are frozen and tested.

PC3 intentionally does not implement a consumer for the proposed
`/perception/pc2/vils/accepted_pc4_status` yet. Adding a second
`/system/fail_safe/mrm_state` writer is forbidden.

## PC4/gateway handoff

The gateway must first open only the reviewed vehicle-to-PC4 state allow-list.
Aggregate `/tf`, `/tf_static`, `/clock`, map services, control, vehicle command,
planning, localization authority, system authority, parameters, services, and
actions must not cross from PC4 into the vehicle domain. Only after the live ego
overlay and endpoint closure pass may the gateway open the reviewed PC4 object and
diagnostic egress.

PC4 must not copy the PC3 GNSS subtraction. It must use one owner-approved rigid
simulator-world-to-Autoware-map transform and the exact deployed map manifest.

## Deployed PC3 map convention

The first VILS trial must retain the current legacy convention atomically:

| File | SHA-256 |
|---|---|
| `lanelet2_map.osm` | `ad3e8673ac336432676e56a93fef1617ed3bc2698fb9f7d97943384b8da5aee6` |
| `pointcloud_map.pcd` | `29d958e74c37da80f345285f9f7ef9f49bc35ce41ffb44a7c82ffe3a70825153` |
| `map_projector_info.yaml` | `beaefae362a9001f6dd7ac72cea36683367725fbdc31cd555c35dc0fd961b99d` |
| `map_config.yaml` | `54dac57de0e662964a84f5cebefcb44625cb08d50423366e81af206bfea0f4e4` |

The active projector is the synthetic 52SCF0 bundle and the current shared GNSS
pose correction is enabled with:

- X subtraction: `60966.4679793288 m`
- Y subtraction: `65973.64540576655 m`
- Z subtraction: `15.816125382 m`

`map_config.yaml` is retained for bundle provenance but is not a current
top-level map-loader input. The evidence collector also hashes all four actual
map-loader parameter files and their installed artifacts.

The native 52SCF60 map is a separate future migration: Lanelet2, PCD, projector,
and related config must move as one atomic bundle while the legacy GNSS correction
is disabled. Switching only the projector or applying both conventions is
forbidden.

## PC3 implementation in this candidate

All files below are new, so no existing file was changed and no `copy_org` was
needed:

- `pc3_vils_contract.yaml`: machine-readable authority, map, topic, time, and
  blocked-proposal contract.
- `config/cyclonedds_vehicle_domain.xml`: pins Domain 10 discovery/data to
  `192.168.9.7`.
- `activate_pc3_vils_environment.sh`: opt-in environment; normal shell defaults
  remain unchanged.
- `collect_pc3_vils_evidence.py`: owner-only, non-overwriting map/source/install,
  process, Chrony, network, and ROS endpoint collector.
- `pc3_vils_bag_topics.txt`: reviewed state/sensing/map/GNSS/diagnostic/TF/MRM
  subscription allow-list with no command or CAN topic.
- `record_pc3_vils_bag.sh`: bounded compressed recording with clean SIGINT close.
- `test/test_collect_pc3_vils_evidence.py`: hardware-free safety and provenance
  regressions.

If a later change touches any existing file, create a non-overwriting same-directory
`(copy_org_HH_260814)` copy before editing. Never overwrite an older `copy_org`.

## Verified tool behavior

Offline validation completed on 2026-08-14 KST:

- Python compilation: PASS.
- evidence collector unit tests: 6/6 PASS.
- Bash syntax: PASS.
- CycloneDDS XML parse: PASS.
- contract YAML parse: PASS.
- `git diff --check`: PASS.
- sourced DDS profile: PASS on active `192.168.9.7/24`.
- ROS 2 CycloneDDS no-daemon discovery: PASS; zero nodes were present.

Read-only smoke run:

`/home/a/pc3_vils_runs/20260814T022052Z-PC3_TOOL_SMOKE_HH_260814_002`

It proved that the collector records the exact Autoware baseline
`947dda782ce90e1d9768e57ae4337e3cf78eee1b`, ros2_ws baseline
`22f521ffb7ddeae893f377e226091641bb540efc`, all four active map hashes, all three
GNSS offsets, nine installed Autoware artifacts, Chrony and network commands, and
zero map drift. Autoware was off, so it also correctly recorded zero relevant
processes, zero ROS nodes, and all required runtime topics as absent. No ROS daemon
was left running. This is a tooling smoke result, not Stage 1.5 evidence.

## Stage 1.5 acceptance still required after power-on

With PC4 and its gateway absent, collect a common run ID across all hosts and prove
at least ten continuous minutes of:

- fresh `/localization/kinematic_state` and twist-derived
  `/localization/acceleration`;
- accepted NDT and EKF output with one stable `map -> base_link` edge writer;
- canonical and relay LiDAR continuity and exactly one physical LiDAR owner;
- source and installed hashes matching the run manifest;
- the active map hashes and GNSS offset staying unchanged;
- one MRM-state writer and NORMAL/NONE baseline under the approved owner policy;
- all four hosts on the accepted Chrony source with recorded reach, offset, jitter,
  and no clock step; and
- MANUAL/PARK, no command/CAN endpoints, and independently observed zero CAN TX.

`/localization/acceleration` is derived from twist; it is not a direct physical
accelerometer measurement.

## Open owner decisions

The preparation documents do not yet provide deterministic numerical acceptance
limits for Chrony offset/jitter, control-point residual, ego overlay residual, TF
jump, maximum stream gap, or PC4 TTL. The four owners must freeze those values
before claiming PASS. They must also resolve the sole MRM owner and the typed
accepted-PC4 status policy before required-mode testing. Until then, shadow and
optional parked testing may collect evidence, but moving or actuating tests remain
blocked.
