<!-- HH_260814 - Documented the parked PC1 receive-only preparation profile and its limits. -->
# PC1 VILS vehicle-status RX-only profile

This post-v2 prototype supports only parked, MANUAL, no-actuation VILS Stages 1.5–4.
It does not authorize an engage request, autonomous mode, CAN transmission, or Stage 5 motion.

## Why this profile exists

The historical `run_bridge` starts the SocketCAN receiver, SocketCAN sender, and
`twistController2VCU2EPS2ACC_node`. It is therefore a full physical RX/TX control path and must not
be used to collect status during a no-actuation VILS stage.

`vehicle_status_rx_only.launch.xml` includes only `socket_can_receiver.launch.py`. It is an internal
composition file; invoking it directly bypasses the guards and is not an approved test entry point.
The installed `run_vehicle_status_rx_only.sh` runner adds fail-closed preflight and bounded polling
checks for:

- known sender, control-adapter, raw CAN writer, and full-bridge commands;
- `/to_can_bus` and `/to_can_bus_fd` publisher/subscriber endpoints;
- repeatedly proven `can0` `UP`/`ERROR-ACTIVE`/500-kbit/s/`LISTEN-ONLY` state and unchanged ifindex;
- `can1`, `can2`, and `can3` remaining administratively down;
- unchanged TX packet counters on all four CAN interfaces;
- exactly one receiver from the reviewed executable/component/core-library paths and hashes, with
  those exact libraries present in its runtime memory map and a captured launch process
  identity/group;
- exactly one publisher for each required raw/status topic, with the reviewed message type, receiver
  node name/namespace, and an unchanged DDS GID;
- the approved wrapper/child launch paths and hashes, PC1 NIC, Domain 10, CycloneDDS profile, package
  prefix, and Chrony source; and
- successful capture of live transport samples for raw CAN plus velocity, steering, gear, and
  control mode.

ROS graph query failures and unparsable endpoint counts are faults, not zero-endpoint evidence. The
runner captures `/proc` start times for the supervisor, launch, watchdog, and receiver before using
their PIDs. This prevents a delayed watchdog or cleanup path from signalling a reused PID. It starts
the receiver in a dedicated process group but holds the child at `SIGSTOP` until a final repeated
provenance/network/clock/endpoint/physical gate passes. It then applies bounded INT/TERM/KILL cleanup,
never waits without a proven terminal child state, verifies zero receiver/transmit residue, and only
then hashes a newly allocated evidence directory.

The controller `LISTEN-ONLY` mode and independent physical TX inhibition are the continuous prevention
layers. Fast CAN/process/identity guards run before and after every required sample, between bounded
command groups in the `during` snapshot, and about once per second afterward. Full network, clock,
launch, lifecycle/parameter, DDS publisher/GID, and transmit-endpoint checks bracket the
required-sample batch and the `during` snapshot phase, then repeat after each completed 30-second
steady interval. Long ROS graph probes are
bounded and have fast physical-safety checkpoints between them. An unreadable guarded receiver
identity, ROS graph, CAN, network, or clock state is a fault rather than proof that the resource
disappeared.

The runner is intentionally fixed to physical `can0`, vehicle Domain 10, CycloneDDS, and an explicitly
configured PC1 `CYCLONEDDS_URI`. Alternate domains or interfaces belong to a separate isolated bench
profile and cannot be selected with this vehicle runner.

## Static and parked use

```bash
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash

export CYCLONEDDS_URI=file:///home/a/autoware/src/migration_work/config/cyclonedds_pc1.xml
export VILS_PC1_EXPECTED_TIME_SOURCE="<four-PC-approved-chrony-source>"
export VILS_PC1_MAX_LAST_OFFSET_MS="<approved-ms>"
export VILS_PC1_MAX_RMS_OFFSET_MS="<approved-ms>"
export VILS_PC1_MAX_ROOT_DISPERSION_MS="<approved-ms>"
ros2 run ros2_socketcan run_vehicle_status_rx_only.sh --check-only
ros2 run ros2_socketcan run_vehicle_status_rx_only.sh --duration-seconds 600
```

The preflight intentionally fails while `can0` is down, stopped, bus-off, not `LISTEN-ONLY`, when any
of `can1..can3` is up, or when a known transmit path or transmit ROS endpoint is present. The historical
`/home/a/scripts/start.sh` brings `can0` and `can1` up in normal mode and is explicitly forbidden during
this window. Do not bring interfaces up merely to make the check pass: an owner-approved RX-only setup
and rollback procedure is still required. The vehicle operator must independently confirm the physical
connection, MANUAL/PARK, stationary state, and transmission inhibition. Evidence is stored below
`/home/a/vils_pc1_evidence` by default. `--evidence-root` and `VILS_PC1_EVIDENCE_ROOT` may select only
that reviewed root or one of its descendants. The runner never changes permissions on an existing
directory; an existing root must already be owned by the operator and mode `0700`.

The clock thresholds intentionally have no defaults. The four-PC review must approve and record the
same source plus maximum last offset, RMS offset, and root dispersion before this runner can pass.
Operator values may only tighten the runner's reviewed ceilings: 10 ms last offset, 10 ms RMS offset,
and 50 ms root dispersion. Changing those ceilings requires another source review.

The full provenance check deliberately creates fresh, no-daemon ROS graph clients and is repeated at
every readiness and snapshot boundary. A short duration such as 30–60 seconds can still abort before
readiness on a distributed graph and is useful mainly as a fail-closed test. Reserve the documented
600-second window for parked evidence collection and treat a deadline crossed before `vils_ready` as
a failed run, never as a partial PASS. Captured ROS messages establish transport liveness only; their
decoded MANUAL/PARK/near-zero fields are not independent vehicle-state proof.

## Explicit limitations

Controller-level `LISTEN-ONLY` is mandatory, not an alternative to a physical safety control. A real
Stage 1.5 window additionally requires owner approval of the setup/rollback procedure, an independently
enforced physical TX-inhibition control, operator evidence of MANUAL/PARK/stationary state, zero engage
calls, and simultaneous PC2/PC3 baseline acceptance. Until those exist, only static validation and the
expected fail-closed `--check-only` result are approved. Stage 5 remains blocked until both command
freshness watchdogs, request-bit fail-closed behavior, exclusive CAN ownership, audited IONIQ EV
geometry, enforced speed limits, and the reviewed availability/MRM chain are implemented and accepted.

The runner converts catchable terminal stop signals (`HUP`, `INT`, `QUIT`, and `TERM`) into a
fail-closed cleanup and ignores terminal `TSTP` so Ctrl+Z cannot suspend the supervisor or its probe
children. No userspace program can handle `SIGKILL`, `SIGSTOP`, kernel failure, or power loss;
controller `LISTEN-ONLY` and independent physical TX inhibition remain mandatory even when the
software supervisor is healthy.

ROS 2 Humble's graph CLI does not expose a publisher host PID, so the provenance gate is not a
cryptographic DDS-to-process attestation. It combines the unique local process executable, captured
PID/PGID/session/environment, lifecycle/parameter state, unique graph publisher identity/type, and
stable GID to reject accidental duplicates or stale endpoints in the trusted four-PC lab. A hostile
same-name publisher requires DDS security or an application-level signed/nonce heartbeat and is
outside this profile.
