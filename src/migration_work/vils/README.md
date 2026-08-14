<!-- HH_260814 - Document the PC3-only read-only evidence workflow for actual four-PC VILS preparation. -->
# PC3 VILS evidence tooling

Status: **PROTOTYPE; READ-ONLY EVIDENCE TOOLING; NO VEHICLE OR PC4 OBJECT AUTHORITY**

This directory implements only the PC3 preparation work allowed by the reviewed
four-PC runbook. It does not add a PC4 object subscriber, object fuser, simulator
TF, canonical perception publisher, engage request, CAN writer, or MRM policy.

The immutable rollback baselines remain:

- PC3 Autoware: 947dda782ce90e1d9768e57ae4337e3cf78eee1b
- PC3 ros2_ws: 22f521ffb7ddeae893f377e226091641bb540efc

The source branch containing this tooling is a post-v2 integration candidate. It
must remain marked PROTOTYPE until the four-host owner review and staged acceptance
are complete.

## Why this tooling exists

PC4's same-host surrogate test proved a bounded live object path and 203/203
ordered application payloads, but it did not prove the real PC3 map bytes,
map-to-base-link authority, four-host time, distributed network, or vehicle
localization. PC3 must provide those exact facts before the PC4 gateway can be
opened toward the vehicle domain.

## Safety boundary

The collector:

- never launches Autoware, a hardware driver, a service, an action, or CAN;
- never changes map, GNSS, Chrony, network, MRM, or ROS parameters;
- runs read-only host and ROS graph commands with timeouts;
- writes only to a new owner-only evidence directory;
- refuses to overwrite an existing run or snapshot;
- records command failure rather than hiding missing evidence.

The bag helper subscribes only to PC3 state, map, sensing, GNSS, TF, diagnostics,
and MRM topics. It contains no planning/control/vehicle command or CAN topic.

## Map convention gate

Lanelet2, PCD, map_projector_info, map_config, and the GNSS poser offset are one
atomic convention.

Allowed choices are:

1. Keep the actually deployed legacy 52SCF0 bundle and keep the reviewed GNSS
   projected-position offset enabled.
2. Deploy the native C-track 52SCF60 bundle atomically and disable the legacy
   GNSS offset in a separate reviewed release.

Do not mix those choices and do not copy the GNSS subtraction into PC4. The
collector records bytes and parameters; it does not approve either convention.

## Create a run

Use a unique run ID shared by PC1 through PC4:

    source /opt/ros/humble/setup.bash
    source /home/a/autoware/install/setup.bash
    source ./activate_pc3_vils_environment.sh

    python3 collect_pc3_vils_evidence.py init \
      --run-id VILS_STAGE15_001 \
      --evidence-root /home/a/pc3_vils_runs \
      --map-dir /home/a/Autoware_Map/C_track \
      --autoware-root /home/a/autoware \
      --ros2-root /home/a/ros2_ws

The command prints the new run directory. It refuses to reuse an existing path.
Runtime snapshots fail closed unless this exact versioned DDS profile remains
active, so vehicle-domain discovery cannot silently select Wi-Fi or the LiDAR LAN.

## Capture staged snapshots

Capture before, one or more during samples, and after:

    python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase before
    python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase during
    python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase after

Each snapshot records current map hashes, GNSS offset values, source/install
provenance, process arguments, Chrony, network, and ROS graph/topic endpoint data.
The `during` phase receives a sequence suffix so repeated samples never overwrite.

## Record ROS evidence

Only after Stage 1.5 prerequisites are physically confirmed, run the read-only bag
helper in another terminal:

    ./record_pc3_vils_bag.sh <run_dir> 600

The second argument is the requested duration in seconds. The helper records the
reviewed topic list with zstd file compression and sends SIGINT at the duration.
It does not decide whether MANUAL/PARK, no-actuation, common time, or MRM ownership
is acceptable; operators and independent recorders must prove those gates first.

## Finalize

After the `after` snapshot:

    python3 collect_pc3_vils_evidence.py finalize --run-dir <run_dir>

Finalization requires before/during/after snapshots and writes a non-overwriting
SHA-256 manifest. Failed and partial runs must be preserved and labeled; never
repair evidence in place.

## Tests

Run without ROS or hardware:

    python3 -m unittest discover -s test -p 'test_*.py' -v

## Current blocking facts

- The external active C-track files are not assumed to equal the Git map candidate.
- PC3 Chrony is only a proposed common source until all four hosts share and accept
  the same source, offset, jitter, reach, and step-free window.
- The PC2 accepted-status message and PC3 availability/MRM consumer are unresolved
  post-v2 proposals and are intentionally not implemented here.
- A fresh post-power 10-minute localization, NDT/EKF, TF, LiDAR, and MRM baseline is
  required before PC4 connects.
