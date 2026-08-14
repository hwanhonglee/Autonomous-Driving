<!-- HH_260814 - Define the distributed, source-owner data-acquisition protocol for the VILS paper. -->
<!-- cspell:ignore VILS vils -->
# VILS paper dataset acquisition

Status: **POST-v2 PROTOTYPE; OFFLINE-PREPARED; NOT A COMPLETED VEHICLE DATASET**

The paper dataset is recorded locally on all four source-owner PCs under one shared
`run_id`. PC3 coordinates manifests and checksums, but it must not record every raw
stream over DDS. A central all-topic recorder would add network loss, CPU load, and
storage contention to the system being measured.

## Recording ownership

| Host | Local authoritative evidence | Recommended root |
|---|---|---|
| PC1 | canonical objects, route, trajectory, controller/final command, vehicle reports, MRM, CAN RX/TX counters | `/home/a/pc1_vils_runs/<UTC>-<run_id>/` |
| PC2 | physical objects, raw PC4 ingress, validation decisions, fusion provenance, candidate/canonical tracked and predicted objects | `/home/a/pc2_vils_runs/<UTC>-<run_id>/` |
| PC3 | active map/config hashes, GNSS/INS, NDT/EKF, `map -> base_link`, LiDAR source, diagnostics, MRM, time evidence | `/home/a/pc3_vils_runs/<UTC>-<run_id>/` |
| PC4 | CARLA per-tick actor ground truth, ego mirror, detector/tracker/adapter output, TX events, gateway audit, fault schedule | `/home/a/pc4_vils_runs/<UTC>-<run_id>/` |

PC2 owns the main fusion measurements, but it is not the only recorder. PC4 output
cannot be used as its own ground truth: CARLA actor state must be captured directly.
Likewise, PC3 NDT/EKF is an operational pose reference, not independent accuracy
ground truth unless an independent RTK, total-station, or motion-capture reference is
available.

## Common identity and clocks

Every host manifest must contain the same:

- `run_id`, scenario, condition, stage, mode, replicate, and random seed;
- intended UTC start window and actual start/end timestamps;
- route, CARLA world/settings, map digest, and simulator-to-map transform digest;
- Git commit, dirty status, installed binary/config digest, ROS domain, RMW, and DDS
  profile digest; and
- Chrony source, reach, offset, jitter, stratum, boot ID, and any clock-step event.

Cross-host one-way latency is publishable only after all four hosts pass the common
clock gate. Before that, report only per-host monotonic processing time, message
delivery, order, duplicates, and inter-arrival gaps.

## PC3 profiles

PC3 uses two separate local bags:

1. `core`: low-bandwidth localization, GNSS/INS, NDT metrics, TF, diagnostics, and
   MRM. Record this for the entire run.
2. `lidar`: the canonical cloud that PC3 relays to PC2. Record this only when raw
   perception reproducibility is required. PC2 records its received relay locally,
   so PC3 does not duplicate both canonical and relay clouds in one bag.

The recorder uses split MCAP files with no real-time compression. This reduces CPU
interference and limits damage from a sudden power loss. Compress only after the run
has stopped and the uncompressed checksums have been sealed.

## Prepare a new PC3 run

Never continue a previous or aborted run directory. Use the same run ID chosen by
the four-host operator:

```bash
cd /home/a/pc3-vils-integration.U0Ogk3/src/migration_work/vils
python3 paper_dataset/prepare_pc3_paper_run.py \
  --run-id VILS_B0_20260815_001 \
  --scenario-id C_TRACK_STATIC_01 \
  --condition-id B0_REAL_ONLY \
  --replicate-id 1 \
  --stage 1.5 \
  --mode real_only \
  --random-seed 1001 \
  --planned-duration-sec 600
```

The command prints a new owner-only run directory. It calls the existing PC3
evidence collector and freezes the paper contract, both topic allow-lists, and both
QoS profiles into that run manifest.

## Record on PC3 after all physical gates pass

```bash
source /opt/ros/humble/setup.bash
source /home/a/autoware/install/setup.bash
source /home/a/pc3-vils-integration.U0Ogk3/src/migration_work/vils/activate_pc3_vils_environment.sh

python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase before

paper_dataset/record_pc3_paper_bag.sh <run_dir> core 600
paper_dataset/record_pc3_paper_bag.sh <run_dir> lidar 600  # optional, local PC3 NVMe only

python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase during
python3 collect_pc3_vils_evidence.py snapshot --run-dir <run_dir> --phase after
python3 collect_pc3_vils_evidence.py finalize --run-dir <run_dir>
```

For a simultaneous core and LiDAR window, start the two recorder commands in two
terminals after both display their preflight result. Do not use `ros2 bag record -a`.

## Pilot storage measurement

Before a formal run, record a fresh 60-second pilot for each profile and calculate:

```text
required_bytes = measured_60s_bytes * planned_seconds / 60 * 1.20
```

The observed PC3 canonical PointCloud2 was approximately 1.52 MB at 10 Hz, or about
9.15 GB raw for ten minutes. The recorder therefore requires at least a 5 GiB base
reserve plus 20 MB/s for the requested LiDAR duration. The formal manifest must use
the measured pilot size rather than an assumed compression ratio.

## Experimental conditions

| Condition | PC4 | PC2 selection | Vehicle actuation |
|---|---|---|---|
| B0 Stage 1.5 | fully off | physical only | disabled; ten-minute baseline |
| B1 Stage 2 | raw shadow | physical canonical unchanged | disabled |
| C1 Stage 3 | validated debug candidate | physical canonical unchanged | disabled |
| C2 Stage 4 | accepted `hybrid_optional` | one canonical PC2 writer | parked/no sink |
| C3 required mode | reviewed accepted-status policy | blocked until owner review | parked/no sink only |
| Stage 5 | approved static target | approved canonical path | separate written approval, closed course, <=0.833 m/s |

Preserve physical occupancy and physical-priority validation on the real vehicle.
Unsafe ablations belong in offline replay or same-host SIL, not a moving vehicle.

## Paper endpoints

- transport: application message delivery ratio, duplicates, order, maximum gap,
  latency, and jitter; packet PDR is reported separately from pcap;
- validation/fusion: accept/reject accuracy, physical retention, virtual inclusion,
  association correctness, duplicate/ghost rate, and provenance completeness;
- planning: object onset to prediction, trajectory, and control response, planned
  stop position, minimum clearance, acceleration, and jerk;
- fault handling: TTL removal, physical-stream continuity, restart/session recovery,
  and availability/MRM response under the approved mode;
- alignment: surveyed control-point and ego-overlay x/y/z/yaw RMS and maximum error;
  and
- system cost: CPU, GPU, RSS, VRAM, DDS queue/drop, rate, and maximum gap.

Messages are not independent statistical samples. Use independent fresh-process
runs as the analysis unit. Three runs per condition are only a pilot floor. Freeze a
power/sample-size plan before formal inference; a practical starting point is at
least ten independent runs per condition across at least three power-cycle/day
blocks when the power analysis does not demand more.

## Interrupted runs

An interrupted run is never repaired or concatenated with a later boot. Record one
of `COMPLETE_PASS`, `COMPLETE_FAIL`, `ABORTED_POWER_LOSS`, `ABORTED_OPERATOR`, or
`CLEANUP_FAIL`, preserve all partial files, and start a new `run_id`.

The 2026-08-14 preflight run
`20260814T053915Z-VILS_4PC_20260814_143846` contains only the initial manifest and
`before` snapshot. It is marked `ABORTED_POWER_LOSS`, is not paper-sample eligible,
and must remain excluded from acceptance statistics.
