<!-- HH_260814 - Publish the recovered PC3 field-run evidence and its analysis limits. -->
# PC3 result: VILS_PC123_REAL_20260814_1816_R01

## Result classification

- Status: `PARTIAL_RECOVERED_POWER_LOSS`
- Purpose: real-vehicle communication and system-behavior analysis
- Formal VILS scenario: no
- Safety-acceptance eligible: no
- Communication-analysis eligible: yes
- Vehicle power loss ended the cross-PC recording early.

The PC3 core recorder completed and validated before the power loss. The cross-PC
recorder was interrupted, but all six MCAP chunks survived. Its original directory
was copied byte-for-byte before `ros2 bag reindex` reconstructed `metadata.yaml`.
The reconstructed bag opens normally and all published SHA-256 digests pass.

## Captured windows

| Dataset | Local window (KST) | Duration | Messages | Size | Result |
|---|---|---:|---:|---:|---|
| PC3 core | 18:18:34.789–18:28:33.569 | 598.779949 s | 822,307 | 350.2 MiB | complete and validated |
| Cross-PC link | 18:24:29.611–18:29:51.592 | 321.980505 s | 127,414 | 136.1 MiB | partial, reindexed and readable |

The core bag covers localization initialization, EKF odometry/acceleration, NDT
scores and timing, selected GNSS, NovAtel BESTPOS/INSPVAX/INSSTDEV/RXSTATUS/IMU,
RTCM, TF, diagnostics, and MRM. Its raw MCAP is retained on PC3 because it contains
high-volume diagnostics and precise GNSS trajectory data. This result directory
publishes its validation report, bag inventory, recovery record, and digests.

The cross-PC bag is the communication-analysis artifact. It contains PC4 virtual
tracked objects, PC2 detection/tracking/canonical objects, the planned trajectory,
trajectory-follower and final control commands, vehicle reports, operation-mode
availability, and MRM state.

## Cross-PC counts and observed average rates

Rates below are `count / recovered_duration`; they describe observed messages at
PC3 and are not a packet-level delivery ratio.

| Topic group | Count | Mean observed rate |
|---|---:|---:|
| PC4 virtual tracked objects | 3,204 | 9.951 Hz |
| PC2 detected objects | 2,992 | 9.292 Hz |
| PC2 tracked objects | 3,235 | 10.047 Hz |
| PC2 canonical predicted objects | 3,234 | 10.044 Hz |
| Planning trajectory | 3,216 | 9.988 Hz |
| Trajectory-follower control command | 10,717 | 33.285 Hz |
| Final control command | 10,718 | 33.288 Hz |
| Vehicle velocity report | 16,098 | 49.997 Hz |
| MRM state | 3,221 | 10.004 Hz |

## What this data can answer

- continuity, gaps, order, and observed rates at the PC3 DDS participant;
- timing relationships among PC4 objects, PC2 outputs, trajectories, commands, and
  vehicle reports when message stamps allow a defensible match;
- PC3 localization/NDT/GNSS/MRM continuity during the overlapping driving window;
- control-mode, availability, command, and vehicle-response timelines; and
- power-loss truncation behavior and MCAP recovery.

## Limits that must remain visible

- The run was not the frozen formal paper scenario and exceeded the earlier
  low-speed VILS preparation envelope.
- PC2 canonical output includes the physical perception pipeline. Similar message
  counts do not prove that a particular PC4 object was accepted or fused.
- `/diagnostics/pc4/object_adapter` and
  `/perception/pc2/vils/accepted_pc4_status` were not present in the recovered bag,
  so source session, validator decision, and accepted-object provenance cannot be
  reconstructed from PC3 alone.
- Cross-host one-way latency is publishable only after matching PC1–PC4 clock
  evidence proves a common time base and bounded offset/jitter for this run.
- PC3 NDT/EKF is an operational pose reference, not independent ground truth.
- Power loss ended the crosslink window at 321.98 seconds; later data must use a new
  `run_id` and must not be appended here.

## Files

- `pc3_core_bag_info.txt`: validated core inventory and per-topic counts.
- `pc3_core_recording_validation.json`: automated duration/message gates.
- `pc3_crosslink_bag_info.txt`: recovered cross-PC inventory.
- `pc3_crosslink_topic_counts.csv`: machine-readable topic counts and rates.
- `power_loss_recovery.json`: classification and recovery provenance.
- `paper_trial.json`: immutable pre-run manifest; its initial status is superseded
  by `power_loss_recovery.json`, not rewritten after the interruption.
- `source_checksums.sha256`: digests for the retained complete local source artifacts.
- `published_checksums.sha256`: directly verifies every published data artifact.
- `raw_crosslink/`: approved cross-PC MCAP chunks and metadata when Git LFS is used.

The untouched power-loss copy and the complete core MCAP remain under the local PC3
run directory recorded in `LOCAL_DATA_LOCATION.txt`; that local path is evidence
provenance, not a portable download URL.
