# PC1 result: VILS_PC123_REAL_20260814_1816_R01

Status: **NON-FORMAL / PARTIAL RECOVERED AFTER POWER LOSS / NEGATIVE INTEGRATION TRIAL / NOT AN END-TO-END VILS PASS**

## Outcome

This run captured the PC1 Planning/Control and vehicle/CAN boundary together with object and localization/TF topic streams attributed by the configured architecture to PC2 and PC3, plus the namespaced shadow-object topic attributed to PC4. The PC1 SQLite bag does not preserve authenticated remote-host ownership. It also lacks independent PC4 simulator ground truth and does not show that PC2 accepted or fused the object into the canonical stream. The data therefore supports topic observability, failure analysis, and resource measurements—not a claim of virtual-obstacle avoidance, braking causality, AEB validation, or moving-test acceptance.

The shared run ID was assigned retrospectively. It is linked to the PC3 result by 3,110 byte-identical, same-order serialized PC4-object payloads, not by wall-clock overlap alone. See [`RUN_LINKAGE.md`](RUN_LINKAGE.md). This acquisition was not executed from the frozen formal runbook; the missing formal gates are listed in [`analysis/FORMAL_RUN_GAPS.md`](analysis/FORMAL_RUN_GAPS.md).

## Capture identity

| Item | Value |
|---|---|
| Shared run ID | `20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01` |
| PC1 local capture ID | `pc1_20260814_181555` |
| Recorder start | 2026-08-14 18:15:56.875 KST |
| Last recovered message | 2026-08-14 18:29:42.159 KST |
| rosbag2 metadata duration | 825.258834 s (13 min 45.259 s) |
| First-to-last recovered timestamp span | 825.283893 s |
| Extended message count | 2,184,011 |
| Conservative pre-truncation window | DB0–DB12, 780.009 s, 2,064,803 messages |
| Final bag payload size | 737,767,424 bytes raw; 142,131,008 bytes published as 14 Zstandard files |
| PC1 Autoware commit | `e495c0c39a5a1dc349204693e584488e8575f535` |
| PC1 ros2_ws commit | `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34` |
| Termination | Abrupt vehicle/PC power loss; no graceful recorder/logger stop |

DB13 was physically truncated. It was recovered into a separate candidate, 7,899 valid continuation records were promoted, and the candidate replaced only the working publication copy after ID, topic-reference, timestamp-index, and `PRAGMA quick_check` validation. The corrupt original and conservative recovery remain preserved locally on PC1 and are intentionally not duplicated in Git. See [`analysis/RECOVERY_PROVENANCE.md`](analysis/RECOVERY_PROVENANCE.md).

For the strongest paper claims, use DB0–DB12. Use DB13 only when the recovery method and its limitations are disclosed.

## Key recorded streams

| Stream | Messages | Extended rate/content summary |
|---|---:|---|
| `/from_can_bus` | 1,490,203 | Raw physical receive evidence |
| `/to_can_bus` | 27,510 | ID `0x630` command-side evidence |
| `/vehicle/status/velocity_status` | 41,262 | Maximum longitudinal speed 16.276 m/s |
| `/vehicle/status/control_mode` | 41,282 | Modes 1 and 4 observed |
| `/current/cruise_mode_status` | 41,286 | Two true intervals; second is right-censored by power loss |
| `/localization/kinematic_state` | 40,580 | Header-to-bag age p95 76.94 ms |
| `/localization/acceleration` | 40,534 | Header-to-bag age p95 78.10 ms |
| `/tf` | 89,043 | `map→base_link` analyzed separately, p95 age 71.85 ms |
| PC2 tracker objects | 7,966 | 9.656 Hz overall; 7,137 non-empty |
| PC2 canonical predicted objects | 7,939 | 9.624 Hz overall; 7,138 non-empty |
| PC4 shadow tracked objects | 4,415 | 9.946 Hz; 2,252 non-empty; maximum one object |
| Planning trajectory | 8,207 | 15–645 points; max planned point velocity up to 11.104 m/s |
| Final control command | 27,464 | Commanded velocity max 0.306 m/s; acceleration mostly -1.5 m/s² |
| Hazard status | 8,227 | `emergency=true` in 6,671 samples |
| MRM state | 8,232 | NORMAL/NONE for every recorded sample |

## PC4 and canonical-object finding

- PC4 shadow objects were recorded from 18:22:18.264 to 18:29:42.068 KST at 9.946 Hz overall.
- PC4 was non-empty in 2,252 messages, from 18:24:23.331 through 18:28:14.337 KST. The conservative-window analysis found stable CAR UUID `a26a7b22b87f55e893b4681f5302df71` in 2,177 frames over 218.738 s.
- The PC4 object topic was namespaced and remained separate from PC2 tracker and canonical `PredictedObjects`.
- The bag contains no `/diagnostics/pc4/object_adapter`, `/perception/pc2/vils/accepted_pc4_status`, or `/perception/pc2/vils/candidate_objects` evidence.
- Conservative-window comparison found no exact PC4 UUID in canonical objects and no nearest canonical/tracker object within 2 m of the PC4 object. At the minimum recorded ego-to-PC4 center distance of 0.392 m, ego speed was 3.889 m/s and the nearest canonical object center was 5.222 m away. These are map-frame center-distance observations, not footprint, collision, lane, or causal-response proofs.
- Reproduce the comparison with [`analysis/analyze_vils_pc4_integration_evidence.py`](analysis/analyze_vils_pc4_integration_evidence.py); the summarized output and 2,252-row alignment table are [`analysis/VILS_PC4_INTEGRATION_EVIDENCE.json`](analysis/VILS_PC4_INTEGRATION_EVIDENCE.json) and [`analysis/VILS_PC4_PC2_ALIGNMENT.csv`](analysis/VILS_PC4_PC2_ALIGNMENT.csv).

## Vehicle/control observation

Cruise, decoded vehicle control mode, ROS `/to_can_bus` request bit, and physical `/from_can_bus` ID `0x630` transitions aligned in the recording:

- request active from bag start to about 18:17:18 KST;
- request inactive until about 18:21:31.94 KST;
- request active from then through the abrupt end of the bag.

The second active interval is right-censored; its recorded duration is about 490.2 s. Vehicle speed reached 16.276 m/s while the request signal was active. This is observational correlation only. The current AEB configuration uses the physical obstacle pointcloud and does not consume predicted objects, so this run is not AEB evidence.

## Time and performance limits

PC1 metrics cover only the final approximately 2.5–3 minutes. PC1 Chrony retained PC3 (`192.168.9.7`) as its reference ID, but the PC3 source was never selected as `^*` in the captured source blocks and its reference time was stale. Root dispersion was 18.33–18.62 ms. Therefore:

- do not report the object header-to-bag differences as one-way network latency;
- do not claim sub-millisecond four-PC synchronization;
- do not infer PC2, PC3, or PC4 clock quality from PC1 ping data.

See [`metrics/PC1_METRICS_AND_CHRONY.md`](metrics/PC1_METRICS_AND_CHRONY.md).

## Paper-use disposition

| Claim | Disposition |
|---|---|
| The PC4-namespaced object topic was recorded on vehicle Domain 10 at about 9.94 Hz | Supported; originating-host/simulator provenance is not independently proven |
| A stable PC4 virtual CAR was present for about 218.7 s | Supported within the conservative window |
| Named Planning/Control, vehicle/CAN, object, localization/TF, and PC4-facing streams were co-recorded | Supported for the topics listed in `metadata.yaml`; remote host ownership is architectural attribution, not authenticated by the PC1 bag |
| PC2 accepted or fused the PC4 object into canonical objects | Not supported; recorded comparison is negative |
| PC1 planned or braked because of the PC4 object | Not supported |
| AEB responded to PC4 predicted objects | Not supported and inconsistent with the active AEB input configuration |
| Four-PC clocks were synchronized tightly enough for one-way latency | Not supported |
| This was a successful or safe moving Stage 5 VILS test | Not supported |

The detailed claim matrix is in [`analysis/PAPER_CLAIM_MATRIX.csv`](analysis/PAPER_CLAIM_MATRIX.csv).

## Files and validation

- `rosbag/`: `metadata.yaml`, 14 Git-LFS-backed `.db3.zst` files that restore the validated raw `.db3` files, bag info, and DB integrity results.
- `metrics/`: raw PC1 system/GPU/ping/Chrony/process sidecars plus the audited summary.
- `analysis/`: semantic JSON, reproducible analysis scripts, recovery provenance, formal-run gaps, and the claim matrix.
- `published_checksums.sha256`: SHA-256 for every published source file in this PC1 run package, excluding the manifest itself.

After `git lfs pull`, verify the complete package from this directory with:

```bash
sha256sum -c published_checksums.sha256
```

Validation performed before publication:

- all 14 publication DBs: `PRAGMA quick_check = ok`;
- message IDs and topic references validated for recovered DB13;
- selected ROS message types deserialized with zero errors in the published semantic analysis;
- compressed and restored-raw bag hashes verified against their dedicated manifests;
- run-package hashes generated after final editing and recorded in `published_checksums.sha256`;
- no ROS replay, publish, service, action, parameter mutation, CAN mutation, or vehicle actuation was performed during recovery and publication.

## Reproduce the analyses

Use the ROS 2 Humble/Autoware environment corresponding to PC1 commit
`e495c0c39a5a1dc349204693e584488e8575f535`. From this run directory, first restore
the bag as described in [`rosbag/README.md`](rosbag/README.md), then run:

```bash
python3 analysis/analyze_bag_semantics.py --bag-dir rosbag \
  > analysis/BAG_SEMANTIC_SUMMARY.json
python3 analysis/analyze_vils_pc4_integration_evidence.py \
  --bag-dir rosbag \
  --json-output analysis/VILS_PC4_INTEGRATION_EVIDENCE.json \
  --csv-output analysis/VILS_PC4_PC2_ALIGNMENT.csv
```

Both scripts open SQLite read-only and never start a ROS node, replay a bag, or
publish data. Re-running them overwrites the named analysis outputs, so use a clean
copy when independently auditing the committed artifacts.

## Safety warning

The bag includes raw and generated CAN frames, control commands, and vehicle-status transitions. Never replay it on Domain 10, on a vehicle-connected ROS graph, or while a SocketCAN sender/control adapter is active. Analysis must use an isolated ROS domain with physical CAN interfaces down or absent.
