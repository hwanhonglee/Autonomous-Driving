<!-- HH_260810 - Define the PC4 result package for the 2026-08-14 outdoor four-PC shadow run. -->
# PC4 actual four-PC test result

<!-- HH_260810 - Record the result status without promoting an interrupted run to nominal acceptance. -->
Status: **EVIDENCE INTEGRITY PASS / BOUNDED PARTIAL RESULT / NOMINAL 4-PC PASS NOT CLAIMED**

<!-- HH_260810 - Identify the run and its immutable local source. -->
## Run identity

| Item | Value |
|---|---|
| Date and timezone | 2026-08-14, Asia/Seoul (KST) |
| Run ID | `20260814T092321.301564497Z-outdoor_route_aligned_shadow_response-b0fed535-04fb-41f6-85e7-48e04c426867` |
| Adapter session | `bfdab0f6-926f-47d1-b66b-dc3ae8215d0b` |
| Vehicle domain | ROS Domain 10 |
| PC4 private domain | ROS Domain 42, localhost-only source stack |
| PC4 actuation | `VILS_PC4_SIM_ACTUATION_ARMED=0` |
| Capture completion | `INCOMPLETE`, exit status `130` |
| Interruption | Vehicle power loss followed by the operator's stop request |
| Integrity | 65/65 source checksums verified |

<!-- HH_260810 - Preserve the exact local evidence path and its aggregate checksum. -->
The complete 549 MiB source evidence remains read-only on PC4 at:

```text
/home/a/carla-autoware-universe/autoware/pc4_vils_runs/linked_vehicle_evidence/20260814T092321.301564497Z-outdoor_route_aligned_shadow_response-b0fed535-04fb-41f6-85e7-48e04c426867
```

Its `checksums.sha256` file has SHA-256
`b04ae3dcc9251767c760e776946bde82a404ded5d6c55cd550edd7c10f58aee2`.
The two rosbag databases total about 536 MiB and are stored in this branch with Git LFS. Their
adjacent `metadata.yaml` files, exact hashes, and compact measurements are included here as well.

<!-- HH_260810 - State the supported result before the detailed measurements. -->
## Final result

This run demonstrated all of the following:

- PC1, PC2, and PC3 vehicle-side data on Domain 10 were observed together with PC4 CARLA/Autoware data on private Domain 42.
- The actual route and PC4 route used the same goal and the same ordered six-lanelet segment sequence.
- The PC4 route start was aligned within 8.142 mm in XY and 0.017167 degrees in yaw.
- PC4 virtual-object presence and absence produced repeatable changes in the PC4-local Planning trajectory.
- Every captured adapter TX record matched its same-host observer by serialized CDR and order.
- Every Domain 42 shadow sample observed on Domain 10 during connected intervals matched byte-for-byte and remained ordered.
- Vehicle power and Ethernet loss produced one contiguous missing interval while PC4-local acquisition remained alive.
- Rosbags, metrics, lifecycle records, and checksum evidence were safely flushed and finalized.

This run did **not** demonstrate any of the following:

- PC2 application ingress, validator, TTL, session/map gate, fuser, or accepted-source status for the PC4 stream
- PC4 virtual objects influencing the actual PC1 canonical Planning result
- a four-host common clock or cross-host one-way latency
- byte-identical PC3 and PC4 maps or accepted 3-D map alignment
- verified actual-vehicle closed-loop command causality
- an uninterrupted nominal 660-second four-PC trial

<!-- HH_260810 - Separate successful checks from blocked or untested claims. -->
## Result matrix

| Check | Result | Boundary |
|---|---|---|
| Evidence checksum | PASS | 65/65 files verified |
| Managed capture cleanup | PASS | 5/5 children stopped, return code 0, no owned residue |
| PC4/CARLA cleanup | PASS | matching processes 0; ports 2000, 2001, and 2002 closed |
| Same goal and lanelet sequence | PASS | goal and ordered segment IDs exactly equal |
| Route-start SE(2) alignment | PASS | XY 8.142 mm; yaw 0.017167 degrees |
| Route-start Z/map identity | BLOCKED | Z differs by 1.597399 m; map hashes differ |
| PC4 object to PC4 Planning response | PASS | repeatable trajectory change in A/B/A/B/A sequence |
| Adapter TX to observer CDR | PASS | 7,511/7,511 exact; missing, duplicate, reorder, mismatch all zero |
| D42 to D10 connected-window CDR | PASS | all samples in both connected ranges exact and ordered |
| Whole-run D42 to D10 delivery | PARTIAL | 1,498 source samples absent during power/link outage |
| PC2-local PC4 ingress | NOT TESTED | no PC2-local recorder or PC2 application subscriber |
| PC2 validator/fuser acceptance | NOT IMPLEMENTED/TESTED | absent from the current PC2 v2.0.0 baseline |
| PC4 influence on actual Planning | NOT TESTED | PC4 shadow was not fused into the canonical object path |
| Four-host Chrony | NOT ESTABLISHED | only PC4 was recorded; vehicle NTP candidate had reach 0 |
| Nominal full run | INCOMPLETE | capture stopped at about 636 seconds after vehicle power loss |

<!-- HH_260810 - Record the exact route comparison while excluding randomized UUID equality. -->
## Route alignment

The route request used `allow_goal_modification=false` and the following goal:

```text
goal = (-5.672883033752441, -125.20557403564453, -13.1305)
goal_yaw = -3.084712276694325 rad
segments = [30447, 28782, 29266, 28789, 35351, 30005]
```

| Pose | X (m) | Y (m) | Z (m) | Yaw (rad) |
|---|---:|---:|---:|---:|
| Actual Domain 10 route start | 58.1841132283 | -124.981811196 | -14.8060241802 | 3.13571136622 |
| PC4 Domain 42 route start | 58.1900314622 | -124.976219812 | -16.4034235064 | 3.13601099232 |

The resulting PC4-minus-actual difference was `dx=+0.005918 m`, `dy=+0.005591 m`,
`XY=0.008142 m`, `dyaw=+0.017167 degrees`, and `dz=-1.597399 m`.
The route UUIDs were different, as expected for two route instances, and are not an equality criterion.

<!-- HH_260810 - Record the source bag sizes, durations, and topic counts. -->
## Captured data

| Data | Duration | Messages | Topics | Source size |
|---|---:|---:|---:|---:|
| Domain 42 source bag | 636.012794413 s | 318,318 | 33 | 301,322,240 bytes |
| Domain 10 vehicle observer bag | 636.104066779 s | 186,115 | 21 | 261,165,056 bytes |

Additional PC4 evidence includes 621 host-metric samples, 636 NIC samples, 10 Chrony
samples, ROS graph and endpoint snapshots, CARLA inventories, lifecycle records, and bounded
adapter TX/observer event logs.

The operator requested stop about 24 seconds before the planned 660-second deadline. The
`INCOMPLETE` marker therefore describes planned-duration completion, not evidence corruption.

<!-- HH_260810 - Quantify adapter preservation using same-host monotonic timing only. -->
## Adapter CDR and timing result

| Metric | Value |
|---|---:|
| TX snapshot | 7,511 |
| Observer snapshot | 7,512 |
| Exact TX matches | 7,511 |
| Missing / duplicate / reorder / mismatch | 0 / 0 / 0 / 0 |
| Publish-to-observer median | 411.496 us |
| Publish-to-observer p95 | 733.451 us |
| Publish-to-observer p99 | 887.225 us |
| Publish-to-observer maximum | 3.819334 ms |

The observer-only final row is the next empty snapshot captured after the TX prefix was frozen;
it is a bounded-snapshot race, not a middle-of-stream duplicate. These timing values are PC4
same-host monotonic measurements and must not be described as PC4-to-PC2 network latency.

<!-- HH_260810 - Quantify cross-domain shadow delivery and isolate the outage interval. -->
## Domain 42 to Domain 10 shadow transport

| Metric | Value |
|---|---:|
| Domain 42 source samples | 6,331 |
| Domain 10 observed samples | 4,834 |
| Exact source matches | 4,833 |
| Missing source samples | 1,498 |
| Initial Domain 10 extra | 1 |
| Duplicate / reorder | 0 / 0 |
| Whole-run match ratio | 76.3387% |

The exact connected ranges were source indices `0..3835` and `5334..6330`. The single missing
range was `3836..5333`, corresponding to source stamps from 18:30:09.575842 to
18:32:40.022301 KST. The Domain 10 observer gap was 150.649082327 seconds.

The whole-run ratio is **not** a nominal network PDR: the 1,498 missing samples were concentrated
in the vehicle-power and Ethernet-outage interval. The supported connected-window claim is
byte-exact CDR delivery with no reordering.

<!-- HH_260810 - Quantify the repeatable local Planning response to virtual-object state. -->
## PC4 virtual-object causal response

The static CARLA Audi A2 was placed approximately 7.000005 m ahead, removed, respawned at the
same position, and removed again. This produced an A/B/A/B/A absent/present sequence.

| State | Trajectory messages | Points per trajectory | Maximum trajectory speed |
|---|---:|---:|---:|
| Object absent | 3,022 | 162 | 2.7777793407440186 m/s |
| Object present | 3,338 | 163 | 0.25001096725463867 m/s |

For 3,338 object-present trajectories, the minimum trajectory-to-object XY distance ranged from
0.02080 to 0.24871 m, with mean 0.22030 m and median 0.22598 m. Stable object-present intervals
lasted about 218.7 seconds and 107.1 seconds, separated by an 87.9-second absent interval.
The first spawn required approximately 12 seconds before the 0/1 detection oscillation settled.

PC4 localization remained nearly fixed: X span 9.52 mm, Y span 0.187 mm, Z span 6.0 mm, and
first-to-last XY drift 0.819 mm. This supports a PC4-local object-to-trajectory relationship rather
than a response caused by ego motion. It does not establish actual-vehicle response: PC4
actuation was disarmed, and the final PC4 command remained a zero-speed, -1.5 m/s^2 hold.

<!-- HH_260810 - Summarize actual Domain 10 observations without asserting remote-host provenance. -->
## Actual Domain 10 observer result

| Observed topic group | Result before power loss |
|---|---|
| Vehicle velocity | 18,951 samples; mean 5.214 m/s; maximum 16.276 m/s |
| Control mode | 18,962 samples; report value 1 throughout |
| PC2 tracking | 3,807 samples; 0..35 objects; mean 7.741 |
| PC2 prediction | 3,808 samples; 0..35 objects; mean 7.740 |
| Actual trajectory | 3,789 samples; 15..645 points; maximum point velocity 11.104 m/s |
| Trajectory-follower command | 12,617 samples at about 33.31 Hz |
| Final control command | 12,620 samples; velocity 0 and acceleration -1.5 throughout |
| Localization | about 61.05 m straight-line XY displacement over the observed drive |

These are PC4 Domain 10 observer measurements, not PC1/PC2/PC3 host-local recordings. The
vehicle status showed motion while the recorded final canonical control command remained a stop
command. Therefore actual closed-loop causality and custom vehicle-interface semantics remain
inconclusive and require host-local CAN and command evidence.

<!-- HH_260810 - Preserve the unplanned power and carrier failure as a separate fault observation. -->
## Vehicle-power and Ethernet fault observation

- PC3 localization ended at approximately 18:29:36 KST.
- The remaining vehicle-side Domain 10 streams ended near 18:30:03 KST.
- PC4 `enp3s0` lost carrier near 18:30:03.537 KST.
- The checksummed NIC CSV showed zero RX growth for about 152.135 seconds.
- A live-host kernel journal, which is auxiliary and not part of the checksum package, showed four carrier-down intervals totaling about 128.755 seconds.
- Link recovery did not restore PC1, PC2, or PC3 because their vehicle power was already off.
- The PC4-local shadow stream recovered near 10 Hz and remained isolated from the absent vehicle streams.

This was an unplanned battery/power failure, not a controlled fault-injection trial. It must remain
separate from nominal performance statistics.

<!-- HH_260810 - Report PC4-local clock observations and reject cross-host latency claims. -->
## Chrony result

All 10 PC4 Chrony samples selected public source `193.123.243.2`. PC4 system offset ranged from
-0.501892 to +1.973268 ms, and RMS offset ranged from 1.536130 to 1.791266 ms. The intended
vehicle-side candidate `192.168.9.102` remained unreachable with reach 0 in all samples.

No PC1, PC2, or PC3 local Chrony record is part of this package. Consequently, this run supports
PC4-local timing and gap analysis only; it does not support four-host synchronization or one-way
application-latency claims.

<!-- HH_260810 - Record map hashes and the unresolved vertical convention. -->
## Map and transform boundary

| Artifact | PC4 SHA-256 | Actual Domain 10 SHA-256 |
|---|---|---|
| Lanelet map | `a366b9e7ff36ef900d7160f175e7e0a6edecdaa2177fbe633b2704f032ed4f60` | `ad3e8673ac336432676e56a93fef1617ed3bc2698fb9f7d97943384b8da5aee6` |
| Pointcloud map | `3f227f8d0c07377aabe8bf4248ca181f7bd8b3687b329717b3825bf9fcef25b4` | `29d958e74c37da80f345285f9f7ef9f49bc35ce41ffb44a7c82ffe3a70825153` |

The projector latitude and longitude matched, but PC4 altitude was 100.641012 m while the actual
map used 86.641012 m, a 14.0 m difference. The transform remains
`EMPIRICAL_SE2_ROUTE_START_ALIGNED_Z_UNVERIFIED`; exact 3-D map alignment is not claimed.

<!-- HH_260810 - Record safe cleanup and zero runtime residue. -->
## Shutdown result

The capture runner, both rosbag recorders, Chrony and metric collectors, gateway, adapter/logger,
PC4 Autoware, and CARLA were stopped. All five managed capture children recorded `PASS` with
return code 0 and no remaining owned session. Matching PC4 runtime processes and listeners on
ports 2000, 2001, and 2002 were zero after cleanup. PC1, PC2, and PC3 were already powered off;
PC4 sent no remote shutdown command.

<!-- HH_260810 - Define the minimum evidence needed to replace this bounded run with nominal acceptance. -->
## Required next run

1. Supply stable external vehicle power and secure the Ethernet link.
2. Start host-local recorders on PC1, PC2, and PC3 before the scenario.
3. Capture Chrony source, reach, offset, and jitter on all four hosts in the same window.
4. Freeze exact PC3/PC4 map manifests and validate at least three non-collinear control points.
5. Keep the result explicitly `shadow transport only` unless PC2's post-v2 validator/fuser is installed.
6. If the validator/fuser is installed, record PC2 raw ingress, rejection reasons, selected source, and accepted output locally.
7. Run the full 660 seconds without interruption, at least three independent times.
8. Report nominal runs separately from controlled power/link fault trials.

<!-- HH_260810 - Index the compact Git evidence and the Git LFS source artifacts. -->
## Files in this PC4 result folder

- `summary.json`: machine-readable result and claim status
- `metrics.csv`: compact values used in tables and manuscript analysis
- `source_evidence/`: review copies of small source metadata and PC4 metric records
- `source_evidence/SOURCE_ARTIFACTS.sha256`: original 65-file manifest, including rosbag hashes
- `source_evidence/rosbag/domain10_vehicle/`: Domain 10 metadata and Git LFS database
- `source_evidence/rosbag/domain42_source/`: Domain 42 metadata and Git LFS database

The two Git LFS databases remain read-only in the original PC4 evidence root and are published
with these hashes:

```text
39742c96abaa95153a94143714dd6e194b2316ce84293edcb6eab7d39a00d0c4  domain10_vehicle_0.db3
32796358ac3c2911e605fdea0b111d089cec81dfb130f7b9406f3cb88f6ab9c0  domain42_source_0.db3
```
