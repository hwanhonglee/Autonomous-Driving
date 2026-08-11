# PC3 HH_260810 changes

> **Historical snapshot:** this report records the 2026-08-10 stationary stage.
> Current v1.1 behavior, validation, and remaining gates are documented in
> [`PC3_v1.1_detailed_changes.md`](PC3_v1.1_detailed_changes.md), which supersedes
> this report where the two records differ. The original content below is
> preserved for audit and rollback history.

Audit date: 2026-08-10 (Asia/Seoul)

## Outcome

PC3's historical `novatel_oem7_driver` was found inside `/home/a/ros2_ws_original.zip`, restored without altering its source, and rebuilt with `novatel_oem7_msgs`. The active PC3 launch now retains NovAtel as the default receiver and points to the stable USB identity for port1 at the historically verified 115200 baud. The receiver itself was not opened or reconfigured.

The active `run_autoware` alias and launch filename remain unchanged. Both the pre-change active files and all historical/current `(copy_org)` files are preserved. The hardware-disabled PC3 stage passed five valid start/stop cycles after fixing the Hesai component's decoder-thread shutdown.

## Where the OEM7 driver went

| Finding | Classification |
|---|---|
| The live `/home/a/ros2_ws` had been recreated with only `HesaiLidar_ROS_2.0` visible before this work | VERIFIED pre-restore snapshot |
| `/home/a/ros2_ws_original.zip` contains the nested OEM7 source packages | VERIFIED |
| The same normalized OEM7 subtree in the archived workspace and the independent PC3 source ZIP is byte-identical | VERIFIED |
| 137 restored files matched the archive with zero mismatches | VERIFIED |
| The archive/recreation operation happened around the June 2026 workspace change | INFERRED from filesystem/build chronology |
| The person, exact command, and reason that archived/recreated the workspace | UNVERIFIED; no shell/audit/Git evidence proves attribution |

Restored package roots:

- `/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_msgs`
- `/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver`

The targeted build completed successfully and the packages resolve from `/home/a/ros2_ws/install`. No unknown replacement driver was downloaded or substituted.

## Preservation

- Immediate pre-change tree: `migration_work/backups/PC3/pre_HH_260810/`
- Pre-change manifest: `migration_work/backups/PC3/pre_HH_260810/SHA256SUMS`
- Final active/backup manifest: `migration_work/backups/PC3/post_HH_260810/SHA256SUMS`
- Original OEM7 archive SHA256: `fb9ff84cf4f89f25c4790bfd3133c3ba48b901d357fb4d86778b256e90b3e923`
- Existing `(copy_org)` files were not overwritten.
- Newly needed system/Nebula `(copy_org)` files were created from the immediate pre-change active files before editing.

An older `(copy_org)` can intentionally differ from the file that was active immediately before this work. For exact rollback, use the dated pre-change snapshot rather than assuming every historical `(copy_org)` is the latest baseline.

## Active behavior after the change

- PC3 owns system monitoring, map loading, vehicle/sensor static TF, and common sensing structure.
- The sensor-owned pointcloud container is the sole PC3 pointcloud container.
- LiDAR and GNSS hardware drivers default off but remain fully wired for readiness-gated enablement.
- NovAtel remains the selected PC3 GNSS receiver.
- Localization defaults off pending orientation, twist, pointcloud, map-alignment, and TF checks.
- Vehicle interface, MRM command publishers, perception, planning, control, API, and RViz default off.
- Static map TF is `map -> viewer`; future dynamic `map -> base_link` is reserved for localization.
- The existing C_track GNSS translation is now applied once to pose, covariance, and TF consistently.

Every active-file change has an adjacent English `HH_260810` comment. The complete path/block, old/new behavior, reason, validation, and rollback table is in `migration_work/HH_260810_CHANGELOG.md`.

## Validation result

- Targeted OEM7 message/driver build: PASS
- OEM7 CTest: 4/6 unique tests PASS; `bestpos` and `ins1` fail only on the archived active `gnss_link` configuration versus older `gps` reference-bag frame IDs
- OEM7 port/net/file launch argument parsing: PASS; no serial launch was performed
- Targeted Autoware/Nebula build: PASS
- XML and YAML parsing: PASS
- Modified C++ formatting: PASS
- Selected Autoware package tests: PASS
- Exact alias, hardware-disabled start/stop: 5/5 PASS
- Sensor container shutdown: clean in all five accepted cycles
- Residual Autoware/Hesai/OEM7 process after each accepted stop: none
- Physical actuation writer/CAN interface during the staged run: none observed

## Not yet accepted

Live OEM7 is blocked by the current user's missing `dialout` membership and by the need to approve receiver log reconfiguration. Live Hesai settings, `launch_hw=true` teardown, `/autoware_orientation`, localization/map alignment, GPU operation, and four-PC communication remain unverified. These gates are intentionally not bypassed by the hardware-disabled result.
