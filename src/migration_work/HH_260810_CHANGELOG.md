# HH_260810 PC3 changelog

> **Historical snapshot:** this changelog records the 2026-08-10 stationary
> stage. Current v1.1 behavior, validation, and remaining gates are documented
> in [`reports/PC3_v1.1_detailed_changes.md`](reports/PC3_v1.1_detailed_changes.md),
> which supersedes this document where the two records differ. The original
> entries below are preserved for audit and rollback history.

Audit date: 2026-08-10 (Asia/Seoul)
Scope: PC3 stationary, no-actuation bring-up
Runtime status: hardware-disabled stage PASS; live sensors, localization, and four-PC integration remain gated

The `run_autoware` alias and the active `autoware.launch.xml` path/name were not changed. Git branch, commit, and dirty-state provenance are UNVERIFIED because neither active workspace contains Git metadata.

## Change records

| PC | File and line/block | `HH_260810` comment text | Previous behavior | New behavior | Reason | Validation result | Rollback method |
|---|---|---|---|---|---|---|---|
| PC3 | `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:8-111` | “Used the sensor-owned container as PC3's only pointcloud container”; “Kept sensor hardware access disabled…”; “Disabled localization…”; “Kept perception, planning, control, and API ownership off PC3”; “Disabled the actuator-capable vehicle interface…”; “Disabled PC3 MRM command publishers…”; RViz/respawn disabled; generic container gated; MRM and localization-container arguments forwarded | Hardware drivers, vehicle interface, localization, generic pointcloud container, and RViz defaulted on; MRM groups were not independently gated | System, map, vehicle description, and common sensing remain on; hardware access, localization, vehicle interface, MRM commands, perception/planning/control/API, generic container, and RViz default off | Enforce PC3 ownership, one pointcloud container, and stationary no-actuation defaults | XML/package tests PASS; launch argument parsing PASS; five hardware-disabled alias cycles PASS | Restore this exact pre-change file from `migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/.../autoware.launch.xml`, rebuild `autoware_launch` |
| PC3 | `launcher/.../components/tier4_system_component.launch.xml:5-13` | “Added an explicit MRM gate…”; “Forwarded the MRM gate…” | Shared system component had no top-level MRM opt-out | Backward-compatible `launch_mrm` argument is forwarded | Retain diagnostics without PC3 command publishers | XML/package tests PASS; `/system/emergency/control_cmd` had zero publishers in staged runtime | Restore dated pre-change snapshot, rebuild `autoware_launch` |
| PC3 | `universe/.../tier4_system_launch/launch/system.launch.xml:21-115` | “Added an opt-out gate…”; comfortable-stop, emergency-stop, and MRM-handler gate comments | MRM operators and handler always launched with system services | The three command-producing MRM groups obey `launch_mrm`; default remains true for other callers | Avoid changing shared behavior while allowing PC3 no-actuation mode | XML/package tests PASS; no MRM process or command publisher in staged runtime | Restore dated pre-change snapshot, rebuild `tier4_system_launch` |
| PC3 | `sensor_kit/.../sample_sensor_kit_launch/launch/lidar.launch.xml:26-41` | Driver-gate propagation, verified host-address argument reuse, and container-substitution correction comments | Driver was hardcoded on; host address was duplicated; container name was passed as a literal string | Top hardware gate reaches Nebula, host argument is reused, and one resolved sensor-owned container is created | Prevent unintended LiDAR access and duplicate/misnamed containers | XML/package tests PASS; one sensor container; “Hardware connection disabled” in all accepted cycles | Restore dated pre-change snapshot, rebuild `sample_sensor_kit_launch` |
| PC3 | `sensor_kit/.../sample_sensor_kit_launch/launch/gnss.launch.xml:5-36` | Stable NovAtel identity, Boolean hardware gate, and stable port/baud forwarding comments | `/dev/ttyUSB1` was enumeration-dependent; string-expression conditions could still select a receiver under a false hardware gate | Default receiver remains NovAtel; stable by-id port1 and historical 115200 baud are forwarded only inside the Boolean driver gate | Restore the established PC3 OEM7 path without opening or reconfiguring the receiver during dry validation | XML/package tests PASS; OEM7 build/install and launch parsing PASS; OEM7 CTest 4/6 with two pre-existing `gps` versus `gnss_link` reference mismatches; no driver process or serial holder in staged runtime | Restore dated pre-change snapshot, rebuild `sample_sensor_kit_launch`; restored OEM7 source may remain installed but unused |
| PC3 | `launcher/.../config/map/map_tf_generator.param.yaml:4-5` | “Restored the viewer frame so localization remains the sole map-to-base_link authority.” | Static map TF generator targeted `base_link`, conflicting with EKF's dynamic map-to-base_link TF when localization runs | Static transform is `map -> viewer`; localization is reserved as the future dynamic `map -> base_link` authority | Remove deterministic TF authority conflict | YAML parse PASS; staged `map -> viewer` verified; `map -> base_link` absent while localization is off | Restore dated pre-change snapshot, rebuild `autoware_launch` |
| PC3 | `universe/.../autoware_gnss_poser/src/gnss_poser_node.cpp:179-199` | Existing C_track translation applied once; translated shared pose copied without a second offset | Pose and TF used unshifted coordinates while only covariance output received the C_track x/y shift | The existing shift is applied once to the shared pose used by pose, covariance, and TF | Keep GNSS outputs internally consistent | Build, clang-format, and package tests PASS; live GNSS/map alignment remains UNVERIFIED | Restore dated pre-change snapshot, rebuild `autoware_gnss_poser` |
| PC3 | `sensor_component/.../hesai_ros_wrapper.hpp:56-57`; `.../hesai_ros_wrapper.cpp:72-113` | Explicit decoder-thread shutdown, null sentinel, producer stop, and join comments | Wrapper destruction left a joinable decoder thread blocked on a queue and deterministically called `std::terminate` | Destructor stops producers, enqueues a null sentinel, and joins the decoder thread | Make hardware-disabled component-container shutdown clean and repeatable | `nebula_ros` build/format PASS; five accepted cycles exit 0 without SIGABRT or orphans | Restore the dated pre-change header/source, rebuild `nebula_ros`; this intentionally restores the old abort defect |
| PC3 | `/home/a/ros2_ws/src/novatel_oem7_driver` (restored tree; no source edits) | N/A — byte-for-byte restoration, so no source comment was inserted | OEM7 source/install prefixes were absent from the recreated Hesai-only workspace | 137 archived files restored from `/home/a/ros2_ws_original.zip`; `novatel_oem7_msgs` and `novatel_oem7_driver` built successfully | PC3 historically and currently targets NovAtel OEM7 | Archive/source comparison: zero mismatches; targeted build PASS; package tests recorded separately; hardware runtime intentionally not run | Preserve archive; remove restored/build/install trees only through the operator-approved rollback procedure if required |

## Preservation record

- Immediate pre-change snapshot: `migration_work/backups/PC3/pre_HH_260810/`
- Pre-change checksum manifest: `migration_work/backups/PC3/pre_HH_260810/SHA256SUMS` — validation PASS
- Final active and same-directory backup manifest: `migration_work/backups/PC3/post_HH_260810/SHA256SUMS`
- Existing `(copy_org)` files were never overwritten. Newly required `(copy_org)` files were created before their active counterparts were edited and are byte-identical to the immediate pre-change content.
- The dated snapshot, not an older historical `(copy_org)`, is the authoritative immediate rollback source when their contents differ.

## Remaining gates

- Add user `a` to `dialout` and start a new login session before any serial test.
- Approve a maintenance window before launching OEM7 because its startup issues `UNLOGALL THISPORT` and a new LOG command set.
- Validate `/autoware_orientation`, current map alignment, vehicle twist, LiDAR model/UDP destination/calibration, and live TF authority before enabling localization.
- Validate Nebula shutdown with `launch_hw=true`; the five-cycle acceptance covers `launch_hw=false` only.
- Repair and verify the NVIDIA driver before GPU-dependent nodes.
- Complete PC1, PC2, PC4, bridge, Chrony-client, and 10-minute stationary integration checks.
