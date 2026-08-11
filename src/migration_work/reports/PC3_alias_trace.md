# PC3 `run_autoware` Alias and Environment Trace

Audit date: 2026-08-10 KST
Scope: static/read-only trace of the active PC3 shell alias through environment setup, ROS package resolution, and the selected top launch. No alias, shell setup, launch, or source file was edited by this report task.

## Evidence labels

- **VERIFIED**: directly observed in a current file, package index, symlink, checksum, or command output.
- **INFERRED**: follows from verified shell/ROS semantics but was not exercised by starting Autoware.
- **UNVERIFIED**: requires an authorized launch/runtime or provenance source that is not present.

## Result

**VERIFIED:** `run_autoware` is one Bash alias defined at `/home/a/.bashrc:195`; it is not a function or wrapper script. Its exact value is:

```bash
ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/Autoware_Map/C_track vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

The read-only check `bash -ic 'type -a run_autoware; alias run_autoware'` returned only that alias. `/home/a/.bash_aliases`, `/home/a/.bash_profile`, and `/home/a/.bash_login` are absent. A historical `/home/a/.bashrc1_txt:162` contains another alias value, but no active startup file sources it.

**INFERRED:** in the audited interactive shell, `$HOME` expands to `/home/a`, so the effective explicit launch arguments are:

```text
map_path:=/home/a/Autoware_Map/C_track
vehicle_model:=sample_vehicle
sensor_model:=sample_sensor_kit
```

The alias supplies no other launch arguments. Safety therefore depends on the active top launch defaults described below.

## Complete shell-to-launch chain

| Stage | Status | Exact evidence and effect |
|---|---|---|
| Login startup | VERIFIED | `/home/a/.profile:11-16` sources `/home/a/.bashrc` for Bash. No `.bash_profile` or `.bash_login` exists to bypass `.profile`. |
| Interactive guard | VERIFIED | `/home/a/.bashrc:7-12` returns before setup/aliases for non-interactive Bash. `run_autoware` is therefore an interactive-shell alias unless another caller explicitly sources the file in an interactive context. |
| ROS underlay | VERIFIED | `/home/a/.bashrc:153-155` sources `/opt/ros/humble/setup.bash`. |
| Autoware overlay | VERIFIED | `/home/a/.bashrc:156-157` sources `/home/a/autoware/install/setup.bash`. |
| Sensor/driver overlay | VERIFIED | `/home/a/.bashrc:158-159` then sources `/home/a/ros2_ws/install/setup.bash`, making that workspace the final overlay. |
| ROS network environment | VERIFIED | `/home/a/.bashrc:167-170` sets `ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=10`, and console formatting; `/home/a/.bashrc:179-184` selects `rmw_cyclonedds_cpp`. An interactive readback returned the same values. |
| Host network mutation on first shell | VERIFIED | `/home/a/.bashrc:171-177` can run privileged sysctl/link changes and create `/tmp/cycloneDDS_configured` when the marker is absent. This occurs while opening the shell, before the alias, and is outside ROS launch. |
| Alias definition | VERIFIED | `/home/a/.bashrc:193-195` defines the exact direct `ros2 launch` command above. |
| ROS package selection | VERIFIED | After sourcing the same overlays, `ros2 pkg prefix --share autoware_launch` returned `/home/a/autoware/install/autoware_launch/share/autoware_launch`. |
| Installed launch file | VERIFIED | `/home/a/autoware/install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml` is a symlink to `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`. The source file is therefore the active launch data. |
| Top launch | VERIFIED static and hardware-disabled runtime | The post-HH_260810 source was invoked through the exact unchanged alias in five accepted cycles; all five exited 0 without an orphan process. |

There is no script between the alias and `ros2 launch`, no duplicate-start guard in the alias, and no alias-specific shutdown handler.

## Explicit arguments and inherited defaults

The map directory exists and contains the filenames selected by the top launch: `/home/a/Autoware_Map/C_track/lanelet2_map.osm`, `pointcloud_map.pcd`, and `map_projector_info.yaml`. The top launch selects those filenames at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:39-40`.

`VEHICLE_ID` was unset in the audited interactive shell. The current top launch therefore falls back to `vehicle_id=default` through `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:33` unless the operator exports it before invoking the alias.

### Pre-patch snapshot

The manifest-backed pre-change launch is:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```

Its SHA-256 is `caf7cfe61dca710234b20217577e4f98e6affd898c2cfbf646e554c7ceb29eff`, recorded in `/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/SHA256SUMS:7`.

**VERIFIED pre-patch:** sensing driver, localization, perception, planning, control, and API defaulted true at snapshot lines 19-24; vehicle interface defaulted true at line 29; RViz and respawn defaulted true at lines 41-43; and the generic pointcloud container was unconditional at lines 59-63.

### Post-patch active source

The root-owned HH_260810 safety patch was applied during this audit; this report did not edit it. The active source SHA-256 at inspection was `1563e7d713328e3d10bef8a6a9aa9e6cb4de31fb49f5113454b2a97af25fa5a2`.

| Argument | Current default | Evidence | Static consequence of invoking the unchanged alias |
|---|---:|---|---|
| `launch_vehicle` | `true` | active top launch `:17` | Includes vehicle description/TF path. |
| `launch_system` | `true` | `:18` | Includes system monitoring/diagnostics path. |
| `launch_map` | `true` | `:19` | Includes map loaders. |
| `launch_sensing` | `true` | `:20` | Includes sensing processing. |
| `launch_pointcloud_container` | `false` | `:9` | Suppresses the generic top-level pointcloud container. |
| `launch_sensing_driver` | `false` | `:22` | Propagates the hardware-driver-off gate into sensing. |
| `launch_localization` | `false` | `:24` | Suppresses localization by default. |
| `launch_perception`, `launch_planning`, `launch_control`, `launch_api` | `false` | `:26-29` | Defense-in-depth defaults; the PC3-trimmed top file has no include blocks for these four modules. |
| `launch_vehicle_interface` | `false` | `:35` | Keeps robot description but suppresses the conditional vehicle interface include. |
| `launch_mrm` | `false` | `:46`, forwarded at `:90-95` | Suppresses the three MRM command-producing groups. |
| `rviz`, `rviz_respawn` | `false` | `:50-53` | Suppresses PC3 RViz and its respawn. |

**VERIFIED static:** the symlinked install resolves to these values.
**VERIFIED hardware-disabled runtime:** launch parsing passed; five exact-alias cycles confirmed one sensor-owned pointcloud container, no generic pointcloud container, no NovAtel process/serial holder or Hesai hardware listener, no vehicle-interface/MRM/CAN-writer/RViz process, zero staged command-topic publishers, clean Ctrl+C, and no orphan process. This does not validate live sensor hardware or localization.

## Workspace, package, and Git status

Current roots are:

```text
Autoware: /home/a/autoware
ROS driver workspace: /home/a/ros2_ws
```

`colcon list` resolves the relevant Autoware packages from `/home/a/autoware/src`, including `autoware_launch`, `global_parameter_loader`, `tier4_{vehicle,system,map,sensing,localization}_launch`, `sample_sensor_kit_launch`, and `common_sensor_launch`. The ROS driver workspace currently resolves:

```text
hesai_ros_driver       src/HesaiLidar_ROS_2.0
novatel_oem7_driver   src/novatel_oem7_driver/src/novatel_oem7_driver
novatel_oem7_msgs     src/novatel_oem7_driver/src/novatel_oem7_msgs
```

**VERIFIED targeted build status:** `/home/a/autoware/log/build_2026-08-10_19-48-35/events.log:449,455,757,994,1057` records `rc: 0` for `common_sensor_launch`, `tier4_system_launch`, `autoware_launch`, `autoware_gnss_poser`, and `sample_sensor_kit_launch`. A follow-up Nebula build also ended `rc: 0` at `/home/a/autoware/log/build_2026-08-10_19-54-04/events.log:1520`. These targeted successes do not constitute a clean full-workspace build or runtime acceptance.

**VERIFIED:** neither `/home/a/autoware` nor `/home/a/ros2_ws` is a top-level Git repository; `git -C <root> status --short --branch` returns “not a git repository.” No `.git` metadata exists in the audited target source trees for `autoware_launch`, the PC3 launch packages, `sample_sensor_kit_launch`, Hesai, or the restored NovAtel source. `/home/a/autoware/autoware.repos:83-86` declares the desired `autoware_launch` source as Git `main`, but that manifest is not checkout proof.

An unrelated nested Eagleye repository and a generated NovAtel decoder clone under `build/` exist; neither supplies provenance for the audited top launch or restored driver source.

Consequently, these values are **UNVERIFIED** for the target trees:

- Git branch;
- source commit;
- Git dirty/clean state;
- whether local HH changes descend from the manifest's declared revision.

Checksums and the dated backup manifest, not Git, are the available rollback/provenance anchors.

## Separate state-changing helper outside the alias

`/home/a/scripts/start.sh` is not referenced by `run_autoware`. It independently sources Humble at line 1, configures and raises `can0`/`can1` at 500 kbit/s at lines 3-6, and changes tty/video/Docker-socket permissions at lines 8-10. It is therefore outside the verified alias chain and must not be treated as a required setup step for the stationary test.

## Remaining unknowns

- **UNVERIFIED:** whether every operator starts a login/interactive shell that reads this exact startup chain.
- **UNVERIFIED:** whether another service, terminal tab, or historical manual command starts a second sensor driver before the alias.
- **UNVERIFIED:** live cross-PC `ROS_DOMAIN_ID`, RMW, QoS compatibility, and duplicate publishers on PC1/PC2/PC4.
- **UNVERIFIED:** the live-sensor and localization graph, OEM7 serial data, Hesai packet/pointcloud output, `launch_hw=true` teardown, and cross-PC actuator-path isolation. The five-cycle result covers only the hardware-disabled PC3 stage.
