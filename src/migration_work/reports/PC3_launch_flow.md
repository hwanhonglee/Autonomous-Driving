# PC3 Active Launch Flow and NovAtel Provenance

Audit date: 2026-08-10 KST
Scope: static recursive trace of the launch selected by the unchanged PC3 `run_autoware` alias, plus the restored NovAtel source/build provenance. This report task did not edit or start any launch/source file.

## Evidence labels

- **VERIFIED**: directly observed in current source, the active install/package index, a manifest/checksum, or build logs.
- **INFERRED**: deterministic from verified launch substitutions/conditions, but not demonstrated by starting the graph.
- **UNVERIFIED**: requires an authorized runtime test or unavailable repository provenance.

## Snapshot boundary

The pre-HH_260810 launch snapshot is rooted at:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810
```

The pre-patch top launch has SHA-256 `caf7cfe61dca710234b20217577e4f98e6affd898c2cfbf646e554c7ceb29eff`, recorded at `SHA256SUMS:7`. It defaulted the sensing driver, localization, perception, planning, control, API, vehicle interface, RViz, and RViz respawn ON, and it unconditionally created a generic pointcloud container:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:19-24
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:29
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:41-43
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:59-63
```

The root-owned safety patch was applied during this audit. The post-patch active source is:

```text
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```

Its SHA-256 at inspection was `1563e7d713328e3d10bef8a6a9aa9e6cb4de31fb49f5113454b2a97af25fa5a2`. The installed file is a symlink to this source, so it is the launch data resolved by `ros2 launch`. This is **VERIFIED static state**; post-patch runtime validation remains **UNVERIFIED**.

Targeted post-patch builds completed with `rc: 0` for `common_sensor_launch`, `tier4_system_launch`, `autoware_launch`, `autoware_gnss_poser`, and `sample_sensor_kit_launch` at `/home/a/autoware/log/build_2026-08-10_19-48-35/events.log:449,455,757,994,1057`; a follow-up `nebula_ros` build completed with `rc: 0` at `/home/a/autoware/log/build_2026-08-10_19-54-04/events.log:1520`. These are targeted build proofs, not a clean full-workspace build or stationary-test PASS.

## Default post-patch graph

The unchanged alias explicitly supplies only the map path, `sample_vehicle`, and `sample_sensor_kit`. The active defaults yield this static graph:

```text
run_autoware
└── autoware_launch/autoware.launch.xml
    ├── global_parameter_loader/global_params.launch.py                 ON
    ├── generic pointcloud_container.launch.py                          OFF
    ├── tier4_vehicle_launch/vehicle.launch.xml                         ON
    │   ├── robot_state_publisher                                       ON
    │   └── sample_vehicle vehicle_interface.launch.xml                 OFF
    ├── tier4_system_launch/system.launch.xml                           ON
    │   ├── monitoring, duplicate/process checks, diagnostics           ON
    │   └── comfortable stop, emergency stop, MRM handler               OFF
    ├── tier4_map_launch/map.launch.xml                                 ON
    ├── tier4_sensing_launch/sensing.launch.xml                         ON
    │   └── sample_sensor_kit_launch/sensing.launch.xml
    │       ├── LiDAR Nebula container + pointcloud preprocessing       ON
    │       │   └── Nebula hardware access (`launch_hw`)                OFF
    │       ├── IMU correction/bias processing                          ON
    │       ├── selected NovAtel/u-blox hardware driver                 OFF
    │       ├── GNSS poser                                              ON
    │       └── vehicle velocity converter                              ON
    ├── localization                                                    OFF
    ├── perception / planning / control / API                           NOT INCLUDED
    └── RViz                                                            OFF
```

The controlling defaults are at active top-launch lines 9, 17-29, 35, 43-53. Vehicle, system, map, and sensing remain true; generic pointcloud container, sensing hardware drivers, localization, vehicle interface, MRM publishers, and RViz remain false.

## Recursive include evidence

### Global parameters

The top includes `global_parameter_loader/launch/global_params.launch.py` at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:61-67`. The Python launch sets global `use_sim_time` and includes vehicle information at `/home/a/autoware/src/universe/autoware.universe/common/global_parameter_loader/launch/global_params.launch.py:25-46`.

### Pointcloud container ownership

The generic top container is gated off at active top-launch lines 69-76. Sensing instead creates the one intended PC3 container:

- `/home/a/autoware/src/universe/autoware.universe/launch/tier4_sensing_launch/launch/sensing.launch.xml:9-17` pushes namespace `sensing` and selects `sample_sensor_kit_launch`.
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml:10-25` adds `lidar/top` and includes the Hesai/Nebula path.
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml:40-41` now passes `$(var pointcloud_container_name)` rather than the former literal `pointcloud_container_name`.
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py:244-252` creates a container named from that argument in namespace `pointcloud_preprocessor`.

**INFERRED FQN with defaults:**

```text
/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container
```

The Nebula container loads the Hesai wrapper and self/mirror crop, distortion-correction, and ring-outlier components at `nebula_node_container.launch.py:102-242`. The wrapper receives the hardware gate as `launch_hw` at lines 112-122. With `launch_sensing_driver=false`, the component is still constructed but hardware access is intended to remain disabled.

**UNVERIFIED:** actual one-container count, component membership, absence of UDP access, output topics/rates/QoS, failure behavior, and cleanup.

### Vehicle description without interface

The top forwards `launch_vehicle_interface=false` through `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:78-88`. The shared vehicle launch always creates `robot_state_publisher` at `/home/a/autoware/src/universe/autoware.universe/launch/tier4_vehicle_launch/launch/vehicle.launch.xml:13-19`, but its interface include is conditional at lines 21-28. The selected sample interface contains only a `vehicle_id` argument and no nodes at `/home/a/autoware/src/vehicle/sample_vehicle_launch/sample_vehicle_launch/launch/vehicle_interface.launch.xml:1-4`.

### System and MRM gate

The top includes the system component and forwards `launch_mrm=false` at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:90-96`. The component forwards it into the shared launch at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml:6-14`.

With current defaults, the shared system launch retains:

- system monitor, `/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:42-54`;
- duplicate-node checker, processing-time checker, service-log checker, and component-state monitor, lines 57-81;
- system diagnostic monitor and hazard-status converter, lines 98-108.

It gates comfortable-stop and emergency-stop operators at lines 85-95 and the MRM handler at lines 112-116. `launch_dummy_diag_publisher=false` also suppresses lines 120-129.

### Map

The top map include is at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:98-101`; the wrapper supplies C-track map paths at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_map_component.launch.xml:3-12`.

The map launch creates `/map/map_container` with pointcloud-map loader, Lanelet2 loader, Lanelet2 visualization, and vector-map TF generator at `/home/a/autoware/src/universe/autoware.universe/launch/tier4_map_launch/launch/map.launch.xml:19-48`, plus map hash and projection loaders at lines 50-59.

Map file presence and hardware-disabled staged publication are **VERIFIED**: vector and pointcloud maps each had one publisher, and `map -> viewer` was present without `map -> base_link`. Semantic map validity, live GNSS alignment, and localization-time TF authority remain **UNVERIFIED**.

### Sensing, LiDAR, IMU, and GNSS

The top sensing include at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:103-106` forwards `launch_sensing_driver` through:

```text
autoware_launch/.../tier4_sensing_component.launch.xml:3-9
  -> tier4_sensing_launch/launch/sensing.launch.xml:9-17
  -> sample_sensor_kit_launch/launch/sensing.launch.xml:7-34
```

The sample sensor launch branches to LiDAR, IMU, GNSS, and the vehicle-velocity converter at `sample_sensor_kit_launch/launch/sensing.launch.xml:8-34`.

LiDAR follows:

```text
sample_sensor_kit_launch/launch/lidar.launch.xml:23-46
  -> common_sensor_launch/launch/hesai_Pandar_64.launch.xml:30-48
  -> common_sensor_launch/launch/nebula_node_container.launch.py
```

The propagated gate is visible at `lidar.launch.xml:26-27`, `hesai_Pandar_64.launch.xml:30-32`, and `nebula_node_container.launch.py:121`. Current static parameters remain Pandar64, sensor IP `192.168.2.101`, host IP `192.168.2.150`, port `2368`, and frame `hesai_top` at `lidar.launch.xml:4,28-36`. They must not be interpreted as live endpoint proof.

The IMU launch does not start a separate hardware driver; it always includes IMU corrector and gyro-bias estimator at `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/imu.launch.xml:4-19`, consuming NovAtel raw IMU by default.

The GNSS launch selects NovAtel and declares the stable path and baud at `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml:4-7`. The by-id path currently resolves to `/dev/ttyUSB1`. Both u-blox and NovAtel hardware includes are nested below `launch_driver` at lines 18-38, so neither is included with the top default false. The GNSS poser remains unconditional at lines 40-50 and will wait for its inputs.

### Localization dormant branch

Localization is gated off at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:108-114`. If explicitly enabled later, the top forwards the sensor-owned container FQN at lines 110-113; the wrapper selects NDT, gyro odometry, and `/sensing/lidar/top/mirror_cropped/pointcloud_ex` at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml:4-16`; the shared launch then includes pose/twist estimation, fusion, and error monitoring at `/home/a/autoware/src/universe/autoware.universe/launch/tier4_localization_launch/launch/localization.launch.xml:31-48`.

Localization remains **UNVERIFIED and intentionally OFF** pending pointcloud, velocity, IMU/orientation, map, initialization, and TF validation.

### Modules absent from PC3 top launch

The current file ends after localization and RViz at line 130. It contains no perception, planning, control, or API include blocks. Their false arguments at lines 26-29 are defense in depth, not the only ownership boundary.

## Separate ROS-driver entry points

The live driver workspace contains three packages: `hesai_ros_driver`, `novatel_oem7_driver`, and `novatel_oem7_msgs`.

The independent manual Hesai entry point is `/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py:5-8`; it starts `hesai_ros_driver_node`. Its historical settings are in `/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml:4-11,33-45`. It is not part of the active Autoware path: the corresponding include is commented at `sample_sensor_kit_launch/launch/lidar.launch.xml:15-19`, while the active path uses Nebula. Shell-history entries 194, 199, 203, 518, 546, 645, 654, 733, and 2082 show past manual starts.

The NovAtel launch can likewise be started manually; normalized shell-history entries 173 and 210 used `oem7_port.launch.py`, `/dev/ttyUSB1`, and 115200. The active Autoware include is the conditional path at `gnss.launch.xml:30-38`. A manual driver must not be started concurrently when that gate is later enabled.

The u-blox source tree is absent from live `/home/a/ros2_ws`, although `/opt/ros/humble/share/ublox_gps` exists. The current receiver selection is NovAtel and all hardware drivers default off.

## NovAtel provenance, restoration, and build

### Source archive

Primary recovery archive:

```text
/home/a/ros2_ws_original.zip
SHA-256: fb9ff84cf4f89f25c4790bfd3133c3ba48b901d357fb4d86778b256e90b3e923
```

`unzip -tq` passes. The exact archived repository prefix is:

```text
ros2_ws/src/novatel_oem7_driver/
```

with package roots:

```text
ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/
ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_msgs/
```

An independent archive, `/home/a/Downloads/Autonomous-Driving-IONIQ_EV_307_PC3_r.zip` (SHA-256 `ea3f533e16e25c2675181bf2364c451e627739be62ddf0e8b0fe16e98c1e845f`), contains the same normalized 137-file NovAtel subtree. The normalized subtree SHA-256 is `8b01b0d2997176e61fc52d0227a1c5d31619266499d79b92528b9292314bca64` in both archives.

### Restored live source and byte verification

The repository was restored to:

```text
/home/a/ros2_ws/src/novatel_oem7_driver
```

A read-only name-set and per-file byte comparison against `/home/a/ros2_ws_original.zip` returned:

```text
ARCHIVE_FILES=137
LIVE_FILES=137
NAME_SET_DIFF=<empty>
BYTE_COMPARE_CHECKED=137
BYTE_MISMATCHES=0
```

Representative live/archive-identical SHA-256 values are:

```text
27daa715deed9ad98ef7bd51198d2b5d38c5c4aa50372f7bc6659228f3ac772c  novatel_oem7_driver/package.xml
55da924fb4a9f89be82c7f5514f6ad012b4cb1563fffcd8ff948b2e961a09d41  novatel_oem7_msgs/package.xml
8c40545d0b2bd4d51c0be0e3d2098a7a7a577fe9aac68bb867a99fb055e01981  launch/oem7_port.launch.py
2b68296878fff23a7cc07472b9ec4bc39fdae75edaa493840f4000e4d795db83  config/std_msg_topics.yaml
```

The locally preserved `std_msg_topics.yaml` uses `gnss_link` for GPSFix, NavSatFix, and INSSTDEV; restoring a generic upstream checkout would lose that vehicle-specific state.

### Build/install proof

Both packages identify as version `20.6.0` at:

```text
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/package.xml:3-26
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_msgs/package.xml:3-25
```

The restored build completed successfully on 2026-08-10. `/home/a/ros2_ws/log/build_2026-08-10_19-38-08/events.log:2291` records `novatel_oem7_msgs` `rc: 0`; line 3564 records `novatel_oem7_driver` `rc: 0`. Build and install roots now exist at:

```text
/home/a/ros2_ws/build/novatel_oem7_msgs
/home/a/ros2_ws/build/novatel_oem7_driver
/home/a/ros2_ws/install/novatel_oem7_msgs
/home/a/ros2_ws/install/novatel_oem7_driver
```

After sourcing the normal shell chain, package resolution returns:

```text
/home/a/ros2_ws/install/novatel_oem7_msgs/share/novatel_oem7_msgs
/home/a/ros2_ws/install/novatel_oem7_driver/share/novatel_oem7_driver
```

The driver launch creates `novatel/oem7/main`, executable `novatel_oem7_driver_exe`, and consumes the forwarded port/baud at `/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/launch/oem7_port.launch.py:39-81`.

The archived source has no `.git` metadata. Its branch/commit and dirty state remain **UNVERIFIED**. The build did successfully fetch/build the external NovAtel EDIE dependency declared at `/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/CMakeLists.txt:45-58`, but the generated build clone is not provenance for the restored driver repository itself.

## Runtime validation result and remaining gates

The hardware-disabled stage is **VERIFIED PASS**. Five accepted exact-alias cycles proved no OEM7 process/serial holder or Hesai hardware listener, one sensor-owned pointcloud container, no manual duplicate driver, system monitors present with MRM groups absent, no vehicle interface or staged command-topic publisher, map publication with `map -> viewer`, clean scoped Ctrl+C, and no orphan process. The accepted logs are enumerated in `PC3_start_stop_stability.md`.

Still required before enabling the next stage:

- approve and validate live NovAtel/Hesai hardware access and exclusive ownership;
- measure GNSS/IMU/orientation/velocity/pointcloud topics and QoS/rates;
- validate map alignment and the sole dynamic `map -> base_link` authority before localization;
- validate Nebula teardown with `launch_hw=true`;
- prove cross-PC duplicate and no-actuation conditions; and
- complete the four-PC ten-minute stationary integration run.
