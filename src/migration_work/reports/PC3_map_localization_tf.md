# PC3 Map, Localization, and TF Audit

Audit date: 2026-08-10 KST
PC role: system, sensing, map, localization, TF, and foundational services
Audit method: initial read-only source/config and stopped-graph inspection, followed by an authorized rebuild and five exact-alias, hardware-disabled start/stop cycles. Localization and physical sensor access remained disabled.

## Result

| Scope | Result | Meaning |
|---|---|---|
| Current stopped runtime | PASS-at-rest | No persistent Autoware graph or TF publishers were observed. |
| Pre-patch launch-time map/localization/TF | BLOCKED | Static and historical evidence shows a duplicate `map -> base_link` authority, inconsistent GNSS pose products, missing localization inputs, and an unvalidated C_track alignment. |
| HH_260810 patch | APPLIED / BUILT | Localization defaults OFF, vector-map TF is `map -> viewer`, and the preserved C_track translation is applied once to the shared GNSS pose before pose, covariance pose, and TF are produced. |
| Hardware-disabled post-patch runtime | PASS, 5/5 | Every stable exact-alias cycle loaded the map, broadcast `map -> viewer`, exited with launcher code 0, and shut down the map and sensing containers cleanly without orphans. |
| Live localization and sensor alignment | NOT ACCEPTED | GNSS orientation, translated-pose equality/alignment, topic rates/QoS, NDT/EKF activation, and the sole live `map -> base_link` authority remain unproven because localization and sensor hardware stayed OFF. |

The immutable pre-change evidence snapshot is rooted at:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810
```

Line numbers described as pre-patch refer to that snapshot or to the active files before the HH_260810 patch.

## Evidence classes

- **Current runtime evidence** describes only the stopped system inspected on 2026-08-10.
- **Static evidence** proves what the launch/config/source will attempt, not that it is currently running.
- **Historical runtime evidence** proves that a path ran at the stated log date; it does not prove current hardware or current graph state.
- **Post-patch runtime evidence** proves only the exact hardware-disabled alias and readiness boundary used in the five stable cycles; it does not extend to localization, sensor hardware, or the four-PC graph.

## Current runtime evidence

- Environment observed: ROS 2 Humble, `ROS_DOMAIN_ID=10`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- `ros2 daemon status` reported that the daemon was not running before and after discovery.
- Safe `--no-daemon --spin-time 2` discovery found only the transient ROS CLI node and CLI-created `/parameter_events` and `/rosout` endpoints.
- No persistent Autoware, map, localization, TF, RViz, Hesai, NovAtel, u-blox, or component-container process was found.
- Therefore no live map topics, localization output, QoS contract, rate, TF tree, or TF authority could be measured.

This is a PASS only for the current stopped state.

## Map inputs and loaders

The effective alias selects:

```text
/home/a/Autoware_Map/C_track
```

The directory contains:

- `lanelet2_map.osm`;
- `pointcloud_map.pcd`;
- `map_projector_info.yaml`.

`pointcloud_map_metadata.yaml` is absent. For one PCD this is not necessarily fatal: the pointcloud map loader synthesizes metadata for a single PCD at:

```text
/home/a/autoware/src/universe/autoware.universe/map/map_loader/src/pointcloud_map_loader/pointcloud_map_loader_node.cpp:80
/home/a/autoware/src/universe/autoware.universe/map/map_loader/src/pointcloud_map_loader/pointcloud_map_loader_node.cpp:106
```

The active map launch loads the pointcloud map, Lanelet2 map, visualization, vector-map TF generator, map hash, and projection loader at:

```text
/home/a/autoware/src/universe/autoware.universe/launch/tier4_map_launch/launch/map.launch.xml:22
/home/a/autoware/src/universe/autoware.universe/launch/tier4_map_launch/launch/map.launch.xml:59
```

No map-validator include is present in that active launch. File presence and successful parsing therefore must not be reported as semantic map validation.

Historical evidence that the Lanelet2 map loaded and was published exists at:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:276
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:280
```

That log is evidence for the 2026-05-27 run only.

## Pre-patch TF conflict

Pre-patch map TF configuration selected `base_link` as the vector-map viewer frame:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml:3
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml:4
```

The vector-map TF source publishes a static transform from `map_frame` to `viewer_frame`:

```text
/home/a/autoware/src/universe/autoware.universe/map/autoware_map_tf_generator/src/vector_map_tf_generator_node.cpp:76
/home/a/autoware/src/universe/autoware.universe/map/autoware_map_tf_generator/src/vector_map_tf_generator_node.cpp:92
```

The EKF configuration also enables TF publication at 50 Hz in the map frame:

```text
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml:6
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml:8
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/localization/ekf_localizer.param.yaml:51
```

The EKF source publishes dynamic `map -> base_link` after activation:

```text
/home/a/autoware/src/universe/autoware.universe/localization/autoware_ekf_localizer/src/ekf_localizer.cpp:64
/home/a/autoware/src/universe/autoware.universe/localization/autoware_ekf_localizer/src/ekf_localizer.cpp:233
/home/a/autoware/src/universe/autoware.universe/localization/autoware_ekf_localizer/src/ekf_localizer.cpp:249
```

Historical runtime proof of the static conflicting edge is explicit:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:279
```

It reports a static `map -> base_link` broadcast. Once EKF activates, the same parent/child pair has two authorities.

### Applied HH_260810 resolution

- `viewer_frame: viewer` is active at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml:5`.
- EKF remains the sole intended `map -> base_link` authority when localization is later enabled.
- Localization defaults OFF at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:24` until a live TF audit proves there is exactly one authority.

All five hardware-disabled runs logged the static `map -> viewer` broadcast. They do not prove the EKF edge because localization remained OFF.

## Pre-patch GNSS pose inconsistency

The pre-patch GNSS poser creates the base pose in the map frame and publishes it without the C_track translation:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:173
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:180
```

It then copies that pose into `PoseWithCovarianceStamped` and applies the translation only to the copy:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:184
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:191
```

The effective translation is:

```text
x := x - 60953.77526469596 - 16.0
y := y - 65983.92232890474 + 17.0
z := z
```

Finally, the pre-patch TF uses the untranslated base pose:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:213
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:216
```

Thus `/sensing/gnss/pose`, `/sensing/gnss/pose_with_covariance`, and the GNSS TF cannot describe the same map-frame pose.

### Applied HH_260810 C_track policy

The C_track translation is intentionally preserved and is now applied exactly once to the shared `gnss_base_pose_msg` before any of these consumers:

1. publish `PoseStamped`;
2. copy the pose into `PoseWithCovarianceStamped`;
3. broadcast the GNSS-derived TF.

The source invariant is:

```text
pose.position == pose_with_covariance.pose.pose.position == GNSS TF translation
```

This removes the internal inconsistency without claiming that the translation is correct for the current map. Live alignment against `/home/a/Autoware_Map/C_track/map_projector_info.yaml`, Lanelet2, and the PCD remains a mandatory gate.

Active source evidence is:

```text
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:173
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:180
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:187
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:194
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp:219
```

The untouched `gnss_poser_node (copy_org).cpp` has no track translation, while `gnss_poser_node (copy_C_track).cpp` contains different track-specific constants. Neither file alone proves which translation matches the current map.

## GNSS and orientation readiness

Current hardware enumeration identifies a NovAtel receiver with three stable by-id serial ports and a separate u-blox receiver. NovAtel is the selected owner.

The restored source packages are located at:

```text
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_msgs
```

The NovAtel launch creates `novatel/oem7/main`, accepts a port and baud, and publishes the configured topics at:

```text
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/launch/oem7_port.launch.py:39
/home/a/ros2_ws/src/novatel_oem7_driver/src/novatel_oem7_driver/launch/oem7_port.launch.py:80
```

Historical runtime evidence proves `/dev/ttyUSB1` opened at 115200, initialized the receiver, and detected an Epson G320N IMU:

```text
/home/a/.ros/log/novatel_oem7_driver_exe_2970_1779860034926.log:29
/home/a/.ros/log/novatel_oem7_driver_exe_2970_1779860034926.log:35
```

The same log shows NavSatFix on `fix`, frame `gnss_link`, and raw IMU on `imu/data_raw`, frame `imu_link`:

```text
/home/a/.ros/log/novatel_oem7_driver_exe_2970_1779860034926.log:12
/home/a/.ros/log/novatel_oem7_driver_exe_2970_1779860034926.log:15
```

A later launch failed because the package was not in the active install overlay at that time:

```text
/home/a/.ros/log/2026-07-16-15-56-28-403069-a-2855/launch.log:3
```

The hardware-disabled alias now starts successfully with the restored source available. This does not prove live NovAtel serial access, current device identity, data validity, or the hardware-enabled shutdown path.

The GNSS poser is configured to use `GnssInsOrientationStamped`:

```text
/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/config/gnss_poser.param.yaml:7
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml:13
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml:39
```

No local publisher for `/autoware_orientation` was found in source or the stopped graph. NovAtel's `sensor_msgs/Imu` output cannot be substituted by name-only remapping because the message types differ. Localization therefore remains OFF until a valid orientation source/adapter is implemented and observed.

## Localization input readiness

The PC3 wrapper selects NDT pose estimation, gyro odometry, and the Hesai mirror-cropped pointcloud:

```text
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml:4
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml:7
```

The LiDAR distortion corrector requires both vehicle twist and IMU:

```text
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py:209
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py:220
```

Gyro odometer likewise requires IMU and vehicle twist:

```text
/home/a/autoware/src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/gyro_odometer.launch.xml:4
/home/a/autoware/src/universe/autoware.universe/launch/tier4_localization_launch/launch/pose_twist_estimator/gyro_odometer.launch.xml:7
```

The sample vehicle interface is empty and supplies no `/vehicle/status/velocity_status`. Historical runtime evidence confirms the consequence:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:282
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:633
```

Those lines report missing gyro-odometer twist and an empty distortion-corrector twist queue. The same run shows EKF inactive and initialization rejected because the vehicle-stop condition was not satisfied:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:632
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:634
```

### Applied localization gate

The root patch keeps `launch_localization` default OFF. It may be enabled only after all of the following are live and validated:

- `/map/vector_map` and the pointcloud map are available;
- C_track GNSS translation aligns with both maps;
- `/autoware_orientation` has one valid publisher and compatible QoS;
- `/sensing/imu/imu_data` has the expected frame, type, QoS, and rate;
- `/sensing/vehicle_velocity_converter/twist_with_covariance` has one proven read-only source;
- `/sensing/lidar/top/mirror_cropped/pointcloud_ex` has the expected frame, QoS, and rate;
- static sensor transforms match measured current-vehicle extrinsics;
- `map -> base_link` has exactly one authority;
- the vehicle-stop check correctly reports the stationary vehicle.

## Calibration caution

The effective base-to-sensor-kit calibration is all zeros:

```text
/home/a/autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensors_calibration.yaml:1
/home/a/autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensors_calibration.yaml:8
```

Sensor-specific transforms are in:

```text
/home/a/autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml:1
/home/a/autoware/src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml:37
```

The `(copy_org)` calibration describes the prior multi-Velodyne vehicle and must not be restored wholesale. Neither current nor original values count as current-vehicle calibration without measurement.

## Post-patch hardware-disabled validation

Five stable runs used the exact `run_autoware` alias with `launch_sensing_driver=false` and `launch_localization=false`. In every cycle the Lanelet2 visualization reported the map loaded, the vector-map TF generator broadcast `map -> viewer`, Ctrl+C was applied only after that readiness point, the launch harness recorded exit code 0, and post-stop checks found no bad shutdown signature or orphan process.

| Cycle | Launch log | Map/TF readiness | Map container | Sensor container |
|---|---|---|---|---|
| 1 | `/home/a/.ros/log/2026-08-10-19-56-09-947244-a-37623/launch.log` | `:168`, `:174` | clean `:219` | clean `:220` |
| 2 | `/home/a/.ros/log/2026-08-10-20-00-04-431231-a-39099/launch.log` | `:168`, `:174` | clean `:207` | clean `:208` |
| 3 | `/home/a/.ros/log/2026-08-10-20-00-16-066567-a-39672/launch.log` | `:168`, `:174` | clean `:205` | clean `:207` |
| 4 | `/home/a/.ros/log/2026-08-10-20-00-58-921599-a-40260/launch.log` | `:168`, `:174` | clean `:215` | clean `:219` |
| 5 | `/home/a/.ros/log/2026-08-10-20-01-09-770034-a-40803/launch.log` | `:168`, `:174` | clean `:219` | clean `:218` |

The 0.56-second startup-interrupt run at `/home/a/.ros/log/2026-08-10-19-58-34-798961-a-38418/launch.log` is excluded: Ctrl+C arrived while nodes were still importing and loading, before the stable readiness point. Its forced gyro-bias-estimator termination is therefore not counted as a post-start shutdown failure.

## Runtime validation still required

For the next separately authorized stationary validation:

1. Launch with localization OFF and confirm map topics and projection metadata.
2. Confirm exactly one pointcloud-container instance and the intended localization target name.
3. Validate NovAtel fix, raw/corrected IMU, frames, QoS, and rates.
4. Validate one `/autoware_orientation` publisher before changing the localization gate.
5. Compare one timestamp-matched GNSS `PoseStamped`, `PoseWithCovarianceStamped`, and TF sample; translated positions must match numerically.
6. Overlay the translated GNSS pose on Lanelet2 and PCD landmarks; reject the C_track translation if it is not aligned.
7. Enable localization and verify exactly one `map -> base_link` authority, stable NDT score, EKF activation, and stationary initialization.
8. After localization is enabled, repeat five stable start/stop cycles and confirm no orphan map/localization/TF nodes.

## Rollback

Use the dated pre-HH snapshot for an exact rollback of the customized active files. Do not restore the top-level `(copy_org)` wholesale because it contains historical perception, planning, control, and API includes that are outside PC3 ownership.

Rollback is required if:

- translated GNSS pose does not align with the current C_track map;
- pose, covariance pose, and GNSS TF differ;
- `map -> base_link` has multiple authorities;
- localization activates without valid orientation, twist, or IMU;
- the sole pointcloud container crashes or leaves orphan components.
