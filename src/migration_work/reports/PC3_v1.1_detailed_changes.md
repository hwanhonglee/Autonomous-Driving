# PC3 v1.1 detailed changes

> **HH_260811 - This is the current PC3 v1.1 release record.** The dated
> `HH_260810` reports remain preserved as historical stationary-test snapshots,
> but their runtime conclusions are superseded by this document where the two
> records differ.

Audit date: 2026-08-11 (Asia/Seoul)
Host role: PC3 (`spectra`), prepared for the `IONIQ_EV_308_PC3_a/r` release tags
Scope: system, sensing, map, localization, TF, diagnostics, and visible RViz bring-up
Actuation rule: do not authorize AUTO until the live localization and command-path gates at the end of this report pass

## Target Git branches

The upstream repository is `hwanhonglee/Autonomous-Driving`. The v1.1 work must
be prepared as two independent branches based on the corresponding v1.0 heads:

| Workspace | v1.0 base branch | Audited v1.0 commit | Intended v1.1 branch |
|---|---|---|---|
| `/home/a/autoware` | `h2_i/IONIQ_EV_307/PC3_spectra/autoware_universe/v1.0` | `16ca930f4a8cfa9be7852dd17a7bffcb01d3415d` | `h2_i/IONIQ_EV_307/PC3_spectra/autoware_universe/v1.1` |
| `/home/a/ros2_ws` | `h2_i/IONIQ_EV_307/PC3_spectra/ros2_ws/v1.0` | `39872d589c5f3d056d2fba6950e0bbca3bde3f32` | `h2_i/IONIQ_EV_307/PC3_spectra/ros2_ws/v1.1` |

`v1.1` is a branch-name migration. It is not a request to rewrite XML
declarations such as `<?xml version="1.0"?>` or unrelated ROS package versions.

The current workspaces did not contain repository-level `.git` metadata when
first audited. They were subsequently initialized in place with the exact v1.0
commits above as their parents and with the two v1.1 branch names above as their
local heads. In particular, the remote PC3 `ros2_ws/v1.0` branch contains
`src/ublox`, while the recreated live workspace does not. The v1.1 commit
preserves that upstream directory in the Git tree and does not record its local
absence as a deletion.

After the two commits are created, the matching local release tags are
`IONIQ_EV_308_PC3_a` for the Autoware commit and `IONIQ_EV_308_PC3_r` for the
ROS 2 workspace commit. The historical `IONIQ_EV_307_PC3_a/r` tags remain
unchanged.

## Release outcome

PC3 now has a normal full bring-up path with system, map, sensing hardware,
localization, vehicle interface, MRM, and RViz defaults restored. PC3-specific
node and container names avoid shared-DDS name collisions. NovAtel OEM7 is the
primary GNSS/INS receiver, u-blox is a passive position fallback, and a new
credential-isolated NTRIP client validates RTCM3 before topic fanout and NovAtel
port0 injection. GNSS pose publication is fail-closed when orientation is
missing, invalid, or stale. The single Hesai cloud is published on Autoware's
standard pointcloud topic and relayed to the current PC2 legacy topic. LiDAR
shutdown defects, an asynchronous UDP endpoint-lifetime defect, and invalid
`PointCloud2.row_step` metadata were repaired with regression tests. System
memory diagnostics were made locale-independent. NVIDIA hardware rendering was
restored at the operating-system level and RViz was made safe for Snap-hosted
IDE terminals.

The offline build and regression stage is complete. It does not prove RTK FIX,
valid single-antenna azimuth alignment, current sensor calibration, GNSS-to-map
alignment, NDT convergence, MRM readiness across PCs, or safe physical AUTO.

## 1. Top-level launch, ownership, and safety behavior

### `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`

Root problem:

- the temporary `HH_260810` stationary profile had disabled normal hardware,
  localization, vehicle-interface, MRM, and RViz defaults;
- generic ROS node/container names could collide with other PCs on the shared
  ROS domain;
- disabling PC3 MRM without changing the diagnostic contract left an expected
  emergency-control-command publisher absent;
- Snap Visual Studio Code could inject an incompatible `GTK_PATH` into RViz.

Current behavior:

- `launch_sensing_driver`, `launch_localization`, `launch_vehicle_interface`,
  `launch_mrm`, `rviz`, and `rviz_respawn` default to `true` again;
- the generic container defaults to `pc3_pointcloud_container` and remains
  independently gateable with `launch_pointcloud_container`;
- the sensing-owned Hesai container remains separate from the generic
  container;
- PC3 RViz is named `rviz2_pc3`;
- `launch_mrm=true` selects the standard diagnostic graph, while
  `launch_mrm=false` selects the PC3 no-MRM graph;
- localization preprocessing is attached to
  `/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container`;
- RViz receives an empty `GTK_PATH`, preventing Snap's incompatible GTK library
  path from leaking into the process.

PC3's v1.0 source branch already omits the perception, planning, control, and
Autoware API include blocks from this top launch. Those roles remain assigned to
the other PCs; their unused argument declarations are not evidence that PC3
launches the modules.

Rollback: restore the v1.0 branch version or the dated pre-change snapshot, then
rebuild `autoware_launch`. Do not roll back only the diagnostic graph selection
without also reviewing `launch_mrm`.

### MRM gate and diagnostic contract

Files:

- `launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml`
- `universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml`
- `launcher/autoware_launch/autoware_launch/config/system/diagnostic_graph_aggregator/autoware-main-pc3-no-mrm.yaml`
- `/home/a/.bashrc` (host configuration, not part of either source branch)

The shared system launch now has a backward-compatible `launch_mrm` argument.
It gates the comfortable-stop operator, emergency-stop operator, and MRM
handler as one ownership decision while leaving monitors and diagnostics alive.
The optional PC3 no-MRM graph imports the full standard graph and removes only
`/autoware/system/topic_rate_check/emergency_control_command`, which is owned by
the PC1 actuation path in that mode.

The active `run_autoware` alias was restored to its normal launch command without
hidden module-disable overrides. A nearby `HH_260811` comment records why PC3's
MRM heartbeat must remain available to the PC1 vehicle command gate. `.bashrc`
is host state and must be documented separately rather than accidentally added
to an Autoware source commit.

Current default: MRM is enabled. The no-MRM graph is an explicit safety/test
option, not the default runtime graph.

### Unique generic pointcloud container

File:

- `launcher/autoware_launch/autoware_launch/launch/pointcloud_container.launch.py`

The `glog_component` namespace now follows the `container_name` launch
configuration instead of being hard-coded to `pointcloud_container`. This keeps
the logger component aligned with `pc3_pointcloud_container` and avoids a
cross-PC duplicate node name.

## 2. GNSS receiver ownership and failover

### Sensor-kit GNSS integration

Files:

- `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml`
- `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/package.xml`

NovAtel OEM7 remains the primary receiver. Its data driver uses stable USB
port1 at 115200 baud:

```text
/dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_BMHR21430182H-if00-port1
```

The launch now:

- separates raw receiver fixes into
  `/sensing/gnss/novatel/oem7/fix` and
  `/sensing/gnss/ublox/nav_sat_fix`;
- starts the passive u-blox fallback only through an owned non-persistent
  configuration;
- starts NTRIP only inside the sensing hardware gate and only for the NovAtel
  primary mode;
- publishes the arbitration result on `/sensing/gnss/selected/fix`;
- derives `/autoware_orientation` only from validated NovAtel `INSPVAX`;
- feeds `autoware_gnss_poser` from the selected best-available position.

HH_260812 - The selected fix frame now gates attitude use: only `gnss_link` can
consume validated NovAtel INS orientation. A u-blox `gps`-frame fix is emitted
as a degraded position seed with broad yaw variance, and PC3 initialization
accepts it only when NDT/YabLoc reports a reliable map alignment. This retains
fallback availability without combining antennas or presenting inferred yaw as
measured attitude.

Runtime dependencies for GNSS, IMU correction, Hesai/common sensing, NTRIP,
u-blox, `topic_tools`, and velocity conversion were made explicit in
`package.xml`.

### New `autoware_gnss_failover_selector` package

Package root:

```text
universe/autoware.universe/sensing/autoware_gnss_failover_selector/
```

Runtime source:

- `autoware_gnss_failover_selector/failover_node.py`
- `autoware_gnss_failover_selector/selector_logic.py`
- `autoware_gnss_failover_selector/novatel_orientation_node.py`
- `autoware_gnss_failover_selector/orientation_logic.py`
- `autoware_gnss_failover_selector/__init__.py`

Configuration and launch:

- `config/gnss_redundancy.param.yaml`
- `config/ublox_fallback_no_persist.param.yaml`
- `launch/gnss_redundancy.launch.xml`
- `launch/ublox_fallback_driver.launch.xml`

Package and documentation:

- `package.xml`
- `setup.py`
- `setup.cfg`
- `resource/autoware_gnss_failover_selector`
- `README.md`

Tests:

- `test/test_selector_logic.py`
- `test/test_orientation_logic.py`
- `test/test_dual_heading_logic.py`
- `test/test_orientation_node_policy.py`

Position selection behavior:

- validates NavSatFix status, finite coordinates, coordinate bounds, covariance
  shape, optional covariance presence, and optional horizontal variance;
- treats NovAtel as primary with a 1.0-second freshness timeout;
- treats u-blox as fallback with a 2.0-second freshness timeout;
- applies failure, activation, and recovery hysteresis to prevent source
  flapping;
- waits for five continuous seconds of primary recovery before returning from
  fallback;
- publishes no fix when neither source is healthy;
- publishes a transient-local selected-source state and `/diagnostics` status.

Orientation behavior:

- accepts only fresh `INSPVAX` with `INS_SOLUTION_GOOD` status;
- requires finite roll, pitch, azimuth, and uncertainties;
- limits azimuth RMSE to 5 degrees and roll/pitch RMSE to 10 degrees;
- converts NovAtel north-clockwise azimuth to a normalized ROS ENU quaternion;
- publishes source/valid states separately from the position selector;
- decodes `INSPVAX.ext_sol_status` alignment indication and active ALIGN aiding
  for diagnostics.

Future dual-antenna `HEADING2` is checked for `SOL_COMPUTED`, `NARROW_INT`,
freshness, and uncertainty limits, but is diagnostic-only. It is not converted
directly into body attitude because the antenna baseline, heading offset,
mounting rotation, and roll are not established by HEADING2 alone. A future
dual-antenna install should make SPAN/ALIGN produce valid `INSPVAX`; the existing
orientation output path then becomes usable without inventing attitude.

The u-blox configuration uses its stable by-id path, disables startup
configuration, sets load/save masks to zero, disables BBR clearing, and disables
save-on-shutdown. It must not be replaced with a stock configuration that writes
receiver nonvolatile memory without a separate review.

The historical PC3 `ros2_ws/v1.0` tree also contains a locally modified
`src/ublox` version 2.3.0. That source is required for correction-assisted
fallback: its node subscribes to absolute `/rtcm` as
`rtcm_msgs/msg/Message` and passes the bytes to `gps_->sendRtcm()`. The Ubuntu
Humble binary found on this PC can publish ordinary position fallback, but its
`UInt8MultiArray` raw-data path is logging-oriented and does not provide the
same receiver-injection contract. The v1.1 Git tree therefore preserves
`src/ublox`; a fresh checkout must build that overlay before accepting u-blox
RTK fallback. An isolated archive build completed all three packages
(`ublox_serialization`, `ublox_msgs`, and `ublox_gps`) successfully without
opening the receiver. The preserved driver publishes NavSatFix on private
`~/fix`; the fallback launch now remaps that exact private topic to
`ublox/nav_sat_fix` instead of relying on the non-matching relative `fix`
name.

## 3. NovAtel OEM7 driver restoration and corrections

Package root:

```text
/home/a/ros2_ws/src/novatel_oem7_driver
```

The OEM7 message and driver sources were restored from
`/home/a/ros2_ws_original.zip`. The archived tree and the restored initial tree
contained 137 matching files with zero mismatches. The archive SHA256 recorded
by the dated audit is:

```text
fb9ff84cf4f89f25c4790bfd3133c3ba48b901d357fb4d86778b256e90b3e923
```

Modified files after restoration:

| File | Root cause | v1.1 behavior |
|---|---|---|
| `src/novatel_oem7_driver/config/std_init_commands.yaml` | Single-antenna kinematic alignment required a lower explicitly requested threshold. | Sends `SETALIGNMENTVEL 0.833333`, equivalent to 3 km/h. |
| `src/novatel_oem7_driver/src/oem7_message_node.cpp` | Binary receiver output could be mistaken for the ASCII response to an initialization command. | A command attempt succeeds only on exact `OK`; a non-OK response is logged and retried, up to the existing attempt limit. |
| `src/novatel_oem7_driver/src/ins_handler.cpp` | `RAWIMUSX` and `INSPVAS` message IDs were registered twice. | Each INS message ID is registered once, preventing duplicate IMU/INS publication paths. |
| `src/novatel_oem7_driver/novatel_oem7_driver/rosbag_comparison.py` | Historical bags use frame `gps` while the active vehicle configuration correctly uses `gnss_link`. | Explicit per-topic expected frames are asserted on the unit under test before only the legacy reference message is normalized for comparison. |
| `src/novatel_oem7_driver/novatel_oem7_driver/testutil.py` | Tests had no opt-in frame-migration contract. | Derived bag tests provide an explicit topic-to-frame mapping. |
| `src/novatel_oem7_driver/test/bestpos.test.py` | `/fix` and `/gps` legacy reference frames differed from the active configuration. | Both topics explicitly require `gnss_link`. |
| `src/novatel_oem7_driver/test/ins1.test.py` | `/insstdev` legacy reference frame differed from the active configuration. | The topic explicitly requires `gnss_link`. |

Reference bag files were not rewritten. The test adaptation cannot hide a wrong
runtime frame because it first asserts that the unit under test published the
configured `gnss_link` frame.

The 3 km/h threshold does not produce heading while stationary. With one
antenna, usable azimuth still depends on successful SPAN kinematic alignment,
correct IMU/antenna lever arms and rotations, sufficient motion, and
`INS_SOLUTION_GOOD` with acceptable uncertainty.

## 4. New NTRIP/RTCM package

Package root:

```text
/home/a/ros2_ws/src/pc3_ntrip_client
```

Files:

- package: `package.xml`, `setup.py`, `setup.cfg`,
  `resource/pc3_ntrip_client`, `.gitignore`, `README.md`;
- launch/config: `launch/ntrip_client.launch.py`,
  tracked placeholder `config/ntrip_config.yaml`;
- runtime: `pc3_ntrip_client/config.py`, `ntrip.py`, `rtcm.py`, `serial_sink.py`,
  `node.py`, `__init__.py`;
- tests: `test_config.py`, `test_ntrip_protocol.py`, `test_rtcm.py`,
  `test_node_topics.py`, `test_fake_caster_pty.py`.

Behavior:

- reads NTRIP caster, mountpoint, credentials, timeout, reconnect, serial, and
  ROS settings from one private YAML file;
- exposes only the config-file path as a ROS parameter;
- requires the live config to be a regular file with no group/other permission
  bits, rejects symbolic-link traversal where the platform supports
  `O_NOFOLLOW`, rejects oversized or invalid YAML, rejects unchanged
  `CHANGE_ME_*` placeholders, and never logs credentials or HTTP authorization
  headers;
- supports NTRIP 1.0/2.0 success responses, legacy ICY responses, and HTTP
  chunked transfer decoding;
- reconnects with bounded exponential backoff;
- incrementally reconstructs RTCM3, validates CRC-24Q, rejects corrupt frames,
  and resynchronizes after partial or invalid input;
- publishes validated frames on
  `/sensing/gnss/novatel/rtcm` and `/rtcm` as
  `rtcm_msgs/msg/Message`; the latter matches the preserved PC3 u-blox
  receiver-injection source;
- writes the same validated frames to stable NovAtel USB port0 at 115200 baud;
- optionally sends `INTERFACEMODE THISPORT AUTO NONE OFF` before correction
  bytes so the receiver detects RTCM input in RAM;
- never sends `SAVECONFIG` or another persistent receiver-save command;
- reports stream, reconnect, frame, byte, CRC, serial-open, and serial-setup
  health on `/diagnostics`.

The bring-up recorded `www.gnssdata.or.kr:2101` and candidate mountpoint
`CNJU-RTCM32`. Operators must verify both with the provider before entering
them in a private deployment file. Credentials are intentionally omitted from
this report.

Secret handling:

- tracked `config/ntrip_config.yaml` contains only documented `CHANGE_ME_*`
  values and is installed as an operator template;
- the source, installed launch, and Autoware GNSS integration default to the
  external runtime file `/home/a/.config/pc3_ntrip/ntrip_config.yaml`;
- the external directory is mode 0700 and the private file is mode 0600;
- `*.local.yaml` and `*.private.yaml` source-tree variants remain ignored;
- real credential values must never be written to the tracked template,
  staged, committed, copied into a report, or printed in build/test logs.

Receiving valid RTCM proves only correction transport. RTK integer FIX must be
accepted from the live NovAtel solution, especially `BESTPOS.pos_type`; the
driver's NavSatFix status maps several differential, float, and fixed solution
types to a common status and is not sufficient proof by itself.

## 5. GNSS poser, map coordinates, and localization safety

### GNSS pose consistency and orientation gating

Files:

- `universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp`
- `universe/autoware.universe/sensing/autoware_gnss_poser/include/autoware/gnss_poser/gnss_poser_node.hpp`
- `universe/autoware.universe/sensing/autoware_gnss_poser/config/gnss_poser.param.yaml`

The previously existing C-track translation had been applied only to the
covariance-bearing pose. The plain pose and TF retained unshifted coordinates,
creating internally inconsistent outputs. The translation is now applied once
to the shared pose before plain-pose, covariance-pose, and TF publication. The
original map-calibrated values were restored on `HH_260812` and moved from C++
literals into validated parameters:

```text
x := x - projected_position_offset_x_m  # 60966.4679793288
y := y - projected_position_offset_y_m  # 65973.64540576655
z := z - projected_position_offset_z_m  # 15.816125382
```

When INS orientation is enabled, GNSS pose publication now remains inhibited
until orientation has been received and the absolute fix/orientation stamp gap
is at most 1.5 seconds. Non-finite quaternions, near-zero quaternion norms,
non-finite RMSE, and negative RMSE are rejected. Accepted quaternions are
normalized before use.

Rollback: restoring the older implementation also restores inconsistent
pose/covariance/TF coordinates and the unsafe acceptance of missing/stale
orientation; use only as a diagnostic comparison, not as a production fix.

### TF ownership

File:

- `launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml`

The static map TF generator targets `viewer`, leaving dynamic `map -> base_link`
authority to localization. This prevents a static/dynamic TF authority conflict.
Relative to the remote v1.0 PC3 branch, the functional value remains `viewer`;
the v1.1 edit replaces the older date comment with an explicit authority
explanation.

### Localization pointcloud contract

File:

- `launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml`

Localization now consumes the final distortion-corrected, ring-filtered,
`base_link` cloud on `/sensing/lidar/concatenated/pointcloud` instead of the
older top-sensor intermediate mirror-cropped topic.

### Stale input handling

Files:

- `universe/autoware.universe/localization/autoware_gyro_odometer/config/gyro_odometer.param.yaml`
- `universe/autoware.universe/localization/autoware_pose_initializer/src/gnss_module.cpp`

Gyro odometer rejects vehicle-twist or IMU data older than 0.2 seconds. Pose
initializer now computes age as `now - message_stamp`; the former reversed
subtraction could let an old GNSS pose pass the timeout test.

### Existing vehicle/map tuning that differs from remote PC3 v1.0

The following active differences predate `HH_260810/HH_260811` and were not
invented during the current repair session. They are nevertheless part of the
current PC3 tree and require an explicit include/exclude decision for v1.1:

| File | Active difference from remote v1.0 |
|---|---|
| `launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml` | step 0.1, resolution 2.5, 35 iterations, regularization off, critical time 200 ms, skip count 10, score thresholds 3.2/2.1, no-ground scoring off, map update distance 35 m, plus existing six-thread setting |
| `launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml` | voxel size reduced from 3.0 m to 1.2 m on all axes |
| `launcher/autoware_launch/autoware_launch/config/map/pointcloud_map_loader.param.yaml` | whole-map downsample leaf reduced from 3.0 m to 0.7 m; its metadata-path comment was restored during the v1.1 documentation audit |
| `launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml` | dynamic-obstacle-stop module default changed from true to false; PC3 does not launch planning, so ownership and whether this belongs in the PC3 branch must be reviewed |
| `param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml` | active camera, traffic-light camera, Hesai, GNSS, and IMU transforms replace sample geometry with vehicle-specific values |
| `sensor_component/external/nebula/nebula_ros/config/lidar/hesai/Pandar64.param.yaml` | host IP changed from `192.168.2.100` to PC3 `192.168.2.150` |
| `vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml` | wheelbase changed from 3.0 m to 2.7 m |

These values are operationally significant. Preserve them only when their
vehicle measurements and live NDT behavior are accepted. The v1.1 documentation
audit added or expanded adjacent English `HH_250204`, `HH_250205`, `HH_250219`,
`HH_250311`, or `HH_260811` rationale comments while retaining the original
attribution date where it was available.

<!-- HH_260812 - Record the C-track coordinate audit, recovered offsets, and validation evidence. -->
### `HH_260812` C-track coordinate correction audit and recovery

The map-coordinate disagreement was traced to two distinct C-track map
conventions, not to NTRIP or RTK accuracy:

- the active external map under `/home/a/Autoware_Map/C_track` uses a synthetic
  `52SCF0` projector origin and Lanelet latitude/longitude values generated for
  that origin;
- the archived map bundle uses the native C-track (`52SCF60...`) origin and
  natively georeferenced Lanelet latitude/longitude values.

All 13,172 active Lanelet nodes reproduce their stored local x/y values from
the active `52SCF0` origin with a mean error below `0.00006 m`. Therefore only
changing `map_projector_info.yaml` would move the Lanelet map by about 61 km
east and 66 km north relative to the PCD. The projector YAML, Lanelet map, and
PCD must always be changed as one map bundle.

The legacy `gnss_poser` subtraction was originally introduced to translate the
synthetic `52SCF0` projection into C-track-local coordinates. A stopped RTK
integer sample produced the following evidence:

```text
current pose before recovery:             (61.536739, -118.539020,   1.094781)
same sample with original calibrated XYZ: (64.844025, -125.262097, -14.721344)
PCD road height at recovered XY:                                  -15.15 m
base_link height above road:                                        0.429 m
configured wheel radius:                                            0.406 m
```

The recovered point is inside Lanelet relation `30447`; the pre-recovery point
was outside every Lanelet and at least `5.094 m` from the nearest boundary.
The former refactor had changed the calibrated horizontal constants and dropped
the `15.816125382 m` vertical correction. That regression explains the observed
horizontal and vertical disagreement.

The correction is now controlled by `projected_position_offset_enabled` and
three finite-valued offset parameters. The active legacy map enables the exact
calibrated XYZ values. A native C-track-origin map bundle must set the enable
flag to `false`, preventing a duplicate 61 km/66 km translation. Pose,
PoseWithCovariance, and TF continue to consume the same corrected shared pose.

Validation completed with a low-parallel package build, all four registered
package test targets passing, and a hardware-free Domain-229 projection test.
The test loaded all three installed offsets and produced the expected corrected
pose. Final acceptance still requires a fresh `run_autoware` process and live
NDT/EKF convergence because the prior process predated this build.

## 6. IMU path

No separate new physical IMU driver was added. The existing sensor-kit launch
consumes NovAtel OEM7's SPAN-derived raw IMU topic:

```text
/sensing/gnss/novatel/oem7/imu/data_raw
  -> autoware_imu_corrector
  -> /sensing/imu/imu_data
```

The gyro-bias estimator also observes the raw OEM7 IMU and
`/localization/kinematic_state`. This path depends on the attached NovAtel IMU
being present, correctly configured, correctly framed, and publishing valid
timestamps and covariance. The source audit does not prove the current physical
IMU identity or mounting.

## 7. Hesai LiDAR topic contract and stability

### Launch and topic flow

Files:

- `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml`
- `sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/hesai_Pandar_64.launch.xml`
- `sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py`
- `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/package.xml`

The top-level sensing driver gate now reaches Nebula instead of an unconditional
`launch_driver=true`. The PC3 host-IP launch argument is reused, and the
container-name substitution is evaluated correctly instead of being passed as
a literal string.

The final ring-filter output topic is configurable in common launch, retaining
`pointcloud_before_sync` as its backward-compatible shared default. PC3's Pandar
launch overrides it to:

```text
/sensing/lidar/concatenated/pointcloud
```

and requests the final output in `base_link` rather than the sensor frame. A
non-lazy `topic_tools relay` publishes the same cloud to the current PC2 legacy
input:

```text
/sensing/lidar/top/pointcloud_before_sync
```

This relay does not mean PC2 owns a LiDAR. PC3 owns the physical sensor and
temporarily provides PC2's historical input contract during the multi-PC topic
migration.

### Ring filter `PointCloud2` metadata

Files:

- `universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/src/outlier_filter/ring_outlier_filter_node.cpp`
- `universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/test/test_ring_outlier_filter_node.cpp`
- `universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/CMakeLists.txt`

The live ring-filter path resized the payload and set height/width but did not
update `row_step`, producing `row_step=0` for a non-empty one-row cloud. The
filter now sets:

```text
row_step = point_step * width
```

The regression constructs the live point type, runs the filter, and requires a
non-zero compact row step consistent with the output payload.

### UDP endpoint lifetime

Files:

- `sensor_component/transport_drivers/udp_driver/src/udp_socket.cpp`
- `sensor_component/transport_drivers/udp_driver/test/test_udp_data.cpp`

Asynchronous receive operations now use the socket object's member sender
endpoint, whose lifetime covers the pending callback, instead of a temporary or
incorrect endpoint. Receive handlers also preserve the configured local bind
endpoint. The functional UDP test verifies send/receive and that the receiver's
host IP/port remain unchanged.

### Nebula hardware teardown

Files:

- `sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp`
- `sensor_component/external/nebula/nebula_hw_interfaces/test/test_hesai_hw_interface_shutdown.cpp`
- `sensor_component/external/nebula/nebula_hw_interfaces/CMakeLists.txt`

The Hesai hardware interface destructor now stops the sensor interface and TCP
driver. `SensorInterfaceStop()` closes a pending UDP receiver while its callback
state is alive and waits for the I/O context to exit. The regression performs
64 loopback-only construction/start/destruction cycles without contacting LiDAR
hardware.

### Nebula ROS decoder-thread teardown

Files:

- `sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp`
- `sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp`

The previous wrapper could destroy a still-joinable decoder thread blocked on
its packet queue, deterministically causing `std::terminate`/SIGABRT. The
destructor now stops packet producers, pushes a null sentinel, and joins the
decoder thread. Five hardware-disabled exact-alias cycles exited cleanly after
this change. The later hardware-interface regression covers an outstanding
loopback UDP receive; a full physical-sensor start/stop cycle still belongs to
live acceptance.

## 8. System diagnostics, GPU, and RViz

### Locale-independent memory monitor

Files:

- `universe/autoware.universe/system/system_monitor/src/mem_monitor/mem_monitor.cpp`
- `universe/autoware.universe/system/system_monitor/test/src/mem_monitor/free_locale.cpp`
- `universe/autoware.universe/system/system_monitor/test/src/mem_monitor/test_mem_monitor_locale.cpp`
- `universe/autoware.universe/system/system_monitor/CMakeLists.txt`

The memory monitor parses positional output from `free -tb`. A Korean host
locale changes labels and broke that parser. The child environment now forces
`LC_ALL=C` without changing the parent process. A fake `free` executable emits
either Korean or C-locale output, and the regression proves the monitor requests
and parses the C form.

### RViz source changes

File:

- `launcher/autoware_launch/autoware_launch/rviz/autoware.rviz`

Intentional performance/display changes include:

- display the approximately 1,500-point localization downsample topic instead
  of the full-rate legacy LiDAR topic;
- leave the NDT aligned debug cloud disabled by default;
- cap the RViz frame rate at 20 FPS.

The file also contained RViz GUI-generated state differences from remote v1.0.
The v1.1 cleanup restored the opaque `QMainWindow State` cache blob to the v1.0
baseline and retained only the reviewed user-visible display, topic, view, and
performance defaults. The nearby `HH_260811` comments now describe bounded
rendering cost without incorrectly claiming that software rendering is active.

### Operating-system-only GPU recovery

The following work changed the PC3 operating system, not either Git branch:

- removed the conflicting `nvidia-firmware-570-570.211.01` package during the
  broken-package repair;
- completed `apt --fix-broken install`;
- aligned NVIDIA 570 libraries/settings to `570.211.01-0ubuntu1`;
- rebooted into kernel `6.8.0-52-generic`;
- verified the `nvidia` kernel driver owns the RTX 3070;
- verified `nvidia-smi` reports driver `570.211.01`;
- added user `a` to `dialout` and verified the new group in the active session.

An RViz smoke test with `GTK_PATH` sanitized loaded NVIDIA GL libraries, appeared
as a GPU process, used OpenGL 4.6, and exited cleanly. Observed RViz CPU fell from
approximately 140-190% in software rendering to approximately 22%, with RSS
falling from roughly 1.66 GiB to 253 MiB during the comparable smoke check.

These package/group operations cannot be reproduced by a source commit. Record
them in deployment documentation or an installation script only after a
separate review; do not commit package caches, DKMS artifacts, or system logs.

### `HH_260812` persistent PC3 route ownership

<!-- HH_260812 - Record the reviewed Netplan template, exact legacy routes, and privileged deployment. -->
File:

- `migration_work/config/pc3-netplan.yaml`

Before this deployment, `/etc/netplan/config.yaml` assigned a default route and
DNS server to each systemd-networkd-owned Ethernet link:

| Interface | Static address | Legacy default route | Legacy DNS |
|---|---|---|---|
| `enp8s0` | `192.168.9.7/24` | `via 192.168.9.1`, metric `50` | `192.168.9.1` |
| `enp0s31f6` | `192.168.2.150/24` | `via 192.168.2.1`, metric `100` | `192.168.2.1` |

Those persistent declarations recreated both wired defaults after every boot.
Operators consequently had to run both `ip route del default` commands and then
manually start NetworkManager before Wi-Fi could reliably own Internet access.

The reviewed deployment template keeps only these directly connected networks:

```text
enp0s31f6  192.168.2.150/24  Hesai sensor network
enp8s0     192.168.9.7/24    PC1-PC3 distributed ROS network
```

It deliberately defines no wired gateway or DNS server and marks both links
optional so an unplugged sensor/peer cable does not hold boot completion.
NetworkManager remains responsible only for Wi-Fi and must be enabled once with
`systemctl enable --now NetworkManager.service`. The active Wi-Fi DHCP profile
then owns the sole Internet default route, while systemd-networkd continues to
own both static Ethernet links.

<!-- HH_260812 - Record the deployed file ownership, rollback copy, and service ownership boundary. -->
The source template is not applied merely by checking out this repository. Its
one-time deployment created the root-owned mode-`0600` rollback copy
`/etc/netplan/config.yaml.pre-HH_260812`, installed the reviewed template as
root-owned mode-`0600` `/etc/netplan/config.yaml`, ran `netplan generate`, and
accepted an interactive `netplan try`. This deployment was completed on
`HH_260812`.

Post-apply ownership is intentionally split. systemd-networkd owns
`enp0s31f6` and `enp8s0` through the generated
`/run/systemd/network/10-netplan-*.network` files; NetworkManager reports those
Ethernet links as unmanaged. NetworkManager is both active and persistently
enabled, owns Wi-Fi device `wlx200db04a991e`, and has the `Lehong` DHCP profile
connected with autoconnect enabled. Both wired links report
`routable (configured)` and `Required For Online: no`, with no wired DNS or
default-route scope.

<!-- HH_260812 - Record the exact post-apply routes and DNS behavior observed before reboot. -->
The live main IPv4 route table contains the three connected subnets and exactly
one Internet default route:

```text
default via 172.20.10.1 dev wlx200db04a991e proto dhcp metric 600
172.20.10.0/28 dev wlx200db04a991e proto kernel scope link src 172.20.10.12 metric 600
192.168.2.0/24 dev enp0s31f6 proto kernel scope link src 192.168.2.150
192.168.9.0/24 dev enp8s0 proto kernel scope link src 192.168.9.7
```

Route-resolution checks preserve direct wired reachability while sending public
traffic through Wi-Fi:

```text
192.168.2.101 -> enp0s31f6, source 192.168.2.150
192.168.9.2   -> enp8s0, source 192.168.9.7
1.1.1.1       -> wlx200db04a991e via 172.20.10.1, source 172.20.10.12
```

There is one resolver nuance: systemd-resolved has no DNS scope on either wired
link and learns `172.20.10.1` as the Wi-Fi link DNS server, but
`/etc/resolv.conf` remains a regular static file rather than a systemd-resolved
or NetworkManager-managed symlink. It contains `8.8.8.8` and `8.8.4.4`, and
`resolvectl` therefore reports `resolv.conf mode: foreign`. Consumers that read
`/etc/resolv.conf` directly continue to use those static Google resolvers. This
pre-existing resolver-file policy was recorded but not changed as part of route
ownership repair.

<!-- HH_260812 - Define the cold-boot acceptance criteria for persistent route ownership. -->
A cold reboot remains the final persistence check. Acceptance requires
NetworkManager to be `enabled` and `active` without a manual start, the available
`Lehong` profile to reconnect automatically, both wired addresses and connected
routes to return under systemd-networkd with `Required For Online: no`, and no
wired default route or wired DNS scope to reappear. With `Lehong` connected, the
sole default must again be `via 172.20.10.1 dev wlx200db04a991e metric 600`, and
the two direct route-resolution checks above must retain their wired interface
and source-address selections. Passing these checks proves that the three
previous manual commands are no longer required after boot.

## 9. Preservation, reports, and commit exclusions

### Preserved source

- Existing same-directory `(copy_org)` and `(copy_C_track)` files were not
  overwritten.
- The immediate pre-change snapshot is under
  `migration_work/backups/PC3/pre_HH_260810/`.
- Its `SHA256SUMS` manifest currently verifies 39 of 39 entries.
- `migration_work/backups/PC3/post_HH_260810/SHA256SUMS` records the end of the
  earlier stationary stage; active files changed again during v1.1 work, so it
  is a historical manifest rather than a checksum of today's final tree.
- OEM7 reference bags and the original `/home/a/ros2_ws_original.zip` were not
  rewritten.

New same-directory preservation files created during the earlier stage include:

- `launcher/.../components/tier4_system_component.launch (copy_org).xml`;
- `universe/.../tier4_system_launch/launch/system.launch (copy_org).xml`;
- `sensor_component/.../hesai_ros_wrapper (copy_org).hpp`;
- `sensor_component/.../hesai_ros_wrapper (copy_org).cpp`.

The dated snapshot is the authoritative immediate rollback source when an older
historical `(copy_org)` differs from the file that was active just before this
work.

### Historical audit artifacts

The following files remain useful as evidence of the 2026-08-10 stationary
stage, but their live-status statements are superseded by this v1.1 report:

- `migration_work/HH_260810_CHANGELOG.md`;
- `migration_work/reports/PC3_HH_260810_changes.md`;
- `migration_work/reports/PC3_alias_trace.md`;
- `migration_work/reports/PC3_chrony.md`;
- `migration_work/reports/PC3_driver_ownership.csv`;
- `migration_work/reports/PC3_duplicate_analysis.md`;
- `migration_work/reports/PC3_launch_flow.md`;
- `migration_work/reports/PC3_map_localization_tf.md`;
- `migration_work/reports/PC3_sensor_hardware_inventory.md`;
- `migration_work/reports/PC3_start_stop_stability.md`;
- `migration_work/reports/PC3_vehicle_interface_no_actuation.md`;
- `migration_work/config/node_ownership.yaml`;
- `migration_work/config/readiness_checks.yaml`;
- `migration_work/config/topic_contract.yaml`;
- `migration_work/rollback/PC3_rollback.md`.

For example, the older sensor report says NVIDIA is broken and the older GNSS
report says the orientation publisher is unresolved. Those were correct at the
snapshot time but are no longer current conclusions.

### Do not commit

- `/home/a/.config/pc3_ntrip/ntrip_config.yaml` or any other
  credential-bearing variant; the source-tree `config/ntrip_config.yaml` is
  intentionally committed only while every deployment field remains a
  documented `CHANGE_ME_*` placeholder;
- `build/`, `install/`, or `log/` trees from either workspace;
- source-tree `log/` directories;
- `__pycache__/`, `.pytest_cache/`, `.pyc`, test-generated bags, or
  `novatel_oem7_driver/log/`;
- GPU driver packages, DKMS output, apt caches, user/group databases, or other
  OS state;
- the 33 MiB raw migration backup tree as ordinary release source without an
  explicit archival decision; it contains build binaries, logs, and a copied
  workspace archive;
- a deletion of remote `ros2_ws/src/ublox` caused only by its absence from the
  recreated live workspace.

Recommended source commit content is the reviewed active source/config/tests,
the placeholder-only NTRIP template and package documentation, this v1.1 report, and
small text-only ownership/rollback records. Raw backups should remain local or
be placed in a deliberately managed release artifact, not mixed into code.

## 10. Validation evidence

### Build

Autoware low-parallel targeted build: 14 of 14 packages succeeded:

- `autoware_pointcloud_preprocessor`;
- `system_monitor`;
- `autoware_gnss_failover_selector`;
- `autoware_gnss_poser`;
- `autoware_pose_initializer`;
- `autoware_gyro_odometer`;
- `common_sensor_launch`;
- `sample_sensor_kit_launch`;
- `autoware_launch`;
- `nebula_hw_interfaces`;
- `boost_io_context`;
- `boost_serial_driver`;
- `boost_tcp_driver`;
- `boost_udp_driver`.

Relevant logs:

```text
/home/a/autoware/log/build_2026-08-11_18-31-09
/home/a/autoware/log/build_2026-08-11_18-37-14
/home/a/autoware/log/test_2026-08-11_18-37-45
```

`system_monitor` and the GNSS selector were rebuilt after final comment/test
adjustments, and `autoware_launch` was rebuilt after the RViz environment fix.

ROS-driver workspace build: 4 of 4 packages succeeded:

- `pc3_ntrip_client`;
- `novatel_oem7_msgs`;
- `novatel_oem7_driver`;
- `hesai_ros_driver`.

### Targeted regression results

- GNSS failover/orientation selector: 20/20 passed, including the historical
  u-blox private-topic and non-persistent configuration contracts;
- NTRIP client: 13/13 passed after the external-secret and u-blox RTCM contract
  changes;
- preserved historical u-blox overlay: 3/3 packages built successfully in an
  isolated checkout without device access;
- OEM7 file-replay launch tests: 6/6 passed after explicit legacy-frame test
  handling;
- ring-filter row-step regression: 1/1 passed;
- UDP functional data tests: 4/4 passed;
- Hesai hardware shutdown regression: 2/2 passed;
- memory-monitor locale regression: 1/1 passed;
- relevant GNSS poser, pose initializer, gyro odometer, launch XML, and common
  sensor package tests passed;
- XML, Python, and YAML source parsing audit passed for the modified launch and
  config set;
- pre-change backup manifest: 39/39 passed.

The final broad Autoware result still reports pre-existing formatting/lint debt
in the transport-driver packages: one `boost_serial_driver`, twenty
`boost_tcp_driver`, and twelve `boost_udp_driver` lint/style findings. Functional
tests for the changed UDP behavior pass. These 33 lint findings are not evidence
of a runtime regression, but they should not be reported as a completely clean
repository-wide test suite.

No physical serial, NTRIP caster, CAN actuation, or LiDAR sensor was contacted by
the final offline regression runs. The NTRIP end-to-end test uses a local fake
caster and pseudo-terminal; the Hesai teardown test uses loopback UDP.

## 10.1 HH_260812 live localization and degraded-GNSS corrections

HH_260812 - Record the exact live initialization blocker and the bounded fixes
added after the first v1.1 snapshot.

The 2026-08-12 live PC3 run proved that NTRIP was not the reason localization
remained uninitialized. The external mode-0600 NTRIP configuration was loaded,
the caster TCP session was established, CRC-valid RTCM3 was arriving, and
NovAtel reported RTK integer solutions. The pose initializer instead rejected
every automatic request with `The vehicle is not stopped.` PC1 was publishing
zero longitudinal speed but a constant `0.00110999995 m/s` lateral value. The
unchanged upstream stop checker requires all three linear components to remain
below `0.001 m/s`, so this 1.11 mm/s stationary CAN noise blocked activation.

The correction leaves the shared `VehicleStopCheckerBase` source, class layout,
ABI, and original 1 mm/s rule byte-for-byte unchanged. Only the pose initializer
input applies a configurable three-axis norm deadband: speeds below `0.005 m/s`
are normalized to zero before the existing checker, while the three continuous
stopped seconds remain mandatory. A regression test proves that 1.11 mm/s is
normalized, that a sample exactly at 5 mm/s is retained, and that diagonal
motion whose norm exceeds 5 mm/s is also retained.

GNSS behavior now follows a best-available hierarchy:

1. selected NovAtel fix with fresh validated `INS_SOLUTION_GOOD` attitude;
2. selected NovAtel position with identity/held/course yaw and 10 rad^2 yaw
   variance when INS is missing or stale;
3. u-blox position fallback after selector hysteresis when NovAtel itself is
   invalid or absent.

NovAtel attitude is applied only when the selected fix frame is `gnss_link`.
The u-blox `gps` frame therefore cannot inherit stale NovAtel attitude. Fallback
course yaw is updated only after at least 1 m of displacement; otherwise the
last validated yaw, or identity at cold start, is retained. Pose initialization
also rejects an explicit unreliable NDT/YabLoc alignment on PC3 instead of
activating EKF from an untrusted degraded seed.

The u-blox launch now remaps both the preserved driver's private `~/fix` and the
Ubuntu binary's relative `fix` name to `ublox/nav_sat_fix`. The 158 tracked
historical u-blox files were restored into the live `ros2_ws` worktree and all
four overlay packages (`ublox_serialization`, `ublox_msgs`, `ublox_gps`, and
`ublox`) built successfully. The sourced `ublox_gps` prefix is now
`/home/a/ros2_ws/install/ublox_gps`, whose absolute `/rtcm`
`rtcm_msgs/msg/Message` callback forwards corrections to `Gps::sendRtcm`.

Post-correction validation passed:

- six affected Autoware packages built and installed sequentially with one
  worker, followed by an ABI-compatible incremental rebuild;
- GNSS failover contracts: 20/20 passed;
- stationary-noise deadband regression: 1/1 passed;
- pose-initializer existing unit tests: 3/3 passed;
- hardware-free GNSS position-only output produced a valid quaternion with
  exactly 10 rad^2 yaw variance;
- hardware-free source binding proved that a `gps`-frame fix cannot consume
  NovAtel attitude, while a `gnss_link` fix consumes it with its measured
  covariance;
- `VehicleStopCheckerBase` has no source diff from the v1.1 snapshot after the
  PC3-only deadband design replaced the earlier ABI-changing prototype;
- all four preserved u-blox overlay packages built and the overlay prefix was
  selected.

The running Autoware processes predate these installed binaries and parameters.
No live localization service, driver, serial port, or vehicle state was changed
during the patch. A controlled full PC3 restart is required before acceptance.

## 11. Current unresolved live gates

Keep the vehicle in MANUAL/PARK and do not attempt AUTO until all applicable
items below are accepted and recorded:

1. Verify that only PC3 owns the physical Hesai, NovAtel, u-blox, and NTRIP
   paths, with no duplicate manual drivers.
2. Confirm the stable USB identities, receiver baud rates, LiDAR model, sensor
   IP `192.168.2.101`, PC3 host IP `192.168.2.150`, UDP port 2368, calibration,
   return mode, and timestamp source on the current vehicle.
3. Confirm NTRIP authentication and mountpoint, continuous CRC-valid RTCM input,
   NovAtel port0 consumption, and RTK integer FIX from OEM7 `BESTPOS.pos_type`.
4. Confirm OEM7 raw IMU, corrected IMU, `INSPVAX`, uncertainty, and
   `INS_SOLUTION_GOOD`. A stationary single antenna cannot provide an accepted
   yaw merely because RTCM is present.
5. Drive the separately approved alignment maneuver above the configured 3 km/h
   threshold only in a safe area, and verify heading direction against vehicle
   forward motion.
6. Verify the complete TF tree and one authority per edge, especially
   `map -> base_link`, `base_link -> sensor_kit_base_link`, Hesai, GNSS, u-blox,
   and IMU frames.
7. Resolve the observed GNSS/map horizontal and vertical disagreement. Accept
   Lanelet, PCD, projector, GNSS, and vehicle altitude/coordinate conventions as
   one system before seeding localization.
8. Confirm `/sensing/lidar/concatenated/pointcloud` is structurally valid,
   correctly framed, full-rate downstream, visible in RViz through the chosen
   display topic, and received by PC2 through the legacy relay without a second
   LiDAR publisher.
9. Confirm vehicle velocity and steering reports from PC1 are present with the
   expected message types, timestamps, and QoS so distortion correction and gyro
   odometry are not stale.
10. Confirm NDT convergence, EKF output, localization accuracy, and stable
    `map -> base_link` over a sustained stationary and low-speed test.
11. Confirm diagnostics, hazard state, MRM handler/operator heartbeat, and the
    PC1 vehicle command gate agree on MRM ownership. Do not bypass MRM merely to
    make AUTO selectable.
12. Confirm PC1-PC2-PC3 topic discovery, rate, QoS, ROS domain, RMW, time sync,
    and duplicate-node state before adding PC4.
13. Review the command path from planning/control through the vehicle interface
    and physically prove that the intended owner is the only actuator writer.

Only after these gates pass should the team connect PC4, run the stationary
simulation bridge test, and plan a separately supervised AUTO motion test.
