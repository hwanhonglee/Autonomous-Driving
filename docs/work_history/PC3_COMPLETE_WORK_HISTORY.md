<!-- HH_260812 - Record the complete PC3 work history, request traceability, evidence, and handoff. -->

# PC3 Complete Work History

## 1. Purpose and status

This document records the complete PC3 work performed for the IONIQ EV distributed
Autoware system, why each work item was requested, what was changed, how it was
validated, what remains unverified, and what the other PCs must assume when they
integrate with PC3.

It covers both PC3 source workspaces:

- Autoware workspace: IONIQ_EV/PC3/autoware_universe/v2.0.0
- ROS 2 driver workspace: IONIQ_EV/PC3/ros2_ws/v2.0.0

This is a work-history and integration handoff document. It is not a certificate
that the vehicle is ready for autonomous driving. The final source was built and
tested with targeted offline tests, but a controlled post-build vehicle restart,
localization convergence test, and physical AUTO acceptance test remain required.
The vehicle/PC power was unavailable when this shared history was finalized, so no
new live acceptance result is implied here.

The separate PC4 VILS/digital-twin architecture document on this branch defines
the proposed cross-PC simulation contract. This PC3 history does not implement
PC2 fusion or PC4 simulation code.

## 2. Release identity and immutable evidence

| Workspace | Preserved v1.0.0 | Current v2.0.0 | Net change |
|---|---|---|---|
| PC3 Autoware | 16ca930f4a8cfa9be7852dd17a7bffcb01d3415d | 947dda782ce90e1d9768e57ae4337e3cf78eee1b | 93 files, +6,764/-126 |
| PC3 ros2_ws | 39872d589c5f3d056d2fba6950e0bbca3bde3f32 | 22f521ffb7ddeae893f377e226091641bb540efc | 28 files, +1,860/-17 |

Canonical detailed evidence:

- Autoware detailed report:
  https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/migration_work/reports/PC3_v2.0.0_detailed_changes.md
- ROS 2 workspace changelog:
  https://github.com/hwanhonglee/Autonomous-Driving/blob/22f521ffb7ddeae893f377e226091641bb540efc/PC3_v2.0.0_CHANGELOG.md

Autoware v2.0.0 commit sequence:

1. ded0fc8b - Prepare PC3 Autoware v1.1 bring-up.
2. 7a6e140f - Fix PC3 localization initialization and GNSS fallback.
3. 68fd039d - Restore calibrated PC3 C-track GNSS offset.
4. 3eeaa493 - Document persistent PC3 network routes.
5. c64940a6 - Normalize PC3 release branch names.
6. 947dda78 - Finalize PC3 Autoware v2.0.0 release metadata.

ROS 2 workspace v2.0.0 commit sequence:

1. da190430 - Prepare PC3 ROS 2 workspace v1.1.
2. 6aac3b76 - Normalize PC3 ROS 2 release branch names.
3. 22f521ff - Finalize PC3 ROS 2 v2.0.0 release metadata.

The canonical branches are authoritative. Existing IONIQ_EV_307_PC3_a/r and
IONIQ_EV_308_PC3_a/r tags are historical aliases and were not moved. No invented
release-prefixed tags are part of this work.

## 3. Confidentiality and publication boundary

The following are deliberately excluded from this public history:

- Live NTRIP username, password, caster authentication header, and private YAML.
- GitHub personal access tokens or keyring material.
- Build, install, log, cache, bag, and generated binary trees.
- Raw backups that are not required to reproduce the reviewed source.

The tracked NTRIP YAML is a placeholder template only. The operational file stays
outside Git under the user's private configuration directory with owner-only
permissions. This document describes the contract without copying its contents.

Host-level package state, user groups, NetworkManager state, and deployed netplan
are recorded as operations, but they cannot be reproduced by checking out these
Git branches alone.

## 4. PC3 role in the distributed vehicle

PC3 owns or supplies the following functions:

- Physical Hesai LiDAR driver and point-cloud preprocessing.
- NovAtel OEM7 primary GNSS/INS data.
- u-blox degraded GNSS position fallback.
- NTRIP client, RTCM validation, and correction fanout.
- IMU correction path sourced from NovAtel SPAN.
- Point-cloud map and Lanelet-map loading.
- GNSS pose generation, localization initialization, NDT, EKF, and map-to-base
  localization transform.
- PC3 RViz and localization/sensing diagnostics.
- MRM/system components when the PC3 launch_mrm ownership flag is enabled.
- Vehicle-status consumers needed by distortion correction, gyro odometry, and
  localization stop checks.

PC3 does not own PC2 perception, PC1 planning/control/CAN actuation, or PC4
simulation. PC3 publishes the physical cloud and localization/time/TF context
that those PCs consume.

The final PC3 top-level source still omits perception, planning, control, and API
include blocks. Arguments with those names in the launcher do not create those
roles on PC3.

## 5. User-request traceability

### 5.1 Bring-up visibility and missing localization

Request:

- Determine why run_autoware showed maps or preprocessing but no localization.
- Restore the normal launch behavior instead of silently disabling modules.
- Keep RViz visible so each PC's progress can be observed.

Findings:

- A temporary stationary/smoke profile had hardware drivers and localization
  disabled by defaults.
- Processing components could exist without physical GNSS/LiDAR publishers.
- NDT and EKF could not activate without successful localization initialization.

Actions:

- Restored normal PC3 defaults for sensing hardware, localization, vehicle
  interface, MRM, RViz, and RViz respawn.
- Added unique PC3 node/container identities to prevent cross-PC duplicate names.
- Preserved PC3 role separation by continuing to omit PC1/PC2 functional includes.
- Restored the user's run_autoware host alias without hidden disable overrides.

### 5.2 MRM Stop and PC1 vehicle data

Request:

- Explain why AUTO did not move the vehicle and why MRM Stop appeared.
- Restore PC1 vehicle status use without disabling safety handling.

Findings:

- Disabling launch_mrm also removed the MRM handler heartbeat required by PC1
  vehicle_cmd_gate.
- The gate correctly produced emergency braking on heartbeat timeout.
- During PC1 restarts, vehicle velocity/steering publishers disappeared; EKF could
  continue predicting from stale twist and must not be treated as valid localization.

Actions:

- Gated the two MRM operators and handler together as one explicit ownership choice.
- Kept launch_mrm enabled by default.
- Added a no-MRM diagnostic graph only for an explicitly delegated mode; it removes
  the PC1-owned emergency-control-command rate leaf, not the safety monitors.
- Did not disable vehicle_cmd_gate emergency handling and did not inject fake MRM
  heartbeats.

### 5.3 NovAtel, u-blox, NTRIP, RTK, heading, and IMU

Request:

- Use NovAtel as the best GNSS source and u-blox when NovAtel is unavailable.
- Restore the NovAtel source tree, receive RTCM via NTRIP, and target RTK FIX.
- Keep IMU and heading usable and prepare for a future dual antenna.
- Change the kinematic alignment threshold to 3 km/h.

Findings:

- NovAtel binary data could be received while strict receiver initialization
  suppressed all ROS position/INS output after one malformed startup response.
- A raw u-blox position is not equivalent to a calibrated NovAtel vehicle pose.
- NavSatFix status alone cannot distinguish every OEM7 differential, float, and
  integer-fixed solution; BESTPOS.pos_type is the acceptance source.
- A single antenna cannot provide stationary absolute heading. HEADING2 cannot be
  treated as body attitude before baseline/extrinsic calibration.

Actions:

- Added debounced NovAtel-primary/u-blox-fallback selection.
- Added source-bound INSPVAX orientation validation and fail-closed diagnostics.
- Added position-only degraded initialization with broad yaw uncertainty.
- Added the PC3 NTRIP/RTCM client and restored the historical u-blox RTCM contract.
- Changed OEM7 SETALIGNMENTVEL to 0.833333 m/s, equivalent to 3 km/h.
- Retried OEM7 initialization until exact OK instead of accepting binary wakeups.
- Removed duplicate RAWIMUSX and INSPVAS handler registrations.

### 5.4 PC2 has no LiDAR

Request:

- Correct the ownership model because PC2 does not physically own a LiDAR.

Findings:

- PC3 is the physical LiDAR owner.
- PC2 perception expected a historical input topic.

Actions:

- Made PC3 publish the standard base_link cloud on
  /sensing/lidar/concatenated/pointcloud.
- Added a temporary relay to the legacy PC2 input
  /sensing/lidar/top/pointcloud_before_sync.
- Kept a single physical LiDAR publisher and documented the relay as migration
  compatibility, not PC2 sensor ownership.

### 5.5 Invalid PointCloud2 and severe visualization load

Request:

- Explain why the LiDAR cloud was absent or malformed and why RViz was extremely
  slow.

Findings:

- ring_outlier_filter generated compact data but left PointCloud2.row_step at zero.
- RViz was rendering in Mesa software mode because the NVIDIA package upgrade was
  incomplete and the kernel driver was missing.
- RViz subscribed to large/debug clouds, while NDT also consumed significant CPU.

Actions:

- Set row_step to point_step multiplied by width and added a regression test.
- Fixed UDP receive endpoint lifetime and deterministic Hesai/Nebula shutdown.
- Reduced RViz point count/debug displays and capped its target frame rate.
- Repaired the host NVIDIA 570 driver stack and verified hardware rendering.

### 5.6 Localization remained uninitialized despite valid RTK

Request:

- Make run_autoware initialize localization automatically while the vehicle is
  stopped.

Findings:

- PC1 reported longitudinal velocity zero but a constant lateral residual of about
  0.00111 m/s.
- The upstream stopped threshold was 0.001 m/s, so the initializer returned
  "The vehicle is not stopped" every second.
- GNSS, NTRIP, LiDAR, and maps were present; NDT/EKF remained inactive because
  initialization never passed the stop gate.

Actions:

- Added a PC3-only three-axis deadband threshold of 0.005 m/s.
- Preserved the upstream shared checker, the default 0.001 m/s behavior for other
  profiles, and the three-second stationary hold.
- Added boundary and diagonal-velocity tests.
- Added strict rejection of an explicitly unreliable initial NDT/YabLoc result.

### 5.7 C-track projector and historical GNSS subtraction

Request:

- Compare RTK location with the C-track map and recover the prior offset that was
  introduced when the map origin was misunderstood.
- Clarify the native C-track projector name and avoid double correction.

Findings:

- The live external map bundle uses a synthetic 52SCF0 projection coupled to its
  Lanelet coordinates and PCD.
- A separate archived/native C-track bundle uses the intended 52SCF60 projection.
- Changing only map_projector_info would move Lanelet coordinates roughly 61 km
  east and 66 km north relative to the local PCD.
- The old GNSS correction was applied only to pose-with-covariance, leaving plain
  pose and TF inconsistent.
- Removing the vertical correction put the GNSS base pose about 15.7 m above the
  local road cloud.

Actions:

- Restored the measured XYZ correction as validated parameters:
  - subtract X 60966.4679793288 m
  - subtract Y 65973.64540576655 m
  - subtract Z 15.816125382 m
- Applied the correction exactly once to the shared pose before PoseStamped,
  PoseWithCovarianceStamped, and TF publication.
- Added an enable flag so a complete native 52SCF60 map bundle can disable the
  legacy correction.
- Documented that projector YAML, Lanelet map, and PCD are an atomic map bundle and
  must never be mixed independently.

Offline geometry evidence:

- Uncorrected recovered sample: approximately (61.537, -118.539, 1.095).
- Corrected sample: approximately (64.844, -125.262, -14.721).
- The corrected point lies inside Lanelet relation 30447.
- The local road surface was about -15.15 m, giving a plausible base height near
  the configured wheel radius.
- Live post-restart NDT acceptance is still required.

### 5.8 Persistent network routing

Request:

- Stop requiring manual deletion of two wired default routes and manual
  NetworkManager startup after each boot.

Findings:

- The wired sensor/vehicle networks had persistent default gateways and per-link
  DNS entries.
- Wi-Fi should own Internet/default routing; Ethernet should keep only direct
  connected subnet routes.

Actions:

- Added a reviewed netplan template with static wired addresses but no wired
  gateway or DNS.
- Documented systemd-networkd ownership for Ethernet and NetworkManager ownership
  for Wi-Fi.
- Recorded deployed-file permissions, backup, generate/try procedure, and rollback.
- Recorded that the resolver remained a host-level foreign/static configuration.
- Left cold-reboot route persistence as an explicit acceptance test.

### 5.9 Git publication and release normalization

Request:

- Keep v1.0.0 and v2.0.0 separate.
- Record detailed English HH_YYMMDD comments and publish the current PC3 work.

Actions:

- Preserved canonical v1.0.0 branches.
- Published canonical PC3 v2.0.0 branches for both workspaces.
- Added detailed migration, validation, ownership, rollback, and release reports.
- Renamed v1.1-era report filenames to v2.0.0 release names.
- Kept legacy tags historical rather than force-moving them.

## 6. Final PC3 launch and ownership behavior

### 6.1 Top-level launch

Primary files:

- src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
- src/launcher/autoware_launch/autoware_launch/launch/pointcloud_container.launch.py
- src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml
- src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml

Final behavior:

- Sensing driver: enabled by default.
- Localization: enabled by default.
- Vehicle interface: enabled by default.
- MRM system: enabled by default.
- RViz and RViz respawn: enabled by default.
- PC3 generic point-cloud container: independently gated and uniquely named.
- PC3 RViz: uniquely named rviz2_pc3.
- Snap-injected GTK_PATH is removed from RViz environment.
- Perception, planning, control, and API functional includes remain absent from PC3.

### 6.2 MRM and diagnostics

MRM ownership is one coherent group:

- comfortable-stop operator
- emergency-stop operator
- MRM handler

launch_mrm defaults true. If ownership is deliberately delegated elsewhere, the
alternate diagnostic graph removes only the missing emergency-command rate leaf.
It does not turn off the rest of system health monitoring.

MRM recovery can immediately allow an already-generated trajectory command to
pass through PC1's vehicle gate. Therefore any live MRM restoration or fault
injection must begin in MANUAL/PARK with the physical CAN writer controlled.

### 6.3 RViz and processing cost

Reviewed RViz choices:

- Full static point-cloud map display disabled by default.
- Main LiDAR visualization uses the localization downsample cloud, about 1,500
  points per message.
- NDT aligned/debug display disabled.
- Unused camera debug image disabled.
- Target frame rate capped at 20 FPS.
- Vehicle footprint updated for the 2.7 m wheelbase profile.

Host GPU repair changed runtime performance but is not a Git source change.

## 7. GNSS and localization implementation

### 7.1 Position source selector

New package:

- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector

The selector:

- validates finite latitude, longitude, altitude, status, covariance shape, age,
  and optional variance bounds;
- prefers NovAtel while healthy;
- waits before falling back or recovering to avoid source flapping;
- publishes no position when neither receiver is valid;
- publishes selected-source state and diagnostics.

Configured timing:

- primary timeout: 1.0 s
- fallback timeout: 2.0 s
- failure and activation holds: 1.0 s
- primary recovery hold: 5.0 s

Final position contract:

- NovAtel raw fix -> selector primary
- u-blox raw fix -> selector fallback
- selector selected/fix -> GNSS poser best-available position

The final launch remaps both the historical private u-blox fix name and the stock
relative fix name to the common fallback topic. This corrected the earlier live
case where the u-blox process was healthy but the selector saw no messages.

### 7.2 Orientation policy

INSPVAX is accepted only when:

- INS status is INS_SOLUTION_GOOD;
- stamp and values are finite and fresh;
- yaw RMSE is at most 5 degrees;
- roll and pitch RMSE are at most 10 degrees;
- the input age is at most 2.5 seconds.

GNSS poser additionally limits fix-to-orientation pairing to 1.5 seconds and
uses validated INS attitude only for the NovAtel gnss_link source frame. A u-blox
gps-frame fallback cannot inherit stale NovAtel attitude.

Without valid attitude, the position seed:

- starts at identity yaw or holds the last validated yaw;
- updates course heading after at least 1 m of displacement;
- publishes yaw variance 10 rad squared;
- leaves NDT reliability as the acceptance gate.

HEADING2 is diagnostic-only until antenna baseline direction, heading offset,
mounting rotations, and roll behavior are calibrated. A valid fixed HEADING2
sample is not automatically body attitude.

### 7.3 GNSS poser and C-track correction

Changed files:

- autoware_gnss_poser/src/gnss_poser_node.cpp
- autoware_gnss_poser/include/autoware/gnss_poser/gnss_poser_node.hpp
- autoware_gnss_poser/config/gnss_poser.param.yaml
- autoware_gnss_poser/schema/gnss_poser.schema.json
- autoware_gnss_poser/README.md

The C-track correction is parameterized, finite-checked, and applied once to the
shared output pose. When the native 52SCF60 map bundle is used atomically, the
correction must be disabled.

### 7.4 Pose initializer

PC3-specific behavior:

- 0.005 m/s three-dimensional stopped deadband.
- unchanged three-second stationary history.
- strict rejection of an explicitly unreliable initial match.
- corrected stale-GNSS age calculation using now minus message stamp.

The generic package default remains 0.001 m/s so other vehicle profiles do not
silently inherit the PC3 noise accommodation.

### 7.5 NDT and EKF

Localization consumes:

- base_link LiDAR cloud on /sensing/lidar/concatenated/pointcloud;
- GNSS pose/initialization source;
- corrected IMU;
- PC1 vehicle velocity and steering topics.

NDT/EKF must remain inactive until initialization succeeds. A high-rate EKF
output during missing vehicle input is not sufficient evidence of valid pose.

Carried-forward NDT tuning present in the v2 diff includes:

- step 0.1
- resolution 2.5
- maximum iterations 35
- six threads
- regularization disabled
- critical execution threshold 200 ms
- skip limit 10
- transform score threshold 3.2
- nearest-voxel score threshold 2.1
- no-ground scoring disabled
- map update distance 35 m
- localization input voxel size 1.2 m
- whole-map leaf size 0.7 m

These tunings predated the HH_260810/HH_260811 repair and need live performance
acceptance; they must not be presented as newly optimized values.

## 8. LiDAR and point-cloud implementation

### 8.1 Topic and ownership contract

PC3 owns the physical Pandar64 path and publishes:

- standard output: /sensing/lidar/concatenated/pointcloud
- frame: base_link
- temporary PC2 compatibility relay:
  /sensing/lidar/top/pointcloud_before_sync

The launch propagates the sensing hardware gate, host configuration, container
name, output topic, and output frame without introducing a second physical driver.

### 8.2 PointCloud2 row-step repair

ring_outlier_filter compacted the cloud to one row but did not set row_step.
The repair assigns:

    row_step = point_step * width

A regression test verifies the actual faster-filter output contract.

### 8.3 UDP lifetime repair

The async receive endpoint was local to a function and could expire while an
operation still referenced it. The endpoint is now an object member. The receive
path no longer overwrites the configured local bind endpoint.

### 8.4 Hesai and Nebula shutdown repair

- Hesai hardware stop closes the pending UDP receiver and waits for I/O exit.
- Destruction is idempotent.
- A loopback shutdown regression repeatedly constructs, starts, and destroys the
  interface.
- Nebula ROS stops packet producers, pushes a null queue sentinel, and joins the
  decoder thread.
- This removes the deterministic std::terminate/SIGABRT shutdown path.

The requested/declared return mode must still be compared with the physical sensor
at acceptance time; an earlier live device reported a different active return mode.

## 9. IMU path

No separate physical IMU driver was added.

The intended path is:

1. NovAtel OEM7 SPAN publishes raw IMU.
2. autoware_imu_corrector publishes corrected IMU.
3. gyro odometer and localization consume corrected IMU with vehicle twist.
4. gyro-bias estimation also monitors raw OEM7 IMU and localization state.

Physical IMU identity, mounting rotation, timestamps, covariance, and sustained
rate remain live gates.

## 10. PC3 NTRIP client and ROS 2 driver workspace

### 10.1 New pc3_ntrip_client package

The new ROS 2 package provides:

- NTRIP 1.0 and 2.0 requests.
- HTTP 200 and legacy ICY 200 handling.
- incremental HTTP chunk decoding.
- Basic authentication request generation.
- bounded headers, connect timeout, and read timeout.
- exponential reconnect.
- RTCM3 preamble/length parsing and CRC-24Q validation.
- stream resynchronization after corrupt frames.
- validated rtcm_msgs/Message publication.
- correction fanout to NovAtel and the historical u-blox contract.
- sanitized diagnostics and counters.
- POSIX serial injection into the separate NovAtel correction port.
- RAM-only INTERFACEMODE setup without SAVECONFIG.

Known limitations:

- no TLS;
- no periodic GGA transmission;
- a VRS mountpoint requiring GGA may not provide corrections;
- serial setup success confirms write/drain, not an OEM7 OK response;
- no code-enforced exclusive device lock;
- configuration accepts a wider baud range than SerialSink implements.

### 10.2 RTCM topic contract

Validated RTCM frames are published as rtcm_msgs/msg/Message with reliable QoS
and depth 100 on:

- /sensing/gnss/novatel/rtcm
- /rtcm

The preserved PC3 u-blox driver subscribes to absolute /rtcm and calls its receiver
correction-injection method. The stock Humble raw-data logging topic is not an
equivalent replacement.

The current workspace contains and installs the preserved u-blox overlay. Earlier
historical text saying it was absent described an intermediate recreated workspace
and is superseded by the final filesystem/build state.

### 10.3 Credential configuration

Tracked:

- publishable placeholder template;
- documentation explaining how to create a private runtime copy;
- no real endpoint account or password.

Runtime:

- external owner-only YAML outside the repository;
- symlink/regular-file/size/permission/type/range checks;
- placeholder rejection;
- no credential values in diagnostics.

The loader requires no group/other permission bits; 0600 is the deployed mode,
but other owner-only modes such as 0400 can also satisfy the code.

### 10.4 OEM7 initialization and INS fixes

Changed OEM7 behavior:

- SETALIGNMENTVEL 0.833333 for 3 km/h kinematic alignment.
- initialization succeeds only on exact OK.
- binary/non-OK wakeups are retried.
- duplicate RAWIMUSX and INSPVAS handler registrations removed.

Logging caveat:

- intermediate retry warnings omit the response payload;
- the final exhausted-retry error path can still print the last response.

### 10.5 Legacy replay compatibility

Legacy reference bags used gps frame IDs, while the active PC3 configuration uses
gnss_link. Tests now:

1. assert the live UUT frame is gnss_link;
2. normalize only the known legacy reference field;
3. keep strict comparison for all remaining fields.

Covered topics:

- /novatel/oem7/fix
- /novatel/oem7/gps
- /novatel/oem7/insstdev

Reference bags and original-copy files were not rewritten.

## 11. System monitor, GPU, and host recovery

### 11.1 Locale-safe memory diagnostics

The memory monitor parsed English free output while the host locale produced
Korean labels, shifting fields and falsely reporting exhausted memory. Only the
child free process now receives LC_ALL=C. The parent process locale remains
unchanged. A fake localized-free regression verifies the parser.

### 11.2 NVIDIA recovery

Host-only work:

- repaired an interrupted mixed-version NVIDIA package transaction;
- restored the matching NVIDIA 570 kernel/userspace stack;
- rebuilt/installed the kernel module and rebooted into the supported kernel;
- verified RTX 3070 ownership with the NVIDIA driver and nvidia-smi;
- added the runtime user to the serial-device group.

Observed RViz improvement was approximately:

- CPU: 140-190 percent down to about 22 percent.
- RSS: about 1.66 GiB down to about 253 MiB.

This host state is not committed in either source workspace.

## 12. Network persistence

The reviewed source template keeps two direct wired subnets for vehicle/sensor
communication while removing their default routes and DNS authority. Wi-Fi owns
Internet/default routing.

Deployment record:

- Ethernet renderer: systemd-networkd.
- Wi-Fi manager: NetworkManager.
- deployed netplan: root-owned and owner-readable/writable only.
- pre-change backup retained.
- netplan generate and interactive try used before acceptance.
- only the Wi-Fi default route remained in the observed main table.
- direct wired subnet routes remained available.
- the existing static/foreign resolver configuration was recorded, not redesigned.

Final gate: verify the same route ownership after a cold reboot.

## 13. Carried-forward vehicle and sensor parameters

The v2 diff also contains measurements/tuning that predate this repair:

- vehicle wheelbase 2.7 m;
- camera, Hesai, GNSS, and IMU extrinsic changes;
- PC3 LiDAR host address;
- planning preset dynamic-obstacle-stop false;
- the NDT/map downsampling values listed above.

PC3 does not launch planning, so the planning preset should be owned and reviewed
by PC1. Some comments label the calibration/wheelbase as IONIQ EV 307 while the
canonical v2 branch represents the current/308 release. Vehicle-specific physical
measurement is required before those values are accepted for another car.

## 14. Validation evidence

| Area | Result | Limits |
|---|---|---|
| Targeted Autoware build | 14/14 selected packages built | Not a whole-repository clean build |
| GNSS selector/orientation | 20/20 tests passed | Hardware-free |
| NTRIP client | 13/13 tests passed | Fake caster and PTY |
| OEM7 replay | 6/6 launch tests passed | File replay, no receiver |
| Ring row_step | 1/1 passed | Component regression |
| UDP functional tests | 4/4 passed | Loopback |
| Hesai shutdown | 2/2 plus repeated clean cycles | No physical sensor |
| Memory locale | 1/1 passed | Fake localized free |
| Stop deadband | Boundary/noise regression passed | No vehicle motion |
| XML/Python/YAML parse | Passed | Static validation |
| Pre-change manifest | 39/39 matched | Local backup evidence |

The broad test aggregation still reported 33 pre-existing transport style/lint
findings:

- boost_serial_driver: 1
- boost_tcp_driver: 20
- boost_udp_driver: 12

Changed functional UDP tests passed. Do not describe the whole repository as
lint-clean.

Earlier live evidence established at different points:

- NTRIP TCP connection and CRC-valid RTCM.
- NovAtel BESTPOS integer-fixed and good INSPVAX in a successful run.
- LiDAR packet/cloud flow.
- PC1 vehicle status and PC3 localization DDS connectivity.
- MRM heartbeat behavior and vehicle gate safety response.

Those processes predated some final rebuilds/configuration changes. They are useful
root-cause evidence, not final release acceptance.

## 15. Remaining live acceptance gates

Before declaring PC3 ready:

1. Start the final installed PC3 build in MANUAL/PARK.
2. Verify one owner for each NovAtel port, u-blox device, and Hesai UDP socket.
3. Verify NTRIP diagnostics without exposing credentials.
4. Confirm OEM7 BESTPOS.pos_type is integer-fixed when RTK is claimed.
5. Confirm INSPVAX INS_SOLUTION_GOOD, RMSE limits, and forward yaw.
6. Confirm corrected IMU rate/frame/timestamp/covariance.
7. Confirm PC1 velocity and steering rates, QoS, and current timestamps.
8. Confirm the stopped deadband permits initialization only while physically stopped.
9. Confirm initial NDT reliability is accepted before EKF activation.
10. Confirm sustained NDT score, iteration count, skip count, and execution time.
11. Confirm GNSS, NDT, EKF, Lanelet, and PCD overlays agree after correction.
12. Confirm one TF authority per edge and a stable map-to-base_link transform.
13. Confirm PC2 receives only the PC3 LiDAR contract and has no duplicate publisher.
14. Confirm MRM handler/operators and PC1 vehicle gate agree on ownership.
15. Confirm there is exactly one physical CAN command writer.
16. Reboot and confirm wired routes remain direct-only and Wi-Fi owns default routing.
17. Perform any AUTO test only in the closed course with safety driver/E-stop and the
    agreed 3 km/h limit.

No source test in this history proves safe physical actuation.

## 16. Rollback boundary

Rollback choices:

- Source release rollback: use the canonical PC3 v1.0.0 branches.
- GNSS correction rollback: disable the NTRIP launch and preserve receiver RAM-only
  behavior; do not write SAVECONFIG.
- Map rollback: switch projector, Lanelet, and PCD as one bundle; never switch only
  the projector or only the GNSS offset.
- LiDAR compatibility rollback: remove the legacy PC2 relay only after PC2 consumes
  the standard topic.
- MRM rollback: restore a coherent handler/operator owner; never bypass heartbeat
  safety in vehicle_cmd_gate.
- Network rollback: restore the recorded pre-change netplan backup through a local
  console.

## 17. Cross-PC and PC4 handoff

PC3 provides these authoritative inputs to the proposed digital-twin system:

- real vehicle localization and map-to-base_link;
- map/Lanelet/PCD identity and projection convention;
- real vehicle timestamp domain and TF tree;
- physical LiDAR cloud for PC2;
- GNSS/INS/localization diagnostics.

PC4 must not publish competing /tf, /tf_static, /clock, localization, control,
vehicle, or system-authority topics into the vehicle domain. PC4 virtual objects
must use the shared map convention exactly once. In particular, map_projector_info
alone does not describe the legacy C-track correction; the PC4 architecture must
use a reviewed simulator-world-to-Autoware-map transform and validate it against
control points and the live ego pose.

PC2 owns any real-plus-virtual object fusion and remains the single publisher of
the official predicted-object output. PC1 continues to consume the official output
and owns planning/control/CAN policy. The separate VILS document defines the
detailed contract and fault-injection stages.

## 18. Complete changed-file inventory

### 18.1 PC3 Autoware v1.0.0 to v2.0.0

Launcher and configuration:

- src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml
- src/launcher/autoware_launch/autoware_launch/config/localization/ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter.param.yaml
- src/launcher/autoware_launch/autoware_launch/config/localization/pose_initializer.param.yaml
- src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml
- src/launcher/autoware_launch/autoware_launch/config/map/pointcloud_map_loader.param.yaml
- src/launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml
- src/launcher/autoware_launch/autoware_launch/config/system/diagnostic_graph_aggregator/autoware-main-pc3-no-mrm.yaml
- src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
- src/launcher/autoware_launch/autoware_launch/launch/components/tier4_localization_component.launch.xml
- src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml
- src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch (copy_org).xml
- src/launcher/autoware_launch/autoware_launch/launch/pointcloud_container.launch.py
- src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz

Migration evidence:

- src/migration_work/.gitignore
- src/migration_work/HH_260810_CHANGELOG.md
- src/migration_work/config/node_ownership.yaml
- src/migration_work/config/pc3-netplan.yaml
- src/migration_work/config/readiness_checks.yaml
- src/migration_work/config/topic_contract.yaml
- src/migration_work/reports/PC3_HH_260810_changes.md
- src/migration_work/reports/PC3_alias_trace.md
- src/migration_work/reports/PC3_chrony.md
- src/migration_work/reports/PC3_driver_ownership.csv
- src/migration_work/reports/PC3_duplicate_analysis.md
- src/migration_work/reports/PC3_launch_flow.md
- src/migration_work/reports/PC3_map_localization_tf.md
- src/migration_work/reports/PC3_sensor_hardware_inventory.md
- src/migration_work/reports/PC3_start_stop_stability.md
- src/migration_work/reports/PC3_v2.0.0_detailed_changes.md
- src/migration_work/reports/PC3_vehicle_interface_no_actuation.md
- src/migration_work/rollback/PC3_rollback.md

Sensor and transport:

- src/param/autoware_individual_params/individual_params/config/default/sample_sensor_kit/sensor_kit_calibration.yaml
- src/sensor_component/external/nebula/nebula_hw_interfaces/CMakeLists.txt
- src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp
- src/sensor_component/external/nebula/nebula_hw_interfaces/test/test_hesai_hw_interface_shutdown.cpp
- src/sensor_component/external/nebula/nebula_ros/config/lidar/hesai/Pandar64.param.yaml
- src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp
- src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper (copy_org).hpp
- src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp
- src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper (copy_org).cpp
- src/sensor_component/transport_drivers/udp_driver/src/udp_socket.cpp
- src/sensor_component/transport_drivers/udp_driver/test/test_udp_data.cpp
- src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/hesai_Pandar_64.launch.xml
- src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py
- src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml
- src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml
- src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/package.xml

System and localization:

- src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml
- src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch (copy_org).xml
- src/universe/autoware.universe/localization/autoware_gyro_odometer/config/gyro_odometer.param.yaml
- src/universe/autoware.universe/localization/autoware_pose_initializer/CMakeLists.txt
- src/universe/autoware.universe/localization/autoware_pose_initializer/config/pose_initializer.param.yaml
- src/universe/autoware.universe/localization/autoware_pose_initializer/include/autoware/pose_initializer/stop_check_utils.hpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/schema/pose_initializer.schema.json
- src/universe/autoware.universe/localization/autoware_pose_initializer/src/gnss_module.cpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/src/pose_initializer_core.cpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/src/pose_initializer_core.hpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/src/stop_check_module.cpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/src/stop_check_module.hpp
- src/universe/autoware.universe/localization/autoware_pose_initializer/test/test_stop_check_utils.cpp

GNSS selector:

- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/.gitignore
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/README.md
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/autoware_gnss_failover_selector/__init__.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/autoware_gnss_failover_selector/failover_node.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/autoware_gnss_failover_selector/novatel_orientation_node.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/autoware_gnss_failover_selector/orientation_logic.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/autoware_gnss_failover_selector/selector_logic.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/config/gnss_redundancy.param.yaml
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/config/ublox_fallback_no_persist.param.yaml
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/launch/gnss_redundancy.launch.xml
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/launch/ublox_fallback_driver.launch.xml
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/package.xml
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/resource/autoware_gnss_failover_selector
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/setup.cfg
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/setup.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/test/test_dual_heading_logic.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/test/test_orientation_logic.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/test/test_orientation_node_policy.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/test/test_selector_logic.py
- src/universe/autoware.universe/sensing/autoware_gnss_failover_selector/test/test_ublox_fallback_contract.py

GNSS poser, point cloud, system monitor, and vehicle:

- src/universe/autoware.universe/sensing/autoware_gnss_poser/README.md
- src/universe/autoware.universe/sensing/autoware_gnss_poser/config/gnss_poser.param.yaml
- src/universe/autoware.universe/sensing/autoware_gnss_poser/include/autoware/gnss_poser/gnss_poser_node.hpp
- src/universe/autoware.universe/sensing/autoware_gnss_poser/schema/gnss_poser.schema.json
- src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp
- src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/CMakeLists.txt
- src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/src/outlier_filter/ring_outlier_filter_node.cpp
- src/universe/autoware.universe/sensing/autoware_pointcloud_preprocessor/test/test_ring_outlier_filter_node.cpp
- src/universe/autoware.universe/system/system_monitor/CMakeLists.txt
- src/universe/autoware.universe/system/system_monitor/src/mem_monitor/mem_monitor.cpp
- src/universe/autoware.universe/system/system_monitor/test/src/mem_monitor/free_locale.cpp
- src/universe/autoware.universe/system/system_monitor/test/src/mem_monitor/test_mem_monitor_locale.cpp
- src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml

### 18.2 PC3 ros2_ws v1.0.0 to v2.0.0

Workspace metadata and history:

- .gitignore
- PC3_v2.0.0_CHANGELOG.md

NovAtel changes:

- src/novatel_oem7_driver/src/novatel_oem7_driver/config/std_init_commands.yaml
- src/novatel_oem7_driver/src/novatel_oem7_driver/novatel_oem7_driver/rosbag_comparison.py
- src/novatel_oem7_driver/src/novatel_oem7_driver/novatel_oem7_driver/testutil.py
- src/novatel_oem7_driver/src/novatel_oem7_driver/src/ins_handler.cpp
- src/novatel_oem7_driver/src/novatel_oem7_driver/src/oem7_message_node.cpp
- src/novatel_oem7_driver/src/novatel_oem7_driver/test/bestpos.test.py
- src/novatel_oem7_driver/src/novatel_oem7_driver/test/ins1.test.py

New NTRIP package:

- src/pc3_ntrip_client/.gitignore
- src/pc3_ntrip_client/README.md
- src/pc3_ntrip_client/config/ntrip_config.yaml
- src/pc3_ntrip_client/launch/ntrip_client.launch.py
- src/pc3_ntrip_client/package.xml
- src/pc3_ntrip_client/pc3_ntrip_client/__init__.py
- src/pc3_ntrip_client/pc3_ntrip_client/config.py
- src/pc3_ntrip_client/pc3_ntrip_client/node.py
- src/pc3_ntrip_client/pc3_ntrip_client/ntrip.py
- src/pc3_ntrip_client/pc3_ntrip_client/rtcm.py
- src/pc3_ntrip_client/pc3_ntrip_client/serial_sink.py
- src/pc3_ntrip_client/resource/pc3_ntrip_client
- src/pc3_ntrip_client/setup.cfg
- src/pc3_ntrip_client/setup.py
- src/pc3_ntrip_client/test/test_config.py
- src/pc3_ntrip_client/test/test_fake_caster_pty.py
- src/pc3_ntrip_client/test/test_node_topics.py
- src/pc3_ntrip_client/test/test_ntrip_protocol.py
- src/pc3_ntrip_client/test/test_rtcm.py

## 19. Final outcome

PC3 v2.0.0 restores normal visible bring-up while making PC3 process identities
and ownership explicit. It implements NovAtel-first GNSS with safe degraded
u-blox fallback, validates source-bound INS orientation, adds NTRIP/RTCM handling,
repairs C-track coordinate consistency and localization initialization, restores
a valid and stable Hesai cloud path, and records host GPU/network recovery.

Source publication and offline regression are complete. Controlled post-power
restart, final map matching, sustained localization, MRM/CAN ownership, and
closed-course low-speed vehicle acceptance remain the honest final gates.
