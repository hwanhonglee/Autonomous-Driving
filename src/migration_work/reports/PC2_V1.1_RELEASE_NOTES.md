# IONIQ EV 308 PC2 Autoware v1.1 release notes

Date: 2026-08-11 KST

Host role: PC2 (`nebula`) — camera, GPU perception, and perception visualization

Release tag: `IONIQ_EV_308_PC2_a`

## 1. Source and version identity

This release is a vehicle migration, not a claim that the historical repository already contained
an IONIQ EV 308 branch.

| Item | Value |
|---|---|
| Repository | `https://github.com/hwanhonglee/Autonomous-Driving.git` |
| Historical source branch | `h2_i/IONIQ_EV_307/PC2_nebula/autoware_universe/v1.0` |
| Historical source commit | `e047d5f1143ff9f8b3b83204e9e83d121da47cb1` |
| New branch | `h2_i/IONIQ_EV_307/PC2_nebula/autoware_universe/v1.1` |
| New deployment tag | `IONIQ_EV_308_PC2_a` |
| Companion driver branch | `h2_i/IONIQ_EV_307/PC2_nebula/ros2_ws/v1.1` |
| Companion driver tag | `IONIQ_EV_308_PC2_r` |

The branch name preserves the repository's existing 307 PC layout and increments its version. The
new lightweight tag identifies the actual 308 deployment target. The two workspaces remain separate
because the repository's historical layout stores Autoware and `ros2_ws` on separate branches.

## 2. Required PC ownership

The launch was narrowed to one owner per major function.

| Host | Owner role | PC2 dependency |
|---|---|---|
| PC1 (`192.168.9.2`) | mission/planning/control/API; vehicle status and separately managed CAN | consumes PC2 objects and traffic signals |
| PC2 (`192.168.9.110`) | Lucid camera, TLR, generic YOLOX, CenterPoint and perception/fusion, RViz | this release |
| PC3 (`192.168.9.7`) | LiDAR, GNSS/IMU, map, localization and TF | supplies LiDAR, map and timestamped transforms |
| PC4 | simulation/bridge/research data path | must not duplicate PC2 official publishers |

PC2 does not own vehicle interface, planning, control, map, localization, system services, or CAN
write. These modules are disabled both by top-launch defaults and by final arguments appended by the
guarded helper.

## 3. Problems found and corrections

### 3.1 Wrong or ambiguous launch ownership

The historical PC2 top launch included general sensing and perception and could overlap with PC3.
It also depended on a pointcloud component container that was not locally created. Multiple PCs
could launch RViz and duplicate sensing drivers.

Resolution:

- created the uniquely named `pc2_perception_pointcloud_container` locally;
- kept only the PC2 Lucid camera enabled in the sample sensing launch;
- left LiDAR, IMU, GNSS and vehicle-velocity driver includes disabled on PC2;
- set vehicle/system/map/localization/planning/control/API/vehicle-interface defaults to false;
- kept sensing, perception, generic YOLOX and RViz in the same top-level launch;
- disabled RViz respawn and prohibited a separately started RViz or camera launch.

### 3.2 Camera hardware and topic names did not match the live vehicle

The source audit found historical serial `212401044` and `/lucid_vision/camera_top/*` names, while
live Arena discovery proved two different cameras. The operational loop-top camera is serial
`214000332`, device `169.254.0.11`, on `enp1s0f3`. Its current driver publishes
`/lucid_vision/camera/*`.

Resolution:

- bound the active driver launch to `param/loop_top_cam.yaml` and serial `214000332`;
- used frame `traffic_light_camera/camera_link`;
- corrected camera-info, raw, and compressed relay source names to `/lucid_vision/camera/*`;
- disabled the direct raw relay by default, leaving the TLR decompressor as the only normalized
  `image_raw` publisher;
- added the provisional 1920x1200 intrinsic file associated with the 2025-04-23 calibration set;
- selected a bounded daylight baseline: 15 Hz, exposure 10000 us, gain 15, gamma 0.7, JPEG on,
  per-frame rectification off.

The intrinsic is operationally plausible and passed the stationary frame contract, but its serial
provenance is not a factory certificate. Recalibration is mandatory after a camera remount and is
recommended before quantitative vision evaluation.

### 3.3 DDS used the wrong physical interface

PC2 has two interfaces on addresses related to the ROS subnet. CycloneDDS automatic selection could
choose the wrong L2. IP ping alone did not prove ROS discovery.

Resolution:

- pinned process-local CycloneDDS to `enp0s31f6` with the modern `<Interfaces>` URI;
- required Domain 10, `ROS_LOCALHOST_ONLY=0`, and `rmw_cyclonedds_cpp`;
- validated route source `192.168.9.110` toward PC3 before launch;
- bound NetworkManager profiles to their interface names;
- kept Wi-Fi/internet on a different interface and default route.

### 3.4 Camera route ownership was not deterministic

Two link-local camera networks are simultaneously active. A broad `169.254.0.0/16` route can select
the wrong interface. Persisting test routes would silently modify the vehicle configuration.

Resolution:

- the helper validates the exact camera/DDS profiles and saved-route baselines;
- it creates only `169.254.0.11/32` metric 42762 on `enp1s0f3` for the launch lifetime;
- it never uses `nmcli connection modify` and never changes the camera IP;
- it removes only the exact route it owns and verifies saved routes remained unchanged;
- PID, lock, process group, and route cleanup execute on normal exit, SIGINT, and SIGTERM.

SIGKILL or power loss cannot run a shell trap. The route is non-persistent and therefore disappears
after interface reconnect or reboot.

### 3.5 Lucid shutdown could crash the component container

Core-dump inspection showed an Arena callback thread entering an `rclcpp` publisher validity call
after global context shutdown, while the main thread waited in `Arena::Device::StopStream()`.
Destructor-only cleanup was too late.

Resolution is in companion tag `IONIQ_EV_308_PC2_r`:

- register an `rclcpp::Context::add_pre_shutdown_callback` using a weak handler reference;
- gate new callbacks and wait for in-flight callbacks before context invalidation;
- serialize start/stop and make stop idempotent;
- prevent exceptions crossing the vendor callback boundary;
- perform `StopStream -> DeregisterImageCallback -> delete callback -> DestroyDevice -> CloseSystem`;
- retain the same path for component unload and normal destruction.

### 3.6 `topic_tools relay` aborted during shutdown

The distro package was topic_tools 1.1.1. Upgrading to 1.1.2 fixed its discovery-timer race, but an
in-flight serialized publish could still throw `publisher's context is invalid` after SIGINT.

Resolution:

- flatten and commit the complete official topic_tools tag `1.1.2` source, commit
  `0fc927b7c0af0aaffae34a947b6ea4a7f9f97c94`, in a user-space overlay;
- add a context guard around relay publish;
- suppress `RCLError` only when that context is shutting down and rethrow it while ROS is valid;
- force package and dynamic-library resolution to the audited overlay;
- verify `librelay_node.so` SHA-256 before every launch;
- verify the flattened source manifest before building and leave `/opt/ros/humble` unchanged.

The full upstream source, local source patch, source manifest, and reproducible build helper are
committed. Generated build/install trees and binaries are intentionally ignored.

### 3.7 TensorRT resources were destroyed in the wrong order

CenterPoint owns two deserialized TensorRT engines. Its wrapper reset the runtime before the engine,
causing two `mEngineCounter`/runtime destruction errors on every shutdown.

Resolution:

```text
context -> engine -> serialized plan -> runtime
```

The corrected package built successfully and passed the focused two-engine shutdown launch without
TensorRT runtime errors. The clean 86-test artifact predates the final destructor edit and is not
presented as a post-fix unit-test result.

The active PC2 component launch also carries the local default `centerpoint` instead of the v1.0
branch's `centerpoint_tiny`. This is separate from the destructor fix: it preserves the full model
used by the validated PC2 run and has a larger GPU/resource footprint.

### 3.8 Generic YOLOX existed but was not an operational pipeline

In LiDAR-only mode, generic YOLOX had no permanent downstream subscriber and uses lazy image
subscription. It therefore appeared as a node but did not continuously infer. The physical stream
was named camera1 while one-camera fusion consumed internal slot0.

Resolution:

- default PC2 perception mode to `camera_lidar_fusion`;
- keep the physical TLR namespace `camera1/traffic_light`;
- map that stream to internal fusion slot0;
- launch generic `yolox-tiny` from normalized `image_raw`;
- publish `/perception/object_recognition/detection/rois0`;
- wire `rois0` and camera info to ROI-pointcloud, ROI-cluster and detected-object fusion consumers.

This generic eight-class YOLOX is distinct from the traffic-light-specific fine detector and car/
pedestrian classifiers.

### 3.9 YOLOX emitted invalid far corners

The EfficientNMS path clamped only the near corner and calculated width/height from unclamped far
corners. Fusion consumers could receive ROIs extending outside the 1920x1200 image.

Resolution:

- clamp left/top and right/bottom independently to the source image;
- require right >= left and bottom >= top;
- derive width/height from the clamped corners;
- verify ten live messages and every ROI against image bounds.

### 3.10 LiDAR merger argument typo

`input/lidar_ml/objects ` contained a trailing space in the argument name. It was corrected to
`input/lidar_ml/objects` so the active merger receives the configured detector output.

## 4. Final `run_autoware` pipeline

```text
run_autoware alias
  -> migration_work/scripts/run_pc2_autoware.sh
     -> environment / duplicate / network / route / binary checks
     -> ros2 launch autoware_launch autoware.launch.xml
        |-> Lucid serial 214000332
        |   |-> /lucid_vision/camera/image_compressed
        |   |   -> relay -> decompressor
        |   |   -> /sensing/camera/camera1/traffic_light/image_raw
        |   `-> /lucid_vision/camera/camera_info -> relay
        |-> traffic-light map detector, fine detector and classifiers
        |-> generic YOLOX -> /perception/object_recognition/detection/rois0
        |-> three camera-LiDAR fusion consumers
        |-> CenterPoint / object perception pipeline
        `-> RViz
```

No separate `ros2 launch lucid_vision_driver ...`, YOLOX launch, or RViz command is part of the
operator procedure.

The helper appends these arguments after all user-supplied arguments, so they cannot be overridden
from the alias:

```text
launch_vehicle:=false
launch_system:=false
launch_map:=false
launch_localization:=false
launch_planning:=false
launch_control:=false
launch_api:=false
launch_vehicle_interface:=false
```

## 5. Result and exact acceptance boundary

PC2 standalone pipeline result: **PASS** for launch integration, camera payload, generic YOLOX
payload, permanent fusion subscriptions, TLR component initialization, RViz ownership, local
no-actuation boundary, and clean shutdown.

Not yet proven:

- nonempty, scene-correct camera-LiDAR fused objects with live PC3 data;
- final traffic-light semantic messages with map and connected timestamped TF;
- PC1 callback-level receipt and response under a sustained three-PC run;
- final five consecutive cycles while PC1 and PC3 remain online;
- PC4 bridge integration, time sync, duplicate audit, and research-data recording;
- road-driving safety or authorization.

PC3 must provide live `/sensing/lidar/top/pointcloud_before_sync`,
`/sensing/lidar/top/pointcloud_raw_ex`, `/map/vector_map`, `/map/pointcloud_map`, `/tf`, and
`/tf_static`. Endpoint counts alone are insufficient: payload rate, frame IDs, timestamps and
`map -> base_link -> sensor_kit_base_link -> traffic_light_camera/camera_link` must be measured.

See `PC2_V1.1_TEST_REPORT.md` for exact evidence and known failures.

## 6. Installation and operator procedure

Expected checkout locations on PC2:

```text
/home/a/autoware
/home/a/ros2_ws
```

Build the Lucid workspace after checking out `IONIQ_EV_308_PC2_r`, then build the changed Autoware
packages. Build the guarded relay overlay once:

```bash
/home/a/autoware/src/migration_work/scripts/build_topic_tools_overlay.sh
```

The interactive alias must point to the guarded helper:

```bash
alias run_autoware='/home/a/autoware/src/migration_work/scripts/run_pc2_autoware.sh'
```

Normal launch:

```bash
run_autoware
```

Standalone bounded validation:

```bash
REQUIRE_PC3_INPUTS=0 \
  /home/a/autoware/src/migration_work/scripts/validate_pc2_cycles.sh 1
```

Three-PC validation is allowed only after PC1 and PC3 publishers remain active:

```bash
REQUIRE_PC3_INPUTS=1 \
  /home/a/autoware/src/migration_work/scripts/validate_pc2_cycles.sh 5
```

The current validator's PC3 gate checks advertised publishers. The operator must additionally run
payload-rate and TF lookup checks described in the test report before calling the system end-to-end
ready.

## 7. Complete source snapshot and exclusions

The Lucid test/cloudy/e2e/windshield profiles and local `copy_org` reference files are preserved in
the companion branch so the working package is not partially uploaded. They remain inactive and
must not be mistaken for the serial-214 `run_autoware` profile.

All current editable Autoware and driver packages are included, with nested repositories flattened
to ordinary files. Generated build/install output, extracted binary-package duplicates, old bulk
logs, rosbag data, temporary images, Python caches, and nested `.git` metadata are excluded because
they are not unique source packages. See `PC2_V1.1_COMPLETE_SOURCE_SNAPSHOT.md` for exact counts,
paths, and commit-time checks.
