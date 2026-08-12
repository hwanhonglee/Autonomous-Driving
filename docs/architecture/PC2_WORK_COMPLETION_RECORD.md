# PC2 source-audited bring-up and perception integration record

Status date: 2026-08-12 KST

Scope: PC2 only

Document branch: `agent/pc4-digital-twin-integration`

## 1. Purpose and evidence policy

This is the complete PC2 work and handoff record. It explains which instructions were received,
what was inspected, why each change was needed, what source was modified, what was actually
tested, what was published, and what remains unresolved. PC1, PC3, and PC4 are mentioned only
where their interfaces are prerequisites or consumers of PC2 work; their own implementations are
outside this file.

Result labels have strict meanings:

- **PASS — bounded local contract**: only the named contract passed in the recorded PC2 window;
- **BLOCKED — prerequisite absent**: PC2 was ready, but required external data was absent;
- **NOT TESTED / OUT OF SCOPE**: the property was not measured;
- **PROPOSED / NOT IMPLEMENTED**: reviewed future work, absent from the current release.

An endpoint, model load, or a few messages are not promoted to accuracy, safety, production, or
sensor-to-actuation end-to-end claims.

## 2. Instructions received

The work began with a source-audited four-PC Autoware bring-up request. Historical source placed
sensing and perception on PC2 and showed a Lucid driver, but did not prove live hardware, aliases,
IPs, serials, Domain ID, RMW, network topology, or duplicate ownership. The user then instructed
PC2 to:

1. inspect this live PC and physical camera connections before applying historical values;
2. make camera sensing, perception, TLR, YOLOX, and RViz work through one `run_autoware` entry;
3. first finish everything PC2 can test locally, then test PC1/PC3 topics when those PCs run;
4. preserve the stationary/no-actuation boundary on PC2;
5. diagnose MRM-facing integration blockers without making PC2 a control or CAN owner;
6. fix camera stream, shutdown, TensorRT, relay, duplicate-publisher, DDS, route, and NIC issues;
7. use two cameras: Windshield for general object detection and Loop Top for traffic lights;
8. support wired Internet or Wi-Fi Internet without moving vehicle DDS onto a camera/Wi-Fi NIC;
9. preserve all source packages as normal Git files while excluding build/install/log/cache,
   nested Git, submodules, Git LFS pointers, and generated binaries;
10. publish vehicle-neutral PC2 Autoware and ROS 2 workspace v2.0.0 branches with detailed proof;
11. review, but not yet implement, PC2's role in the PC4 digital-twin/vehicle-in-the-loop test.

Work consequently proceeded through source audit, live hardware/network audit, single-camera
recovery, lifecycle stabilization, dual-camera integration, perception validation, Git release,
and PC4 architecture review.

## 3. Audited environment

### 3.1 Compute and software

| Item | Audited value |
|---|---|
| Host | `a`, Neousys Nuvo-8108GC Series |
| OS/kernel | Ubuntu 22.04.5 LTS, Linux 6.8.0-57, x86-64 |
| CPU | Intel Xeon E-2176G, 6 cores / 12 threads |
| Memory | 62 GiB, no swap |
| GPU | NVIDIA GeForce RTX 3070, 8192 MiB |
| NVIDIA/CUDA | driver 595.71.05, CUDA toolkit 12.3 |
| ROS | ROS 2 Humble, `ros-humble-ros-base` 0.10.0 |
| ROS client/RMW | rclcpp 16.0.11, `rmw_cyclonedds_cpp` 1.3.4 |
| DDS environment | Domain 10, localhost-only disabled, CycloneDDS pinned to vehicle NIC |
| Camera SDK | external Lucid Arena SDK 0.1.95 |

Arena SDK, camera firmware, NetworkManager profiles, and models under `/home/a/autoware_data` are
deployment prerequisites and are intentionally not embedded in the source snapshots.

### 3.2 Physical NIC and camera mapping

| Function | PC2 interface/address | Peer/device | Result |
|---|---|---|---|
| Vehicle DDS + current wired Internet | `enp0s31f6`, DHCP `192.168.9.11/24` | PC1 `.2`, PC3 `.7` | direct routes on this NIC |
| Stored vehicle-LAN alternative | `enp0s31f6`, `ROS2` profile `192.168.9.110/24` | PC1 `.2`, PC3 `.7` | no gateway/DNS |
| Windshield Lucid | `enp1s0f0`, `192.168.41.1/24`, MTU 9000 | serial `222301529`, `192.168.41.2` | direct subnet |
| Loop Top Lucid | `enp1s0f3`, `169.254.0.1/24`, MTU 9000 | serial `214000332`, `169.254.0.11` | direct link; owned `/32` during run |

Live Arena discovery identified:

- Windshield: TRI054S-C, serial `222301529`, MAC `1c:0f:af:03:49:78`;
- Loop Top: TRI023S-C, serial `214000332`, MAC `1c:0f:af:01:0c:a5`.

Historical serial `212401044` had no current configuration match. The Windshield host profile was
corrected from `169.254.121.1/24` to `192.168.41.1/24`; the camera's persistent address was not
changed. Both camera profiles have no gateway/DNS and `ipv4.never-default=yes`.

The launcher accepts the audited `.11` DHCP or `.110` static vehicle-LAN source only when routes to
PC1/PC3 use `enp0s31f6`. Internet may use Wi-Fi, but DDS stays on that wired interface. Camera NICs
never become default Internet routes.

## 4. Initial findings and root causes

### 4.1 Launch and ownership

- Historical PC2 and PC3 both included general sensing and RViz, creating duplicate risk.
- The active sample sensor-kit camera include and manual Lucid launch did not share one verified
  topic/configuration contract.
- PC2 perception needed its own pointcloud component container.
- Camera, TLR, generic YOLOX, perception, and RViz had not been proven under one process group.
- PC2 no-actuation intent required source defaults, explicit launch arguments, and runtime proof.

### 4.2 Camera path and calibration

- Two physical devices were discoverable, but the active launch targeted only serial 214.
- Native driver topics and normalized sensor-kit relay sources initially disagreed.
- Historical bags/configs included 1920x1200 images paired with 1600x900 CameraInfo.
- CameraInfo could read a moved-from image header.
- Manual exposure/gain setters ran while auto modes made those settings ineffective.
- CameraInfo used `camera_link` where projection consumers require optical-frame coordinates.
- Two components in one process would each open a process-global Arena system.
- Windshield intrinsic and mounted extrinsic were not production-calibrated.

### 4.3 Lifecycle crashes

An apport core showed the Arena callback thread querying an rclcpp publisher after context
invalidation while the main thread waited inside `Arena::Device::StopStream()->join()`. A normal
node destructor runs too late for this race. Repeated component tests confirmed destructor-only
cleanup was insufficient.

CenterPoint also released its TensorRT runtime before every deserialized engine. The Humble
`topic_tools` relay had separate shutdown races in discovery and in-flight publish paths.

### 4.4 YOLOX and TLR

- Generic YOLOX used lazy subscription and stopped image consumption without an output consumer.
- Camera slot, `image_number`, CameraInfo, and ROI slot were inconsistent.
- TLR nodes could load while map ROI, classification result, and final traffic signal remained
  unproven.
- Camera-LiDAR fusion would be unsafe with provisional Windshield calibration.

### 4.5 DDS and validation

- Multiple eligible NICs caused CycloneDDS to select an interface that could not discover PC1/PC3.
- IP ping was repeatedly mistaken for ROS application discovery/payload proof.
- Sequential graph queries made nominal readiness loops potentially take many minutes.
- A shared Domain cannot be required to become empty when PC2 stops because remote PCs may remain.

## 5. Final PC2 ownership and launch

`run_autoware` resolves to `src/migration_work/scripts/run_pc2_autoware.sh`, which launches the PC2
`autoware.launch.xml`.

Enabled:

- two Lucid cameras;
- PC2-local pointcloud/perception components;
- generic Windshield YOLOX and output diagnostics;
- Loop Top TLR components;
- RViz as a child of the same launch.

Disabled or excluded:

- vehicle interface and physical CAN command ownership;
- foundational system, map, localization, and shared sensor ownership;
- planning, control, and API;
- simulation time and simulation mode;
- camera-LiDAR fusion until calibration acceptance.

Defaults are `launch_sensing=true`, `launch_perception=true`, `launch_yolox=true`, `rviz=true`, and
`perception_mode=lidar`; non-PC2 module flags are false. The helper explicitly reasserts critical
non-PC2 false values after user arguments. This proves the PC2 process-group boundary only, not a
four-PC single-actuator-writer invariant.

## 6. Dual-camera pipeline

### 6.1 Process isolation

`lucid_vision_driver/launch/dual_camera.launch.py` creates two OS processes:

```text
windshield_camera_container -> arena_camera_node_windshield -> serial 222301529
loop_top_camera_container   -> arena_camera_node_loop_top   -> serial 214000332
```

The process boundary avoids opening two Arena systems in one process without a large shared-system
driver rewrite.

### 6.2 Windshield camera0 and generic YOLOX

```text
/lucid_vision/windshield/image
  -> /sensing/camera/camera0/image_raw
  -> /tensorrt_yolox
  -> /perception/object_recognition/detection/rois0
  -> /topic_state_monitor_windshield_yolox_rois

/lucid_vision/windshield/camera_info
  -> /sensing/camera/camera0/camera_info
```

Contract: serial 222, 2880x1860 BGR8, requested 15 Hz, raw-only, automatic exposure/gain, frame
`camera0/camera_optical_link`. The CameraInfo is explicitly transport/2D-test-only. A permanent
topic-state monitor keeps lazy YOLOX active without enabling unsafe 3D fusion. The existing eight
classes remain unchanged; this is not a pedestrian-only model.

### 6.3 Loop Top camera1 and TLR

```text
/lucid_vision/camera/image_compressed
  -> /sensing/camera/camera1/traffic_light/image_raw/compressed
  -> traffic_light_image_decompressor
  -> /sensing/camera/camera1/traffic_light/image_raw
  -> traffic-light fine detector and classifiers

/lucid_vision/camera/camera_info
  -> /sensing/camera/camera1/traffic_light/camera_info
```

Contract: serial 214, 1920x1200 BGR8, requested 15 Hz, JPEG transport, frame
`traffic_light_camera/camera_optical_link`, bounded manual daylight exposure/gain, retained
1920x1200 calibration. The decompressor is the only normalized raw-image publisher.

### 6.4 Frame and fusion policy

Both image/CameraInfo streams now use optical frames. A Windshield link/optical-link description
was added, but its real mounted extrinsic remains unvalidated. Fusion slot 0 is consistently wired
to camera0 image, CameraInfo, and `rois0`; nevertheless `perception_mode=lidar` creates zero camera
fusion components until intrinsic, extrinsic, TF, time alignment, and projection tests pass.

## 7. Reliability changes

### 7.1 Lucid driver

The ROS 2 driver now:

- initializes device, stream, callback, and shutdown ownership state;
- waits for enumeration and prints device model/serial/IP/MAC evidence;
- fails explicitly when a requested serial is absent;
- guards optional registers, clamps writable FPS/binning, and keeps unsupported settings nonfatal;
- uses the gain-auto parameter correctly;
- skips manual exposure/gain writes while auto mode is active;
- preserves a stable local header before moving the image message;
- blocks new callbacks and drains in-flight work before stream stop;
- uses an rclcpp pre-shutdown callback before ROS publisher invalidation;
- serializes start/stop and makes stop idempotent;
- catches exceptions at the SDK callback boundary;
- tears down in order: StopStream, deregister callback, delete callback, destroy device, close
  system;
- uses portable `package://lucid_vision_driver/...` calibration URLs;
- moves inactive historical settings under `archive/legacy_camera_configs`.

### 7.2 TensorRT and ROI

- CenterPoint destruction order is `context -> engine -> plan -> runtime`, eliminating the audited
  runtime-before-engine teardown fault.
- YOLOX clamps published ROI corners to image bounds. The observed sample had no invalid ROI; this
  does not prove every future model output.

### 7.3 Humble relay overlay

Flattened `topic_tools` 1.1.2 source is retained in Autoware. Its relay verifies the exact rclcpp
context before publish and suppresses only shutdown-time `RCLError`, rethrowing errors while ROS is
valid. A source bootstrap builds a user overlay; `/opt/ros` and generated build/install/log output
are not committed.

## 8. Guarded operator workflow

`run_pc2_autoware.sh`:

- validates Domain 10, CycloneDDS, non-localhost mode, and wired NIC selection;
- derives the current audited `.11` or `.110` source without changing profiles;
- validates PC1/PC3 routes and both camera profiles/routes/reachability;
- rejects a second launch, stale guards, standalone Lucid launch, and duplicate publishers;
- builds/verifies the guarded relay overlay;
- installs/removes only its exact nonpersistent Loop Top `/32` route;
- uses a dedicated process group, lock/PID files, and EXIT/INT/TERM cleanup;
- passes non-PC2 module disable arguments explicitly.

`validate_pc2_cycles.sh` adds bounded readiness, payload, safety, and cleanup checks for both camera
processes; publisher cardinality; image dimensions/encoding/frame/stamp/rate; YOLOX graph/samples/
ROI bounds; TLR component readiness; expected zero fusion components; absence of PC2 control/CAN
processes and command publishers; and process/graph/route/guard cleanup. DDS queries are parallel
within a wall-clock deadline, and cleanup targets PC2-owned graph deltas rather than all Domain 10.

Representative commands used:

```bash
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
ip -br -4 address
ip -4 route get 192.168.9.2
ip -4 route get 192.168.9.7
ip -4 route get 192.168.41.2
ip -4 route get 169.254.0.11
IpConfigUtility /list

cd /home/a/ros2_ws
colcon build --packages-select lucid_vision_driver --symlink-install
ros2 launch lucid_vision_driver dual_camera.launch.py --show-args

cd /home/a/autoware/src
./migration_work/scripts/build_topic_tools_overlay.sh
colcon build --packages-select autoware_lidar_centerpoint autoware_tensorrt_yolox \
  --symlink-install
ros2 launch autoware_launch autoware.launch.xml --show-args
REQUIRE_PC3_INPUTS=0 ./migration_work/scripts/validate_pc2_cycles.sh 1
```

Read-only graph probes used direct, process-local DDS settings where possible. Camera persistent IPs
and secrets were never changed or committed.

## 9. Validation results and boundaries

### 9.1 Builds and static checks

- Lucid driver build: PASS;
- CenterPoint and YOLOX builds: PASS;
- source-only topic-tools overlay build: PASS;
- XML, YAML, Python, Bash, xacro, and launch argument expansion: PASS;
- final Autoware and ros2_ws worktrees clean and equal to remote canonical tips.

### 9.2 Camera and YOLOX

| Contract | Observation | Meaning |
|---|---|---|
| Two cameras | approximately 14.9 Hz each standalone | bounded simultaneous transport PASS |
| Windshield | 2880x1860 BGR8, optical frame | transport/message PASS |
| Loop Top | 1920x1200 BGR8, optical frame | transport/message PASS |
| Image/CameraInfo | common ROS header stamp | message contract, not hardware sync |
| YOLOX | 10 `rois0` messages at 11.794 Hz, one object/message, zero invalid ROI | bounded one-scene 2D path PASS |
| Shutdown | both SDK processes completed ordered teardown and exited cleanly | bounded lifecycle PASS |

YOLOX class correctness, false positives/negatives, mAP, distance, weather, 3D pose, tracking, and
planning consumption were NOT TESTED.

### 9.3 Latest retained local cycle

```text
/home/a/autoware/src/migration_work/test_logs/live_camera_cycles/20260812_172914/cycle_1.log
SHA-256: 73e50a6548e304c53e837a813762ccc6a88ee7339fb2e87ff683df2a9dc2c66f
```

This ignored runtime artifact was created after the release commit. It records both cameras,
perception/TLR/YOLOX/RViz launch, ordered camera shutdown, clean child exits, and owned route
removal. It used `REQUIRE_PC3_INPUTS=0`; it is not a distributed semantic test and is not part of
the committed release tree.

### 9.4 TLR

Camera1 compressed transport, decompression, fine detector, car/pedestrian classifiers, ROI
visualizer, and model initialization were present. Map-based ROI and final semantic output were
not proven because valid map/localization/dynamic TF/LiDAR prerequisites were absent.

```text
camera1 transport and TLR model readiness: PASS — bounded local contract
map ROI, signal color/shape, final traffic_signals: BLOCKED — prerequisite absent
planner response to a traffic light: NOT TESTED
```

Historical observation also showed final `traffic_signals` endpoint ambiguity and zero payload;
the next multi-PC test must attribute the sole intended writer by node/GID and measure payload.

### 9.5 Safety scope

The final local cycle observed zero PC2 control/vehicle processes, zero new safety-command
publishers, and zero PC2 `control_cmd` publishers, followed by clean graph/process/route cleanup.
This proves only the PC2 launch boundary, not whole-system actuator safety.

## 10. Complete source modification record

### 10.1 Autoware release delta

From prior published PC2 root `a713e9b70ca5393d3bb2133e9a786f91219f6a70` to canonical v2:

- added `.cspell.json`, `PC2_AUTOWARE_UNIVERSE_V2.0.0.md`, release/integration/network documents,
  and bounded diagnostic probe sources;
- modified `.gitignore`, top PC2 launch, perception-component launch, launch package dependencies,
  sensor-kit camera/sensing launch, sensor-kit description and dependency;
- modified migration README, topic-tools bootstrap, guarded launcher, cycle validator, and YOLOX
  contract probe;
- retained flattened patched `topic_tools` source and metadata;
- deleted runtime `persist.json`, permissive launch copy, and superseded PC2 v1.1 environment,
  path, manifest, release, snapshot, and test reports;
- corrected an inherited license typo and an Eagleye conflict-marker defect required for a clean
  flattened source snapshot.

The CenterPoint TensorRT teardown and YOLOX ROI changes described in Section 7 were performed
during the overall PC2 effort and were already present in the previously published comparison
root. They are therefore part of the current canonical source but are not falsely counted as a
second v2 delta here.

Exact principal paths:

```text
.cspell.json
PC2_AUTOWARE_UNIVERSE_V2.0.0.md
src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml
src/launcher/autoware_launch/autoware_launch/package.xml
src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/camera.launch.xml
src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml
src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/urdf/sensor_kit.xacro
src/migration_work/scripts/build_topic_tools_overlay.sh
src/migration_work/scripts/run_pc2_autoware.sh
src/migration_work/scripts/validate_pc2_cycles.sh
src/migration_work/scripts/probe_yolox_contract.py
src/migration_work/scripts/capture_lucid_frame.py
src/migration_work/scripts/lucid_interface_audit.cpp
src/migration_work/scripts/probe_can_bridge_graph.py
src/migration_work/scripts/probe_outdoor_diagnostics.py
src/migration_work/scripts/probe_pc2_pc3_flow.py
src/migration_work/scripts/probe_pc3_localization_inputs.py
src/migration_work/reports/PC2_V2.0.0_RELEASE_NOTES.md
src/migration_work/reports/PC2_dual_camera_YOLO_TLR_integration.md
src/migration_work/runbooks/PC2_network_and_port_setup.md
```

Additional exact changed/deleted paths in that comparison are:

```text
.gitignore
persist.json                                      (deleted runtime state)
src/migration_work/README.md
src/migration_work/inventory/PC2_V1.1_ENVIRONMENT.md               (deleted)
src/migration_work/inventory/PC2_V1.1_PACKAGE_PATHS.txt             (deleted)
src/migration_work/reference_snapshots/autoware.launch.copy.xml     (deleted)
src/migration_work/reports/PC2_V1.1_COMPLETE_SOURCE_SNAPSHOT.md     (deleted)
src/migration_work/reports/PC2_V1.1_FILE_MANIFEST.md                (deleted)
src/migration_work/reports/PC2_V1.1_RELEASE_NOTES.md                (deleted)
src/migration_work/reports/PC2_V1.1_TEST_REPORT.md                  (deleted)
src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/package.xml
src/tools/image_pipeline/LICENSE
src/universe/external/eagleye/eagleye_util/gnss_converter/README.md
```

### 10.2 ROS 2 workspace release delta

From prior PC2 root `432617f0a12142e8202e18d7a0ebcd2ba7589123` to canonical v2:

- added `PC2_ROS2_WS_V2.0.0.md`, driver v2 record, dual-camera launch, active Windshield param, and
  transport-only Windshield CameraInfo;
- rewrote the driver README and updated CMake/package/Arena discovery metadata;
- modified the Loop Top production param and single-camera diagnostic launch;
- modified all four camera/handler headers and all three implementation files for settings,
  callback, timestamp, and shutdown safety;
- moved five inactive test calibration files and five inactive parameter files into
  `archive/legacy_camera_configs/` without content loss;
- deleted runtime `persist.json` and superseded v1.1 marker.

Production paths:

```text
PC2_ROS2_WS_V2.0.0.md
src/lucid_vision_driver/PC2_V2.0.0.md
src/lucid_vision_driver/launch/dual_camera.launch.py
src/lucid_vision_driver/launch/test_node_container.launch.py
src/lucid_vision_driver/param/windshield_cam.yaml
src/lucid_vision_driver/param/loop_top_cam.yaml
src/lucid_vision_driver/config/windshield_222301529_transport_only.yaml
src/lucid_vision_driver/config/loop_top_214000332_1920x1200.yaml
src/lucid_vision_driver/include/arena_camera/arena_camera.h
src/lucid_vision_driver/include/arena_camera/arena_camera_node.h
src/lucid_vision_driver/include/arena_camera/arena_cameras_handler.h
src/lucid_vision_driver/include/arena_camera/camera_settings.h
src/lucid_vision_driver/src/arena_camera.cpp
src/lucid_vision_driver/src/arena_camera_node.cpp
src/lucid_vision_driver/src/arena_cameras_handler.cpp
```

The exact archived paths are:

```text
src/lucid_vision_driver/archive/legacy_camera_configs/config/test.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/config/test_0416.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/config/test_calibrated.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/config/test_e2e.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/config/test_full_res.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/param/loop_top_cam (copy_org).yaml
src/lucid_vision_driver/archive/legacy_camera_configs/param/loop_top_cam_cloudy.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/param/test.param.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/param/test_e2e.param.yaml
src/lucid_vision_driver/archive/legacy_camera_configs/param/wind_shield_cam.yaml
```

Additional exact ROS 2 metadata paths changed or removed are:

```text
.gitignore
PC2_ROS2_WS_V1.1.md                              (deleted)
persist.json                                     (deleted runtime state)
src/lucid_vision_driver/CMakeLists.txt
src/lucid_vision_driver/README.md
src/lucid_vision_driver/cmake/FindARENA.cmake
src/lucid_vision_driver/package.xml
```

## 11. Git publication and local cleanup

| Workspace | Canonical branch | Canonical tip | Tree |
|---|---|---|---|
| Autoware | `IONIQ_EV/PC2/autoware_universe/v2.0.0` | `44f79b408ccbffb4cad6f56cbee68de841ac38ac` | `69d2655b777954e55b798bbc94318dc016edd1d7` |
| ROS 2 | `IONIQ_EV/PC2/ros2_ws/v2.0.0` | `65515a2fc13266c95a812b11c1df7f7b7f5f184c` | `556c69a60f8ae75e3125cc681b390006c2c3cf15` |

Autoware lineage: parentless source baseline `585d269f`, evidence/probe commit `327e7b3d`, CI/spell
commit `2f2f2090`, PR integration merge `05913140`, canonical merge `44f79b40` (PR #6).

ROS 2 lineage: parentless Lucid baseline `87e2fd49`, integration merge `b0a9ba2b`, canonical merge
`65515a2f` (PR #7). Parentless describes the imported baseline only; canonical tips are two-parent
merges.

All new commits carry DCO sign-off. Autoware DCO, spell, Docker setup, and pre-commit checks passed.
The repository-wide setup-universe job failed independently and was recorded. The ROS tree had no
workflow files, so build/launch validation was local.

The snapshots contain 381 Autoware package manifests and one ROS 2 package manifest, with no
gitlink/submodule, nested `.git`, Git LFS pointer, generated build/install/log/cache, backup,
extracted binary package, or TensorRT engine artifact. `/home/a/autoware` and `/home/a/ros2_ws` are
clean checkouts of their canonical branches. Temporary release worktrees and old workspace folders
were removed from the home directory using recoverable Trash operations.

No neutral canonical v2 tag existed at the status date. Legacy vehicle-number tags are not the
canonical branch tips.

## 12. Known limitations and next PC2 gates

1. Recalibrate Windshield serial 222 at production 2880x1860.
2. Measure production mounted extrinsics and validate image/LiDAR projection.
3. Revalidate Loop Top calibration after remount.
4. Measure hardware timestamp offset, inter-camera/time sync, packet loss, jitter, and long soak;
   common ROS callback stamps are insufficient.
5. Provide valid map, localization, dynamic TF, and LiDAR; prove nonzero TLR semantic output and a
   single attributable final writer.
6. Align the temporary legacy PC2 LiDAR input with the canonical cross-PC pointcloud contract.
7. Enable camera-LiDAR fusion only after calibration, then assess it against ground truth.
8. Measure official `/perception/object_recognition/objects` payload/accuracy and real downstream
   consumption.
9. Replace exact aggregate `/tf` publisher counts with owner/transform/session checks.
10. Repeat start/stop, cable loss, PC loss, clock/QoS/load, and long-duration tests.
11. Add guards against unsafe overrides such as uncalibrated
    `perception_mode:=camera_lidar_fusion` and vehicle-domain `use_sim_time:=true`.

`REQUIRE_PC3_INPUTS=1` currently verifies selected endpoint cardinality, not payload rate, QoS,
frame, transform authority, localization quality, or clock synchronization. The retained flow,
localization, and CAN probes are tools, not completed PASS evidence.

## 13. Future PC4 PG-VILS responsibility

Everything in this section is **PROPOSED / NOT IMPLEMENTED**. Current PC2 v2 contains no PC4
subscriber, validator, fuser, session/sequence manager, TTL cache, provenance sidecar, or injection
mode switch.

The reviewed object-level target requires PC2 to:

1. receive only PC4-native scenario `TrackedObjects` on a namespaced topic in `map`;
2. validate publisher/session, frame, stamp, finite values, map identity, sequence, and TTL;
3. treat each valid message as a full snapshot and retain only the latest snapshot for bounded
   time alignment;
4. keep physical tracking as the main trigger and fail open to physical-only perception;
5. associate overlaps, preserving the physical object's UUID, pose, twist, class, and shape;
6. append only unmatched virtual actors and publish provenance through diagnostics/sidecar data;
7. prevent PC2 physical objects mirrored into PC4 from returning as virtual actors;
8. clear state, disarm injection, and return to shadow on timeout, session change, disconnect,
   malformed input, or map mismatch;
9. remain the sole owner of canonical final predicted objects.

Recommended seam:

```text
physical tracker
  -> /perception/object_recognition/tracking/pc2_physical_objects
PC4 validated full snapshot
  -> PC2 physical-main fuser
  -> /perception/object_recognition/tracking/objects
  -> existing map-based prediction
  -> /perception/object_recognition/objects
```

The current wrapper does not expose all required remaps and the generic merger lacks PC4
session/map/provenance/TTL semantics, so a dedicated gateway/fuser and launch plumbing are needed.
Modes should progress `real_only -> shadow -> inject_stationary -> closed_loop_vils`, defaulting to
shadow with manual arming.

Object-level injection bypasses camera/LiDAR detection, YOLOX, TLR, and possibly tracking or
prediction depending on the message boundary. It can validate message integration and
scenario-specific downstream planning/control response; it cannot validate PC2 sensor perception.
Camera/TLR/YOLO evidence and PG-VILS object-injection evidence are independent, not a single
sensor-to-actuation end-to-end proof.

The shared architecture and exact PC2 fusion rules are in
`PC4_DIGITAL_TWIN_VEHICLE_IN_LOOP.md`.

## 14. Authoritative PC2 release records

Autoware:

- `PC2_AUTOWARE_UNIVERSE_V2.0.0.md`;
- `src/migration_work/reports/PC2_V2.0.0_RELEASE_NOTES.md`;
- `src/migration_work/reports/PC2_dual_camera_YOLO_TLR_integration.md`;
- `src/migration_work/runbooks/PC2_network_and_port_setup.md`;
- `src/migration_work/scripts/run_pc2_autoware.sh`;
- `src/migration_work/scripts/validate_pc2_cycles.sh`.

ROS 2 workspace:

- `PC2_ROS2_WS_V2.0.0.md`;
- `src/lucid_vision_driver/PC2_V2.0.0.md`;
- `src/lucid_vision_driver/README.md`;
- `src/lucid_vision_driver/launch/dual_camera.launch.py`.

## 15. Final status

PC2 now has a source-audited, single-entry dual-camera sensing/perception launch with generic
Windshield 2D YOLOX, Loop Top TLR model readiness, diagnostics, RViz, guarded network/ownership/
cleanup, and bounded local lifecycle evidence. Canonical vehicle-neutral Autoware and ROS 2 source
branches are published and checked out locally.

PC2 has not completed TLR semantics, calibrated camera-LiDAR fusion, distributed perception
accuracy, four-PC closed-loop PG-VILS, or whole-system safety validation. These remain explicit
acceptance gates.
