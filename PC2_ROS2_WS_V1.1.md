# IONIQ EV 308 PC2 ros2_ws v1.1 — Lucid driver record

Date: 2026-08-11 KST

Release tag: `IONIQ_EV_308_PC2_r`

## Source identity

| Item | Value |
|---|---|
| Repository | `https://github.com/hwanhonglee/Autonomous-Driving.git` |
| Historical source branch | `h2_i/IONIQ_EV_307/PC2_nebula/ros2_ws/v1.0` |
| Historical source commit | `ec0971f847c33c2d66e01c6d14035414577817c5` |
| New branch | `h2_i/IONIQ_EV_307/PC2_nebula/ros2_ws/v1.1` |
| Deployment tag | `IONIQ_EV_308_PC2_r` |
| Driver upstream used during development | `autowarefoundation/lucid_vision_driver` `main@2ec2b0d855d62b908cbf81f9ded757f510bf06e5` |

The branch name follows the repository's existing 307 PC2 layout. The new tag identifies the 308
vehicle deployment. This branch must be used with Autoware tag `IONIQ_EV_308_PC2_a`.

The deployment tag is moved to the final reviewed branch commit after the flattened-workspace
checks complete. It therefore includes both the hardware-validated driver fixes and every current
regular package source/configuration file documented below.

The v1.1 branch and `IONIQ_EV_308_PC2_r` tag are published as one parentless root snapshot. The
historical v1.0 commit is retained in this document for provenance only and is not part of the new
branch history.

## Hardware and active contract

The release launches one physically discovered Lucid camera:

| Item | Value |
|---|---|
| serial/model | `214000332`, TRI023S-C |
| MAC / device IP | `1c:0f:af:01:0c:a5`, `169.254.0.11` |
| PC interface | `enp1s0f3`, `169.254.0.1/24`, MTU 9000 |
| camera name | `camera` |
| frame | `traffic_light_camera/camera_link` |
| image | 1920x1200, `bgr8`, 15 Hz |
| compression / rectification | JPEG enabled / rectified publisher disabled |
| exposure / gain / gamma | manual 10000 us / 15 / 0.7 |
| native topics | `/lucid_vision/camera/{image,image_compressed,camera_info}` |

The second discoverable camera, serial `222301529` at `192.168.41.2`, is not launched by this
release. Its host profile is not on the same IPv4 subnet, so discovery must not be restated as a
successful stream test.

Arena SDK is an external prerequisite installed at
`/home/a/ArenaSDK/ArenaSDK_Linux_x64`. The validated library reports version 0.1.95.0. The SDK is
not redistributed in this repository.

## Problems and implemented changes

### Wrong active serial and namespace

The historical snapshot selected a test parameter file and camera-specific namespace that did not
match the current publisher contract. The launch now reads `param/loop_top_cam.yaml`, constructs
container `camera`, node `arena_camera_node_top`, and publishes through the driver's absolute
`/lucid_vision/camera/*` topic scheme.

### Device setup was brittle

The handler now:

- waits 2000 ms for enumeration and logs model/serial/IP/MAC;
- rejects a missing requested serial explicitly;
- provides detailed CreateDevice and StartStream diagnostics;
- treats unsupported/non-writable optional register settings as warnings;
- clamps FPS to the camera node's writable range;
- uses `get_enable_gain_auto()` for gain instead of mistakenly copying exposure-auto;
- centralizes stream ownership in the handler.

Hardware binning register programming from the v1.0 implementation was removed. That is harmless
for the active 1x1 setting but means non-1 binning is not accepted as validated behavior.

### Shutdown race and SIGSEGV

Core-dump inspection proved the Arena callback was still entering ROS publisher code after global
context invalidation while the main thread waited in `StopStream`. A destructor-only fix could not
close that race.

The final lifecycle is:

1. register an `rclcpp::Context` pre-shutdown callback with a weak handler reference;
2. block new image callbacks and wait for the in-flight count to reach zero;
3. stop the stream while ROS publishers and context are still valid;
4. remove the pre-shutdown registration on component unload;
5. repeat the idempotent stop path safely from the destructor;
6. deregister the SDK callback;
7. delete the callback object;
8. destroy the device;
9. close the Arena system.

Start/stop is serialized with a mutex. Exceptions are caught at the SDK callback boundary so a ROS
publish exception cannot unwind through vendor code.

### Message-header correctness

The final commit review found CameraInfo reading `img_msg.header` after `img_msg` had been moved into
the raw publisher. The release instead copies the original callback-local header, preserving the
same stamp/frame contract without depending on moved-from state.

The launch also removes duplicate flip keys and forwards `image_horizontal_flip` and
`image_vertical_flip` once from YAML.

## Active parameters and calibration

`param/loop_top_cam.yaml` selects:

```yaml
camera_name: camera
frame_id: traffic_light_camera/camera_link
pixel_format: bgr8
serial_no: 214000332
fps: 15
horizontal_binning: 1
vertical_binning: 1
enable_rectifying: false
enable_compressing: true
use_default_device_settings: false
exposure_auto: false
exposure_target: 10000
gain_auto: false
gain_target: 15
gamma_target: 0.7
```

The intrinsic file is 1920x1200, plumb-bob, with:

```text
K = [1049.27299628392, 0, 960.63658008225,
     0, 1046.72708005708, 607.72577511556,
     0, 0, 1]
D = [-0.0167435496, 0.02651248057, 0.00062208466, 0.00072085524, 0]
```

It is a provisional copy of the 2025-04-23 `camera_top` calibration and is strongly but not
cryptographically/factory-proven to belong to serial 214000332. It passed the stationary
rectification/frame test. Recalibrate after remount and before publishing quantitative camera
accuracy results.

The current URL is deliberately deployment-specific:

```text
file:///home/a/ros2_ws/src/lucid_vision_driver/config/loop_top_214000332_1920x1200.yaml
```

Deploying to a different filesystem location requires changing the URL and retesting.

## Verification evidence

### Build

- the final v1.1 candidate built successfully in a clean, separate colcon build tree;
- the earlier installed final-shutdown library used by the full PC2 acceptance had SHA-256
  `b38659ec42ae378718d6354d78d6aafd951df58fab1092d419ea748ebc5e7df3`;
- all Arena/GenICam dynamic dependencies resolved.

### Final post-review real-camera test

The candidate itself was launched on isolated ROS Domain 219 for three cycles after the moved-header
and duplicate-flip cleanup. Each cycle:

- enumerated both physical cameras and created serial 214000332;
- reached `StartStream done`;
- delivered CameraInfo width 1920, height 1200, frame
  `traffic_light_camera/camera_link`;
- stopped with SIGINT and launch exit code 0;
- emitted all six required completion markers: callback gate, StopStream, deregister, delete,
  DestroyDevice, CloseSystem;
- produced zero segmentation-fault, invalid-pointer, context-invalid, process-died, or error-line
  matches;
- left no Lucid/component process and did not create a persistent camera route.

Local evidence log hashes:

```text
cycle 1  8ffbeb3f7bd537749828383df5457572843aaa4ff3cacb4f591da7999e4f6cb2
cycle 2  a59952e30da1bf6d9cc8111222afb5fad577f3338f3f357af0f5ff9ed6c75ddc
cycle 3  80d5cd5af60d753aceabcfa8d44712abb38a9547f878379ae0fdafb5b3375a8d
```

The preceding full `run_autoware` cycle measured the camera and CameraInfo at approximately
15.15/15.01 Hz, 1920x1200, matching frame/stamps, and then shut down cleanly. That full test is
documented in the Autoware companion branch.

### Package-test qualification

The last recorded Lucid `colcon test` run predates the final shutdown edits and is not clean:
159 tests, 0 errors, 147 failures, 7 skipped; six of eight CTest lint/schema targets failed.
Failures are concentrated in copyright, cpplint, flake8, lint_cmake, uncrustify, and xmllint.
Therefore v1.1 claims successful compilation and real-device lifecycle testing, not a clean unit/
lint suite.

## Known limitations for follow-up

- `pixel_format` is parsed but the conversion path always treats input as one-channel BayerBG and
  applies `cv::COLOR_BayerBG2BGR`; device pixel format is not set from that parameter.
- the callback does not explicitly reject incomplete Arena frames before reading image data.
- JPEG compression and optional rectification run synchronously in the Arena callback. Rectification
  is disabled in this release, but JPEG cost remains in the acquisition path.
- teardown intentionally favors avoiding use-after-free; constructor/SDK-exception paths are not
  fully RAII-based and require future cleanup.
- if callback deregistration itself throws, the current exceptional cleanup path has not been
  fault-injection tested.
- only serial 214 and 1x1 binning are validated.
- daylight exposure 10000/gain15 is a scene baseline, not a universal night/rain setting.

## Committed runtime file manifest

| Path | SHA-256 |
|---|---|
| `src/lucid_vision_driver/include/arena_camera/arena_camera.h` | `b914e8d8271172283743f3546671060ddf7416be0ffa61bf1d3d706171704103` |
| `src/lucid_vision_driver/include/arena_camera/arena_camera_node.h` | `ff7510f27ebee4629a5e25afbff8d65b992bd9460d97d98ab02938304d538db9` |
| `src/lucid_vision_driver/include/arena_camera/arena_cameras_handler.h` | `6a5dadd4f94c9706f0e27abadb6742a7dab92aa78b08eecd3a68c93627dd8930` |
| `src/lucid_vision_driver/src/arena_camera.cpp` | `db87bf0b5ccb48505d1243d45170b8e9ae10763fc02f24d836b58d5364c3084a` |
| `src/lucid_vision_driver/src/arena_camera_node.cpp` | `4821457b025999c070e46012e25979609876aed778db72ad56cea11cfb885f09` |
| `src/lucid_vision_driver/src/arena_cameras_handler.cpp` | `c6a3dad8fd1001f0ea0375b9f6125f310d960e377f933227bee300a8fb4d69b3` |
| `src/lucid_vision_driver/launch/test_node_container.launch.py` | `60d62b3ba2ede4cea4764aa2f1bfca486b3d27a81fe45ad792b0a0457dddcfb3` |
| `src/lucid_vision_driver/param/loop_top_cam.yaml` | `a5f0a08b338c9b19ab269ec3054ce61b27e1ff3d46a41a755cc84d72d65623a2` |
| `src/lucid_vision_driver/config/loop_top_214000332_1920x1200.yaml` | `f5b00c2e29ef987ab8dd407818c33b26aa1b751762458b97bcc39b638aa884f2` |

## Complete flattened workspace snapshot

The branch now carries a complete snapshot of every regular source file under
`/home/a/ros2_ws/src`: one colcon package and 26 package files. The source checkout's nested
`lucid_vision_driver/.git` directory is intentionally not copied. The package is committed as
ordinary repository content, not as a submodule; there is no `.gitmodules` file and no mode-160000
Gitlink.

Three files retain the already hardware-validated v1.1 versions instead of the local development
workspace versions:

- `src/lucid_vision_driver/launch/test_node_container.launch.py`
- `src/lucid_vision_driver/param/loop_top_cam.yaml`
- `src/lucid_vision_driver/src/arena_camera_node.cpp`

Every other package file mirrors the current workspace source, with trailing whitespace at the end
of four YAML files normalized so the committed diff passes Git whitespace checks. Root
`persist.json` also carries the current stream-selector entries for serials `212401044` and
`214000332`.

Six old target-only backup copies were removed because they are not present in the current source
workspace: `config/test (copy_org).yaml`, `include/arena_camera/arena_camera_node (copy_org).h`,
`launch/test_node_container.launch (copy).py`, `param/test.param (copy).yaml`,
`src/arena_camera_node (copy_org).cpp`, and `src/arena_camera_node (copy_roi).cpp`.

## Active and inactive configuration files

The validated runtime path remains:

```text
launch/test_node_container.launch.py
  -> param/loop_top_cam.yaml
  -> config/loop_top_214000332_1920x1200.yaml
```

The following workspace files are included for source completeness but are inactive because the
validated launch does not reference them:

- calibration/config candidates: `config/test.yaml`, `config/test_0416.yaml`,
  `config/test_calibrated.yaml`, `config/test_e2e.yaml`, and `config/test_full_res.yaml`;
- parameter candidates: `param/loop_top_cam (copy_org).yaml`,
  `param/loop_top_cam_cloudy.yaml`, `param/test.param.yaml`, `param/test_e2e.param.yaml`, and
  `param/wind_shield_cam.yaml`.

These inactive files preserve local experiments and alternate camera settings. Their presence is
not a validation claim; several refer to other serials, resolutions, pixel formats, or exposure
profiles.

## Generated and local-only exclusions

The flattened snapshot excludes nested Git metadata and all generated or machine-local content:

- workspace `build/`, `install/`, `log/`, `bin/`, and `obj/` trees;
- Python `__pycache__/` and `*.pyc` files;
- compiled objects, libraries, executables, and crash cores;
- the stale `src.zip` archive, which contains nested Git metadata and predates current source files;
- the external Arena SDK, camera captures/JPEGs, rosbag data, and machine-specific test logs.

These exclusions do not omit a ROS package or regular package source file.
