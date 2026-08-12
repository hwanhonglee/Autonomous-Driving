# PC2 Autoware Universe v2.0.0 release notes

Release date: 2026-08-12 KST

Target branch:

```text
IONIQ_EV/PC2/autoware_universe/v2.0.0
```

Companion branch:

```text
IONIQ_EV/PC2/ros2_ws/v2.0.0
```

## Scope

This source snapshot defines the PC2 sensing and perception role. Its top-level launch owns camera
sensing, perception, traffic-light recognition, generic YOLOX, diagnostics for the YOLO output,
and RViz. Vehicle interface, map, localization, planning, control, API, and physical actuation
ownership are excluded from the PC2 launch.

## Main changes

- Start two Lucid cameras from one `run_autoware` entry point while isolating each camera in its
  own component-container process.
- Bind the Windshield camera to serial `222301529`, normalize it as camera0, and feed its raw image
  to generic YOLOX.
- Bind the Loop Top camera to serial `214000332`, normalize it as camera1, and feed its compressed
  image and CameraInfo to the traffic-light recognition pipeline.
- Use optical frame IDs for projection-aware consumers.
- Keep camera-LiDAR fusion disabled until the Windshield intrinsic and mounted extrinsic are
  validated.
- Keep YOLOX active in lidar-only mode with a permanent topic-state monitor.
- Add fail-closed network, duplicate-publisher, safety, lifecycle, payload, and cleanup validation.
- Vendor the patched ROS 2 Humble `topic_tools` 1.1.2 source as ordinary files and add a source
  bootstrap script; generated overlay files are intentionally not committed.
- Record the current wired/Wi-Fi/DDS/camera NIC layout and safe NetworkManager procedures.

## Validation

- Lucid driver build: PASS.
- XML, YAML, Python, Bash, xacro, and launch argument checks: PASS.
- Two-camera standalone stream: approximately 14.9 Hz for both devices.
- Final guarded local `run_autoware` cycle: PASS.
- Windshield YOLOX: 10 messages at 11.794 Hz, one observed object per message, zero invalid ROI.
- TLR fine detector, car classifier, pedestrian classifier, and ROI visualizer: 4/4 loaded.
- Control/vehicle process count and new safety publisher count: zero.
- Shutdown: process group, guard files, PC2 graph, and owned route cleaned in 1.563 seconds with no
  TensorRT teardown, process-died, camera memory, or relay shutdown error.

Final local validation log:

```text
migration_work/test_logs/live_camera_cycles/20260812_145512/cycle_1.log
```

The raw runtime log is not committed; the measured evidence and SHA are preserved in
`PC2_dual_camera_YOLO_TLR_integration.md`.

## Known limits

- PC3 map, dynamic TF, and LiDAR publishers were absent during the final local cycle. TLR model
  initialization passed, but final semantic traffic-signal payload did not.
- Windshield calibration is transport/2D-test only. Do not enable camera-LiDAR fusion until a
  camera-specific intrinsic and mounted extrinsic pass projection validation.
- Loop Top calibration must be revalidated after any camera remount.
- Arena SDK remains an external system dependency and is not stored in this Git snapshot.
- A cold TLR TensorRT initialization can exceed 90 seconds; the bounded validator therefore uses a
  150-second readiness deadline.

## Reproduction

Build the companion `ros2_ws` first, then Autoware. Build the guarded topic-tools overlay once from
the flattened source tree:

```bash
cd /home/a/autoware/src
./migration_work/scripts/build_topic_tools_overlay.sh
```

The guarded launcher can also bootstrap that overlay automatically if it is missing.

Local PC2 validation:

```bash
REQUIRE_PC3_INPUTS=0 ./migration_work/scripts/validate_pc2_cycles.sh 1
```

Three-PC validation after PC1 and PC3 are ready:

```bash
REQUIRE_PC3_INPUTS=1 ./migration_work/scripts/validate_pc2_cycles.sh 1
```
