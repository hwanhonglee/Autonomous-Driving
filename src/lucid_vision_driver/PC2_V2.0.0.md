# PC2 Lucid camera workspace v2.0.0

Companion branch:

```text
IONIQ_EV/PC2/ros2_ws/v2.0.0
```

## Runtime design

The active launch uses two physical Lucid cameras:

| Role | Serial | Camera IP | Native prefix | Frame |
|---|---:|---|---|---|
| Windshield generic YOLOX input | `222301529` | `192.168.41.2` | `/lucid_vision/windshield` | `camera0/camera_optical_link` |
| Loop Top traffic-light input | `214000332` | `169.254.0.11` | `/lucid_vision/camera` | `traffic_light_camera/camera_optical_link` |

`dual_camera.launch.py` starts one `component_container` process per camera. This is intentional:
the current driver creates an Arena system per component, while Arena system ownership is
process-global. The process boundary avoids opening two systems in one process.

## Driver reliability changes

- Stop accepting new image callbacks before stream teardown and wait for in-flight callbacks.
- Stop the Arena stream in a ROS pre-shutdown callback before the rcl context becomes invalid.
- Make stop and resource release idempotent.
- Preserve CameraInfo headers before moving the raw image message.
- Catch exceptions at the Arena callback boundary.
- Skip manual exposure and gain register writes while automatic modes are active.
- Preserve hardware binning with bounded register values and non-fatal fallback to software
  binning when registers are unavailable.
- Keep acquisition, packet negotiation, packet resend, and stream startup diagnostics explicit.

## Calibration and configuration

- Loop Top uses the 1920x1200 calibration file
  `config/loop_top_214000332_1920x1200.yaml`.
- Windshield publishes 2880x1860 using
  `config/windshield_222301529_transport_only.yaml`.
- CameraInfo URLs use `package://lucid_vision_driver/...`, so checkout location is not hard-coded.
- The Windshield file is provisional and is not approved for metric projection or 3D fusion.

## Validation

- `colcon build --packages-select lucid_vision_driver --symlink-install`: PASS.
- Simultaneous standalone streams: approximately 14.9 Hz for each camera.
- Image and CameraInfo dimensions, optical frame IDs, and common timestamps: PASS.
- Both camera containers completed StopStream, callback deregistration, device destruction, and
  system close, then exited cleanly.
- Full PC2 guarded cycle started both devices and exited with zero camera-memory/process-died error.

## External dependency

Lucid Arena SDK is not included. Install a compatible Arena SDK and expose its libraries through
the system linker configuration before building this package.
