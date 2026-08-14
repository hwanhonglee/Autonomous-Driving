# Lucid Vision camera driver

This ROS 2 component driver publishes images and `CameraInfo` from Lucid Triton GigE cameras
through the Arena SDK. The PC2 deployment uses two physical cameras and deliberately starts each
camera in a separate `component_container` process because Arena system ownership is
process-global in the current implementation.

## Supported PC2 launch

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 launch lucid_vision_driver dual_camera.launch.py
```

The launch loads these production parameter files:

- `param/windshield_cam.yaml`: Windshield camera, serial `222301529`, native prefix
  `/lucid_vision/windshield`, optical frame `camera0/camera_optical_link`, 2x2 binning, and JPEG
  enabled for the active Autoware relay.
- `param/loop_top_cam.yaml`: Loop Top camera, serial `214000332`, native prefix
  `/lucid_vision/camera`, optical frame `traffic_light_camera/camera_optical_link`.

The other `config/` and `param/` files are retained as historical or bounded-test inputs. They are
not loaded by `dual_camera.launch.py` and must not be treated as production calibration or device
ownership records.

The exact pre-edit `copy_org` files for the 2026-08-14 transport update are retained under
`archive/runtime_update_20260814/`. They are deliberately outside `config/` and `param/`, so
`ament_auto_package(INSTALL_TO_SHARE ...)` does not install them as active resources.

## External prerequisite

Install a compatible Lucid Arena SDK before building. The SDK is not vendored in this repository;
`cmake/FindARENA.cmake` locates the system installation.

## Build

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select lucid_vision_driver --symlink-install
source install/setup.bash
```

## Runtime checks

```bash
ros2 topic hz /lucid_vision/windshield/image_compressed
ros2 topic hz /lucid_vision/camera/image_compressed
ros2 topic echo /lucid_vision/windshield/camera_info --once
ros2 topic echo /lucid_vision/camera/camera_info --once
```

Image publishers use sensor-data QoS, so diagnostic subscribers should request best effort when
needed. The active Autoware sensor-kit launch normalizes the Windshield stream to camera slot 0
for generic YOLOX and the Loop Top stream to camera slot 1 for traffic-light recognition.

## Camera settings and calibration

Device serials, frame IDs, acquisition rate, exposure/gain mode, and calibration URLs are defined
in the active parameter files. Calibration URLs use `package://lucid_vision_driver/...` so the
workspace path is portable.

The Windshield calibration file is transport-only and is not approved for metric projection or
camera-LiDAR fusion. Recalibrate intrinsics and measure the vehicle-frame extrinsic after final
mounting before enabling 3D fusion. Revalidate the Loop Top calibration after remounting as well.

See `PC2_V2.0.0.md` for the tested hardware mapping, shutdown changes, validation evidence, and
known limits.
