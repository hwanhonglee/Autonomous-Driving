# PC2 v1.1 environment and hardware inventory

Snapshot: 2026-08-11 KST. This file records the environment used for validation; it is not a script
that reconfigures hardware.

## Host

| Item | Value |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | `6.8.0-57-generic` |
| CPU | Intel Xeon E-2176G @ 3.70 GHz |
| RAM | 62 GiB |
| GPU | NVIDIA GeForce RTX 3070, 8192 MiB |
| NVIDIA driver | 595.71.05 |
| CUDA compiler | 12.3 (`V12.3.107`) |
| TensorRT development package | `libnvinfer-dev 8.6.1.6-1+cuda12.0` |

## ROS and middleware

| Item | Value |
|---|---|
| ROS distribution | Humble |
| ROS domain | 10 |
| localhost only | 0 |
| RMW | `rmw_cyclonedds_cpp` 1.3.4 |
| CycloneDDS | 0.10.5 |
| DDS interface | `enp0s31f6` |
| Distro topic_tools | 1.1.1, retained under `/opt/ros` |
| PC2 relay overlay | official 1.1.2 plus shutdown guard, user-space only |

The process-local CycloneDDS URI is:

```xml
<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="enp0s31f6"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>
```

## Network layout

| Function | Interface/profile | Host address | Peer/device | MTU |
|---|---|---|---|---:|
| ROS 2 PC network | `enp0s31f6` / `ROS2` | `192.168.9.110/24` | PC1 `.2`, PC3 `.7` | default/1500 |
| windshield Lucid network | `enp1s0f0` / `Lucid Camera Wind Shield` | `169.254.121.1/24` | serial 222301529, device `192.168.41.2` | 9000 |
| loop-top Lucid network | `enp1s0f3` / `Lucid Camera Loop Top` | `169.254.0.1/24` | serial 214000332, device `169.254.0.11` | 9000 |
| internet | `wlx200db04a992d` | DHCP; `172.30.1.2/24` at snapshot | default gateway `172.30.1.254` | default |

All three wired NetworkManager profiles are bound by `connection.interface-name`. Connecting Wi-Fi
changes the internet-facing DHCP address and default route; it does not change the static ROS 2 or
camera addresses. ROS processes must still pin DDS to `enp0s31f6`, because automatic middleware
selection is not guaranteed merely by the route table.

At the snapshot:

```text
192.168.9.2 -> dev enp0s31f6 src 192.168.9.110
192.168.9.7 -> dev enp0s31f6 src 192.168.9.110
169.254.0.11 -> dev enp1s0f3 src 169.254.0.1
```

The windshield camera is Arena-discoverable at layer 2 but its device address `192.168.41.2/24`
does not match host `169.254.121.1/24`. It is not part of this release and must not be called
stream-ready without a separate, authorized network/configuration test.

## Lucid hardware and SDK

| Serial | Model | MAC | Device address | Release role |
|---:|---|---|---|---|
| 214000332 | TRI023S-C | `1c:0f:af:01:0c:a5` | `169.254.0.11` | active loop-top/TLR perception camera |
| 222301529 | TRI054S-C | `1c:0f:af:03:49:78` | `192.168.41.2` | discovered, not launched by PC2 v1.1 |

Arena SDK is manually installed under `/home/a/ArenaSDK/ArenaSDK_Linux_x64`. The resolved Arena
library reports `0.1.95.0`; `libarena.so.0` resolves to `libarena.so.0.1.95`. Arena/GenICam paths
are registered through the system loader configuration.

Active serial-214 image contract:

| Parameter | Value |
|---|---:|
| width x height | 1920 x 1200 |
| frame | `traffic_light_camera/camera_link` |
| format | `bgr8` |
| target rate | 15 Hz |
| JPEG | enabled |
| rectified publisher | disabled |
| exposure/gain | manual, 10000 us / 15 |
| gamma | 0.7 |
| intrinsic fx/fy | 1049.272996 / 1046.727080 |

The calibration URL is intentionally the deployment path
`file:///home/a/ros2_ws/src/lucid_vision_driver/config/loop_top_214000332_1920x1200.yaml`.
Cloning the driver elsewhere requires updating that URL and retesting.

## Workspace overlay order

The guarded helper sources:

```text
/opt/ros/humble/setup.bash
/home/a/autoware/install/setup.bash
/home/a/ros2_ws/install/setup.bash
```

It then prepends the validated topic_tools user-space overlay. The current Autoware install and
Lucid install are symlink builds, so source changes were reflected after the relevant package
rebuilds.
