# PC3 sensor and hardware inventory

Audit snapshot: **2026-08-10 19:45-19:47 KST**
Post-audit restore/build/staged-run update: **2026-08-10**
Host: **`a`**
Original audit mode: read-only and unprivileged. No serial device was opened, no sensor was queried or configured, no process or service was started/stopped, and no launch/configuration file was changed during that audit. The separately authorized post-audit source restore, targeted build, launch-path hardening, and staged runs are recorded below.

## Evidence labels

- **VERIFIED** — directly observed in current enumeration, status output, counters, or an identified local file/log.
- **INFERRED** — multiple observations support the conclusion, but the device itself was not queried.
- **UNVERIFIED** — not provable within the read-only/unprivileged audit boundary.

## Host platform

| Item | Status | Current evidence | Bring-up implication |
|---|---|---|---|
| Computer | VERIFIED | Neousys Technology **Nuvo-8108GC Series**, hostname `a` | Treat this host as PC3. |
| OS/kernel | VERIFIED | Ubuntu 22.04.5 LTS; Linux `6.8.0-52-generic`; x86-64 | Matches a ROS 2 Humble-era host, but kernel/module compatibility must still be checked for third-party drivers. |
| CPU | VERIFIED | Intel Xeon E-2176G, 6 cores/12 threads, 3.70 GHz nominal | Sufficient enumeration only; no performance benchmark was run. |
| RAM/swap | VERIFIED | 62 GiB RAM (about 58 GiB available at snapshot); 486 MiB swap, unused | No memory pressure was present during the audit. |
| Storage | VERIFIED | Samsung SSD 970 PRO 512GB, 476.9 GiB block capacity; root on ext4 | Capacity was enumerated; filesystem health and write endurance were not tested. |
| GPU hardware | VERIFIED | PCI `10de:2488`, NVIDIA GA104 / GeForce RTX 3070 LHR | GPU is physically enumerated. |
| GPU driver | VERIFIED blocker | `nvidia-smi` could not communicate with the driver; no `nvidia`/`nouveau` module was listed; NVIDIA 570 packages were in dpkg state `iU` (unpacked, not configured); PCI output had no `Kernel driver in use` for the GPU | Do not rely on CUDA/TensorRT/GPU perception until package configuration, kernel module loading, and `nvidia-smi` are repaired and reverified. |

## Sensor and peripheral inventory

| Asset | Status | Current evidence | Identity/role limits | Required gate |
|---|---|---|---|---|
| Hesai LiDAR network path | VERIFIED | `enp0s31f6` is `UP,LOWER_UP`, carrier 1, **100 Mb/s full duplex**, MTU 1500, address `192.168.2.150/24`; neighbor `192.168.2.101` was `REACHABLE` at MAC `ec:9f:0d:00:63:99`; a read-only two-second counter sample increased by 15,089,560 bytes and 12,168 packets (**60.358 Mb/s ingress**) | This verifies a high-rate live ingress path and a live `.101` neighbor. It does not by itself decode or identify the packets. The five clean post-fix cycles were hardware-disabled and do not accept live Hesai operation. | Bind exactly one PC3 LiDAR driver only after confirming packet source/port and sensor model; retain `launch_hw=true` for the separately authorized live gate. |
| Hesai Pandar64 identity | INFERRED | Current Autoware sensor-kit launch names model `Pandar64`, sensor `192.168.2.101`, host `192.168.2.150`, UDP data port `2368`, frame `hesai_top`; the separate Hesai SDK config uses `.101`, UDP `2368`, PTC `9347`, real-time source type 1; the intended live Nebula configuration retains `launch_hw=true` | Packet capture failed with `Operation not permitted`, and the sensor was not actively queried. Current model, firmware, return mode, PTP state, and UDP source/port are therefore UNVERIFIED. | Obtain an authorized passive capture or vendor status query during a controlled maintenance window. Do not apply old settings blindly or treat hardware-disabled lifecycle tests as live acceptance. |
| NovAtel GNSS/SPAN USB device | VERIFIED | USB `09d7:0100`, Hexagon NovAtel GPS/GNSS/SPAN sensor, serial `BMHR21430182H`; three serial ports enumerate as `ttyUSB0-2` | USB identity is proven; launch description text calling it `Novatel_pwrpak7_E-1` is not independent model proof. Current firmware, antenna state, baud, INS alignment, and output logs are UNVERIFIED. | Retain the configured persistent port below; authorize runtime only after the remaining permission and receiver-mutation gates are cleared. |
| NovAtel driver port | VERIFIED mapping and configuration | `/dev/serial/by-id/usb-NovAtel_Inc._NovAtel_GPS_Receiver_BMHR21430182H-if00-port1 -> ../../ttyUSB1`; current `gnss.launch.xml` now defaults `oem7_port_name` to that exact by-id path | This is the stable replacement for enumeration-fragile `/dev/ttyUSB1`. The mapping/configuration were verified without opening the device. | Keep the by-id path; do not regress to a numbered tty path. |
| NovAtel serial permission | VERIFIED blocker | `/dev/ttyUSB0-2` are `root:dialout`, mode `0660`; user `a` is not in `dialout`; `dialout:x:20:` has no listed members | The audit intentionally did not attempt to open the device. | Durable fix: add `a` to `dialout`, then fully log out/in or reboot and recheck `id -nG`. Do not use persistent `chmod` workarounds. |
| NovAtel expected serial configuration | VERIFIED historical/current config evidence | Current `gnss.launch.xml` uses the stable by-id port at 115200 baud. All 339 historical OEM7 log endpoint declarations found used its then-name `/dev/ttyUSB1:115200`; the latest successful log (2026-05-27) reported initialization errors=0 and Epson G320N IMU type 41 at 125 Hz | The expected setting is well supported historically, but the receiver's **current** baud/configuration and current attached IMU identity remain UNVERIFIED because the device was not opened. | Do not launch the OEM7 driver as a harmless probe: it begins with `UNLOGALL THISPORT` and sends a new LOG command set. |
| NovAtel software availability | VERIFIED post-restore; pre-restore absence superseded | The original audit found the OEM7 source/install prefixes absent. Post-audit, the source was restored **byte-for-byte** from `/home/a/ros2_ws_original.zip`, and targeted colcon builds of `novatel_oem7_msgs` and `novatel_oem7_driver` succeeded; source and install prefixes are now present | Package/source availability and hardware-disabled container lifecycle are no longer blockers. The five clean cycles did not start OEM7 and therefore do not prove serial access, receiver compatibility, or live GNSS/INS output. | Keep live runtime gated until `dialout` access is active and a controlled receiver-configuration window is approved. |
| NovAtel-derived IMU path | VERIFIED configured path; current data UNVERIFIED | Current `imu.launch.xml` expects `/sensing/gnss/novatel/oem7/imu/data_raw`; historical OEM7 logs advertised `imu/data_raw` under `/sensing/gnss/novatel/oem7` | The hardware-disabled validation cycles intentionally started no OEM7 process, so no current IMU topic/data was expected or proven. | Validate frame, rate, covariance, timestamp source, and stationary bias only during an authorized hardware-enabled run. |
| u-blox GNSS receiver | VERIFIED | USB `1546:01a9`; `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 -> ../../ttyACM0`; node is `root:dialout` mode `0660` | Device presence is proven. Its present antenna/fix state, baud/protocol, and intended redundancy role are UNVERIFIED. Active sensor-kit GNSS defaults to NovAtel, not u-blox. | Assign an explicit primary/backup role before launching; do not start it accidentally as a second fix source. |
| Camera on PC3 | VERIFIED absence of local enumeration; physical/network camera UNVERIFIED | No `/sys/class/video4linux` directory; USB inventory contains no identified camera; PCI inventory contains no camera controller; no camera/ROS sensor process was visible | This does not exclude an unenumerated or Ethernet/GigE camera. Historical source places a Lucid camera role on PC2, not proof of current cabling. | Keep PC3 camera launch disabled unless hardware, IP/serial, and ownership are positively identified. |
| CAN on PC3 | VERIFIED absence of visible interface/controller; physical wiring UNVERIFIED | `ip -details -brief link show type can` returned no interfaces; PCI/USB enumeration showed no identified CAN adapter; no CAN process was visible | A powered-off, disconnected, proprietary, or serial-line CAN device cannot be ruled out solely by enumeration. | Keep all physical vehicle-write paths disabled until interface, adapter, directionality, and safety ownership are proven. |
| USB foot switch | VERIFIED | USB `0c45:7403`, identified by `lsusb` as Microdia Foot Switch | Its operational purpose and any ROS integration are UNVERIFIED. | Leave unassigned until the vehicle operator confirms its function. |

## Network interfaces relevant to bring-up

| Interface | Status | Address/role evidence | Notes |
|---|---|---|---|
| `enp0s31f6` | VERIFIED UP | `192.168.2.150/24`, 100 Mb/s full duplex, MTU 1500; high-rate ingress and neighbor `.101` | Strongly inferred LiDAR network. Default route via `.2.1` exists but neighbor `.2.1` was `FAILED`; do not depend on that gateway without verification. |
| `enp8s0` | VERIFIED UP | `192.168.9.7/24`, MTU 1500; preferred default route via `.9.1`; peers `.9.2`, `.9.12`, `.9.110` visible | Likely four-PC/backbone network; exact PC-to-IP mapping remains UNVERIFIED in this report. |
| `enp11s0`-`enp14s0` | VERIFIED DOWN | Four Intel I211 interfaces, no addresses | No sensor ownership can be inferred while down. |
| `wlx200db04a991e` | VERIFIED UP | Realtek USB Wi-Fi, `192.168.0.4/24` | Provides a third routed network; account for DDS/NTP/firewall exposure. |

## Driver ownership and duplication findings

- **VERIFIED original snapshot:** No running Autoware, ROS 2, Hesai, NovAtel, u-blox, RViz, or CAN process was visible. No UDP listener for 2368/9347 was visible to the unprivileged audit.
- **VERIFIED post-audit restore/build:** OEM7 source was restored byte-for-byte from `/home/a/ros2_ws_original.zip`; targeted colcon builds of `novatel_oem7_msgs` and `novatel_oem7_driver` succeeded; the stable NovAtel by-id port is configured.
- **VERIFIED first hardware-disabled run:** No NovAtel process started, no process held `/dev/ttyUSB1`, and no UDP listener bound ports 2368 or 9347. This is the expected safe result and confirms that hardware gates remained effective; it is not a GNSS or LiDAR functional test.
- **VERIFIED:** The active sensor-kit LiDAR path launches the Nebula `HesaiRosWrapper`, configured with sensor model `Pandar64`, through `common_sensor_launch/nebula_node_container.launch.py`.
- **VERIFIED:** A separate, built Hesai SDK driver remains under `/home/a/ros2_ws/src/HesaiLidar_ROS_2.0`, with its own `launch/start.py` and the same `.101:2368` endpoint.
- **INFERRED risk:** Launching both paths would create competing LiDAR ownership and duplicate/contending packet processing. PC3 must use exactly one LiDAR driver path.
- **VERIFIED:** The GNSS poser is configured to consume `/autoware_orientation`, but source search found no publisher in the audited PC3 tree. Orientation availability is therefore UNVERIFIED and is a localization bring-up gate.
- **VERIFIED resolved shutdown cause:** The staged-run `SIGABRT` was deterministic: Nebula teardown left its decoder thread joinable. The thread-lifecycle fix was applied and the affected code rebuilt.
- **VERIFIED post-fix lifecycle acceptance:** Five valid hardware-disabled exact-alias launch/shutdown cycles completed cleanly with no orphan processes. Evidence directories are `2026-08-10-19-56-09-947244-a-37623`, `2026-08-10-20-00-04-431231-a-39099`, `2026-08-10-20-00-16-066567-a-39672`, `2026-08-10-20-00-58-921599-a-40260`, and `2026-08-10-20-01-09-770034-a-40803` under `/home/a/.ros/log`.
- **UNVERIFIED/unaccepted live gates:** The shutdown result accepts only hardware-disabled exact-alias lifecycle behavior. The intended live Nebula setting remains `launch_hw=true`; live Hesai packets/pointcloud, live NovAtel GNSS/INS, and four-PC time-source/offset behavior remain separately gated and unaccepted.

The machine-readable ownership matrix is in `PC3_driver_ownership.csv`.

## Permission and audit limits

- Passive `tcpdump` on `enp0s31f6` failed: `You don't have permission to capture on that device (socket: Operation not permitted)`.
- `ethtool` is not installed; link speed/duplex were read from the kernel's sysfs attributes instead of detailed PHY diagnostics.
- Serial nodes were inspected only through symlinks, udev/sysfs metadata, file modes, historical logs, and holder checks; the original audit and all hardware-disabled validation cycles did not open them.
- Absence from `ps`, `ss`, PCI, USB, Video4Linux, or `ip link` proves only absence from those visible interfaces at the audit time, not physical absence from the vehicle.
- During the original read-only audit, no active discovery, ping, port scan, sensor command, ROS launch, or hardware reconfiguration was performed. Later exact-alias ROS cycles were separately authorized and kept hardware-disabled.

## Primary evidence paths

- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml`
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/hesai_Pandar_64.launch.xml`
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py`
- `/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml`
- `/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py`
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml`
- `/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/imu.launch.xml`
- `/home/a/ros2_ws_original.zip`
- `/home/a/ros2_ws/src/novatel_oem7_driver`
- `/home/a/ros2_ws/install/novatel_oem7_driver`
- `/home/a/ros2_ws/install/novatel_oem7_msgs`
- `/home/a/.ros/log/novatel_oem7_driver_exe_2970_1779860034926.log`
- `/home/a/.ros/log/2026-08-10-19-56-09-947244-a-37623`
- `/home/a/.ros/log/2026-08-10-20-00-04-431231-a-39099`
- `/home/a/.ros/log/2026-08-10-20-00-16-066567-a-39672`
- `/home/a/.ros/log/2026-08-10-20-00-58-921599-a-40260`
- `/home/a/.ros/log/2026-08-10-20-01-09-770034-a-40803`
