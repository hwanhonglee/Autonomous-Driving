# PC3 Duplicate Node, Publisher, Container, Driver, and TF Analysis

Audit date: 2026-08-10 KST
Scope: PC3 plus cross-PC duplicate and shared-domain risks relevant to the stationary four-PC test.
Audit method: initial read-only source/config and stopped-graph inspection, followed by an authorized rebuild and five exact-alias, hardware-disabled PC3 start/stop cycles.

## Result

| Scope | Result |
|---|---|
| Current stopped runtime | No persistent duplicate nodes, publishers, containers, drivers, or TF broadcasters observed. |
| Pre-patch launch topology | BLOCKED by confirmed duplicate-risk entry points and historically observed two pointcloud containers. |
| HH_260810 topology | APPLIED / BUILT: generic pointcloud container off, LiDAR flag propagated, one sensor-owned pointcloud container, RViz off, localization off, and MRM off. |
| Hardware-disabled PC3 runtime | PASS, 5/5: one sensor pointcloud-container FQN was observed, the Hesai wrapper explicitly stayed hardware-disabled, every stable shutdown was clean, and no orphan remained. |
| Hardware-on/four-PC uniqueness | NOT ACCEPTED: physical-driver uniqueness, publisher counts/QoS, localization TF authority, and cross-PC ownership remain unresolved. |

The pre-change snapshot is rooted at:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810
```

## Current runtime evidence

- ROS daemon was stopped before and after discovery.
- Safe no-daemon discovery found no persistent Autoware nodes or component containers.
- Only transient CLI-created `/parameter_events` and `/rosout` endpoints were visible.
- Process scans found no active Hesai, NovAtel, u-blox, RViz, map/localization, TF, vehicle-interface, CAN, or bridge process.
- No CAN interface and no listeners on historical Hesai ports 2368, 2380, or 9347 were observed.

This establishes a clean stopped baseline, not launch-time uniqueness.

## Risk matrix

| Risk | Pre-patch evidence | Historical runtime proof | Applied resolution / tested boundary | Still unproven |
|---|---|---|---|---|
| Two pointcloud containers | Top creates `/pointcloud_container`; LiDAR passes literal `pointcloud_container_name` to a second Nebula container. | 2026-05-27 log shows both FQNs. | Generic container is OFF, substitution is corrected, and five cycles show only `/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container` as the pointcloud container. | Localization membership and fault behavior after localization is enabled. |
| Two Hesai driver entry points | Nebula Hesai include plus manual `hesai_ros_driver/start.py`. | Nebula ran on 2026-05-27; manual driver ran and crashed on 2026-06-04. | Driver flag is propagated; exact alias used only the Nebula wrapper, with hardware disabled, and no orphan/manual driver was found after five stops. | Current LiDAR endpoint, one hardware-on publisher, QoS/rate, and exclusion of externally started/manual drivers during four-PC operation. |
| Driver-disable flag bypass | `lidar.launch.xml` hardcodes `launch_driver=true`. | Historical Nebula hardware connection proves the path was active. | Replaced with `$(var launch_driver)`; all five logs say `Hardware connection disabled`. | Exactly-once start and clean stop when `launch_hw=true`; this mode is explicitly unaccepted. |
| Duplicate `map -> base_link` authority | Vector-map TF configured with viewer `base_link`; EKF also publishes `map -> base_link`. | 2026-05-27 log records static `map -> base_link`. | Restore vector-map viewer to `viewer`; leave EKF sole base-link authority. | Live TF graph and authority warnings after EKF activation. |
| GNSS product inconsistency | C_track translation applied only to covariance pose, not pose/TF. | Source was compiled into the prior active build; no current live sample. | Apply the same preserved translation once to the shared base pose before all three outputs. | Live numerical equality and map alignment. |
| RViz duplication across PCs | PC1, PC2, and PC3 historical top launches default RViz on; PC3 pre-patch default is true. | PC3 RViz started in 2026-05-27 log. | PC3 RViz defaults off and was absent from the five tested launches; select at most one explicit GUI owner. | PC1/PC2 live process audit during four-PC test. |
| PC2/PC3 sensing overlap | Source audit shows both roles historically include general sensing. | No simultaneous current graph was available. | PC3 owns LiDAR/GNSS/common sensing; PC2 retains only a physically attached unique camera driver if proven. | Cross-PC topic publisher counts and Lucid camera ownership. |
| GNSS receiver duplication | NovAtel and u-blox devices are both physically present; launch supports both branches. | Historical NovAtel driver ran; no current simultaneous driver graph. | Select NovAtel only and keep u-blox branch inactive. | One fix/IMU source, one namespace, correct device identity, no service/manual second launch. |
| MRM command publishers versus remote gate | PC3 system launches MRM publishers; PC1 gate subscribes to emergency topics. | PC3 MRM components started in 2026-05-27 log. | MRM command-producing nodes are gated off by default; five exact-alias logs do not spawn those components. | Cross-PC publisher/subscriber graph and bridge directions. |
| Stale/orphan processes | Shell history shows repeated driver and Autoware starts; no duplicate-start guard was proven. | Manual Hesai launches and aborted partial Autoware launches exist in logs. | Exact alias plus graceful scoped stop passed five stable cycles with no orphan. | Repeat under authorized hardware-on and four-PC conditions. |

## Pointcloud-container evidence

The pre-patch top launch creates the generic container:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:59
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:63
```

The pre-patch LiDAR launch passes a literal string instead of the variable:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml:38
```

Nebula creates a composable container under its pointcloud-preprocessor namespace:

```text
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py:244
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py:254
```

Historical runtime proof shows:

```text
/pointcloud_container
/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container_name
```

at:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:3
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:58
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:152
```

The generic container later died with exit code `-11`:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:639
```

### Applied one-container design

- Do not refactor shared `common_sensor_launch/nebula_node_container.launch.py` into a loader; it is also used by Velodyne and Robosense launchers.
- Gate the PC3 top generic container OFF.
- Correct `container_name` to `$(var pointcloud_container_name)` in the PC3 Hesai launch.
- Use the resulting sensor-owned container as the localization preprocessing target.

Expected FQN with the default name:

```text
/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container
```

This satisfies the explicit one-container requirement with the smallest PC3-local change. The five hardware-disabled cycles show the expected FQN and no generic `/pointcloud_container`; localization remained OFF, so combined sensing/localization component membership and fault behavior are still untested.

## Hesai wrapper shutdown regression and fix

The first hardware-disabled post-patch cycle exposed a source-level destructor defect, not a sensor-network failure. The pre-fix wrapper started `decoder_thread_` before the `launch_hw` branch, ran an infinite loop, and blocked in a queue with no close operation:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper (copy_org).cpp:72
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper (copy_org).hpp:56
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/common/mt_queue.hpp:54
```

Because the destructor was defaulted while the `std::thread` remained joinable, destruction invoked `std::terminate`. The failing log proves the wrapper was in hardware-disabled packet-subscription mode and then aborted from component destruction:

```text
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:116
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:228
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:231
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:240
/home/a/.ros/log/2026-08-10-19-49-41-365876-a-32964/launch.log:252
```

The rebuilt fix gives the wrapper an explicit destructor, treats a null queue item as a stop sentinel, resets packet producers/callback ownership, enqueues the sentinel, and joins the decoder thread:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp:56
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:72
/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp:102
```

This fix is accepted only for `launch_hw=false`. Hardware-on remains explicitly **UNACCEPTED**: `HesaiHwInterface::SensorInterfaceStop()` is still a stub returning `ERROR_1`, while its destructor only finalizes the TCP driver. UDP callback and hardware teardown order have not been proven:

```text
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:33
/home/a/autoware/src/sensor_component/external/nebula/nebula_hw_interfaces/src/nebula_hesai_hw_interfaces/hesai_hw_interface.cpp:208
```

## Five-cycle post-fix result

All five valid cycles used the exact alias, left sensor hardware disabled, waited until the map/vector-TF and sensing components were loaded, then stopped with Ctrl+C. The launch harness recorded exit code 0 and no bad signature or orphan for every cycle. The log-local shutdown evidence is:

| Cycle | Launch log | Hardware-disabled proof | Gyro clean | Sensor container clean |
|---|---|---|---|---|
| 1 | `/home/a/.ros/log/2026-08-10-19-56-09-947244-a-37623/launch.log` | `:113` | `:187` | `:220` |
| 2 | `/home/a/.ros/log/2026-08-10-20-00-04-431231-a-39099/launch.log` | `:116` | `:189` | `:208` |
| 3 | `/home/a/.ros/log/2026-08-10-20-00-16-066567-a-39672/launch.log` | `:116` | `:189` | `:207` |
| 4 | `/home/a/.ros/log/2026-08-10-20-00-58-921599-a-40260/launch.log` | `:112` | `:178` | `:219` |
| 5 | `/home/a/.ros/log/2026-08-10-20-01-09-770034-a-40803/launch.log` | `:111` | `:198` | `:218` |

The aborted 0.56-second startup-interrupt run at `/home/a/.ros/log/2026-08-10-19-58-34-798961-a-38418/launch.log` is excluded from the five-cycle result. Ctrl+C arrived while Python nodes were still importing and system monitors/components were still loading; its gyro process then exhausted launch's shutdown escalation. The sensor container nevertheless finished cleanly, and five subsequent stable cycles show no gyro or sensor-container shutdown defect. No gyro source patch is justified by that startup-interruption case.

## Hesai duplicate-driver evidence

The Autoware path is:

```text
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml:9
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml:25
/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/hesai_Pandar_64.launch.xml:30
```

The pre-patch line below bypasses the top disable flag:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml:26
```

The independent manual entry point is:

```text
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py:5
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py:9
```

Its current configuration still carries historical IP, ports, frame, and topics:

```text
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml:4
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml:6
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml:33
/home/a/ros2_ws/src/HesaiLidar_ROS_2.0/config/config.yaml:45
```

Historical Nebula operation used `192.168.2.101:2368`:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:152
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:157
```

Historical manual-driver operation and crash is recorded at:

```text
/home/a/.ros/log/2026-06-04-15-27-42-324691-a-33489/launch.log:3
/home/a/.ros/log/2026-06-04-15-27-42-324691-a-33489/launch.log:6
```

These logs do not prove simultaneous execution, but they prove two independent hardware-driver entry points that must never be used together.

## RViz evidence

Pre-patch PC3 RViz defaults true and is included at:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:41
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:110
```

Historical proof that PC3 started RViz is:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:52
```

The applied patch sets PC3 RViz off. Four-PC uniqueness still requires live PC1/PC2/PC4 process inspection.

## TF and GNSS duplication distinction

The pre-patch map/EKF issue is a true duplicate authority for one parent-child pair. The GNSS issue is different: it is inconsistent data generated by one source path. Both must be checked separately:

- TF duplicate check: exactly one authority for `map -> base_link`;
- GNSS consistency check: translated `PoseStamped`, `PoseWithCovarianceStamped`, and GNSS TF have identical position and timestamp semantics.

The applied patch preserves the current C_track translation but applies it once to the shared pose. Runtime map alignment remains unresolved.

## Runtime validation checklist

For the next separately authorized stationary test:

1. Capture `ros2 node list`, `ros2 component list`, and exact process list.
2. Confirm one pointcloud-container FQN and record every loaded component.
3. For every critical topic, record type, publisher count, publisher node/PC, subscriber count, subscriber node/PC, and QoS.
4. Confirm one Hesai hardware wrapper and no `hesai_ros_driver_node` manual process.
5. Confirm NovAtel is the only GNSS driver and u-blox remains inactive.
6. Confirm PC3 RViz is absent.
7. Confirm one `map -> base_link` authority and no TF duplicate warnings.
8. Confirm MRM command publishers are absent.
9. Stop with Ctrl+C and verify all role-owned nodes/containers disappear.
10. Repeat five stable cycles after enabling any new boundary: localization, sensor hardware, or the four-PC graph.

## PASS conditions

- exactly one physical driver per verified sensor;
- exactly one PC3 pointcloud container;
- exactly one owner for each critical topic;
- no duplicate node FQN;
- no duplicate static or dynamic TF authority;
- one selected RViz owner at most, with PC3 off;
- no MRM or vehicle-actuation publisher on PC3;
- no stale process after five shutdowns;
- no use of historical sensor IP/serial settings without current hardware proof.

The hardware-disabled PC3 alias is **PASS, 5/5** for stable startup/shutdown. The duplicate analysis remains **BLOCKED for hardware-on and four-PC launch** until the remaining live checks pass; `launch_hw=true` is explicitly unaccepted.
