# PC3 Vehicle Interface and No-Actuation Audit

Audit date: 2026-08-10 KST
Test objective: stationary four-PC communication test with zero physical actuation.
Audit method: initial read-only source/config and stopped-graph inspection, followed by an authorized rebuild and five exact-alias, hardware-disabled PC3 start/stop cycles. No test command was published and no physical sensor or vehicle interface was enabled.

## Result

| Scope | Result | Meaning |
|---|---|---|
| Current stopped PC3 runtime | PASS-at-rest | No vehicle interface, command writer, CAN process, ROS command graph, or open serial device was observed. |
| Pre-patch PC3 launch defaults | BLOCKED | Vehicle-interface and system/MRM paths default ON. The selected sample interface is empty, but MRM command publishers can reach a remote PC1 gate on the shared domain. |
| HH_260810 patch | APPLIED / BUILT | Vehicle interface defaults OFF and MRM command-producing nodes are gated OFF while system monitoring remains ON. |
| PC3 hardware-disabled stability | PASS, 5/5 | Five stable exact-alias cycles exited with launcher code 0 and left no bad shutdown signature or orphan process. |
| Four-PC no-actuation proof | NOT ESTABLISHED | PC1/PC4 command sinks, bridge directions, live publisher/subscriber counts, CAN state during a four-PC launch, and hardware-enabled paths are not yet validated. |

The pre-change snapshot is:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810
```

## Current runtime evidence

Read-only inspection found:

- ROS daemon stopped before and after discovery;
- no persistent Autoware, vehicle-interface, PACMod, SocketCAN, bridge, MRM, planning, control, or component-container process;
- no CAN interface from `ip -details link show type can`;
- no relevant loaded CAN module in the inspected state;
- `/dev/ttyACM0` and `/dev/ttyUSB0` through `/dev/ttyUSB2` present but not open according to `fuser`;
- no persistent ROS nodes, command publishers, or command subscribers during two-second no-daemon discovery.

This proves only that PC3 was inert at audit time. It does not predict what an unpatched launch would create.

## Pre-patch local vehicle path

The pre-patch top launch defaulted the vehicle interface ON and forwarded that flag into the vehicle launch:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:29
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:65
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:74
```

The generic vehicle launch always starts robot-state publication and conditionally includes the selected interface:

```text
/home/a/autoware/src/universe/autoware.universe/launch/tier4_vehicle_launch/launch/vehicle.launch.xml:13
/home/a/autoware/src/universe/autoware.universe/launch/tier4_vehicle_launch/launch/vehicle.launch.xml:27
```

For the effective `sample_vehicle`, the interface file contains only `vehicle_id` and no nodes:

```text
/home/a/autoware/src/vehicle/sample_vehicle_launch/sample_vehicle_launch/launch/vehicle_interface.launch.xml:1
/home/a/autoware/src/vehicle/sample_vehicle_launch/sample_vehicle_launch/launch/vehicle_interface.launch.xml:4
```

Therefore the selected local model statically provides no:

- physical command subscriber;
- CAN/PACMod writer;
- raw command converter;
- engage publisher;
- local vehicle status reader.

This is strong evidence for the current `sample_vehicle` selection, but it is not a durable safety boundary: changing `vehicle_model` while `launch_vehicle_interface=true` could include an actuator-capable interface.

## PC3 MRM command path

Pre-patch system launch unconditionally included comfortable-stop and emergency-stop operators plus the MRM handler:

```text
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:81
/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:111
```

The emergency-stop operator consumes the normal control command and publishes an emergency control command:

```text
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/launch/mrm_emergency_stop_operator.launch.py:38
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/launch/mrm_emergency_stop_operator.launch.py:41
```

Its source creates the publisher/subscriber and generates deceleration/zero-steering commands:

```text
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/src/mrm_emergency_stop_operator/mrm_emergency_stop_operator_core.cpp:31
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/src/mrm_emergency_stop_operator/mrm_emergency_stop_operator_core.cpp:43
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/src/mrm_emergency_stop_operator/mrm_emergency_stop_operator_core.cpp:101
/home/a/autoware/src/universe/autoware.universe/system/mrm_emergency_stop_operator/src/mrm_emergency_stop_operator/mrm_emergency_stop_operator_core.cpp:148
```

Active parameters specify `-3.0` acceleration and `-3.0` jerk:

```text
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml:3
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/system/mrm_emergency_stop_operator/mrm_emergency_stop_operator.param.yaml:5
```

The MRM handler publishes emergency gear and hazard-light commands:

```text
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/launch/mrm_handler.launch.xml:10
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/launch/mrm_handler.launch.xml:17
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/src/mrm_handler/mrm_handler_core.cpp:45
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/src/mrm_handler/mrm_handler_core.cpp:67
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/src/mrm_handler/mrm_handler_core.cpp:110
/home/a/autoware/src/universe/autoware.universe/system/mrm_handler/src/mrm_handler/mrm_handler_core.cpp:147
```

The comfortable-stop operator publishes planning velocity-limit commands:

```text
/home/a/autoware/src/universe/autoware.universe/system/mrm_comfortable_stop_operator/launch/mrm_comfortable_stop_operator.launch.py:38
/home/a/autoware/src/universe/autoware.universe/system/mrm_comfortable_stop_operator/launch/mrm_comfortable_stop_operator.launch.py:42
```

Historical proof that both MRM operator components actually started exists at:

```text
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:54
/home/a/.ros/log/2026-05-27-14-33-50-207045-a-2721/launch.log:57
```

## Cross-PC actuation risk

PC1's vehicle command gate statically accepts the emergency control, gear, and hazard-light inputs and produces vehicle-domain outputs:

```text
/home/a/autoware/src/universe/autoware.universe/launch/tier4_control_launch/launch/control.launch.xml:91
/home/a/autoware/src/universe/autoware.universe/launch/tier4_control_launch/launch/control.launch.xml:114
```

Consequently, an empty local PC3 sample interface does not prove four-PC no-actuation. On a shared ROS domain, PC3 MRM publishers can affect a PC1 or bridged command sink even though PC3 itself has no CAN writer.

## Stateful CAN helper

The alias itself does not invoke `/home/a/scripts/start.sh`, but shell history shows it was often run before `run_autoware`.

The helper configures and raises CAN0/CAN1 at 500 kbit/s:

```text
/home/a/scripts/start.sh:3
/home/a/scripts/start.sh:6
```

It also grants mode `777` to tty/video devices and the Docker socket:

```text
/home/a/scripts/start.sh:8
/home/a/scripts/start.sh:10
```

The interfaces were absent during this audit. Do not run this helper for the stationary test unless a later, separately authorized procedure proves that raising those interfaces is necessary and non-actuating.

## Applied HH_260810 controls

The active patch:

1. keep `launch_vehicle=true` for `robot_state_publisher` and the sensor TF model;
2. set `launch_vehicle_interface=false` by default;
3. add a `launch_mrm` gate, default false at the PC3 top level;
4. apply the gate only to comfortable-stop operator, emergency-stop operator, and MRM handler;
5. retain non-command system monitoring, duplicate-node checking, diagnostics, and hazard-status conversion;
6. leave perception, planning, control, and API outside the PC3 top-level include graph;
7. keep RViz off for the lightweight stationary test.

The top-level defaults are directly visible at:

```text
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:17
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:35
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:46
/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:50
```

The MRM gate is forwarded at `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml:93` and applied to the three command-producing includes at:

```text
/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:85
/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:91
/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml:112
```

## Post-patch PC3 shutdown validation

Five stable exact-alias runs used the patched defaults: vehicle interface OFF, MRM OFF, localization OFF, sensing hardware OFF, and RViz OFF. The authorized harness recorded launcher exit code 0 for all five runs and found no bad shutdown signature or orphan process:

```text
/home/a/.ros/log/2026-08-10-19-56-09-947244-a-37623/launch.log
/home/a/.ros/log/2026-08-10-20-00-04-431231-a-39099/launch.log
/home/a/.ros/log/2026-08-10-20-00-16-066567-a-39672/launch.log
/home/a/.ros/log/2026-08-10-20-00-58-921599-a-40260/launch.log
/home/a/.ros/log/2026-08-10-20-01-09-770034-a-40803/launch.log
```

This is strong PC3 startup/shutdown evidence for the tested defaults. It is not a four-PC command-path proof: no simultaneous PC1/PC4 subscriber/bridge graph, CAN counters, or physical writer state was captured, and `launch_hw=true` was not tested or accepted.

## Required command-path proof

The stationary test may be marked PASS only after live discovery proves all of the following:

| Item | Required observation |
|---|---|
| PC3 physical writer | No vehicle-interface, PACMod, SocketCAN transmitter, CAN bridge, or equivalent writer node/process. |
| PC3 command publishers | No PC3 publisher for `/system/emergency/control_cmd`, `/system/emergency/gear_cmd`, or `/system/emergency/hazard_lights_cmd`. |
| Command subscribers | No physical or bridged actuator-capable subscriber on any PC for control, gear, hazard, engage, or raw vehicle command topics. |
| CAN state | CAN interfaces remain absent/down unless separately approved; no TX process and no unexpected counters. |
| Engage | No engage/autonomous-mode publisher, service call, or state transition. |
| Bridge | PC4 bridge directions exclude commands to a physical actuator endpoint. |
| Vehicle state | Any retained velocity/control-mode/status source is proven read-only. |
| Stop behavior | Ctrl+C removes the exact launch process group and all child nodes without broad kill commands. |
| Stability | The five PC3-only hardware-disabled cycles passed; repeat with the authorized four-PC graph and verify no orphan nodes, writers, bridges, or CAN interfaces. |

For each command topic, record `ros2 topic info <topic> -v`, publisher node/PC, subscriber node/PC, type, QoS, and bridge direction. Topic absence on PC3 alone is insufficient if another PC owns a sink.

## Stationary operating constraints

- Vehicle physically stationary and secured.
- Do not run `/home/a/scripts/start.sh`.
- Do not start a manual CAN bridge or `run_bridge` alias.
- Keep PC1 physical vehicle interface and PC4 actuator-capable bridge paths stopped or domain-isolated.
- Do not publish, echo, replay, or service-call control/gear/hazard/engage commands as a test stimulus.
- A read-only vehicle-status source may be enabled only after its implementation is audited independently from any writer.

## Rollback and stop criteria

Immediately stop the exact test launch and return to the current stopped state if any of these appears:

- CAN0/CAN1 becomes UP unexpectedly;
- any vehicle command writer or bridge starts;
- an MRM command publisher exists despite `launch_mrm=false`;
- a physical command subscriber appears on PC1, PC3, or PC4;
- an engage/autonomous-mode transition occurs;
- the launch cannot shut down cleanly.

After stopping, verify no residual ROS nodes, component containers, vehicle interfaces, CAN/bridge processes, open serial devices, or CAN interfaces remain. Use the dated pre-HH snapshot for file rollback; do not restore the historical top-level `(copy_org)` wholesale because it would also restore modules outside PC3 ownership.
