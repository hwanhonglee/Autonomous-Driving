<!-- HH_260810 - Defined the PC1 changes and evidence required for actual VILS testing. -->
# PC1 actual-test changes and data

<!-- HH_260810 - Preserved the immutable PC1 release while separating later safety work. -->
Status: **NO OBJECT-PATH SOURCE CHANGE THROUGH STAGE 4; TEST WRAPPER AND LATER CAN SAFETY WORK REQUIRED**

<!-- HH_260810 - Recorded the exact PC1 rollback identities. -->
## 고정 기준

<!-- HH_260810 - Bound PC1 work to the reviewed Autoware and ROS workspace commits. -->
- Autoware v2.0.0: `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8`
<!-- HH_260810 - Preserved the exact PC1 ROS workspace release for rollback. -->
- ros2_ws v2.0.0: `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34`

<!-- HH_260810 - Restated PC1 authority without assigning PC4 or PC2 responsibilities to it. -->
PC1은 실제 Planning·Control·vehicle interface·CAN 정책 owner이다. PC1은 PC4
`TrackedObjects`를 직접 구독하지 않고 PC2가 유일하게 publish하는 canonical
`/perception/object_recognition/objects`만 기존 방식으로 소비한다.

<!-- HH_260810 - Classified every PC1 change by test stage. -->
## 변경 항목

<!-- HH_260810 - Added one auditable stage decision to each PC1 change row. -->
| Change comment | Stage | 변경 여부 | 조치 |
|---|---|---|---|
| HH_260810 - Kept the physical baseline exactly on the audited release. | 1.5 | Core source 변경 없음 | PC4/gateway 없이 기존 canonical Planning baseline 수집 |
| HH_260810 - Required receive-only status acquisition instead of the historical full CAN bridge. | 1.5–4 | 별도 승인된 test wrapper 또는 동등한 상태 공급원 필수 | sender와 control adapter가 없는 명확한 이름의 receive-only launch를 post-v2 시험 branch에 추가 |
| HH_260810 - Prevented PC4 candidate data from entering PC1 directly. | 2–3 | Object path 변경 금지 | PC2 physical canonical만 소비하고 candidate는 PC2 debug에서만 기록 |
| HH_260810 - Reused the existing canonical Planning boundary in no-actuation fusion. | 4 | Core source 변경 없음 | PC2 sole canonical publisher를 기존 Planning이 그대로 소비 |
| HH_260810 - Deferred PC1 command safety code until physical actuation is separately approved. | 5 | 신규 source 필요 | control_cmd와 cruise/control-request 독립 age watchdog·request-bit clear·exclusive CAN writer/ID allowlist·실차 geometry/speed enforcement |

<!-- HH_260810 - Explained why the historical operator bridge cannot be used for shadow acquisition. -->
기존 `run_bridge`는 단순 DDS bridge나 receive-only logger가 아니다. CAN receiver뿐 아니라
SocketCAN sender와 `twistController2VCU2EPS2ACC_node`를 함께 시작하여 `/to_can_bus`를 통해
물리 CAN TX가 가능하다. 따라서 Stage 1.5–4에는 실행하지 않는다.

<!-- HH_260810 - Proposed a minimal PC1 test-tooling seam without modifying the released launch. -->
### 제안 receive-only 시험 seam

<!-- HH_260810 - Kept the historical operator alias and active launch untouched. -->
- 기존 `run_bridge` alias와 `can_brdige.launch.xml`은 보존한다.
<!-- HH_260810 - Required an explicitly named wrapper that cannot start any sender or adapter. -->
- post-v2 test branch에 `run_vehicle_status_rx_only` 같은 별도 명령과 receive-only launch를 만든다.
<!-- HH_260810 - Limited the receive path to decoded vehicle reports and evidence output. -->
- CAN receiver·decoder·read-only logger만 허용하며 `socket_can_sender_node_exe`와
  `twistController2VCU2EPS2ACC_node` 포함 시 launch를 거부한다.
<!-- HH_260810 - Required graph and kernel counter evidence before accepting the wrapper. -->
- 실행 전/중/후 `/to_can_bus` endpoint 0, sender process 0, `can0` TX counter delta 0을 검사한다.
<!-- HH_260810 - Made one non-actuating status source mandatory before the physical baseline begins. -->
- 위 RX-only profile 또는 vehicle owner가 동등하게 승인한 non-actuating 상태 공급원이 없으면
  velocity/steering/gear/control-mode evidence를 수집할 수 없으므로 Stage 1.5를 시작하지 않는다.

<!-- HH_260810 - Specified exact PC1 source work needed only for a later moving stage. -->
### Stage 5 이전 source 후보

<!-- HH_260810 - Required runtime provenance before selecting the control-to-CAN source file to patch. -->
1. 실행 중인 PC1에서 `ros2 pkg prefix ros2_socketcan`, resolved executable/library path, build/install
   symlink, process executable와 SHA-256을 기록하여 Autoware mirror와 ros2_ws overlay 중 실제 runtime
   source를 확정한다. mirror 경로만 수정해 runtime이 바뀐다고 가정하지 않는다.
<!-- HH_260810 - Added command freshness protection to the proven active control-to-CAN converter. -->
2. provenance로 확정한 active `twistController2VCU2EPS2ACC_node.cpp`에 마지막 정상
   `control_cmd` receive time과 monotonic age watchdog을 추가하고 그 overlay를 rebuild한다.
<!-- HH_260810 - Added an independent age and validity gate for the locally cached request source. -->
3. `/current/cruise_mode_status` 또는 실제 active control-request source의 마지막 정상 receive time,
   monotonic age와 validity를 `control_cmd`와 독립적으로 검사한다.
<!-- HH_260810 - Prevented either stale input from retaining an actuation request. -->
4. `control_cmd` 또는 control-request source 중 하나라도 stale/invalid이면 cached steering/acceleration
   반복 전송을 중단하고 검증된 safe fallback을 적용하며 request bit를 fail-closed로 clear한다.
<!-- HH_260810 - Restricted the global CAN topic and frame identifiers to one reviewed owner. -->
5. SocketCAN sender 앞에서 `/to_can_bus` writer identity와 허용 CAN ID, 특히 `0x630`,을
   deny-by-default로 검사한다.
<!-- HH_260810 - Required actual vehicle parameters instead of simulator defaults. -->
6. 실제 IONIQ EV geometry와 planner/controller `0.833 m/s` 제한을 versioned config로 고정한다.
<!-- HH_260810 - Kept the required-source failure chain proposed until its vehicle-side owners implement it. -->
7. proposed PC2 accepted-source status→proposed PC3 availability/MRM policy→PC1 existing gate 경로의
   type/node/owner/source를 post-v2에서 구현하고 stationary/no-sink bench에서 먼저 검증한다.

<!-- HH_260810 - Linked PC1 object consumption and the independent AEB limitation to immutable source. -->
PC1 Planning의 canonical object 소비는 [behavior planning](https://github.com/hwanhonglee/Autonomous-Driving/blob/c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8/src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml#L172-L218)과
[motion planning](https://github.com/hwanhonglee/Autonomous-Driving/blob/c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8/src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml#L119-L176)에 유지한다.
현재 AEB는 [physical pointcloud를 사용하고 predicted objects를 사용하지 않는다](https://github.com/hwanhonglee/Autonomous-Driving/blob/c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8/src/launcher/autoware_launch/autoware_launch/config/control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml#L1-L9).
따라서 PC4 object Planning 반응을 AEB 검증으로 표현하지 않는다.

<!-- HH_260810 - Listed the exact PC1 topics that must be retained in every actual test run. -->
## PC1에서 취득할 ROS 데이터

<!-- HH_260810 - Added one auditable purpose to each PC1 data row. -->
| Change comment | Topic | Type | 목적 |
|---|---|---|---|
| HH_260810 - Captured the only object boundary PC1 is allowed to consume. | `/perception/object_recognition/objects` | `autoware_perception_msgs/msg/PredictedObjects` | PC2 canonical freshness·writer GID·object content |
| HH_260810 - Preserved route provenance for every Planning comparison. | `/planning/mission_planning/route` | `autoware_planning_msgs/msg/LaneletRoute` | route UUID·segments·start/goal |
| HH_260810 - Measured the final planned response to accepted objects. | `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | point count·velocity profile·semantic digest |
| HH_260810 - Captured the controller output before the vehicle command gate. | `/control/trajectory_follower/control_cmd` | `autoware_control_msgs/msg/Control` | planner-to-controller comparison |
| HH_260810 - Captured the final command without permitting a CAN sink. | `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | final command·age·rate·hash |
| HH_260810 - Measured physical speed independently of PC3 pose. | `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | decoded status·unit/rate evidence |
| HH_260810 - Measured steering response without assigning localization authority. | `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | steering comparison |
| HH_260810 - Proved the parked gear condition for no-actuation stages. | `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | PARK state and mapping evidence |
| HH_260810 - Treated the current control-mode report as an unvalidated local-toggle diagnostic. | `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | comparison transition only; physical MANUAL authority requires independent ECU/operator evidence |
| HH_260810 - Recorded availability and MRM state without assigning current PC3 policy implementation. | `/system/fail_safe/mrm_state` | `autoware_adapi_v1_msgs/msg/MrmState` | live-audited owner·NORMAL/NONE baseline; future accepted-PC4 loss policy remains proposed |
| HH_260810 - Paired decoded vehicle reports with their raw receive evidence. | `/from_can_bus` or timestamped read-only `candump can0` | `can_msgs/msg/Frame` or raw SocketCAN frames | raw ID/DLC/payload/stamp joined to decoded status for unit sign and enum validation |

<!-- HH_260810 - Required graph metadata and payload timing in addition to a bag file. -->
각 topic에는 `ros2 topic info --verbose`의 publisher/subscriber node·GID·QoS, message source stamp,
PC1 receive wall/monotonic time, rate, gap, age, CDR hash를 같은 `run_id`로 저장한다.

<!-- HH_260810 - Listed PC1 host and CAN evidence that cannot be inferred from ROS topics alone. -->
## PC1에서 취득할 비-ROS 자료

<!-- HH_260810 - Required exact source and installed provenance. -->
- Git commit/dirty status, installed prefix, executable/library/config SHA-256.
<!-- HH_260810 - Required physical vehicle state evidence for every no-actuation stage. -->
- MANUAL/PARK·정지 상태의 timestamped operator checklist와 vehicle reports.
<!-- HH_260810 - Required kernel-level CAN counter evidence around the complete test window. -->
- `can0` link state 및 RX/TX packets·bytes·errors 전/중/후 snapshot; Stage 1.5–4 TX delta `0`.
<!-- HH_260810 - Required absence evidence for every actuator-capable process. -->
- SocketCAN sender·control adapter·`cansend/cangen/canplayer` process와 CAN device FD inventory.
<!-- HH_260810 - Required raw receive evidence before accepting custom decoder mappings. -->
- read-only `/from_can_bus` bag 또는 timestamped `candump can0`과 decoded vehicle reports의
  time-aligned CAN ID/DLC/payload→unit/sign/gear/control-mode mapping table.
<!-- HH_260810 - Required service and action call auditing rather than relying on topic absence. -->
- `/vehicle/engage`, `/autoware/engage`, operation-mode service/action 호출 audit log.
<!-- HH_260810 - Required host time and network state to align distributed evidence. -->
- `chronyc tracking/sources/sourcestats`, NIC/route/MTU/firewall, process tree, CPU/RSS metrics.

<!-- HH_260810 - Defined PC1 pass gates for the no-actuation vehicle stages. -->
## PASS와 중단 기준

<!-- HH_260810 - Required stable planning input and output before attributing any PC4 effect. -->
- Stage 1.5는 PC4 없이 canonical publisher 정확히 1개, route SET, stable non-zero baseline
  trajectory, critical duplicate publisher 0을 10분 이상 만족해야 한다.
<!-- HH_260810 - Kept every no-actuation stage electrically quiet. -->
- Stage 2–4는 sender/adapter process 0, `/to_can_bus` publisher와 actuator subscriber 0,
  engage/autonomous 호출 0, `can0` TX delta 0이어야 한다.
<!-- HH_260810 - Aborted attribution when the baseline trajectory is already a stop path. -->
- baseline trajectory가 start-planner 또는 다른 원인으로 이미 전 구간 zero velocity이면 PC4
  obstacle 인과시험을 중단하고 baseline을 먼저 복구한다.
<!-- HH_260810 - Prevented a required-source fault from silently becoming a clear scene. -->
- required mode의 PC4 loss가 availability false/engagement inhibit 또는 승인된 MRM으로 이어지지
  않으면 Stage 4 이상을 중단한다.

<!-- HH_260810 - Defined the PC1 rollback target and ordering. -->
## PC1 rollback

<!-- HH_260810 - Kept the vehicle stationary before any software rollback. -->
1. MANUAL/PARK·정지를 먼저 확인한다.
<!-- HH_260810 - Separated raw Stage 2 transport rollback from selector-aware Stage 3 and Stage 4 rollback. -->
2. Stage 2이면 PC2 selector/cache가 없으므로 gateway를 즉시 종료한다. Stage 3–4이면 PC2 mode를
   `real_only`로 전환하고 accepted cache/session을 clear한 뒤 physical-only canonical writer가
   정확히 하나인지 확인하고 gateway를 종료한다.
<!-- HH_260810 - Closed transport immediately when Stage 3 or Stage 4 de-selection could not be proved. -->
3. Stage 3–4의 source de-selection이 bounded timeout 내 실패해도 gateway를 즉시 종료하고 required-mode
   fault로 기록한다. 이후 Domain 10의 PC4-owned endpoints가 0인지 확인한다.
<!-- HH_260810 - Removed any receive-only test tooling without starting the historical full bridge. -->
4. 별도 receive-only wrapper를 종료하고 잔존 process/lock/endpoint가 0인지 확인한다.
<!-- HH_260810 - Restored the exact immutable PC1 release rather than partial file edits. -->
5. 필요하면 PC1 Autoware/ros2_ws를 위 v2 commit의 approved profile로 atomic 복원한다.
<!-- HH_260810 - Reproved zero actuation after rollback. -->
6. `/to_can_bus` endpoint 0, sender 0, CAN TX delta 0과 canonical Planning baseline을 다시 기록한다.
