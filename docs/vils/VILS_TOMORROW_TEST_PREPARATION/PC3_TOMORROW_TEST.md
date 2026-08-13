<!-- HH_260810 - Defined the PC3 execution sheet for the 2026-08-14 stationary four-PC preparation test. -->
# PC3 tomorrow actual-test sheet — 2026-08-14

<!-- HH_260810 - Declared the evidence boundary before any operator treats configuration as a passed vehicle test. -->
Status: **CURRENT PC3→PC2 LIDAR CONTRACT IS CODE-ALIGNED; ACTUAL CROSS-PC DATA PASS IS NOT YET PROVEN**

<!-- HH_260810 - Stated the single operational objective for the first vehicle-side session. -->
이 문서의 첫 목적은 PC3가 소유한 실제 Pandar64 데이터를 PC3 Nebula 전처리와 현재 legacy
relay를 통해 PC2 perception까지 전달하고, source·relay·network receive·downstream perception을
하나의 `run_id`로 증명하는 것이다. 첫 baseline에서는 source code나 topic contract를 바꾸지 않는다.

<!-- HH_260810 - Prevented this stationary evidence run from opening any physical command authority. -->
이 시험은 MANUAL/PARK·차량 고정·no-actuation 조건에서만 수행한다. CAN transmit, engage,
autonomous-mode request, control-command injection, vehicle interface 변경은 이 문서의 범위가 아니다.

<!-- HH_260810 - Pinned the immutable source identities used for every static conclusion in this sheet. -->
## Audited immutable baselines

<!-- HH_260810 - Recorded each immutable branch and commit without treating the branch name as sufficient provenance. -->
| Change comment | Workspace | Branch | Audited commit |
|---|---|---|---|
| HH_260810 - Pinned the PC3 sensing localization and map source snapshot. | PC3 Autoware | `IONIQ_EV/PC3/autoware_universe/v2.0.0` | `947dda782ce90e1d9768e57ae4337e3cf78eee1b` |
| HH_260810 - Pinned the PC3 driver and map companion snapshot. | PC3 ros2_ws | `IONIQ_EV/PC3/ros2_ws/v2.0.0` | `22f521ffb7ddeae893f377e226091641bb540efc` |
| HH_260810 - Pinned the PC2 sensing and perception source snapshot. | PC2 Autoware | `IONIQ_EV/PC2/autoware_universe/v2.0.0` | `44f79b408ccbffb4cad6f56cbee68de841ac38ac` |
| HH_260810 - Pinned the PC2 camera and driver companion snapshot. | PC2 ros2_ws | `IONIQ_EV/PC2/ros2_ws/v2.0.0` | `65515a2fc13266c95a812b11c1df7f7b7f5f184c` |

<!-- HH_260810 - Required runtime provenance because a Git branch does not prove which installed bytes are executing. -->
시험 manifest에는 각 host의 resolved source path, installed package prefix, executable/library/config hash,
dirty state, ROS Domain, RMW, DDS XML, NIC/IP/route와 process arguments를 기록한다. 위 commit과 다른
installed byte가 발견되면 그 상태를 별도 baseline으로 승인하기 전까지 시험을 중단한다.

<!-- HH_260810 - Linked the exact PC3 single-LiDAR output and compatibility relay implementation. -->
- [PC3 Pandar64 output and legacy relay](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml#L1-L58)
<!-- HH_260810 - Linked the common wrapper that forwards the final pointcloud topic into Nebula. -->
- [PC3 Pandar64 common launch](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/hesai_Pandar_64.launch.xml#L33-L54)
<!-- HH_260810 - Linked the PC3 raw decoder remap and preprocessing components. -->
- [PC3 Nebula raw output remap](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py#L112-L159) and [PC3 preprocessing and final publisher](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/launch/nebula_node_container.launch.py#L173-L243)
<!-- HH_260810 - Linked the PC2 hardware-free sensing role and its active pointcloud input contract. -->
- [PC2 disabled LiDAR include](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml#L7-L35) and [PC2 current perception input](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/perception.launch.xml#L47-L49)
<!-- HH_260810 - Linked the authoritative statements that withhold current physical LiDAR and cross-PC acceptance. -->
- [PC3 unresolved live LiDAR gate](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/migration_work/reports/PC3_v2.0.0_detailed_changes.md#L980-L1007) and [PC2 final local cycle with PC3 inputs absent](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/migration_work/reports/PC2_dual_camera_YOLO_TLR_integration.md#L463-L470)

<!-- HH_260810 - Defined the exact active pointcloud path without attributing any LiDAR hardware to PC2. -->
## Exact PC3 Pandar64 to PC2 perception path

<!-- HH_260810 - Made the physical owner producer relay and consumer boundaries explicit. -->
```text
PC3 physical Pandar64
  UDP: sensor 192.168.2.101 -> PC3 sensor NIC 192.168.2.150:2368
  -> PC3 Nebula HesaiRosWrapper
     pandar_points -> pointcloud_raw_ex
  -> PC3 CropBoxFilterComponent (self crop)
  -> PC3 CropBoxFilterComponent (mirror crop)
  -> PC3 DistortionCorrectorComponent
  -> PC3 RingOutlierFilterComponent
  -> /sensing/lidar/concatenated/pointcloud
  -> PC3 /sensing/lidar/pc3_lidar_legacy_relay
  -> /sensing/lidar/top/pointcloud_before_sync
  -> Vehicle ROS Domain 10
  -> PC2 perception PointCloud2 subscribers
  -> PC2 ground segmentation and object detection
  -> PC2 tracking
  -> PC2 map-based prediction
  -> /perception/object_recognition/objects
```

<!-- HH_260810 - Identified the actual producer of the misleadingly named single-LiDAR concatenated topic. -->
`/sensing/lidar/concatenated/pointcloud`는 현재 별도 concatenate node의 output이 아니다. PC3의
단일 Pandar64 cloud가 self/mirror crop과 distortion correction을 거친 뒤, PC3
`RingOutlierFilterComponent`가 `output_pointcloud_topic` remap을 통해 직접 publish한다.

<!-- HH_260810 - Recorded that the unused concatenation include must not be mistaken for the active publisher. -->
PC3 `lidar.launch.xml`의 별도 `pointcloud_preprocessor.launch.py` include는 주석 처리되어 있다.
따라서 topic 이름의 `concatenated`는 현재 단일 LiDAR용 canonical contract 이름이며 multi-LiDAR
concatenation 실행 증거가 아니다.

<!-- HH_260810 - Preserved the current release-compatible relay as the first actual baseline. -->
PC3의 `launch_legacy_pointcloud_relay` 기본값은 `true`이고, PC2 active perception input 기본값은
`/sensing/lidar/top/pointcloud_before_sync`이다. 현재 두 v2 기본값은 이미 일치하므로 첫 baseline
전에 PC2 direct remap이나 PC3 relay 제거를 수행하지 않는다.

<!-- HH_260810 - Distinguished PC2 perception processing from physical sensing ownership. -->
PC2에는 LiDAR 하드웨어와 LiDAR driver가 없다. PC2는 PC3가 Domain 10에 publish한
`sensor_msgs/msg/PointCloud2`를 구독하고 detection·tracking·prediction을 수행한다. PC2의
pointcloud container는 perception component container이며 물리 센서 ownership 증거가 아니다.

<!-- HH_260810 - Declared the current evidence limit before defining tomorrow's acceptance gates. -->
## Current evidence limit

<!-- HH_260810 - Prevented offline and hardware-disabled tests from becoming an actual cross-PC claim. -->
PC3 authoritative report의 final regression은 물리 LiDAR에 접속하지 않았고 Hesai teardown은 loopback
UDP로 검증되었다. PC2 final local cycle도 PC3 input publisher가 0인 상태였다. 따라서 현재 근거는
실차용 code contract와 offline regression이며 실제 PC3→PC2 LiDAR data PASS가 아니다.

<!-- HH_260810 - Prevented generic upstream test bags from being treated as this vehicle's evidence. -->
Nebula source tree에 포함된 generic Hesai test bag은 현재 IONIQ EV, 현재 sensor serial/calibration,
현재 PC3 NIC와 PC2 receive path의 증거로 사용하지 않는다.

<!-- HH_260810 - Prevented the same-host CARLA result from replacing the missing physical distributed test. -->
PC4에서 완료한 same-PC CARLA Domain 10→5 결과도 이 물리 LiDAR·vehicle LAN·cross-PC DDS gate를
대체하지 않는다.

<!-- HH_260810 - Defined all non-pointcloud authorities that must remain stable during the LiDAR test. -->
## PC3 authority and ownership caveats

<!-- HH_260810 - Separated implemented ownership from pending runtime acceptance for each PC3 responsibility. -->
| Change comment | Area | Tomorrow rule | Blocking condition |
|---|---|---|---|
| HH_260810 - Kept physical sensing ownership exclusively on PC3. | Pandar64 and preprocessing | PC3 driver·preprocess·canonical cloud·legacy relay만 publish | PC2/PC4 LiDAR driver, manual duplicate driver, duplicate pointcloud writer |
| HH_260810 - Required active map bytes rather than trusting filenames or Git candidates. | Map | 실제 loaded Lanelet2·PCD·projector·map config의 absolute path와 SHA-256 기록 | mixed map convention, unresolved active path, hash mismatch |
| HH_260810 - Preserved PC3 as localization and dynamic TF authority. | Localization and TF | odometry·acceleration·NDT/EKF health와 `map→base_link` edge owner 기록 | stale localization, reset, jump, zero/duplicate/unapproved edge writer |
| HH_260810 - Kept the proposed common clock separate from accepted time authority. | Chrony | PC1–PC4가 같은 approved source·bounded offset/jitter·no-step를 보인 뒤에만 distributed latency 계산 | different source, lost reach, excessive offset/jitter, clock step |
| HH_260810 - Required live MRM ownership discovery instead of assigning it to PC3 by assumption. | MRM | `/system/fail_safe/mrm_state`의 actual node/GID/type와 exactly one fresh writer를 live audit | missing/duplicate/stale writer or unapproved state transition |
| HH_260810 - Kept accepted-PC4 loss routing as post-v2 work. | VILS availability | PC2 accepted status→vehicle availability/MRM 연결은 구현·review·parked test 후에만 사용 | current PC3 v2를 implemented PC4-loss consumer로 오인 |
| HH_260810 - Preserved the no-actuation boundary for every preparation stage. | CAN and command | receive-only evidence만 수집하고 CAN TX delta와 experimental command writer를 0으로 유지 | any test-owned command writer, CAN TX increment, engage or AUTO call |

<!-- HH_260810 - Recorded candidate map bytes without claiming they are the active vehicle deployment. -->
PC3 ros2_ws v2에서 확인한 C-track candidate SHA-256은 Lanelet2
`1dd9ec1cd816ad231cfb7fca672a1367d8c74dd573cdc591c2228caee9f1a2ee`, PCD
`34b93d4249cf72dd2747b947e63a4a806c0559fcb7d49ecc5d8ec63e043d3aa8`, projector
`94105d23dac3a87acf54ab6b9a2f2761164346ccea5e3b66950ddc66454ef097`, map config
`337ecd648fcc339a66f638ddc985a2b30dd81be81e45de739ffb860e9a49e92e`이다.

<!-- HH_260810 - Required runtime map verification because candidate hashes alone do not identify loaded files. -->
위 값은 candidate 식별자일 뿐 내일 PC3가 실제 load한 파일의 증거가 아니다. runtime resolved path,
loader arguments와 시험 직전 계산한 hash가 승인 manifest와 일치해야 한다.

<!-- HH_260810 - Defined the preflight order that keeps PC4 and every actuator path absent from the physical baseline. -->
## Preflight sequence

<!-- HH_260810 - Added a fail-closed prerequisite to every preflight action. -->
| Change comment | Step | Required action | PASS evidence |
|---|---:|---|---|
| HH_260810 - Started from an immutable run identity. | 1 | UTC/KST time window, operator, vehicle, scenario, source/install/config hashes와 `run_id` 생성 | four-host manifest가 같은 `run_id` 사용 |
| HH_260810 - Kept the vehicle physically non-actuating. | 2 | MANUAL/PARK, wheel restraint and safety supervisor 확인; PC4·gateway·experimental command paths OFF | signed safety checklist; gateway and PC4 process count 0 |
| HH_260810 - Proved the physical sensor network before starting ROS data claims. | 3 | PC3 sensor NIC/IP/route/link/MTU와 Pandar64 IP·port·model·calibration·return mode·timestamp source 확인 | exact NIC and sensor manifest; no duplicate manual driver |
| HH_260810 - Proved the vehicle DDS network independently from the sensor NIC. | 4 | PC1–PC3 Domain 10 NIC/IP/route/firewall/multicast/RMW/DDS XML 확인 | intended interface only; no localhost-only setting |
| HH_260810 - Required a common clock window before latency measurement. | 5 | 모든 participating host의 Chrony tracking/sources/sourcestats를 같은 창에서 수집 | approved common source and bounded clock state |
| HH_260810 - Verified all PC3 foundational authorities before PC2 consumes data. | 6 | active map, localization, NDT/EKF, TF edge owners와 live MRM owner 확인 | one approved authority per required resource |
| HH_260810 - Preserved the v2 compatibility path for the first run. | 7 | PC3 legacy relay ON과 PC2 `before_sync` subscription default 유지 | expected launch arguments and no direct-remap profile |
| HH_260810 - Opened recorders before the physical source. | 8 | PC3 source-side, PC2 receive/downstream, independent graph/network recorders 시작 | recorder timestamps and output paths in manifest |

<!-- HH_260810 - Defined exact PC3 and PC2 evidence streams for the current relay baseline. -->
## Required topics and endpoint evidence

<!-- HH_260810 - Assigned one source owner consumer and evidence purpose to every pointcloud boundary. -->
| Change comment | Topic or endpoint | Expected producer | Expected consumer | Required evidence |
|---|---|---|---|---|
| HH_260810 - Captured the first decoded PC3 PointCloud2 boundary. | `/sensing/lidar/top/pointcloud_raw_ex` | PC3 Nebula `hesai_ros_wrapper_node` | PC3 self-crop stage | writer/subscriber GID·QoS·frame·stamp·rate·point count |
| HH_260810 - Captured the active intermediate topics without making their names external contracts. | runtime-resolved self-cropped, mirror-cropped and rectified PointCloud2 topics | PC3 preprocessing components | next PC3 preprocessing component | resolved names·component/node GID·count·rate·maximum gap |
| HH_260810 - Captured the sole PC3 canonical single-LiDAR output. | `/sensing/lidar/concatenated/pointcloud` | PC3 `ring_outlier_filter` | PC3 localization and PC3 relay | exactly one writer·QoS·base_link/frame evidence·payload integrity |
| HH_260810 - Captured the current PC3-owned compatibility output consumed by PC2. | `/sensing/lidar/top/pointcloud_before_sync` | PC3 `pc3_lidar_legacy_relay` | PC2 perception subscribers | one PC3 writer·PC2 subscriber GIDs·QoS compatibility·two-sided counts |
| HH_260810 - Proved PC2 derives obstacle data from the PC3 cloud rather than owning a sensor. | `/perception/obstacle_segmentation/pointcloud` | PC2 perception | PC2 and PC1 consumers as launched | fresh output·upstream lineage·one approved writer |
| HH_260810 - Recorded the PC2 detector boundary. | runtime-resolved PC2 detected-object output | PC2 detection | PC2 tracking | type·writer/subscriber GID·frame·stamp·object count |
| HH_260810 - Recorded the canonical tracked boundary before prediction. | `/perception/object_recognition/tracking/objects` | PC2 tracker | PC2 map-based predictor | exactly one writer·UUID/count/rate/freshness |
| HH_260810 - Recorded the sole canonical object boundary consumed by Planning. | `/perception/object_recognition/objects` | one PC2 map-based predictor | PC1 Planning and object consumers | exactly one writer·PredictedObjects lineage·freshness |

<!-- HH_260810 - Required pointcloud structural checks that endpoint counts alone cannot provide. -->
각 PointCloud2 stream에서 `frame_id`, source stamp, receive wall/monotonic time, height, width,
`point_step`, `row_step`, fields, endianness, `is_dense`, data length, finite-point count와 semantic digest를
수집한다. non-empty one-row cloud는 `row_step == point_step * width`와 payload-size consistency를
만족해야 한다.

<!-- HH_260810 - Required exact source-to-relay identity and timing comparison. -->
PC3 canonical과 PC3 relay output은 동일 header stamp·frame·dimensions·fields·point count·semantic
digest를 가져야 한다. relay receive-to-publish delay, PC2 receive delay, message count와 최대
inter-message gap을 별도로 계산한다.

<!-- HH_260810 - Prevented ROS-message continuity from being mislabeled as packet delivery ratio. -->
source/receive sequence 또는 semantic digest reconciliation은 application-message MDR로 보고한다.
NIC/pcap packet 통계가 없으면 packet PDR이라는 용어를 사용하지 않는다.

<!-- HH_260810 - Listed the non-LiDAR streams required to attribute failures correctly. -->
| Change comment | Topic or artifact | Owner | Required evidence |
|---|---|---|---|
| HH_260810 - Bound perception to the active vector map. | `/map/vector_map` | PC3 | writer GID·transient-local QoS·loaded map hash |
| HH_260810 - Bound NDT and pointcloud filtering to the active PCD. | `/map/pointcloud_map` and loader services | PC3 | writer GID·loaded PCD hash·service state |
| HH_260810 - Captured authoritative ego pose and twist. | `/localization/kinematic_state` | PC3 | fresh continuous odometry·rate·age·jump/reinit count |
| HH_260810 - Captured authoritative measured acceleration. | `/localization/acceleration` | PC3 | fresh stream·frame·rate·age |
| HH_260810 - Proved localization authority at the edge level. | `map→base_link` over `/tf` | PC3 | exactly one approved edge writer·continuity·no jump |
| HH_260810 - Preserved static sensor geometry ownership. | required edges over `/tf_static` | PC3 and approved vehicle description | one approved writer per edge·transient-local QoS |
| HH_260810 - Captured the motion input required by distortion correction. | `/sensing/vehicle_velocity_converter/twist_with_covariance` | PC3 converter from approved vehicle status | fresh rate·age·source lineage |
| HH_260810 - Captured the IMU input required by distortion correction. | `/sensing/imu/imu_data` | PC3 sensing | fresh rate·age·frame·covariance |
| HH_260810 - Audited the actual MRM writer without assigning it by documentation. | `/system/fail_safe/mrm_state` | live-audited vehicle owner | exact `autoware_adapi_v1_msgs/msg/MrmState` type·node/GID·freshness·state timeline |
| HH_260810 - Preserved the no-actuation evidence window. | command endpoints, CAN FDs and counters | independent safety recorder | no experimental writer·sender 0·CAN TX delta 0·engage/AUTO calls 0 |

<!-- HH_260810 - Required a staged runtime sequence so every failure has an attributable boundary. -->
## Runtime test sequence

<!-- HH_260810 - Defined bounded phases with explicit stop decisions instead of one opaque full-stack launch. -->
| Change comment | Phase | Action | Continue only when |
|---|---:|---|---|
| HH_260810 - Established a PC4-free three-PC baseline. | A | PC1–PC3를 approved no-actuation profile로 시작하고 PC4/gateway는 계속 OFF | graph ownership, map, clock and safety preflight PASS |
| HH_260810 - Proved the physical driver and raw decoded cloud first. | B | PC3 Pandar64/Nebula source를 관찰 | one driver and fresh valid `pointcloud_raw_ex` |
| HH_260810 - Proved every PC3 preprocessing boundary. | C | crop·distortion·ring-filter outputs를 source stamp로 연결 | no unexplained loss, malformed payload or stale input |
| HH_260810 - Proved the canonical and legacy relay pair. | D | canonical and `before_sync` payload를 동시 기록 | semantic equality and bounded relay delay |
| HH_260810 - Proved actual cross-PC receipt rather than publisher discovery. | E | PC2에서 `before_sync` payload와 subscriber endpoint를 기록 | nonzero received messages, matching lineage and bounded age |
| HH_260810 - Proved downstream perception from the received physical cloud. | F | PC2 obstacle/detection/tracking/prediction outputs를 기록 | fresh pipeline and one canonical tracked/predicted writer |
| HH_260810 - Sustained the baseline long enough to expose resets and gaps. | G | 승인된 stationary/low-risk scene에서 최소 10분 연속 기록 | no driver restart, ownership drift, clock step or excessive gap |
| HH_260810 - Required clean shutdown as part of the acceptance result. | H | owned processes만 역순 종료하고 post-stop graph·actor·FD·port residue 확인 | all owned resources removed and unrelated PC1–PC3 authority preserved |

<!-- HH_260810 - Prevented an existing publisher-count script from being treated as sufficient payload evidence. -->
PC2의 기존 `REQUIRE_PC3_INPUTS=1` gate는 publisher availability 중심이며 canonical→relay payload
동일성을 단독으로 증명하지 않는다. 기존 `probe_pc2_pc3_flow.py`도 `before_sync`와 `raw_ex`는
구독하지만 canonical `concatenated`는 구독하지 않는다. 내일 시험에서는 별도 read-only recorder로
세 boundary를 동시에 수집한다.

<!-- HH_260810 - Defined the exact acceptance gates required before declaring the first actual cross-PC PASS. -->
## PASS criteria

<!-- HH_260810 - Required every independent gate because a partial pass cannot establish the end-to-end path. -->
| Change comment | Gate | PASS requirement |
|---|---|---|
| HH_260810 - Proved exclusive PC3 physical ownership. | Hardware ownership | Pandar64 driver 1, manual/duplicate driver 0, PC2/PC4 LiDAR driver 0 |
| HH_260810 - Proved the decoded physical source is live. | Raw PointCloud2 | nonzero fresh `raw_ex`, valid structure, approved frame/stamp/rate and no unexplained restart |
| HH_260810 - Proved the final canonical PC3 cloud. | Preprocessing | valid fresh canonical output, one ring-filter writer, bounded processing delay and gap |
| HH_260810 - Proved the compatibility relay is faithful. | Relay | one PC3 relay writer; canonical and `before_sync` semantic digests/counts reconcile; delay within approved bound |
| HH_260810 - Proved physical network delivery into PC2. | Cross-PC receive | PC2 receives nonzero matching `before_sync` messages with compatible QoS and bounded age; PC2 publishers on both LiDAR input topics 0 |
| HH_260810 - Proved the PC2 object chain is driven by the received cloud. | Perception | fresh obstacle/detection/tracking outputs and exactly one PC2 canonical PredictedObjects publisher |
| HH_260810 - Proved map and localization validity independently. | Map and localization | active hashes approved; NDT/EKF healthy; continuous odometry/acceleration; one `map→base_link` writer |
| HH_260810 - Proved the common-time assumption used by latency metrics. | Time | same approved Chrony source; accepted offset/jitter/reach/stratum; no step during run |
| HH_260810 - Proved the preparation test never acquired command authority. | Safety | no test-owned control writer, sender, CAN TX increment, engage/AUTO call or unapproved MRM transition |
| HH_260810 - Proved repeatable operation rather than a brief endpoint appearance. | Duration and cleanup | at least 10 continuous minutes, no excessive gap/ownership drift, clean scoped shutdown and no residue |

<!-- HH_260810 - Withheld the overall result until every gate has signed evidence. -->
모든 gate가 같은 `run_id`와 common-time window에서 PASS하고 PC2·PC3 owner가 원본 자료를 승인한
경우에만 **actual PC3→PC2 physical LiDAR baseline PASS**라고 기록한다. 하나라도 미충족이면 부분
결과 이름을 사용하고 cross-PC/full-live PASS를 선언하지 않는다.

<!-- HH_260810 - Defined immediate abort conditions before an operator encounters them. -->
## Immediate abort conditions

<!-- HH_260810 - Aborted on physical source or publisher ownership ambiguity. -->
- Pandar64/manual driver 중복, sensing pointcloud duplicate writer, unexpected writer GID change.
<!-- HH_260810 - Aborted on malformed or temporally invalid sensor data. -->
- invalid `row_step`, non-finite payload, wrong frame, old/future/non-monotonic stamp, source restart 또는 승인 범위를 넘는 gap.
<!-- HH_260810 - Aborted on cross-PC loss before downstream behavior becomes unattributable. -->
- canonical은 fresh하지만 relay 또는 PC2 receive가 missing/stale하거나 QoS incompatible인 경우.
<!-- HH_260810 - Aborted on coordinate localization or time authority failure. -->
- active map hash mismatch, NDT/EKF failure, duplicate `map→base_link`, Chrony source change/step.
<!-- HH_260810 - Aborted on unresolved or unsafe system-state ownership. -->
- missing/duplicate/stale MRM writer 또는 승인되지 않은 availability/MRM transition.
<!-- HH_260810 - Aborted immediately on any physical command capability or action. -->
- experimental control/CAN writer, sender/adapter process, CAN TX 증가, engage/AUTO call 발견.

<!-- HH_260810 - Deferred direct canonical migration until the unchanged current baseline is measured. -->
## Relay retention and optional direct-remap decision

<!-- HH_260810 - Selected the current PC3 relay path as tomorrow's minimal-change baseline. -->
내일 첫 시험은 **relay 유지**가 기본이다. 이는 PC2·PC3 v2의 현재 no-argument 동작이며 source
변경 없이 실제 데이터 경로를 검증하고 즉시 rollback할 수 있다.

<!-- HH_260810 - Documented the bounded costs of keeping the legacy relay. -->
relay 유지에는 추가 full-rate subscribe/publish hop, CPU·memory·DDS traffic, latency/jitter와 하나의
failure point가 따른다. canonical과 legacy 두 topic contract도 함께 관리해야 한다.

<!-- HH_260810 - Kept direct remapping as a separately reviewed post-v2 optimization. -->
PC2 direct remap은 relay baseline PASS 뒤 별도 post-v2 profile로만 평가한다. PC2 inner
`perception.launch.xml`은 이미 `input/pointcloud` argument를 제공하므로 PC2-specific outer profile
또는 scoped remap에서 canonical `/sensing/lidar/concatenated/pointcloud`를 선택하고, 기존
no-argument legacy 동작은 유지한다. generic `tracking.launch.xml`은 수정하지 않는다.

<!-- HH_260810 - Defined the evidence required before removing the compatibility relay. -->
direct profile은 current relay와 동일 scene에서 source/receive count, semantic digest, frame/stamp,
rate/gap, end-to-end age, PC3/PC2 CPU·RSS와 NIC bytes를 A/B 비교한다. direct path가 승인되기 전에
PC3 relay를 disable하지 않는다.

<!-- HH_260810 - Defined rollback ordering that restores PC2 input before stopping any source. -->
## Rollback

<!-- HH_260810 - Preserved evidence and safety state before any rollback mutation. -->
1. 새 experiment 또는 PC4가 연결돼 있다면 PC4 source와 gateway를 먼저 중지하되 independent
   recorders는 유지한다.
<!-- HH_260810 - Re-established the current PC3 compatibility writer before returning PC2 to the legacy input. -->
2. direct-remap 시험 중이었다면 PC3 legacy relay가 ON이고 `before_sync` writer가 fresh한지 먼저
   확인한다.
<!-- HH_260810 - Returned PC2 to the immutable legacy subscription only after its source exists. -->
3. PC2를 현재 v2 legacy subscription profile로 되돌리고 `before_sync` subscriber와 actual receive를
   확인한다.
<!-- HH_260810 - Required physical-only perception proof before removing experimental configuration. -->
4. PC2 obstacle/detection/tracking/prediction freshness와 one-writer canonical ownership을 재확인한다.
<!-- HH_260810 - Removed only reviewed post-v2 profiles and never overwrote the audited releases. -->
5. experimental direct-remap profile만 비활성화하고 PC2·PC3 v2 branch/tag 또는 `(copy_org)`를
   덮어쓰지 않는다.
<!-- HH_260810 - Stopped the physical source only after the consumer rollback is proven. -->
6. 시험 종료이면 PC2 perception process, PC3 relay, preprocessing, Pandar64 source를 승인된 역순으로
   scoped stop한다.
<!-- HH_260810 - Completed rollback with residue and immutable evidence checks. -->
7. graph writer/subscriber GID, process, socket, file descriptor, port, guard file, CAN counter와 clock
   상태를 재확인하고 recorders를 마지막에 종료한 뒤 evidence root SHA-256을 생성한다.

<!-- HH_260810 - Prevented a rollback from silently restoring unsafe command-capable defaults. -->
rollback은 broad source overwrite나 historical backup 전체 복원으로 수행하지 않는다. actual installed
snapshot과 pinned v2 provenance를 비교하여 승인된 profile만 되돌리며, vehicle interface·MRM·CAN
ownership은 별도 safety owner의 절차를 따른다.

<!-- HH_260810 - Defined the minimum artifact layout needed for a reviewable next-day handoff. -->
## Required handoff artifacts

<!-- HH_260810 - Assigned one immutable evidence object to each review question. -->
| Change comment | Artifact | Minimum content |
|---|---|---|
| HH_260810 - Preserved exact software and deployment identity. | `run_manifest` | four-host identity·commit/install hashes·arguments·domain/RMW/DDS·NIC·map·scenario·operator |
| HH_260810 - Preserved the full common-time state. | `clock_window` | Chrony tracking/sources/sourcestats before·during·after |
| HH_260810 - Preserved physical and DDS topology separately. | `network_inventory` | sensor NIC and vehicle DDS NIC link/IP/route/MTU/firewall/multicast/offload/socket state |
| HH_260810 - Preserved source-to-relay payload lineage. | `pc3_pointcloud_chain` | raw/intermediate/canonical/relay bags·endpoint snapshots·semantic digest/count/rate/gap CSV |
| HH_260810 - Preserved actual cross-PC receipt and downstream causality. | `pc2_receive_and_perception` | legacy input·obstacle/detected/tracked/predicted bags and GID/timing lineage |
| HH_260810 - Preserved coordinate and localization authority. | `map_localization_tf` | active hashes·loader arguments·NDT/EKF health·odom/accel·edge-level TF audit |
| HH_260810 - Preserved system safety ownership and no-actuation proof. | `mrm_and_no_actuation` | MRM node/type/GID/state timeline·command endpoints·process/FD/CAN counters·engage log |
| HH_260810 - Preserved cleanup as part of the test result. | `shutdown_audit` | stop order·exit status·post-stop graph/process/socket/FD/port/guard residue |

<!-- HH_260810 - Defined the final claim language for tomorrow's operator report. -->
## Allowed final claim language

<!-- HH_260810 - Provided the only wording allowed after all actual source-to-consumer gates pass. -->
PASS 시: `The actual PC3 Pandar64 PointCloud2 stream was processed by the PC3 Nebula chain, republished by the PC3-owned legacy relay, received by PC2 perception over Vehicle Domain 10, and propagated through the physical object pipeline during the recorded stationary baseline.`

<!-- HH_260810 - Provided bounded wording for any incomplete or failed run. -->
미완료 시: `The PC3 and PC2 v2 code contracts are statically aligned, but the actual cross-PC physical LiDAR path remains unverified because the recorded run did not satisfy all source, relay, receive, downstream, map, time, ownership, and safety gates.`

<!-- HH_260810 - Prevented tomorrow's physical baseline from being conflated with later PC4 VILS claims. -->
이 PC3→PC2 baseline PASS만으로 PC4 virtual-object ingress, PC2 VILS validator/fuser, PC1 Planning
response, cross-PC PC4 latency, packet PDR 또는 실제 차량 VILS 전체 PASS를 주장하지 않는다.
