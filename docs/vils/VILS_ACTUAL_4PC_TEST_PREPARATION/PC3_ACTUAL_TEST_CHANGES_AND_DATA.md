<!-- HH_260810 - Defined the PC3 changes and evidence required for actual VILS testing. -->
# PC3 actual-test changes and data

<!-- HH_260810 - Preserved PC3 source authority while requiring deployment evidence. -->
Status: **NO VIRTUAL-OBJECT PATH CHANGE; MAP LOCALIZATION TF AND TIME ACCEPTANCE REQUIRED**

<!-- HH_260810 - Recorded exact PC3 rollback identities. -->
## 고정 기준

<!-- HH_260810 - Bound PC3 source work to the reviewed Autoware release. -->
- Autoware v2.0.0: `947dda782ce90e1d9768e57ae4337e3cf78eee1b`
<!-- HH_260810 - Preserved the exact PC3 ROS workspace release for rollback. -->
- ros2_ws v2.0.0: `22f521ffb7ddeae893f377e226091641bb540efc`

<!-- HH_260810 - Restated PC3 authority without assigning virtual-object ownership to it. -->
PC3는 physical LiDAR·GNSS/INS·map bundle·localization·실차 `map→base_link` authority이다.
PC3 Chrony는 proposed common-time source이며 four-host acceptance 전에는 accepted time authority가
아니다. PC4 object subscriber, virtual-object fuser, PC4 TF, simulator localization 또는 canonical
perception publisher를 PC3에 추가하지 않는다.

<!-- HH_260810 - Linked PC3 role and handoff requirements to immutable documentation. -->
근거는 [PC3 role boundary](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC3_WORK_HISTORY.md#L78-L101)와
[remaining acceptance and handoff](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC3_WORK_HISTORY.md#L743-L804)이다.

<!-- HH_260810 - Classified PC3 changes by deployment risk. -->
## 변경 항목

<!-- HH_260810 - Added one auditable decision to each PC3 change row. -->
| Change comment | 범위 | 변경 여부 | 조치 |
|---|---|---|---|
| HH_260810 - Kept real map localization and TF ownership on PC3. | Object integration | `NO SOURCE CHANGE` | PC4 object path를 PC3에 넣지 않음 |
| HH_260810 - Required runtime proof of the actually deployed map bytes. | Map manifest | `CONFIG/DEPLOYMENT ONLY` | active Lanelet2/PCD/projector/config SHA-256를 한 manifest로 고정 |
| HH_260810 - Prevented legacy and native coordinate conventions from being mixed. | Map/GNSS convention | 명시적 선택 필요 | legacy map+offset 또는 native bundle+offset disabled 중 하나를 atomic 배포 |
| HH_260810 - Preserved one real map-to-base-link authority. | Localization/TF | source 변경 없음이 원칙 | EKF·NDT·GNSS ownership/rate/jump/reinit evidence 확보 |
| HH_260810 - Required a common four-host time source before latency or moving claims. | Chrony | proposed source and host configuration acceptance | PC1–4 source identity·reach·offset·jitter·step-free window 확보 전 authority claim 금지 |
| HH_260810 - Kept the proposed MRM and availability chain unimplemented until post-v2 review. | Failure routing | `PROPOSED; POST-v2 SOURCE/CONFIG REQUIRED` | PC2 accepted status를 PC3 availability/MRM policy에 연결할 exact type/node/topic/owner를 구현하고 parked acceptance 수행 |

<!-- HH_260810 - Recorded immutable Git map candidates without assuming they are currently deployed. -->
## Map bundle 판정

<!-- HH_260810 - Added one exact Git-object digest to each PC3 map candidate row. -->
| Change comment | PC3 ros2_ws Git object | SHA-256 |
|---|---|---|
| HH_260810 - Pinned the reviewed Lanelet2 candidate bytes. | `Autoware_Map/C_track/lanelet2_map.osm` | `1dd9ec1cd816ad231cfb7fca672a1367d8c74dd573cdc591c2228caee9f1a2ee` |
| HH_260810 - Pinned the reviewed pointcloud candidate bytes. | `Autoware_Map/C_track/pointcloud_map.pcd` | `34b93d4249cf72dd2747b947e63a4a806c0559fcb7d49ecc5d8ec63e043d3aa8` |
| HH_260810 - Pinned the reviewed projector candidate bytes. | `Autoware_Map/C_track/map_projector_info.yaml` | `94105d23dac3a87acf54ab6b9a2f2761164346ccea5e3b66950ddc66454ef097` |
| HH_260810 - Pinned the reviewed map configuration candidate bytes. | `Autoware_Map/C_track/map_config.yaml` | `337ecd648fcc339a66f638ddc985a2b30dd81be81e45de739ffb860e9a49e92e` |

<!-- HH_260810 - Prevented Git contents from being mistaken for the currently active deployment. -->
위 hash는 commit `22f521...`의 immutable candidate일 뿐 실제 시험 시 PC3가 load한 file이라는 증거가
아니다. 각 run에서 resolved absolute map path와 파일 hash, map loader process arguments를 다시
기록해야 한다.

<!-- HH_260810 - Disclosed the current cross-host map mismatch before allowing injection. -->
현재 PC4 local map hash는 Lanelet2 `a366b9e7...`, PCD `3f227f8d...`, projector
`06c1b450...`로 위 PC3 candidate와 다르다. 따라서 filename이나 route similarity만으로
map-correct를 선언할 수 없다.

<!-- HH_260810 - Recorded the competing projection conventions that require one atomic decision. -->
PC3 Git projector candidate는 C-track native LocalCartesianUTM origin을 갖지만, PC3 v2 GNSS poser에는
legacy correction `x-=60966.4679793288`, `y-=65973.64540576655`, `z-=15.816125382`가 enabled로
남아 있다. [projector source](https://github.com/hwanhonglee/Autonomous-Driving/blob/22f521ffb7ddeae893f377e226091641bb540efc/Autoware_Map/C_track/map_projector_info.yaml#L1-L10)와
[GNSS correction](https://github.com/hwanhonglee/Autonomous-Driving/blob/947dda782ce90e1d9768e57ae4337e3cf78eee1b/src/universe/autoware.universe/sensing/autoware_gnss_poser/config/gnss_poser.param.yaml#L15-L20)을
동시에 검토한다.

<!-- HH_260810 - Defined the two safe map deployment choices without selecting one from incomplete evidence. -->
### 허용 가능한 두 선택

<!-- HH_260810 - Preserved the current deployed convention as the minimal-change option. -->
1. 실제 active legacy bundle과 GNSS offset을 유지하고 PC4 simulator-world transform을 그
   좌표계에 맞춰 승인한다.
<!-- HH_260810 - Required an atomic post-v2 migration for the native map option. -->
2. native bundle을 배포하고 legacy offset을 disable하는 별도 PC3 post-v2 release를 만든 뒤
   GNSS/NDT/EKF/TF/control-point acceptance를 처음부터 다시 수행한다.

<!-- HH_260810 - Forbade mixed deployment and duplicated coordinate corrections. -->
두 선택을 섞거나 PC4에 PC3 GNSS subtraction을 다시 적용하면 안 된다. 실제 active convention을
runtime evidence로 확인하기 전에는 어느 선택도 승인 상태가 아니다.

<!-- HH_260810 - Listed the exact PC3 ROS data needed for every actual test. -->
## PC3에서 취득할 ROS 데이터

<!-- HH_260810 - Added one auditable purpose to each PC3 data row. -->
| Change comment | Topic or TF edge | Type | 목적 |
|---|---|---|---|
| HH_260810 - Captured the authoritative real ego pose and twist. | `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | PC4 `ego_actual`, PC1 Planning, rate/age/jump |
| HH_260810 - Captured authoritative measured acceleration. | `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | ego comparison and dynamics context |
| HH_260810 - Proved edge-level localization authority instead of aggregate TF counts. | `map→base_link` | `tf2_msgs/msg/TFMessage` transport | node/GID·rate·continuity·duplicate edge 0 |
| HH_260810 - Measured the physical LiDAR authority delivered toward PC2. | `/sensing/lidar/concatenated/pointcloud` | `sensor_msgs/msg/PointCloud2` | frame·stamp·point count·rate·gap |
| HH_260810 - Preserved map-loader authority and transient-local availability. | `/map/vector_map` | runtime-resolved LaneletMapBin type | one owner·map digest linkage |
| HH_260810 - Preserved pointcloud-map authority and service state. | `/map/pointcloud_map` and map-loader services | runtime-resolved map types | one owner·loaded PCD evidence |
| HH_260810 - Recorded NDT health rather than inferring localization from odometry alone. | NDT pose/score/diagnostics | resolved runtime types | convergence·jump·reinit·score |
| HH_260810 - Recorded the live-audited MRM baseline without claiming an implemented PC4-loss policy. | `/system/fail_safe/mrm_state` | `autoware_adapi_v1_msgs/msg/MrmState` | actual node/GID owner·NORMAL/NONE baseline; accepted-PC4 loss transition remains post-v2 proposed work |

<!-- HH_260810 - Required exact endpoint and payload timing evidence around PC3 authority topics. -->
각 stream/TF edge에는 publisher node·GID·QoS, source stamp, receive time, rate, maximum gap,
frame IDs와 full bag을 같은 `run_id`로 남긴다. `/tf` publisher 총수만으로 authority를 판단하지 않고
`map→base_link` edge별 writer를 검사한다.

<!-- HH_260810 - Listed PC3 host artifacts needed to validate map and time contracts. -->
## PC3에서 취득할 비-ROS 자료

<!-- HH_260810 - Required exact active-map provenance and projection state. -->
- resolved map paths, four-file SHA-256, projector/origin/grid, GNSS legacy-offset enable/value.
<!-- HH_260810 - Required surveyed geometry instead of route-only agreement. -->
- 3개 이상 non-collinear surveyed control point의 CARLA raw 좌표·PC3 map 좌표·변환 결과·residual.
<!-- HH_260810 - Required a live overlay time series around the real ego. -->
- real `base_link`와 PC4 `ego_actual` overlay의 x/y/z/yaw error RMS·max 및 사전 정의한 충분한
  sample plan에서만 p95를 보고한다.
<!-- HH_260810 - Required fresh localization acceptance after every power or map change. -->
- GNSS fix/INS status, NDT convergence/score, EKF continuity, reinitialization count, post-power 10분 안정성.
<!-- HH_260810 - Required all-host time evidence during one shared window. -->
- `chronyc tracking`, `sources -v`, `sourcestats -v` 전/중/후 raw output과 source/stratum/reach/offset/jitter.
<!-- HH_260810 - Required network and host provenance for distributed timing claims. -->
- NIC/IP/route/MTU/firewall/multicast, process tree, source/install/config hashes, CPU/RSS.

<!-- HH_260810 - Defined PC3 pass gates before PC4 data can be map-correct or time-correct. -->
## PASS와 중단 기준

<!-- HH_260810 - Required one approved and byte-identified map convention. -->
- 실제 active bundle·projector·GNSS offset 선택이 manifest와 일치하고 mixed convention이 없어야 한다.
<!-- HH_260810 - Required geometric alignment beyond matching filenames and routes. -->
- 3개 이상 non-collinear control point와 live ego overlay가 owner-approved tolerance를 만족해야 한다.
<!-- HH_260810 - Required unique and continuous localization and TF authority. -->
- PC3 odometry·acceleration·`map→base_link`가 10분 이상 continuous하며 duplicate edge, unapproved
  NDT jump/reinit이 없어야 한다.
<!-- HH_260810 - Required a common clock before distributed latency or motion claims. -->
- PC1–4가 동일한 approved Chrony source를 선택하고 offset/jitter/step-free 기준을 만족해야 한다.
<!-- HH_260810 - Prevented Stage 4 required-mode testing before the proposed vehicle-side fault policy exists. -->
- PC2 accepted status를 구독하는 proposed PC3 post-v2 availability/MRM component의 exact
  interface/node/owner, unit tests, one-writer proof와 parked/no-sink fault acceptance가 없으면
  `hybrid_required` 이상을 중단한다.

<!-- HH_260810 - Defined atomic PC3 rollback without simulator authority leakage. -->
## PC3 rollback

<!-- HH_260810 - Kept the vehicle stationary during map and localization rollback. -->
1. 차량 MANUAL/PARK·정지를 확인한다.
<!-- HH_260810 - Applied stage-aware virtual-source removal before changing PC3 authority. -->
2. Stage 1.5에는 gateway가 없어야 한다. Stage 2이면 PC2 selector/cache를 기다리지 않고 gateway를
   즉시 종료한다. Stage 3–4이면 PC2를 `real_only`로 전환하고 cache/session clear와 physical-only
   canonical ownership을 bounded timeout으로 확인한 뒤 gateway를 종료하며, 확인 실패 시에도
   gateway를 즉시 닫고 fault로 기록한다.
<!-- HH_260810 - Stopped localization dependents only after the vehicle and virtual path were safe. -->
3. localization dependent nodes를 owner-approved 순서로 안전하게 종료한다.
<!-- HH_260810 - Restored the complete approved bundle and offset policy as one unit. -->
4. Lanelet2·PCD·projector·map config·GNSS offset을 pre-test manifest의 한 세트로 atomic 복원한다.
<!-- HH_260810 - Required fresh localization acceptance after rollback. -->
5. GNSS/NDT/EKF/`map→base_link`와 3-point/ego overlay를 다시 검증한다.
<!-- HH_260810 - Preserved the immutable source baseline and failed evidence. -->
6. source가 바뀌었다면 PC3 v2 approved profile로 복원하고 failed run evidence는 보존한다.
