<!-- HH_260810 - Defined the PC4 changes and evidence required for actual VILS testing. -->
# PC4 actual-test changes and data

<!-- HH_260810 - Distinguished implemented same-host tooling from unfinished distributed deployment. -->
Status: **LIVE SURROGATE PATH BOUNDED PASS; ACTUAL PRIVATE-DOMAIN VEHICLE INTEGRATION NOT YET ACCEPTED**

<!-- HH_260810 - Restated PC4 authority and its forbidden boundaries. -->
PC4는 CARLA scenario/actors, live object adapter, synchronized source metadata, approved
simulator-world→map transform, private-domain gateway, `ego_actual` visualization 및 TX evidence를
소유한다. PC4는 vehicle-domain canonical object, `/clock`, `/tf*`, localization, vehicle status,
planning, control, system/MRM, CAN, engage 또는 actuation authority를 소유하지 않는다.

<!-- HH_260810 - Summarized exactly what the current same-host live test proved. -->
## 현재 실측에서 확보한 것

<!-- HH_260810 - Added one bounded claim and limitation to each current evidence row. -->
| Change comment | 실측 | 결과 | 제한 |
|---|---|---|---|
| HH_260810 - Proved the live LiDAR-derived object path through local Planning. | live CARLA→detection→tracking→adapter→prediction→Planning | `BOUNDED PASS` | 동일 PC Domain 10→5 |
| HH_260810 - Proved one contiguous application-payload sequence. | adapter TX vs receiver bag | 203/203 ordered CDR match | packet PDR/cross-PC latency 아님 |
| HH_260810 - Proved deterministic route semantics for one captured start and goal. | start/goal/23 lane segments | exact route-semantic match | PC3 map authority·surveyed transform 아님 |
| HH_260810 - Proved object-dependent Planning response with no motion or CAN. | present→absent→fresh-present | stop→move→stop response class | moving closed loop·real dynamics 아님 |
| HH_260810 - Proved no checked actuation path in the bounded surrogate run. | engage/CAN endpoint/process audit | zero checked publishers/calls | physical vehicle 자체가 없었음 |

<!-- HH_260810 - Kept full Stage 1 and vehicle deployment explicitly unpassed. -->
full live/map-correct Stage 1은 아직 PASS가 아니다. PC4 local map과 PC3 candidate map hash가
다르고, PC3-approved numeric transform/control points/ego overlay가 없으며, camera 6/7 traffic-light
TensorRT containers가 `-11`로 종료된 full-stack defect가 남아 있다.

<!-- HH_260810 - Classified PC4 implementation and deployment work by status. -->
## 변경 항목

<!-- HH_260810 - Added one auditable state to each PC4 change row. -->
| Change comment | 항목 | 현재 상태 | 실제시험 전 조치 |
|---|---|---|---|
| HH_260810 - Preserved the canonical internal tracker topic. | active `tracking.launch.xml` | HEAD·backup과 byte-identical | 변경 금지; output adapter를 외부에 둠 |
| HH_260810 - Treated the implemented full-snapshot adapter as a prototype rather than a vehicle-boundary validator. | live object adapter | 동일-PC data path 동작; localhost-only; content/stamp/GID checks 일부 구현 | private source로 유지하고 current actor/session을 고정; map/transform binding과 distributed timing은 신규 acceptance 필요 |
| HH_260810 - Separated the surrogate gateway from an unimplemented LAN-capable vehicle gateway. | Domain 10→5 gateway | same-host test only | actual private-domain→10 exact allowlist와 per-domain NIC confinement을 별도 disabled config/runner로 구현·검증 후만 활성화 |
| HH_260810 - Required PC3-approved map identity and numeric transform. | map/transform | `LOCAL_ONLY_NOT_PC3_VERIFIED` | exact PC3 bundle·3+ control points·numeric transform SHA·overlay acceptance |
| HH_260810 - Required the real ego to follow recorded and then live PC3 state instead of simulated control integration. | `ego_actual` | code scaffold exists but archived-real-state runtime acceptance NOT RUN | Stage 1 immutable real-state replay PASS 후 Stage 2 reverse-only PC3 odom/accel live mirror; PC1 status는 telemetry only |
| HH_260810 - Distinguished the CARLA sensor hero from the required comparison model. | `ego_model` | `NOT IMPLEMENTED`; current CARLA hero is the synthetic sensor ego only | 별도 PC1-final-command-driven debug model을 구현·검증하고 vehicle state publish 금지 |
| HH_260810 - Corrected actor respawn lineage in future session evidence. | actor provenance | actor 40 metadata가 respawn actor 41 뒤에도 남은 이력 | session/event log에 old/new CARLA actor ID와 UUID lineage를 atomic 기록 |
| HH_260810 - Moved evidence out of volatile temporary storage. | evidence retention | 주요 live raw evidence가 `/tmp` 기반 | immutable timestamped run dir·mode 0400/0600·manifest/checksum로 보존 |
| HH_260810 - Separated the independent traffic-light crash from LiDAR-path acceptance. | camera 6/7 TensorRT | `-11` defect | engine compatibility 해결 또는 LiDAR-only scope를 명시하고 full-stack PASS 금지 |

<!-- HH_260810 - Defined the exact actual-deployment domain mapping. -->
## Actual Domain과 gateway 계약

<!-- HH_260810 - Kept PC4 in a private domain distinct from the vehicle domain. -->
- PC1–PC3 vehicle domain은 `10`을 유지한다.
<!-- HH_260810 - Kept the private-domain value provisional until fleet discovery proves it unused. -->
- PC4 private domain 후보는 `42`이며 실제 LAN에서 미사용·승인됨을 preflight로 확인한다.
<!-- HH_260810 - Prevented the Planning surrogate domain from becoming a deployment dependency. -->
- Domain `5`는 논문/Planning Simulator surrogate 시험용이며 실제 4-PC topology에 포함하지 않는다.
<!-- HH_260810 - Required one owned deny-by-default gateway process. -->
- gateway는 정확히 한 process, explicit source/destination domain, exact topic/type/QoS/remap만 허용한다.
  현재 localhost-only adapter 설정을 느슨하게 바꾸지 않고, gateway의 PC4-domain participant는 local
  source와 통신하며 vehicle-domain participant만 approved NIC를 사용하도록 per-domain DDS binding을
  별도 구현·실측해야 한다. official bridge가 이 분리를 만족하는지는 아직 증명되지 않았다.
<!-- HH_260810 - Required the actual vehicle gateway to start disabled and evidence-gated. -->
- proposed `vehicle_gateway_42_to_10.disabled.yaml`은 `enabled:false`로 추가하고 Stage 2 approval
  artifact 없이는 실행 경로가 없어야 한다.

<!-- HH_260810 - Defined the exact permitted PC4-to-vehicle routes. -->
PC4→Domain 10은 `/perception/pc4/virtual_obstacles/tracked_objects`
(`TrackedObjects`, `map`, reliable/volatile/depth 1)와 proposed live diagnostic
`/diagnostics/pc4/object_adapter` (`DiagnosticArray`)만 후보 allowlist다. 이 diagnostic은 기존 replay
allowlist의 `/diagnostics/pc4/replay_adapter` 및 `/diagnostics/pc4/tx_event_logger`를 live profile에서
대체하는 **contract revision requiring owner approval**이다. 한 profile에서 live와 replay diagnostic
sets를 동시에 허용하지 않으며 unselected set은 route와 vehicle-domain endpoint 모두 0이어야 한다.
services/actions/parameters와 wildcard route는 empty여야 한다.

<!-- HH_260810 - Defined the exact read-only vehicle-to-PC4 routes and private remapping. -->
Domain 10→PC4는 PC3 odom/accel와 PC1 velocity/steering/gear/control-mode, final `control_cmd`,
trajectory, route만 exact allowlist하며 PC4 내부 `/vils/in/**`로 remap한다. aggregate `/tf`와 map
services는 bridge하지 않고 map bundle은 offline manifest로 배포한다.

<!-- HH_260810 - Listed every topic family forbidden from PC4 into the vehicle domain. -->
금지 항목은 `/clock`, `/tf`, `/tf_static`, `/localization/**`, `/vehicle/**`, `/planning/**`,
`/control/**`, `/system/**`, `/to_can_bus`, `/from_can_bus`, CAN namespaces, engage/API set interfaces,
모든 service/action/parameter이다.

<!-- HH_260810 - Defined the live adapter output and validation contract. -->
## 현재 adapter 구현과 actual prerequisite

<!-- HH_260810 - Recorded only the checks that the current adapter actually performs. -->
현재 adapter는 exact input/output topic, allowed runtime domain 10 또는 42, localhost-only private DDS
profile hash, one source/output writer, positive/future/non-monotonic source stamp, `frame_id=map`, bounded
finite object content와 session-scoped UUID namespace를 검사한다. 첫 source stamp에 fixed source-to-wall
offset을 만들고 source delta를 보존하며 local session/sequence와 output CDR digest를 기록한다.

<!-- HH_260810 - Disclosed metadata and temporal checks that are not yet accepted at the vehicle boundary. -->
`map_hash`, `transform_hash`, `source_identity`, bridge/readiness state는 현재 operator-supplied metadata이며
object payload나 immutable manifest와 cryptographically bound되어 검증되지 않는다. `transform_failure_count`
역시 실제 transform validator의 증거가 아니다. 이전 live run에서는 simulator가 wall time보다 약
1.824배 빨라 fixed epoch output이 약 275.35초 future가 된 temporal FAIL이 있으므로, pacing 구현을
fresh run에서 wall-rate/nonfuture/age 기준으로 다시 승인해야 한다.

<!-- HH_260810 - Defined fail-closed vehicle-boundary work that remains unimplemented. -->
actual gateway 전에는 PC3 map/transform manifest binding, current actor lineage, source session metadata,
output CDR/stamp/count join, distributed receive age, old/future/replay policy를 별도 vehicle-boundary
validator에서 fail-closed로 구현해야 한다. 현재 adapter만으로 이 acceptance를 주장하지 않는다.

<!-- HH_260810 - Defined the full future adapter and actor contract after the missing gates are implemented. -->
## 목표 adapter와 actor 계약

<!-- HH_260810 - Preserved the exact source boundary from the canonical tracker. -->
- input은 private PC4 domain의 `/perception/object_recognition/tracking/objects`이다.
<!-- HH_260810 - Preserved the dedicated namespaced output contract. -->
- output은 `/perception/pc4/virtual_obstacles/tracked_objects`이며 각 message는 full snapshot이다.
<!-- HH_260810 - Required stable session-scoped identities and deterministic deletion. -->
- actor UUID는 session 내 actor lifetime 동안 stable하고 session 간 distinct하며 deletion은 다음
  fresh snapshot의 absence로 표현한다.
<!-- HH_260810 - Required source-time preservation across synchronized wall-clock mapping. -->
- simulator source time은 per-session fixed source→synchronized-wall epoch mapping으로 변환하고
  original source stamp를 event log에 별도 보존한다; receive time으로 stamp하지 않는다.
<!-- HH_260810 - Required bounded validation before every vehicle-facing acceptance. -->
- current adapter의 frame/stamp/content checks에 더해 PC2-side vehicle-boundary validator가 object CDR,
  output stamp/count, session/sequence, approved map/transform manifest를 exact join하고 old/future/replay를
  fail-closed로 검사한다.
<!-- HH_260810 - Distinguished empty scenes from source failures in health metadata. -->
- fresh empty snapshot과 missing/stale source를 health/sequence로 구분한다.
<!-- HH_260810 - Required actor identity and CARLA lifecycle evidence around every run. -->
- CARLA actor ID/type/role/blueprint/spawn transform/query/destroy와 adapter UUID/session을 연결한다.

<!-- HH_260810 - Defined the real and modeled ego boundaries on PC4. -->
## `ego_actual`과 `ego_model`

<!-- HH_260810 - Assigned real ego authority exclusively to PC3 odometry. -->
`ego_actual` pose/twist는 PC3 `/localization/kinematic_state`, acceleration은 PC3
`/localization/acceleration`을 사용한다. PC1 status는 DBC/unit 검증 전 비교 telemetry일 뿐이며
PC3 odometry twist를 덮어쓰지 않는다.

<!-- HH_260810 - Kept the recorded-real-state acceptance separate from the archived object replay result. -->
Stage 1에서는 immutable recorded real localization source를 `ego_actual`에 replay하여 frame/map/time,
pose/twist/acceleration count와 digest, read-only ownership을 검증해야 한다. 이 runtime acceptance는 아직
수행되지 않았으며 archived TrackedObjects 2,341개 replay나 D10→D5 object test로 대체되지 않는다.

<!-- HH_260810 - Kept the model ego independent and non-authoritative. -->
`ego_model`은 아직 구현되지 않았다. 목표 설계는 PC1 final control command를 별도 simulator dynamics에
적용해 expected response를 비교하는 debug object다. 현재 CARLA hero는 sensor ego이므로 `ego_model`로
간주하지 않는다. 향후 위치/yaw/velocity/steering/acceleration/latency error만 PC4 debug namespace에
남기며 vehicle-domain localization/status/TF를 publish하지 않는다.

<!-- HH_260810 - Listed exact PC4 ROS and simulator evidence for every actual run. -->
## PC4에서 취득할 데이터

<!-- HH_260810 - Added one auditable purpose to each PC4 evidence row. -->
| Change comment | Data or artifact | 목적 |
|---|---|---|
| HH_260810 - Captured the live sensor-to-tracker causal source chain. | `/carla_pointcloud`, preprocessed cloud, detected objects, tracker output | type/QoS/frame/stamp/count/rate and causal continuity |
| HH_260810 - Captured exactly what PC4 offers to the vehicle boundary. | namespaced TrackedObjects | seq/session/object UUID/full snapshot/CDR hash/rate/age |
| HH_260810 - Captured source health independently of object emptiness. | `/diagnostics/pc4/object_adapter` | readiness/map/transform/session/bridge/stale/reject counters |
| HH_260810 - Captured one independent transmitter-side observation. | TX event JSONL/logger | source/output digests and monotonic timing |
| HH_260810 - Preserved exact CARLA scene provenance. | world/map/build/settings/actor/sensor inventory | scenario reproduction and actor lifecycle |
| HH_260810 - Bound every coordinate transform to reviewed bytes and residuals. | map/transform manifest | PC3 hashes·numeric transform·control points·overlay |
| HH_260810 - Proved one owned gateway and exact allowlist. | gateway config/process/endpoint audit | domain/type/QoS/remap/GID and forbidden route 0 |
| HH_260810 - Measured host load and simulator pacing. | CPU/GPU/RSS/VRAM/tick/source-rate logs | 1x/2x/4x load characterization |
| HH_260810 - Measured the actual LAN only after four-host time acceptance. | PC4/PC2 bags and two-sided pcap | application MDR·latency/jitter and packet evidence kept separate |
| HH_260810 - Proved clean lifecycle and rollback. | process/session/port/actor pre-mid-post inventory | orphan 0·port residue 0·actor residue 0 |

<!-- HH_260810 - Required durable non-overwriting evidence storage. -->
각 run은 `/home/a/carla-autoware-universe/autoware/pc4_vils_runs/<UTC>-<run_id>/` 같은 새 경로에
저장하고 `umask 077`, source/config/binary hash manifest, root checksum, immutable source bag/pcap를
사용한다. `/tmp` 결과만으로 actual acceptance를 선언하지 않는다.

<!-- HH_260810 - Defined PC4 pass gates before opening the vehicle gateway. -->
## PASS와 중단 기준

<!-- HH_260810 - Required nonempty live tracking and lifecycle semantics in the private domain. -->
- live actor create/modify/delete와 nonempty detection/tracking/output, UUID lifetime, empty snapshot,
  source restart behavior를 private domain에서 반복 검증한다.
<!-- HH_260810 - Required staged map and overlay acceptance before vehicle-facing object publication. -->
- PC3 active map hashes·projection·3+ static control points를 먼저 승인한다. 그 뒤 PC4 egress가
  hard-disabled된 reverse-only profile로 `ego_actual`/live overlay를 승인하며, object/diagnostic egress
  profile은 이 두 gate와 gateway endpoint-zero 전환을 모두 통과한 뒤에만 연다.
<!-- HH_260810 - Required four-host synchronized time before reporting latency. -->
- 공통 Chrony acceptance 전에는 message continuity만 보고하며 one-way latency 수치를 내지 않는다.
<!-- HH_260810 - Scoped forbidden-interface proof to gateway crossings and vehicle-domain PC4 ownership. -->
- gateway route table에서 forbidden route/service/action/parameter가 0이고 Domain 10에 PC4/gateway-owned
  forbidden writer GID가 0이어야 한다. full private PC4 Autoware graph 내부의 `/clock`, `/tf*`,
  localization/planning topics는 예상되므로 내부 publisher 0을 요구하지 않는다.
<!-- HH_260810 - Required clean source health and honest scope around independent crashes. -->
- camera 6/7 TRT defect가 남으면 full-stack PASS를 금지하고 LiDAR-object-only scope로만 기록한다.

<!-- HH_260810 - Defined PC4 rollback without touching vehicle authorities. -->
## PC4 rollback

<!-- HH_260810 - Removed transport immediately in raw shadow and after de-selection in selector-aware stages. -->
1. Stage 2에서는 selector/cache를 기다리지 않고 gateway를 즉시 종료한다. Stage 3–4에서는 PC2가
   `real_only` 또는 `shadow`이고 accepted PC4 snapshot이 clear되었는지 bounded timeout으로 확인한 뒤
   gateway를 종료하며, 확인 실패 시에도 gateway를 즉시 닫고 fault로 기록한다.
<!-- HH_260810 - Proved zero vehicle-domain PC4 endpoints after gateway removal. -->
2. Domain 10의 PC4 object/diagnostic publisher와 PC4/gateway-owned forbidden endpoint가 0인지
   확인한다. unrelated vehicle owners의 정상 endpoints나 private source graph가 0일 필요는 없다.
<!-- HH_260810 - Stopped adapter and logger before simulator actors. -->
3. adapter/TX logger를 owned-session cleanup으로 종료하고 session-stop evidence를 저장한다.
<!-- HH_260810 - Removed only session-owned actors and sensors. -->
4. current session의 CARLA actors/sensors를 destroy하고 absence를 actor ID query로 검증한다.
<!-- HH_260810 - Stopped CARLA last and verified host residue. -->
5. CARLA를 종료하고 process/session/ports 2000–2002/locks/ROS endpoints residue 0을 확인한다.
<!-- HH_260810 - Preserved failed evidence and kept automatic re-arm disabled. -->
6. 실패 evidence를 보존하고 reconnect/restart 후 자동 arm하지 않는다.
