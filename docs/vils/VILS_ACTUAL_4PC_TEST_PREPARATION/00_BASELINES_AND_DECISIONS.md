<!-- HH_260810 - Recorded immutable VILS baselines and the evidence cutoff used for deployment decisions. -->
# Baselines and deployment decisions

<!-- HH_260810 - Identified the exact shared architecture snapshot used for every comparison. -->
## Git 기준

<!-- HH_260810 - Added one immutable identity row for each reviewed repository scope. -->
| Change comment | 범위 | Immutable commit | 판정 |
|---|---|---|---|
| HH_260810 - Pinned the shared architecture and three host histories. | Shared VILS docs | `4cb1a7959a4d53daa384b3be22d9149365ba3251` | 설계 기준; 구현 완료를 의미하지 않음 |
| HH_260810 - Pinned the PC1 Autoware v2 baseline. | PC1 Autoware v2.0.0 | `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8` | immutable rollback baseline |
| HH_260810 - Pinned the PC1 ROS workspace v2 baseline. | PC1 ros2_ws v2.0.0 | `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34` | immutable rollback baseline |
| HH_260810 - Pinned the PC2 Autoware v2 baseline. | PC2 Autoware v2.0.0 | `44f79b408ccbffb4cad6f56cbee68de841ac38ac` | immutable rollback baseline |
| HH_260810 - Pinned the PC2 ROS workspace v2 baseline. | PC2 ros2_ws v2.0.0 | `65515a2fc13266c95a812b11c1df7f7b7f5f184c` | immutable rollback baseline |
| HH_260810 - Pinned the PC3 Autoware v2 baseline. | PC3 Autoware v2.0.0 | `947dda782ce90e1d9768e57ae4337e3cf78eee1b` | immutable rollback baseline |
| HH_260810 - Pinned the PC3 ROS workspace v2 baseline. | PC3 ros2_ws v2.0.0 | `22f521ffb7ddeae893f377e226091641bb540efc` | immutable rollback baseline |

<!-- HH_260810 - Recorded exact content digests for the four remote architecture documents. -->
## Remote 문서 SHA-256

<!-- HH_260810 - Added one immutable content digest row for each owner-authored VILS document. -->
| Change comment | Git file | SHA-256 |
|---|---|---|
| HH_260810 - Bound decisions to the reviewed shared architecture bytes. | `VILS_SHARED_ARCHITECTURE.md` | `926490e17913764165760433c50774c686dc7c1bbb8a83c92ff400fc68cbff1a` |
| HH_260810 - Bound PC1 decisions to the reviewed history bytes. | `VILS_PC1_WORK_HISTORY.md` | `d3fb20966974d6c55083fe3d3cc917cc8da2504aa46d78b3611163a57dad575c` |
| HH_260810 - Bound PC2 decisions to the reviewed history bytes. | `VILS_PC2_WORK_HISTORY.md` | `0a400384ca0af6a002020e59e36906cfe3c3a9265755d30b0d8cd9f63870ee2b` |
| HH_260810 - Bound PC3 decisions to the reviewed history bytes. | `VILS_PC3_WORK_HISTORY.md` | `464243e80101fa35e754cc96ad7966a3f4b6570525a8cf960a83a4616ec238ab` |

<!-- HH_260810 - Linked immutable source pages close to the claims they support. -->
검토 원문은 [shared ownership and interface contract](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_SHARED_ARCHITECTURE.md#L70-L115),
[PC1 handoff](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC1_WORK_HISTORY.md#L643-L757),
[PC2 implementation gap](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC2_WORK_HISTORY.md#L537-L570),
[PC3 remaining gates and handoff](https://github.com/hwanhonglee/Autonomous-Driving/blob/4cb1a7959a4d53daa384b3be22d9149365ba3251/docs/vils/VILS_PC3_WORK_HISTORY.md#L743-L804)이다.

<!-- HH_260810 - Recorded the current PC4 artifacts used to derive real-deployment requirements. -->
## 현재 PC4 근거

<!-- HH_260810 - Added one exact digest and claim boundary to each current PC4 artifact row. -->
| Change comment | Artifact | SHA-256 | 사용 범위 |
|---|---|---|---|
| HH_260810 - Bound the completed live surrogate observation to its exact report. | `../PC4_DOMAIN10_TO_DOMAIN5_LIVE_ALIGNMENT_TEST.md` | `3a75e96d9b85abb9beab9491ca04b56b7be600d6a156b34b6dbdaacf6b7288b6` | 동일-PC live object path·route semantics·Planning response |
| HH_260810 - Bound the PC4 historical narrative to its current file. | `../VILS_PC4_WORK_HISTORY.md` | `9e4dacd8bbfe77224e42467bc1d1417b5a8ad8ad7b9c261ceec8d1f74cac83e0` | 구현 및 제한 이력 |
| HH_260810 - Proved that the canonical PC4 tracking launch remained untouched. | active `tracking.launch.xml` | `4597620eadc8780495d5c29053a42ae53783a50142ce974ac275a047bdce529f` | canonical internal topic preservation |
| HH_260810 - Bound the surrogate gateway configuration without promoting it to deployment. | `planning_gateway_10_to_5.domain_bridge.yaml` | `2f309339ca1e3af3d79895b2b6642f853365ab5c320018a4e8ab55cd93606898` | 동일-PC test only |
| HH_260810 - Bound the captured CARLA start and route reference. | `domain10_alignment_reference_v2.yaml` | `20d7c8f663c27890b61556a24ab8e1fe1e0584696db442b5ee01a35c19260398` | captured-run fixture only |

<!-- HH_260810 - Stated the exact bounded facts from the current PC4 live test. -->
현재 실측은 live CARLA LiDAR→CenterPoint→tracking→PC4 adapter→one-topic Domain 10→5
bridge→map prediction→Planning/Control을 관측했다. 같은 start/goal과 23개 ordered Lanelet segment,
203/203 contiguous application CDR payload, obstacle-present/absent/fresh-process-present의 서로 다른
trajectory response를 확인했다.

<!-- HH_260810 - Disclosed every evidence boundary that remains outside the current PC4 result. -->
이 실측은 동일 호스트·loopback DDS이다. PC2 validation/fusion, cross-PC LAN, packet capture,
four-host Chrony, PC3-authoritative map/transform, 실제 dynamics, physical MANUAL state, CAN TX,
engage 및 actuation은 검증하지 않았다. 따라서 `full live/map-correct Stage 1`과 실제 Stage 2는
아직 PASS가 아니다.

<!-- HH_260810 - Resolved the per-host change question by stage. -->
## 최종 변경 판정

<!-- HH_260810 - Added one stage-specific deployment decision row for each host. -->
| Change comment | PC | 실제 수정 여부 | 정확한 이유 |
|---|---|---|---|
| HH_260810 - Kept PC1 unchanged while PC2 owns the object boundary. | PC1 | Stage 1.5–4 core source `NO SOURCE CHANGE` | PC1은 이미 PC2 canonical `PredictedObjects`를 소비하며 PC4를 직접 구독하면 ownership 위반 |
| HH_260810 - Required a safe PC1 receive-only test path instead of the historical full bridge. | PC1 | Stage 1.5–4 test wrapper `CONFIG/DEPLOYMENT ONLY; REQUIRED` | 기존 `run_bridge`는 CAN RX·TX·control adapter를 함께 실행하므로 no-actuation 시험에 사용 금지 |
| HH_260810 - Identified the mandatory post-v2 implementation host. | PC2 | Stage 3–4 `NEW SOURCE REQUIRED` | v2에는 PC4 ingress validator·TTL·session/map gate·fuser·selector·accepted status가 없음 |
| HH_260810 - Preserved the PC3 object path and accepted authority boundary. | PC3 | Object source `NO SOURCE CHANGE` | PC3는 실제 map/localization/TF authority이며 PC4 object path owner가 아님; Chrony는 proposed source로 four-host 승인 대기 |
| HH_260810 - Required an atomic PC3 deployment decision only if the map convention changes. | PC3 | Map/GNSS 선택 `CONFIG/DEPLOYMENT ONLY` 또는 post-v2 atomic release | native map과 legacy GNSS offset convention을 섞으면 double offset 위험 |
| HH_260810 - Assigned all simulator-boundary implementation to PC4. | PC4 | Stage 1–4 `NEW SOURCE/CONFIG REQUIRED` | live adapter·approved transform·private-domain gateway·health/provenance·evidence 필요 |
| HH_260810 - Deferred moving actuation work until both command inputs fail closed. | PC1 | Stage 5 `NEW SOURCE REQUIRED` | cached command와 cruise/control-request source의 독립 age watchdog·request-bit clear·exclusive CAN writer/ID policy가 아직 acceptance gate |

<!-- HH_260810 - Preserved the exact generic launch seams instead of recommending topic renaming. -->
PC2 v2의 `tracking.launch.xml`에는 이미 `output/objects` 인자가 있고 `prediction.launch.xml`에는
이미 `input/objects` 인자가 있다. 따라서 generic canonical 내부 이름은 바꾸지 않고 outer
perception/VILS launch에서 physical tracker output과 fuser output을 연결하는 것이 권장 seam이다.
[PC2 tracking seam](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/object_recognition/tracking/tracking.launch.xml#L19-L44)과
[prediction seam](https://github.com/hwanhonglee/Autonomous-Driving/blob/44f79b408ccbffb4cad6f56cbee68de841ac38ac/src/universe/autoware.universe/launch/tier4_perception_launch/launch/object_recognition/prediction/prediction.launch.xml#L3-L12)을 사용한다.

<!-- HH_260810 - Defined branch and worktree policy for future implementation. -->
## Branch와 배포 정책

<!-- HH_260810 - Kept every released v2 ref immutable. -->
- 각 PC의 audited v2 commit에서 별도 read-only comparison worktree를 만든다.
<!-- HH_260810 - Required named post-v2 integration branches for experimental changes. -->
- 구현 후보는 `agent/vils-pc1-integration`, `agent/vils-pc2-integration`,
  `agent/vils-pc3-integration`, `agent/vils-pc4-integration` 같은 별도 branch에서 관리한다.
<!-- HH_260810 - Prevented installation overlays from hiding source and binary provenance. -->
- 각 run manifest에 source commit·dirty status·installed package path/hash·config hash를 함께 남긴다.
<!-- HH_260810 - Required an explicit reversible deployment package rather than manual live edits. -->
- 실제 PC에는 reviewed patch bundle과 manifest로만 배포하고 원본 v2 profile을 rollback target으로 유지한다.
<!-- HH_260810 - Prevented post-v2 prototypes from being described as released vehicle software. -->
- candidate branch 결과는 owner review와 독립 4-PC acceptance 전까지 `PROTOTYPE`으로 표시한다.
