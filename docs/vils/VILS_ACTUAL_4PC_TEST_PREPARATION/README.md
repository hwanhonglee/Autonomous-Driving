<!-- HH_260810 - Defined the actual four-PC VILS preparation package and its evidence boundary. -->
# VILS actual four-PC test preparation

<!-- HH_260810 - Kept this package at preparation status until the distributed vehicle test is executed. -->
Status: **IMPLEMENTATION AND DATA-ACQUISITION PLAN; ACTUAL FOUR-PC TEST NOT YET RUN**

<!-- HH_260810 - Recorded the purpose of comparing the shared Git architecture with current PC4 evidence. -->
이 폴더는 Git의 VILS shared architecture와 PC1·PC2·PC3 작업 이력, 그리고 현재 PC4의
CARLA→Planning Simulator 실측을 함께 비교하여 실제 차량 시험 전에 확정해야 할 변경점과
수집 자료를 PC별로 분리한다. PC1·PC2·PC3의 고정 `v2.0.0` 기준은 수정하지 않으며,
필요한 변경은 별도 read-only worktree에서 시작한 post-v2 VILS integration branch 후보로만
정의한다.

<!-- HH_260810 - Summarized the most important deployment decision before listing details. -->
## 핵심 결론

<!-- HH_260810 - Added one auditable classification to each host decision row. -->
| Change comment | PC | Stage 1.5–2 | Stage 3–4 | Stage 5 |
|---|---|---|---|---|
| HH_260810 - Kept the existing PC1 object boundary while requiring a non-actuating status source. | PC1 | Core source 변경 없음; 승인된 별도 receive-only status profile은 필수 | 기존 canonical `PredictedObjects` 소비 유지; object-path 변경 없음 | command와 control-request freshness watchdog 및 CAN writer/ID ownership을 post-v2에서 추가해야 함 |
| HH_260810 - Preserved PC2 core source and the existing subscription to the PC3-owned LiDAR relay. | PC2 | PC2에는 LiDAR 하드웨어가 없음; PC3 `pointcloud_before_sync` relay path를 baseline으로 유지·검증 | **신규 validator·TTL·session/map gate·state machine·physical-main fuser 필요**; direct canonical subscription은 별도 post-v2 migration 후보 | Stage 3–4 장기 안정성과 required-mode loss routing 통과 후 별도 승인 |
| HH_260810 - Kept PC3 as the real map localization and TF authority while leaving time acceptance pending. | PC3 | Object-path source 변경 없음; 실제 배포 map·localization·TF 확보; proposed Chrony source는 four-host 승인 필요 | 동일 map/localization/TF authority 유지; PC4 object subscriber/fuser 추가 금지 | 선택한 map/GNSS convention과 proposed post-v2 availability/MRM chain 승인 필요 |
| HH_260810 - Assigned the virtual-source adapter transform gateway and evidence roles to PC4. | PC4 | Live adapter·private domain·transform·evidence·disabled gateway 준비 | namespaced `TrackedObjects`와 health만 송신; canonical write 금지 | 실제 PC1–3의 승인 상태를 따르며 CAN·engage·actuation authority 없음 |

<!-- HH_260810 - Stated the single object-ownership path that every deployment profile must preserve. -->
실제 배포에서 PC4는 `/perception/pc4/virtual_obstacles/tracked_objects`만 제공하고, PC2가 이를
검증·선택·융합한 뒤 기존 map-based prediction으로 유일한 canonical
`/perception/object_recognition/objects`를 만든다. PC1은 그 canonical `PredictedObjects`를
기존 방식으로 소비하고, PC3는 실제 map/localization/TF authority를 유지한다. PC3 Chrony는
공통 time-source 후보이며 네 호스트가 같은 source와 허용 offset/jitter를 보이기 전에는 accepted
time authority로 간주하지 않는다.

<!-- HH_260810 - Separated the same-host surrogate topology from the real vehicle topology. -->
## Surrogate와 실제 배포의 Domain 대응

<!-- HH_260810 - Preserved the bounded purpose of the completed same-host test. -->
```text
이미 수행한 동일-PC 논리 시험
  Domain 10: PC4 CARLA source
      -> exact one-topic TrackedObjects bridge
  Domain 5: Planning Simulator surrogate
```

<!-- HH_260810 - Defined the target distributed topology without reusing the surrogate domains as deployment proof. -->
```text
실제 4-PC 배포 목표
  PC4 private Domain (현재 후보 42)
      -> deny-by-default one-topic gateway
  Vehicle Domain 10
      <- PC3 physical LiDAR/sensing PointCloud2
      -> PC2 PointCloud2 subscription/perception
      -> PC2 validation/fusion/prediction
      -> PC1 Planning/Control
      <- PC3 map/localization/TF; proposed common-time source pending four-host acceptance
```

<!-- HH_260810 - Prevented the bounded same-host evidence from becoming a distributed-system claim. -->
현재 Domain 10→5 결과는 live LiDAR-derived object가 Planning과 Control 출력까지 도달하고,
203개 application payload가 동일 호스트에서 연속 전달되며, 장애물 유무가 trajectory에 다른
반응을 만든다는 것을 보였다. 이 결과는 cross-PC DDS, vehicle LAN, PC2 validator/fuser,
Chrony, packet PDR, 실제 vehicle dynamics, CAN 또는 actuation을 검증하지 않았다.

<!-- HH_260810 - Indexed every preparation artifact by its operational purpose. -->
## 파일 구성

<!-- HH_260810 - Added one auditable description to each preparation artifact row. -->
| Change comment | 파일 | 용도 |
|---|---|---|
| HH_260810 - Centralized immutable source identities and claim boundaries. | `00_BASELINES_AND_DECISIONS.md` | Git 기준 commit·문서 hash·현재 PC4 근거·변경 판정 |
| HH_260810 - Isolated PC1 planning status and CAN safety preparation. | `PC1_ACTUAL_TEST_CHANGES_AND_DATA.md` | PC1 변경/비변경·수집 topic·Stage gate |
| HH_260810 - Isolated the only mandatory Stage 3 and Stage 4 source implementation. | `PC2_ACTUAL_TEST_CHANGES_AND_DATA.md` | PC2 validator/fuser/state-machine package 설계와 시험 |
| HH_260810 - Preserved PC3 authority while specifying map time and localization evidence. | `PC3_ACTUAL_TEST_CHANGES_AND_DATA.md` | PC3 map/localization/TF/Chrony 증거와 atomic 선택 |
| HH_260810 - Bound PC4 virtual-source work to namespaced data and evidence. | `PC4_ACTUAL_TEST_CHANGES_AND_DATA.md` | PC4 adapter/transform/gateway/provenance 준비 |
| HH_260810 - Made every permitted and forbidden cross-PC route machine-readable after comment filtering. | `VILS_ACTUAL_4PC_TOPIC_CONTRACT.csv` | topic/type/owner/QoS/frame/mode/검증 계약 |
| HH_260810 - Assigned evidence artifacts to exact hosts and stage gates after comment filtering. | `VILS_ACTUAL_4PC_EVIDENCE_MATRIX.csv` | PC별 bag/graph/map/time/network/CAN 자료 목록 |
| HH_260810 - Ordered the test from isolated PC4 prerequisites through blocked actuation. | `VILS_ACTUAL_4PC_STAGE_RUNBOOK.md` | Stage 1–5 순서·PASS·중단·fault/load matrix |
| HH_260810 - Defined manual physical-only recovery and evidence preservation. | `VILS_ACTUAL_4PC_ROLLBACK.md` | 단계별 rollback 및 zero-residue 확인 |

<!-- HH_260810 - Defined how operators must interpret change labels in the per-PC documents. -->
## 변경 상태 용어

<!-- HH_260810 - Added one exact meaning to each change-status label. -->
| Change comment | 상태 | 의미 |
|---|---|---|
| HH_260810 - Protected the audited v2 behavior from unnecessary edits. | `NO SOURCE CHANGE` | 해당 stage에서 v2.0.0 core source를 바꾸지 않음 |
| HH_260810 - Limited changes to launch configuration host policy or evidence tooling. | `CONFIG/DEPLOYMENT ONLY` | 새 branch의 wrapper/profile 또는 host 설정만 필요 |
| HH_260810 - Marked source work that must be implemented and reviewed before the stage. | `NEW SOURCE REQUIRED` | post-v2 integration branch에 신규 node/package/test 필요 |
| HH_260810 - Prevented a proposal from being read as an implemented interface. | `PROPOSED` | 이름·type·threshold가 owner review 전 후보임 |
| HH_260810 - Prevented execution across an unmet safety or scientific gate. | `BLOCKED` | 요구 근거가 없으므로 다음 stage 실행 금지 |

<!-- HH_260810 - Preserved the user's annotation and rollback requirements for future edits. -->
## 편집 규칙

<!-- HH_260810 - Required non-overwriting backups for every future modification of an existing file. -->
- 기존 파일을 수정하기 전 같은 위치에 비덮어쓰기 `(copy_org)` 백업을 만든다.
<!-- HH_260810 - Required the exact English annotation prefix around every changed logical block. -->
- 모든 변경 logical block에는 native syntax로 `HH_260810 - Description in English` 주석을 둔다.
<!-- HH_260810 - Kept all vehicle baselines immutable and separated experimental changes by branch. -->
- PC1·PC2·PC3 `v2.0.0` commit/tag는 직접 수정하거나 강제 이동하지 않는다.
<!-- HH_260810 - Kept execution authorization separate from documentation and implementation. -->
- 이 문서의 명령·파일 후보는 구현/검토 계획이며 실제 PC·LAN·CAN·engage 실행 승인이 아니다.

<!-- HH_260810 - Defined the required parser behavior for annotated CSV evidence contracts. -->
## CSV 읽기 규약

<!-- HH_260810 - Preserved the required HH comments while keeping data records rectangular. -->
두 CSV는 사용자 주석 규칙 때문에 각 header/data row 바로 앞에 `# HH_260810 - ...` 줄을 둔다.
일반 `csv.DictReader`에 원본을 바로 넣지 말고 UTF-8 text에서 빈 줄과 첫 non-space 문자가 `#`인
줄을 제거한 뒤 CSV parser에 전달해야 한다. comment를 제거한 결과는 topic contract가 17 columns,
evidence matrix가 10 columns로 균일해야 하며 이 조건을 CI/doc audit에서 검사한다.

<!-- HH_260810 - Provided one deterministic parser example for the annotated CSV files. -->
```python
from pathlib import Path
import csv

rows = (
    line for line in Path("VILS_ACTUAL_4PC_TOPIC_CONTRACT.csv").read_text(encoding="utf-8").splitlines()
    if line.strip() and not line.lstrip().startswith("#")
)
records = list(csv.DictReader(rows))
```
