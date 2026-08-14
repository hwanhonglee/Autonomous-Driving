<!-- HH_260810 - Defined manual and evidence-preserving rollback for every actual VILS stage. -->
# VILS actual four-PC rollback

<!-- HH_260810 - Required physical safety and source de-selection before process cleanup. -->
Rollback 원칙은 `vehicle safe state → PC2 source de-selection → gateway removal → PC4 cleanup →
PC1-PC3 baseline verification` 순서다. 자동 재연결이나 process restart는 re-arm 권한을 주지 않는다.

<!-- HH_260810 - Listed conditions that require immediate rollback. -->
## Rollback trigger

<!-- HH_260810 - Triggered rollback on any actuation-capable path in a no-actuation stage. -->
- Stage 1.5–4에서 CAN TX delta, sender/control adapter, `/to_can_bus`, engage/autonomous call이 발생한다.
<!-- HH_260810 - Triggered rollback on object or localization authority drift. -->
- canonical object writer 또는 `map→base_link` owner가 zero/duplicate/unapproved 상태가 된다.
<!-- HH_260810 - Triggered rollback on invalid virtual data reaching a selected output. -->
- stale/future/replay/map-mismatch/frame-mismatch/malformed object가 candidate/canonical에 들어간다.
<!-- HH_260810 - Triggered rollback when optional mode interrupts physical operation. -->
- PC4 loss가 physical perception·localization·trajectory continuity를 깨뜨린다.
<!-- HH_260810 - Triggered rollback when required-source loss becomes a clear road. -->
- required mode에서 PC4 loss가 unavailable/inhibit/MRM 대신 empty clear scene으로 해석된다.
<!-- HH_260810 - Triggered rollback on map time or lifecycle provenance failure. -->
- map/transform hash drift, Chrony source step, actor/session lineage mismatch, orphan process/actor가 발생한다.

<!-- HH_260810 - Defined selector-aware rollback only for stages where the PC2 integration exists. -->
## Stage 3–4 공통 rollback 순서

<!-- HH_260810 - Put the physical vehicle in its safest operator-controlled state first. -->
1. safety operator가 차량 정지와 MANUAL/PARK를 확인한다.
<!-- HH_260810 - Returned PC2 selection to the immutable physical-only state. -->
2. PC2 mode를 `real_only`로 전환하고 accepted PC4 snapshot·TTL cache·session·armed state를 clear한다.
<!-- HH_260810 - Required source de-selection evidence before removing transport. -->
3. canonical tracked/predicted가 physical-only이며 writer가 각각 승인된 하나인지 확인한다.
<!-- HH_260810 - Closed transport immediately if local source de-selection could not be proved. -->
   PC2 mode/cache clear 또는 physical-only proof가 bounded timeout 내 완료되지 않으면 gateway를 즉시
   종료하고 required-mode fault로 기록하며 다음 절차를 진행하지 않는다.
<!-- HH_260810 - Removed the virtual transport before stopping the virtual source. -->
4. PC4→vehicle gateway를 먼저 종료하고 Domain 10의 PC4 endpoints가 0인지 확인한다.
<!-- HH_260810 - Stopped PC4 application nodes with owned-session evidence. -->
5. PC4 adapter/TX logger를 종료하고 session-stop·reject/accept summary를 fsync된 evidence로 고정한다.
<!-- HH_260810 - Removed only the current PC4 session actors and sensors. -->
6. current session CARLA actor/sensor ID를 destroy하고 read-only ID query로 absence를 검증한다.
<!-- HH_260810 - Stopped the simulator after its actors and clients were clean. -->
7. CARLA를 종료하고 process/session/ports 2000–2002/locks residue가 0인지 확인한다.
<!-- HH_260810 - Removed PC2 experimental nodes after the canonical path is safe. -->
8. PC2 candidate/fuser/validator profile을 종료하고 v2 physical pipeline을 재확인한다.
<!-- HH_260810 - Removed only the PC1 receive path while independent rollback recorders remain active. -->
9. PC1 receive-only wrapper를 종료하고 sender/adapter/CAN endpoint가 계속 0인지 확인한다. 독립
   graph/process/CAN recorders는 아직 종료하지 않는다.
<!-- HH_260810 - Preserved PC3 authority until downstream consumers are safe. -->
10. PC3는 마지막에 정상 operator 절차로 종료하고 map/localization/TF evidence를 마감한다.
<!-- HH_260810 - Finalized independent evidence only after every stack and residue check completed. -->
11. 네 host의 post-stop process/endpoint/port/lock/actor/CAN counter 검사를 기록한 뒤 independent
   recorders를 마지막으로 종료하고 checksum manifest를 고정한다.

<!-- HH_260810 - Defined the simpler rollback for a failed physical baseline. -->
## Stage 1.5 rollback

<!-- HH_260810 - Kept PC4 disconnected when the physical baseline is unhealthy. -->
- PC4와 gateway를 계속 off 상태로 둔다.
<!-- HH_260810 - Kept independent recorders active while the baseline stack shuts down. -->
- receive-only wrapper와 PC1→PC2→PC3 또는 각 owner 승인 역순으로 stack을 종료하되 independent
  recorders는 post-residue/CAN check가 끝날 때까지 유지하고 마지막에 종료한다.
<!-- HH_260810 - Rejected attribution of a baseline fault to the absent virtual source. -->
- canonical/pointcloud/localization/TF/MRM/trajectory 문제를 PC4 원인으로 분류하지 않는다.
<!-- HH_260810 - Required a new complete baseline window after correction. -->
- physical 문제 수정 후 처음부터 새 `run_id`로 10분 baseline을 다시 수행한다.

<!-- HH_260810 - Defined transport-only rollback for a failed shadow stage. -->
## Stage 2 rollback

<!-- HH_260810 - Confirmed the physical safe state without waiting for a nonexistent Stage 2 selector. -->
- 차량 정지와 MANUAL/PARK를 확인하되 Stage 2에는 PC2 selector/cache/accepted status가 없으므로 이를
  기다리거나 조작하지 않는다.
<!-- HH_260810 - Removed the transport immediately without changing PC1-PC3 source. -->
- gateway를 즉시 종료하며 PC1–PC3 v2 source나 canonical object path는 변경하지 않는다.
<!-- HH_260810 - Proved that every PC4-owned vehicle-domain endpoint disappeared. -->
- Domain 10 PC4/gateway-owned object·diagnostic·forbidden endpoint가 0인지 확인한다.
<!-- HH_260810 - Required the physical baseline to remain invariant after source removal. -->
- PC2 canonical writer/content, PC3 localization/TF, PC1 trajectory, CAN TX delta가 Stage 1.5 상태로
  돌아왔는지 확인한다.
<!-- HH_260810 - Removed PC4 owned processes and actors after transport closure. -->
- PC4 adapter/actors/CARLA를 owned order로 종료하고 zero residue를 확인한다.

<!-- HH_260810 - Defined candidate-fusion rollback without touching canonical Planning. -->
## Stage 3 rollback

<!-- HH_260810 - Returned candidate validation to shadow before stopping transport. -->
- PC2 source manager를 `shadow` 또는 `real_only`로 전환하고 candidate cache/session을 clear한다.
<!-- HH_260810 - Required canonical physical continuity before removing the gateway. -->
- canonical PredictedObjects가 계속 physical-only이며 publisher/GID가 승인된 하나인지 확인한다.
<!-- HH_260810 - Removed PC4 transport and retained all reject and association evidence. -->
- gateway를 종료하고 PC4를 정리하며 fault seed/schedule와 association/reject logs를 보존한다.
<!-- HH_260810 - Restored the immutable PC2 profile if the integration process remains unhealthy. -->
- integration branch launch가 clean stop되지 않으면 v2 exact profile로 atomic 복원하고 새 baseline을 요구한다.

<!-- HH_260810 - Defined canonical-fusion rollback in source-selection order. -->
## Stage 4 rollback

<!-- HH_260810 - Required explicit physical selection before any node termination. -->
- stationary MANUAL/PARK에서 PC2 `real_only` 전환이 성공하고 accepted status가 inactive인지 확인한다.
<!-- HH_260810 - Required sole physical canonical ownership before closing the gateway. -->
- physical tracker와 기존 predictor path, canonical publisher 1개, baseline trajectory를 확인한다.
<!-- HH_260810 - Removed the source only after PC2 became safe. -->
- gateway→PC4 adapter→actors→CARLA 순으로 종료한다.
<!-- HH_260810 - Restored any implemented parked-only availability prototype by exact manifest. -->
- reviewed post-v2 PC3 availability/MRM prototype과 PC1 gate test config를 실제로 배포했던 경우에만
  pre-test manifest로 atomic 복원한다; v2에 해당 기능이 존재한다고 가정하지 않는다.

<!-- HH_260810 - Blocked Stage 5 because a complete runtime safety rollback is not yet accepted. -->
## Stage 5 rollback status

<!-- HH_260810 - Prevented a process stop from being mistaken for a safe physical fallback. -->
현재 Stage 5의 승인된 runtime rollback은 없다. 기존 PC1 control-to-CAN path가 stale cached command를
반복할 수 있고 exclusive CAN ownership도 닫히지 않았으므로 단순 process kill을 safety rollback으로
간주할 수 없다. 별도 safety review가 watchdog·request-bit clear·CAN writer allowlist·manual takeover·
E-stop·MRM·0.833 m/s enforcement를 승인하기 전 Stage 5를 시작하지 않는다.

<!-- HH_260810 - Defined per-host post-rollback verification. -->
## Host별 rollback 확인

<!-- HH_260810 - Added one exact verification set to each host row. -->
| Change comment | PC | 확인 항목 |
|---|---|---|
| HH_260810 - Reproved Planning input and zero actuation on PC1. | PC1 | canonical input fresh; baseline trajectory; sender/adapter 0; `/to_can_bus` endpoints 0; CAN TX delta 0; engage calls 0 |
| HH_260810 - Reproved stage-appropriate physical-only canonical ownership on PC2. | PC2 | Stage 1.5–2: existing physical tracker and one canonical predictor with no selector/cache; Stage 3–4: mode `real_only`, accepted cache empty, physical tracker fresh, one canonical predictor, VILS candidate endpoints 0 |
| HH_260810 - Reproved authoritative map localization and TF on PC3. | PC3 | approved map manifest; odom/accel fresh; one `map→base_link`; MRM baseline; no PC4-derived authority |
| HH_260810 - Reproved source and transport residue absence on PC4. | PC4 | gateway/adapter/CARLA/actor/sensor/process/port/lock endpoints 0; private domain clean |

<!-- HH_260810 - Defined immutable baseline restoration without partial file rollback. -->
## Source와 config 복원

<!-- HH_260810 - Restored PC1 from its exact audited releases and verified installed provenance. -->
- PC1: Autoware `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8`, ros2_ws
  `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34` approved profile; resolved checkout와 installed hashes 검증.
<!-- HH_260810 - Restored PC2 from its exact audited releases and verified installed provenance. -->
- PC2: Autoware `44f79b408ccbffb4cad6f56cbee68de841ac38ac`, ros2_ws
  `65515a2fc13266c95a812b11c1df7f7b7f5f184c` physical-only profile; resolved checkout와 installed hashes 검증.
<!-- HH_260810 - Restored PC3 as one atomic map and localization unit with full identities. -->
- PC3: Autoware `947dda782ce90e1d9768e57ae4337e3cf78eee1b`, ros2_ws
  `22f521ffb7ddeae893f377e226091641bb540efc`와 pre-test map/projector/GNSS-offset manifest 전체;
  resolved checkout와 installed hashes 검증.
<!-- HH_260810 - Restored PC4 to private isolated mode with every vehicle gateway disabled. -->
- PC4: actual gateway disabled, adapter private-only, canonical/vehicle/CAN routes 없음.

<!-- HH_260810 - Required evidence preservation even when rollback succeeds. -->
## Evidence 보존

<!-- HH_260810 - Preserved original failure artifacts without in-place repair. -->
- failed bag/pcap/JSONL/log/config/process snapshot을 수정하거나 덮어쓰지 않는다.
<!-- HH_260810 - Recorded rollback as a new event sequence rather than rewriting the failed run. -->
- rollback 시작·mode transition·gateway stop·cache clear·process stop·post-check를 append-only timeline으로 남긴다.
<!-- HH_260810 - Required independent checksums for the failed and recovered states. -->
- failure와 rollback-complete artifact 각각 별도 SHA-256 manifest를 만든다.
<!-- HH_260810 - Prevented automatic continuation after a successful cleanup. -->
- zero residue가 PASS여도 다음 run은 새 `run_id`, fresh processes, manual re-arm과 Stage 1.5 재확인으로 시작한다.
