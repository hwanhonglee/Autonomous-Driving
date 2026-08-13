<!-- HH_260810 - Defined the audited scope of the next-day four-PC preparation set. -->
# VILS next-day four-PC test preparation

Status: **PREPARATION ONLY — no actual four-PC or vehicle acceptance has been completed**

Date: 2026-08-13

Target test date: 2026-08-14

Base architecture: `../VILS_SHARED_ARCHITECTURE.md`

<!-- HH_260810 - Preserved every released vehicle-PC baseline as immutable evidence. -->
## Immutable baselines

| Change comment | Host | Audited baseline |
|---|---|---|
| HH_260810 - Preserved the released PC1 Autoware baseline. | PC1 Autoware | `c8c6a0b795d91ccf9f9efe95e54084dd8c0481d8` |
| HH_260810 - Preserved the released PC1 ROS overlay baseline. | PC1 ros2_ws | `8658adf6512fd3374ebcd2b4a3a0ba2656a0ce34` |
| HH_260810 - Preserved the released PC2 Autoware baseline. | PC2 Autoware | `44f79b408ccbffb4cad6f56cbee68de841ac38ac` |
| HH_260810 - Preserved the released PC2 ROS overlay baseline. | PC2 ros2_ws | `65515a2fc13266c95a812b11c1df7f7b7f5f184c` |
| HH_260810 - Preserved the released PC3 Autoware baseline. | PC3 Autoware | `947dda782ce90e1d9768e57ae4337e3cf78eee1b` |
| HH_260810 - Preserved the released PC3 ROS overlay baseline. | PC3 ros2_ws | `22f521ffb7ddeae893f377e226091641bb540efc` |
| HH_260810 - Bound this preparation set to the reviewed shared contract. | Shared VILS docs | `4cb1a7959a4d53daa384b3be22d9149365ba3251` |

<!-- HH_260810 - Prevented tomorrow's test from rewriting any released vehicle baseline. -->
Do not amend, force-push, overwrite, or patch these v2.0.0 refs. Any later PC2
validator/fuser/selector prototype belongs on a new post-v2 integration branch and is outside the
initial baseline and raw-shadow test.

<!-- HH_260810 - Separated verified same-host evidence from unverified distributed behavior. -->
## Current evidence boundary

| Change comment | Item | Current status | What it does not prove |
|---|---|---|---|
| HH_260810 - Kept the archived object replay claim bounded to one host. | Archived 2,341-message PC4 replay | `BOUNDED PASS` | Vehicle LAN, cross-PC PDR/latency, PC2 fusion, real dynamics |
| HH_260810 - Kept the Domain 10-to-5 planning result bounded to a same-host surrogate. | CARLA-to-Planning-Simulator SIL | `BOUNDED SAME-HOST PASS` | PC1-PC3 runtime, Chrony, physical network, CAN, actuation |
| HH_260810 - Recorded that the released LiDAR configuration still needs live vehicle proof. | PC3 Pandar64 to PC2 perception | `CONFIGURED, NOT YET CROSS-PC ACCEPTED` | Sustained real payload delivery and end-to-end perception continuity |
| HH_260810 - Prevented incomplete map and time work from becoming a Stage 2 acceptance claim. | PC4 map/time/ego alignment | `NOT PASSED` | Map-correct aligned shadow or moving VILS |
| HH_260810 - Recorded the missing vehicle-LAN transport boundary. | PC4 private-domain to vehicle-domain gateway | `NOT READY FOR ACTUAL LAN` | Current runners are localhost-only and must not be reused unchanged |

<!-- HH_260810 - Limited the next-day activity to the safest meaningful stages. -->
## Intended next-day scope

1. **Stage 1.5 — existing three-PC baseline:** PC4 stays disconnected. Prove that PC3 physical
   sensing/map/localization feeds PC2 perception and that PC1 produces a stable planning baseline.
2. **Stage 2A — raw transport rehearsal:** only if all preflight gates pass, transmit the PC4
   namespaced `TrackedObjects` stream to an isolated vehicle-domain logger. Do not connect it to
   PC2 canonical perception or PC1 Planning.
3. **Stage 2B — aligned shadow acceptance:** remains blocked until map hashes, numeric transform,
   common time, `ego_actual`, and the reviewed LAN gateway all pass.

<!-- HH_260810 - Distinguished a useful transport rehearsal from a complete architecture stage. -->
Stage 2A may produce application-level continuity and restart evidence, but it must be reported as
`TRANSPORT REHEARSAL / NOT STAGE 2 ACCEPTANCE` while alignment or time gates remain open.

<!-- HH_260810 - Enumerated the hard preflight blockers that operators must not bypass. -->
## Hard preflight gates

- PC1 needs an approved receive-only vehicle-status path or equivalent safe status source; the
  historical `run_bridge` is actuation-capable and is forbidden for Stages 1.5 through 4.
- PC3 must prove the live Pandar64 publisher, PC3 relay, PC2 subscriber, localization, and
  `map -> base_link` ownership at runtime. Source code configuration alone is not a PASS.
- PC4's current adapter and same-host gateway runners require `ROS_LOCALHOST_ONLY=1`. A separate,
  reviewed LAN-capable gateway profile must keep source nodes isolated while exposing only the
  exact allowlist to Domain 10.
- Do not invent a PC4 vehicle-LAN IP address. The network owner must allocate and verify an unused
  address, route, interface binding, firewall, and DDS participant path.
- Do not calculate one-way latency until all four hosts prove one accepted clock source and an
  approved offset/jitter window.
- Do not claim map-correct alignment until the active PC3 map bundle, PC4 bundle, projector, numeric
  simulator transform, and at least three non-collinear control points are accepted.

<!-- HH_260810 - Made the four host responsibilities explicit for the field team. -->
## Host ownership summary

| Change comment | Host | Next-day responsibility | Explicitly forbidden |
|---|---|---|---|
| HH_260810 - Kept PC1 on its existing canonical planning boundary. | PC1 | Planning baseline, canonical `PredictedObjects` observation, read-only PC4 shadow logger if approved | PC4 direct Planning input, `run_bridge`, CAN sender, engage, actuation |
| HH_260810 - Kept PC2 as the physical perception and final canonical object owner. | PC2 | Consume PC3 relay output; detection/tracking/prediction; canonical writer evidence | LiDAR driver, PC3 relay, PC4 fusion/selection during Stage 2A |
| HH_260810 - Kept PC3 as physical sensing map localization and TF owner. | PC3 | Pandar64, pointcloud relay, map, localization, TF, current system diagnostics | Planning, perception ownership, PC4 object fusion, CAN writer |
| HH_260810 - Limited PC4 to a namespaced virtual source and evidence role. | PC4 | CARLA/replay source, object adapter, session evidence, reviewed raw-shadow gateway | Vehicle canonical object, `/clock`, `/tf*`, localization, vehicle, planning, control, system, CAN |

<!-- HH_260810 - Indexed each owner document for field use. -->
## Files

- `PC1_TOMORROW_TEST.md` — PC1 preflight, observations, no-actuation proof, stop conditions.
- `PC2_TOMORROW_TEST.md` — PC2 physical perception baseline and canonical ownership.
- `PC3_TOMORROW_TEST.md` — PC3 real LiDAR, relay, map, localization, TF, and time evidence.
- `PC4_TOMORROW_TEST.md` — PC4 source, gateway readiness, session, map, and transport limits.
- `STAGE_1_5_AND_2_RUNBOOK.md` — ordered start, gate, fault, and shutdown procedure.
- `EVIDENCE_AND_GO_NO_GO.md` — common manifest, artifacts, claim limits, and final decision sheet.

<!-- HH_260810 - Preserved the single canonical object authority across every allowed next-day step. -->
## Non-negotiable object path

```text
PC3 physical Pandar64 PointCloud2
  -> PC2 detection/tracking/prediction
  -> /perception/object_recognition/objects
  -> PC1 Planning

PC4 virtual TrackedObjects
  -> /perception/pc4/virtual_obstacles/tracked_objects
  -> Stage 2A logger only
  -X-> PC2 canonical path
  -X-> PC1 Planning
```

<!-- HH_260810 - Kept physical actuation outside the entire preparation set. -->
CAN transmission, autonomous engage, physical movement, control-gate bypass, and Stage 3 through
Stage 5 claims are outside this preparation set.
