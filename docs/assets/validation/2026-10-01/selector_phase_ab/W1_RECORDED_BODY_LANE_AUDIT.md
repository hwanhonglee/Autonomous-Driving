# W1 recorded body: 20 s clear, 150 s inset-sweep failure

<!-- HH_261001 - Keep actual lane geometry, predicted plan mathematics, and route success separate. -->

The unchanged W1 checkpoint is `b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76`. This read-only assessment uses the actual recorded Prius bounding box, full native CARLA poses, archived active OpenDRIVE, intended route lane IDs, and the original lane/body geometry routines. It does not infer containment from event counters or rewrite failed manifest status.

| Recorded scope | Body poses / sweeps | Outside raw intended lane | Outside 10 cm inset | Worst raw sweep clearance |
|---|---:|---:|---:|---:|
| 20 s | 400 / 399 | 0 / 0 | 0 / 0 | 0.408228 m |
| 150 s | 3,000 / 2,999 | 0 / 0 | 0 / 17 sweeps | 0.078149 m |

The 150 s gate is **FAIL** under the unchanged 10 cm inset requirement. The 17 failing sweep intervals end at native frames 3457–3473. Maximum inset outside area is 0.0122804 m², at 3467→3468, elapsed 108.10 s. This is an inset-only certification failure, not an observed body intrusion outside the raw sampled lane. The actual sampled body has minimum raw clearance 0.102196 m, just 2.196 mm inside the inset; its conservative swept envelope has only 78.149 mm raw clearance. No margin or threshold is relaxed.

At the worst witness, rear-reference recorded progress is 194.952573 m (independently projected actor-reference arc 196.377161 m), vx 2.328710 m/s, CTE 0.404004 m, acknowledged current throttle 0.118980, brake 0, steering −0.0290874, and control source `learned_trajectory_tracker`. These values describe the observed interval, not proof of the cause of its lateral position.

[Worst geometric witness](warmup_W1_150s_body_lane_v1/worst_sweep_witness.png) · [witness provenance](warmup_W1_150s_body_lane_v1/worst_sweep_witness.json) · [20 s summary](warmup_W1_20s_body_lane_v1/summary.json) · [150 s summary](warmup_W1_150s_body_lane_v1/summary.json)

The body/sweep checks are bounded snapshot-linear geometry, not proof of continuous physical motion. Every body polygon, sweep envelope, outside area, raw/inset clearance, and intended lane assignment is retained in `all_native_body_sweeps.jsonl.gz`. ACK reconstruction passes. Original episode/owner failure and absent goal completion remain unchanged; there is no map-wide claim, promotion, vehicle approval, or training admission.

## Predicted plans are a separate mathematical diagnostic

All 12 original plans are retained for each distinct inference: 2,196 candidates in 20 s and 17,796 in 150 s. The existing nine-bound checker is reused without changing its 1e−6 tolerance. These are model predictions, never expert targets or actual observed future trajectories. Because raw NN outputs have no independent body-yaw head, the ninth metric uses the XY-derived tangent; it is not a measured vehicle-yaw label.

All selected plans pass scalar speed and acceleration/deceleration checks. Strict all-nine counts are 44/183 selected in 20 s and 82/1,483 in 150 s. Many failures are tiny numerical overshoots at the decoder boundary, so the count alone must not be presented as gross physical infeasibility. The 150 s selected maxima include XY speed 8.33337093 versus 8.33333333 m/s, lateral acceleration 2.80014748 versus 2.8 m/s², XY acceleration 2.90006989, and XY deceleration −2.90000785 versus ±2.9 m/s². The largest curvature value is 0.27599563 versus 0.2 rad/m on candidate 1, frame 1386, future point 23: its XY step is only 0.000152049 m, speed 0.00151979 m/s, and implied lateral acceleration 6.38e−7 m/s². It is retained for low-displacement numerical sensitivity review rather than silently clipped or described as aggressive physical steering. `all_candidate_nine_bounds.jsonl.gz` contains every point and violation index. This math report does not replace the runtime gate or actual-body gate.

## Reproduction

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.audit_simulation_research_body_lane_20261001 -- --run storage/experiments/validation/2026-10-01/warmup_supervision_W1_town07_long_v1 --output /new/nonexistent/body_audit_output
```

Auditor SHA256 `6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac`; 20 s summary `9cee034847a57bad13c0ca688332bee3f5ccc218c2753f4ca3dda528923e1e4b`; 150 s summary `ea06a8e384e4a0f04bdf691390b947c61516ba1c00e321dd23e5398ce4ac178f`. Four body/candidate tests plus two failed-window census tests pass. No NN forward, training, simulator launch, shared-source edit, or original validator change is performed by these tools.
