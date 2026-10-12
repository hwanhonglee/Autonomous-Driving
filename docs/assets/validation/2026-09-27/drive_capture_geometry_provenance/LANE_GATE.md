# Future-capture intended-lane body gate

<!-- HH_260906 - Distinguish sampled body intrusion from numerical margin failure and reject training admission. -->

**Town01 right B, 9 km/h, independently projected 30–120 m window: FAIL; not admitted to training.** The archived [machine-readable audit](B_lane_gate_30_120.json) covers 720 contiguous 20 Hz actor snapshots and 719 bounded adjacent-snapshot sweep envelopes. The [worst-turn view](B_lane_worst_witness.png) plots the recorded road-208 lane (green), its 10 cm inner corridor (blue), and the Prius body/sweep (red). This is a BasicAgent expert recording, not learned-model or Autoware closed-loop driving.

| Test | Outside un-eroded sampled intended lane | Outside 10 cm inner lane |
| --- | ---: | ---: |
| Actor body at 20 Hz | 68/720 frames; maximum 0.838 m² | 78/720; maximum 1.166 m² |
| Conservative snapshot-linear body sweep | 75/719 intervals; maximum 1.105 m² | 84/719; maximum 1.476 m² |

The earliest inner-corridor failure is the frame 2727–2728 sweep at independent arc 44.956–45.075 m: 0.00764 m² outside the 10 cm inset, but **0 m² outside the un-eroded sampled lane**. It is a certification-margin failure, not evidence by itself of physical lane intrusion. At frame 2669 a 0.180 m² inset-only discrepancy came from the ~2 mm gap where recorded roads 18 and 208 meet; the validator permits only a ≤5 mm numerical closure of *consecutive route-listed lane polygons*, for both raw and inset comparisons. It never adds an arbitrary adjacent driving lane.

The worst inner-corridor witness is frames 2766–2767 at independent arc 50.520–50.785 m, while the intended lane is only `(road 208, section 0, lane -1)`. Its interpolated sweep is **1.105 m² outside even the un-eroded sampled lane**, and 1.476 m² outside the 10 cm inset. The plotted vehicle body cuts across the inside of the mapped turn. This is a substantive recorded-lane intrusion under the archived OpenDRIVE geometry, not a 10 cm margin artifact. CARLA lane-invasion sensor event count of zero does not override the full-body polygon result.

The [validator](../../../../../tools/e2e/audit_future_actor_lane_containment.py) is deliberately future-only. It requires the same episode's `capture_contract.actor_body_geometry` (CARLA actor-local 3-D box), `active_map_opendrive.xodr` and SHA256, the archived route SHA256, a recognized collector source SHA256, the same initialized actor ID, tick-bound CARLA actor poses, and contiguous 20 Hz frames. It projects each raw CARLA actor pose onto the archived route independently of `route_progress_m`. A duplicated route waypoint may be collapsed for geometry **only** when its XYZ, road/section/lane IDs, and distance are identical; Town01 B has two such command-only duplicates. At junctions, `Map.get_waypoint` can select a different overlapping road (208 vs 218), so the validator matches only the route-declared road/section/lane through bounded `get_waypoint_xodr` centerline search, rejecting mismatches or ambiguous matches. A body may straddle a road-ID seam only if both lanes are consecutive route-listed lanes intersecting its bounded actor-arc interval.

The output is not an exact physical continuous-time proof. OpenDRIVE lane edges are tessellated at 5 cm and eroded by 10 cm; CARLA 3-D box vertices are projected to ground. The sweep encloses the two 20 Hz body polygons plus a rotation bound under **snapshot-linear translation/Euler interpolation**; unobserved true motion between ticks, map construction error, road-surface semantics, and real-vehicle safety remain unproven. `PASS` would mean only this bounded offline geometric gate passed, **not** that an episode is trainable or the learned model can turn safely. `UNSUPPORTED` is returned if source geometry, lane identity, frame continuity, or the bounded interpolation cannot be established; `FAIL` records any tested containment violation.

Reproduce without a CARLA server or GPU, from `/home/a/autoware_e2e` (use fresh output names because the command refuses overwrite):

```bash
python3 tools/e2e/audit_future_actor_lane_containment.py \
  --episode storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode \
  --start-progress-m 30 --end-progress-m 120 \
  --output /tmp/town01_B_lane_gate_recheck.json \
  --plot-output /tmp/town01_B_lane_worst_recheck.png
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_audit_future_actor_lane_containment.py
```

Exit code 2 is expected for this failing episode. No raw file was modified, no additional CARLA capture/PRO6000 training was run for this gate, and `training_data_approved` remains `false`.
