# Independent non-TEST right-turn scout: no capture route qualified

<!-- HH_260928 - Preserve the bounded offline search and all unchanged driving gates. -->

Status: **NO_PHYSICAL_TURN_PASS**. This is a static route search, not a CARLA drive or a trainable episode. The [report](report.json) pins a packaged `C_track_1_0_7` OpenDRIVE source with SHA256 `3f760b8e…37ec7be27`, the archived [moving cohort split plan](../../../../../storage/experiments/training/2026-09-15/moving_cohort_v1/cohort.json) with six C-track TRAIN slots, the existing `carla_vad_30kph_v2` *geometry* profile, the route serializer, and the 1 m offline CARLA GlobalRoutePlanner. This scout did not open Town03 DEV or reserved/held-out TEST maps. The packaged XODR has **not** been matched to a live CARLA world.

The fixed search enumerated all 65 right-heading driving connectors without a connector signal reference. For each connector, it tried every `entry.previous(upstream)[i]` × `exit.next(downstream)[j]` pair at exactly four distance scopes. A route had to be at least 146 m long, traverse the selected connector, contain exactly one `RIGHT` block and no other maneuver, put the turn inside the unchanged 30–120 m window, and pass the existing physical-turn profile. That profile name is a route-shape check, not approval to drive at 30 km/h.

| Upstream / downstream | Endpoint pairs | ≥146 m | Selected connector | Single RIGHT | Turn in window | Physical PASS |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 50 / 90 m | 650 | 342 | 238 | 2 | 2 | 0 |
| 60 / 80 m | 692 | 343 | 233 | 1 | 1 | 0 |
| 60 / 100 m | 896 | 493 | 304 | 1 | 1 | 0 |
| 70 / 80 m | 777 | 422 | 274 | 0 | 0 | 0 |
| **Total** | **3,015** | **1,600** | **1,049** | **4** | **4** | **0** |

The best rejected path by the fixed ranking is junction 1106, road 1117/lane −1, `previous(60m)[0] → next(100m)[0]`: 175.553 m total and RIGHT at 60.216–75.564 m. Its exact endpoint coordinates and route hash are in the report. The unchanged physical gate rejects an 18.007% five-metre uphill grade at route 38.695–43.695 m against its 9.983% cap. A physical-turn failure remains a failure; this route is not a capture proposal. A connector lacking an OpenDRIVE signal reference also does not prove that the *entire route* is free of stopping signals. No body/corridor, actual 30-second uninterrupted `>0.5 m/s` movement, 64 measured future points, or `≤2.5 m` goal result was observed. The goal tolerance and those driving gates remain unchanged. `capture_authorized=false`, `dataset_admitted=false`, and `training_executed=false`.

Reproduce offline from the workspace root, writing to a **fresh** path (the published report is intentionally overwrite-protected):

```bash
python3 tools/workspace_python.py tools/e2e/scout_independent_c_track_right.py \
  --output /tmp/c_track_right_scout_recheck.json
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_scout_independent_c_track_right.py
```

The next useful experiment is still route scouting at a separate non-TEST physical site. Freeze a specific live-XODR-matched route and endpoint before a BasicAgent capture, screen its full signal approach and 10 cm inset Prius body/sweep, then run the original 9 km/h, 30–120 m, 30-second movement, 6.4-second future, nine-motion-bound, and 2.5 m goal checks. This search does not warrant another live capture or a model improvement claim.
