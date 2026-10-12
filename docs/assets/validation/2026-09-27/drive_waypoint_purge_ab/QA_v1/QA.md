# Town01 right-turn BasicAgent waypoint purge B/C raw QA

C status: **NECESSARY_CHECKS_CLEAR_NOT_ADMITTED**. Dataset admitted: **false**. Training started: **false**. Learned control: **false**.

Both arms use one pinned Town01 route and BasicAgent expert settings except `base_min_distance` 3.0 → 1.5 m; normalized steering cap remains 0.45. This is one same-site expert-control diagnostic, not a new training site.

| Raw measure | B | C |
| --- | ---: | ---: |
| Selected camera anchors | 360 | 363 |
| Selected span (s) | 35.900000535 | 36.20000054 |
| Peak XY curvature (/m) | 0.18584991563123351 | 0.19145867432240396 |
| Curvature violating anchors | 0 | 0 |
| Unique violating 100 ms ticks | 0 | 0 |
| Maximum independent CTE (m) | 1.3702200367507655 | 0.7031983349737531 |
| Minimum selected speed (m/s) | 2.334808279113687 | 2.3127477170693376 |
| Maximum normalized current steer | 0.44999998807907104 | 0.44999998807907104 |
| Maximum measured tire angle (rad) | 0.4619739674239415 | 0.46209709398601834 |
| Turn signed yaw change (rad) | -1.3549752167934745 | -1.4630112732605554 |
| Goal reached | True | True |

The separate lane gate projects the recorded CARLA actor pose onto the same active OpenDRIVE route. Its frame range may differ from the base_link camera anchor window. It tests full actor polygons at 20 Hz and bounded snapshot-linear sweeps against the sampled raw lane and a 10 cm inset.

| Lane measure | B | C |
| --- | ---: | ---: |
| Lane gate status | FAIL | PASS |
| Frames checked | 720 | 727 |
| Raw outside bodies | 68 | 0 |
| Inset outside bodies | 78 | 0 |
| Raw outside sweeps | 75 | 0 |
| Inset outside sweeps | 84 | 0 |
| Worst raw body outside (m²) | 0.8378544202852405 | 0.0 |
| Worst inset body outside (m²) | 1.1655467331878109 | 0.0 |
| Worst raw sweep outside (m²) | 1.1049803314517235 | 0.0 |
| Worst inset sweep outside (m²) | 1.4763120998943602 | 0.0 |

C failed checks: none.

All selected anchors, each real 64-step future and the unchanged original nine motion bounds are retained in the companion JSONL/summary. The lane gate does not prove exact physical continuous time, and no result here admits data or proves learned/Autoware driving.

C's 363 selected anchors span 36.2 s with minimum native speed 2.313 m/s.
All 363 × 64 = 23,232 actual future points precede the brake tail; all nine
unchanged bounds have zero violating anchors, including curvature 0.191459 /m
against the 0.2 /m limit. All 7,866 six-camera JPEGs decode and align; ACK
alignment passes, collision and lane-invasion sensor counts are zero, and the
owned lifecycle ended with its port released. These necessary checks do not
change the no-admission decision.

The [B/C top-down plot](bc_topdown_raw_turn.png) centers C's ego at its
pre-reported peak-curvature anchor and overlays the original route centerline
and both measured paths. It does not draw lane boundaries; the independently
computed lane result is [C_lane_gate_30_120.json](C_lane_gate_30_120.json).
The [C six-camera contact sheet](C_raw_six_camera_first_middle_last.png)
preserves full native 640×360 pixels at the fixed first/middle/last selected
anchors (2679, 3041, 3403). The [recipe](visual_recipe.py) and
[hash manifest](visual_manifest.json) pin all source and output bytes.
These are offline BasicAgent expert diagnostics, not CARLA/Autoware GUI or
learned-driving footage. Focused raw, ACK, nine-bound and lane-gate tests:
**160 passed**.
