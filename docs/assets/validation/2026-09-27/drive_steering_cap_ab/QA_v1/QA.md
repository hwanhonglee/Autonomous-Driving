# Town01 right-turn steering cap A/B raw QA

B status: **FAIL_CLOSED**. Dataset admitted: **false**. Training started: **false**. Learned control: **false**.

Both arms use the same pinned Town01 route and effective BasicAgent settings except the normalized steering cap, 0.80 → 0.45. B's collector adds geometry recording; this is one same-site expert-control diagnostic.

| Raw measure | A | B |
| --- | ---: | ---: |
| Selected camera anchors | 360 | 360 |
| Selected span (s) | 35.900000535 | 35.900000535 |
| Peak XY curvature (rad/m) | 0.22988931336413704 | 0.18584991563123351 |
| Curvature violating anchors | 63 | 0 |
| Unique violating 100 ms ticks | 1 | 0 |
| Maximum independent CTE (m) | 1.378498098225662 | 1.3702200367507655 |
| Maximum normalized current steer | 0.541984498500824 | 0.44999998807907104 |
| Maximum measured tire angle (rad) | 0.5460525941715589 | 0.4619739674239415 |
| Turn signed yaw change (rad) | -1.3540242019682072 | -1.3549752167934745 |
| Goal reached | True | True |

B failed checks: full_body_in_intended_lane.

The current actor bbox and active OpenDRIVE file are byte-verified, but exact full-body intended-lane containment remains unsupported and fails closed. The nine original motion bounds are unchanged; no violating anchors or ticks are dropped. ACK and RGB outcomes are in `summary.json`; per-anchor futures, native progress and JPEG checks are in the companion JSONL files. This is not an Autoware GUI or learned-driving result.

The cap changed observed expert control: B reached normalized steer 0.45000 versus A's
0.54198, and the measured tire-angle peak changed from 0.54605 to 0.46197 rad.
The corresponding trajectory changed at the turn, reducing the peak discrete XY
curvature from 0.229889 to 0.185850 /m (unchanged limit 0.2 /m). This one
same-site A/B does not establish a general causal or safety guarantee.

The [B six-camera contact sheet](raw_six_camera_first_middle_last.png) contains
uncropped native 640×360 pixels from the fixed first/middle/last selected anchors
(frames 2619, 2977, 3337). The [B top-down raw-coordinate plot](topdown_raw_turn_peak.png)
centers the ego at the reported peak-curvature anchor, overlays original route
and measured XY, and marks the peak tick. Its [recipe](visual_recipe.py) and
[hash manifest](visual_manifest.json) pin the inputs and outputs. These are
offline BasicAgent expert diagnostics, not CARLA/Autoware GUI captures.

Focused B audit tests: **4 passed**. No dataset was admitted or training run.
