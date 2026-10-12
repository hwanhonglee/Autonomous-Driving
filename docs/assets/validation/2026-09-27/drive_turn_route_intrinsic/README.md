# Town01 right p06: nominal route body-versus-lane diagnosis

<!-- HH_260906 - Separate route-centerline footprint feasibility from the observed BasicAgent tracking failure. -->

**Result: `PROVISIONAL_PASS` for the idealized route only; observed B remains `FAIL` and training admission remains `false`.** The route centerline itself is not shown to force a Prius body outside the archived intended lane. This is a static offline geometry result, **not** a successful drive, a learned-model test, or proof that the path is dynamically safe.

| Same archived B evidence, route progress 30–120 m | Nominal route probe | Observed B actor |
| --- | ---: | ---: |
| Body positions / tick-bound bodies | 1,820 | 720 |
| Bounded body sweeps | 1,819 | 719 |
| Bodies outside raw sampled intended lane | 0 | 68 |
| Sweeps outside raw sampled intended lane | 0 | 75 |
| Bodies / sweeps outside 10 cm inset lane | 0 / 0 | 78 / 84 |
| Worst raw sweep area outside lane | 0 m² | 1.10498 m² near road 208, 50.5–50.8 m |

The nominal calculation reads the **archived B** [`route.json`](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode/route.json), actor-local Prius bounding box, and exact live-run [`active_map_opendrive.xodr`](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode/active_map_opendrive.xodr). The [probe](nominal_route_probe.py) first uses the existing source/hash and road-section-lane-ID checks, collapses only identical command-change nodes, interpolates the recorded **rear-axle base_link** path and yaw at intervals no greater than 0.05 m, shifts each pose forward by half of the recorded 2.85 m wheelbase to the CARLA actor center, projects the full 3-D box to ground, and applies the **same archived-map corridor reconstruction, 10 cm inset, and conservative sweep envelope** used by the [observed-body audit](../drive_capture_geometry_provenance/B_lane_gate_30_120.json). It does not launch a CARLA server or alter source episodes.

Even this ideal nominal path has a small local margin: minimum body clearance is **0.165 m** to the sampled raw lane edge and **0.065 m** to the 10 cm inner edge, near 44.46 m. With the conservative swept envelope, those minima are **0.124 m** and **0.024 m**, near 44.41–44.46 m. Thus “nominally contained” is not “robust to tracking error.” The observed B trajectory cuts beyond the lane despite passing its fixed XY-curvature bound; lowering the steering cap solved one motion metric, not body containment.

Reproduce from the workspace root, with the archived B episode present:

```bash
python3 docs/assets/validation/2026-09-27/drive_turn_route_intrinsic/nominal_route_probe.py \
  --episode storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode \
  --start-m 30 --end-m 120 --step-m 0.05
```

The probe returned `PROVISIONAL_PASS`, 0/1,820 raw or inset body violations, and 0/1,819 raw or inset sweep violations. Its source hashes are route `49d0c147…`, XODR `97a7f6ac…`, collector `5dd1654a…`, and B states `a25affab…`. The source reporter still explicitly sets `physical_continuous_time_proven=false` and `training_data_approved=false`: piecewise-linear route/yaw interpolation and sampled CARLA XODR lane tessellation are not an exact real-world lane boundary or a vehicle-dynamics proof. No collision, obstacle, lane-change, new-town, or learned-controller result follows from this probe.

A narrower 0.025 m probe also returned 0/3,621 body and 0/3,620 sweep violations; this sampling check does not enlarge the physical-validity claim. The probe deliberately requires the currently checked-out collector to match the archived collector SHA, so later source changes require the matching revision or a separately reviewed provenance update.

**Next one-factor decision:** retain B's 9 km/h, normalized steering cap 0.45, route, seed, camera/physics settings, and all nine motion plus full-body lane gates. For one preregistered BasicAgent expert C trial, change only `base_min_distance` **3.0 → 1.5 m** ([B/C plan](../drive_waypoint_purge_ab/PLAN.md)). At the observed ~2.46 m/s, the purge distance hypothesis changes from about 4.23 to 2.73 m; it might reduce inside-corner cutting but could increase curvature or steering oscillation. Until the owned C capture and independent audit pass **both** curvature and full-body lane gates, do not admit either B or C as training data. Do not rewrite the map route or claim a causal controller explanation from this static comparison alone.
