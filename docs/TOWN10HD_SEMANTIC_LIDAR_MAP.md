# Town10HD_Opt semantic-LiDAR localization map

`Town10HD_Opt` is the packaged Town whose inspected CARLA 0.9.15 assets do not
include an HD-map PCD.  `generate_carla_semantic_lidar_map.py` fills that gap
with CARLA 0.9.15 semantic-LiDAR physics ray casts against real collision
geometry.  These are not rendered pixels, and it does not turn
Lanelet2/OpenDRIVE lines into synthetic points.

This bundle and every result below apply only to the optimized
`Town10HD_Opt` profile.  Standard `Town10HD` is a distinct profile and remains
unverified; the optimized map bundle is not evidence for it.

## Evidence contract

- The packaged server must be cold-started directly on `Town10HD_Opt`.  The
  collector verifies the map name and SHA-256 of the OpenDRIVE returned by the
  server and never invokes `client.load_world`.  Collection is loopback-only;
  the TCP-listener PID, `/proc` executable, CARLA 0.9.15 client/server version,
  Python API archive, and server executable are pinned and audited.
- A synchronous 360-degree semantic LiDAR scans deterministic XYZ tiles from
  `carla.Map.generate_waypoints(10.0)`.  Every `(road, section, lane)` key has at
  least one pose, and vertically stacked roads are separated by 4 m Z bands.
- Buildings, road surfaces/lines, sidewalks, walls, poles, signs, terrain, and
  other explicitly static semantic classes are retained.  Pedestrian, vehicle,
  and dynamic labels are rejected.  A cold world may expose only its spectator
  and traffic infrastructure actors; sensors, `static.prop`, vehicles, walkers,
  and every other runtime actor are rejected.  The exact actor set is checked
  before and after every scan and its type multiset is pinned across resumes.
- Sensor-local hits are transformed with the measurement's own CARLA world
  transform and then reflected as `(x, y, z) -> (x, -y, z)` for the ROS Local
  map frame.  After every sensor teleport, a measurement that still reports the
  previous pose is audited and discarded; at most two extra synchronous frames
  are allowed to settle.  The 5 cm / 0.1 degree acceptance thresholds are not
  relaxed.
- Every scan is independently voxelized and saved with a SHA-256 checkpoint.
  Final global voxel merging is disk-partitioned, so resume does not require all
  raw points in RAM.  The output is a PCL `binary_compressed` XYZ PCD.
- Completion requires global Lanelet-node QA and straight plus turn route QA.
  It also requires anti-sparse semantic coverage: multiple retained structural
  classes, vertical extent, dense per-scan chunks, and structural hits across
  the map.  A generated PCD that fails is preserved for audit.  Passing makes a
  structural candidate only; actual Autoware localization/drive evidence is the
  usability admission.

## Recorded collection and reproduction commands

`data/generated/town10hd_opt_semantic_lidar_v1` is retained as the audited
39-scan fail-closed attempt that exposed a one-frame stale measurement at scan
39.  Because the sequencing implementation is part of the immutable plan
fingerprint, do not edit or resume that directory.  The corrected `v2`
collection completed and is now the immutable audited source.  The commands
below record how it was produced; use a different output root for an independent
reproduction rather than overwriting or resuming the audited directory.

The collection used a dedicated packaged server in terminal A.  The wrapper
remained in the foreground so it owned and cleaned up only the CARLA process it
started.

```bash
cd /home/a/autoware_e2e
scripts/e2e/run_carla_map.sh Town10HD_Opt \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
```

In terminal B, the plan was generated without changing CARLA world settings:

```bash
cd /home/a/autoware_e2e
source scripts/e2e/env.sh
python3 scripts/e2e/generate_carla_semantic_lidar_map.py \
  --host 127.0.0.1 --port 2100 \
  --output-root data/generated/town10hd_opt_semantic_lidar_v2 \
  --lanelet-map "/home/a/carla-autoware-universe/op_carla/op_agent/autoware-contents/maps/Town10HD/lanelet2_map.osm" \
  --route-root artifacts/validation/2026-08-31/maps/town10hd_opt/catalog/routes/town10hd_opt \
  --carla-content-root "$CARLA_ROOT/CarlaUE4/Content" \
  --cooked-asset-token Town10 \
  --plan-only
```

After review, that exact plan was resumed for collection and QA:

```bash
python3 scripts/e2e/generate_carla_semantic_lidar_map.py \
  --host 127.0.0.1 --port 2100 \
  --output-root data/generated/town10hd_opt_semantic_lidar_v2 \
  --lanelet-map "/home/a/carla-autoware-universe/op_carla/op_agent/autoware-contents/maps/Town10HD/lanelet2_map.osm" \
  --route-root artifacts/validation/2026-08-31/maps/town10hd_opt/catalog/routes/town10hd_opt \
  --carla-content-root "$CARLA_ROOT/CarlaUE4/Content" \
  --cooked-asset-token Town10 \
  --resume
```

The pinned Town10HD_Opt OpenDRIVE produces 680 sampled waypoints and 275 scan
poses with the defaults.  All 168 road/section/lane keys are covered; the
furthest sampled waypoint is 10.595 m from a selected scan pose.  The completed
checkpoint records all 275 scans.  `provenance.json` records
`COMPLETE_QA_PASS` with global QA `PASS`, including the Lanelet2 map and all
lane-follow, straight, left, and right route-to-PCD checks.  Global Lanelet-node
alignment has 0.153 m XY p95 and 100% within 1 m.

The installed PCL `binary_compressed` ROS-frame PCD contains 6,818,935 points,
is 66,015,232 bytes, and has SHA-256
`1f02775a8d18a5a7d566f80a2db0108806201b5d1739daee7c9bab9f6aa67641`.
The audited collection session took 26.765 seconds of wall time.  The plan's
27.5-million-return and 41.25-simulation-second figures remain upper bounds,
not measured output counts or durations.

The auditable outputs are:

- `scan_plan.json`: exact poses, map pins, sensor settings, coordinate and class
  contracts, estimates, and a stable plan fingerprint;
- `checkpoint.json` and `chunks/`: resumable per-scan hashes/counts;
- `pointcloud_map.pcd`: final ROS-frame voxel map;
- `provenance.json`: runtime settings restoration, actor audits, output hash and
  bounds, global Lanelet metrics, and per-route straight/left/right metrics.

`COMPLETE_QA_PASS` certifies map construction and alignment only.  It does not
by itself claim that Autoware VAD completed a closed-loop drive.

## Installed result and readiness refresh

After `provenance.json` reported `COMPLETE_QA_PASS`, the collection was installed
as the pinned Town10HD_Opt full-map candidate.  The installer re-verified the
PCD, checkpoint, static/dynamic actor audits, exact generator and
route-validator snapshots, Lanelet and OpenDRIVE pins, and the Town-named
cooked-asset inventory.  The recorded installation command was:

```bash
cd /home/a/autoware_e2e
source scripts/e2e/env.sh
python3 scripts/e2e/install_generated_semantic_lidar_bundle.py \
  --collection-root data/generated/town10hd_opt_semantic_lidar_v2 \
  --lanelet-map "/home/a/carla-autoware-universe/op_carla/op_agent/autoware-contents/maps/Town10HD/lanelet2_map.osm"
```

The readiness JSON and Markdown can then be rebuilt for every packaged Town,
including exact straight and turn route-to-Lanelet/PCD preflight:

```bash
python3 scripts/e2e/prepare_packaged_town_full_maps.py \
  --deep-alignment \
  --route-root artifacts/validation/2026-08-31/maps \
  --json-output artifacts/validation/2026-08-31/town_full_map_readiness.json \
  --markdown-output artifacts/validation/2026-08-31/TOWN_FULL_MAP_READINESS.md
```

## Frozen matrix and post-matrix supplemental runs

The terminal `autoware_vad_town_matrix` artifact was frozen before this PCD
existed.  Its `town10hd_opt` straight and turn rows therefore truthfully remain
`BLOCKED`, and they must not be rewritten or described as matrix validation.
The current structural-readiness PASS and the supplemental runs below do not
retroactively change that historical matrix result.  Standard `Town10HD` also
remains distinct and unverified.

After bundle installation, two same-profile `Town10HD_Opt` trials completed
outside the frozen matrix root.  Both are `full_stack` executions using
`vad_route_manager_hybrid` with `hybrid_route_assisted` trajectory geometry,
and both reached the goal.  They are post-matrix supplemental evidence only;
no matrix PASS is claimed.

| Supplemental route | Result | Captured evidence |
|---|---|---|
| [`town10hd_opt_straight_s0000_p00`](../artifacts/validation/2026-08-31/maps/town10hd_opt/catalog/routes/town10hd_opt/straight/town10hd_opt_straight_s0000_p00.json) | [PASS: goal reached](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/straight/attempt_001/result.json) | [PNG](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/straight/attempt_001/autoware_rviz_fullscreen.png), [GIF](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/straight/attempt_001/autoware_rviz_drive.gif), [bag](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/straight/attempt_001/bag/metadata.yaml), [analysis](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/straight/attempt_001/route_result.png) |
| [`town10hd_opt_left_s0000_p00`](../artifacts/validation/2026-08-31/maps/town10hd_opt/catalog/routes/town10hd_opt/left/town10hd_opt_left_s0000_p00.json) | [PASS: goal reached](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/turn/attempt_001/result.json) | [PNG](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/turn/attempt_001/autoware_rviz_fullscreen.png), [GIF](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/turn/attempt_001/autoware_rviz_drive.gif), [bag](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/turn/attempt_001/bag/metadata.yaml), [analysis](../artifacts/validation/2026-08-31/supplemental_vad/maps/town10hd_opt/trials/turn/attempt_001/route_result.png) |
