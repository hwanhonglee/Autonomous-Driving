# CTrack isolated source-map probe

<!-- HH_261001 - Separate source-label recovery, image quality, and lifecycle evidence. -->

The staging source map produced native road and lane pixels in a parent-owned stationary probe: **Road=1: 87,776 pixels; RoadLine=24: 2,965 pixels**. Its live map was `map_package/Maps/C_track_1_0_7/C_track_1_0_7`, and live OpenDRIVE SHA256 matched `3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27`. This is a source-editor result with a freestanding camera near the reviewed left-route start; it is not a repair of the installed shipping package or a claim of identical actor placement.

The [v1 raw RGB](../../../../../storage/experiments/validation/2026-10-01/ctrack_isolated_stationary_20261001/capture/rgb.png) fails image quality. It contains 696 yellow pixels in the top 32 rows and a visible Unreal skydome warning. The fixed center-sky ROI is 91.3% near-white. `dataset_admitted=false` and `train_eligible=false` remain in force.

The original [owner result](../../../../../storage/experiments/validation/2026-10-01/ctrack_isolated_stationary_20261001/owner_result.json) remains `FAIL`: the initial cleanup check ran before the namespace reaper and ports finished disappearing. A separate [late postmortem](v1_postmortem.json) confirms both original process generations absent, no live original process-group members, all three ports released, and source fingerprints unchanged. It sends no signals and does not rewrite that failure.

## Isolation and dependencies

Bubblewrap enforces a read-only root, including shared CARLA/Unreal content and binaries. Only the private run directory and explicit private configuration, Saved, Intermediate, temporary, user cache/config/data, and engine Saved/Intermediate binds are writable. GPU devices remain accessible through `/dev`; `/dev/shm` is private. The existing 18 GB engine DDC is read-only, with a private write layer. No full project copy, hardlinked config, build, install, or shared-source edit was required. Initial staging occupied 1.1 MB before runtime caches.

The [sandbox receipt](../../../../../storage/experiments/validation/2026-10-01/ctrack_isolated_source_20261001/sandbox_smoke.json) records 86 shared files refusing write-open with `EROFS`, 11 private writable directories, a distinct private config inode, and 96 unchanged source fingerprints. Engine/project/plugin module manifests share BuildId `d4742064-e4f5-473b-ab50-cb434393f5c3`.

The [serialized direct-import audit](direct_import_audit.json) decodes UE4 package version 522 and checks exact table boundaries. All **3,028 imported Game/Engine content-package records resolve to existing files**. Exact StaticMesh imports include 10 Road, 10 RoadLine, and 30 SideWalk meshes; SideWalk includes curb and gutter meshes. The suspicious printable `C_track_1_0` path is not an imported package. This checks direct package imports, not transitive load success; the complete shared content trees stay mounted. Runtime logs additionally report missing walker-animation references, which are outside the map's direct import check.

## RGB diagnosis and controlled v2

The v1 RGB sensor inherited `enable_postprocess_effects=false` from the earlier collector. Local `SceneCaptureSensor.cpp:552` then forces linear gamma; `ConfigureShowFlags` at line 705 disables lighting, directional/direct lights, sky lighting, eye adaptation, and tone mapping. This provides a concrete explanation for poor RGB lighting and clipping.

The yellow text has a separate precise source. `SkyAtmosphere.usf:1645` implements the editor HUD; `BasePassRendering.cpp:906` draws it before scene geometry. Text left visible indicates missing sky-dome coverage in those pixels. `SkyAtmosphereRendering.cpp:1814` also skips normal sky pixels when a sky material is present. Changing exposure alone cannot be assumed to repair geometry coverage, and hiding the notification would not demonstrate a repair.

V2 changed only the RGB sensor's postprocess flag to `true`; semantic rendering remained `false`. It preserved the same map, route-relative camera, 640×360/FOV90/pitch−12, ClearNoon weather, and raw-label handling. Its [actual owner result](../../../../../storage/experiments/validation/2026-10-01/ctrack_isolated_stationary_20261001_v2/owner_result.json) is `PASS`, including source preservation and drained process/port cleanup. The [raw RGB](../../../../../storage/experiments/validation/2026-10-01/ctrack_isolated_stationary_20261001_v2/capture/rgb.png) has a rendered sky, trees, and road lighting; warning pixels fell from 696 to 7, and the center-sky near-white fraction fell from 91.3% to 0%. Native road/lane counts stayed exactly 87,776/2,965. This controlled observation supports the sensor render-mode explanation for this fixed viewpoint. It does not establish render quality along a drive or for Town13.

V2 also added a cleanup helper that first signals the exact owned Unreal child, then waits for both group termination and two consecutive free-port observations. V1 scripts and failure receipts remain preserved.

```bash
python3 -B tools/e2e/run_owned_ctrack_isolated_probe_20261001_v2.py
# The runtime owner runs --execute only when the workspace runtime lease is free.
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_ctrack_isolated_probe_20261001.py \
  learning/tests/test_ctrack_imports_20261001.py \
  learning/tests/test_ctrack_cleanup_v2_20261001.py
```

The three test files passed 12 tests before v2 launch. No capture is automatically approved for training or independent evaluation.

## Prepared moving external diagnostics

The versioned moving owner and worker are prepared for the original reviewed left and straight routes, each in a separate fresh owned session. Both CPU preflights passed; the combined isolation, import, cleanup, and moving-contract tests passed 17 tests. The parent coordinates simulator launches; preparation itself did not launch a moving run.

```bash
python3 -B tools/e2e/run_owned_ctrack_moving_20261001.py --scenario left --execute
python3 -B tools/e2e/run_owned_ctrack_moving_20261001.py --scenario straight --execute
```

Run these sequentially under the existing owner lease. Default outputs are `storage/experiments/validation/2026-10-01/ctrack_{left,straight}_external_diagnostic_20261001`. Each records 240 synchronized RGB/depth/semantic sets at 10 Hz, using the reviewed BasicAgent/Prius route and 15 km/h target. RGB postprocessing is enabled; depth/semantic remain raw. PNGs are accompanied by all three raw BGRA arrays, camera matrices, per-sensor frame IDs/timestamps/poses/hashes, ego poses and controls, intrinsics, native-label counts, and RGB warning counts. The source map, original packaged map/XODR/binary, Python API, and navigation/mapping code are bound in provenance.

The new offline checker is `tools/e2e/audit_ctrack_isolated_moving_20261001.py --source <run>/capture --scenario left|straight --output <fresh-report.json>`. It keeps the prior road/lane/depth/route-window thresholds and adds raw RGB/PNG agreement and independent per-sensor identity checks. Completion alone does not grant evaluation certification.

These new captures have `intended_role=external_map_diagnostic_evaluation`, `train_eligible=false`, `dataset_admitted=false`, and `independent_dev_approved=false`. They stay out of map training while the fixed models are compared. Historical human exposure to CTrack and the source/rendering change are disclosed; this is not a newly certified independent site. The historical split registry is unchanged, and its capture-only permission is recorded separately from the current diagnostic use.
