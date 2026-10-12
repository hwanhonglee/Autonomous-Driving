# Full-recording oracle-assisted semantic accumulation — review draft

<!-- HH_261001 - Declare every frame, unchanged recipe, resource bound and evidence boundary before new inference. -->

Prepared only. **Do not register or execute before parent review.** This is a new prospective study of already-completed image endpoints, not an unseen endpoint preregistration. Prior image scores and fixed panels are known. Neither confidence recovery nor full-recording projection has run under this plan. No fit, capture, GPU, SSH, installation, protected-data admission or model promotion is authorized here.

## Exact inputs and comparison

Both CTrack routes remain exposed external diagnostic EVAL: LEFT240 and STRAIGHT240, chronological indices0..239, every recorded frame. LEFT retains its original owner FAIL and unavailable original live map replay; the separate forensic raw QA supports this new offline computation only. STRAIGHT retains its separate owner PASS/raw QA. No current or future record is relabeled TRAIN.

Use **both current geometry-study endpoints**, control and geometry, from local `map_route_geometry_matched_20261001_v1/fit`. Summary SHA `caa1a60bcff8179b80de0a5861aa2ab0497d838638fbf80a3f4790e8074e11ef`; control weights SHA `7bd474649e0698497a5eef94430993626dc0e77c29bca4de759a5a9b46248416`; geometry weights SHA `70b5077c0e1c75f44b731692646e16ebcd784d8a3dc9a6c989d4934ffff1cd83`. The existing endpoint preregistration SHA is `b91586fbbf905b085b861a73db127d4b5c22dbb905d20170d78790a9646b857a`. Both completed native-mask summaries and independent all-pixel audits must be PASS and bound before registration. Historical photometric weights are not this matched control.

Registration records the exact fit/source closure, original QA roles/receipts, all240 raw artifact hashes per route, records/summary hashes and four saved240-mask artifact hashes. Revalidate before and after computation. RGB remains in place; no copying, resizing, input augmentation or target-driven pixel mask is allowed.

## Unchanged recipe

Use the existing `mapping_projection.py` (`c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee`) and `mapping.py` (`264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867`) without edits. The only extension over the previous three-frame plumbing is the fixed chronological240-frame denominator and serialization count ceiling240.

- Native RGB640×360, horizontal FOV90°, unchanged lens/intrinsic/pose contract. Recover maximum float32 softmax confidence using two CPU threads and two native BLAS threads, no CUDA. **960 learned forwards**, two endpoints×two routes×240; compare both full-frame logit and probability argmax bit-exactly to each saved uint8 mask. Any mismatch fails the study; no tolerance, frame omission or alternative backend search.
- Exact stride3 confidence samples `[120,214] float32`, not quantized; unsampled pixels are never queried. Each forward's full-confidence projection must exactly equal its reconstructed sampled-confidence projection before accumulation. Raw confidence payload is98,611,200bytes before lossless NPZ compression.
- Oracle CARLA depth and camera world pose provide geometry. Depth strict(.1,40)m, stride3, confidence≥.5, static classes road/sidewalk/lane-marking/fixture/sign/pole; other excluded. No ground-height filter, GT-derived masking, topology completion or unseen-region interpolation. Oracle semantics are a separate reference map only and never enter learned projection.
- Unchanged `.2m` cells, maximum200,000 cells per map. `SemanticEvidenceMap.update()` keeps the maximum per-frame confidence for each cell/class, then accumulates frame counts/confidence sums. Both arms and oracle receive every timestamp in order. A capacity failure aborts and preserves evidence; no eviction, segmentation, stride change or enlarged cap.
- “Online” means chronological updates over the existing recording, **not a new live CARLA run**. Compare exact per-frame accumulated snapshots and final serialized grids to a separate no-forward replay from saved mask/confidence/depth/pose. This cannot repair LEFT's historical live-equality failure.

## Outputs and independent replay ABI

New output only: `docs/assets/validation/2026-10-01/map_geometry_full_projection/run_v1/`, fresh directory, no resume/overwrite.

- `started.json`: policy, preregistration SHA, complete source/input/model bindings and UTC creation.
- `<scenario>_<index:03d>_<arm>_confidence.npz`: only `confidence_stride3`, exact float32[120,214]. 960 files. Numeric digest is SHA256 of C-order float32 bytes.
- `<scenario>_<index:03d>_frame.json`: all480 frame receipts. Exact index/sensor-frame/timestamp; oracle/control/geometry projected point counts and canonical point hashes; learned confidence file/numeric hash and exact-argmax assertion; all three accumulated cell counts and canonical snapshot hashes. Fixed0/119/239 additionally bind PNG and diagnostic confusion. Runtime timing is descriptive, not an equality criterion.
- `<scenario>_<oracle|control|geometry>_grid.npz`: six complete grids, sorted int64 cell coordinates, int8 label insertion order, int16 observed-frame counts≤240, float64 confidence sums. No pickle. Exact insertion-order preservation protects tied dominant-label behavior. Summary binds canonical snapshot SHA, cell count and timestamp per grid.
- Six fixed ego-centered three-column PNGs at0/119/239: oracle semantics, control segmentation, geometry segmentation, all using oracle geometry. They show all preceding frames but explicitly disclose that the fixed viewport clips older/off-screen cells.
- Two final full-route three-column PNGs: common bounds/scale include every stored cell footprint and all240 ego positions, plus initial/final pose and final yaw. Shared world-x/y bounds are derived once from the union across all three maps and the complete path, with1m padding. Raster overlap at overview scale is unavoidable and disclosed; complete numerical grids remain available. No selected-frame coverage claim.
- `summary.json`: every artifact SHA, all480 frame receipts, six final-grid bindings, eight PNGs, final cell-level diagnostics, native thread settings, source/input unchanged evidence and elapsed time. It marks online/offline equality pending until `independent_replay.json` is PASS. Diagnostic IoU is null where oracle class support is absent; learned false positives remain counted. Unknown is not free space. Image pixels, BEV cells and known350 metrics retain separate denominators.
- The separate verifier checks all artifacts, all1,440 point streams and incremental map snapshots, six final numeric grids including order/ties, eight PNG pixel arrays, source/role boundaries and immutable hashes. It calls no model/inference function. Its own receipt supplies exact replay PASS; original summary remains immutable. Verification failure is a separate retained receipt.

## Prospective resource and failure bounds

This new study allows **128MiB aggregate output**, including partial attempts, summary and replay receipt, with2MiB reserved for failure/verification evidence. This supersedes nothing in prior10MiB three-frame studies or30MiB image evaluations. At start require128MiB above the unchanged **5GiB free-disk floor**; every write checks both bounds before opening. Existing RGB, previous results and frozen documents are never deleted to make room.

Runner and independent replay each have a1,800-second wall budget. Parent may apply a larger external process watchdog only for termination/cleanup; it must not silently extend this computation limit. On timeout, capacity, source, argmax, ABI or resource failure, retain all already-written artifacts plus `failure.json`/`verification_failure.json` where the same disk floor permits. If even a small receipt would cross the floor, print it to the owner log without weakening the guard. No partial PASS, hidden filtering, retry-overwrite or automatic larger budget.

Attempted forwards increment before the model call; completed forwards increment immediately when it returns, before finite/argmax checks or confidence writes. Failure receipts separately count successful oracle/learned map updates and fully persisted frame receipts. Current scenario/index/arm/stage, confidence-persistence state and available prediction/confidence shape/dtype/numeric hashes remain visible, including an argmax mismatch or a write failure after inference.

## Review and later execution

First review new runner, separate verifier, both test files and this plan. Then parent may register once; registration dynamically hashes these files, the font and inherited immutable endpoint/model/projection dependencies, avoiding circular embedded self-hashes. No source edits after registration. Default mode validates an existing registration only; it never creates outputs or invokes inference.

After explicit review, the intended commands are:

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/project_map_geometry_full_20261001.py --preregister
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/project_map_geometry_full_20261001.py --execute
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/verify_map_geometry_full_projection_20261001.py
```

The deliverable is **oracle-assisted full-recording semantic evidence accumulation**, not camera-only mapping, learned depth/localization, ground-plane truth, lane topology, a completed HD map, real-time10Hz, independent certification or any driving-feature qualification. Coarse CARLA mesh tagging and current semantic errors may accumulate into false structures; report them rather than adjust the recipe.
