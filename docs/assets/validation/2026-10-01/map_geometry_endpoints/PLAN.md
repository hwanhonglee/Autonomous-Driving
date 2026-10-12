# HH_261001 — Fixed geometry-source endpoint diagnostics

This prospective evaluation is frozen before new final endpoints are imported locally. It does not authorize fitting, promotion, additional scenario selection or a new TEST read. Parent executes it only after both fixed 10,000-step `control` and `geometry` endpoints have been saved. Both are fresh from original context-B; the **sole paired comparison** is geometry minus this new study's control. Historical photometric predictions are a separately labeled reference with a different exposure schedule, never the matched control.

Study `map_route_geometry_matched_20261001_v1`; prepared remote manifest `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d`; training source bundle `20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c`; separate parent fit authorization `8e4e586d8d80eb0bb3c645ad01975954ba34dfb3f29cdaac07fc5d610da1fa38`. The local import is `storage/experiments/training/2026-10-01/map_route_geometry_matched_20261001_v1/fit`. Only small source/manifest receipts and final fit artifacts are imported; new geometry TRAIN sensor arrays remain remote. Checkpoint pickle bytes are hashed, never loaded. Exact safe-NPZ names, float32/finite/shapes, individual tensor digests and aggregate trained-state digest must agree before inference.

## Immutable observations and metrics

Use the existing four scenarios in order: CTrack LEFT, CTrack STRAIGHT, paired Town01 p01 TRAIN, paired Town02 p00 TRAIN. Every scenario retains all240 original native640×360 frames. CTrack uses TRUE RGB; the paired TRAIN scenarios use FALSE and TRUE RGB against their unchanged common semantic targets. No crop, resize, augmentation, deduplication, confidence cutoff, frame filtering or learned depth/pose input. Raw-source QA/owner distinctions, LEFT's original failure and unavailable live equality remain unchanged.

The 2,880 new model/condition/frame predictions cover 663,552,000 pixels. Store lossless uint8 all240 masks, every per-frame confusion, aggregate7×7 confusion, class support, IoU and false positives. An absent-GT class has null IoU, not a favorable invented score. Compare every new mask with the historical photometric endpoint's mask, and retain all per-class deltas independently of the matched result. Native RGB/GT/control/geometry panels use exactly0/119/239: twelve canvas PNGs, 72 unscaled640×360 subpanels. No favorable-image selection.

CPU only, two Torch and two native BLAS threads, batch1. Record cold first forward+argmax and p50/p95; do not call this unloaded performance or end-to-end10Hz. Maximum600s per scenario; every failure remains a failed denominator. No online optimizer, new thresholds or auto-selection.

The existing trainer's known DEV118 and Town02 holdout232 receipt is pinned and preserved **separately**. This evaluator opens neither split's arrays and does not rerun those350 forwards. The independently implemented final-fit/known350 verifier owns its own recount. CTrack remains exposed EVAL only; paired TRAIN is resubstitution, not generalization. None of these known/exposed observations is independent certification.

## Resource bound

The aggregate fresh output root is `storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1`, with one subdirectory per scenario. All new output bytes must remain **strictly below30MiB**; every NPZ/PNG is compressed in memory before a guarded exclusive write. Reserve2MiB for a failure receipt and keep at least5GiB free on disk. Prior full photometric scenario directories total24,959,139bytes, an estimate only—not a promise about new prediction compression.

If the actual bytes cannot fit, abort and retain partial artifacts plus the complete per-frame counts/timing of finished forwards and hashes of their computed masks in the failure receipt. Explicitly disclose when full prediction masks could not be persisted. Do not drop frames/panels, weaken a guard, relabel partial output as complete or delete historical data. A remote-storage stage requires separate parent review and declaration; it is not an automatic fallback. No new prediction outputs, forward or remote connection are made by `--preregister` or the default validation command.

## Execution after parent import

Registration (before local endpoint import):

```sh
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.evaluate_map_geometry_endpoints_20261001 -- --preregister
```

Parent supplies the actual immutable final `summary.json` SHA, not a metric-selected endpoint. For **each** scenario in the fixed order above (CLI names `ctrack_left`, `ctrack_straight`, `train_town01_p01`, `train_town02_p00`):

```sh
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.evaluate_map_geometry_endpoints_20261001 -- --execute --scenario SCENARIO --expected-fit-summary-sha256 FINAL_SUMMARY_SHA256
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.verify_map_geometry_endpoints_20261001 -- --scenario SCENARIO --expected-fit-summary-sha256 FINAL_SUMMARY_SHA256
```

The independent verifier uses a separately written target mapping/count routine and literal panel coordinates. It recounts every predicted pixel, timing aggregate, paired/historical delta and native panel, with no repeated inference. Source/data/model hashes must still agree afterward. Report all four outcomes and known350 separately; a successful recount is evidence integrity, not model quality approval.
