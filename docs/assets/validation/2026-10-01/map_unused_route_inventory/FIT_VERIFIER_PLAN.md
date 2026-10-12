# Independent final-fit and known350 recount protocol

<!-- HH_261001 - Freeze a read-only verifier before final fit completion/import, without inspecting active candidates. -->

Prepared while the parent-owned geometry fit is running; **no active checkpoint, candidate prediction or training result was opened**. This verifier is separate from the four-scenario external evaluator. It imports no trainer helper, PyTorch, GPU code or pickle and performs no forward, fit, model selection or promotion.

Local completed import is fixed to `storage/experiments/training/2026-10-01/map_route_geometry_matched_20261001_v1/fit`. Parent must first confirm remote completion/import and supply the exact final summary SHA. Missing/failed/incomplete summaries are rejected before endpoint reads. A top-level or per-arm failure/aborted-state artifact cannot become PASS.

## Fixed independent checks

1. Chain `started.json → arm/training.json → fit_complete.json → summary.json`, exact control/geometry arms, both complete10k endpoints before DEV marker, parent/source/data/plan/package+fit approval, fixed class weights, GPU0 UUID, recorded idle preflight, and exact nine-file archived source closure. Historical candidate `train_eligible=false` remains intact; separate study authorizations carry the narrow role.
2. Exactly100 history rows/arm at steps100..10000, finite nonnegative mean losses, increasing wall times, and bounded completed training. This verifies recorded execution evidence; it does not claim independent hardware attestation of optimizer calls.
3. Safe nativefloat32 NPZ keys/shapes match the original parent, every value finite, all per-tensor byte hashes and the canonical numeric state digest agree. Parent SHA `ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e` and initial state `edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67` are fixed. Checkpoint `.pt` files are hashed **as bytes only**, never deserialized.
4. Independently reconstruct PCG64 sampling, exact old-Town01 replay mapping, crop sequence, bounded TRUE-only photo RNG/gate trace, all per-slot and physical-source exposure counts. Every original729 and TRUE480 example is retained in both arms; every new480 appears in geometry. Control adds22,750 old-example exposures; geometry uses22,750 new-source exposures. Common paired exposures and original Town02 exposures remain identical. No reweighting, relabeling or label-based sample choice.
5. Recount all saved known350 masks using only existing local target arrays: Town01 DEV118 and Town02p02 holdout232, two endpoints = **700 prediction frames /161,280,000 pixels**. No new400MiB training-array import and no decode/copy of RGB evaluation arrays. Require exact native640×360 uint8 ABI, seven-class bounds, complete counts, full7×7 confusion, target/predicted/FP support, all IoUs and accuracy. Absent-target-class IoU is null with false positives retained. No crop, confidence threshold, masked region or frame omission.
6. Recheck every read artifact hash after recount. Persist one fresh JSON under128KiB, containing receipt/hash-chain findings, history/tensor/exposure checks, full known-split confusion and metrics, source/prediction hashes, verifier SHA and explicit zero-forward/no-pickle/no-GPU flags.

Training-array assurance is the pinned remote package/preflight and frozen run's source/data hash-check receipts; the verifier **does not pretend to independently reopen remote-only geometry arrays**. The original trainer does not put prediction-file SHA into its score receipt: this verifier pins each imported saved prediction file for the first time and requires its exact all-pixel confusion/metrics to match the recorded result. No new spatial prediction was generated. The small parent NPZ and local known target archives already existed.

Expected final sourcebundle `20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c`, prepared manifest `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d`, fit approval `8e4e586d8d80eb0bb3c645ad01975954ba34dfb3f29cdaac07fc5d610da1fa38`. No candidate-dependent threshold or change to the frozen trainer is permitted.

## Execution only after parent confirms completion

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' \
  python3 -B tools/workspace_python.py tools/e2e/verify_map_route_geometry_fit_20261001.py \
  --expected-summary-sha256 PARENT_CONFIRMED_FINAL_SUMMARY_SHA256 \
  --output docs/assets/validation/2026-10-01/map_unused_route_inventory/known_split_independent_recount.json
```

Output schema: `portable_e2e.map_geometry_fit_independent_recount.v1`; success status `PASS_DIAGNOSTIC_NOT_PROMOTED`; `total_recounted_pixels=161280000`, `unique_known_frames=350`, `prediction_frames_both_arms=700`. `known_checks` has each split/arm's complete support/confusion/metrics; it is **not merged with** the separate external four-scenario denominator.

22 CPU unit tests cover immutable source pins, independently reconstructed schedule/photo/retention, valid/rejected histories, actual parent tensor state without Torch, wrong tensor shape/dtype/keys/nonfinite values, native recount/absent-class false positives, invalid masks, failure/incomplete guards and nonfinite JSON. These tests open only prior parent/frozen plan artifacts and tiny synthetic fixtures, never the active fit.
