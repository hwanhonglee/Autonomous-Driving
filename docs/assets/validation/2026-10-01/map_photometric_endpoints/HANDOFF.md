# Frozen photometric endpoint evaluation handoff

<!-- HH_261001 - New evaluation only; old study sources and evidence remain immutable. -->

Registered before local final import/predictions:
`7019e192782a05eb4cfc2066a43c01ef02c1e8c32a5573341355a090e91e2711`.

Frozen sources:

- Evaluator: `9ef2dd610715ffacccc1385c94479f197bbb4feaf71619c8771f5ef7297eb5d8`
- Independent verifier: `836cc98e36ec201029cccbd334423c9b3d1c803df3b70b58b8ff6e4c24ed4885`
- Sixteen new tests: `90a91b5b8fea4c6470bcd6e589d28a871c6a17cdd7eb90dd510b3c22da50ac4e`
- PLAN: `3ddfcbacf9322e3ad97e8c0a6c3e7cbb66999c7e869637362791b9f4f6697bc0`

New tests16/16 and combined new + prior endpoint + CPU-replay tests28/28 pass.
Registration validation passes. No new endpoint inference preceded registration.
Parent owns remote final import into
`storage/experiments/training/2026-10-01/map_photometric_matched_20261001_v1/fit/`.

After the parent supplies the imported `summary.json` SHA256, execute and
independently recount each scenario in this fixed order: `ctrack_left`,
`ctrack_straight`, `train_town01_p01`, `train_town02_p00`. Replace the example
scenario and explicit summary hash; do not use intermediate checkpoints.

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/evaluate_map_photometric_endpoints_20261001.py --execute --scenario ctrack_left --expected-fit-summary-sha256 EXACT_IMPORTED_SUMMARY_SHA256
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py tools/e2e/verify_map_photometric_endpoints_20261001.py --scenario ctrack_left --expected-fit-summary-sha256 EXACT_IMPORTED_SUMMARY_SHA256
```

All outputs are fresh `map_photometric_endpoints_<scenario>_20261001_v1`
directories under `storage/experiments/validation/2026-10-01`.
Finish all four regardless of early scores. New summaries explicitly label
`iou_delta_photometric_minus_control` and `preceding_appearance_comparison`.
The preceding reference is the old appearance endpoint, never its old FALSE
control. Historical CTrack LEFT FAIL and unavailable live-map equality remain.
No CTrack TRAIN, promotion, GPU inference, simulator launch or new fitting.
