# Frozen matched-endpoint evaluation handoff

<!-- HH_261001 - Operational commands; immutable scientific policy lives in PLAN/preregistration. -->

Ready before endpoint import; no predictions made. Registration SHA256:
`dbd037ee055103e53b40d43e8aec57247386929bf990beac20cca6f414438616`.
Six local safeguard/render/recount tests pass. The original matched training
package and all historical capture/evaluation records remain unchanged.

Parent imports the complete finished remote run into the fresh directory:
`storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1/fit/`.
Required contents include `source/`, `inputs/`, `control/`, `appearance/`,
`started.json`, `fit_complete.json`, and final `summary.json`. The exact final
summary SHA must be supplied explicitly; both endpoints and all source/input
bindings are checked before safe data-only NPZ loading. Checkpoint files are
hashed for provenance but never unpickled by this evaluator.

Once authorized, run each scenario in its own process, sequentially to keep
the evaluator within two CPU compute threads. Set `FIT_SUMMARY_SHA` to the
parent-reviewed imported final summary hash, not a partial fit receipt.

```bash
python3 -B tools/e2e/evaluate_map_appearance_endpoints_20261001.py \
  --scenario ctrack_left --expected-fit-summary-sha256 "$FIT_SUMMARY_SHA" --execute
python3 -B tools/e2e/verify_map_appearance_endpoints_20261001.py \
  --scenario ctrack_left --expected-fit-summary-sha256 "$FIT_SUMMARY_SHA"
```

Repeat these two commands for `ctrack_straight`, `train_town01_p01`, and
`train_town02_p00`. No simulator or GPU is used. Without `--execute`, the
evaluator only validates the frozen preregistration and does not infer.

Fresh outputs are
`storage/experiments/validation/2026-10-01/map_appearance_endpoints_<scenario>_20261001_v1/`.
Each has all prediction arrays, per-frame and aggregate confusion/support/IoU,
fixed full-FOV PNGs, raw-QA replay, endpoint hashes, and an independently generated
`prediction_audit.json`. Total planned recount is 663,552,000 predicted pixels
and 72 native full-FOV panels across all four scenarios. No automatic promotion.

CTrack results are exposed external diagnostics, with LEFT's forensic-only
qualification and original owner FAIL retained. Paired Town01/Town02 scores
are exact TRAIN-frame resubstitution diagnostics, not held-out validation.

## Completed results

All four scenarios finished and passed independent recount: 663,552,000
prediction pixels and72 native panels. The original handoff and preregistration
above are preserved. See [Korean result summary](RESULTS_KO.md),
[all-class results and all12 fixed PNGs](RESULTS.md),
[fixed-panel review](FIXED_PANEL_REVIEW.md), and
[proposed next pure-photometric A/B](NEXT_PHOTOMETRIC_PROPOSAL.md).
No model promotion or HD-map completion is claimed.
