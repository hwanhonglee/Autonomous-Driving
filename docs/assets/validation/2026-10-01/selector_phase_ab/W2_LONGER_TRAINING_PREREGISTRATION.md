# W2: fixed four-block continuation of the W1 warmup-supervision protocol

<!-- HH_261001 - Prepare a duration-only research fit; require exact first-block reproduction and never use DEV for endpoint selection. -->

Preparation only: no GPU training launched by this change. The parent schedules the personal remote GPU0 job after reviewing the frozen runner and tests. No shared model, decoder, loss, dataset, collector or previous training source is modified.

## Hypothesis and comparison

The only scientific intervention relative to W1 is longer optimization under the unchanged W1 protocol. W1 used 3,000 scheduled batches, skipped the all-warmup batch at one-based position 1,693, and completed 2,999 actual AdamW updates. W2 repeats that exact 3,000-batch index schedule FOUR times in order: 12,000 scheduled batches, 11,996 actual updates, 48,000 scheduled row positions, 47,984 forwarded rows, and 46,196 positively supervised row exposures. Global skipped positions are 1,693 / 4,693 / 7,693 / 10,693. A skipped batch performs no forward, zero_grad, backward or optimizer step; AdamW weight decay and momentum must not advance on it.

Start from the original AH checkpoint, NOT the W1 checkpoint. W1 did not save optimizer state. Initialize one fresh AdamW and seed exactly once using the original W1 initialization order; retain this same in-memory model and optimizer through all four blocks. Do not reset the RNG or optimizer at block boundaries, reconstruct momentum, restore W1 weights, or adjust the schedule after seeing losses or results.

The original data and objective are unchanged: 2,787 TRAIN examples, 105 source-provenance warmup anchors with scalar weight zero, 2,682 active unique anchors with weight `11996/11549`, all 200 original STOP/full-hold identities protected, all histories/labels/validity masks unchanged, and the 192 driving futures crossing imposed tail retained. Use the same per-row original base loss plus 0.5 times the first-20-point near-horizon loss, with the original dynamic full-composite oracle. No new head, auxiliary loss, pre-clamp acceleration term, feature, phase input, sampling rebalance or data addition.

All model parameters remain trainable. Seed 20260906; batch size 4; AdamW learning rate 0.00003, weight decay 0.0001, gradient clipping 5.0. Preserve original framework defaults and call ordering. No AMP, scheduler, optimizer restart, per-block seed reset, early stopping, metric-based checkpoint selection, sweep or automatic retry.

## Immutable identities

- Original AH checkpoint bytes: `4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91`.
- Original W1 final model-state tensor digest: `ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f`.
- Original W1 checkpoint bytes: `b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76` (identity evidence only, never loaded for fitting).
- Original W1 plan bytes: `fe80269a6b634073586f75c132159de5bad4b631a807f0d8368ce4f3bb98e764`.
- K baseline plan bytes: `4c52532bedca9e146be0d06fb98f9e27b7263ce8cd13840426f81fa88dce4075`.
- TRAIN dataset manifest: `8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957`.
- TRAIN dataset fingerprint: `4dc6f23b62cc3b65154fb3414b1b509f3ef4696f6aa8bda9f8ee4535143cceda`.
- Warmup mask: `2c46c730096452291e10eed040c228e31ca2dc6f2ef424a02820e8d04efc3b1e`.
- Single-block exposure order: `f0d3d522b3ec4787b84286f82e61017a6aee4619270a99451d3cea08e35110d9`.
- Frozen original AH TRAIN cache: `763dcbb76046317e6cbc9254eec0ee4232bbf01cae4b8407a9f9cf761093e856`.

Validate all original W1 source hashes against the new personal worktree before use. Archive the full inherited source closure, the new runner/tests/plan, reference W1 declaration/receipt, fixed baseline plan and provenance proof in the new output. The original W1 result/source snapshot is read-only.

## Mandatory block-one identity barrier

At exactly 3,000 scheduled batches / 2,999 optimizer updates, archive the actual model in `training/first_block_identity_checkpoint.pt`, then write `first_block_identity.json` with expected and actual model-state digests, checkpoint digest, update/sample/exposure counts, and unchanged optimizer identity. This checkpoint is solely a reproduction witness, not a candidate chosen using metrics.

Require actual model-state digest EXACTLY `ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f`. Numerical closeness is insufficient. If it differs, preserve that checkpoint and a failure receipt, STOP before scheduled batch 3,001, perform no DEV pass, and do not silently substitute the old W1 model. A failed reproduction is a failed duration-only experiment, not permission to relax the gate. No first-block probes or extra train-mode forwards are permitted before this barrier.

On equality, continue immediately with the same optimizer object and its existing moments. Saving identity evidence does not reset, reload or mutate the live optimizer. There are no other intermediate model endpoints for selection. The only research candidate is the fixed 11,996-update final checkpoint. It records the complete accounting and optimizer continuity; its optimizer state is retained for audit, not automatically resumed by this runner.

## Fixed endpoint evaluation

Only after the final checkpoint and fixed-end receipt exist may the runner compute its fixed TRAIN probe and load/evaluate the known Town03 DEV. Use the exact W1 TRAIN probe identities and unchanged probe procedure. Evaluate DEV once, on the W2 final endpoint only, with the original 665 examples and original metrics/gates. Previously published W1 DEV results are reference evidence; do not reevaluate W1 or select an endpoint using DEV. DEV is repeatedly used development data, not independent heldout evidence. TEST remains untouched. No training-data approval, model promotion, closed-loop success or vehicle-control approval is implied.

## Resource, failure and handoff envelope

Use only the existing personal workspace `/home/pro6000/personal/hwanhong/portable_e2e`, its `venvs/py312`, and a fresh copy of the original `source_worktrees/drive_warmup_supervision_ab_20261001_v1` plus the three new W2 files. Physical GPU0 UUID is `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`; retain the original exclusive GPU0 flock, idle-memory/utilization checks and single-visible-device verification. Do not install packages or use GPU1.

Preserve the original environment: unset `PYTHONPATH`, `PYTHONHOME`, `PYTHONSTARTUP`, `PYTHONUSERBASE`; set `CUDA_VISIBLE_DEVICES` to the UUID, `CUBLAS_WORKSPACE_CONFIG=:4096:8`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=16`, `MKL_NUM_THREADS=16`. Freeze Python/Torch/CUDA/cuDNN versions in the start receipt. The first-block exact identity gate remains authoritative even if these text versions match.

Fit budget is fixed at 3,600 wall seconds, checked before/after every scheduled batch. Exhaustion or nonfinite state aborts the experiment rather than producing an incomplete candidate. Preserve partial checkpoint/history/failure evidence; no automatic restart or continuation. W1 measured 394.17 seconds for one block, so four equivalent blocks suggest approximately 26.3 minutes of fitting, plus preflight/checkpoint/probe/DEV overhead. This is an estimate, not a deadline extension or throughput guarantee.

The new runner is `tools/e2e/run_ah_warmup_duration_w2_20261001.py`; tests are `learning/tests/test_warmup_duration_w2_20261001.py`. `--contract-only` is local and does not initialize CUDA or load data. `--validate-only` performs the same CPU/data/source preflight in the personal remote environment but does not acquire GPU or train. Fresh output is required; existing output is never overwritten. The parent reviews and launches the final command separately.
