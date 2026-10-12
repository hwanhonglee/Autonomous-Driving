# Fixed initial-acceleration A/B

<!-- HH_261001 - Freeze a bounded gradient repair before any new DEV metric or candidate closed-loop result. -->

The clean AH Town07 diagnostic records 366 acknowledged learned commands out of 377 driving commands, 15.2206 m progress, then a 9.25 s stationary tail. All 183 inference outputs meet the unchanged deadline; there are no collision/lane-invasion events and no emergency override. This is a reproducible learned-control failure, not route completion or feature approval. Full six-camera playback and raw-output analysis remain beside this plan.

TRAIN-only annotation audit of K1's 2787 anchors finds 295 near-stationary anchors, 105 warmup holds, and only eight driving low-speed starts with a representable first acceleration. None of those eight passes the full future-trajectory runtime gate. The newer five moving straight episodes provide no low-speed starts. This experiment does not solve that data shortage or admit the learned diagnostic as expert trajectory data.

Actual AH TRAIN gradients show zero derivative at a negative first acceleration clamped to zero speed, and at upper-speed saturation; the added pre-clamp term restores a derivative. However, at the recorded Town07 start anchor 000037, the full-horizon oracle is candidate 5 while the selected candidate is 3. Selection and supervision coverage remain possible causes; the clamp is not claimed to be the sole cause of the live stall.

## Frozen pair

- Parent: exact AH checkpoint SHA256 `4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91`.
- Arms: `A_ah_continuation` and `B_preclamp_acceleration`, both initialized independently from that same parent with a new optimizer.
- Each arm: 3000 steps, batch 4, seed 20260906, AdamW learning rate 3e-5, weight decay 1e-4, gradient clipping 5.0. Four complete 750-batch K1 schedules; two straight, one left, one right per batch. Exact ordered exposure hashes must match.
- A: unchanged composite trajectory loss plus 0.5 times the frozen 2 s AH near-horizon loss.
- B: the same loss plus fixed weight 0.05 times first-five-point acceleration smooth-L1.
- Acceleration target: adjacent recorded target-speed differences divided by 0.1 s; the first difference uses measured current speed. Both speed values must lie in [0, 30/3.6] and absolute target acceleration must be at most 2.9 m/s². Both adjacent target masks must be valid. Impossible pairs are omitted from this additional term only, not clipped or relabeled. Original target masks/data are unchanged.
- Apply the auxiliary only to the unchanged full-horizon composite oracle when it is one of the six DRIVE candidates. STOP oracles receive exactly zero additional loss and gradient. Negative acceleration and hold targets remain included.
- Existing model architecture, physical decoder, 12-candidate ABI, runtime gates, optimizer sampler, original base objective, and original train histories are unchanged. A forward hook observes the existing trajectory head; it does not replace its output.
- No tuning, sweep, retry, intermediate checkpoint, early stopping or DEV checkpoint selection. Save both fixed final checkpoints before the single frozen Town03 DEV evaluation of each. Report all nine existing absolute metric gates and B-versus-A nonregression, including failures. The unchanged dataset loader validates the shared corpus, which can read non-TRAIN contract records before fitting; no DEV model metrics are computed during training.
- Fixed TRAIN probes cover every episode and sequence indices 0/35/37/39/40/80 plus the last anchor. They report first-0.5 s speed, raw-acceleration loss and eligibility; they are not independent validation.
- Then test both exact final checkpoints using the same owned 20 s Town07 research route/controller/gates. Preserve stops, collisions, interventions, raw plans, teacher proposals and command ownership. No learned driving success, training admission or deployment follows automatically.

## Code and verification

- Trainer: `tools/e2e/run_ah_initial_acceleration_ab_20261001.py`, SHA256 `08c2349c3dc497767b25acb6b64f6dd45f830aff531d75b3a55db4858388dd93`.
- Loss/hook: `learning/portable_e2e/initial_acceleration_research.py`, SHA256 `239d75d7481f0173d86762929d41caa46af0aaabc6bf8812586eeb08fcbca962`.
- New tests: 7 passed, including nonzero dead-zone gradient, STOP zero gradient, negative acceleration, impossible pairs, invalid masks, no mutation, hook removal, frozen dependency digests and a real CPU optimizer/final-checkpoint smoke.
- Checkpoint schema: `portable_e2e.initial_acceleration_ab_20261001.v1`.
- Output: `<fresh_run>/<arm>/training/final_checkpoint.pt`. Each final records the parent digest, exact arm, 3000 steps, state digest, source hashes and exposure digest. Safe NPZ export and live control bind each new exact checkpoint separately; original AH metadata remains unchanged.
- Estimated training for the pair: roughly 12–20 minutes on the approved physical GPU0, plus dataset validation, fixed probes, final DEV and export. This estimate is not a deadline or success guarantee.

Remote entrypoint, from a new canonical-layout personal source worktree, with the existing personal py312 environment and approved GPU0 UUID only:

```bash
env CUDA_VISIBLE_DEVICES=GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5 CUBLAS_WORKSPACE_CONFIG=:4096:8 PYTHONNOUSERSITE=1 OMP_NUM_THREADS=16 MKL_NUM_THREADS=16 /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python tools/workspace_python.py -m scripts.e2e.run_ah_initial_acceleration_ab_20261001 -- --baseline-plan /home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/moving_recovery_k_20260922_v1/plan.json --parent-checkpoint /home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/driving_near_20260926_ah_v1/K1_current_plus_successful4/training/final_checkpoint.pt --output /home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/ah_initial_acceleration_ab_20261001_v1 --validate-only
```

After a successful read-only CPU preflight, remove only `--validate-only` for the one fixed paired training run. Parent agent owns staging, GPU-idle checks, execution and later simulator runs. No GPU1 use, remote installation or foreign-process control is authorized by this plan.
