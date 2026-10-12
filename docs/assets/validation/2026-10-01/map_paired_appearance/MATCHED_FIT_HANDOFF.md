# Matched appearance package: frozen handoff

<!-- HH_261001 - Actual QA-reviewed study registration; no GPU launch by builder. -->

Ready, not fitted. Local package:
`storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1`
(430 MiB). Original diagnostic captures, failed v1, v2 owner receipts, and
fixed-model evaluations are unchanged. Only the new study registry authorizes
these exact paired TRAIN-route frames for this study; CTrack remains EVAL-only.

Both actual v2 capture audits were replayed and matched saved QA byte-for-byte
before registration. Both root review receipts cover native indices 0000/0119/0239
in both RGB conditions. Strong glare/washed highlights and blur are retained,
not learned quality qualification or filtering. All 480 paired frames are used.

| Registered member | SHA256 |
| --- | --- |
| Data manifest | `32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8` |
| Study TRAIN registry | `456b8b31ef2a3426d4c732236e7c549792ebe9d9c1cb246e0b055496441718af` |
| Source bundle | `0b6d5b4b394b7223f96701f185054656fb2aca694889b6b336e5f760514dda86` |
| Trainer | `22c1564b3c8e50e671f6ffb480fae23ca8107b6ef0c09ce3b6a4ba3aa2117968` |
| Shared contract | `9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96` |
| Builder | `44b63c994e4b9c6d044c809eb9350a4221eacf948a1e62d2413fef7d27d85fd0` |

Copy the complete package to the fresh remote directory
`/home/pro6000/personal/hwanhong/dataset/map_appearance_matched_20261001_v1`.
Copy its `source/` contents to the fresh remote directory
`/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_appearance_matched_20261001_v1`.
Do not overwrite existing remote paths; inspect any collision first.
Existing read-only evaluation data must already be present at
`/home/pro6000/personal/hwanhong/dataset/map_town02_p00_route_holdout_20260929_v1`.

Parent-owned remote CPU validation, from that exact source directory:

```bash
env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP -u PYTHONUSERBASE \
  CUDA_VISIBLE_DEVICES=GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5 \
  CUBLAS_WORKSPACE_CONFIG=:4096:8 PYTHONNOUSERSITE=1 \
  OMP_NUM_THREADS=16 MKL_NUM_THREADS=16 \
  /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B \
  train_map_appearance_matched_20261001.py \
  --expected-data-sha256 32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8 \
  --expected-source-sha256 0b6d5b4b394b7223f96701f185054656fb2aca694889b6b336e5f760514dda86 \
  --expected-registry-sha256 456b8b31ef2a3426d4c732236e7c549792ebe9d9c1cb246e0b055496441718af \
  --validate-only
```

For the parent-authorized fit, run the same command without `--validate-only`.
It requires vacant physical GPU0 and the shared nonblocking training lease;
there is no fallback GPU, install, resume, overwrite or automatic promotion.
Fresh output is `portable_e2e/runs/diagnostics/map_appearance_matched_20261001_v1`.
The fixed training pair has a 1,800-second budget; known evaluation follows only
after both final endpoints are saved. A failed partial output is evidence and
must not be silently resumed or deleted.

## Completed local verification

- 17 tests pass: 12 matched-study safeguards plus 5 immutable paired-v2 tests.
- Independent full package reload passes hashes, exact uint8 array ABIs,
  all 480 decoded RGB/target hashes, ordered frame IDs, common weights and schedule.
- Both arms consume 80,000 sample IDs: 48,231 original replay exposures and 31,769
  paired exposures, each sample 66 or 67 times. Every complete epoch includes all 1,209 IDs.
- Same B 245,831-parameter model loads through finite float32 NPZ only. CPU
  synthetic forward/backward yields all finite gradients. Initial learned-state
  hash is `edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67`.
- No GPU, simulator, installation or remote mutation was used for preparation.

Original 729 replay bytes are unchanged. Paired false/true arms share labels,
crop boxes, sample order, architecture, parent weights, optimizer settings and
one class-weight vector. Depth and pose are provenance only, never model inputs.
Known DEV/holdout measurements are final-only exposed diagnostics, not new
independent certification. CTrack failure motivated this intervention and its
later reuse must remain explicitly exposed diagnostic evaluation.
