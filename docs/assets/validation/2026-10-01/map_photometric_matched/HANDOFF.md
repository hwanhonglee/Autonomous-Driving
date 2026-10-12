# Frozen pure-photometric A/B handoff

<!-- HH_261001 - Prepared and reviewed inputs; root alone owns the scheduled GPU launch. -->

Ready, not fitted. The new thin package is3.9 MiB:
`storage/experiments/training/2026-10-01/map_photometric_matched_20261001_v1`.
It hash-references the existing430 MiB `map_appearance_matched_20261001_v1`
package. No large arrays, writable hardlinks, source config changes or new
simulator captures were created.

| Member | SHA256 |
| --- | --- |
| Thin manifest | `b7458a62bac606d0d33c0d565a11fd7f6b4eeda8ba8300d82d9eafaa9f9c3b40` |
| New study-only registry | `4e5b220400315d082bde1ae80471ee0e9f81dd3840234082eaabce4c6a0663a6` |
| New source bundle | `bacf1892edc10d03c35c4a4b867638f5423594d602b6f9f53c82373e8512961d` |
| Trainer | `32bee26d1bd68a9ad5a7a123712c75b6d8708ae2156b093b6efd30d7eed9884b` |
| Photometric contract | `371184537591c0591e51ad40859f2feed5bd842414b1d4216bf2eba15bb7e666` |
| Training plan | `c5fc73fe9aa60c1f1e10beaa14433d812eeccc457d2bd4760d382a33abc90169` |
| Fixed TRAIN preview receipt | `9f742b133bee8cdeaad882fec6905056b8b0fea164ea9503eb8ffa86f230ca55` |

## Parent-owned transfer and execution

Copy the thin package to the fresh remote directory
`/home/pro6000/personal/hwanhong/dataset/map_photometric_matched_20261001_v1`.
Copy its `source/` contents into the fresh directory
`/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_photometric_matched_20261001_v1`.
Do not overwrite existing paths. Leave the existing reference data and all
historical results unchanged.

Required existing read-only data:

- `/home/pro6000/personal/hwanhong/dataset/map_appearance_matched_20261001_v1`
- `/home/pro6000/personal/hwanhong/dataset/map_town02_p00_route_holdout_20260929_v1`

From the exact new source directory, first run CPU validation:

```bash
env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP -u PYTHONUSERBASE \
  CUDA_VISIBLE_DEVICES=GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5 \
  CUBLAS_WORKSPACE_CONFIG=:4096:8 PYTHONNOUSERSITE=1 \
  OMP_NUM_THREADS=16 MKL_NUM_THREADS=16 \
  /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B \
  train_map_photometric_matched_20261001.py \
  --expected-data-sha256 b7458a62bac606d0d33c0d565a11fd7f6b4eeda8ba8300d82d9eafaa9f9c3b40 \
  --expected-source-sha256 bacf1892edc10d03c35c4a4b867638f5423594d602b6f9f53c82373e8512961d \
  --expected-registry-sha256 4e5b220400315d082bde1ae80471ee0e9f81dd3840234082eaabce4c6a0663a6 \
  --validate-only
```

Only after root review and completion of the existing W pair, run that command
without `--validate-only`. The shared physical-GPU0 lease and idle preflight are
mandatory. No install, alternate GPU, resume or overwrite is supported. Fresh
run output is `portable_e2e/runs/diagnostics/map_photometric_matched_20261001_v1`.
Both fresh fits have the fixed1,800-second pair budget; known evaluation opens
only after both final checkpoints are saved. A failed output is preserved.

## Frozen experimental checks

- Original B initialization, architecture and1209 frame IDs remain the same.
  Both arms have80,000 serialized exposures:48,231 original and31,769 paired.
  All original729 inputs are unmodified; both arms use paired TRUE480.
- Only candidate paired TRUE crops call the archived grayscale/brightness
  transform. Expected calls:15,886 ON out of31,769 eligible. The complete gate
  trace and final independent PCG64 state are pinned. No sampling/crop RNG is
  consumed by this policy.
- Control is freshly trained, not reused. Its final learned-state digest must
  equal the preceding identical appearance model:
  `d4ebb807000d6b0713c4f41cc96dfe8ddb05aac06166b878d678bd37533039b5`.
  Its checkpoint, history and endpoint metadata are preserved even if that
  strict reproducibility guard fails.
- Local thin/reference metadata and full original data reload passed. The
  first/middle/last scheduled batches match the predecessor appearance-input
  RGB and targets byte-for-byte in both new arms before candidate augmentation.
- Eleven new tests pass, including exact archived transform/RNG behavior,
  original-only exclusion, target/crop equality, new registry boundaries,
  no checkpoint reuse, and all24 native preview panels' pixel replay.

## TRAIN-only review and known risk

All six fixed TRAIN examples were inspected, with seeded policy draws plus
declared ON examples at blend0.5/brightness0.35. Geometry/target alignment is
unchanged, but thin distant paint and fixtures become harder to see. Clipped
highlights are not recovered. Every example was retained, with no strength
or probability retuning and no CTrack input or learned prediction used for QA.

The prior September29 policy trial FAILED retention: Town05 lane+6.158pp but
Town01 lane−5.485pp, principally far-field false positives. Its policy, plan,
results and spatial-diagnosis receipts are archived in the new package. This
test differs in applying the policy only to TRUE480 and initializing from B;
it is not a claim that the policy is already successful.

Preview PNGs are in [train_previews_v1](train_previews_v1/). CTrack remains
EVAL-only. No model promotion or HD-map completion follows from this study.
