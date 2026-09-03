# Portable E2E trajectory baseline

This package is the ROS-independent training core for a future CARLA +
real-vehicle planner. It is deliberately independent of the existing VAD code.

It provides:

- the versioned `common_10hz_v1` six-camera data contract;
- a standard-library-only validator with split-leakage checks;
- a framework-neutral metadata, trajectory, route, and calibration loader;
- a deterministic dependency-free CPU control-flow smoke;
- an optional real PyTorch baseline that decodes all six images and predicts
  six candidate 6.4-second trajectories and speeds;
- masked best-of-K loss, resumable checkpoints, open-loop evaluation,
  trajectory PNG evidence, and fixed-split report comparison.

The learned output is a trajectory, never throttle, brake, or steering. No
checkpoint produced here is approved for vehicle control. Obstacle avoidance,
lane-change safety, Autoware integration, closed-loop validation, and the
geometry-aware BEV candidate remain later gated work.

## No-install contract smoke

From the repository root, with Python 3.10 or newer:

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 -m portable_e2e.control_flow_smoke

python3 -m portable_e2e.validate --help
```

This smoke never imports PyTorch, Pillow, ROS, or CARLA.

## Read-only raw archive verification

Large ZIP files can be checked before extraction with the standard-library-only
verifier. Supply the exact object size and a locally recorded SHA-256. Add
`--verify-payload-crc` to stream every member and verify local headers, lengths,
and CRC without extracting anything.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 scripts/e2e/verify_zip_archive.py /path/to/archive.zip \
    --expected-size 123456789 \
    --expected-sha256 '<64-lowercase-hex-characters>' \
    --verify-payload-crc > /path/to/new-report.json
```

The verifier fails closed on a changed file, symlink, unsafe/colliding path,
encrypted or unsupported member, multi-disk archive, malformed ZIP/ZIP64
metadata, configured resource limits, or a payload CRC error. A PASS proves
archive integrity under those checks; it does not grant data-use rights or
qualify the contents for training.

Gzip-compressed TAR archives have a separate streaming verifier. It reads the
entire gzip/TAR stream without extraction and rejects unsafe paths, links,
devices, sparse/vendor size overrides, duplicate portable paths, CRC/EOF
failures, and configured expansion/resource limits.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 scripts/e2e/verify_tar_archive.py /path/to/archive.tgz \
    --expected-size 123456789 \
    --expected-sha256 '<64-lowercase-hex-characters>' \
    > /path/to/new-report.json
```

An extracted or locally collected dataset tree can be hashed and compared with
another tree without following links. The optional second path must contain
exactly the same regular-file paths and bytes: missing, changed, and extra files
all fail.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 scripts/e2e/verify_tree_manifest.py \
    /path/to/source_tree /path/to/copied_tree \
    > /path/to/new-tree-comparison.json
```

For an SSH transfer the two trees are on different machines, so generate one
manifest on each host and compare their fingerprints as shown in the full
operating guide.

Prepared datasets additionally bind every selected camera sample to a hashed
`source-manifest.v1`: native frame ID, native timestamp, raw sequence index,
source artifact digest, prepared-image digest, and exact six-camera selection.
The stored camera timestamp must equal the native timestamp. Route input is not
accepted as arbitrary sample metadata; the validator reconstructs the fixed
1 m-spaced, 120 m look-ahead local route from the hashed episode route and the
anchor ego pose. Raw-source terms and artifact authenticity still require a
human release review.

## Real image-based trainer

The real trainer additionally requires an approved isolated environment with
PyTorch, NumPy, and Pillow. Do not install those packages into system Python,
Conda `base`, or another user's environment.

```bash
python -m portable_e2e.train /path/to/common10_dataset \
  --run-dir /path/to/new_run \
  --split train \
  --device cpu \
  --max-steps 1

python -m portable_e2e.evaluate /path/to/common10_dataset \
  --checkpoint /path/to/new_run/checkpoints/latest.pt \
  --output-dir /path/to/new_evaluation \
  --split val \
  --device cpu
```

GPU use is never automatic: only an explicit `--device cuda:<index>` can create
a CUDA context. Existing run or evaluation directories are not overwritten.
Evaluation runs one unmeasured batch warm-up and records its batch/sample count.
Compared reports must have identical warm-up counts, batch size, device,
runtime—including PyTorch/OMP/MKL thread settings—and hardware. Forward latency
is reference-only because this workflow does not reserve shared-system resources
or scheduler capacity.
Checkpoint loading requires a PyTorch release that supports restricted
`weights_only=True` loading, verifies the bytes it actually loads, and rejects
symlinks. A digest still proves identity rather than publisher trust, so use
only checkpoints whose recorded provenance you recognize.

The canonical data and model configurations are:

- [`config/common_10hz_v1.contract.json`](config/common_10hz_v1.contract.json)
- [`config/perspective_trajectory_v0.model.json`](config/perspective_trajectory_v0.model.json)

The full beginner operating guide is
[`../docs/portable-e2e-training.md`](../docs/portable-e2e-training.md). Dataset
selection and model gates are documented in
[`../docs/portable-e2e-datasets.md`](../docs/portable-e2e-datasets.md) and
[`../docs/portable-e2e-model-v0.md`](../docs/portable-e2e-model-v0.md).
