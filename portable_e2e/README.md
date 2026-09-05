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
  trajectory PNG evidence, and fixed-split report comparison;
- a fail-closed tensor/runtime contract and an isolated Autoware shadow adapter.

The learned output is a trajectory, never throttle, brake, or steering. No
checkpoint produced here is approved for vehicle control. The shadow adapter
publishes only under `/planning/portable_e2e/` and has not yet run in a live
ROS/CARLA campaign. Obstacle avoidance, lane-change safety, drivable/collision
selection, fallback/MRM, learned closed-loop validation, and the geometry-aware
BEV candidate remain later gated work.

The current research baseline is the ten-epoch v0 checkpoint. On all 337
Town03 validation samples it reports 6.4-second ADE/FDE of
`6.567681/16.171295 m`, speed MAE `1.872983 m/s`, yaw MAE `0.379922 rad`,
kinematic speed MAE `1.697376 m/s`, and model-forward time
`0.702218 ms/sample`. These are open-loop values, not full-runtime timing. The
runtime geometry gate v6 passed `0/337` for every candidate and the learned
selection, while the selector chose candidate 1 for `337/337`. Selected
failures included geometric speed, speed disagreement, step, reported/geometric
speed rate, distance disagreement, curvature, and lateral acceleration at
`326/337`; heading at `310/337`, backward step at `94/337`, and speed at
`47/337`. Eleven samples were rejected first because the recorded current speed
exceeded the strict 30 km/h input cap.

The source checkpoint SHA-256
`370f12dbfa15cc17fa29931bc3c9dd3140dbd7c0a61d976af296fd223b2becf0` was
exported to the pinned, non-executable `.runtime.npz` bundle SHA-256
`b9b10e1604ac59eb4375b233d80b8f7ea04d983c0b841d6afddbc39e008c292c`. Its local CPU
strict load passed. A `.pt` file is accepted only by the trainer, evaluator,
and read-only auditor; live shadow inference accepts only the pinned runtime
bundle. This does not establish live ROS/CARLA execution or a 10 Hz pass. The
physical v1 decoder is implemented and unit-tested but remains untrained.

## No-install contract smoke

From the repository root, with Python 3.10 or newer:

```bash
CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
  python3 -m portable_e2e.control_flow_smoke

python3 -m portable_e2e.validate --help
```

This smoke never imports PyTorch, Pillow, ROS, or CARLA.

## Fresh CARLA Common10 quick start

As of 2026-09-05, fresh Town07 straight, C-track left-turn, and Town03
right-turn runs have passed native collection and `common_10hz_v1` planning
validation at a 30 km/h target. Town07 309 and C-track 304 samples form the
train split; Town03 337 samples are validation, for 950 total. This is a data
and open-loop pipeline result, not proof of learned closed-loop autonomy.

This workflow needs the repository's packaged CARLA 0.9.15 setup. It does not
install Python packages. From terminal A, keep the exact map server running:

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"

scripts/e2e/run_carla_map.sh Town07 \
  --port 2100 --quality Low \
  -- -windowed -ResX=1280 -ResY=720 -nosound
```

Wait for `CARLA_READY`. In terminal B, collect into a new, nonexistent run
path. Do not add `--allow-map-load`; cold-start a different map instead.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
source scripts/e2e/env.sh

route_file="$REPO_ROOT/docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
native_episode="$REPO_ROOT/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode"
test -f "$route_file"
test ! -e "$native_episode"
mkdir -p "$(dirname "$native_episode")"

python3 scripts/e2e/collect_carla_vad_expert.py \
  "$native_episode" "$route_file" \
  --host 127.0.0.1 --port 2100 \
  --physics-hz 20 --capture-hz 10 \
  --target-speed-kmh 30 --max-duration-sec 60 \
  --stationary-warmup-sec 3.5 --stationary-tail-sec 6.5 \
  --spawn-z-offset-m 0.5 --weather ClearNoon --seed 0
```

The three declared phases are `stationary_warmup`, `driving`, and
`stationary_tail`. They provide real history and 6.4-second stopping context
without retiming or duplicating camera frames. Each state contains a physical
front-wheel measurement converted to ROS-positive-left
`steering_tire_angle_rad`, plus `speed_limit_mps` converted from CARLA's
measured speed-limit API. Missing fields fail closed.

Convert a clean-HEAD native episode with the exact adapter interface below.
The example license identifier records internal/manual-review status; it is
not a grant of dataset use or redistribution rights.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"

native_episode="$REPO_ROOT/datasets/raw/carla/common10_v1/2026-09-04/30kph/town07/straight/ClearNoon/seed_0000/run_001/episode"
dataset_id='carla-common10-30kph-20260904-v1'
prepared_dataset="$PORTABLE_E2E_ROOT/datasets/prepared/$dataset_id"
test -d "$native_episode"
test ! -e "$prepared_dataset"
test -z "$(git status --porcelain)"
mkdir -p "$(dirname "$prepared_dataset")"

CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
python3 scripts/e2e/prepare_carla_common10_dataset.py \
  --episode "train=$native_episode" \
  --output "$prepared_dataset" \
  --dataset-id "$dataset_id" \
  --license-id project-generated-carla-internal-review \
  --image-mode copy \
  --validate-mode planning
```

The verified run produced 309 train samples over 30.800000459 seconds at
9.999999851 Hz, with 100.000002 ms p99 gap, zero camera bundle skew, and 100%
valid future points. `schema`, planning, image hashes, native frame binding,
and canonical route reconstruction passed. The report deliberately leaves raw
source content review and pixel decode as `NOT_RUN`, records no online runtime
test or independent raw-state trajectory recomputation, and still requires
manual release review. An offline 1 ms bundle PASS is a timestamp-skew gate,
not inference latency.

The newly collected C-track and Town03 turn episodes are distinct from the
historical short Autoware visualization runs. The new episodes are qualified
Common10 expert data; the historical PNG/GIF remain evidence of a different
VAD runtime and do not show this portable checkpoint driving Autoware. See the
[current validation report](../docs/validation-2026-09-05-portable-e2e-common10-30kph.md)
for the exact boundary.

The one-epoch values in that 2026-09-05 report are retained as a historical
duration baseline. The current ten-epoch comparison and its research-only
boundary are in the
[2026-09-06 duration A/B evidence](../docs/assets/validation/2026-09-06/portable_e2e_v0_duration_ab_v1/README.md).

## Read-only raw archive verification

Large ZIP files can be checked before extraction with the standard-library-only
verifier. Supply the exact object size and a locally recorded SHA-256. Add
`--verify-payload-crc` to stream every member and verify local headers, lengths,
and CRC without extracting anything.

```bash
(
  set -o noclobber
  CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
    python3 scripts/e2e/verify_zip_archive.py /path/to/archive.zip \
      --expected-size 123456789 \
      --expected-sha256 '<64-lowercase-hex-characters>' \
      --verify-payload-crc > /path/to/new-report.json
)
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
(
  set -o noclobber
  CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
    python3 scripts/e2e/verify_tar_archive.py /path/to/archive.tgz \
      --expected-size 123456789 \
      --expected-sha256 '<64-lowercase-hex-characters>' \
      > /path/to/new-report.json
)
```

An extracted or locally collected dataset tree can be hashed and compared with
another tree without following links. The optional second path must contain
exactly the same regular-file paths and bytes: missing, changed, and extra files
all fail.

```bash
(
  set -o noclobber
  CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
    python3 scripts/e2e/verify_tree_manifest.py \
      /path/to/source_tree /path/to/copied_tree \
      > /path/to/new-tree-comparison.json
)
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

Two source-specific, dependency-free inspectors can audit the downloaded raw
archives without extraction or image decoding:

```bash
(
  set -o noclobber
  python3 scripts/e2e/inspect_bench2drive_mini_archives.py \
    /path/to/exact-mini-10-archive-directory > /path/to/new-b2d-report.json
)

(
  set -o noclobber
  python3 scripts/e2e/inspect_nuscenes_mini_adapter.py \
    /path/to/v1.0-mini.tgz /path/to/can_bus.zip \
    --nuscenes-audit-report /path/to/tar-full-audit.json \
    --can-bus-audit-report /path/to/zip-full-audit.json \
    > /path/to/new-nuscenes-report.json
)
```

Exit code zero means the raw structure passed; it does not imply Common-10Hz
qualification. Read Bench2Drive's `common_10hz_qualification` and
`conversion_readiness`, or nuScenes' `common_10hz_v1`, respectively. Reports
contain absolute source paths; keep raw reports private unless paths are
explicitly scrubbed and the publication copy is revalidated.

The nuPlan mini DB, map, and camera-group-0 archives have passed full payload
CRC checks and archive-level log/member correspondence only. Dataset Terms and
intended use are still awaiting explicit review. SQLite
record-to-JPEG-to-calibration-to-ego-to-map joins, calibration-based mapping
from eight native cameras to six Common10 roles, deterministic route creation,
and a policy for native scenes shorter than 30 seconds are not yet evaluated.
The nuPlan Common10 adapter and training path therefore remain blocked.

## Real image-based trainer

The real trainer additionally requires the user's personal project venv. The
2026-09-04 verified environment uses Python 3.12.3, PyTorch `2.13.0+cu130`,
NumPy `2.5.2`, and Pillow `12.3.0`. Activate only
`$PORTABLE_E2E_ROOT/venvs/py312`; never install into system Python, Conda
`base`, another project, or another user's environment.

```bash
export PORTABLE_E2E_ROOT="${PORTABLE_E2E_ROOT:-$HOME/portable_e2e}"
source "$PORTABLE_E2E_ROOT/venvs/py312/bin/activate"
export PIP_REQUIRE_VIRTUALENV=true
export PYTHONNOUSERSITE=1
export CUDA_VISIBLE_DEVICES=''
```

Start with CPU while the GPU remains hidden:

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

A validated corpus containing both domains can use deterministic finite
without-replacement balancing. Domain indices are shuffled independently, then
weighted-interleaved so every prefix stays within one sample of its target
exposure. Quotas, excluded samples, seed derivation, and observed counts are
bound to the v1 run/checkpoint schema.

The default `uniform_without_replacement` policy likewise uses every sample
exactly once per epoch, but its name does not mean a uniformly random global
permutation. With two domains it preserves the dataset's original proportions
while smoothly interleaving domain order. The balanced policy replaces those
proportions with explicit ratios and records any unused majority-domain samples.

```bash
python -m portable_e2e.train /path/to/common10_dataset \
  --run-dir /path/to/new_balanced_run \
  --split train \
  --device cpu \
  --sampling-policy domain_balanced_without_replacement \
  --domain-ratio carla=1 \
  --domain-ratio real=1
```

GPU use is never automatic: only an explicit `--device cuda:<index>` can create
a CUDA context. This project is restricted to physical GPU 0. Query that
device's UUID immediately before a run and expose only the UUID; inside the
process the single visible device is named `cuda:0`.

```bash
gpu_uuid="$(nvidia-smi -i 0 --query-gpu=uuid --format=csv,noheader | tr -d '[:space:]')"
case "$gpu_uuid" in GPU-*) ;; *) printf 'GPU 0 UUID lookup failed\n' >&2; exit 2 ;; esac

CUDA_VISIBLE_DEVICES="$gpu_uuid" python -m portable_e2e.train \
  /path/to/common10_dataset \
  --run-dir /path/to/new_gpu_run \
  --split train \
  --device cuda:0 \
  --limit-samples 4 \
  --batch-size 1 \
  --num-workers 0 \
  --max-steps 1
```

If physical GPU 0 has another compute process, wait. Never expose physical GPU
1, kill another user's process, reset a GPU, or reboot the shared server for a
training run. Existing run or evaluation directories are not overwritten.
Evaluation runs one unmeasured batch warm-up and records its batch/sample count.
Compared reports must have identical warm-up counts, batch size, device,
runtime—including PyTorch/OMP/MKL thread settings—hardware, and evaluation
domain composition. Aggregate and per-domain metrics/counts are both validated
and shown; the report also carries the training sampling plan hash, policy, and
observed domain exposure. Forward latency is reference-only because this
workflow does not reserve shared-system resources or scheduler capacity.
Checkpoint loading requires a PyTorch release that supports restricted
`weights_only=True` loading. The loader rejects symlinks and files larger than
8 GiB before hashing or deserialization, then verifies the bytes read from the
same open file. A digest still proves identity rather than publisher trust, so
use only checkpoints whose recorded provenance you recognize.

Those checkpoint-loading rules apply to training, evaluation, and offline
audit. The live shadow runtime never deserializes `.pt`; it reconstructs the
model only from a separately pinned, non-executable `.runtime.npz` bundle.

Trainer CLI output, evaluation JSON, and comparison JSON may contain absolute
run, checkpoint, or input-report paths. Keep raw reports private. Before adding
generated JSON to `docs/assets` or Git, scrub those paths and revalidate the
publication copy; generated comparison Markdown omits them.

The canonical data and model configurations are:

- [`config/common_10hz_v1.contract.json`](config/common_10hz_v1.contract.json)
- [`config/perspective_trajectory_v0.model.json`](config/perspective_trajectory_v0.model.json)
- [`config/perspective_trajectory_physical_v1.model.json`](config/perspective_trajectory_physical_v1.model.json)

The full beginner operating guide is
[`../docs/portable-e2e-training.md`](../docs/portable-e2e-training.md). Dataset
selection and model gates are documented in
[`../docs/portable-e2e-datasets.md`](../docs/portable-e2e-datasets.md) and
[`../docs/portable-e2e-model-v0.md`](../docs/portable-e2e-model-v0.md). The
shadow-only launch procedure and the feature-level release gates are in
[`../docs/portable-e2e-shadow-runtime.md`](../docs/portable-e2e-shadow-runtime.md)
and
[`../docs/portable-e2e-feature-roadmap.md`](../docs/portable-e2e-feature-roadmap.md).
