# Frozen geometry trainer — parent execution only

<!-- HH_261001 - Hand off the exact independently named pair without granting or launching its fit. -->

Status: source freeze and **29/29 CPU tests PASS**, including real parent safe-NPZ tensor ABI/initial state. No trainer/GPU/SSH/install action by this preparer. Parent completed remote CPU packaging; separate fit approval is pending.

| Artifact | SHA-256 |
| --- | --- |
| `training_source_v1/bundle.json` | `20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c` |
| `map_route_geometry_contract_20261001.py` | `f73b4ec07f054102f2ffa316e5af001454b415729caf52f860591cdecae2d681` |
| `train_map_route_geometry_matched_20261001.py` | `4de75aedb780e9eb36e39d9b0f2cd239425f2b4d72af45bc3610f7f109d6a34e` |
| Remote prepared manifest | `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d` |
| Frozen training plan | `a803f683f8677e0b04c484445f4b5ee23645b1c60733b1cfb49dd067e14912c0` |
| Pre-fit weight wording correction | `ae6e56e3fd186e467abde0a9d0d89a5943b9b36541dc46bf49c7a50ac4decc09` |

The closure contains nine source/document files plus `bundle.json`; copy only that small directory using the existing personal transport to:

`/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/map_route_geometry_matched_20261001_v1/`

Never edit it after transfer. Existing immutable input package stays at `/home/pro6000/personal/hwanhong/dataset/map_route_geometry_matched_20261001_v1/`; reference429MiB package is not copied. Fresh output is `/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/map_route_geometry_matched_20261001_v1/`, with arms **control** and **geometry**.

## CPU preflight while GPU is occupied

From the exact source directory above:

```sh
env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP -u PYTHONUSERBASE \
  CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 OMP_NUM_THREADS=16 MKL_NUM_THREADS=16 \
  /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B \
  train_map_route_geometry_matched_20261001.py \
  --expected-source-sha256 20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c \
  --expected-data-sha256 db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d \
  --validate-only
```

This reads/validates TRAIN arrays, hashes, source/frame IDs, fixed class weights and schedule. It does not create the run, inspect GPU state, acquire the GPU lease, load DEV arrays, or infer fit permission.

## Separate fit approval and command

After parent review/preflight, the parent can create the private package's `fit_authorization.json`, then hash the **actual bytes**. Required content below is an example, not approval:

```json
{
  "schema": "portable_e2e.map_geometry_fit_authorization.v1",
  "study_id": "map_route_geometry_matched_20261001_v1",
  "fit_authorized": true,
  "prepared_manifest_sha256": "db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d",
  "source_bundle_sha256": "20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c",
  "reference_plan_sha256": "5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68",
  "package_authorization_sha256": "844bbc685eb0e49c427687d53432d622d3d8123e519e870423c460375bdfe43e",
  "steps_per_arm": 10000,
  "arms": ["control", "geometry"],
  "common_class_weights": [0.13594788312911987, 0.13577374815940857, 0.2083463817834854, 1.4355568885803223, 1.9548143148422241, 2.183657646179199, 0.9459031224250793],
  "no_dev_checkpoint_selection": true,
  "ctrack_train_eligible": false
}
```

Only when physical GPU0 is free and parent authorizes the fit, from that same source directory:

```sh
env -u PYTHONPATH -u PYTHONHOME -u PYTHONSTARTUP -u PYTHONUSERBASE \
  CUDA_VISIBLE_DEVICES=GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5 \
  CUBLAS_WORKSPACE_CONFIG=:4096:8 PYTHONNOUSERSITE=1 OMP_NUM_THREADS=16 MKL_NUM_THREADS=16 \
  /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B \
  train_map_route_geometry_matched_20261001.py \
  --expected-source-sha256 20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c \
  --expected-data-sha256 db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d \
  --expected-fit-approval-sha256 REVIEWED_FIT_APPROVAL_SHA256
```

No overwrite/resume or fallback options exist. GPU0 must pass idle+exact UUID+shared lease checks. Training/final-save bounds35min per arm, fixed known evaluation5min per arm, failed-state preservation up to60seconds. Python timeouts preserve failure records, not a promise against an uninterruptible OS/driver.

## Endpoint evaluator interface

`summary.json` status must be `GEOMETRY_SOURCE_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED`, with exact `prepared_manifest_sha256`, `source_bundle_sha256`, `reference_plan_sha256`, fit authorization SHA, `source_and_data_unchanged=true`, `dev_used_during_optimization=false`.

`arms.control` and `arms.geometry` each bind 10k/complete, checkpoint+NPZ SHA, initial/final tensor state SHA, individual tensor ABI/hashes, actual sample/crop sequences, per-slot and per-physical-source exposures, and identical `photo_trace` (`eligible_calls`, `applied_calls`, `sample_gate_trace_sha256`, `final_rng_state`). Per-arm `training.json` repeats those records. Final initial states must match; both actual slot/crop traces and photo traces must equal preregistration.

`fit_complete.json` explicitly says `BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV`. Only then are known Town01DEV118/Town02p02holdout232 opened. All predictions and seven-class confusion/pixel support/null absent-class IoU are stored, with `known_evaluation.json` nested split→arm. External four-scenario evaluation is separate and preregistered by the evaluator owner; no CTrack data is loaded by this trainer.

Any `failure.json`, absent completion receipt, partial count, or hash mismatch fails qualification. `aborted_state.npz` is explicitly **not an endpoint** and cannot be selected or resumed. No auto promotion, independent-validation, HD-map or driving-completion claim.
