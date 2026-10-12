# Remote-only packaging handoff — not executed

<!-- HH_261001 - Keep every new sensor array off the nearly full local filesystem. -->

Source: `tools/e2e/prepare_map_route_geometry_remote_20261001.py`, SHA-256 **8c656bb1324049e27c9ec0c6fc2a533f465f529beb84ab9ae733ec9443d4d5e0**. Reference plan **5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68**. Candidate manifest **3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6**.

No remote connection, upload, array packaging or training has run. No authorization receipt exists yet. This source cannot train a model and refuses array materialization anywhere except host `pro6000`, existing personal `/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312`, exact source directory below, and a fresh `prepared/` output.

After parent review, use the existing personal transport to create only this fresh private study directory:

`/home/pro6000/personal/hwanhong/dataset/map_route_geometry_matched_20261001_v1/`

Before transfer, check remote free space; budget at least 2GiB for raw+compressed arrays+temporary memory-backed processing. There is no local array or tar staging, deletion, overwrite or install.

1. Copy `reference_plan.json`, `audit_v1/candidate_manifest.json` to that directory and the single packager into its `source/` subdirectory. Verify the three hashes above remotely.
2. Generate the exact transfer list on stdout and stream it directly into the existing rsync transport's `--files-from=-`, source root `/home/a/autoware_e2e/`, destination the fresh study's `raw/`. Preserve listed relative paths. Command to produce the list:

   ```sh
   OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 -B tools/workspace_python.py tools/e2e/prepare_map_route_geometry_remote_20261001.py --print-files
   ```

   Exactly **1,008 files / 248,309,612 bytes**: 480 PNG+480 sensor NPZ, 44 source/owner/route/lifecycle references and four metadata manifests. The candidate frames are the only NPZ files. This excludes whole mapping directories, protected DEV/holdout arrays, rendered maps, old checkpoints and duplicate base caches. Existing shared base arrays are referenced in-place from `/home/pro6000/personal/hwanhong/dataset/map_appearance_matched_20261001_v1/` after full SHA verification; no new local or remote copy of that 430MiB package.
3. Parent—not this preparer—creates reviewed `package_authorization.json` with exact fields:

   ```json
   {
     "schema": "portable_e2e.map_geometry_package_authorization.v1",
     "study_id": "map_route_geometry_matched_20261001_v1",
     "candidate_manifest_sha256": "3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6",
     "reference_plan_sha256": "5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68",
     "study_train_authorized": true,
     "package_authorized": true,
     "fit_authorized": false,
     "full480_source_qa_reviewed": true,
     "protected_dev_distance_reviewed": true,
     "raw_guardrail_ontology_caveat_reviewed": true,
     "ctrack_train_eligible": false,
     "broad_registry_modified": false
   }
   ```

   This example is **not an approval**. Hash actual reviewed bytes and supply that hash to the command; there is no default approval and no broad site-role rewrite.
4. Parent runs the packager in the existing remote personal CPU environment, replacing `REVIEWED_APPROVAL_SHA256` with that real hash:

   ```sh
   CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 PYTHONNOUSERSITE=1 \
   /home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/bin/python -B \
   /home/pro6000/personal/hwanhong/dataset/map_route_geometry_matched_20261001_v1/source/prepare_map_route_geometry_remote_20261001.py \
     --materialize-remote \
     --expected-builder-sha256 8c656bb1324049e27c9ec0c6fc2a533f465f529beb84ab9ae733ec9443d4d5e0 \
     --expected-plan-sha256 5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68 \
     --expected-approval-sha256 REVIEWED_APPROVAL_SHA256
   ```

Outputs only fresh remote `prepared/new_false480.npz`, `training_schedule.npz`, `manifest.json`. Source/raw hashes are checked before and after. All480 IDs and per-frame class counts must match the candidate manifest. The package preserves shared labels and exact original seven-class mapping, including tag28→other/tag2→sidewalk barrier caveat. It does not relabel, drop, evaluate or train any frame.

Separate trainer implementation/freeze and explicit fit authorization remain required. Queue behind parent-owned W2/drive work. No GPU is touched by packaging.
