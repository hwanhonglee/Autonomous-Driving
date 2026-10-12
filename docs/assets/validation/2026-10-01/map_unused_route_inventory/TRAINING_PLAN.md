# Geometry-source matched pair — frozen endpoint protocol

<!-- HH_261001 - Preregister the two final endpoints and preserve the exact source-only intervention. -->

Study: `map_route_geometry_matched_20261001_v1`. Existing package authorization grants only this study-specific TRAIN role and personal remote packaging; **separate fit authorization is still required**. No broad site-role change, local arrays, CTrack TRAIN, simulator, install or GPU fallback.

## Locked inputs and sole factor

Reviewed remote package manifest: `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d`; raw-derived `new_false480.npz`: `f1fd23b786aa4dee7373054ed1580a4ad7a87715e05a62d2ec31b4ae35591f05`; matched schedule archive: `80659aa9eeecd3de590623a88f357a615dd39f74f268c756ee18ed46ff9ef5d9`. Source closure binds these and the unchanged reference plan `5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68`.

Both arms use common original729 FALSE + paired480 TRUE. Extra480 slots use fixed old Town01 replay in **control**, versus full p02/p04 raw240 each in **geometry**. Exact index list and order remain the reference plan. All 480 new frames, including 32 lane-absent p04 views and historical slow/stationary positions, remain included. The old tiny-sign rejection, inter-route overlap and raw barrier taxonomy caveat are unchanged.

Both fresh fits initialize original context-B NPZ `ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e`; exact initial state `edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67`, 245,831 parameters. No preceding favorable endpoint is reused as initialization/control.

Architecture, native RGB640×360/255, seven classes, semantic loss, fresh AdamW lr3e-5/weight_decay1e-4, clip5, seed20261001, batch8 and **10,000 steps/80,000 exposures each** are unchanged. Use the exact actual preceding class-weight vector in [the pre-fit correction](WEIGHT_PROVENANCE_CORRECTION.md), not recalculated geometry weights. Same aligned crop sequence `19077c804ffc32131ea1851388ae943bdf848e206fb2cc9e7de691e9866e05ba` and common slot sequence `93aa16695cfcf9a8e4a0ae0560a46727de8347e526107e0ae583e063106c6d28`.

Per arm: original slots34,507, TRUE slots22,743, extraFALSE slots22,750; new geometry p02/p04 exposures11,376/11,374. Control's actual original examples total57,257 because extra slots replay original data. The common TRUE exposures and original Town02 exposures remain identical. This differs from prior 1,209-slot runs; old endpoints are not substituted for this fresh matched control.

Archived gray/brightness policy is applied after shared crop in **both arms**, only to **729≤slot<1209**. New FALSE slots≥1209 are never augmented. Separate PCG64 seed20261003, applied11,349/22,743; trace `7e969c9c239f9c1db32d175c2ecb28c4fb94847ceb23d6b0d3e637bb4a8731ec`. Labels are unchanged.

## Endpoints, receipts and evaluation order

Run control then geometry sequentially on parent-owned personal physical GPU0 under the existing exclusive nonblocking lease. UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5` must be idle; no process termination or GPU1 fallback. Existing personal py312 venv/source/run paths only.

Each arm has one final `checkpoint.pt` with optimizer state, safe finite-float32 `weights.npz`, individual tensor hashes/shapes, initial/final state hashes, actual slot/crop sequence hashes, per-slot and physical-source exposures, common class weights, photo trace and per100-step loss history. No intermediate evaluation, “best” checkpoint selection, early convergence stop, DEV loss or hyperparameter sweep.

Only after **both complete 10k checkpoints and weights are saved and hash-verified** is `fit_complete.json` written. Then fixed Town01 DEV118 and Town02 p02 holdout232 are opened. Save every prediction and full7×7 confusion, target/predicted/FP pixel counts, null IoU for absent target classes, and no promotion thresholds. Both are previously exposed research splits, not independent DEV/TEST. External CTrack/paired four-scenario CPU evaluation remains a separate fixed-endpoint operation; this trainer does not open CTrack.

The training/final-save phase is bounded to **2,100 seconds per arm** and known evaluation to **300 seconds per arm** (40 minutes combined, below45). Python alarm plus loop deadlines enforce these bounds within the owned process; they are not an OS guarantee against an uninterruptible driver/kernel. No unrelated process is signaled. Budget/nonfinite/identity failure preserves histories, a clearly named non-endpoint `aborted_state.npz` where possible, failure receipts, and already-saved finals; it never resumes or selects a partial arm. An arm failure before both finals prevents DEV opening.

Both final endpoints remain diagnostic/non-promoted even if metrics improve. Source/input hashes are rechecked after fitting/evaluation. Failure records and outputs are never overwritten. A new run requires a new reviewed version; no automatic retry with more steps, data changes or relaxed checks.
