<!-- HH_260814 - Define the source-owner result layout for four-PC VILS evidence. -->
# VILS result packages

Each PC publishes its own evidence under a directory named for the source owner.
All hosts in one experiment must use the same `run_id`; files from different runs
must never be merged merely because their wall-clock windows overlap.

```text
result/
  PC1/<run_id>/  planning, control, vehicle, CAN, and safety evidence
  PC2/<run_id>/  physical perception, PC4 ingress, validation, fusion, and provenance
  PC3/<run_id>/  localization, GNSS, TF, diagnostics, MRM, and cross-PC observations
  PC4/<run_id>/  CARLA ground truth, adapter output, gateway, and fault schedule
```

Rules:

- Commit reports, manifests, topic counts, validation results, and SHA-256 digests.
- Store approved large binary evidence through Git LFS; never add large files as
  ordinary Git blobs.
- Do not publish live credentials, GitHub tokens, NTRIP passwords, private runtime
  configuration, or secrets recovered from process environments.
- Preserve interrupted runs and label them explicitly. Never concatenate data from
  a later boot into an interrupted run.
- A result marked useful for communication analysis is not automatically a safety
  acceptance result or a completed formal paper scenario.

PC owners should add their own same-run result directory without rewriting another
PC's evidence.

## Current result ownership

| PC | Result present on this branch | Note |
|---|---|---|
| PC1 | no | Add the PC1-local artifact from `/home/a/pc1_vils_runs/`; do not substitute a remote observation. |
| PC2 | yes | `20260814T091155Z-pc2-power-interrupted` (bounded partial evidence; no PC2 rosbag was started). |
| PC3 | yes | `20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01` |
| PC4 | yes | PC4 source evidence and bounded partial result are retained under `PC4/`. |
