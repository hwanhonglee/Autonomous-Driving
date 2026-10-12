# HH_261001 — Prospective V3 moving capture QA, pending authority

This capture-preserving QA applies only to the first capture at
`storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1`
under the unchanged resource-guard task plan and the predeclared V3 source
addendum. Run it after the owner has finished and the server is stopped. It
does not start CARLA, convert a dataset, edit a recording, or approve TRAIN data.

`audit_town07_low_speed_moving_v3_20261001.py` runs the frozen V1 numerical
auditor over the entire independently selected 40–120 m anchor set and all
history, native guard, future, body, pixel, ACK, event, timing and source gates.
It retains the original numerical summary and every ledger. It then checks
that the V3 owner command, immutable addendum, eight archived `agents` Python
files, actual imported module/class paths, before/after worker snapshots,
postcheck and original worker exit all belong to that same prospective run.
The V1 report's historical source-limitation field remains in the nested
summary; the separate V3 field records whether the eight-file capture-time
source supplement resolved that specific gap. Other Python, OS and compiled
dependencies are outside this source supplement.

Use a fresh output directory. The two hashes are taken from the completed
capture, then passed explicitly so the QA cannot select a different owner plan
or native manifest:

```sh
qa_run=storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1
qa_episode="$qa_run/episode"
test -d "$qa_episode" || qa_episode="$qa_run/episode.partial"
test -f "$qa_run/owner_plan.json" && test -f "$qa_episode/manifest.json"
qa_owner_sha=$(sha256sum "$qa_run/owner_plan.json" | cut -d ' ' -f 1)
qa_native_sha=$(sha256sum "$qa_episode/manifest.json" | cut -d ' ' -f 1)
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py \
  -m scripts.e2e.audit_town07_low_speed_moving_v3_20261001 -- \
  --owner-plan-sha256 "$qa_owner_sha" \
  --native-manifest-sha256 "$qa_native_sha" \
  --output docs/assets/validation/2026-10-01/town07_low_speed_moving_v3_qa_v1
```

Exit 0 means `NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW`: the fixed
numerical checks and V3 eight-file source evidence both cleared. It is not a
qualification/admission receipt. A numerical failure is
`BLOCKED_NECESSARY_CHECKS`; a source failure with clear numerical checks is
`BLOCKED_SOURCE_PROVENANCE`; unavailable or malformed evidence is
`BLOCKED_UNASSESSED_EVIDENCE`. In every case `training_data_approved=false`,
`dataset_conversion_performed=false` and
`admission_authority=NONE_NEW_SCOPE_REQUIRES_REVIEW`. Failed selected windows
remain in the denominator and outside-window failures remain in the original
numerical report. No retry, trimming, label rewriting, conversion or training
follows automatically.

The synthetic V3 checks and the frozen V1/source-wrapper tests passed together:
`37 passed` on 2026-10-01, without starting a simulator. The real capture is
now complete. Its exact owner/native hashes were passed to this QA, which
returned `BLOCKED_UNASSESSED_EVIDENCE`; the [unaltered blocked receipt](town07_low_speed_moving_v3_qa_v1/summary.json)
records `declared worker closure differs: tools/e2e/carla_physics_substeps.py`.
The original fixed plan declared this optional file, while the owner archived
only imported local modules. The run used `physics_substep_profile=inherited`,
so the optional module was not imported or included in the owner source archive.
The file's current SHA matches the plan, but that is **not** an archive-time
source proof and does not turn the frozen gate into a pass.

<!-- HH_261001 - Preserve the failed frozen QA while reporting separate, non-admission diagnostics. -->

A separate [forensic raw summary](town07_low_speed_moving_v3_forensic_v2/summary.json)
examined the same pinned owner/native records without changing either record
or the frozen QA. It found 327 independently selected anchors spanning 32.6 s;
all selected windows passed the raw camera/history/future/body/discrete-bound
checks, all 4,188 JPEGs decoded, and ACK/snapshot checks passed. Four raw
acceleration failures outside the protected windows remain visible, including
the original 3.19 m/s² launch and tail braking. It also found two further
auditor defects that a later protocol must fix **before** a new capture:

- The frozen control replay compares direct CARLA actor-center planar speed
  with `hypot(vx, vy)` at `base_link`. `base_link_state` applies the yaw-rate
  lateral correction, so these are not identical quantities. In the existing
  capture, 1,131/1,195 normal proposals differ by more than the frozen 1e-9
  comparison; maximum absolute difference is 0.001531 m/s. A future audit
  must reconstruct the correct reference point or record the direct sample,
  without silently widening a posthoc tolerance. The separate diagnostic
  reconstructed actor-center speed as `hypot(vx, vy + 2.85/2*yaw_rate)` from
  the recorded coordinate contract; it matched all 1,195 policy samples
  within the **unchanged** 1e-9 m/s criterion (maximum error 8.88e-16 m/s).
- The frozen audit unpacks `geometry_futures` as `(rows, summary)` even though
  that helper returns `(summary, rows)`. After the earlier failure is resolved,
  this would raise a `KeyError` before a complete QA decision.

The forensic result remains `FORENSIC_ONLY_SOURCE_CLOSURE_BLOCKED`, with
`fixed_control_replay=false`, `training_data_approved=false`, and no dataset
conversion. The original V3 QA is **not** retrospectively passed or used to
admit data. A new prospective, correctly sourced capture and audit are the
next step.
