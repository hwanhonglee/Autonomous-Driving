# HH_261001 — Fixed 9 km/h moving-task necessary checks, not admission

This is a new, narrowly scoped **audit-only** protocol. It does not amend the sealed
15 km/h moving authority, the native collector, the dataset converter, or any
failed episode. No dataset conversion, label rewriting, or TRAIN admission is
authorized by a clear result. Parent review and an explicit new authority remain
required. In particular, the failed `town07_low_speed_start_expert_v2` capture is
not an eligible input, even where its windows had clear mathematical checks.

## Exact prospective identity

The only input is `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1`
(its original `episode` or retained `episode.partial`). The independently
declared resource-guard plan is `town07_low_speed_moving_train_plan_v2_resource_guard.json`,
SHA256 `b8116231db1a5a548b1e890f21d403feb50bcef9403b49cbdcba5e3d055dfee1`.
Slot: `T07-LOW-CRUISE-20261001-V1`. Only prospective 9 km/h expert collection on the
pinned Town07 TRAIN route is in scope. Creation must follow declaration. The
unexecuted v1 declaration remains preserved, not silently replaced.

Owner-plan and native-manifest hashes are supplied externally after collection;
they cannot override this literal declaration, route, slot, run, or source policy.
The original owner result, native error/status, moving outcome, and whole-recording
goal/stop quality are copied into the report with original raw file hashes.

## Fixed denominator and protected interval

Independently project every recorded rear-reference XY onto the pinned route.
Select **every** driving camera anchor at inclusive 40–120 m until the first native
observation beyond 120 m permanently closes selection. Require at least 30 s
between first and last selected camera, exact 70 native warmup and 130 tail
observations, and the actual final selected future before tail. Compare independent
membership to saved flags and separately replay the frozen task state machine.
No score, curvature, model output, later successful slice, or optional trimming
defines membership.

For every selected anchor require all 10 actual camera history observations
covering at least 0.9 s, their same-frame causal native states, a preceding native
guard, and all 64 actual 0.1 s future targets. The future includes its real native
right interpolation bracket. Every native interval in their union must be
contiguous at 20 Hz and driving. The guard's incoming derivative/sweep is also
checked conservatively; this adds the immediately previous interval rather than
dropping an unfavorable edge. No future crosses stationary tail or an observation
gap. Future labels are recomputed from unchanged recorded states only; no generated
trajectory or model rollout is expert truth.

All selected windows pass together or the moving task's necessary checks fail.
Outputs retain all camera-future, selected-window, native-geometry/bounds, pixel,
projection, and control ledgers. Failed members stay in the denominator.

## Conditions and scope

Global blockers (anywhere, including warmup/tail):

- Wrong plan/source/input/route/native/owner identities; non-prospective or old failed capture.
- Incomplete native capture, task completion, cleanup, owner source/port checks, or disk-floor evidence.
- Changed inherited physics: 20 Hz synchronous, 10 Hz six-camera bundle, original substepping true/.01/10, rendering enabled.
- Missing/misaligned/corrupt original six-camera JPEGs, calibration mismatch, any clock gap, any ACK/control/state conversion mismatch.
- Any recorded collision or lane-invasion event, including outside the protected interval.
- Missing fixed anchors, insufficient history/future/span, progress reentry, early agent/catalog termination, or incomplete original raw records.

Protected-window blockers:

- Native planar speed above the unchanged physical maximum; scalar speed derivative
  or raw longitudinal `(vx_now-vx_prev)/dt` outside ±2.9 m/s² (original 1e-6
  numerical tolerance). Raw API `ax` is retained separately, never smoothed or
  substituted for either derivative.
- Any of the original nine discrete 6.4 s future checks, or first-step checks for
  the nine observed history edges. Limits and low-displacement heading semantics
  come from the unchanged pinned helpers; unavailable evidence is not a pass.
- Actor bounding-box footprint or conservative adjacent-native swept envelope
  outside the intended lane union inset by 0.10 m, using the original 1e-6 m² area
  tolerance. Raw-lane and inset margins/outside areas are both retained. Geometry
  is based on archived active XODR, actual actor bbox/pose and route lane IDs,
  not zero event counts. The bound is snapshot-linear, not a continuous-time proof.

Pre-entry launch and post-support cleanup native derivative/body violations are
preserved in `outside_protected`, plus complete whole-capture summaries. They do
not become selected training anchors or goal/launch/stop demonstrations. GoalFalse
remains GoalFalse; failed whole-recording goal/native quality remains FAIL. This
scoped interpretation is the proposed **new** authority boundary, not an assertion
that old successful-episode requirements have already been satisfied. If evidence
cannot be reconstructed safely, the report is `BLOCKED_UNASSESSED_EVIDENCE`.

## Causal controls and source limitation

Require every expert proposal to match its actual acknowledged command, frame,
same-frame observed speed and timestamp. Replay the exact launch/ramp/zero-normal-
brake/low-speed-steering wrapper; reproduce CARLA float32 **control setters only**,
never round or clip measured labels. Preserve unchanged BasicAgent emergency
returns separately. The final moving-support tick's explicit full brake is cleanup,
not another expert proposal and not stop supervision. All 20 Hz observed controls
must match their prior accepted ACK. ACK proves recorded server acceptance and
observed-command agreement, not independent physical actuator measurement.

The owner archives workspace sources. The subclass manifest binds the worker,
local planner and vehicle controller source hashes. The inherited external
`basic_agent.py` is checked against its known current hash, but its own bytes are
not separately archived/bound at capture time by this frozen worker. The report
explicitly sets `full_external_capture_time_source_closure_proven=false`. This
remains an authority-review block even if every necessary numerical check clears;
no full provenance certification is manufactured. Frozen capture sources are not
changed by this audit.

## Execution after parent-owned capture only

No live collection or training is started by this tool. The parent supplies the
two recorded hashes and a fresh output directory:

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py \
  -m scripts.e2e.audit_town07_low_speed_moving_pending_20261001 -- \
  --owner-plan-sha256 OWNER_PLAN_SHA256 \
  --native-manifest-sha256 ORIGINAL_NATIVE_MANIFEST_SHA256 \
  --output docs/assets/validation/2026-10-01/town07_low_speed_moving_pending_audit_v1
```

Results are `NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW`,
`BLOCKED_NECESSARY_CHECKS`, or `BLOCKED_UNASSESSED_EVIDENCE`. Every outcome contains
`training_data_approved=false`, `dataset_conversion_performed=false`, and
`admission_authority=NONE_NEW_SCOPE_REQUIRES_REVIEW`. A CLI zero exit means only
that necessary checks were clear, not admission.

Synthetic tests cover the complete 321-anchor fixed denominator, both protected
boundaries, off-task pulse retention, missing history/future, nine-metric and body
sweep failures, saved membership tampering, retroactive/wrong/old-source identity,
causal control/ACK replay, float32 controls, cleanup separation, and unchanged
capture/math source pins. No real capture yet exists, so end-to-end real-data
qualification is not claimed.
