# HH_261001 — Prospective V4 Town07 low-speed moving recapture and QA

V4 is one new, create-only TRAIN capture at
`storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v4`.
The earlier V1/V3 capture and QA receipts are immutable. V3 forensic work found
327 fixed 40–120 m windows, but V3 remains unqualified: its planned
`carla_physics_substeps.py` was absent from the owner source archive, the frozen
control replay compared two different vehicle reference points, and the frozen
future audit consumed `(summary, rows)` as `(rows, summary)`.

The V4 physical capture policy is unchanged: the same Town07 ClearNoon TRAIN
straight route, BasicAgent/v2 normal control, inherited physics, 20 Hz native
ticks, 10 Hz six-camera bundles, 9 km/h target, 3.5 s warmup, 6.5 s tail,
40–120 m selected progress, 10-camera history, 6.4 s future and one 180 s
maximum attempt. The owner retains the 5 GiB floor, 512 MiB preflight capture
reserve and 32 MiB closing reserve. No threshold, source episode, route,
controller, window, or failed frame is substituted or relaxed.

Before any server start, the V4 owner archives every source named in the V4
task plan and source/QA addendum, including the unused inherited-physics module,
and verifies current and archived hashes. It archives the same eight fixed
CARLA `agents` Python files and records actual inherited class/module identities
before and after the worker. A missing file or changed hash fails before launch
or during postcheck. The source attestation covers declared Python files and
the eight inherited package files; it is not an OS/binary attestation.

The V4 QA runs the frozen numerical checks over every selected window and all
original history, native guard, future, body, pixel, ACK, event, timing and
source gates. Two adapters correct the identified audit defects:

1. Reconstruct the actual actor-center speed used by the normal policy from
   recorded rear-reference state as
   `hypot(vx, vy + wheelbase_m/2 * yaw_rate)`. Independently match that to the
   recorded CARLA world-center planar speed and to the policy observation,
   each at the original absolute tolerance `1e-9 m/s`.
2. Receive `geometry_futures` as `(summary, rows)` and pass the original rows
   to the fixed window census. All nine 64-point future bounds remain intact.

The V4 selected denominator is fixed to 327. A different count or any failed
selected window blocks the necessary-check result; no filtering is allowed.
Outside-protected native failures and whole-recording goal/stop quality remain
visible. Even a clear V4 result is `NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW`:
`training_data_approved=false`, no conversion, and no new admission authority.

V4 tests steady-cruise data provenance in the 40–120 m corridor. The learned
A/B stalls observed near 15–25 m are outside that scope. A separate prospective
10–30 m low-throttle/recovery capture would be needed to study that basic-driving
bottleneck; this protocol cannot shift the fixed window retroactively.

The task plan must be declared first, then the source/QA addendum; both are
create-only. The exact plan SHA256 is recorded in the addendum and both SHA256
values are required by the V4 owner. A read-only owner dry-run can be used to
check disk and command preflight. This protocol does not authorize a simulator
run during preparation. A future authorized launch uses the owner
`--execute` flag and an empty V4 capture path.

The create-only task plan is
`storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v4.json`,
SHA256 `92b95eb6f7c535a4d6d0c4bc463db0960aeb86834bc8d6c02c3ae8a6ac8e2015`.
The addendum at
`storage/experiments/validation/2026-10-01/town07_low_speed_moving_v4_source_and_qa_addendum.json`
binds this exact digest and must be supplied by its own recorded SHA256.

After a finished capture and stopped server, hash its `owner_plan.json` and
native `manifest.json` and run the V4 auditor with both hashes, the frozen task
plan/addendum hashes, and a fresh external output directory:
`docs/assets/validation/2026-10-01/town07_low_speed_moving_v4_qa_v1`.
The auditor retains the full ledgers or a fail-closed unassessed-evidence
record. No capture, dataset conversion, training or admission follows QA
automatically.
