# Portable E2E project operating requirements

<!-- HH_261002 - Persist the user's continuing goals and operating limits across agent/context handoffs. -->

## Continuing objective

Build and validate our own portable E2E driving model against the nine main domains and thirty subfunctions in `docs/portable-e2e-feature-roadmap.md`. Develop automatic mapping as a separate model supporting both live inputs and recorded drives. Keep the driving model usable without an automatically generated HD map; map assistance is an additional, separately validated input.

Prioritize reliable basic driving, speed tracking, turns, and goal stopping before adding further capabilities. Adding a head or performing more optimizer updates does not itself demonstrate a driving capability. Use actual closed-loop measurements and independent evaluation to decide whether a change helps.

The evolving status and latest evidence are indexed in `docs/assets/validation/2026-10-02/README.md`. Read that index and the relevant latest result before resuming; do not infer present status from old IDE tabs or conversation claims. Do not count a short Town07 precursor, teacher drive, synthetic test, or offline loss improvement as a formally approved main/subfunction. Retain the roadmap's prospective evaluation criteria and failed-attempt denominator.

## Environment and safety limits

<!-- HH_261012 - Preserve the user's clarification that fixmoving names a future platform rather than a vehicle model. -->

- `fixmoving` is the name of the user's future platform, not a vehicle model or manufacturer. Keep its branch as a platform development baseline; do not infer hardware, CAN/DBW specifications or require a vehicle model to define this branch's purpose. Any later connection to an actual vehicle still requires its own specifications and safety authorization.
- Local machine: CARLA collection, inference, closed-loop testing, and real visual evidence. Remote PRO6000: model training/evaluation under `/home/pro6000/personal/hwanhong`.
- Use only physical GPU 0 on PRO6000. Verify availability immediately before a job; do not use GPU 1 or terminate another user's work.
- Install packages only in the user's private `hwanhong` venv when installation is actually needed and within the user's authorization. No system/base-environment installation, arbitrary reboot, or remote Autoware build as a substitute for training.
- Serialize shared CARLA/GPU jobs; use the existing ownership locks, declared ports, actor/PID-generation guards, source/asset seals, and cleanup checks. Do not attach to, reload maps in, or kill another test's simulator.
- Simulator work does not authorize real-vehicle actuation or automatic model promotion/deployment. Keep research, teacher data, and approved vehicle functionality separate.
- Do not train on failed or explicitly denied raw captures, hide failed tails, extrapolate missing true futures, fabricate stationary curvature labels, or lower safety/evaluation thresholds to obtain PASS.
- Preserve existing user edits and the canonical workspace layout. Do not restore removed legacy paths/symlinks. Do not overwrite frozen experiments, declarations, sources, or model exports; new experimental interventions require explicit new revisions and fresh outputs.
- Add code-change explanations using `HH_YYMMDD - English explanation` with the actual date. Use valid comment/provenance syntax for each file format.

## Communication and evidence

Explain, in readable Korean, the overall objective, main/subfunction counts, current part of the objective, actual improvements and remaining problems, the next implementation/training/testing sequence, and an approximate duration when there is supporting evidence. Distinguish **running**, **completed**, **prepared**, and **not yet implemented**; never say training is continuing if the job has ended.

Explain whether each activity runs locally or on PRO6000, whether control comes from the learned model or a teacher, and whether a failure is from model behavior, runtime latency, vehicle control, or infrastructure. Do not invent credit balances, exact future token cost, resource availability, performance equivalence to other models, or completion dates.

During ongoing work provide concise, substantive progress updates; do not end a turn after only a tiny check if the authorized implementation and safe relevant next step remain. A request to continue does not authorize new destructive/system/shared-resource actions.

Store understandable, categorized results and reproduction links. Visual evidence must come from actual recorded inputs/state, keep the vehicle centered with route/traveled/model trajectories visible, and retain complete declared intervals and failures. Distinguish recorded diagnostic renderings from genuine Autoware/RViz screenshots; never fabricate captures or call an illustrative rendering an actual test.
