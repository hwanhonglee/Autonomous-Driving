# Frozen warmup-supervision live comparison

<!-- HH_261001 - Register both endpoints and unchanged closed-loop conditions before fitting. -->

Parent is the original AH checkpoint
4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91.
After both fixed final full-model checkpoints exist, export W0_full_supervision
then W1_warmup_anchor_off to data-only NPZ, with actual checkpoint/array/metadata
hashes and verified parent/final tensor digests. No best-DEV checkpoint selection.

Run both arms exactly once, in that order, on the same owned AH-v2 Town07 TRAIN
route, ClearNoon/seed0/Epic, original six cameras and postprocessing, 20Hz physics,
10Hz inference, 20s total budget and 9km/h research cap. Keep the existing raw
argmax selection, speed preview, controller, runtime gates, initial expert-history
handoff and emergency policy unchanged. Actual initial histories must be recorded;
the same configuration does not guarantee bit-identical physical startup states.

Keep every native state, six-camera image, all twelve raw candidate outputs,
selected trajectory, acknowledged command, intervention and source/lifecycle
receipt. Report progress, low-speed duration, signed-speed/reverse behavior,
first moving-to-stopping-candidate transition, latency and event counts.
Produce full-duration vehicle-centered camera/trajectory PNG and GIF evidence.
Neither a zero exact-stall tail caused by velocity chatter nor zero lane-sensor
events establishes success or full body containment.

Do not omit a poor arm, enforce a speed floor, select an alternative candidate,
relabel expert motion as learned success or extend the budget after inspecting
results. Preserve original owner failures; a later independent release observation
may supplement but never overwrite them. This is an exposed TRAIN-route diagnostic,
not goal completion, all-map generalization, 30km/h or real-vehicle approval.
No automatic model promotion, dataset admission or capability completion.
