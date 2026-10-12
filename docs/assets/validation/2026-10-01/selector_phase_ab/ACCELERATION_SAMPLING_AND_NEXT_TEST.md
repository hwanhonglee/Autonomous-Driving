# HH_261001 — Acceleration sampling: verified mechanism and bounded next test

## What is established

The [native sampling audit](actual_acceleration_sampling.json) rules out a coordinate-sign or stale-frame explanation in the three archived AH/S0/S1 runs. Logged `ax` exactly equals the current immutable snapshot's world acceleration projected by current yaw. The acceleration API agrees with same-tick native20Hz world-velocity differences to at most approximately 1.02e-6 m/s². Before the selected-mode switch, zero-lag vector RMSE is approximately 1.5–1.8e-8; shifting by one tick makes it approximately 1.0–1.5 m/s².

The underlying velocity actually oscillates between successive native ticks. Camera10Hz samples one phase. AH sampled the higher-acceleration phase; S0/S1 sampled the lower phase, including negative instantaneous `ax` while the two-tick velocity increase remained positive. Averaging the two **native20Hz** measurements agrees with the corresponding10Hz world-velocity difference to within 7.7e-7 m/s². Averaging successive camera10Hz `ax` values would retain the same-phase alias and is not the proposed remedy.

The [cap trace](actual_speed_cap_trace.json) establishes that the 9 km/h cap bound before the first5→1 switch. It changed throttle on12 preceding AH inferences but only1 preceding inference each for S0/S1. Their acceleration-input divergence began before cap binding. The cap, maximum throttle0.5, training's faster expert acceleration, native oscillation, and learned mode selection must not be conflated into one proven cause.

## Causal transform and turn/reference-point validation

Prepared Common10 contains each anchor's observed timestamp, orientation quaternion, velocity, and yaw rate. Raw20Hz TRAIN records were located and SHA-matched for original Town07 and CTrack-left; Town01 native raw was not locally verified. No yaw-rate approximation is needed to rotate successive velocities.

The collector's `vy` is shifted from its API reference point toward the rear axle, whereas its `ax/ay` retain the original API acceleration convention. To preserve that convention, use the source-declared wheelbase `L`, not a newly estimated physical wheelbase:

```text
v_center_body(t) = [vx(t), vy(t) + (L/2) * yaw_rate(t)]
v_world(t) = R(yaw(t)) * v_center_body(t)
a_camera_body(t) = R(-yaw(t)) * [v_world(t) - v_world(t-1)] / dt
```

Here `t-1` is the immediately preceding observed camera frame, `dt≈0.1s`. The new helper uses only current/past states. It preserves the original `ax/ay` if there is no previous camera pose; it does not zero them. Missing/noncausal cadence, nonfinite state, or unsupported reference wheelbase fails explicitly. It changes only copied acceleration feature channels, not labels, validity masks, routes, images, velocity, steering, or commands.

The [source-pinned law audit](camera_acceleration_law.json) covers611 retained TRAIN intervals, including161 CTrack intervals with |yaw rate|>0.05 rad/s. The exact-pose, reference-corrected derivative matches the causal native two-tick acceleration reference within1.35e-7 m/s². Naive body-velocity differencing would err by as much as0.648 m/s² longitudinally and9.171 m/s² laterally on CTrack. The helper is `tools/e2e/causal_camera_acceleration_research_20261001.py`; its tests cover rotation, reference-point correction, causality, original first-observation fallback, and nonmutation.

## Approximate replay, explicitly not live evidence

The [exact-input attempt](jpeg_input_replay_verification.json) failed because actual inference consumed raw BGRA but old archives retained JPEG95. One unchanged baseline forward already differed numerically. No exact replay claim follows from these files.

A separately authorized **approximate identical-JPEG** experiment subsequently ran all183 recorded inferences for each of AH, S0 and S1:1098 forwards total, zero optimizer updates, zero live runs. Each pair used the same JPEG tensors and all other original inputs; only the derived acceleration history differed. All12 raw candidate outputs, all3294 JPEG hashes, both input-tensor hashes per frame, and exact checkpoint identities are retained under:

`storage/experiments/validation/2026-10-01/approx_camera_acceleration_replay_v1/`

The JPEG baseline selected the recorded candidate on549/549 frames, but its output arrays were not bit-exact (global maximum discrepancies approximately0.04012m XY,0.02038m/s speed,0.02056 logits).

| Fixed model | Selected candidate changed | First5→1 in recorded sequence, baseline→intervention | Low-speed preview effect |
| --- | ---: | --- | --- |
| AH | 6/183 | 1.9→1.4s | Mean change≈−0.001m/s |
| S0 | 6/183 | 0.9→1.3s | Mean change0 |
| S1 | 6/183 | 0.9→1.3s | Mean change0 |

These are selections on the **original recorded states**, not new vehicle trajectories or predicted closed-loop progress. No stalled-start cure was demonstrated. The transformation is not uniformly beneficial when attached to existing weights without matched training.

## Proposed next bounded intervention — no new fit authorized by this note

First finish the already frozen W0/W1 experiment and use its new exact-input archives for an independently labeled sensitivity check. Do not alter either frozen training run or either live controller to incorporate this transform.

If the parent chooses a subsequent input-semantics experiment, use one fixed pair from original AH: raw acceleration inputs versus this exact camera-interval transform applied identically to **all TRAIN histories and runtime histories**. Keep the same full-model parameter set, original composite oracle/base+near loss, original labels, all2787 records including warmup/STOP, same seeded order and fixed3000-update budget, and matched fresh optimizers. Do not also change warmup supervision, cap, throttle, gains, decoder, or mask. Cache the pose-derived features causally by episode/sample identity before shuffling; never differentiate across episode boundaries or use future target velocities. Use preserved original acceleration only for the first available observation without a predecessor, and record that fallback count.

The numerical13-feature ABI remains unchanged but the acceleration **semantics must be versioned and checkpoint-bound**; a transformed model must never silently load under the raw-input runtime contract. Both final checkpoints must precede one frozen known-DEV evaluation. Keep STOP and1.5–2.5m/s diagnostics separate from immediate starts; evaluate both models in the same owned bounded simulation with their matching preprocessing. This proposal tests a measured sampling issue but does not add low-speed expert coverage, prove feature completion, or authorize promotion.

Any9-versus30km/h cap test is a separate intervention. The existing collector treats BasicAgent brake>0.05 as an emergency: keeping its target at9km/h could confound a30km/h learned-cap arm through overspeed braking. Resolve and preregister that shared teacher protocol explicitly without weakening the safety override before claiming a cap-only comparison.
