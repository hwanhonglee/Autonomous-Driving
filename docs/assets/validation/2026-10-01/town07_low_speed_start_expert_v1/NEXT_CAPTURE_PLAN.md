<!-- HH_261001 - Freeze one combined controller protocol, preserving the original failure and all admission gates. -->
# One new expert-only start/stop capture

This is a combined data-quality protocol, not a controlled A/B experiment or a guaranteed repair. No learned rollout is relabeled as an expert trajectory. The complete v1 failure remains intact and unapproved.

Evidence motivating the intervention:

- V1 reached the actual goal (0.954373 m upstream, measured stopped dwell 2 s), with clean source/port/ACK checks, but failed whole-episode QA. Its worst native deceleration was −18.465346 m/s² under acknowledged **normal** brake 0.15 before the goal window; it was not emergency or tail braking.
- Historical constant-brake Town07 calibration measured minima of −19.796/−19.823/−24.626/−19.701/−16.318/−16.378 m/s² for brake 0.02/0.04/0.06/0.08/0.10/0.12. Merely lowering the brake cap to 0.02 is not supported as a reliable remedy. Historical V4 zero-normal-brake runs passed scalar QA at −1.602/−1.746 m/s², but used a different coast governor/speed and remained unapproved for future geometry.
- All 21 v1 initial low-speed full futures failed curvature at the same physical event near 8.903 s: path heading changed +0.0114868 rad over 0.0469085 m (0.244877 rad/m). Measured body yaw changed only −0.0001843 rad; applied steering was tiny but nonzero. Tiny steering is not established as the cause. Historical v6 zero-steer captures retained launch curvature failures even with zero measured wheel angle.

Frozen v2 intervention: normal planner brake cap **0**, stop-speed-envelope desired deceleration **0.5 m/s²**, normal steering **0 only at measured planar speed ≤0.75 m/s**. Keep normal launch throttle **0.13** until measured **0.5 m/s**, timeout **8 s**, then the same **0.05/s** throttle ramp capped at **0.20**. Keep calling the original PID; do not reset its state. Record raw normal proposal, capped proposal, pre/post steering control, final agent proposal, ACK and next physical state. BasicAgent emergency output remains downstream and unchanged.

Everything else is fixed: exact original 210.597591 m Town07 TRAIN route; packaged CARLA source/API/map pins; Prius, ClearNoon, seed 2026100101; 20 Hz native / 10 Hz six cameras; 9 km/h target; 3.5 s warmup and 6.5 s tail; 180 s simulated budget. The original upstream 1 m goal window, measured speed ≤0.1 m/s, 2 s continuous dwell, full tail, all scalar and nine full-future bounds remain unchanged. Coasting short, overshooting, timing out, or any QA failure remains failure. No clipping, window filtering, retiming, automatic retry, dataset admission or feature pass.

Parent launches **one** attempt after the learned A→B pair. This agent has not launched it.

```bash
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py \
  -m scripts.e2e.run_owned_town07_low_speed_start_v2_20261001 -- --execute

OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py \
  -m scripts.e2e.audit_town07_low_speed_start_v2_20261001 -- \
  --run storage/experiments/validation/2026-10-01/town07_low_speed_start_expert_v2 \
  --output docs/assets/validation/2026-10-01/town07_low_speed_start_expert_v2/raw_audit
```

Omit `--execute` for read-only preflight. Fresh output is mandatory; original v1 owner, worker, model sources and evidence are not changed. Source closure is archived and checked by the owner. The raw auditor retains every full/partial window and every image check, including initial-start and phase counts; its success is an audit-completion result, not data admission.
