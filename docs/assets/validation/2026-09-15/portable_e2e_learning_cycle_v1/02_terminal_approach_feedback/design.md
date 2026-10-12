# V9 — Prospective terminal feedback, fixed reference physics

<!-- HH_260906 - Freeze the control hypothesis before actual collection; observational shadow replay is not a rollout or a success claim. -->

This revision keeps the existing Town07 straight TRAIN route, ClearNoon, seed 0,
Prius, nominal 28.8 km/h cruise, original PID gains, 20 Hz native/control and 10 Hz
six-camera capture. Both cases explicitly use reference_10ms with max_substeps 10.
The failed previous 5 ms setting is not adopted. No vehicle/model actuation, data
admission, remote command, installation, TEST payload access or training is authorized
by this experiment itself.

There are exactly two prospective cases, coast A then feedback B, one attempt each.
The inherited coast arm must pass execution, scalar, camera, initialization, ACK and
independent control-policy checks before B can run. Full future gate failures remain
reported and cannot authorize admission. Any operational failure blocks B; no retry.

## Control hypothesis

The inherited controller releases normal pedals based on one empirical coast-distance
reference. The previous fine-setting capture became motionless outside the goal; no
normal hold brake was latched. Remaining distance and measured velocity can provide a
bounded feedback request without changing numerical physics or goal-completion rules.
This motivates the new controller but does not predict recovery or generalization.

The feedback arm latches only after the existing coast phase, during driving, at
measured speed <= 0.75 m/s, without failure/emergency or original hold latch. Outside
the original measured goal window, with remaining route arc r and measured planar speed v:

```text
target = min(0.35, max(0.12, 0.25 * max(r - 0.75, 0)))  [m/s]
error = max(target - v, 0)                             [m/s]
desired_throttle = min(0.15, 0.08 * min(1, error / 0.10) + 0.5 * error)
requested_throttle = min(desired_throttle, previous_request + 0.10 * measured_dt)
```

Throttle decreases immediately. The tapered feed-forward term is continuous and zero
at/above the target; it does not maintain a positive pedal after target speed is reached.
Both target and throttle are zero inside the original goal window. There is no new
normal service braking until the inherited measured hold latch, which retains brake 1.
Maximum target 0.35 m/s is a reference, not a proven measured-speed limit. The original
actual-speed and native acceleration/deceleration checks still apply.

Original goal tolerance 1 m (route arc AND planar distance), upstream/no-overshoot
checks, stopped speed <= 0.1 m/s, measured dwell 2 s, coast limit 45 s, warmup 3.5 s,
stationary tail 6.5 s and total limit 180 s are unchanged. The coast clock continues
through feedback; the latch does not restart a timeout. Emergency authority is unchanged.
Existing low-speed emitted steering zero and stopped-brake guard remain separate hooks.
The feedback reference is passed through the existing PID target-speed interface. Its
gains and update method are unchanged, but its internal target/history need not match
the coast arm after feedback activation, even before the first positive emitted pedal.
The prefix comparison concerns measured physical state and emitted controls, not hidden
PID history or an unchanged target-speed input.

## Required evidence and unresolved risks

Record the measured decision source, all mathematical/requested/stored pedal values,
pre-emergency normal output, final output, ACK and subsequent observed control. Preserve
same-row emergency supersession without rewriting previous observations. A requested
pedal or accepted ACK does not prove exact physical application time.

Compare all physical rows through the first actual changed normal/final pedal SOURCE,
excluding that source's treated next_control only. Merely latching the feedback state is
not a changed pedal: pinned old-reference shadow analysis found the <=0.75 latch at
native index 1008, while the first prospective nonzero throttle would occur at index
1113. Shadow replay is not a feedback rollout. The actual new pair must establish its
own first changed request and measured prefix.

Risks remain: different low-speed vehicle response, early stop, overshoot, throttle
cycling, timeout, and inherited launch/hold-boundary future curvature failures. Report
all original future count/rate/max results and native intervals. Do not adjust labels,
retime targets, erase failed captures, loosen thresholds or start training on diagnostic
results. One pair cannot establish repeatability or all-Town operation.
