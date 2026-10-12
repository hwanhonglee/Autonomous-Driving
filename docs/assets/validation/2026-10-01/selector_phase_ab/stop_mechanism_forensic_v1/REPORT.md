# HH_261001 — Town07 15–25 m stop-mechanism forensic audit

## Decision

The Oct 1 A/B sudden stops are **real 20 Hz CARLA state changes, not just 10 Hz camera/display stutter**. The selected model trajectory was **not asking for zero speed at the instant** of the first or second drop. Every relevant command was accepted, frame-bound, and read back as requested. That does **not** prove the requested brake produced a particular physical torque. The likely failure surface is the coupled **positive-but-declining preview target → almost-zero throttle / small brake → nonlinear low-speed CARLA response**. The existing recordings cannot isolate which of those components is causal.

These runs remain failed route-completion diagnostics; no model or real-vehicle promotion follows from this audit.

## Native-tick evidence

`vx` is the recorded forward base-frame speed. Each event also changes the *raw CARLA world XY velocity*, and raw CARLA acceleration projected along the preceding travel direction agrees with the 50 ms speed difference. Values below are the first large-deceleration tick; A/B second stops finish at zero on the next tick under the same held command.

| Run / native frame | Route m | Forward speed m/s | Raw along-travel acceleration m/s² | Requested throttle / brake | Selected 0.3 s target m/s | 0.3 brake after zero? |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| A 1075 | 16.397 | 0.873 → 0.001 | −17.431 | 0 / 0.00010 | 0.872 | frame 1077 |
| B 1510 | 15.706 | 0.785 → 0.001 | −15.690 | 0 / 0.00129 | 0.784 | frame 1514 |
| A 1199 | 25.130 | 1.449 → 0.521 → 0.001 | −18.558 | 0 / 0.02307 | 1.416 | frame 1201 |
| B 1640 | 25.033 | 1.459 → 0.532 → 0.001 | −18.529 | 0 / 0.01086 | 1.443 | frame 1642 |

At the 25 m second tick, A/B retained the same selected targets (~1.416/~1.443 m/s) and small held brakes while speed became ~0.001 m/s. The next *fresh* inference then selected near-zero speed, causing the 0.3 hold brake. Thus the conspicuous 0.3 command is a **response after the measured stop**, not its initiating command. In the preceding second before the first A/B drops, commanded throttle was only 0.0034–0.0214 / 0.0021–0.0165, with zero brake. That near-idle regime is a plausible contributor but is not an intervention test.

The older W1 20 s run, which shares A's model tensors but is a separate physical execution, showed a partial drop at 14.894 m (1.421→0.792 m/s, brake 0.02894, selected target 1.484) and stops at 23.508 and 25.275 m (1.201→0.148 and 1.198→0.146 m/s, brakes 0.03291/0.02893, selected targets 1.154/1.375). Its separate 150 s run also showed rapid drops near 15.0, 15.94 and 24.71 m. Variation in exact stop location and the prior [BasicAgent same-position pass](../directional_live_pair_v1/README.md) argue against a universally impassable map point, but do not distinguish control from contact/physics.

## Command and timing audit

The four archived runs have 400/400/400/3,000 contiguous state frames. Simulation timestamps advance by 0.050000001 s throughout. On *every* state, the expected receipt exists, is `ACKNOWLEDGED` and server-accepted, was issued after the preceding frame, and its throttle/brake/steer equals the next recorded `current_control` to 1e−6. Each logged learned decision also matches its receipt and command frame. All eight checked mismatch categories are zero in all four runs. A/B each include 366 acknowledged learned commands; W1 20 s includes 366 and W1 150 s includes 2,966. The direct raw-velocity and acceleration evidence means a low image Hz alone cannot explain the vehicle's measured stopping. This audit does not evaluate wall-clock rendering smoothness or PC load.

The tracker uses the selected trajectory's 0.3 s speed and proportional gain 0.7. If target and current speed are nearly equal, it commands little or no throttle; if slightly below, it commands small brake. It does not directly regulate acceleration or preserve a minimum rolling torque. This explains *why those commands were requested*, not why CARLA converted a very small brake or zero throttle into a one/two-tick ~16–21 m/s² slowdown. Acknowledgement proves command delivery/readback, not wheel force, tire contact, brake torque, or their precise actuation timing. Collision and lane-invasion sensors reported no event, but absence of events does not certify a full contact-free trajectory.

## What would settle it

Run the already proposed *native-only, one-owner* speed/throttle isolation at matched initial speed and route locations before any more fitting: same 15–16 and ~25 m positions, fixed 50 ms tick, no model/vision, small throttle versus 0.2 throttle, with brake held exactly zero, then a separately pinned small-brake arm if disk and safety gates permit. Compare world velocity, world acceleration, ACK/current control, collision/contact telemetry and repeats. Only if the collapse persists with brake zero should the low-throttle/physics branch gain support. If it disappears, isolate the small-brake transfer function. A model/controller change should be judged on new owned A/B drives, not the existing offline preview alone. Do not use these diagnostic clips as training labels.

## Reproduce without CARLA or GPU

From repository root:

```bash
python3 tools/e2e/forensic_town07_stop_mechanism_20261001.py
```

The command reads only the four archived `storage/experiments/validation/2026-10-01/` runs: `directional_gradient_A_town07_sparse20_v1`, `directional_gradient_B_town07_sparse20_v1`, `warmup_supervision_W1_warmup_anchor_off_town07_v1`, and `warmup_supervision_W1_town07_long_v1`. It prints per-source SHA-256, the join-integrity counters, and each native-tick event. It does not decode image data, load checkpoint tensors, write artifacts, or start simulation. The exploratory candidate definition is *previous `vx` >0.7 m/s, `Δvx/Δt` <−8 m/s², and route progress 14–26 m*; this is a forensic screen, not a driving-quality acceptance threshold.
