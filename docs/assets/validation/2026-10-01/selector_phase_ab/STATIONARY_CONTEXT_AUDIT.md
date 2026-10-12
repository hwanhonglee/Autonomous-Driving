# HH_261001 — Stationary-context audit, not a causal conclusion

## Findings

The TRAIN-only [input audit](train_stationary_context_alias.json) uses the exact frozen AH cache and verifies 123 original TRAIN anchors against their cached targets. It compares all 105 artificial warmup anchors with the six stationary driving anchors at sequence 35/37. There are no new model forwards, optimizer steps, DEV measurements, label changes, or discarded rows.

No pair has byte-identical full model inputs. In full-history warmup versus first driving sequence 35, fused-context cosine similarity is approximately 0.99944–0.99999. Camera pixels differ, and the stopped steering history changes. This is evidence of nearby contexts, not proof of nonlinear nonseparability or contradictory identical inputs.

The externally imposed 3.5-second full-service-brake warmup does create a specific supervision question. At full-history warmup sequence 9, the target first exceeds 0.1 m/s after 2.9 seconds; by sequence 34 it does so after 0.4 seconds. Across this interval, the car remains approximately stationary with the same route and command. The original full-horizon oracle traverses candidates 1, 0, 3, 4, and 5. Absolute time, capture phase, brake/throttle command, and an external release trigger are not named model inputs. All such anchors after sequence 9 have a full ten-sample history mask. Real small state/pixel differences remain available; an irreducibility claim would therefore be stronger than the evidence.

Legacy starts also have a conspicuous measured steering transient before translation: approximately 0.1463, 0.4075, and 0.4319 radians at sequences 35–37, with sign varying by route. This is an observable cue, not evidence that the trained head actually relies on it.

## Actual handoff counterevidence

The separate [live handoff audit](stationary_context_live_handoff.json) reconstructs the ten history frames from exact camera receipts, with raw-state and command-file SHA256 identities. Its new association tests pass (3 tests).

| Run | Handoff speed | Launch steering history | Initial selected mode | First 5→1 transition |
| --- | ---: | --- | ---: | --- |
| Original AH v2 | 0.7620 m/s | Approximately zero | 5 | +1.9 s at 2.3271 m/s |
| S0 uniform selector | 0.7986 m/s | Opposite sign to original Town07 TRAIN | 5 | +0.9 s at 2.0368 m/s |
| S1 phase-balanced selector | 0.7302 m/s | Exactly matches original Town07 TRAIN steering history | 5 | +0.9 s at 2.0358 m/s |

S1's other ego-history differences are tiny (largest per-feature difference approximately 3.1e-6), but full camera/route identity is not established. All three select a moving plan and accelerate after handoff, then switch toward slowing trajectories and eventually stall. Consequently:

- Missing legacy steering cues cannot explain S1's failure.
- The live failure is launch followed by regression to a stall, not refusal to launch at the first learned command.
- Identical source/controller code did not produce identical startup states. The pair remains a fixed-protocol research comparison, not an exactly state-matched causal experiment.
- Warmup semantics deserve correction in future collection contracts, but this audit does not establish them as the sole or principal cause of the observed stalls.

## Concrete prospective remedy — proposal only

The next useful acquisition target is genuinely policy-active low-speed driving: an observable release from rest, sustained 0.5–2.5 m/s driving, and a real measured goal stop. Current fitted selector heads do not substitute for that missing coverage. No claim is made that another learning-rate or epoch increase will solve it.

For a new versioned acquisition contract, explicitly record the control owner and whether an anchor is eligible as a policy target. Keep imposed setup braking in the raw episode and available as causal input history; do not silently teach the policy to reproduce an unobserved setup countdown. Preserve genuine expert STOP/hold examples and their reasons. If a task really requires an external wait/release decision, expose the corresponding causal intent in both training and runtime through a reviewed contract change, not a fabricated elapsed-time label.

This is not permission to filter the existing 105 warmup anchors, reclassify failed captures, or use the learned trial's future as expert truth. Any research ablation of existing acquisition-only anchors requires a separate frozen declaration, explicit counts, and preserved STOP supervision. The proposed source/provenance distinction must be applied before inspecting outcomes, rather than selecting convenient successful windows afterward.

The completed low-speed expert v2 already supplied 22/22 initial 10 Hz windows clearing all nine existing future bounds. The whole episode nevertheless failed native acceleration (+3.1939 m/s²) and measured goal stopping (overshoot while moving), so it remains unadmitted. The immediate data dependency is a validated source/actuator protocol and successful whole-episode QA—not another relabeling rule. The planned 0.10-throttle revision was paused because historical evidence does not support calling it gentler.

All research approval, training admission, and feature-acceptance statuses remain unchanged.
