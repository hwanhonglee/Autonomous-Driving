<!-- HH_261001 - Freeze one TRAIN-phase selector weighting hypothesis after a failed generator-loss pair. -->
# Frozen-AH selector phase-weighting pair

The completed acceleration pair did not cure the stall. All 56 fixed TRAIN probe selected/oracle IDs remained unchanged after either continuation. Six stopped-start probes still select DRIVE3 rather than the full-composite oracle DRIVE5. Original AH has an existing runtime-gate-passing DRIVE5 at all six, with preview speed 0.596–0.705 m/s, while selected DRIVE3 is zero. The full-composite oracle advantage is 11.26–12.57 cost units; original score gradients already point in the correct direction. This motivates isolating sampling/loss mass from candidate generation, not claiming a missing selector gradient or new launch truth.

This bounded experiment uses exact original AH SHA `4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91`, not either regressed continuation. Keep its architecture and existing two Linear selector heads (3,084 parameters). Freeze every other parameter, normalization buffer, encoder, fusion and all DRIVE/STOP generation. Cache TRAIN fused256 context, all12x64 candidates, original logits, targets/masks and authenticated unchanged full-composite costs exactly once in eval mode. No candidate is an oracle merely because it moves or passes runtime checks.

Both arms reset AdamW (LR1e−4, weight decay1e−4), seed20260906, gradient clip5, batch4, exactly3,000 optimizer steps /12,000 exposures. Both begin with the same AH head bytes. No early stop, hyperparameter sweep, checkpoint selection, automatic retry, new head, label clipping/retiming, data admission or success promise.

Both arms use the same new cyclic-uniform schedule: four complete permutations of all2,787 TRAIN anchors plus852 positions. Seeded permutation-boundary identities already in an incomplete batch move to the end of that cycle; no within-batch duplicate is allowed. Every anchor has at least four exposures. This common schedule is different from the preceding whole-model K1-balanced pair; the **only S0-versus-S1 difference** is loss weighting.

The frozen objective per anchor remains `0.1 × CE(full-composite oracle) + 0.1 × expected detached full-composite regret`. The original all12 full-composite oracle is never changed by a phase or near-speed heuristic.

Assign exactly one TRAIN stratum, in this priority order:

1. `stop_or_full_hold`: frozen AH composite oracle index≥6, **or** every original valid target speed≤0.1 m/s. This priority protects STOP even if a target would otherwise satisfy start conditions.
2. `low_speed_immediate_start`: otherwise abs(actual current vx)≤0.5 m/s, first three future labels valid, and recorded target speed at0.3 s >max(0,current vx)+0.1 m/s.
3. `low_speed_other`: otherwise abs(actual current vx)≤0.5 m/s, including delayed launch and warmup/hold cases.
4. `moving`: every remaining anchor.

All four must be nonempty. These thresholds define experimental weighting, not relaxed target-quality or runtime gates. All original labels, masks and stops remain. Existing low-speed full-future feasibility limitations remain unresolved.

S0 (`S0_uniform_selector`) uses ordinary per-example mean. S1 (`S1_phase_balanced_selector`) uses a fixed50:50 mixture of ordinary mean and equal-stratum mean. With scheduled phase count n_g and N=12,000, each S1 sample receives fixed weight `0.5 + 0.5*N/(4*n_g)`. Thus every original sample has weight≥0.5. Freeze exact counts and weights in `cache_receipt.json` before either fit; they may not be tuned from DEV or live outcomes.

Checks: all2,787 anchors cached and exposed; exact identical ordered exposure hash; immutable cache hash; generator+buffer state SHA excludes only four original selector tensors and must match AH, S0 and S1. CPU tests also require all12 decoded XY and speed outputs to be bit-identical after a real head optimizer step. Save finite full-model checkpoints with schema `portable_e2e.selector_phase_ab_20261001.v1`, distinct actual checkpoint hashes and parent-AH identity. No model promotion.

Save **both** irreversible final checkpoints before loading DEV evaluation inputs. Then run each once on the same known Town03 DEV, unchanged nine absolute metric gates and paired nonregression. Report fixed TRAIN phase oracle agreement/regret/near-speed error before/after; do not select an endpoint from TRAIN probes. Any later simulator trials keep the same bounded AH controller/runtime gates and their separate preregistration. Known DEV and TRAIN-route simulation are not independent capability validation.

## Earlier exposure wording correction

The previous acceleration pair retained a2,787-anchor TRAIN pool, but its first four K1 schedules exposed2,748 unique anchors (39 unsampled). This addendum corrects any reading of “all2,787 retained” as “all2,787 exposed.” Original frozen plans, schedules, checkpoints and failure evidence remain unchanged. The new pair enforces complete exposure directly.

## Execution ownership

Parent stages the new source bundle in personal storage and runs only physicalGPU0 after read-only preflight and idle/lease checks. This agent does not launch remote training. Dataset K1 plan/manifest and original AH checkpoint remain exact; source-only tests and separate new tools do not alter frozen learning modules.
