# HH_261001 — Bounded full-model warmup-supervision pair (frozen protocol, not launched)

## Fixed question and scope

Does removing only externally imposed setup-warmup anchor supervision improve active low-speed planning, including the observed moving-candidate-5 → slowing-candidate-1 → stall failure? This is a source-semantics hypothesis, not a claimed cure. The previous full-model acceleration auxiliary regressed and the frozen-context selector phase pair did not resolve the stall. Neither result justifies a learning-rate or epoch sweep.

Use the exact original AH parent checkpoint SHA256 `4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91`, K1 manifest SHA256 `8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957`, and all 2,787 original TRAIN records. Both arms use the unchanged physical decoder, 12 candidates, 64 future points, original composite-oracle loss plus AH's 0.5 × first-20-point near loss. No preclamp auxiliary, new mode oracle, architecture change, positive-speed floor, or target clipping.

## Matched arms and bounded execution

- `W0_full_supervision`: ordinary original per-anchor loss.
- `W1_warmup_anchor_off`: exactly the same loss, with zero scalar weight only on source-provenance-identified setup warmup anchors.

Both arms fine-tune the **full model**, initialized afresh from AH, with fresh AdamW (learning rate 3e-5, weight decay 1e-4, gradient clip 5). Both share seed 20260906, batch size 4, and the existing fully covering cyclic-uniform 12,000-position order. This deliberately differs from the old balanced K1 sampler, identically in both arms. It retains every record in the forward/history/input pool and does not alter its images, labels, valid-future mask, or cache provenance.

Why full-model: a loss-provenance defect can affect learned context and generated mode shapes, not merely the final four selector tensors. The prior frozen-generator selector pair left start oracle agreement at 6/18 and still stalled after moving. A full-model pair can test that additional mechanism. It also risks generation regression, so this is a tightly bounded pair and not a promotion shortcut.

The fixed schedule contains 451 warmup exposures and 11,549 active exposures. Scheduled step 1693 (one-based), indices `[617, 619, 322, 317]`, is the only all-warmup batch. Skip this batch identically in both arms: no forward, backward, optimizer/momentum update, weight decay, or normalization update. Do not resample. Report 3,000 scheduled steps, 2,999 actual optimizer updates, 12,000 enumerated positions, and 11,996 model-forward/loss positions separately.

On non-skipped batches, W0 weights are 1. W1 active-anchor weights are the fixed global normalization `11996/11549 = 1.0387046497532253`; its warmup-anchor weights are 0. This gives both arms equal average total scalar weight across the executed schedule without changing within-batch denominators or adapting weights to outcomes. Original per-row loss is averaged over the original batch of four, after multiplying these weights. The native base and near losses already normalize by each row's valid count before averaging rows; unweighted per-row evaluation is therefore mathematically equivalent, including unequal valid-future counts. The test compares both loss and output gradients at tolerance 1e-6; floating-point reduction ordering is not claimed bit-identical. Both arms use this same reducer. No optimizer step is allowed on the skipped batch merely because its gradients happen to be zero: AdamW and momentum can still mutate parameters.

## Exact mask construction and integrity checks

Read the pinned K1 dataset manifest and its referenced TRAIN episode manifests. Verify each manifest SHA256, source collection-config SHA256, and sample JSONL SHA256 before using phase metadata. Associate each loaded example by exact `(episode_id, sample_id, sequence_index, anchor_timestamp_ns)`, never by list position alone.

For each original episode with a nonempty `native_result.capture_phases.stationary_warmup`, convert its first timestamp and the driving phase's first timestamp to integer nanoseconds with the same nearest-nanosecond convention used by the dataset adapter. The proposed scalar mask is:

`warmup_off = source_control == full_service_brake AND warmup_begin_ns <= anchor_timestamp_ns < driving_begin_ns`

The reported warmup tick duration/order must agree with the source capture schedule. Zero-warmup episodes remain wholly active. Unknown, missing, nonmonotonic, conflicting, or unhashed provenance fails preflight; do not infer phase from low speed, target shape, model errors, or sequence index. Store a receipt for every record, including interval/source hashes, chosen mask, and a digest of the full mask vector plus exposure order. For this exact pinned dataset, require 105 warmup records (35 each in the three original episodes) and 2,682 active records. Cross-check the 105 identities against the independently verified TRAIN context audit, rather than accepting its counts alone.

The original phase inventory is retained: 200 STOP/full-hold, 18 immediate low-speed start, 54 other low-speed active, and 2,410 moving examples. Every masked warmup belongs to the former `low_speed_other` group, not STOP/full-hold. Refuse the run if any of the 200 original STOP/full-hold records loses supervision. The same protected identities are used throughout; do not recalculate protection from the evolving model's predictions.

## Warmup versus tail and cross-boundary futures

Masking is at the **anchor**, not individual future points. A warmup anchor remains weight zero even if its future enters genuine driving. Its images and measured states remain available as history to later driving anchors. All original target points and validity flags remain untouched.

The pinned 2,787-row TRAIN inventory has no `stationary_tail` anchors. Some driving anchors nevertheless have futures extending into the imposed full-service-brake tail. Keep those whole futures unchanged in both arms and report their exact counts/identities from source timestamps in preflight. This pair does not attempt to solve tail ownership semantics or select only physically convenient portions of a trajectory.

The tail lacks the warmup's hidden timer-triggered *release*, but it is not automatically genuine traffic-stop supervision: the original source terminated at a route-goal tolerance and then commanded external full brake. The remaining route geometry may help identify that endpoint; there is no direct external goal-stop/hold or traffic-light intent feature. Do not claim perfect intent distinguishability. Altering the tail or introducing a go/hold input would be a separate intervention, not a second unannounced change in W1.

## Frozen assessment protocol

Save both fixed-budget final checkpoints before reading any DEV metric. Use one unchanged Town03 known-DEV pass per endpoint and the existing nine absolute/nonregression gates. It is known DEV, not an independent heldout or a checkpoint-selection set. No intermediate evaluations, early stopping, retry, LR adjustment, or endpoint choice based on it.

Before fitting, freeze TRAIN probe identities for (a) all 18 existing immediate-start records, (b) all records with current forward speed in [1.5, 2.5] m/s, (c) all 200 protected STOP/full-hold records, and (d) the original fixed warmup/start/boundary anchors. Compare original AH and both final endpoints. Record all 12 candidate trajectories, selected index/logit margin, unchanged full-oracle index/cost, selected-minus-current speed at 0.3/0.5/1.0 seconds, near speed error, and all candidate gate results. Do not call any merely positive-speed candidate an expert oracle.

After both endpoints and the fixed offline report exist, run **both** on the same existing 20-second Town07, 9 km/h-capped simulation protocol, unchanged controller and emergency gates. Preserve every actual trial, failed owner audit, startup difference, selected-mode transition, collision/lane event, emergency, reverse/chatter, and zero-speed interval. In addition to handoff metrics, report the 1.5–2.5 m/s phase: first 5→1 transition, each candidate's 0.3-second desired speed, commanded brake/throttle, resulting measured deceleration, and stationary time after the transition. Identical code is not identical initial simulator state; record the actual ten-frame handoff histories for both.

Neither Town03 nor the repeatedly observed Town07 loop is novel heldout evidence. For an independent later validation, reserve an entire newly collected, whole-episode-QA-passing low-speed episode (including its start, active cruise, and goal-stop phases) by episode ID and source hash **before the pair runs**; do not put overlapping windows or another copy of that episode in TRAIN. If such a valid episode is unavailable, say independent heldout evaluation is unavailable and do not substitute failed expert data, relabeled learned futures, or old DEV as independent evidence. The bounded research fit can still be reported as research with that limitation.

## Implementation and resource envelope

Implement only new helper/trainer/tests, reusing the immutable native loss and runtime gates. Tests must verify per-row unweighted loss/gradient parity with the original batch loss, zero warmup output-gradient with active/STOP gradients preserved, no target/input mutation, full-history retention, exact shared schedule/skip behavior, no AdamW mutation at the skipped step, source-interval boundary handling, and finite checkpoints.

The new trainer is `tools/e2e/run_ah_warmup_supervision_ab_20261001.py`; the source-only helper is `tools/e2e/warmup_supervision_research_20261001.py`. Checkpoint schema is `portable_e2e.warmup_supervision_ab_20261001.v1`. Each arm writes `training/final_checkpoint.pt` and `training_receipt.json`. Its `fit.global_step=2999` denotes actual optimizer updates; `scheduled_steps=3000`, `samples_seen=11996`, `scheduled_samples=12000`, and `skipped_schedule_steps_one_based=[1693]` make the accounting explicit. `active_supervision_exposures` is 11996 for W0 and 11549 for W1. Top-level parent/final/initial tensor hashes, plan hash, warmup-mask hash, and exposure-order hash bind the candidate identity. Both completed checkpoints receive a single `both_final_checkpoints_complete.json` receipt before any known-DEV evaluation.

The prior two 3,000-step full-model arms took roughly 12 minutes of exclusive physical GPU0. Four per-row loss calls may increase this pair's runtime; report the measured 100-step throughput and update the estimate without changing the fixed budget. Fixed TRAIN probes and known-DEV evaluation add overhead. No GPU1, concurrent driving training, real-vehicle execution, automatic training-data admission, feature approval, or checkpoint promotion. Source implementation does not imply that execution has begun.
