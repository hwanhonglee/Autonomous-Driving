# Pure photometric matched A/B, 20261001 v1

<!-- HH_261001 - Two fresh equal fits; no CTrack training or automatic promotion. -->

Reuse the immutable original729+pairedTRUE480 training package by content hash,
without duplicating its430 MiB arrays. Both new arms use identical1,209 sample
IDs,80,000 exposures, the same crop boxes, target pixels, class weights, original
B starting weights,245,831-parameter MapContextRecovery, fresh AdamW,10,000steps,
batch8, seed20261001, LR3e-5, WD1e-4 and gradient clip5. No new capture, head,
RGB exposure mixture, extra optimization steps, depth or pose input.

Control is a fresh replay of the preceding appearance-input recipe: paired
TRUE480 with no photometric transform. Its final tensor-state digest must equal
the preceding identical appearance model. Retain the fresh checkpoint and
history even though old reference weights exist; do not skip this fit.

Candidate uses those same inputs. Only paired TRUE IDs729..1208 can receive
the unchanged archived `train_map_photometric_pilot.py::photometric` policy
after the shared crop: joint probability0.5, grayscale blend U[0,1), brightness
scale U[0.35,1), BT.601 coefficients .299/.587/.114, float32 arithmetic, clip
and round-to-nearest-even into uint8. Original729 crops stay unchanged. Targets
never enter the image transform and never change. No independent tuning of the
two coupled components is claimed. A separate PCG64 seed20261003 drives only
candidate paired TRUE calls, never sampling/crops or original729. Preregister
the expected application count, ID/gate trace and final RNG state.

The archived policy's earlier September29 pilot FAILED retention: Town05 lane
improved6.158pp but Town01 lane declined5.485pp, mainly far-field false positives.
That pilot used all729 scratch2000 training, not this pairedTRUE-only B-context
continuation. Reuse is a bounded hypothesis test, not proof of a successful
policy. No probability, range or seed is chosen from new CTrack scores.

Before new study registration, inspect all six fixed TRAIN-only TRUE inputs
at0/119/239 for both paired routes, their deterministic archived-policy draws,
and a declared conditional-ON low-brightness example. Keep every input and
target unchanged. Preview QA is input inspection, not learned-model selection
or independent certification. Preserve adverse historical receipts verbatim.

The new study-specific registry references the prior immutable package; it
does not alter prior study permissions or diagnostic roles. CTrack remains
EVAL-only, and p02/known DEV remain outside optimization. Both endpoints must
be sealed before known DEV118/holdout232 image arrays open. Preregistered full
CTrack and paired TRAIN diagnostics follow separately, retaining LEFT's original
FAIL/live-equality-unavailable qualification and all old-lighting retention losses.
No intermediate evaluation, seed selection, threshold search or model promotion.

Root launches only after review and completion of the existing W pair, using
the existing personal pro6000 py312 venv and leased physicalGPU0. No installs,
alternate GPU, overwrite, resume or shared source/config edits. The two-fit
training budget remains1,800seconds; final evaluation follows both saved fits.
