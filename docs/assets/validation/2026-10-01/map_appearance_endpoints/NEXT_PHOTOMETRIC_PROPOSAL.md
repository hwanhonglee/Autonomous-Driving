# Next bounded map cycle: pure photometric A/B

<!-- HH_261001 - Proposal only; no new fit, capture, source edit or registration executed. -->

Recommendation: one controlled pure-photometric experiment on the existing
immutable TRAIN bytes is plausible, but not a promised fix. Do not change the
architecture, add a head, increase steps, or train on CTrack.

## Evidence and limits

The appearance endpoint fits paired TRAIN-true road at99.51%/99.57% IoU and
sidewalk at97.60%/97.73%, yet CTrack STRAIGHT road is27.68% and66.30% of its GT
road pixels become sidewalk. CTrack LEFT road improves to78.04%, but lane IoU
remains21.09% (STRAIGHT26.93%). Known old-lighting DEV/holdout lose IoU in all
seven classes; fixture/pole retention is particularly weak. This is a
representation/generalization tradeoff, not evidence that another head or
more identical steps is needed.

All fixed0/119/239 panels were inspected. Town02 TRAIN-true000/239 already have
strong building shadows, which appearance handles well. Therefore the CTrack
failure is not proven to be generic shadow blindness. Color/intensity shortcuts
are plausible; material texture, contextual layout and white-edge marking
coverage remain competing explanations. Global photometric augmentation can
test the first hypothesis; it cannot create new meshes, recover clipped paint,
or certify robustness to all other changes.

## Single-factor design

- Both arms use exactly the existing729 original replay plus480 paired TRUE
  frames, unchanged targets, original B initialization, same245,831-parameter
  MapContextRecovery, common class weights,10,000steps, batch8, LR3e-5, WD1e-4,
  fresh equal AdamW, identical serialized sample IDs and crop boxes.
- Reference: the current appearance-input recipe with no photometric transform.
  Candidate: that same recipe plus one fixed pure-photometric policy on the
  new TRUE480 samples only. Original729 replay stays pixel-identical in both
  arms. Do not simultaneously switch false/true exposure mixing or architecture.
- Prefer reusing the preexisting archived photometric pilot policy rather than
  selecting strength from CTrack scores: joint gate0.5, grayscale blend
  uniform[0,1], brightness scale uniform[0.35,1], rounding/clipping to uint8.
  It is defined in the already archived `train_map_photometric_pilot.py`.
  Its seed stream must be separate from shared sampling/crops; labels never
  enter the RGB transform and do not change.
- Before registration, inspect deterministic transformed TRAIN-only fixed
  examples and test no mutation, target equality, finite values and RNG
  independence. If the policy is unsuitable, disclose that before any fitting;
  do not tune its strength using CTrack labels or checkpoint scores.
- Create a new study-only permission referencing the immutable package. Raw
  diagnostics and the current study registry remain untouched. No new site
  admission and no CTrack TRAIN use.

Both endpoints must finish before the same final-only known-split and external
diagnostics. Repeat all four full sequences and independently recount every
prediction, including all-class old-lighting retention. CTrack is now further
exposed diagnostic reuse, never an independent development gate. No adaptive
checkpoint/seed/threshold selection and no automatic promotion.

This is proposed for root approval, not launched. If photometric invariance
does not reduce the road-to-sidewalk error without unacceptable retention loss,
the next evidence-gathering step is TRAIN-only coverage of lit asphalt/shadows,
white markings, and vegetation-versus-pole textures on already registered TRAIN
routes. Audit exact route boundaries and content first; do not assume a route
contains those features or repurpose CTrack/p02 holdout. More compute on the
same narrow appearances is not the default next move.
