# Matched appearance-only continuation proposal

<!-- HH_261001 - Proposed next study; no GPU launch, fitting or automatic admission here. -->

Keep the current B spatial-context architecture and the same exact B learned
weights in both arms. The initialization NPZ SHA256 is
ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e.
Do not change architecture while testing RGB exposure.

| Arm | Byte-identical original replay | New same-frame TRAIN-route inputs | Total pool |
| --- | --- | --- | ---: |
| Control | Original Town01+Town02 train_b 729 frames | All 240 Town01 p01 + 240 Town02 p00 RGB-false frames | 1,209 |
| Appearance | The exact same original 729 frames | The matched 480 RGB-true frames with identical labels/poses/order | 1,209 |

The original train_b NPZ remains immutable (SHA256
d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3).
The paired frames must pass the v2 owner, synchronization/hash/pose/semantic
and live/raw replay audit plus fixed full-FOV review. Make a new study-specific
TRAIN registration/package for these exact old TRAIN routes; do not edit their
diagnostic capture receipts or broad site registry. CTrack never enters either
pool. Town02 p02 and Town01 DEV remain outside both pools.

Proposed fixed optimization, declared before fitting:

- Same existing B weights, all parameters trainable, fresh identical AdamW
  states; learning rate 3e-5, weight decay 1e-4, batch 8, 10,000 steps each.
- Same seed, paired sample-ID sequence, batch order, crop coordinates and
  augmentation random draws. Class weights computed once from the common
  target arrays, therefore identical. No arm-specific RGB normalization.
- Every original example reappears each epoch/shuffle cycle; do not replace
  old samples with new ones or silently change the original replay fraction.
- All 480 new frame IDs remain paired, including late stationary Town01 frames;
  disclose their redundancy. No per-arm quality filter, score selection or
  deduplication that changes the 729+480 count.
- GPU0 only when parent schedules it; bounded separate outputs/source archives.
  No mid-run DEV or CTrack checkpoint selection. Save both fixed final weights
  before evaluation.

Evaluation after both endpoints are fixed: unchanged Town01 DEV118 and Town02
p02 internal holdout232, plus the existing frozen CTrack LEFT/STRAIGHT EVAL480.
Report all classes/support/confusion, old-replay retention, true-versus-false
exposure gains, regression costs and latency. No automatic promotion or new
performance thresholds selected after seeing results.

CTrack failure motivated this intervention, so its subsequent measurements
are exposed diagnostic reuse, not untouched independent certification. Paired
TRAIN metrics diagnose treatment effects but are not held-out quality proof.
The comparison estimates the composite RGB postprocess exposure intervention,
not individual lighting/gamma causes or pure map-geometry generalization.
