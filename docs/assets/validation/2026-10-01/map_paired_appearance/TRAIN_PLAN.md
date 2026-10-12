# Matched appearance-only B continuation, 20261001 v1

<!-- HH_261001 - Parent-approved bounded study; register actual reviewed data before fitting. -->

Both arms initialize from the exact existing MapContextRecovery B NPZ
ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e.
Architecture and all starting learned tensors are identical. Use fresh AdamW
in both arms: 10,000 steps, batch8, learning rate3e-5, weight decay1e-4,
gradient norm clip5, seed20261001. No mixed precision or altered loss heads.

Each pool has exactly1,209 sample IDs: the byte-identical original729 replay,
then all240 Town01 right-p01 and all240 Town02 straight-p00 paired captures.
Control uses new RGB-false; appearance uses their same-frame RGB-true. Both
use the same shared semantic targets. No crop, dedup or frame filtering in
packaging. Explicit reviewed frames0/119/239 in both RGB conditions and both
actual v2 capture QAs are required before the study TRAIN registry is created.

Ship the same80,000 sample IDs and crop boxes to both arms. Shuffle complete
1,209-ID epochs, truncating only the final partial epoch; no original replay
sample is omitted from any complete epoch. Aligned crop augmentation matches
the prior50%-gate, uniform0.65–1.0 scale, bilinear RGB/nearest target policy.
Use the same boxes in both arms, not new RNG-dependent image decisions.
Compute one sqrt-frequency class-weight vector from the common1,209 targets.
No photometric augmentation, per-arm weights or extra architecture change.

This is one new study-specific TRAIN permission, not broad dataset admission.
The raw diagnostics retain train_eligible=false and are never overwritten.
CTrack, Town01 DEV and Town02 p02 are excluded from fitting. Future CTrack
measurements are exposed diagnostic reuse because its failure motivated the
intervention, not independent certification.

Train only on the parent's scheduled personal pro6000 physicalGPU0 and existing
py312 venv, under its cooperative lock. No install or alternate-GPU fallback.
One1,800-second pair budget, fresh output, fixed source/data/registry hashes.
Save both final checkpoints before opening known DEV/holdout image arrays.
No intermediate DEV, CTrack or per-epoch metric-based selection. Compare the
two fixed endpoints afterward on unchanged Town01 DEV118/Town02 p02 holdout232;
CTrack evaluation is a separately owned frozen-data diagnostic. No automatic
promotion or post-hoc performance gate.
