# Matched appearance endpoint evaluation, 20261001 v1

<!-- HH_261001 - Preregister before imported final endpoints or diagnostic predictions. -->

Evaluate both fixed 10,000-step matched appearance endpoints, control and
appearance, with the same MapContextRecovery architecture. Require the exact
study data/source/registry hashes, both complete final endpoint receipts,
common initialization, sample/crop hashes, and an explicitly supplied final
fit-summary SHA256. No intermediate checkpoint, new training, threshold
selection, DEV tuning or automatic promotion is permitted here.

Four fixed scenarios, every native frame retained:

| Scenario | Frames | RGB conditions | Interpretation |
| --- | --- | --- | --- |
| CTrack LEFT | 240 | true | Exposed external-map diagnostic; forensic raw QA |
| CTrack STRAIGHT | 240 | true | Exposed external-map diagnostic; full owner/replay QA |
| Town01 right-p01 | 240 | false and true | Exact TRAIN-frame resubstitution diagnostic |
| Town02 straight-p00 | 240 | false and true | Exact TRAIN-frame resubstitution diagnostic |

Total: 2,880 model/frame/condition predictions and 663,552,000 predicted pixels.
Use CPU, two PyTorch compute threads and two interop threads, batch one,
frame-major order, conditions false then true where available, control then
appearance. Input is native 640×360 RGB divided by255 only. No image resize,
crop, sample filtering, deduplication, augmentation, depth or pose model input.
Save every prediction and per-frame confusion. Report class pixel support,
confusion, IoU and appearance-minus-control deltas. If a GT class is absent,
its IoU is null; its false positives remain reported. No performance thresholds.
Report forward+argmax p50/p95 latency including cold first inference, without
an unloaded-machine or complete-pipeline 10Hz claim.

Only indices 0/119/239 get comparison PNGs. Preserve all source pixels in each
native full-FOV RGB/GT/control/appearance panel; paired sets show both RGB
conditions. Prior root native-input review receipts are hash-bound. Require
full raw QA replay before inference and raw/receipt/model/source hash checks
afterward. An independent verifier remaps raw targets, recounts every saved
prediction and checks every native panel pixel without rerunning inference.

CTrack remains outside fitting and EVAL-only, but its observed failure motivated
this intervention: this is exposed diagnostic reuse, not independent certification.
Camera configured geometry matches the historical TRAIN geometry; RGB postprocess,
map materials and source-editor versus packaged rendering differ. Do not retain
the earlier disproven claim that configured camera geometry differed.

LEFT's original failed-owner record is immutable. Its independently qualified
raw data are usable, but original live-map equality remains unavailable. Never
rename the old FAIL to PASS. Paired Town01/Town02 frames are now in this exact
study's TRAIN pool; their post-fit scores cannot support held-out generalization.
No broad data admission, CTrack training permission or model promotion follows.

The preregistration records unknown future endpoint weights through the frozen
study/fit-completion contract, not invented hashes. Each actual evaluation
records the imported final summary SHA and exact control/appearance NPZ hashes
before any inference. Historical sources and this preregistration remain unchanged.
