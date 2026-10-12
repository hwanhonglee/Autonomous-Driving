# Pure photometric endpoint evaluation, 20261001 v1

<!-- HH_261001 - Freeze all scenarios before importing or inspecting the new photometric endpoint or its results. -->

Evaluate both fixed 10,000-step endpoints from
`map_photometric_matched_20261001_v1`: **control** is the fresh no-augmentation
TRUE480 reference, and **photometric** is its same-data archived-photometric
candidate. Both share original729 + pairedTRUE480, the original B parent,
architecture, 80,000 scheduled exposures/crops, optimizer and class weights.
Never confuse this control with the preceding study's paired-FALSE control.
Require both final endpoint receipts, complete study/source/registry pins,
the exact expected photo trace, fresh-control reproduction of the preceding
appearance model's tensor-state hash, and the imported final summary SHA.

Evaluate exactly these four scenarios regardless of early results:

| Scenario | Frames | RGB conditions | Role |
| --- | --- | --- | --- |
| CTrack LEFT | 240 | true | Exposed external diagnostic; forensic raw QA |
| CTrack STRAIGHT | 240 | true | Exposed external diagnostic; full owner/replay QA |
| Town01 right-p01 | 240 | false, true | Exact TRAIN-frame resubstitution diagnostic |
| Town02 straight-p00 | 240 | false, true | Exact TRAIN-frame resubstitution diagnostic |

Use the same raw receipts, full-native 640×360 RGB/255, seven classes,
batch one, two Torch intraop/interop threads and two OpenBLAS threads.
No resize, crop, augmentation, filtering, deduplication, learned thresholds,
intermediate checkpoint selection, or new fitting. All 2,880 predictions
and 663,552,000 predicted pixels are retained. Save per-frame/aggregate
confusions, GT/predicted/TP/FP/FN/union counts, class IoU with absent-GT IoU
null and FP retained, and photometric-minus-control deltas.

Record full forward+argmax CPU p50/p95 including the cold first frame;
concurrent simulator/load makes latency diagnostic, not unloaded 10Hz proof.
Only fixed indices 0/119/239 get full-FOV RGB/GT/control/photometric panels,
with both RGB conditions for paired TRAIN. An independent verifier remaps
raw targets, recounts every saved prediction and checks every panel pixel.
No performance gate, automatic promotion, HD-map or route-completion claim.

Separately bind preceding appearance-study results and compare this fresh
control's predictions/metrics against that preceding **appearance** endpoint,
not its old control. Record exact predicted-label equality and per-class
metric deltas without rewriting old records. Candidate-minus-preceding-
appearance is supplementary; the primary matched comparison is candidate
minus this study's fresh no-augmentation control. Do not select frames or
checkpoints based on either comparison.

CTrack remains EVAL-only and excluded from all optimization. Its repeated
historical exposure motivated this bounded hypothesis: this is not pristine
holdout testing or independent certification. LEFT's original owner FAIL
remains unchanged, and its original live-map equality remains unavailable.
Camera configured geometry matches historical TRAIN, while RGB postprocess,
source/editor versus packaged rendering and map materials differ. Paired
TRAIN scores are resubstitution, not held-out generalization. Retain known
old-lighting DEV/holdout retention results separately; no tuning or selection
is permitted from those results or from CTrack.

New output directories and new preregistration only. Imported finals belong
under the new study package's `fit/`; no existing package, scientific source,
preregistration, historical FAILED receipt or earlier prediction is modified.
