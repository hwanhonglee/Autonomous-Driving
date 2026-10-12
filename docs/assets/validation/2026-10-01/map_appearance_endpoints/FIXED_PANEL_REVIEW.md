# Fixed native panel review

<!-- HH_261001 - Assistant visual inspection, not independent human certification. -->

The review uses only preregistered indices 0/119/239, without frame selection
from scores. Each PNG includes the original native RGB, raw semantic target,
and both fixed model predictions. Independent recount receipts verify every
panel pixel against the corresponding original array. This is an assistant
visual interpretation, not a new human quality approval or deployment gate.

## CTrack LEFT

All three fixed panels were inspected. The appearance model markedly reduces
the control model's sign-class predictions over foliage. The centerline is
better recovered at 119/239, and the road is more continuous. At 000, lane paint
is still largely missed. Curb/white-edge confusion, road-to-sidewalk patches,
and spurious pole/light-like regions remain. Across the complete sequence,
15.11% of GT road pixels are predicted as sidewalk by appearance.

The original owner FAIL remains. This observation uses forensic-qualified raw
RGB/targets and does not restore unavailable original live-map equality.

## CTrack STRAIGHT

All three fixed panels were inspected. Appearance better identifies the yellow
centerline and suppresses much of the control model's foliage-as-sign error.
However, 000/119 show extensive road-to-sidewalk fragmentation. Frame 239 has a more
continuous lower road region but still misses much of the white edge and marks
tree structures as poles. In the complete sequence, 66.30% of GT road pixels
become sidewalk under appearance. This explains why centerline improvement
does not imply an overall road recovery.

The RGB has substantial tree shadows. A shadow/material sensitivity hypothesis
is plausible, but the fixed panels do not isolate shadow causality from other
material, texture, or contextual differences.

## Paired TRAIN Town01 right-p01

All three false/true paired panels were inspected. Appearance recovers the
major road/sidewalk/centerline layout on true RGB, while control largely fails
on those lit inputs. On false RGB both keep the main layout, but appearance
introduces additional fixture/pole-like artifacts, especially at 000. Glare and
washed highlights remain present in the raw true images; no image was repaired
or filtered. These are exact TRAIN frames, so the good true-RGB fit is
resubstitution evidence, not held-out generalization.

## Paired TRAIN Town02 straight-p00

All three false/true paired panels were inspected. Appearance largely restores
the lit road, sidewalks and yellow centerline where control fails. Both models
retain strong coarse geometry on false RGB, but appearance adds fixture/pole
errors. The true inputs at 000/239 contain strong building-cast shadows that
appearance handles substantially better than the CTrack straight road. Thus
the CTrack failure cannot be attributed simply to an inability to handle any
shadow; material/texture, road appearance and marking coverage remain plausible
contributors. These are also exact TRAIN frames, not generalization evidence.

All 12 preregistered comparison PNGs have now been visually inspected. No new
frames were selected, modified or removed based on these observations.
