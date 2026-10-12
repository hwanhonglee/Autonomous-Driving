# Fixed-frame learned BEV projection: feasibility and bounded next step

<!-- HH_261001 - Disclose the missing confidence prerequisite rather than silently changing the historical recipe. -->

## Existing recipe and exact missing prerequisite

The historical `map_learned_projection_bridge` uses the unchanged
`project_predicted_frame` and `SemanticEvidenceMap` implementations:
stride3, optical-axis depth strictly 0.1–40m, maximum softmax confidence at
least0.5, confidence-weighted evidence, other excluded, 0.2m cells, and
at most one maximum-confidence vote per cell/class/frame. Depth and pose
are CARLA oracle geometry. Oracle semantics enter only a separate reference.
Unseen cells remain unknown; there is no free-space or topology inference.

All four new evaluated captures retain co-registered 640×360 BGRA depth,
finite rigid `world_from_camera`, FOV90 and the reviewed fixed indices
0/119/239. Both new models' saved semantic masks and independent audits
exist. **Their NPZs contain only uint8 `prediction`**, not confidence or
logits. Argmax does not determine confidence. Therefore the confidence0.5
filter and weighted votes cannot be reproduced from the saved masks alone.
Filling confidence with1 or removing the filter would be a different recipe.
No such substitution has been executed.

## Exact-recipe option: separately authorize bounded confidence extraction

Use only the same frozen new control and photometric endpoints, same CPU2
backend, same full-native RGB preprocessing and fixed 0/119/239 inputs.
Four scenarios × three fixed positions comprise36 model/frame/condition
forwards:12 CTrack and24 paired-TRAIN. Do not train, augment, change weights,
capture again, or select by confidence/score. Require the newly computed
argmax to match every pixel of the already saved class mask before using
its maximum softmax probability. Keep the saved mask as the projection label.

Retain exact float32 confidence on the existing stride3 sample lattice
(120×214), not full dense probabilities; the projection never consults the
other confidence pixels. Keep original arrays/receipts immutable. The
sampled confidences need at most3,697,920 uncompressed payload bytes for
all36 forwards, before compression. Verify independently that sparse
confidence serialization reconstructs exactly the samples used by the
unmodified projection implementation. Do not round or quantize confidence.

Accumulate only0→119→239 within each capture and condition, never the other
237 frames. The common depth/pose oracle reference can be reused for both
RGB conditions, with provenance explicit. Preserve learned versus oracle
sources and all six static semantic classes; report road/marking/sidewalk
subsets separately without filtering points based on GT or world height.
There is no new floor-height threshold: these are XY-projected semantic
evidence cells, not independently validated ground surfaces.

Publish each final oracle/control/photometric sparse grid as compressed
data-only arrays; counts and confidence sums reproduce the existing grid
snapshot exactly. Render12 fixed comparison PNGs (one per scenario/position,
both conditions included for TRAIN) with identical views, ego at panel
center, explicit oracle depth/pose labels, and dark unknown areas. Reuse links
to the already saved full-FOV RGB comparison PNGs instead of duplicating RGB;
the new PNGs contain only equal-scale oracle/control/photometric BEV panels
(both RGB conditions for TRAIN). No interpolated coverage or unobserved-frame
fill. Keep a strict10MiB total output budget,
staging all bytes in memory before exclusive file creation; if the cap is
exceeded, fail without quietly omitting maps/frames or changing resolution.

Independent replay must use saved labels, sampled confidence and raw
depth/pose only, with zero inference. Verify every projected point/count,
every accumulated cell/class/frame-count/confidence sum, all artifact hashes,
unknown/provenance flags and ego coordinates. Report fixed-grid occupancy
overlap and dominant-label agreement as diagnostic wiring statistics, not
new pass thresholds or metric-map accuracy. All prior source and artifact
hashes are rechecked after execution.

## Scope and interpretation

This extends the historical fixed-frame bridge, not a240-frame map. It cannot
show complete-route coverage, independent localization/depth accuracy,
lane connectivity, HD-map completion or deployed autonomous driving. CTrack
remains exposed EVAL-only. LEFT's original failed owner and unavailable live
map equality remain. Paired TRAIN remains resubstitution. No new gate,
threshold tuning, hidden filtering, model selection or promotion is allowed.

If confidence-only re-inference is not authorized, stop at this proposal.
A separately registered hard-label/unit-vote study is possible, but must be
named as a new recipe and cannot claim historical confidence-filter equivalence.
