# Town02 p02 medium/large sign object QA — preregistration

<!-- HH_260906 - Freeze object-level QA before reading additional source frames or component outcomes. -->

This plan does **not** reopen the failed tiny-sign gate: Town02 p02 has **0/232** retained whole-frame 1–4 px sign scenes against the original ≥24 requirement. The new question is whether its *medium/large* CARLA tag-8 pixels can support a later TRAIN-only pilot. No data admission, model training, TEST access, or threshold change is authorized by this QA.

## Frozen source and denominator

- Owner: `storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1` (Epic, `Town02_Opt`, route SHA256 `cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810`).
- Retention: use **only** the 232 precomputed `retained_stems` in `../map_town02_p02_train_scout/summary.json`; do not choose frames based on their image content. This summary pins the source owner, raw frame count, retained mask, and an initial total of **58,074** tag-8 pixels. The script must recompute that total from the native semantic arrays and fail if it differs.
- Inputs are the owner’s 640×360 native RGB PNG and semantic NPZ for each retained stem. No resizing, model prediction, or recomputed vehicle-distance selection can redefine the denominator.

## Component inventory and review selection

1. Extract 8-connected components of tag 8 in **every** retained semantic frame. Record frame, pixel count, native bounding box, centroid, source SHA256, and total pixels. An all-frame count or a raw 8-connected component is **not** an object identity.
2. Sort components by pixel count descending, then frame number and bounding box ascending. The review queue is the shortest deterministic prefix covering **at least 95%** of all retained tag-8 pixels, plus any components needed to resolve visibly distinct signs in those frames. Unreviewed pixels stay in the denominator and cannot be called valid or invalid. If a selected component is too small/occluded to establish RGB correspondence, classify its pixels **uncertain**, not valid.
3. Publish native full-FOV RGB/semantic/overlay and 4× nearest-neighbor crops for each reviewed component in frame-indexed contact sheets. Keep source arrays and image hashes in a machine-readable inventory. Manual RGB correspondence must be checked against the original view, not only the enlargement; dilation around masks may help display but must not change any count.
4. Propose temporal tracks by frame order, nearby projected position and changing apparent size. A provisional association is **not** a distinct physical sign. For each track, inspect first/middle/nearest frames and any ambiguous overlap/split; require stable RGB sign appearance and separated image/world context to claim a distinct object. Do not count frame repetitions or two components of one sign as independent objects. If tracking cannot be certified from these sensors, mark distinct-object criterion **inconclusive**.
5. Label reviewed pixels `rgb_confirmed_sign`, `invalid_non_sign`, or `uncertain`; report reviewed/total coverage and `(invalid+uncertain)/reviewed` conservatively. A component with mixed or doubtful pixels is wholly `uncertain` unless a pixel-level boundary is manually and reproducibly recorded. This rule biases against accidental admission.

## Fixed gate and allowed conclusions

Medium/large TRAIN candidacy requires **all**: ≥95% of all retained tag-8 pixels visually inspected; ≤5% invalid/uncertain pixels **within reviewed**; ≥3 *different* RGB-confirmed physical signs; ≥24 retained frames with at least one RGB-confirmed **single sign component ≥5 px**. A connected component touching an image border or split across adjacent tag-8 blobs needs specific review. A frame with only total tag-8 ≥5 px but each sign component <5 px does not satisfy the 24-frame test. If tracking, component quality, or review coverage cannot be certified, the verdict is **inconclusive**; the source remains excluded. The original tiny-sign failure remains unchanged regardless of this verdict.

Finally, inspect native full-FOV RGB/semantic road-and-lane boundaries at start/middle/end retained frames and report any occlusion/alignment concern. This is a separate data-admission prerequisite, not a shortcut to qualify tag 8. Independent sign-capable DEV and a locked one-factor D0/D1 learning plan in `../map_next_training_design/README.md` remain required even if this QA passes. Town04/Town07/Town15 held-out and Town06/Town10HD reserved TEST are not opened.

## 2026-09-27 execution and evidence-format clarification

<!-- HH_260906 - Clarify the existing visualization after inspection without changing any frozen selection or gate. -->

The frozen denominator, 241-component queue, and four gate thresholds above were not changed. The originally published 31 component sheets contain a **320×180 downsized full-frame context** next to an exact 4× nearest-neighbor component crop; their heading's “native full FOV” wording was imprecise. A separate [113-frame native full-FOV evidence set](native_fov_evidence/manifest.json) therefore displays original **640×360 RGB, CARLA semantic palette, and tag-8 overlay** pixels without resizing. Its 38 panels and manifest link every selected frame to the source RGB/NPZ SHA256 values. These are deterministic scientific views of immutable capture pixels, not synthetic edits, model output, or extra denominator frames.

The byte-preserved [initial decisions](manual_review_decisions.json) cover sheets 01–03 and 19. The [append-only decision ledger](manual_review_addendum.json) records full original-sheet and native-frame inspection for sheets 04–18 and 20–31, including image-edge slivers and disconnected pole parts. The scorer was narrowly expanded to validate the declared original and added sheet sets separately, require no overwritten initial labels, bind the native frame ledger to all 113 selected frames, and then apply the **same fixed thresholds**. Passing this object-only gate is `PASS_NOT_ADMITTED`, not dataset admission or tiny-sign qualification. The separate road/lane sample and independent DEV prerequisite remain insufficient for admission.
