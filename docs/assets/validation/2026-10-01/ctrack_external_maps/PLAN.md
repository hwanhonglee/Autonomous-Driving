# Fixed map A/B external CTrack diagnosis

<!-- HH_261001 - Fixed before predictions; no fitting, threshold tuning, or promotion. -->

Evaluate the frozen 20261001 context-recovery A and B NPZ weights on every
one of 240 native 640x360 frames from each LEFT/STRAIGHT episode. No dedup,
subsampling, filtering, crop, resize, augmentation, normalization beyond /255,
retraining, checkpoint selection, or learned thresholds. CPU two threads,
batch one, A then B for each frame; include the first cold inference in timing.

Model input is RGB only. Apply the existing seven-class raw CARLA mapping:
1 road, 2 sidewalk, 24 lane_marking, 7 traffic_light_fixture, 8 traffic_sign,
6 pole; every other tag maps to other. Report the full confusion matrix,
ground-truth/predicted/true-positive/false-positive/false-negative/union pixel
counts and IoU per class. Ground-truth-absent class IoU is null, even when
false-positive predictions exist; those false-positive counts remain visible.
Report p50/p95 forward-plus-argmax latency, not end-to-end pipeline speed.

Save all 240 predictions from each model, all per-frame confusion counts,
and fixed-index 0/119/239 native full-FOV RGB/GT/A/B comparison PNGs. No scores
are used to choose images. Review those three raw RGB images before execution
and supply an explicit review note. Technical QA covers all 240 raw pairs;
three-image render review is not comprehensive certification.

LEFT requires the exact qualified forensic raw audit, leaving its old owner
FAIL and unavailable original live-map equality explicit. STRAIGHT requires
the v2 owner PASS, successful cleanup, source fingerprints and full live/raw
replay equality. Both require roles/hashes, complete raw sensor sync/pose QA,
and a current replay of the immutable QA contract before model inference.

These weights were learned from Town01/Town02 only, not CTrack. Historical
human CTrack RGB exposure and source repairs are disclosed. CTrack uses
postprocess=true whereas TRAIN used false; the camera rig differs too. This
tests combined source/render/rig/map-domain shift, not pure geometry
generalization, independent certification, HD map correctness, or live driving.

All data remain external_map_diagnostic_evaluation, train_eligible=false,
dataset_admitted=false, and independent_dev_approved=false. There are no
performance acceptance gates and no automatic model promotion. Any later
training use needs separate site/route registration and a new holdout boundary.
