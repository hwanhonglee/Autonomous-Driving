# Frozen false/true postprocess diagnosis on existing TRAIN routes

<!-- HH_261001 - Registered before paired predictions; no fitting or checkpoint selection. -->

Use the two byte-pinned Town01 right-p01 and Town02 straight-p00 captures from
the paired appearance owner. All 240 frames per route are retained, including
late Town01 stationary frames. The same semantic/depth/pose targets serve both
RGB conditions; no resize, crop, per-arm sampling or denoising is allowed.

Evaluate frozen context-recovery A and B weights on both false and true RGB,
CPU two threads, batch one, frame-major false A/B then true A/B. This gives
960 model-frame predictions per route / 1,920 across the two routes. Save all
prediction arrays, confusion matrices, class pixel supports and per-frame
counts. Ground-truth-absent class IoU is null with false positives still counted.

Publish true-minus-false IoU deltas within each fixed model, with no performance
threshold, fit, checkpoint selection or promotion. Record forward-plus-argmax
latency including cold first inference; it is not a clean runtime benchmark.
Fixed indices 0/119/239 produce eight full-FOV native panels each: false/true
RGB, shared GT shown in each column, A false/true, and B false/true.

Require the full paired raw-data auditor PASS-equivalent QA_CONDITIONAL and
an explicit review note for both RGB conditions at those fixed indices before
predictions. All original source, owner, route, sensor and image hashes remain
unchanged. Source data and models are not automatically admitted/promoted.

This is a causal comparison of the composite CARLA postprocess switch on two
already-used TRAIN corridors, not individual gamma/lighting settings and not
unseen-map generalization. CTrack remains external diagnostic EVAL only.
No future fitting decision is implied by completing this diagnostic.
