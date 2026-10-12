# CTrack sky-warning QA correction

<!-- HH_261001 - Freeze a source-specific validator correction, not model-score selection. -->

The immutable moving LEFT capture has 240 raw synchronized RGB/depth/semantic
pairs, 94.05 m travel, and zero collisions. The old yellow-top-32-pixel rule
mistook yellow tree foliage for the Unreal sky warning in 165 frames. The
original owner FAIL and collector FAILED receipts remain untouched.

V2 detects the specific warning using the real 512x8 MiniFont bitmap embedded
in the pinned Unreal asset and the exact two warning strings and screen
coordinates in SkyAtmosphere.usf. It requires at least three matched glyphs in
one shader line, foreground yellow salience >=40, foreground-minus-background
salience >=35, and normalized glyph correlation >=0.85. Whole-frame yellow
counts remain descriptive, not pass/fail. No RGB pixels are altered or hidden.

Calibration is exposed QA data: the known bad stationary v1 image, corrected
stationary v2 image, and every moving LEFT frame. Preliminary checks find 22
matched glyphs in the positive and zero in each of 241 negatives; maximum
color-qualified correlation over moving negatives is 0.411. Synthetic solid
yellow and displaced font are negative controls. These are not independent
general OCR validation or learned-map evaluation results. Occluded warnings
with fewer than three visible matching glyphs may escape; full-FOV review is
still required.

Forensic LEFT re-audit is restricted to the exact original owner and summary
SHA256 values. It reruns all raw-pair/hash/sensor-sync/pose/depth/route/collision
checks and replaces only the warning classifier. The collector aborted before
its final replay/map save, so original live-map equality is **unavailable**.
An independently rebuilt raw oracle map checks saved per-frame projection
counts but never substitutes for that missing live equality claim.

The separate straight v2 owner changes only the process-local warning detector,
archives its new source/font dependencies, and keeps full live/replay equality,
240 frames, all other thresholds, source protection and lifecycle checks.
Neither run is TRAIN eligible or automatically promoted. Both remain external
map diagnostics; lit postprocess=true RGB and the capture rig differ from
TRAIN, so future model comparison is not a pure map-geometry generalization test.
