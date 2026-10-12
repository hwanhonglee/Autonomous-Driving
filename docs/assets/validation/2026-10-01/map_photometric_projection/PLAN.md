# Authorized fixed-frame confidence recovery and BEV replay

<!-- HH_261001 - Freeze unchanged projection parameters before confidence recovery or new BEV outputs. -->

The parent explicitly authorized36 read-only CPU forwards to recover missing
confidence, with zero fitting and no simulator/GPU/installation. Follow the
unchanged-recipe option in PROPOSAL.md: same four scenarios, models, native
RGB inputs and fixed0/119/239, and require every recovered argmax pixel to
match the previously saved prediction. Keep both logits-argmax and softmax-
argmax identity checks. No confidence inference existed when registering.

Use the original mapping/projection source hashes, all six static labels,
stride3, strict0.1–40m depth, confidence>=0.5, per-cell/class/frame maximum
confidence votes, 0.2m grid, and200000-cell capacity. CARLA depth/pose remain
oracle geometry; oracle semantics populate only the separate reference.
No height/floor gate, label tuning, geometric correction or interpolation.
Only the three fixed observations accumulate;237 skipped frames stay skipped.

Retain exact stride3 float32 confidence, final sparse grids with counts,
float64 confidence sums and insertion order (including ties), and12 equal-
scale ego-centered BEV-only PNGs. Link to existing RGB panels without copying
them. Every output byte is staged in memory and the complete result plus
reserved replay-receipt space must fit10MiB before exclusive creation.
Failure never changes thresholds, resolution or selected frames to fit.

Freeze full artifact/source identities before execution. Replay independently
from raw depth/pose plus saved masks/confidence, with zero inference. Compare
every projected point hash, intermediate count, final sparse cell payload,
dominant-label tie order and rendered PNG pixel. Verify ego-center markers
and unknown backgrounds. Report occupancy/common-cell agreement plus six-
class cell supports and confusion on the union; unknown is not free space.
These are descriptive oracle-assisted plumbing measures, not approval gates.

No CTrack TRAIN, independent certification,240-frame mapping, route coverage,
HD-map, deployed geometry or driving-success claim. Preserve LEFT's original
owner FAIL/unavailable live equality and TRAIN resubstitution distinctions.
