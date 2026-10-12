# Same-frame RGB appearance control on existing TRAIN routes

<!-- HH_261001 - Prepare the next bounded causal diagnostic, not CTrack training. -->

Use the existing packaged runtime and exactly two already-used TRAIN routes:

1. Town01 right p01, route SHA256
   df68ac6046064faefe5d6a19425e0b038e94d2171878741f5c713a35ec0ce8de.
2. Town02_Opt straight p00, route SHA256
   520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b.

Do not use Town02 p02 (internal holdout), Town01 DEV, CTrack EVAL, or sealed
sites. Keep current datasets, splits, weights and original captures immutable.

Version a new collector/owner rather than edit the frozen historical collector.
One physical Prius BasicAgent drive per route, 240 frames at 10 Hz, target
15 kph, the same reviewed route bytes/traffic policy, ClearNoon and Epic.
Use four synchronized cameras attached at x=1.5 m/y=0/z=2 m, pitch=-12,
640x360/FOV90/lens_k=0/lens_kcube=0:

- RGB false: explicit enable_postprocess_effects=false (historical condition).
- RGB true: same attributes except enable_postprocess_effects=true.
- Raw semantic and depth: postprocess=false, shared targets for both RGB arms.

Record all effective blueprint attributes, each sensor frame/timestamp/pose,
raw BGRA digests, PNG/NPZ hashes, ego state, control and all route positions.
Reject mismatched frame IDs/timestamps/poses, missing frames, collisions,
source/owner identity mismatch, cleanup failure, or failed live/raw map replay.
Do not lower other QA thresholds. Save the two full RGB sequences without
masking warnings or replacing appearance. Adding the second RGB sensor changes
load, so capture latency is not an isolated sensor-performance benchmark.

First use is a fixed-model diagnostic, not automatic training admission:
evaluate both frozen A/B models on every false/true pair with the same target,
publish pixel support/IoU/confusion for all classes, and fixed first/middle/last
full-FOV comparisons. This isolates the composite CARLA postprocessing switch
on familiar routes, not individual tonemapping/lighting effects and not map
generalization. It cannot by itself prove why CTrack fails.

Only after source QA and paired diagnosis may an explicitly registered
Town01/Town02 TRAIN appearance augmentation be prepared; CTrack remains EVAL
and no model or split is automatically promoted. Existing owner lifecycle and
process/port ownership checks must cover the new four-camera collector before
the parent runs it. No simulator was launched by this plan.

## Prepared implementation and bounded route extent

`tools/e2e/run_owned_map_paired_rgb_20261001.sh` defaults to read-only preflight;
`--execute` is required. It owns the packaged server and collector process groups,
retains source/BasicAgent archives, checks all shutdown receipts, and invokes
`collect_map_paired_rgb_20261001.py`. Both route preflights and six dedicated
tests pass. The separate auditor is `audit_map_paired_rgb_20261001.py`.

Town01 right p01 is 79.78 m long; its historical 240-frame capture traveled
79.10 m, so late stationary frames are expected and retained. Town02 p00 is
177.13 m long, but the existing TRAIN capture used only a 94.30 m prefix; its
junction window starts at 95.46 m. The new 240-frame experiment does not claim
full-route or junction completion, and does not silently extend the prefix.

Fresh suggested owner outputs are
`storage/experiments/validation/2026-10-01/map_paired_town01_p01_20261001_v1`
and `map_paired_town02_p00_20261001_v1` in the same parent. Supply each exact
route path recorded in its historical owner plan, `--port 2400 --execute`.
Four-camera data appear in the owner's `mapping` directory. Audit after complete
shutdown with `--source <owner>/mapping --output <fresh-report.json>`.

No subsequent map fit is authorized by this script. The next fit design must
hold architecture, starting weights, sample count, sample order, optimizer,
step budget and original-data replay fixed, changing only RGB postprocess
exposure between matched arms. Do not simultaneously change the architecture.
The current fixed A/B comparison is a diagnosis, not that future causal fit.
