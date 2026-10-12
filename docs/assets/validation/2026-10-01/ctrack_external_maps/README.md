# Frozen map models fail on the corrected CTrack source

<!-- HH_261001 - Report external failure and correct rig provenance, without changing preregistered history. -->

Both models were evaluated on all 240 frames of each route, with no fitting,
filtering or model selection. B improves road IoU, but both miss most lane
markings and sidewalks. The earlier known-route improvement does **not**
establish external-map generalization.

| IoU | LEFT A | LEFT B | STRAIGHT A | STRAIGHT B |
| --- | ---: | ---: | ---: | ---: |
| Road | 8.24% | 24.44% | 15.85% | 17.81% |
| Lane marking | 0.095% | 0.065% | 3.12% | 1.59% |
| Sidewalk | 1.66% | 0.75% | 3.06% | 2.89% |
| Other | 46.59% | 34.59% | 44.85% | 32.76% |

No ground-truth traffic-light fixtures, traffic signs or poles occur in either
sequence; their IoUs are null, not proof of success. False positives remain
visible in the pixel counts. For example, B predicts traffic-sign class on
8,604,760 LEFT pixels and 7,433,930 STRAIGHT pixels despite zero positive GT.

## Important provenance correction

The frozen plan/results initially repeated an unsupported claim that the
camera rig differs from TRAIN. [Archived-source verification](camera_rig_correction.json)
supersedes that claim: every Town01 TRAIN origin and Town02 p00 used the same
configured 640x360/FOV90, zero lens distortion, x=1.5 m/z=2 m/pitch=-12 degree
camera as these CTrack runs. Historical raw data do not provide today's complete
per-sensor runtime metadata, so this is configured-geometry equality.

The confirmed rendering difference is TRAIN postprocess=false versus CTrack
true. Map/mesh/material style and packaged versus source-editor runtime also
differ. Their causal contributions have not been isolated. The immutable
preregistration and result receipts are retained rather than silently edited.

## Audit and limitations

- [Preregistration](preregistration.json) preceded all predictions; [fixed plan](PLAN.md).
- 960 model-frame predictions / 221,184,000 predicted pixels were saved and
  recounted exactly, including all per-frame and aggregate confusion matrices.
- Native pixels in all 24 comparison panels match their raw RGB/GT/predictions.
- LEFT is [forensic raw recovery](../ctrack_isolation/left_forensic_raw_v2.json):
  old owner FAIL is preserved and original live-map equality remains unavailable.
- STRAIGHT has [complete v2 owner/raw QA](../ctrack_isolation/straight_raw_v2.json),
  including live/replay equality and clean lifecycle.
- CPU two-thread p95 A/B: LEFT 109.4/130.8 ms; STRAIGHT 115.0/130.9 ms. This is
  diagnostic forward/argmax timing, potentially co-loaded, not pipeline 10Hz proof.
- CTrack stays external diagnostic EVAL only, outside TRAIN. Historical human
  RGB exposure/source repair is disclosed; no independent certification or promotion.

| Route | Full-FOV first | Fixed middle | Last | Raw result / audit |
| --- | --- | --- | --- | --- |
| LEFT | [0](../../../../../storage/experiments/validation/2026-10-01/ctrack_left_fixed_maps_20261001_v1/left_000_full_fov.png) | [119](../../../../../storage/experiments/validation/2026-10-01/ctrack_left_fixed_maps_20261001_v1/left_119_full_fov.png) | [239](../../../../../storage/experiments/validation/2026-10-01/ctrack_left_fixed_maps_20261001_v1/left_239_full_fov.png) | [metrics](../../../../../storage/experiments/validation/2026-10-01/ctrack_left_fixed_maps_20261001_v1/summary.json), [recount](../../../../../storage/experiments/validation/2026-10-01/ctrack_left_fixed_maps_20261001_v1/prediction_audit.json) |
| STRAIGHT | [0](../../../../../storage/experiments/validation/2026-10-01/ctrack_straight_fixed_maps_20261001_v1/straight_000_full_fov.png) | [119](../../../../../storage/experiments/validation/2026-10-01/ctrack_straight_fixed_maps_20261001_v1/straight_119_full_fov.png) | [239](../../../../../storage/experiments/validation/2026-10-01/ctrack_straight_fixed_maps_20261001_v1/straight_239_full_fov.png) | [metrics](../../../../../storage/experiments/validation/2026-10-01/ctrack_straight_fixed_maps_20261001_v1/summary.json), [recount](../../../../../storage/experiments/validation/2026-10-01/ctrack_straight_fixed_maps_20261001_v1/prediction_audit.json) |

Next controlled experiment: paired postprocess=false/true RGB sensors on the
same tick and mount in already-used Town01/Town02 TRAIN corridors. Shared raw
semantic/depth/pose labels allow appearance comparison without adding CTrack
to TRAIN. See [paired capture plan](PAIRED_APPEARANCE_PLAN.md).
