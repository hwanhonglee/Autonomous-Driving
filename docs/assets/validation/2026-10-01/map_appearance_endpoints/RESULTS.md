# Matched appearance: all fixed endpoint results

<!-- HH_261001 - Complete preregistered outputs; no model selection or promotion. -->

All four scenarios completed and independently recounted: 2,880 predictions, 663,552,000 pixels, and 72 native full-FOV panels. No frames were filtered or selected by performance.

Both models use the same B architecture, initialization, 10,000 steps, sample/crop exposure, labels, optimizer and common weights. The sole paired TRAIN-input factor is RGB postprocess=false (control) versus true (appearance).

CTrack is exposed external diagnostic reuse, not independent certification. Paired Town01/Town02 scores below are exact TRAIN-frame resubstitution, not held-out validation. LEFT's original owner FAIL and unavailable live-map equality remain unchanged.

Preregistration SHA: `dbd037ee055103e53b40d43e8aec57247386929bf990beac20cca6f414438616`. Final fit summary SHA: `463741ec65e9fce501bcb06dbaecc6c02d5b46bd566d6510ca0a7050b54e5253`.

All IoUs and deltas below are percentages/percentage points. A null IoU means no GT pixels; false-positive counts remain visible.

## ctrack_left

Role: `exposed_external_map_diagnostic`. QA: `QA_CONDITIONAL_FORENSIC_RAW`; original live-map equality: `None`.

### RGB postprocess=true

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 31.9943 | 89.7561 | +57.7618 | 27,055,306 | 11,333,098 | 2,071,362 |
| road | 44.2643 | 78.0431 | +33.7788 | 22,382,541 | 4,808,168 | 1,092,205 |
| sidewalk | 0.7803 | 29.1004 | +28.3201 | 4,861,116 | 26,085 | 3,493,499 |
| lane_marking | 3.1472 | 21.0916 | +17.9444 | 997,037 | 2,226 | 25,626 |
| traffic_light_fixture | null | null | null | 0 | 5,672 | 21,806 |
| traffic_sign | null | null | null | 0 | 13,412,818 | 220,743 |
| pole | null | null | null | 0 | 1,320,465 | 1,260,462 |

Fixed native panels (indices 0/119/239):

- [ctrack_left_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_left_20261001_v1/ctrack_left_000_full_fov.png)
- [ctrack_left_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_left_20261001_v1/ctrack_left_119_full_fov.png)
- [ctrack_left_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_left_20261001_v1/ctrack_left_239_full_fov.png)

## ctrack_straight

Role: `exposed_external_map_diagnostic`. QA: `QA_CONDITIONAL`; original live-map equality: `True`.

### RGB postprocess=true

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 32.9679 | 79.5563 | +46.5884 | 26,213,749 | 15,427,446 | 2,091,069 |
| road | 31.0642 | 27.6797 | -3.3845 | 25,287,084 | 4,433,236 | 2,179,864 |
| sidewalk | 2.5873 | 5.8061 | +3.2188 | 2,948,595 | 411,443 | 17,395,664 |
| lane_marking | 4.4452 | 26.9281 | +22.4830 | 846,572 | 1,880 | 37,251 |
| traffic_light_fixture | null | null | null | 0 | 8,625 | 175,245 |
| traffic_sign | null | null | null | 0 | 10,532,126 | 223,600 |
| pole | null | null | null | 0 | 1,395,969 | 1,653,070 |

Fixed native panels (indices 0/119/239):

- [ctrack_straight_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_straight_20261001_v1/ctrack_straight_000_full_fov.png)
- [ctrack_straight_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_straight_20261001_v1/ctrack_straight_119_full_fov.png)
- [ctrack_straight_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_straight_20261001_v1/ctrack_straight_239_full_fov.png)

## train_town01_p01

Role: `training_resubstitution_appearance_diagnostic`. QA: `QA_CONDITIONAL`; original live-map equality: `True`.

### RGB postprocess=false

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 99.5481 | 99.0110 | -0.5371 | 22,651,984 | 19,150 | 29,880 |
| road | 99.9466 | 99.9046 | -0.0420 | 22,755,983 | 3,448 | 3,593 |
| sidewalk | 99.3738 | 98.9620 | -0.4118 | 9,327,256 | 31,519 | 59,004 |
| lane_marking | 99.5635 | 99.0181 | -0.5454 | 151,934 | 630 | 1,440 |
| traffic_light_fixture | 78.0178 | 43.7543 | -34.2635 | 47,675 | 12,974 | 59,951 |
| traffic_sign | 93.2972 | 88.7930 | -4.5043 | 61,194 | 4,361 | 7,504 |
| pole | 83.1540 | 71.5957 | -11.5583 | 299,974 | 53,487 | 102,374 |

### RGB postprocess=true

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 36.0531 | 98.5675 | +62.5144 | 22,651,984 | 23,680,841 | 63,138 |
| road | 19.7635 | 99.5064 | +79.7429 | 22,755,983 | 6,081,159 | 30,150 |
| sidewalk | 0.3074 | 97.5955 | +97.2881 | 9,327,256 | 1,725,071 | 137,489 |
| lane_marking | 9.8011 | 83.2486 | +73.4475 | 151,934 | 2,396 | 27,681 |
| traffic_light_fixture | 0.0051 | 66.0385 | +66.0334 | 47,675 | 30,351 | 23,482 |
| traffic_sign | 2.5135 | 74.9898 | +72.4763 | 61,194 | 900,918 | 19,985 |
| pole | 7.8154 | 59.6753 | +51.8598 | 299,974 | 347,719 | 161,031 |

Fixed native panels (indices 0/119/239):

- [train_town01_p01_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town01_p01_20261001_v1/train_town01_p01_000_full_fov.png)
- [train_town01_p01_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town01_p01_20261001_v1/train_town01_p01_119_full_fov.png)
- [train_town01_p01_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town01_p01_20261001_v1/train_town01_p01_239_full_fov.png)

## train_town02_p00

Role: `training_resubstitution_appearance_diagnostic`. QA: `QA_CONDITIONAL`; original live-map equality: `True`.

### RGB postprocess=false

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 98.2814 | 97.7388 | -0.5426 | 22,955,590 | 60,673 | 80,683 |
| road | 99.9395 | 99.9009 | -0.0386 | 22,096,394 | 3,396 | 4,714 |
| sidewalk | 99.1336 | 98.7640 | -0.3696 | 9,504,223 | 40,141 | 67,469 |
| lane_marking | 99.6808 | 99.2432 | -0.4376 | 155,855 | 460 | 1,125 |
| traffic_light_fixture | 63.6465 | 55.3875 | -8.2590 | 49,748 | 26,209 | 34,836 |
| traffic_sign | 75.9603 | 61.5899 | -14.3704 | 16,675 | 5,114 | 9,943 |
| pole | 58.9102 | 51.3310 | -7.5792 | 517,515 | 293,012 | 373,072 |

### RGB postprocess=true

| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 30.7597 | 96.5336 | +65.7739 | 22,955,590 | 21,992,776 | 144,845 |
| road | 29.1866 | 99.5685 | +70.3819 | 22,096,394 | 5,926,071 | 22,778 |
| sidewalk | 0.0815 | 97.7251 | +97.6436 | 9,504,223 | 979,884 | 117,997 |
| lane_marking | 41.4160 | 83.9637 | +42.5478 | 155,855 | 5,672 | 27,941 |
| traffic_light_fixture | 0.0090 | 41.4634 | +41.4544 | 49,748 | 5,739 | 61,309 |
| traffic_sign | 0.1256 | 48.1721 | +48.0465 | 16,675 | 3,074,280 | 16,369 |
| pole | 2.3872 | 38.1186 | +35.7314 | 517,515 | 1,186,771 | 553,490 |

Fixed native panels (indices 0/119/239):

- [train_town02_p00_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town02_p00_20261001_v1/train_town02_p00_000_full_fov.png)
- [train_town02_p00_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town02_p00_20261001_v1/train_town02_p00_119_full_fov.png)
- [train_town02_p00_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_train_town02_p00_20261001_v1/train_town02_p00_239_full_fov.png)

## External road error direction

Complete-sequence raw-target confusion, not selected frames. The appearance model's STRAIGHT road loss is primarily road pixels becoming sidewalk.

| Scenario | Model | GT road pixels | Road predicted sidewalk | Fraction % |
| --- | --- | ---: | ---: | ---: |
| ctrack_left | control | 22,382,541 | 16,779 | 0.0750 |
| ctrack_left | appearance | 22,382,541 | 3,382,729 | 15.1132 |
| ctrack_straight | control | 25,287,084 | 326,852 | 1.2926 |
| ctrack_straight | appearance | 25,287,084 | 16,765,211 | 66.2995 |

## Known final-only DEV/holdout retention

These unchanged splits were evaluated after both endpoints were saved. Root separately verified all 161,280,000 prediction pixels; receipt: `known_split_independent_recount.json`. They are exposed known splits, not new independent certification. All seven class IoUs are lower for appearance than control on both splits.

| Split | Class | Control IoU % | Appearance IoU % | Delta pp |
| --- | --- | ---: | ---: | ---: |
| dev_town01 | other | 97.9035 | 96.9358 | -0.9677 |
| dev_town01 | road | 99.7318 | 99.5135 | -0.2183 |
| dev_town01 | sidewalk | 97.3805 | 95.3173 | -2.0632 |
| dev_town01 | lane_marking | 98.0247 | 95.8059 | -2.2188 |
| dev_town01 | traffic_light_fixture | 46.3725 | 45.9082 | -0.4643 |
| dev_town01 | traffic_sign | 7.4436 | 5.0419 | -2.4016 |
| dev_town01 | pole | 47.9829 | 38.7104 | -9.2725 |
| holdout_town02_p02 | other | 95.5279 | 93.6447 | -1.8832 |
| holdout_town02_p02 | road | 98.8295 | 98.6812 | -0.1483 |
| holdout_town02_p02 | sidewalk | 94.4316 | 93.3181 | -1.1136 |
| holdout_town02_p02 | lane_marking | 99.1155 | 97.7985 | -1.3170 |
| holdout_town02_p02 | traffic_light_fixture | 29.4722 | 19.3780 | -10.0942 |
| holdout_town02_p02 | traffic_sign | 77.5302 | 67.9850 | -9.5452 |
| holdout_town02_p02 | pole | 44.7221 | 39.3318 | -5.3903 |

## CPU2 diagnostic latency

Forward+argmax only, including first cold inference. Other local work may overlap; not an unloaded-runtime or complete-pipeline10Hz claim.

| Scenario | RGB | Control p95 ms | Appearance p95 ms |
| --- | --- | ---: | ---: |
| ctrack_left | true | 118.825 | 118.970 |
| ctrack_straight | true | 130.792 | 129.773 |
| train_town01_p01 | false | 120.218 | 124.264 |
| train_town01_p01 | true | 121.721 | 117.991 |
| train_town02_p00 | false | 133.733 | 134.532 |
| train_town02_p00 | true | 133.729 | 133.195 |

No promotion thresholds, adaptive checkpoint selection, new DEV tuning, CTrack TRAIN admission or broad dataset admission were performed.
