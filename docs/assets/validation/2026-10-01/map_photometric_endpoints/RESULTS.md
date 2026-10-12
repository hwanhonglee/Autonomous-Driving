# Pure photometric matched endpoints: all fixed results

<!-- HH_261001 - Report all preregistered outputs without selection, retuning or promotion. -->

All four scenarios independently recounted: 2,880 predictions, 663,552,000 pixels and 72 native full-FOV panels. No frames filtered.

Both fresh models use original729 + pairedTRUE480, the original B parent, architecture, 10,000 steps and identical sample/crop exposure. Only the candidate applies the archived photometric policy to TRUE480 during fitting. The reference is NOT the preceding FALSE-RGB control.

CTrack remains exposed EVAL-only diagnostic reuse. Paired TRAIN is resubstitution, not held-out validation. LEFT original owner FAIL and unavailable live-map equality remain unchanged.

Preregistration: `7019e192782a05eb4cfc2066a43c01ef02c1e8c32a5573341355a090e91e2711`. Imported final fit: `27f2951c6a62da6c4ee76ccd1dae24350afaf85cc1b41824ae50dbeecada6c6d`.

IoU values are percentages; deltas are percentage points. Absent-GT classes have null IoU, with false positives retained.

## ctrack_left

Role: `exposed_external_map_diagnostic`. QA `QA_CONDITIONAL_FORENSIC_RAW`; original live equality `None`.

### RGB postprocess=true

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 89.7561 | 91.9294 | 2.1733 | 27,055,306 | 2,071,362 | 1,503,477 |
| road | 78.0431 | 88.6368 | 10.5938 | 22,382,541 | 1,092,205 | 1,234,202 |
| sidewalk | 29.1004 | 51.1359 | 22.0355 | 4,861,116 | 3,493,499 | 1,175,341 |
| lane_marking | 21.0916 | 27.2760 | 6.1844 | 997,037 | 25,626 | 12,689 |
| traffic_light_fixture | null | null | null | 0 | 21,806 | 69,251 |
| traffic_sign | null | null | null | 0 | 220,743 | 39,108 |
| pole | null | null | null | 0 | 1,260,462 | 712,676 |

CPU2 forward+argmax p95: control 130.833 ms; photometric 130.961 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

Fixed full-FOV panels (0/119/239):

- [ctrack_left_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_left_20261001_v1/ctrack_left_000_full_fov.png)
- [ctrack_left_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_left_20261001_v1/ctrack_left_119_full_fov.png)
- [ctrack_left_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_left_20261001_v1/ctrack_left_239_full_fov.png)

## ctrack_straight

Role: `exposed_external_map_diagnostic`. QA `QA_CONDITIONAL`; original live equality `True`.

### RGB postprocess=true

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 79.5563 | 76.8270 | -2.7293 | 26,213,749 | 2,091,069 | 5,000,666 |
| road | 27.6797 | 41.0107 | 13.3310 | 25,287,084 | 2,179,864 | 1,320,968 |
| sidewalk | 5.8061 | 6.7858 | 0.9797 | 2,948,595 | 17,395,664 | 11,546,426 |
| lane_marking | 26.9281 | 38.2038 | 11.2757 | 846,572 | 37,251 | 34,277 |
| traffic_light_fixture | null | null | null | 0 | 175,245 | 64,975 |
| traffic_sign | null | null | null | 0 | 223,600 | 141,125 |
| pole | null | null | null | 0 | 1,653,070 | 974,203 |

CPU2 forward+argmax p95: control 130.279 ms; photometric 130.352 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

Fixed full-FOV panels (0/119/239):

- [ctrack_straight_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_straight_20261001_v1/ctrack_straight_000_full_fov.png)
- [ctrack_straight_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_straight_20261001_v1/ctrack_straight_119_full_fov.png)
- [ctrack_straight_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_ctrack_straight_20261001_v1/ctrack_straight_239_full_fov.png)

## train_town01_p01

Role: `training_resubstitution_appearance_diagnostic`. QA `QA_CONDITIONAL`; original live equality `True`.

### RGB postprocess=false

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 99.0110 | 99.2629 | 0.2519 | 22,651,984 | 29,880 | 34,614 |
| road | 99.9046 | 99.8888 | -0.0158 | 22,755,983 | 3,593 | 3,774 |
| sidewalk | 98.9620 | 98.8638 | -0.0982 | 9,327,256 | 59,004 | 70,038 |
| lane_marking | 99.0181 | 99.0284 | 0.0103 | 151,934 | 1,440 | 1,415 |
| traffic_light_fixture | 43.7543 | 72.6823 | 28.9280 | 47,675 | 59,951 | 16,419 |
| traffic_sign | 88.7930 | 89.9180 | 1.1250 | 61,194 | 7,504 | 6,511 |
| pole | 71.5957 | 75.4634 | 3.8677 | 299,974 | 102,374 | 75,910 |

CPU2 forward+argmax p95: control 136.678 ms; photometric 136.159 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

### RGB postprocess=true

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 98.5675 | 98.5808 | 0.0133 | 22,651,984 | 63,138 | 106,814 |
| road | 99.5064 | 99.3882 | -0.1182 | 22,755,983 | 30,150 | 25,187 |
| sidewalk | 97.5955 | 97.2037 | -0.3919 | 9,327,256 | 137,489 | 157,058 |
| lane_marking | 83.2486 | 82.0980 | -1.1506 | 151,934 | 27,681 | 29,711 |
| traffic_light_fixture | 66.0385 | 65.8933 | -0.1452 | 47,675 | 23,482 | 22,751 |
| traffic_sign | 74.9898 | 70.7932 | -4.1967 | 61,194 | 19,985 | 24,892 |
| pole | 59.6753 | 60.6719 | 0.9966 | 299,974 | 161,031 | 121,002 |

CPU2 forward+argmax p95: control 135.894 ms; photometric 136.711 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

Fixed full-FOV panels (0/119/239):

- [train_town01_p01_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town01_p01_20261001_v1/train_town01_p01_000_full_fov.png)
- [train_town01_p01_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town01_p01_20261001_v1/train_town01_p01_119_full_fov.png)
- [train_town01_p01_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town01_p01_20261001_v1/train_town01_p01_239_full_fov.png)

## train_town02_p00

Role: `training_resubstitution_appearance_diagnostic`. QA `QA_CONDITIONAL`; original live equality `True`.

### RGB postprocess=false

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 97.7388 | 97.9384 | 0.1996 | 22,955,590 | 80,683 | 97,824 |
| road | 99.9009 | 99.8871 | -0.0138 | 22,096,394 | 4,714 | 5,898 |
| sidewalk | 98.7640 | 98.7066 | -0.0575 | 9,504,223 | 67,469 | 70,320 |
| lane_marking | 99.2432 | 99.1851 | -0.0581 | 155,855 | 1,125 | 1,226 |
| traffic_light_fixture | 55.3875 | 57.9387 | 2.5512 | 49,748 | 34,836 | 29,125 |
| traffic_sign | 61.5899 | 65.7228 | 4.1329 | 16,675 | 9,943 | 8,047 |
| pole | 51.3310 | 53.5326 | 2.2016 | 517,515 | 373,072 | 314,311 |

CPU2 forward+argmax p95: control 137.291 ms; photometric 136.379 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

### RGB postprocess=true

| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| other | 96.5336 | 97.1026 | 0.5690 | 22,955,590 | 144,845 | 213,381 |
| road | 99.5685 | 99.4861 | -0.0824 | 22,096,394 | 22,778 | 19,193 |
| sidewalk | 97.7251 | 97.4115 | -0.3136 | 9,504,223 | 117,997 | 132,765 |
| lane_marking | 83.9637 | 84.8373 | 0.8735 | 155,855 | 27,941 | 25,247 |
| traffic_light_fixture | 41.4634 | 47.5576 | 6.0942 | 49,748 | 61,309 | 41,291 |
| traffic_sign | 48.1721 | 45.5727 | -2.5995 | 16,675 | 16,369 | 16,799 |
| pole | 38.1186 | 40.5004 | 2.3818 | 517,515 | 553,490 | 383,193 |

CPU2 forward+argmax p95: control 136.161 ms; photometric 136.769 ms. Cold first included; not unloaded or end-to-end 10Hz proof.

Separate preceding appearance reference: fresh control agrees on 55,296,000/55,296,000 predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.

Fixed full-FOV panels (0/119/239):

- [train_town02_p00_000_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town02_p00_20261001_v1/train_town02_p00_000_full_fov.png)
- [train_town02_p00_119_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town02_p00_20261001_v1/train_town02_p00_119_full_fov.png)
- [train_town02_p00_239_full_fov.png](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_photometric_endpoints_train_town02_p00_20261001_v1/train_town02_p00_239_full_fov.png)

## External road-to-sidewalk errors

| Scenario | Model | GT road | Road predicted sidewalk | Fraction % |
| --- | --- | ---: | ---: | ---: |
| ctrack_left | control | 22,382,541 | 3,382,729 | 15.1132 |
| ctrack_left | photometric | 22,382,541 | 1,072,091 | 4.7899 |
| ctrack_straight | control | 25,287,084 | 16,765,211 | 66.2995 |
| ctrack_straight | photometric | 25,287,084 | 10,966,947 | 43.3698 |

## Known old-lighting retention

Final-only known DEV/holdout, independently recounted by the parent: 161,280,000 additional pixels. No intermediate selection; known/exposed splits are not independent certification.

| Split | Class | Fresh TRUE control % | Photometric % | Delta pp |
| --- | --- | ---: | ---: | ---: |
| dev_town01 | other | 96.9358 | 97.2339 | 0.2980 |
| dev_town01 | road | 99.5135 | 99.4569 | -0.0566 |
| dev_town01 | sidewalk | 95.3173 | 94.5485 | -0.7688 |
| dev_town01 | lane_marking | 95.8059 | 96.6471 | 0.8412 |
| dev_town01 | traffic_light_fixture | 45.9082 | 48.1612 | 2.2530 |
| dev_town01 | traffic_sign | 5.0419 | 6.9906 | 1.9487 |
| dev_town01 | pole | 38.7104 | 42.5182 | 3.8078 |
| holdout_town02_p02 | other | 93.6447 | 94.4913 | 0.8467 |
| holdout_town02_p02 | road | 98.6812 | 98.5160 | -0.1652 |
| holdout_town02_p02 | sidewalk | 93.3181 | 92.5076 | -0.8105 |
| holdout_town02_p02 | lane_marking | 97.7985 | 96.8308 | -0.9677 |
| holdout_town02_p02 | traffic_light_fixture | 19.3780 | 24.8893 | 5.5113 |
| holdout_town02_p02 | traffic_sign | 67.9850 | 69.6796 | 1.6946 |
| holdout_town02_p02 | pole | 39.3318 | 42.9046 | 3.5727 |

No promotion, CTrack training admission, HD-map completion or closed-loop driving claim follows.
