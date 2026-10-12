# Fixed geometry-source endpoints: all results

<!-- HH_261001 - Preserve all scenarios, regressions and historical distinctions. -->

Both fresh control/geometry endpoints completed10,000 steps. The only paired intervention is old Town01 FALSE replay versus new p02/p04 FALSE route sources in the matched extra480 slots. Common TRUE augmentation, parent, model, class weights, crop/sample budget and losses remain fixed.

**All four fixed diagnostics independently recounted:**2,880 predictions,663,552,000 pixels,72 native full-FOV subpanels. No frame filtering. CTrack is exposed EVAL only; paired TRAIN is resubstitution. LEFT original owner FAIL and unavailable original live equality remain. This is not independent certification, mapping completion or model promotion.

IoU values below are percentages; deltas are percentage points. Null denotes an absent GT class; false positives remain counted. Historical photo comparisons are retained separately in `RESULTS.json` and are not the new paired control.

## ctrack_left

Role: `exposed_external_map_diagnostic`; source QA `QA_CONDITIONAL_FORENSIC_RAW`.

### RGB postprocess=true

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 92.9980 | 92.8328 | -0.1653 | 27,055,306 | 1,210,360 | 868,911 |
| road | 88.7866 | 89.0940 | 0.3074 | 22,382,541 | 1,692,636 | 2,300,524 |
| sidewalk | 48.5072 | 48.9423 | 0.4351 | 4,861,116 | 849,791 | 398,101 |
| lane_marking | 26.9345 | 28.0283 | 1.0938 | 997,037 | 10,050 | 22,457 |
| traffic_light_fixture | null | null | null | 0 | 66,350 | 28,170 |
| traffic_sign | null | null | null | 0 | 50,150 | 56,000 |
| pole | null | null | null | 0 | 713,158 | 848,145 |

CPU2 forward+argmax p95: control136.468ms; geometry129.294ms. Cold first included; not end-to-end10Hz.

Fixed native PNGs: [000](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_left/ctrack_left_000_full_fov.png) · [119](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_left/ctrack_left_119_full_fov.png) · [239](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_left/ctrack_left_239_full_fov.png)

## ctrack_straight

Role: `exposed_external_map_diagnostic`; source QA `QA_CONDITIONAL`.

### RGB postprocess=true

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 77.9953 | 80.9854 | 2.9901 | 26,213,749 | 3,187,617 | 2,332,182 |
| road | 62.4749 | 72.3295 | 9.8546 | 25,287,084 | 2,380,040 | 2,277,079 |
| sidewalk | 9.9894 | 13.9235 | 3.9341 | 2,948,595 | 6,697,685 | 4,780,272 |
| lane_marking | 39.3014 | 39.4205 | 0.1191 | 846,572 | 26,848 | 36,218 |
| traffic_light_fixture | null | null | null | 0 | 51,054 | 17,459 |
| traffic_sign | null | null | null | 0 | 65,886 | 80,273 |
| pole | null | null | null | 0 | 1,363,317 | 1,293,319 |

CPU2 forward+argmax p95: control120.829ms; geometry120.835ms. Cold first included; not end-to-end10Hz.

Fixed native PNGs: [000](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_straight/ctrack_straight_000_full_fov.png) · [119](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_straight/ctrack_straight_119_full_fov.png) · [239](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/ctrack_straight/ctrack_straight_239_full_fov.png)

## train_town01_p01

Role: `training_resubstitution_appearance_diagnostic`; source QA `QA_CONDITIONAL`.

### RGB postprocess=false

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 99.2797 | 99.1424 | -0.1373 | 22,651,984 | 32,998 | 38,347 |
| road | 99.9075 | 99.8984 | -0.0091 | 22,755,983 | 4,260 | 6,655 |
| sidewalk | 98.9618 | 98.8048 | -0.1570 | 9,327,256 | 54,031 | 60,825 |
| lane_marking | 99.3633 | 99.0664 | -0.2969 | 151,934 | 884 | 1,344 |
| traffic_light_fixture | 75.2847 | 71.3819 | -3.9028 | 47,675 | 13,886 | 16,917 |
| traffic_sign | 92.8343 | 91.6452 | -1.1891 | 61,194 | 4,145 | 4,780 |
| pole | 73.5972 | 70.2958 | -3.3013 | 299,974 | 93,033 | 109,632 |

CPU2 forward+argmax p95: control143.234ms; geometry140.916ms. Cold first included; not end-to-end10Hz.

### RGB postprocess=true

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 98.1035 | 98.1065 | 0.0031 | 22,651,984 | 86,264 | 92,609 |
| road | 99.2015 | 99.2159 | 0.0144 | 22,755,983 | 92,898 | 87,821 |
| sidewalk | 96.2409 | 96.2742 | 0.0333 | 9,327,256 | 207,379 | 206,559 |
| lane_marking | 83.0192 | 83.4446 | 0.4254 | 151,934 | 27,157 | 26,238 |
| traffic_light_fixture | 68.7451 | 70.4339 | 1.6888 | 47,675 | 18,161 | 15,918 |
| traffic_sign | 76.4859 | 76.4839 | -0.0020 | 61,194 | 17,023 | 16,642 |
| pole | 53.0696 | 53.1167 | 0.0471 | 299,974 | 185,871 | 183,307 |

CPU2 forward+argmax p95: control140.681ms; geometry142.870ms. Cold first included; not end-to-end10Hz.

Fixed native PNGs: [000](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town01_p01/train_town01_p01_000_full_fov.png) · [119](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town01_p01/train_town01_p01_119_full_fov.png) · [239](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town01_p01/train_town01_p01_239_full_fov.png)

## train_town02_p00

Role: `training_resubstitution_appearance_diagnostic`; source QA `QA_CONDITIONAL`.

### RGB postprocess=false

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 97.6425 | 97.4216 | -0.2209 | 22,955,590 | 100,059 | 91,052 |
| road | 99.8894 | 99.8909 | 0.0016 | 22,096,394 | 5,394 | 6,812 |
| sidewalk | 98.6585 | 98.5992 | -0.0593 | 9,504,223 | 57,159 | 63,202 |
| lane_marking | 99.2374 | 99.0728 | -0.1647 | 155,855 | 1,115 | 1,387 |
| traffic_light_fixture | 54.5616 | 55.0930 | 0.5314 | 49,748 | 33,983 | 33,464 |
| traffic_sign | 70.0830 | 70.1338 | 0.0508 | 16,675 | 6,088 | 5,899 |
| pole | 49.4462 | 47.0839 | -2.3623 | 517,515 | 399,029 | 454,255 |

CPU2 forward+argmax p95: control135.935ms; geometry137.469ms. Cold first included; not end-to-end10Hz.

### RGB postprocess=true

| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |
|---|---:|---:|---:|---:|---:|---:|
| other | 95.9252 | 95.9213 | -0.0040 | 22,955,590 | 177,522 | 184,940 |
| road | 99.2893 | 99.2972 | 0.0079 | 22,096,394 | 94,012 | 91,721 |
| sidewalk | 96.5289 | 96.5519 | 0.0229 | 9,504,223 | 161,776 | 159,654 |
| lane_marking | 85.8281 | 85.6936 | -0.1345 | 155,855 | 22,385 | 22,750 |
| traffic_light_fixture | 42.9955 | 44.7173 | 1.7217 | 49,748 | 48,118 | 43,073 |
| traffic_sign | 47.7222 | 45.6555 | -2.0667 | 16,675 | 13,661 | 14,721 |
| pole | 32.6601 | 32.4794 | -0.1807 | 517,515 | 637,946 | 636,060 |

CPU2 forward+argmax p95: control135.696ms; geometry135.616ms. Cold first included; not end-to-end10Hz.

Fixed native PNGs: [000](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town02_p00/train_town02_p00_000_full_fov.png) · [119](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town02_p00/train_town02_p00_119_full_fov.png) · [239](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1/train_town02_p00/train_town02_p00_239_full_fov.png)

## External road predicted as sidewalk

| Scenario | Control % of GT road | Geometry % of GT road |
|---|---:|---:|
| ctrack_left | 3.2411 | 0.8860 |
| ctrack_straight | 24.4833 | 16.9093 |

## Separate known350 retention

Original trainer predictions on known Town01 DEV118 and Town02 p02 holdout232 were independently recounted without new forwards:161,280,000 pixels across both models. These are separate denominators, not additional external or unseen validation.

| Split | Class | Control % | Geometry % | Delta pp |
|---|---|---:|---:|---:|
| dev_town01 | other | 97.1711 | 98.0442 | 0.8731 |
| dev_town01 | road | 99.5401 | 99.7304 | 0.1903 |
| dev_town01 | sidewalk | 94.9484 | 96.8701 | 1.9217 |
| dev_town01 | lane_marking | 96.8835 | 97.4823 | 0.5988 |
| dev_town01 | traffic_light_fixture | 44.5109 | 66.2978 | 21.7868 |
| dev_town01 | traffic_sign | 7.9255 | 8.2124 | 0.2869 |
| dev_town01 | pole | 39.5370 | 46.4366 | 6.8996 |
| holdout_town02_p02 | other | 95.4252 | 94.8108 | -0.6144 |
| holdout_town02_p02 | road | 98.7990 | 98.5906 | -0.2083 |
| holdout_town02_p02 | sidewalk | 94.2188 | 93.1704 | -1.0484 |
| holdout_town02_p02 | lane_marking | 97.5411 | 98.5349 | 0.9938 |
| holdout_town02_p02 | traffic_light_fixture | 31.1085 | 36.3946 | 5.2860 |
| holdout_town02_p02 | traffic_sign | 70.8670 | 74.3141 | 3.4471 |
| holdout_town02_p02 | pole | 41.8362 | 39.9904 | -1.8458 |

## Evidence and limits

Fresh four-scenario output bytes: **24,297,045**, strictly below30MiB. [Frozen plan](PLAN.md), [preregistration](preregistration.json), [complete original metrics, masks/panel hashes and all recounts](RESULTS.json).

No new projection experiment, geometry/depth learning, HD-map topology, full-route coverage, driving validation or real-sensor validation is implied. The raw guardrail ontology caveat and limited Town01 FALSE source diversity remain. No thresholds, protected splits or failed records were changed.
