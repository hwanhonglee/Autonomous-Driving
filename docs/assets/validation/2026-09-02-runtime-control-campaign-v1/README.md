# Autoware VAD runtime/control visual evidence

This is a curated, review-oriented copy of the canonical CARLA campaign. Source evidence remains immutable; bags, MKV files, logs, and other bulk runtime files are intentionally excluded.

- Campaign: `autoware_vad_runtime_control_campaign_v1`
- Publication target: `docs_mirror`
- Campaign status: `COMPLETE_30KPH_60KPH_COMPLETE_FAILED`
- 30 km/h control decision: `HOLD`
- Source snapshot SHA-256: `8989fb74039f747921c2d6f552fab696b2adc909737ce6256bcc9e7abd86e6ee`
- Fullscreen images passed the vehicle-centered and visible-path capture contract.
- Every copied byte is sealed in `publication_manifest.json` and `SHA256SUMS`.

## 30 km/h A/B evidence

| Scenario | Variant | Evidence | File |
|---|---|---|---|
| C-track turn | baseline | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/c_track_turn/A_baseline/01_autoware_vehicle_centered_fullscreen.png) |
| C-track turn | baseline | drive | [02_autoware_drive.gif](30kph/c_track_turn/A_baseline/02_autoware_drive.gif) |
| C-track turn | baseline | path_vs_control | [03_path_vs_control.png](30kph/c_track_turn/A_baseline/03_path_vs_control.png) |
| C-track turn | baseline | steering_tracking | [04_steering_tracking.png](30kph/c_track_turn/A_baseline/04_steering_tracking.png) |
| C-track turn | baseline | speed_profile | [05_speed_profile.png](30kph/c_track_turn/A_baseline/05_speed_profile.png) |
| C-track turn | candidate | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/c_track_turn/B_candidate/01_autoware_vehicle_centered_fullscreen.png) |
| C-track turn | candidate | drive | [02_autoware_drive.gif](30kph/c_track_turn/B_candidate/02_autoware_drive.gif) |
| C-track turn | candidate | path_vs_control | [03_path_vs_control.png](30kph/c_track_turn/B_candidate/03_path_vs_control.png) |
| C-track turn | candidate | steering_tracking | [04_steering_tracking.png](30kph/c_track_turn/B_candidate/04_steering_tracking.png) |
| C-track turn | candidate | speed_profile | [05_speed_profile.png](30kph/c_track_turn/B_candidate/05_speed_profile.png) |
| C-track turn | comparison | comparison | [06_A_baseline_vs_B_candidate_control_decision.png](30kph/c_track_turn/comparison/06_A_baseline_vs_B_candidate_control_decision.png) |
| Town03 turn | baseline | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town03_turn/A_baseline/01_autoware_vehicle_centered_fullscreen.png) |
| Town03 turn | baseline | drive | [02_autoware_drive.gif](30kph/town03_turn/A_baseline/02_autoware_drive.gif) |
| Town03 turn | baseline | path_vs_control | [03_path_vs_control.png](30kph/town03_turn/A_baseline/03_path_vs_control.png) |
| Town03 turn | baseline | steering_tracking | [04_steering_tracking.png](30kph/town03_turn/A_baseline/04_steering_tracking.png) |
| Town03 turn | baseline | speed_profile | [05_speed_profile.png](30kph/town03_turn/A_baseline/05_speed_profile.png) |
| Town03 turn | candidate | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town03_turn/B_candidate/01_autoware_vehicle_centered_fullscreen.png) |
| Town03 turn | candidate | drive | [02_autoware_drive.gif](30kph/town03_turn/B_candidate/02_autoware_drive.gif) |
| Town03 turn | candidate | path_vs_control | [03_path_vs_control.png](30kph/town03_turn/B_candidate/03_path_vs_control.png) |
| Town03 turn | candidate | steering_tracking | [04_steering_tracking.png](30kph/town03_turn/B_candidate/04_steering_tracking.png) |
| Town03 turn | candidate | speed_profile | [05_speed_profile.png](30kph/town03_turn/B_candidate/05_speed_profile.png) |
| Town03 turn | comparison | comparison | [06_A_baseline_vs_B_candidate_control_decision.png](30kph/town03_turn/comparison/06_A_baseline_vs_B_candidate_control_decision.png) |
| Town07 straight | baseline | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town07_straight/A_baseline/01_autoware_vehicle_centered_fullscreen.png) |
| Town07 straight | baseline | drive | [02_autoware_drive.gif](30kph/town07_straight/A_baseline/02_autoware_drive.gif) |
| Town07 straight | baseline | path_vs_control | [03_path_vs_control.png](30kph/town07_straight/A_baseline/03_path_vs_control.png) |
| Town07 straight | baseline | steering_tracking | [04_steering_tracking.png](30kph/town07_straight/A_baseline/04_steering_tracking.png) |
| Town07 straight | baseline | speed_profile | [05_speed_profile.png](30kph/town07_straight/A_baseline/05_speed_profile.png) |
| Town07 straight | candidate | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town07_straight/B_candidate/01_autoware_vehicle_centered_fullscreen.png) |
| Town07 straight | candidate | drive | [02_autoware_drive.gif](30kph/town07_straight/B_candidate/02_autoware_drive.gif) |
| Town07 straight | candidate | path_vs_control | [03_path_vs_control.png](30kph/town07_straight/B_candidate/03_path_vs_control.png) |
| Town07 straight | candidate | steering_tracking | [04_steering_tracking.png](30kph/town07_straight/B_candidate/04_steering_tracking.png) |
| Town07 straight | candidate | speed_profile | [05_speed_profile.png](30kph/town07_straight/B_candidate/05_speed_profile.png) |
| Town07 straight | comparison | comparison | [06_A_baseline_vs_B_candidate_control_decision.png](30kph/town07_straight/comparison/06_A_baseline_vs_B_candidate_control_decision.png) |

## 30 km/h selected baseline regressions

These are the exact post-fix depth-1 loopback regressions; the curator never scans for a newest-looking directory.

| Scenario | Evidence | File |
|---|---|---|
| C-track turn | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/c_track_turn/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) |
| C-track turn | drive | [02_autoware_drive.gif](30kph/c_track_turn/C_selected_baseline_regression/02_autoware_drive.gif) |
| C-track turn | path_vs_control | [03_path_vs_control.png](30kph/c_track_turn/C_selected_baseline_regression/03_path_vs_control.png) |
| C-track turn | steering_tracking | [04_steering_tracking.png](30kph/c_track_turn/C_selected_baseline_regression/04_steering_tracking.png) |
| C-track turn | speed_profile | [05_speed_profile.png](30kph/c_track_turn/C_selected_baseline_regression/05_speed_profile.png) |
| Town03 turn | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town03_turn/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) |
| Town03 turn | drive | [02_autoware_drive.gif](30kph/town03_turn/C_selected_baseline_regression/02_autoware_drive.gif) |
| Town03 turn | path_vs_control | [03_path_vs_control.png](30kph/town03_turn/C_selected_baseline_regression/03_path_vs_control.png) |
| Town03 turn | steering_tracking | [04_steering_tracking.png](30kph/town03_turn/C_selected_baseline_regression/04_steering_tracking.png) |
| Town03 turn | speed_profile | [05_speed_profile.png](30kph/town03_turn/C_selected_baseline_regression/05_speed_profile.png) |
| Town07 straight | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](30kph/town07_straight/C_selected_baseline_regression/01_autoware_vehicle_centered_fullscreen.png) |
| Town07 straight | drive | [02_autoware_drive.gif](30kph/town07_straight/C_selected_baseline_regression/02_autoware_drive.gif) |
| Town07 straight | path_vs_control | [03_path_vs_control.png](30kph/town07_straight/C_selected_baseline_regression/03_path_vs_control.png) |
| Town07 straight | steering_tracking | [04_steering_tracking.png](30kph/town07_straight/C_selected_baseline_regression/04_steering_tracking.png) |
| Town07 straight | speed_profile | [05_speed_profile.png](30kph/town07_straight/C_selected_baseline_regression/05_speed_profile.png) |

## 60 km/h selected pilot

- Status: `COMPLETE_FAILED`
- Explicit source: `30_60kph/town06_straight_60kph_pilot_best_effort_image_depth1_v3`

| Evidence | File |
|---|---|
| fullscreen | [01_autoware_vehicle_centered_fullscreen.png](60kph/town06_straight/selected_pilot/01_autoware_vehicle_centered_fullscreen.png) |
| drive | [02_autoware_drive.gif](60kph/town06_straight/selected_pilot/02_autoware_drive.gif) |
| path_vs_control | [03_path_vs_control.png](60kph/town06_straight/selected_pilot/03_path_vs_control.png) |
| steering_tracking | [04_steering_tracking.png](60kph/town06_straight/selected_pilot/04_steering_tracking.png) |
| speed_profile | [05_speed_profile.png](60kph/town06_straight/selected_pilot/05_speed_profile.png) |

## 60 km/h geometry A/B

The corridor 0.20 m candidate is diagnostic evidence only; it does not replace the selected pilot v3.

- Geometry decision: `HOLD`
- Independent speed contract: `FAIL`
- Real-vehicle ready: `false`
- Exact candidate source: `30_60kph/town06_straight_60kph_geometry_corridor_0p2_v4`
- Comparison JSON: `50_reports/town06_60kph_geometry_corridor_ab_v4.json`

| Variant | Evidence | File |
|---|---|---|
| geometry_corridor_0p2_hold | fullscreen | [01_autoware_vehicle_centered_fullscreen.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/01_autoware_vehicle_centered_fullscreen.png) |
| geometry_corridor_0p2_hold | drive | [02_autoware_drive.gif](60kph/town06_straight/B_geometry_corridor_0p2_hold/02_autoware_drive.gif) |
| geometry_corridor_0p2_hold | path_vs_control | [03_path_vs_control.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/03_path_vs_control.png) |
| geometry_corridor_0p2_hold | steering_tracking | [04_steering_tracking.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/04_steering_tracking.png) |
| geometry_corridor_0p2_hold | speed_profile | [05_speed_profile.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/05_speed_profile.png) |
| geometry_corridor_0p2_hold | route_result | [06_route_result.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/06_route_result.png) |
| geometry_corridor_0p2_hold | runtime_load | [07_runtime_load_analysis.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/07_runtime_load_analysis.png) |
| geometry_corridor_0p2_hold | longitudinal_response | [08_longitudinal_response.png](60kph/town06_straight/B_geometry_corridor_0p2_hold/08_longitudinal_response.png) |
| geometry_comparison | comparison | [09_A_selected_pilot_v3_vs_B_geometry_corridor_0p2_HOLD.png](60kph/town06_straight/comparison/09_A_selected_pilot_v3_vs_B_geometry_corridor_0p2_HOLD.png) |

## Integrity

`publication_manifest.json` records the source and published SHA-256 for each asset. A later run replaces or removes only paths authorized by the previously verified manifest; unmanaged files and canonical source evidence are never removed.
