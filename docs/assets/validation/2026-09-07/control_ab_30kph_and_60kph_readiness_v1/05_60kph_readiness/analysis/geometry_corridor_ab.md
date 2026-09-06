# Town06 60 km/h geometry-only A/B

- Geometry decision: **HOLD** (PARTIAL_IMPROVEMENT_INSUFFICIENT)
- Independent 60 km/h speed contract: **FAIL**
- Real-vehicle ready: **false**
- Behavioral delta: straight-route corridor half-width 0.50 m → 0.20 m
- Coupled serialized delta: turn-outward width 0.50 m → 0.20 m (inactive on straight route; route-manager invariant)

## Pair integrity

- Effective source route SHA-256: `ae019ba6f839935919e7b11fa3a3131255849bfbc7ae191b8a517a3182233018`
- Canonical geometry/identity SHA-256: `51c189c0803363df9896b9e1caab596d507bd7f01e115cf57b0ce70656cff5dc`
- Speed-profile configuration/invariants, controller, command gate, raw converter/maps, trajectory code: byte-identical
- Each speed/diagnosis artifact is bound to its own aligned route and immutable rosbag manifest

## Measured comparison

| Metric | A baseline | B corridor 0.20 m |
|---|---:|---:|
| Conditioned peak curvature p95 (1/m) | 0.019054 | 0.014826 |
| Conditioned peak curvature max (1/m) | 0.063131 | 0.046651 |
| Mid-route 4–9 m/s cap samples | 51 | 43 |
| Mid-route cap exposure (s) | 10.200 | 8.600 |
| Maximum speed (m/s / km/h) | 10.100 / 36.36 | 9.754 / 35.11 |
| ≥15 m/s sustained (s) | 0.000 | 0.000 |
| Maximum CTE (m) | 0.524 | 0.221 |
| Maximum lateral acceleration (m/s²) | 0.437 | 0.341 |
| Maximum trajectory correction (m) | 4.601 | 4.243 |

## Geometry gates

| Gate | Status | Requirement |
|---|---|---|
| `baseline_runtime_camera_health` | PASS | runtime health PASS, RTF >= 0.90, camera coverage >= 99%, receipt p95 <= 40 ms |
| `candidate_runtime_camera_health` | PASS | runtime health PASS, RTF >= 0.90, camera coverage >= 99%, receipt p95 <= 40 ms |
| `candidate_goal_reached` | PASS | route goal must be reached even if the independent speed contract fails |
| `conditioned_curvature_p95_material_reduction` | PASS | candidate conditioned peak-curvature p95 <= 80% of baseline |
| `conditioned_curvature_supports_15mps` | FAIL | conditioned peak-curvature p95 <= 0.004444444 1/m |
| `conditioned_curvature_maximum_supports_15mps` | FAIL | maximum conditioned peak curvature <= 0.004444444 1/m |
| `conditioned_curvature_maximum_no_kink_regression` | PASS | candidate maximum conditioned curvature must not exceed baseline |
| `midroute_4_to_9mps_caps_eliminated` | FAIL | candidate must have no 4-9 m/s horizon caps outside the terminal braking buffer |
| `candidate_cte` | PASS | maximum absolute CTE <= 1.00 m |
| `candidate_lateral_acceleration` | PASS | maximum lateral acceleration <= 1.20 m/s^2 |
| `candidate_correction_no_regression` | PASS | maximum trajectory correction increase <= 0.05 m |

## Independent speed gates

| Gate | Status | Requirement |
|---|---|---|
| `exposure_status` | FAIL | route result speed-exposure status PASS |
| `minimum_speed_reached` | FAIL | maximum speed >= 15.0 m/s |
| `minimum_sustained_duration` | FAIL | >= 15 m/s sustained for >= 1.0 s |
| `overspeed_ceiling` | PASS | maximum speed <= 18.0 m/s |

The geometry decision does not imply 60 km/h qualification. The result remains CARLA simulation-only and must not be transferred to a real vehicle.
