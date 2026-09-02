# CTrack camera-source cadence A/B

- Status: **PASS**
- Decision scope: Functional/integrity gates passed. Performance deltas are descriptive; this analyzer does not impose an optimization-adoption threshold.
- Baseline: `speed_30kph` (`sensor_tick=0.0 s`)
- Candidate: `speed_30kph_camera_source_5hz` (`sensor_tick=0.2 s`)

## Window contract

RTF uses the route-evaluation interval. Camera rates use the whole rosbag as reported by `latency/e2e_latency.json`; this analyzer does not reread the bag. Receipt efficiency therefore remains an explicitly cross-window descriptive sanity ratio.

## Comparison

| Trial | Arm | RTF | Front wall Hz | Front stamp Hz | Max camera gap s | Receipt efficiency | Bundle coverage | Candidate acceptance | Inference p95 ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| straight | baseline | 0.319011 | 1.736707 | 5.000000 | 0.200000 | 1.088809 | 100.000% | 100.000% | 38.074 |
| straight | candidate | 0.999625 | 4.992644 | 5.000000 | 0.200000 | 0.998904 | 100.000% | 100.000% | 31.317 |
| turn | baseline | 0.998307 | 4.992801 | 5.000000 | 0.200000 | 1.000253 | 100.000% | 100.429% | 38.576 |
| turn | candidate | 0.999539 | 4.992597 | 5.000000 | 0.200000 | 0.998980 | 100.000% | 100.435% | 31.493 |

## Candidate change from baseline

| Trial | RTF | Front wall receipt | Receipt efficiency | Inference p95 |
|---|---:|---:|---:|---:|
| straight | +213.352% | +187.478% | -8.257% | -17.748% |
| turn | +0.123% | -0.004% | -0.127% | -18.361% |

## Integrity gates

- [x] `terminal_c_track_straight_turn_pass`
- [x] `runtime_profile_invariants_equal`
- [x] `source_route_sha_and_payload_equal`
- [x] `mapping_delta_exactly_six_camera_sensor_ticks`
- [x] `whole_bag_six_camera_coverage_at_least_99_percent`
- [x] `all_six_camera_stamp_gaps_at_most_0_25_sec`
- [x] `whole_bag_candidate_front_boundary_gate`
- [x] `vad_no_runtime_supersession_pruning_or_coalescing`
- [x] `route_manager_blas_threads_pinned_to_one`
- [x] `vad_published_equals_mailbox_taken`
- [x] `functional_route_and_speed_pass`

## Route identity

- `straight`: `38376adb07bf8c07bb1f503d3ea915114ce7dffc4799004f2ac1a85b610fee43` (byte SHA and canonical payload equal)
- `turn`: `1bb68ae7dc18dc4503d1962ea55a169befb82ec5fa871d89da23b8ae1a647b64` (byte SHA and canonical payload equal)

A single straight/turn pair is descriptive. Use paired repeated runs and a predeclared performance threshold before making an optimization-adoption claim.
