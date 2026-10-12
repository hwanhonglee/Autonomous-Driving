<!-- HH_261001 - Preserve the complete failed capture, distinguishing future-window improvement from native/full-route QA. -->
# V2 raw QA: useful start windows, failed complete episode

The source/owner cleanup checks are clean. This expert-only run is **not admitted to TRAIN** and is not a successful route/stop demonstration.

| Check | Actual result |
| --- | --- |
| Native states / six-camera bundles / decoded images | 1,890 /945 /5,670 |
| Full6.4s futures | 881:35 warmup +846 driving |
| Incomplete futures retained | 64; none silently removed |
| All nine unchanged10Hz future bounds | Zero violations in every assessed full/prefix window |
| Genuine initial low-speed full futures | 22/22 clear; v1 was0/21 |
| Native20Hz acceleration | One +3.193852m/s² violation; limit2.9 unchanged |
| Native20Hz deceleration | Minimum−1.782431m/s²; zero violations |
| Endpoint | 0.070788m overshoot, still moving1.500171m/s |
| Required goal stop/dwell/tail | Not completed |
| Collision /lane events /ACK mismatches | 0 /0 /0 |

The native acceleration failure is frame1346→1347: vx0.25443694→0.41412618m/s in50ms. Both actual steering command and measured wheel angle are zero, throttle remains0.13, brake0 and gear1. Neither an emergency override nor a launch-to-ramp transition explains that interval directly. Simulator internal tire/engine state was not instrumented, so its mechanism is not established.

The stop-speed envelope entered approach only at6.968976m remaining and2.484492m/s. Normal pedals were both zero by frame3060; the car still crossed the endpoint at1.500171m/s. Requested envelope deceleration0.5m/s² is not a measured coast-deceleration guarantee. Earlier explicitly measured coasting would be a separate protocol, not a retrospective adjustment to this run.

Allraw states, controls, images and failed owner evidence are preserved. The native failure and missing stop/tail prevent whole-episode approval despite improved10Hz initial futures. No filtered TRAIN corpus, relabeled model rollout, corrected target, relaxed threshold or third attempt was produced.

See [complete raw audit](raw_audit/summary.json), [every full/partial future](raw_audit/all_raw_futures.jsonl.gz) and [every image check](raw_audit/all_image_checks.jsonl.gz). Original recording: `storage/experiments/validation/2026-10-01/town07_low_speed_start_expert_v2/episode.partial`.
