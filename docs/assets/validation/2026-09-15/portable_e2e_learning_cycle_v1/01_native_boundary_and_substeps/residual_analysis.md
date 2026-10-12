<!-- HH_260906 - Independent offline analysis of the completed reference case and retained failed fine-substep case; no source, labels, thresholds, or admission changed. -->

# Numerical-substep first-pair residual analysis

The fine-substep case did **not** complete the unchanged task. It stopped outside the goal window, never latched hold braking, and eventually failed the 45-second coast timeout. B2 and A2 were not run under the prospective stop-on-failure policy. This is an incomplete two-attempt campaign, not a favorable ABBA result.

## Evidence and denominator

| Observation | A1 reference maximum 10 ms | B1 fine maximum 5 ms |
| --- | ---: | ---: |
| Native states | 1,472 | 1,704 |
| Camera anchors / decoded original JPEGs | 736 / 4,416 | 852 / 5,112 |
| Full 64-point future windows | 671 | 788 |
| Original curvature-failing full windows | 115 (17.138599%) | 63 (7.994924%) |
| Overlapping failing steps / unique endpoint ticks | 115 / 3 | 63 / 1 |
| Maximum original curvature, rad/m | 18.444801307949692 | 0.3350743647969682 |
| Original other-eight-metric failing steps | 0 | 0 |
| Hold latch / measured goal dwell / tail | reached / 2 s / 130 observations | never / 0 / 0 |
| Capture disposition | complete diagnostic, not admitted | failed partial capture, not admitted |

Both cases independently pass initialization, acknowledged-control transport, unchanged immediate-hold/zero-steering receipt replay, and the recorded eleven-field numerical-setting application check. B1 nonetheless fails scalar/task completion. Its 64 incomplete future-anchor records remain retained; its full-window denominator is not substituted for A1's. The lower observed curvature count and maximum do not override the failed task or missing two cases.

The requested and read-back maximum substep settings are 0.01 and 0.005 seconds, respectively; both retain 20 Hz native snapshots, 10 Hz cameras, and maximum substep count 10. These are verified configuration values, not measured internal integration steps or proof of physical accuracy. Restoration is not independently established by the eleven-field application receipt.

## Numerical treatment starts before warmup

Reference A1 matches all 1,472 frozen v7 immediate observations exactly across the 25 declared physical/phase/current-control fields plus next control, excluding clocks, frame IDs and policy metadata.

Fine B1 already differs at native index 0 during stationary warmup. Reference actor/base Z is 0.446100115776062 m; fine Z is 0.4485500752925873 m, a 2.4499595 mm difference. Their raw vertical velocities are -0.9800000786781311 and -0.9800001382827759 m/s. All 1,472 overlapping row-index comparisons differ somewhere in the declared physical/control fields. Those indices are not a retimed trajectory alignment. There is no untreated cross-arm warmup prefix.

Coast release is also not the same measured state: A1 enters at native index 813, remaining arc 21.468556265993385 m and speed 3.000875695107009 m/s; B1 enters at index 803, remaining arc 21.493199564792036 m and speed 3.002413082900551 m/s. Thus the later stopping difference cannot be localized to one hold-brake event: B1 never reaches that event.

## Fine B1 stops under zero requested and observed pedals

The first coast transition from speed above 0.1 m/s to at most 0.1 m/s is frame 2941 → 2942 (native indices 1198 → 1199):

| Field | Source frame 2941 | Following frame 2942 |
| --- | ---: | ---: |
| Timestamp, s | 64.3318760280963 | 64.38187602884136 |
| Planar base speed, m/s | 0.1735420979809585 | 0.00019591067016308655 |
| Remaining arc, m | 1.7538285954533421 | 1.7503495786863539 |
| Planar goal distance, m | 1.7538480648461177 | 1.7503690742003155 |
| Requested throttle / steer / brake | 0 / 0 / 0 | 0 / 0 / 0 |
| Observed throttle / steer / brake | 0 / 0 / 0 | 0 / 0 / 0 |
| Both observed front-wheel steering angles | 0 degrees | 0 degrees |
| Body yaw, rad | 1.5692423722988353 | 1.56924516861798 |
| Recorded yaw rate, rad/s | 2.5559765063634788e-9 | 8.703130774298551e-8 |
| Actor pitch, degrees | -0.0005190943484194577 | -0.009125132113695145 |

Hand brake, reverse and manual-gear-shift flags are false. The unchanged requested gear is 0 while observed automatic gear is 1; this receipt distinction is not a newly selected factor or an explanation of the stopping event. No emergency, collision, or lane-invasion event is recorded.

The interval lasts 0.05000000074505806 seconds. In world ROS XY:

- Actor displacement is [0, 0.00347900390625] m.
- Virtual-base displacement is [0.0000039847779191859445, 0.003479015733688584] m.
- Actor signed trapezoidal velocity integral is [0.000006747130462867957, 0.004343445039524343] m.
- Virtual-base signed trapezoidal velocity integral is [0.000006750322006076926, 0.0043434450345734964] m.
- Actor/base position-minus-integral residual norms are 0.864467464 / 0.864433725 mm.

The recorded pose-offset change is about 3.984778 micrometres in X and 0.0118274 micrometres in Y. The declared yaw-first/pitch-second decomposition attributes approximately 3.984750 micrometres X to the finite yaw change and only 0.00002794 micrometres X to pitch. This is coordinate decomposition, not identification of a physical engine cause.

For comparison, reference A1's first near-goal **commanded hold-brake** interval (2985 → 2986) observes speed 0.09253888089333304 → 0.00008096845009663729 m/s, brake 0 → 1, and actor/base trapezoidal residual norms 1.365006046 / 1.922630007 mm. These are different events and different entry speeds. The smaller B1 residual is not a controlled physical-accuracy improvement. Even A1's first coast crossing below 0.1 m/s is gradual: 0.10046914014507173 → 0.09979546389561134 m/s.

## Out-of-goal standstill is not successful goal dwell

B1 reaches the low-speed crossing 19.800000295 seconds after coast release. Its first exactly zero planar speed is frame 2950 at 64.78187603480183 seconds. From that observation through final frame 3446, 497 observations span 496 native intervals, 24.800000370 seconds, with exactly zero planar speed and no actor or virtual-base XY displacement.

From the initial low-speed crossing through termination, all observed throttle, steering and brake values remain zero. All next requests remain zero until the final abort request. Remaining arc never falls below 1.7502885600083005 m; final planar goal distance is 1.7503080563113358 m. Every hold latch remains false and every goal-hold counter remains zero. There are no stationary-tail observations.

The final source at 89.58187640435062 seconds records coast elapsed 45.000000670552254 seconds and the unchanged reason `comfortable_v7_hold_brake_ramp_coast_timeout`. It still observes brake zero, then requests full brake as failed-capture cleanup. No subsequent recorded native observation is claimed to show that abort request's physical effect. The final brake is therefore not evidence that commanded hold braking caused the earlier stop.

B1's retained original curvature failures occur at 100 ms endpoint tick 600, frame 2943, immediately after the native speed collapse. All 63 overlapping failures remain in the coast phase, not a hold phase. The last 100 ms virtual-base displacement is 3.448529363497179 mm. No launch or hold failure is silently relabeled as passing.

## Interpretation and limits

This pair demonstrates numerical-setting sensitivity of the sampled trajectory and operational failure under the fine setting. It does not establish saturation, contact, sleep, integration order, solver internals, or any other unique mechanism. The original planar omega-cross-offset velocity correction is already present and was independently reconstructed; no missing-shift or speculative 3D-angular-axis explanation is asserted.

All original full-future gates, raw observations and failed-case bytes remain unchanged. Smaller curvature metrics cannot compensate for stopping outside the goal or turn an incomplete campaign into a favorable result. No data admission, model inference/training, threshold change, or new capture follows from this note.

## Reproduction and hashes

Calculations used the frozen `summarize_one` / `native_residuals` functions in the substep summary and the new native-boundary pure interval functions. Each complete/partial case was independently re-audited, including the original JPEG decode/hash checks and original future audit. Opened plan/source/raw/image fingerprints were rechecked afterward. Subsequent interval calculations reauthenticated the two native state files before and after. No files other than this new note were written.

| Evidence | SHA-256 |
| --- | --- |
| Prospective plan | dcfb17fa955208879606dce4c618ef0720522f780e046d4c5fde3aeabfd937e0 |
| A1 states | ee85e3fbbf90251fd30facf071a5f6c2f1a1537e8de940fc888a3552f8ef9731 |
| B1 partial states | 6350203e4afa39d65fee8dfccb39571d52d69dff98e3368691e9e2c7802f761a |
| B1 partial manifest | 22a87b01376cb6aa0d90fc70720b3ada8249a5e208ec9ff00b8adf17cf339b80 |
| B1 camera metadata | 40a63120da482b1580041ad57c731de8ce4b1bbe55595b2ce7d536cfec5da56f |
| A1 owned result | a4ffa07afdd136208077ef474bc30928b370032ec2480c11d481e2faae81df9f |
| B1 owned result | 7806ff7edda08fa80314c0dc7379bef24bf86f2b4591d870888fd2f796e75851 |
| A1 durable execution result | ab2e90cf1b0d48711f1e759ae04887d3766e9fb997e30db9744ce9679bd785ab |
| B1 durable execution result | 0efc510efe9861ea08810c5f209049c84a8aafe604893b338d74cc5bf7ddc406 |
| Frozen substep summary source | a5655f6764e87ceb7fda67a0341dab8b3c95ea7fe36db6a7857eba06cc33dcb7 |
| Native-boundary source | e72d07671bda2e854e78416cfb9e0e21c1fdd2802b0b248857e4de2d0795e34b |
| Original curvature-conditioning source | 165b1545ee476349bdb58cd4e8fc610a85242ff5d44a1c0edd8024a954ad1385 |
| Original raw-future audit source | 0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00 |

