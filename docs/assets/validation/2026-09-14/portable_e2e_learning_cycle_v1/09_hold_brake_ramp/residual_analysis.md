<!-- HH_260906 - Independent offline raw-motion comparison; no labels, gates, execution sources or admission decisions changed. -->

# V7 first-pair residual analysis

The declared 0.5 s normal hold-brake ramp changed the recorded brake requests and readbacks, but did not reduce, shift, or spread the original curvature failures in the first completed pair. This note independently compares `01_immediate_A1` with `02_ramp_B1` only. It is not a completed four-case ABBA verdict; B2/A2 are not included in these calculations.

## Original failures retained

Both cases contain 1,472 native states (70 warmup, 1,272 driving, 130 tail), 736 camera anchors, and 671 complete 64-point future windows. Both have 115 curvature-failing anchors, 115 overlapping failing steps, and three unique 100 ms endpoint ticks. The other eight original metric categories have zero failing steps. All 38,423 assessed curvature values and flags per case were independently replayed exactly against the unchanged raw-future audit. Actor-to-virtual-base pose replay residual is zero.

| Episode-relative 100 ms endpoint | A1 frame | B1 frame | Phase | Overlapping failures | Maximum original curvature (rad/m) |
| --- | ---: | ---: | --- | ---: | ---: |
| 50 | 1806 | 2014 | launch | 50 | 0.8055053150536444 |
| 51 | 1808 | 2016 | launch | 1 | 0.23031410370668795 |
| 651 | 3008 | 3216 | service-brake hold | 64 | 18.444801307949692 |

The hold-boundary minimum/maximum across its 64 windows are also identical: 18.355100608072227 / 18.444801307949692 rad/m. The global worst remains the hold boundary. The launch residuals are unchanged. These windows overlap; 115 failures are not 115 independent maneuvers. Original interpolation and held-heading behavior were retained, including targets 2–96 ns before the nearest native hold endpoint; no timestamps or targets were shifted.

## The treatment reached the recorded control interface

The first hold-source observation is native index 1301 (zero-based): A1 frame 3007 at 69.35016980092041 s; B1 frame 3215 at 69.46152286999859 s. Both observe speed 0.09253888089333304 m/s, remaining route arc 0.9976328737526217 m, and current brake zero. Their next brake differs: A1 requests 1.0; B1 mathematically requests 0.1, stored as float32 0.10000000149011612.

B1 normal requests follow the predeclared measured-clock formula `min(1, (source_time - hold_source_time + 0.05) / 0.5)`. The ten successive stored values are:

`0.10000000149011612, 0.20000000298023224, 0.30000001192092896, 0.4000000059604645, 0.5, 0.6000000238418579, 0.699999988079071, 0.800000011920929, 0.9000000357627869, 1.0`.

Each is observed in `current_control.brake` on the following native state. Full B1 brake is first requested at +0.45000000670552254 s (frame 3224), and first read back at +0.5000000074505806 s (frame 3225). A1 full brake is first read back at +0.05000000074505806 s. All independently checked normal formula/projection comparisons have zero mismatches; both unchanged ACK audits pass. No emergency override occurs. Throttle, output steering, and both measured front wheel angles are zero around hold entry. These are command/readback observations, not proof of physical brake torque or substep actuation timing.

## Measured motion did not change

All 1,302 pre-treatment observations through the hold source match exactly across the 25 declared physical/control fields, with only that source's already-treated `next_control` excluded. More strongly, all 1,472 paired states match exactly across the 24 non-control physical/phase fields, including actor pose, virtual-base pose, velocities, accelerations, yaw rates, wheel angles, route progress and cross-track error. Only brake differs in nine `next_control` records (indices 1301–1309) and nine `current_control` records (1302–1310); other serialized control fields match.

In both arms the first following observation has speed 0.00008096845009663729 m/s. The 100 ms interval ending at tick 651 travels 5.428664284167243 mm at the virtual base versus 5.402667271049307 mm at the raw actor reference, with body-yaw change 0.0003058640511954813 rad. Its adjacent-segment curvature is 18.4448029184775 rad/m at the virtual base versus 3.6595691763410634 rad/m at the actor reference. The actor comparison is diagnostic only and does not replace the original virtual-base labels or erase their failures.

From hold source to final driving state, both arms have virtual-base travel 1.3178849777397443 mm and net displacement 1.248902492976292 mm; actor-reference travel is 1.010253492644676 mm and net displacement 0.9486255519024385 mm. Both first report exactly zero speed at +0.45000000670552254 s, finish with 40 unchanged dwell intervals (recorded 2.0 s; measured source-to-finish interval 2.0000000298023224 s), and retain 130 tail observations with zero measured speed and zero tail pose displacement. Final remaining route arc is 0.9967020310592432 m.

Thus this pair does not support the proposed ramp as a remedy for the observed hold-boundary geometry. The difference between 0.1 and 1.0 requested brake produced no difference in the sampled physical trace here. That observation does not identify saturation, contact, numerical integration, or another engine mechanism, nor establish behavior between samples or in other conditions. No new data admission, model inference/training, curvature tolerance, or general safety claim follows.

## Evidence binding

The original raw-future, curvature-conditioning, camera and ACK functions ran offline without output generation. All 8,832 JPEGs were decoded and checked by the existing camera audit. Thirty-six plan/source/input/raw-metadata files were hashed before and after the comparison; no differences occurred. The separate whole-trace comparison reauthenticated both state files before and after. JPEG decoding is not claimed as a second post-comparison hash pass over every image.

- Prospective plan SHA-256: `a7708081681efb7dddf9fc52e547a0950b500dcb2c36f5be9f1deaa2402d0a48`.
- A1 states SHA-256: `e4ae07c92cf4763b1e1c72e90c78c63edd0c55b81ae9414c8db50456069399c3`.
- B1 states SHA-256: `0cc6a15ee24921fa107645da5a6516a03ccef62a9d2a112aee59525f3c8bcc5b`.
- Raw-future audit SHA-256: `0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00`.
- Curvature-conditioning audit SHA-256: `165b1545ee476349bdb58cd4e8fc610a85242ff5d44a1c0edd8024a954ad1385`.
