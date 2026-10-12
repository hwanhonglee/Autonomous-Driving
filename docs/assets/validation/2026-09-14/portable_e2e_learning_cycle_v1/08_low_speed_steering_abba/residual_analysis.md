<!-- HH_260906 - Read-only residual analysis of immutable v6 observations; no target, gate or admission changes. -->

# V6 residual curvature observations

The zero-steering intervention reduced the observed failure incidence, but it did not clear the original geometry checks. The ABBA campaign is incomplete: A1, B1 and B2 completed; A2's single declared attempt was blocked before owner/collector startup because port 2101 was occupied. No A2 capture directory was created and no retry was made. Consequently there is no full four-case comparative verdict or training-data approval.

## Recomputed scope and denominators

The independent CPU-only check called the unchanged `measured_future_audit` and underlying interpolation/target-feasibility functions on original native states, route and frame-bound camera metadata. It then replayed every A1/B1 curvature value and flag with the pure curvature-conditioning math. Temporary Python metric tuples were represented as JSON arrays in memory for that replay; no input or output target file was rewritten. Camera metadata was checked, but this residual analysis did not independently decode JPEG pixels.

| Case | Native states | Camera anchors | Full 64-point windows | Violating windows | Overlapping failing steps | Unique 100 ms endpoints | Maximum curvature (rad/m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A1 native | 1,462 | 731 | 666 | 134 | 612 | 39 | 16.79010100779868 |
| B1 zero | 1,472 | 736 | 671 | 115 | 115 | 3 | 18.444801307949692 |
| B2 zero | 1,472 | 736 | 671 | 115 | 115 | 3 | 18.444801004335837 |

All eight other original metric categories had zero violating steps in these recomputations. A1/B1 curvature values and flags replayed exactly, and their raw-actor-to-existing-virtual-base pose replay residual was zero. B2's counts were independently recomputed with the original future audit; matching counts do not mean every interpolated scalar is byte-identical. The different window denominators are retained. Overlapping failures are not independent driving events.

## B1's three remaining endpoints

Times below are relative to the first recorded native state, not wall time or the start of driving. The source sequence remains the original 20 Hz recording. Future targets retain their original nanosecond interpolation: failure targets precede their nearest native endpoint by 1–96 ns, rather than being retimed onto it.

| Episode 100 ms tick / native frame | Observed phase and elapsed time | Original failure occurrences | Observation |
| --- | --- | ---: | --- |
| 50 / 1851 | `launch_low`, 5.000000074505806 s | 50 | First assessed displacement after held/sub-0.1 mm steps; 49 older windows still use their retained body-forward initial heading, plus one first-point window. |
| 51 / 1853 | `launch_low`, 5.100000075995922 s | 1 | Only the new window's first-point body-forward comparison fails; the adjacent-segment comparison is different. |
| 651 / 3053 | `service_brake_hold`, 65.1000009700656 s | 64 | The native 100 ms interval includes the service-brake command transition at frame 3052. |

At all three endpoints and their immediately surrounding native samples, the requested/read-back steering and both measured front-wheel steering angles are exactly zero. Therefore increasing the 0.75 m/s steering cutoff is not supported as a remedy for these remaining failures.

At tick 50, the native virtual-base displacement is 0.009225020726629467 m, with a body-heading displacement-direction offset of −0.007430793732122299 rad. The original interpolated-window curvatures are 0.8055042996466454–0.8055053150536444 rad/m. The actor-reference displacement is 0.00922474569692042 m; its corresponding diagnostic body-heading curvature is about 0.210178 rad/m. Vehicle body yaw changes by only 0.00003555320055270705 rad. The automatic reported gear changes from 0 to 1 in this launch interval; the observations do not establish that gear selection caused the residual.

At tick 51, the native virtual-base displacement is 0.03266680181137672 m. The original first-point window curvature is 0.23031410370668795 rad/m. In contrast, the immediately adjacent native segment's virtual-base curvature is 0.0017534002004176966 rad/m, and the actor-reference body-heading curvature is 0.18444217599238505 rad/m. These are different geometric questions: the original window starts from body-forward heading, not the preceding path-segment heading. Its failure remains a failure.

At tick 651, the native virtual-base displacement is 0.005428664284167243 m. Its adjacent-segment direction changes by −0.10013064283204254 rad, giving 18.4448029184775 rad/m in native-endpoint geometry; the unchanged interpolated-window maximum is 18.444801307949692 rad/m. This direction change is not vehicle body yaw: body yaw changes by only 0.0003058640511954813 rad. The actor-reference displacement is 0.005402667271049307 m, with adjacent-segment curvature 3.6595691763410634 rad/m. The existing 1.425 m virtual-reference conversion contributes 0.00043585624820252633 m to the ROS-world X displacement over this interval. Current brake readback is 0, 0, 1 across frames 3051–3053; the full-brake request is recorded at frame 3052, and speed falls from 0.09253888089333304 m/s at that source to 0.00008096845009663729 m/s at frame 3053. Command acceptance/readback does not prove physical substep actuator timing.

The actor/reference comparison is diagnostic only. It does not replace the training reference point, prove physical COM/rear-axle identity, or authorize ignoring small displacements. All three steps exceed the unchanged 0.0001 m assessment threshold. No calibrated noise bound or engine/contact/numerical cause has been established.

## Relation to A1 and a prospective next revision

A1 has 3 launch, 3 normal-PID, 32 low-speed-coast and 1 service-hold failing endpoints. B1 has 2 launch and 1 service-hold endpoint, with none remaining in normal PID or low-speed coast. Thus the observed reductions coexist with a worse maximum curvature and unresolved launch/hold transients.

A1's brake-entry interval has a similar-magnitude body-yaw change (−0.000309326160612855 rad) and virtual-reference lateral contribution (approximately 0.441 mm). Unlike B1, its preceding steering readback is nonzero before the inherited stopped-brake guard sets it to zero. B1's persistence with zero measured wheel angles means the residual cannot be explained solely by a nonzero emitted steering request.

A next revision should prospectively isolate one longitudinal transition factor while retaining the v6 zero-steering arm: either a bounded launch-pedal/ramp change **or** a normal service-hold brake-onset ramp, not both in one comparison. Declare the exact commands, timing, attempt budget and acceptance criteria before any fresh capture. Keep emergency full braking, existing speed/route limits, measured ≤0.1 m/s stop criterion, continuous 2 s dwell, 6.5 s tail, all original windows and all failed attempts. Neither proposed intervention is known to pass. Do not change label reference points, retime observations, relax gates, add retries, or promote these captures automatically.

## Evidence identity

- Prospective `plan.json`: `f0d4ee95bb1ab9f46117e42107a2b396950e16cf9f2b918bd4f6d46718fe131a`.
- `execution_attempts.json`: `f2025e1c2b8fa7d711a80ab943b309632e0fd5f6dbda6807347afac1c38b02b4`.
- Original feasibility source: `8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894`; original preparation/interpolation source: `c63c7e5b7e0c06336c885f2d6120e3e8a8a74daf372ddf5df167eac2b4b4b666`.
- The A1/B1 residual computation rehashed its 21 source/metadata inputs before and after without changes. B2's three raw metadata files were separately rehashed before/after its count recomputation. No simulator, model inference, GPU training or dataset export was performed by this analysis.
