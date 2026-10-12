<!-- HH_260906 - Independent offline analysis of both finalized v9 cases; preserve every original failure and distinguish requested control from measured response. -->

# Terminal feedback: completed task, worse raw-path quality

Both fresh reference-10ms cases completed the unchanged goal, speed, dwell and tail requirements. Feedback B nevertheless worsened the original future-path tests and is not an adoption or data-admission result. No learned model controlled these cases; this analysis does not start training, export data or change any gate.

## Complete denominators and outcome

| Recorded or independently audited quantity | Coast A | Feedback B |
| --- | ---: | ---: |
| Native states | 1,472 | 1,511 |
| Warmup / driving / tail states | 70 / 1,272 / 130 | 70 / 1,311 / 130 |
| Camera anchors / original JPEGs audited | 736 / 4,416 | 756 / 4,536 |
| Full 64-point future windows | 671 | 691 |
| Curvature-failing full windows | 115 (17.138599%) | 201 (29.088278%) |
| Overlapping curvature-failing steps / unique endpoints | 115 / 3 | 938 / 16 |
| Maximum original curvature, rad/m | 18.444801307949692 | 573.8659331637988 |
| Lateral-acceleration-failing full windows / steps | 0 / 0 | 64 / 64 |
| Maximum original XY lateral acceleration, m/s² | 0.4146364970830196 | 5.503501348242993 |
| Final planar goal distance, m | 0.9967049338772591 | 0.2934777645755596 |
| Final remaining route arc, m | 0.9967020310592432 | 0.2934548169408231 |
| Final driving speed, m/s | 0 | 0 |
| Independently measured continuous goal dwell, s | 2.0000000298023224 | 2.0000000298023224 |
| Scheduled measured tail | 130 ticks / 6.5 s | 130 ticks / 6.5 s |
| Coast-entry to completed dwell, s | 26.400000393390656 | 28.35000042244792 |

The other seven original future metrics have zero failing steps in both cases. Both owner exits are zero, and the frozen full auditor reports execution/scalar/pixel/initialization/ACK/control-protocol checks clear. Its combined evidence contains all 2,983 native states, 1,492 anchor records, 8,952 original JPEGs, 2,983 control witnesses and 2,981 native intervals. None of the 64 or 938 failure incidences is removed. Overlapping windows are not independent trials.

Both cases use identical recorded eleven-field before/requested/after physics receipts: 20 Hz native updates, 10 Hz cameras, maximum substep delta 0.01 s and maximum substeps 10. Those receipts establish the recorded settings, not measured internal timesteps or an independent eleven-field restoration proof. No collision, lane-invasion or emergency event is recorded in either case.

## Activation is not the first changed pedal request

Coast A matches all 1,472 frozen v8 reference-A1 observations exactly across the declared 25 physical/phase/current-control fields plus the complete seven-field next-control dictionary. Absolute clocks, actor/frame IDs and new policy metadata are excluded; there is no retiming.

The two fresh v9 cases also match all 25 physical fields through native index 1,113 inclusive: 1,114 observations. Their next controls match through index 1,112. At index 1,113 only the next throttle first differs, zero in A versus 0.00010600927635096014 in B. The first following measured state differs at index 1,114.

For B, the distinct source events are:

| Event | Native index / frame | Coast elapsed, s | Remaining arc, m | Measured speed, m/s | Next throttle / brake |
| --- | --- | ---: | ---: | ---: | --- |
| Feedback activation latch | 1008 / 2935 | 9.750000145286322 | 5.535031551698978 | 0.7453615965394953 | 0 / 0 |
| First retained positive request | 1113 / 3040 | 15.000000223517418 | 2.7941566827801125 | 0.34991845440360575 | 0.00010600927635096014 / 0 |
| Last positive request | 1167 / 3094 | 17.700000263750553 | 1.9623238199831974 | 0.2998417402531554 | 0.004210979212075472 / 0 |
| First subsequent zero-throttle request | 1168 / 3095 | 17.75000026449561 | 1.947171858778944 | 0.2993148569986889 | 0 / 0 |
| First measured goal-window entry | 1241 / 3168 | 21.40000031888485 | 0.9963052178268299 | 0.1996039992915702 | 0 / 0 |
| Original measured hold latch | 1340 / 3267 | 26.350000392645597 | 0.29444675971834045 | 0.09953850693595595 | 0 / 1 |

There are 55 consecutive positive terminal requests, indices 1,113–1,167. Their source timestamps span 2.7000000402331352 s; including the interval commanded by the last source gives 2.7500000409781933 s. The maximum stored throttle is 0.06285110861063004 at index 1,150, below the fixed 0.15 request limit. The mathematical upward slew is independently replayed from measured timestamps; float32 storage is checked separately. Requests decrease without a downward slew restriction.

Throttle is already zero about 1.95 m before the endpoint and remains zero through goal entry and hold. After entering the goal window, B travels another 0.7018584581084895 m of route arc over 4.950000073760748 s before speed first meets the original 0.1 m/s hold criterion. B finishes 0.7032271693016996 m closer to the endpoint than A, but requires 39 additional native ticks, or 1.95 scheduled seconds. Closer position alone is not a better-quality trajectory.

The 0.35 m/s value is a target, not a guaranteed measured-speed cap: activation itself occurs at 0.7453615965 m/s. The existing longitudinal PID receives the changed target from the activation latch, even while the overridden final throttle remains zero. No direct PID-memory rewrite or extra hook was introduced, but identical internal PID numeric histories are not claimed. The exact prefix above concerns observed physical state and serialized control, not hidden controller state.

## Added moving failures, not only a standstill singularity

All 16 B curvature endpoints are retained in the frozen summary:

| Observed endpoint phase | Episode-relative 100 ms ticks | Unique endpoints | Overlapping failing steps |
| --- | --- | ---: | ---: |
| Launch | 50, 51 | 2 | 51 |
| Terminal feedback | 557, 558, 561, 563, 564, 572, 573, 574, 576, 577, 578, 579, 580 | 13 | 823 |
| Service-brake hold | 671 | 1 | 64 |

The original launch incidences remain unchanged. The 13 newly observed terminal-feedback endpoints occur during the positive-pedal portion, with 100 ms virtual-base displacements from 30.059895 to 34.366579 mm. Their maximum curvature is 0.43944803193615267 rad/m. These are moving, centimetre-scale segments, more than 300 times the unchanged 0.1 mm curvature-assessment threshold; the additional failures cannot all be described as a near-zero standstill issue.

Across the 55 positive-request source observations, speed ranges from 0.29733855252813546 to 0.34991845440360575 m/s. Without smoothing, these samples contain 13 interior local maxima and 14 interior local minima (strict comparison with the preceding sample and the corresponding non-strict comparison with the following sample). Over the first-positive-source through pre-hold observations, recorded yaw spans 1.5701433196956092–1.570707909846706 rad, yaw rate spans -0.03149236466872917–0.03163366200931469 rad/s, and actor pitch spans -0.003114566206932068–0.0020149056799709797 degrees. These are sampled fluctuations, not identification of a tire, contact, integrator or driveline mechanism. Both observed front-wheel steering angles and final requested steering remain zero from the first positive request onward.

At the first following positive-throttle observation, B speed is 0.3349877435635239 m/s, down from 0.34991845440360575 at the source. Its actor/base position-minus-signed-trapezoidal-velocity residual norms are 0.278344 / 0.373995 mm; A's corresponding interval gives 0.051963 / 0.254013 mm. A positive request therefore is not relabeled as an immediately observed acceleration or a physically exact actuation response.

## Hold-boundary geometry remains unresolved

B's first hold interval, frames 3267→3268, retains current brake 0 at the source, requested brake 1, and observed brake 1 at the following frame. Speed falls from 0.09953850693595595 to 0.00008788669645513435 m/s. In world ROS XY:

- Actor displacement: [-0.0001983642578125, 0.000946044921875] m.
- Virtual-base displacement: [-0.0008395301091468355, 0.0009464989548462199] m.
- Actor signed trapezoidal velocity integral: [0.000002444570676563531, 0.002432810217086773] m.
- Virtual-base signed trapezoidal velocity integral: [0.0005344674107366178, 0.0024325599825561813] m.
- Actor/base residual norms: 1.500265 / 2.023919 mm, versus A's 1.365006 / 1.922630 mm at its own hold boundary.

B's measured yaw change is -0.00044994106616891294 rad while integrated endpoint yaw rates give +0.0003733494027341623 rad. Actor pitch changes from -0.00021856604143977165 to -0.006133509334176779 degrees. The recorded base-minus-actor displacement change in X is -0.641166 mm; the declared yaw-first, old-pitch decomposition accounts for essentially that X change, while the pitch-second X contribution is approximately 7.5e-12 m. The existing planar omega-cross-offset correction was already applied; its signed interval X contribution is +0.532023 mm. This mismatch is preserved, not assigned a unique physical cause or repaired by a new angular-axis assumption.

B's hold source is native index 1,340, an existing even-index camera anchor. A's hold source is odd index 1,301. The worst B curvature is the original first future point of anchor frame 3267, not a manufactured three-position curve: its anchor-relative displacement is [0.0009461107020756693, 0.0008399752911273162] m, norm 0.0012651813902703974 m, and heading difference from anchor body-forward is 0.7260444991489939 rad. Their original ratio is 573.8659331637988 rad/m. The unchanged interpolation targets 71.470071158 s and retains the native 50 ms midpoint. All 64 lateral-acceleration failure incidences occur at this same 100 ms endpoint. Different event/camera phase and millimetre geometry are relevant observations, not permission to shift cameras, drop a first point, waive the flag, or ascribe the whole difference to sampling alone.

Both holds retain 170 following native intervals through the original dwell and tail, including three backward virtual-base intervals each. Exactly zero planar speed first occurs nine native intervals after each hold source. Sustained later standstill does not prove physical consistency at the transition.

## Interpretation and authentication

The failed v8 fine-5ms case motivated concern about an out-of-goal stall, but v9 does not reproduce that condition: physics is held at reference-10ms, and the untreated coast case already completes. This pair consequently does not demonstrate rescue of a reproduced early stall. It shows that this prospective feedback law executes and completes on this fixed Town07 condition while adding moving curvature failures and retaining a worse hold-boundary result. Do not adopt B or admit either case. One pair does not establish repeatability, all-map ability or a unique physical cause.

The new frozen analysis seal verifies all six payloads. Native comparisons and interval calculations above were independently repeated from finalized raw files; the full-future counts are taken from the sealed, unchanged full auditor. The completed A full JPEG/ACK/protocol/future audit was independently rerun; B's full JPEG result is the frozen full-auditor result, not an additional pixel rerun performed for this note. All opened raw/source hashes were checked again. This note is the only new file written by this analysis.

| Evidence | SHA-256 |
| --- | --- |
| Prospective v9 plan | 11fd19f89db1dac0aaab203bbec845528e6e4be4c26101ce4069be9590b94087 |
| A states | ddc62e6f37d8880843e1f2996971387a786149572bc00c1d5d4a39cf0c683ba8 |
| B states | 62f1856762f1fdd3f1c709934dd9f9200891ca7b95b6b60b0d9b60aba771cf22 |
| B manifest | 2582c2a1069e4c1ea001e309510e31cc9ef8025453636ddd4d6ba32063cf165b |
| B owner result | 0d1e94ec437d39989bc6c807541ab4967670d05334653e6ac68536f8ec60a320 |
| Full summary | c0d0320b78e5b81463a7a2bb8b385573cff1b37acdd4b58298cd270d2e3530bc |
| Full future-window payload | 4e2a3c23ab089e5964baec0618ded2c2b1b17f69ce9172aa57cd42c3389c1c25 |
| Six-payload analysis seal | e9b7f9112bc1b18e3f3d16fb82d3250649b4b68f21595e6cd8d642f9d5769dae |
| Frozen v9 helper | fd37f4ce1b937f7a0ce27f9a680af459216b023510b5a7eb90fb8f25947f2428 |
| Frozen v9 auditor | 4a2ae58fd8dd30775fa4eb34a6a1d76a44e339c2ef829d55511aa6297f2544fa |
| Native-boundary pure interval helper | e72d07671bda2e854e78416cfb9e0e21c1fdd2802b0b248857e4de2d0795e34b |
