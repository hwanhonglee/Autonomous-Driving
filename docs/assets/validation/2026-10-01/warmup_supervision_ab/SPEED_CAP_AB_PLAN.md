# W1 speed-cap study: operating regime, not new weights

<!-- HH_261001 - Declare both new trials before either run, including the common teacher-speed change. -->

W1 advanced200.51m but failed to finish the210.60m TRAIN route in150s at a
9km/h cap. Original source TRAIN collection was30km/h. This motivates a
bounded operating-regime comparison; it does not assume cap mismatch causes
the failure, and does not replace the planned valid low-speed training data.

Run ONE20s trial each, ordered C9 then C30, using the SAME W1 checkpoint,
Town07 TRAIN route, ClearNoon/seed0/Epic, 20Hz physics and10Hz6-camera inference,
original camera/weights/controller gains/argmax/runtime gates. The sole factor
between these two NEW runs is tracker speed_cap_mps:2.5 vs30/3.6. Both retain
the physical decoder and unchanged runtime speed bound. No speed floor or
candidate substitution. Use the original tracker with measured wheel geometry.

Set BasicAgent teacher speed to30km/h in BOTH arms. This common change is
necessary because the teacher's normal overspeed brake is also the original
emergency override: leaving it at9km/h would invalidate the30km/h condition.
Do not disable that override or original collision/lane/CTE/staleness gates.
The teacher's startup and all later proposals/overrides are recorded separately.
Thus previous W1 trials with teacher9 are NOT the matched C9 control.

Same20s simulated total/240s owner worker wall budget, 1.2s stationary warmup,
5GiB free-space start guard, fixed route and2440–2442 owned packaged CARLA.
Only this explicit new worker allows the declared30km/h cap; older workers and
their9km/h guards remain untouched. No60km/h, new map, real vehicle, signal
override, installation or automatic follow-up launch is authorized by this plan.

Retain every state, camera JPEG, raw candidate output, selected plan, ACK,
intervention, source and cleanup receipt. Save actual model inputs at fixed
inference indices0,10,20,... in both arms to limit disk use. These are a subset,
not proof of all-input replay. Measure net progress, speed, low-speed/terminal
duration, actual candidate switches, CTE/body/inset-sweep margins and latency.
Keep both full-duration centered GIFs/PNGs, including unsuccessful intervals.
Repeat neither arm to select a favorable outcome. No automatic checkpoint
promotion, TRAIN admission or main/subfunction approval from these diagnostics.
