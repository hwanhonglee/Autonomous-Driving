# W1 fixed longer-horizon Town07 diagnostic

<!-- HH_261001 - Register a new study without changing the earlier 20-second pair or its guards. -->

W1 checkpoint b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76
advanced29.756804593m in the completed20s pair, versus W0's4.227276041m.
This motivates ONE new150s-total diagnostic on the identical210m Town07 TRAIN
route. This is not a second matched A/B and not independent generalization.
The earlier30s worker restriction remains unchanged in its original source;
this new worker explicitly implements only this150s prospective protocol.

Keep W1 weights, six cameras, ClearNoon/seed0/Epic,20Hz physics/10Hz camera,
9km/h cap, raw candidate argmax, tracker and original emergency/teacher policy.
No speed floor, alternative candidate, signal override or new training data.
Initial expert startup remains disclosed. Reaching the positional goal is NOT
proof of learned goal-stop: original setup/cleanup braking remains external.
Keep every native state, six JPEGs, all model outputs, ACKs and interventions.
Archive actual six-field model input tensors at inference indices0,10,20,...
(fixed1Hz subset, not all inference inputs) to bound local storage use.
These lossless sampled tensors supplement, not replace, full trajectory logs.
Do not claim unsampled or bit-identical numerical replay from this subset.

Fixed limits:150s simulated total,900s worker wall time,>=5GiB free at start;
hard emergency on existing collision/lane events, |CTE|>1m or runtime failure.
No existing quality threshold is weakened. One exclusively owned packaged CARLA
server uses2440–2442, then its own process groups are stopped and source/port
receipts retained. No foreign process termination. No retrospective extension.

Report total and phase-specific progress, speed/terminal stall, intervention
count, original capture outcome, all-body/snapshot-linear-sweep containment,
latency and full-duration vehicle-centered visual evidence. Preserve failures.
No automatic promotion, training admission, all-map,30km/h or real-car approval.
