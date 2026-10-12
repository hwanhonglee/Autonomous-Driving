# Frozen selector-only live comparison

<!-- HH_261001 - Register identical bounded driving conditions before selector fitting or evaluation. -->

After both fixed final selector checkpoints have been saved and their frozen-generation
identity verified, export both to the data-only NPZ contract, pinned to their own exact
checkpoint, array and metadata SHA256. Parent is the original AH checkpoint
4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91, not the regressed
initial-acceleration B.

Run S0_uniform_selector then S1_phase_balanced_selector exactly once each on the
unchanged owned Town07 AH-v2 TRAIN route, original six-camera geometry/postprocess,
20 Hz physics, 10 Hz camera/inference, 20 s total duration, 9 km/h research cap,
seed0, ClearNoon/Epic, raw argmax, identical external tracker and unchanged runtime
gates. Expert commands only supply the same initial history/start handoff; later
expert proposals are logged but never substituted as driving success.

Keep all 400 native states, 200 six-camera frames, all12 raw outputs, selected
plans, command ACK/next-state alignment, interventions, failures and original
owner shutdown receipts. Do not skip a poorly scoring arm or alter speed preview,
minimum speed, fallback selection or time budget after results. Report actual
progress, full low-speed and signed-velocity history, events, latency, source
identity and full20s centered PNG/GIF. A short terminal exact-zero tail is not
recovery if low-speed forward/backward chatter dominates the record.

Original owner Err98 or any lifecycle failure is retained; a separate later
read-only process/port/source check may supplement but not overwrite it.
This is one known-TRAIN-route diagnostic per candidate, not route completion,
independent generalization, all-body-lane proof, 30km/h approval or real-vehicle use.
No automatic model promotion, training-data admission or feature completion.
