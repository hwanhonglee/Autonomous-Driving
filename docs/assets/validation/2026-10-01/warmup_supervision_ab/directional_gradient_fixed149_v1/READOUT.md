# HH_261001 — Directional-gradient fixed149 CPU counterfactual

Both released final endpoints were replayed on all 149 predeclared archived W1-long inputs. A reproduced all original raw output tensors and all original selected trajectories, runtime gates and tracker proposals exactly. The replay ran on CPU with two threads, using the original inference model and original 9 km/h-cap controller. No simulator, GPU, model update, alternative target, outcome filter or new actuator acknowledgement was used.

The endpoint binding is `a6b674a471f2bec5c1c153d912a61fac5ecffae72ef797f6cbc9a86574efac8e`. A has the original W1 state `ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f`; B has state `6964bc82fbf4eb5742f732aaa05e01967f0b8e69a177652312fa5e251e7cf0ab`. Both complete training receipts, all 42 restricted-export evidence files, both numeric NPZ state digests, replay dependencies and snapshot hashes were checked. All checked bytes remained unchanged after replay.

| Fixed historical inputs, n = 149 | A/native | B/directional |
| --- | ---: | ---: |
| Selected trajectory runtime-gate pass | 149 | 149 |
| All individual candidates passing runtime gates | 1,788 | 1,788 |
| Selected DRIVE / STOP | 148 / 1 | 148 / 1 |
| Exact-zero 0.3-second speed previews | 40 | 6 |
| Speed previews at or below 0.05 m/s hold threshold | 40 | 8 |
| Positive-throttle proposals | 99 | 135 |
| Positive-brake proposals | 50 | 14 |
| Mean selected first-3 speed, m/s | 1.654454 | 1.676355 |
| Mean selected first-20 speed, m/s | 2.441233 | 2.369454 |
| Mean selected all-64 speed, m/s | 4.784865 | 4.338972 |

There are 37 raw-argmax changes. Of the 40 original exact-zero previews, 33 become greater than 0.05 m/s, five remain exactly zero and two become positive but remain below the controller's hold threshold. Among the 33 above-threshold cases, 27 change candidate index; the old A-selected candidate still has exactly zero preview under B in every one of those 27 cases. Six improve without changing the selected candidate index. Those six are sequences 117, 123, 128, 133, 135 and 137, all at the same historical W1 stalled progress of 200.827611 m; B proposes approximately 0.2013–0.2054 m/s there. These correlated historical samples are not six independent recoveries or new physical motion.

One regression is preserved explicitly: sequence 3, historical progress 4.667628 m, changes candidate 5 to 1 and changes the preview from 0.572454 m/s to exactly zero. The resulting counterfactual proposal changes from throttle 0.400718 to brake 0.3. Sequences 58 and 64 change zero to 0.017772 and 0.004779 m/s respectively, but still propose brake 0.3 under the unchanged hold rule.

The independent fixed-result report is in `../directional_gradient_comparison_v1/`. Both arms executed exactly 2,999 updates, A reproduces the historical W1 fixed TRAIN-probe payloads exactly, and both arms pass 6/9 original absolute DEV metric gates plus 3/3 decoder-geometry checks. B lowers eight of nine known DEV errors, but still fails all three right-turn metric gates: ADE 3.762721 m, FDE 11.160899 m and speed MAE 1.680579 m/s. Immediate-start TRAIN first-five speed MAE worsens from 0.249446 to 0.252197 m/s; protected STOP/hold improves from 0.269596 to 0.260636 m/s. These are fixed known TRAIN/DEV observations, not independent validation or a TEST evaluation.

All four prospectively selected gradient-trace batches record zero inward events. Their lower-saturated counts are 10, 8, 0 and 0, with zero incoming gradient at every counted lower-saturated element. The traces include unselected and zero-weight paths and cover only four batches; they do not establish an absence of inward events across the full fit. The endpoint and replay differences do not by themselves identify which training examples or gradient events caused them.

This is an offline counterfactual on W1 states, not live B performance, proof of smoother motion, physical lane-body clearance, deadline compliance, training admission or model promotion. Runtime trajectory gates here are not physical future body sweeps. There are zero new A/B control acknowledgements and zero new simulator states. The full unfiltered row evidence, including regressions, is retained in `records.jsonl`.

Evidence digests:

- `summary.json`: `84a6e83414d96cfdc52d78260195b0fd3d44711d5eb7c1dfde013303d55f9268`
- `records.jsonl`: `6d8d6a405d6b67df9731fde7a940ba20152147fe6db2c2a2a0ff5f33ec6d5ec5`
- Independent `comparison.json`: `176f188b68368c2675698efafdd2e97ed01bbbab69d37316c815ce03f1ef79b8`

The records occupy 1,925,925 bytes; the fixed149 replay and independent report together occupy under 3 MiB. The frozen reporter/replay test set passed 28 focused tests before the released outcome was read.
