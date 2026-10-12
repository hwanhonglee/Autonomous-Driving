# HH_261001 — Measured fixed-input W1/W2 comparison

All 149 fixed snapshots completed on CPU with two compute threads. W1 reproduced all 149 published raw-output hashes, selected objects, all-candidate trajectory gates and actual learned tracker commands exactly. W2 was evaluated on those same recorded W1 states; no W2 simulator state or actuator ACK exists here. Source, endpoint and snapshot hashes were unchanged. Five focused tests passed. Generated evidence is about 1.95 MB, below the 10 MiB limit.

| Measured same-input output | W1 | W2 |
|---|---:|---:|
| Raw selected DRIVE / STOP | 148 / 1 | 148 / 1 |
| Selected 64-point trajectory-gate passes | 149 / 149 | 149 / 149 |
| All-candidate trajectory-gate passes | 1788 / 1788 | 1788 / 1788 |
| Exactly zero selected 0.3 s preview | 40 | 14 |
| Positive-throttle proposals | 99 | 124 |
| Positive-brake proposals | 50 | 25 |
| Mean selected speed, first 3 points, m/s | 1.65445 | 1.69059 |
| Mean selected speed, first 20 points, m/s | 2.44123 | 2.54871 |
| Mean selected speed, all 64 points, m/s | 4.78486 | 4.91338 |
| Mean selected path length, all 64 points, m | 30.62313 | 31.44562 |

Raw argmax changed in 31/149 inputs, but DRIVE/STOP category did not change. Of W1's 40 exactly zero-preview inputs, W2 retained zero in 14 and proposed positive throttle in 26. Those 26 all switched candidate index 0 → 5; their new preview speeds were 0.362712–0.410419 m/s. Crucially, W2's candidate index 0 remained exactly zero at the preview on all 26 inputs. This supports a directly observed selection change, not a claim that additional training repaired that candidate's lower-clamp gradient blockage.

Among the 47 inputs with historical W1 |vx| ≤ 0.1 m/s, selected zero-preview counts were W1 39 and W2 13. The extra zero in the full sample is outside that near-stationary subset; do not conflate these counts with the prior 39 selected-DRIVE gradient cases.

The raw prefix means above are predictions, not target errors or observed motion. Passing computational trajectory gates does not establish lane-body clearance, comfort, live deadline performance, route progress, new-data admission or improved closed-loop driving. Unselected outputs are retained as hashes, preview arrays and complete gate results; full selected XY/speed arrays are in the row journal. No reranking, controller change, filtering, invented target, training update, GPU, SSH or simulator launch was used.

Evidence: `summary.json` SHA256 `8fc7c3c6e0d202f5782f651d82b40b3e42a9c2e693a03e27facd1d6936a35701`; `records.jsonl` SHA256 `4fe43706616530aab02146a6e34e2c72966d4f50efc3f7b24ddee30600b37a53`. Replay source `tools/e2e/replay_w2_fixed149_counterfactual_20261001.py` SHA256 `436ad5c7b7e20d2ca6722e8ddc54cbaaa9a9b536a142922b1bb6e439cfbcc479`; tests SHA256 `4dc49bc26398bddf36e59c2c7f559b06a07154845e2cd9e2a4441e925632bc74`.
