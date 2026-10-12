# HH_261001 — Fixed directional-gradient A/B accounting

Both fixed endpoints recounted without a new neural forward, training update, TEST read or live run. Neither model is promoted.

A and B each executed 2,999 updates. A reproduces the exact W1 model tensor state. All 34 archived source files and both numeric export states are verified.

| Fixed TRAIN group | n | A oracle match | B oracle match | A first5 speed MAE | B first5 speed MAE |
| --- | ---: | ---: | ---: | ---: | ---: |
| original_immediate_start18 | 18 | 15 | 15 | 0.249446 | 0.252197 |
| protected_stop_hold200 | 200 | 179 | 180 | 0.269596 | 0.260636 |
| moving_speed_1p5_to_2p5 | 16 | 16 | 16 | 0.160660 | 0.151840 |
| right_fixed_probes | 186 | 172 | 173 | 0.195626 | 0.188503 |

| Known DEV metric | A | B | B − A |
| --- | ---: | ---: | ---: |
| combined_val / ade_6p4s_m | 2.270810 | 2.019402 | -0.251408 |
| combined_val / fde_6p4s_m | 5.968201 | 5.950600 | -0.017601 |
| combined_val / speed_mae_6p4s_mps | 0.970120 | 0.955305 | -0.014816 |
| town03_right_turn_dev / ade_6p4s_m | 4.169466 | 3.762721 | -0.406746 |
| town03_right_turn_dev / fde_6p4s_m | 11.362698 | 11.160899 | -0.201799 |
| town03_right_turn_dev / speed_mae_6p4s_mps | 1.693151 | 1.680579 | -0.012572 |
| town03_straight_dev / ade_6p4s_m | 0.320056 | 0.228249 | -0.091808 |
| town03_straight_dev / fde_6p4s_m | 0.425684 | 0.597336 | +0.171652 |
| town03_straight_dev / speed_mae_6p4s_mps | 0.227251 | 0.210130 | -0.017121 |

| B scheduled batch | Lower saturated | Inward | Outward blocked | Zero | Minimum z |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 10 | 0 | 0 | 10 | -0.271943 |
| 1000 | 8 | 0 | 0 | 8 | -0.245142 |
| 2000 | 0 | 0 | 0 | 0 | 0.281121 |
| 3000 | 0 | 0 | 0 | 0 | 0.349223 |

These four trace samples include unselected and zero-weight paths. They are not global training rates or evidence of safer motion.

Original absolute gates: A 6/9; B 6/9.

All denominators, failures, individual changes, geometry gates and STOP/tail provenance remain in comparison.json. Live driving and future physical body sweeps were not evaluated.
