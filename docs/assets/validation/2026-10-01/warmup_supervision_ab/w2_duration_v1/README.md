# HH_261001 — Fixed W1 versus W2 duration comparison

W2 worsened all nine frozen 6.4 s DEV metrics. Start oracle agreement stayed
15/18 while early-speed error worsened on 13/18 starts. STOP/hold agreement
improved 179/200 → 188/200. This is mixed TRAIN fit improvement with clear known-DEV
regression, not evidence that extending training solved the live driving failure.

Fixed results recounted; neither model is promoted. No new training, neural evaluation or TEST read was performed.

W1: 2,999 optimizer updates. W2: 11,996 updates across four unchanged schedules; exact first-block W1 state `ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f` verified by receipt chain.

Archived training closure: 27/27 files verified. W2 actual fit time: 1614.2 s. Export NPZ numeric digest independently matches the final trained-state receipt.

| Fixed TRAIN probe group | n | W1 oracle match | W2 oracle match | W1 early speed MAE | W2 early speed MAE |
| --- | ---: | ---: | ---: | ---: | ---: |
| original_immediate_start18 | 18 | 15 | 15 | 0.249446 | 0.261234 |
| protected_stop_hold200 | 200 | 179 | 188 | 0.269596 | 0.245079 |
| moving_speed_1p5_to_2p5 | 16 | 16 | 16 | 0.160660 | 0.160337 |
| right_fixed_probes | 186 | 172 | 175 | 0.195626 | 0.178011 |

Oracle means each model’s existing dynamic best-of-12 objective; not an invented moving-candidate label.

STOP/hold200 acquisition phases: {'driving': 200}; masked warmup anchors: 0; futures crossing imposed tail: 50. No genuine traffic/goal-stop claim is inferred.

| Fixed group | Oracle matches gained / lost | Early speed MAE improved / worsened / equal |
| --- | ---: | ---: |
| immediate_start18 | 0 / 0 | 3 / 13 / 2 |
| stop_hold200 | 12 / 3 | 25 / 6 / 169 |
| moving_1p5_to_2p5 | 0 / 0 | 4 / 12 / 0 |
| right_fixed | 5 / 2 | 20 / 14 / 152 |

| Known DEV metric | W1 | W2 | W2 − W1 |
| --- | ---: | ---: | ---: |
| combined_val / ade_6p4s_m | 2.270810 | 2.352248 | +0.081438 |
| combined_val / fde_6p4s_m | 5.968201 | 6.503628 | +0.535426 |
| combined_val / speed_mae_6p4s_mps | 0.970120 | 1.039527 | +0.069407 |
| town03_right_turn_dev / ade_6p4s_m | 4.169466 | 4.255285 | +0.085818 |
| town03_right_turn_dev / fde_6p4s_m | 11.362698 | 11.843679 | +0.480980 |
| town03_right_turn_dev / speed_mae_6p4s_mps | 1.693151 | 1.803852 | +0.110701 |
| town03_straight_dev / ade_6p4s_m | 0.320056 | 0.396994 | +0.076938 |
| town03_straight_dev / fde_6p4s_m | 0.425684 | 1.017051 | +0.591367 |
| town03_straight_dev / speed_mae_6p4s_mps | 0.227251 | 0.254230 | +0.026980 |

Original absolute gates: W1 6/9, W2 6/9. All original metrics, failures, denominators, geometry gates and individual probe changes are retained in `comparison.json`.

This is a bounded duration test on the same data, not a new data-coverage remedy, model approval or proof of a live cure.
