# Moving direction-balanced replay F comparison

- Verdict: `FAIL_OPEN_LOOP_DIAGNOSTIC_GATE`
- Scope: fixed DEV, open-loop trajectory evaluation only
- Lower is better for ADE, FDE, and speed MAE
- No TEST, closed-loop, export, promotion, head-addition, all-map, or vehicle claim

## Strict Pareto gate

| Scope | Reference | Metric | F | Maximum | Result |
|---|---:|---|---:|---:|---|
| right_turn | C | ade_6p4s_m | 2.8673983425 | 2.93531437488 | PASS |
| right_turn | C | fde_6p4s_m | 7.17653207689 | 7.79386611402 | PASS |
| right_turn | C | speed_mae_6p4s_mps | 0.91737966633 | 1.10761542331 | PASS |
| straight | D | ade_6p4s_m | 7.2097485189 | 3.76474018795 | FAIL |
| straight | D | fde_6p4s_m | 17.4842483067 | 11.0274412436 | FAIL |
| straight | D | speed_mae_6p4s_mps | 2.77203306083 | 1.78775049601 | FAIL |
| combined | D | ade_6p4s_m | 5.00918938612 | 3.56324554776 | FAIL |
| combined | D | fde_6p4s_m | 12.2606393901 | 10.3535367917 | FAIL |
| combined | D | speed_mae_6p4s_mps | 1.83215618877 | 1.63502205323 | FAIL |

## Geometry gate

| Measure | F count | Required | Result |
|---|---:|---:|---|
| selected | 664/665 | >= 664/665 | PASS |
| any_candidate | 664/665 | >= 664/665 | PASS |
| all_candidates | 664/665 | >= 664/665 | PASS |

## F versus C

| Scope | Metric | Reference | F | F-reference | Change | Direction |
|---|---|---:|---:|---:|---:|---|
| right_turn | ade_6p4s_m | 2.93531437488 | 2.8673983425 | -0.0679160323801 | -2.313757% | improved |
| right_turn | fde_6p4s_m | 7.79386611402 | 7.17653207689 | -0.617334037135 | -7.920768% | improved |
| right_turn | speed_mae_6p4s_mps | 1.10761542331 | 0.91737966633 | -0.190235756979 | -17.175254% | improved |
| straight | ade_6p4s_m | 8.09086501308 | 7.2097485189 | -0.881116494173 | -10.890263% | improved |
| straight | fde_6p4s_m | 19.3253297689 | 17.4842483067 | -1.84108146226 | -9.526779% | improved |
| straight | speed_mae_6p4s_mps | 3.04879499645 | 2.77203306083 | -0.276761935615 | -9.077748% | improved |
| combined | ade_6p4s_m | 5.47820267269 | 5.00918938612 | -0.469013286578 | -8.561445% | improved |
| combined | fde_6p4s_m | 13.4815657245 | 12.2606393901 | -1.2209263344 | -9.056265% | improved |
| combined | speed_mae_6p4s_mps | 2.06506944949 | 1.83215618877 | -0.232913260711 | -11.278713% | improved |

## F versus D

| Scope | Metric | Reference | F | F-reference | Change | Direction |
|---|---|---:|---:|---:|---:|---|
| right_turn | ade_6p4s_m | 3.3671321111 | 2.8673983425 | -0.499733768603 | -14.841525% | improved |
| right_turn | fde_6p4s_m | 9.69762996149 | 7.17653207689 | -2.5210978846 | -25.997052% | improved |
| right_turn | speed_mae_6p4s_mps | 1.48637246661 | 0.91737966633 | -0.568992800281 | -38.280634% | improved |
| straight | ade_6p4s_m | 3.76474018795 | 7.2097485189 | +3.44500833096 | +91.507200% | regressed |
| straight | fde_6p4s_m | 11.0274412436 | 17.4842483067 | +6.45680706312 | +58.552178% | regressed |
| straight | speed_mae_6p4s_mps | 1.78775049601 | 2.77203306083 | +0.984282564826 | +55.057043% | regressed |
| combined | ade_6p4s_m | 3.56324554776 | 5.00918938612 | +1.44594383836 | +40.579405% | regressed |
| combined | fde_6p4s_m | 10.3535367917 | 12.2606393901 | +1.90710259833 | +18.419818% | regressed |
| combined | speed_mae_6p4s_mps | 1.63502205323 | 1.83215618877 | +0.19713413554 | +12.056971% | regressed |

## F versus E

| Scope | Metric | Reference | F | F-reference | Change | Direction |
|---|---|---:|---:|---:|---:|---|
| right_turn | ade_6p4s_m | 3.52332785381 | 2.8673983425 | -0.655929511308 | -18.616761% | improved |
| right_turn | fde_6p4s_m | 8.96875929275 | 7.17653207689 | -1.79222721587 | -19.983001% | improved |
| right_turn | speed_mae_6p4s_mps | 1.22475389805 | 0.91737966633 | -0.307374231716 | -25.096816% | improved |
| straight | ade_6p4s_m | 5.60837236701 | 7.2097485189 | +1.60137615189 | +28.553314% | regressed |
| straight | fde_6p4s_m | 15.109218254 | 17.4842483067 | +2.37503005273 | +15.719080% | regressed |
| straight | speed_mae_6p4s_mps | 2.47595361384 | 2.77203306083 | +0.296079446993 | +11.958198% | regressed |
| combined | ade_6p4s_m | 4.55174072371 | 5.00918938612 | +0.457448662403 | +10.049972% | regressed |
| combined | fde_6p4s_m | 11.9974367598 | 12.2606393901 | +0.263202630219 | +2.193824% | regressed |
| combined | speed_mae_6p4s_mps | 1.84188699328 | 1.83215618877 | -0.00973080450431 | -0.528306% | improved |

## Interpretation boundary

This report compares fixed-DEV open-loop trajectory metrics only. A passing gate does not authorize TEST use, closed-loop driving, export, promotion, head addition, or vehicle control.
