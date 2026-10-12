# 전후 전체 비교: 같은 고정 자료에서 평가

새 TRAIN은 재평가, 새 HOLDOUT은 미학습 같은 Town07 경로입니다. 기존 known DEV는 이미 사용한 진단 자료입니다.

## 새 회복 9개 TRAIN / 2개 HOLDOUT / 기존 저속 327개

| 자료 | 기록 | 분모 | ADE 부모→최종 (m) | FDE 부모→최종 (m) | 순간 속도 MAE 부모→최종 (m/s) | raw 속도 MAE 부모→최종 (m/s) |
|---|---|---:|---:|---:|---:|---:|
| recovery_train | carla_town07_town07_signed_recovery_train_00_ClearNoon_s2026100710_4847458ce66b | 302 | 0.646399 → 0.048311 | 1.191190 → 0.105222 | 0.213994 → 0.020003 | 1.754388 → 0.020788 |
| recovery_train | carla_town07_town07_signed_recovery_train_01_ClearNoon_s2026100711_1d087d0ec5d1 | 302 | 0.645351 → 0.046930 | 1.189251 → 0.103575 | 0.213721 → 0.020118 | 1.754913 → 0.020871 |
| recovery_train | carla_town07_town07_signed_recovery_train_02_ClearNoon_s2026100712_4c59f3872ced | 302 | 0.646594 → 0.045368 | 1.190304 → 0.100165 | 0.214038 → 0.020778 | 1.749743 → 0.021377 |
| recovery_train | carla_town07_town07_signed_recovery_train_03_ClearNoon_s2026100713_ee072fdeaef0 | 302 | 0.641889 → 0.044979 | 1.182847 → 0.100682 | 0.212887 → 0.020483 | 1.743031 → 0.021164 |
| recovery_train | carla_town07_town07_signed_recovery_train_04_ClearNoon_s2026100714_f421d97cfecb | 302 | 0.641389 → 0.045480 | 1.181483 → 0.101159 | 0.212770 → 0.020449 | 1.739887 → 0.021190 |
| recovery_train | carla_town07_town07_signed_recovery_train_05_ClearNoon_s2026100715_3c461aebc905 | 302 | 0.636995 → 0.048440 | 1.174628 → 0.106442 | 0.211681 → 0.019774 | 1.747991 → 0.020516 |
| recovery_train | carla_town07_town07_signed_recovery_train_06_ClearNoon_s2026100716_105072a8936e | 302 | 0.641573 → 0.046122 | 1.181169 → 0.102269 | 0.212805 → 0.020333 | 1.745568 → 0.021119 |
| recovery_train | carla_town07_town07_signed_recovery_train_07_ClearNoon_s2026100717_b88900f9bd75 | 302 | 0.646137 → 0.047569 | 1.188022 → 0.103442 | 0.213903 → 0.020126 | 1.758072 → 0.020816 |
| recovery_train | carla_town07_town07_signed_recovery_train_08_ClearNoon_s2026100718_b264a23159f2 | 302 | 0.641854 → 0.048889 | 1.180972 → 0.105888 | 0.212816 → 0.020340 | 1.760916 → 0.020957 |
| recovery_holdout | carla_town07_town07_signed_recovery_holdout_00_ClearNoon_s2026100810_668ab812450f | 302 | 0.640457 → 0.044300 | 1.180879 → 0.098881 | 0.212509 → 0.020575 | 1.742935 → 0.021225 |
| recovery_holdout | carla_town07_town07_signed_recovery_holdout_01_ClearNoon_s2026100811_7696127af0ae | 302 | 0.642008 → 0.047507 | 1.182050 → 0.106290 | 0.212898 → 0.020613 | 1.747493 → 0.021320 |
| v4 | carla_town07_town07_straight_v4_research_train_ClearNoon_s2026100101_36b7c6dc98b9 | 327 | 0.078348 → 0.034600 | 0.176099 → 0.080718 | 0.024229 → 0.024056 | 0.027300 → 0.026662 |

## 기존 TRAIN 2487개 / known DEV 599개 — 모든 기록과 직진·회전

`raw_curvature`와 `raw_preferred_speed`는 원래 SmoothL1 손실입니다. m/s 단위인 값은 `raw_spatial_preferred_speed_mae_mps`뿐입니다. 기존 고속 자료는 decoder를 거치지 않았습니다.

| 자료 | 그룹/기록 | 분모 | 성분 | 부모 | 최종 | 변화 (최종−부모) |
|---|---|---:|---|---:|---:|---:|
| legacy_TRAIN2487_raw_only | aggregate | 2487 | total | 0.055563689 | 0.048560446 | -0.007003243 |
| legacy_TRAIN2487_raw_only | aggregate | 2487 | raw_curvature | 0.009454073 | 0.007097111 | -0.002356962 |
| legacy_TRAIN2487_raw_only | aggregate | 2487 | raw_preferred_speed | 0.053200170 | 0.046786168 | -0.006414002 |
| legacy_TRAIN2487_raw_only | aggregate | 2487 | raw_spatial_preferred_speed_mae_mps | 0.153081371 | 0.149810306 | -0.003271065 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | total | 0.079973209 | 0.102091625 | +0.022118416 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_curvature | 0.031288372 | 0.023865457 | -0.007422915 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_preferred_speed | 0.072151115 | 0.096125259 | +0.023974144 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_spatial_preferred_speed_mae_mps | 0.173469290 | 0.203362772 | +0.029893482 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | total | 0.004617905 | 0.004421450 | -0.000196455 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_curvature | 0.001085056 | 0.000887258 | -0.000197798 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_preferred_speed | 0.004346641 | 0.004199636 | -0.000147006 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_spatial_preferred_speed_mae_mps | 0.054904185 | 0.063518307 | +0.008614122 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | total | 0.005532415 | 0.005686394 | +0.000153979 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_curvature | 0.000999798 | 0.000891230 | -0.000108568 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_preferred_speed | 0.005282465 | 0.005463586 | +0.000181121 |
| legacy_TRAIN2487_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_spatial_preferred_speed_mae_mps | 0.077809417 | 0.073786218 | -0.004023198 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | total | 0.224590883 | 0.167710278 | -0.056880605 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_curvature | 0.029974739 | 0.020780235 | -0.009194504 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_preferred_speed | 0.217097198 | 0.162515219 | -0.054581979 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_spatial_preferred_speed_mae_mps | 0.392435279 | 0.341735627 | -0.050699652 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | total | 0.008154960 | 0.007211718 | -0.000943241 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_curvature | 0.001299511 | 0.001739892 | +0.000440381 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_preferred_speed | 0.007830082 | 0.006776745 | -0.001053337 |
| legacy_TRAIN2487_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_spatial_preferred_speed_mae_mps | 0.082914328 | 0.079001047 | -0.003913281 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | total | 0.116268289 | 0.072844057 | -0.043424232 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_curvature | 0.012429700 | 0.008732331 | -0.003697368 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_preferred_speed | 0.113160864 | 0.070660974 | -0.042499890 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_spatial_preferred_speed_mae_mps | 0.235547448 | 0.204180692 | -0.031366757 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | total | 0.020637797 | 0.030042210 | +0.009404413 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_curvature | 0.002216207 | 0.002271590 | +0.000055383 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_preferred_speed | 0.020083745 | 0.029474312 | +0.009390567 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_spatial_preferred_speed_mae_mps | 0.162666136 | 0.141903857 | -0.020762279 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | total | 0.009173204 | 0.019353263 | +0.010180058 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_curvature | 0.002109656 | 0.001813593 | -0.000296063 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_preferred_speed | 0.008645790 | 0.018899864 | +0.010254074 |
| legacy_TRAIN2487_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_spatial_preferred_speed_mae_mps | 0.076224970 | 0.121097650 | +0.044872680 |
| legacy_TRAIN2487_raw_only | curved | 328 | total | 0.087138884 | 0.100467030 | +0.013328146 |
| legacy_TRAIN2487_raw_only | curved | 328 | raw_curvature | 0.039723740 | 0.030170319 | -0.009553421 |
| legacy_TRAIN2487_raw_only | curved | 328 | raw_preferred_speed | 0.077207949 | 0.092924450 | +0.015716501 |
| legacy_TRAIN2487_raw_only | curved | 328 | raw_spatial_preferred_speed_mae_mps | 0.195957722 | 0.213696897 | +0.017739175 |
| legacy_TRAIN2487_raw_only | straight | 2159 | total | 0.050766716 | 0.040674684 | -0.010092032 |
| legacy_TRAIN2487_raw_only | straight | 2159 | raw_curvature | 0.004855439 | 0.003591779 | -0.001263660 |
| legacy_TRAIN2487_raw_only | straight | 2159 | raw_preferred_speed | 0.049552856 | 0.039776740 | -0.009776117 |
| legacy_TRAIN2487_raw_only | straight | 2159 | raw_spatial_preferred_speed_mae_mps | 0.146567502 | 0.140104515 | -0.006462987 |
| previously_exposed_known_DEV599_not_independent | aggregate | 599 | total | 0.141593428 | 0.181520717 | +0.039927289 |
| previously_exposed_known_DEV599_not_independent | aggregate | 599 | raw_curvature | 0.023550019 | 0.024391460 | +0.000841441 |
| previously_exposed_known_DEV599_not_independent | aggregate | 599 | raw_preferred_speed | 0.135705923 | 0.175422852 | +0.039716929 |
| previously_exposed_known_DEV599_not_independent | aggregate | 599 | raw_spatial_preferred_speed_mae_mps | 0.315628865 | 0.385127660 | +0.069498795 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | total | 0.242171650 | 0.287139111 | +0.044967462 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_curvature | 0.048576114 | 0.049393956 | +0.000817842 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_preferred_speed | 0.230027621 | 0.274790622 | +0.044763001 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_spatial_preferred_speed_mae_mps | 0.364718804 | 0.408089347 | +0.043370542 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | total | 0.058493738 | 0.094256739 | +0.035763000 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_curvature | 0.002872972 | 0.003733910 | +0.000860939 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_preferred_speed | 0.057775496 | 0.093323261 | +0.035547766 |
| previously_exposed_known_DEV599_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_spatial_preferred_speed_mae_mps | 0.275069799 | 0.366156267 | +0.091086468 |
| previously_exposed_known_DEV599_not_independent | curved | 85 | total | 0.059605267 | 0.060897544 | +0.001292277 |
| previously_exposed_known_DEV599_not_independent | curved | 85 | raw_curvature | 0.113846542 | 0.121546187 | +0.007699646 |
| previously_exposed_known_DEV599_not_independent | curved | 85 | raw_preferred_speed | 0.031143631 | 0.030510997 | -0.000632634 |
| previously_exposed_known_DEV599_not_independent | curved | 85 | raw_spatial_preferred_speed_mae_mps | 0.119148198 | 0.117914463 | -0.001233735 |
| previously_exposed_known_DEV599_not_independent | straight | 514 | total | 0.155151781 | 0.201468129 | +0.046316348 |
| previously_exposed_known_DEV599_not_independent | straight | 514 | raw_curvature | 0.008617715 | 0.008325017 | -0.000292698 |
| previously_exposed_known_DEV599_not_independent | straight | 514 | raw_preferred_speed | 0.152997353 | 0.199386875 | +0.046389522 |
| previously_exposed_known_DEV599_not_independent | straight | 514 | raw_spatial_preferred_speed_mae_mps | 0.348120804 | 0.429316613 | +0.081195809 |
