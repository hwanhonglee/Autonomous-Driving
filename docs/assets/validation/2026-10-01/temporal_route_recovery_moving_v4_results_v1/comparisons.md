# 전후 전체 비교: 같은 고정 자료에서 평가

새 TRAIN은 재평가, 새 HOLDOUT은 미학습 같은 Town07 경로입니다. 기존 known DEV는 이미 사용한 진단 자료입니다.

## 새 회복 9개 TRAIN / 2개 HOLDOUT / 기존 저속 327개

| 자료 | 기록 | 분모 | ADE 부모→최종 (m) | FDE 부모→최종 (m) | 순간 속도 MAE 부모→최종 (m/s) | raw 속도 MAE 부모→최종 (m/s) |
|---|---|---:|---:|---:|---:|---:|
| recovery_train | carla_town07_town07_signed_recovery_train_00_ClearNoon_s2026100710_4847458ce66b | 302 | 0.662276 → 0.646399 | 1.223697 → 1.191190 | 0.213994 → 0.213994 | 1.754388 → 1.754388 |
| recovery_train | carla_town07_town07_signed_recovery_train_01_ClearNoon_s2026100711_1d087d0ec5d1 | 302 | 0.661459 → 0.645351 | 1.223256 → 1.189251 | 0.213721 → 0.213721 | 1.754913 → 1.754913 |
| recovery_train | carla_town07_town07_signed_recovery_train_02_ClearNoon_s2026100712_4c59f3872ced | 302 | 0.663435 → 0.646594 | 1.227297 → 1.190304 | 0.214038 → 0.214038 | 1.749743 → 1.749743 |
| recovery_train | carla_town07_town07_signed_recovery_train_03_ClearNoon_s2026100713_ee072fdeaef0 | 302 | 0.658587 → 0.641889 | 1.215791 → 1.182847 | 0.212887 → 0.212887 | 1.743031 → 1.743031 |
| recovery_train | carla_town07_town07_signed_recovery_train_04_ClearNoon_s2026100714_f421d97cfecb | 302 | 0.658998 → 0.641389 | 1.217914 → 1.181483 | 0.212770 → 0.212770 | 1.739887 → 1.739887 |
| recovery_train | carla_town07_town07_signed_recovery_train_05_ClearNoon_s2026100715_3c461aebc905 | 302 | 0.655927 → 0.636995 | 1.215927 → 1.174628 | 0.211681 → 0.211681 | 1.747991 → 1.747991 |
| recovery_train | carla_town07_town07_signed_recovery_train_06_ClearNoon_s2026100716_105072a8936e | 302 | 0.661132 → 0.641573 | 1.221473 → 1.181169 | 0.212805 → 0.212805 | 1.745568 → 1.745568 |
| recovery_train | carla_town07_town07_signed_recovery_train_07_ClearNoon_s2026100717_b88900f9bd75 | 302 | 0.667173 → 0.646137 | 1.233615 → 1.188022 | 0.213903 → 0.213903 | 1.758072 → 1.758072 |
| recovery_train | carla_town07_town07_signed_recovery_train_08_ClearNoon_s2026100718_b264a23159f2 | 302 | 0.665267 → 0.641854 | 1.234416 → 1.180972 | 0.212816 → 0.212816 | 1.760916 → 1.760916 |
| recovery_holdout | carla_town07_town07_signed_recovery_holdout_00_ClearNoon_s2026100810_668ab812450f | 302 | 0.657253 → 0.640457 | 1.216844 → 1.180879 | 0.212509 → 0.212509 | 1.742935 → 1.742935 |
| recovery_holdout | carla_town07_town07_signed_recovery_holdout_01_ClearNoon_s2026100811_7696127af0ae | 302 | 0.661029 → 0.642008 | 1.221228 → 1.182050 | 0.212898 → 0.212898 | 1.747493 → 1.747493 |
| v4 | carla_town07_town07_straight_v4_research_train_ClearNoon_s2026100101_36b7c6dc98b9 | 327 | 0.104327 → 0.078348 | 0.257479 → 0.176099 | 0.024229 → 0.024229 | 0.027300 → 0.027300 |

## 기존 TRAIN 2487개 / known DEV 599개 — 모든 기록과 직진·회전

`raw_curvature`와 `raw_preferred_speed`는 원래 SmoothL1 손실입니다. m/s 단위인 값은 `raw_spatial_preferred_speed_mae_mps`뿐입니다. 기존 고속 자료는 decoder를 거치지 않았습니다.

| 자료 | 그룹/기록 | 분모 | 성분 | 부모 | 최종 | 변화 (최종−부모) |
|---|---|---:|---|---:|---:|---:|
| previously_exposed_known_DEV599_raw_not_independent | aggregate | 599 | total | 0.141163208 | 0.141593431 | +0.000430223 |
| previously_exposed_known_DEV599_raw_not_independent | aggregate | 599 | raw_curvature | 0.021829127 | 0.023550020 | +0.001720893 |
| previously_exposed_known_DEV599_raw_not_independent | aggregate | 599 | raw_preferred_speed | 0.135705926 | 0.135705926 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | aggregate | 599 | raw_spatial_preferred_speed_mae_mps | 0.315628868 | 0.315628868 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | total | 0.241343990 | 0.242171654 | +0.000827664 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_curvature | 0.045265456 | 0.048576117 | +0.003310661 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_preferred_speed | 0.230027626 | 0.230027626 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9 | 271 | raw_spatial_preferred_speed_mae_mps | 0.364718812 | 0.364718812 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | total | 0.058391891 | 0.058493740 | +0.000101849 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_curvature | 0.002465575 | 0.002872971 | +0.000407396 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_preferred_speed | 0.057775497 | 0.057775497 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320 | 328 | raw_spatial_preferred_speed_mae_mps | 0.275069799 | 0.275069799 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | curved | 85 | total | 0.056112567 | 0.059605263 | +0.003492696 |
| previously_exposed_known_DEV599_raw_not_independent | curved | 85 | raw_curvature | 0.099875759 | 0.113846543 | +0.013970784 |
| previously_exposed_known_DEV599_raw_not_independent | curved | 85 | raw_preferred_speed | 0.031143628 | 0.031143628 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | curved | 85 | raw_spatial_preferred_speed_mae_mps | 0.119148194 | 0.119148194 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | straight | 514 | total | 0.155228003 | 0.155151785 | -0.000076217 |
| previously_exposed_known_DEV599_raw_not_independent | straight | 514 | raw_curvature | 0.008922582 | 0.008617716 | -0.000304867 |
| previously_exposed_known_DEV599_raw_not_independent | straight | 514 | raw_preferred_speed | 0.152997357 | 0.152997357 | +0.000000000 |
| previously_exposed_known_DEV599_raw_not_independent | straight | 514 | raw_spatial_preferred_speed_mae_mps | 0.348120809 | 0.348120809 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | aggregate | 2487 | total | 0.055914846 | 0.055563688 | -0.000351159 |
| legacy_TRAIN2487_resubstitution_raw_only | aggregate | 2487 | raw_curvature | 0.010858708 | 0.009454073 | -0.001404635 |
| legacy_TRAIN2487_resubstitution_raw_only | aggregate | 2487 | raw_preferred_speed | 0.053200169 | 0.053200169 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | aggregate | 2487 | raw_spatial_preferred_speed_mae_mps | 0.153081375 | 0.153081375 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | total | 0.081429037 | 0.079973208 | -0.001455829 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_curvature | 0.037111684 | 0.031288370 | -0.005823315 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_preferred_speed | 0.072151116 | 0.072151116 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7 | 266 | raw_spatial_preferred_speed_mae_mps | 0.173469314 | 0.173469314 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | total | 0.004505462 | 0.004617907 | +0.000112445 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_curvature | 0.000635278 | 0.001085057 | +0.000449779 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_preferred_speed | 0.004346643 | 0.004346643 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p03_CloudyNoon_s0102_f586e8899633 | 328 | raw_spatial_preferred_speed_mae_mps | 0.054904197 | 0.054904197 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | total | 0.005454384 | 0.005532415 | +0.000078030 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_curvature | 0.000687676 | 0.000999798 | +0.000312121 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_preferred_speed | 0.005282465 | 0.005282465 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_c_track_1_0_7_c_track_1_0_7_straight_s0000_p06_ClearNoon_s0101_1ed315f2e1cb | 328 | raw_spatial_preferred_speed_mae_mps | 0.077809410 | 0.077809410 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | total | 0.226563177 | 0.224590875 | -0.001972302 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_curvature | 0.037863947 | 0.029974740 | -0.007889207 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_preferred_speed | 0.217097189 | 0.217097189 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f | 310 | raw_spatial_preferred_speed_mae_mps | 0.392435286 | 0.392435286 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | total | 0.008097159 | 0.008154961 | +0.000057802 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_curvature | 0.001068305 | 0.001299511 | +0.000231207 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_preferred_speed | 0.007830083 | 0.007830083 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town01_town01_straight_s0000_p02_ClearNoon_s0101_1662f5a7e646 | 328 | raw_spatial_preferred_speed_mae_mps | 0.082914331 | 0.082914331 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | total | 0.116490206 | 0.116268286 | -0.000221920 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_curvature | 0.013317384 | 0.012429700 | -0.000887684 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_preferred_speed | 0.113160860 | 0.113160860 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a | 271 | raw_spatial_preferred_speed_mae_mps | 0.235547437 | 0.235547437 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | total | 0.020513578 | 0.020637795 | +0.000124217 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_curvature | 0.001719336 | 0.002216206 | +0.000496871 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_preferred_speed | 0.020083744 | 0.020083744 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240 | 328 | raw_spatial_preferred_speed_mae_mps | 0.162666135 | 0.162666135 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | total | 0.008980234 | 0.009173205 | +0.000192971 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_curvature | 0.001337773 | 0.002109656 | +0.000771884 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_preferred_speed | 0.008645791 | 0.008645791 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | carla_town07_town07_straight_s0000_p02_CloudyNoon_s0102_384d262a3bf2 | 328 | raw_spatial_preferred_speed_mae_mps | 0.076224974 | 0.076224974 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | curved | 328 | total | 0.090020975 | 0.087138886 | -0.002882088 |
| legacy_TRAIN2487_resubstitution_raw_only | curved | 328 | raw_curvature | 0.051252093 | 0.039723738 | -0.011528354 |
| legacy_TRAIN2487_resubstitution_raw_only | curved | 328 | raw_preferred_speed | 0.077207951 | 0.077207951 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | curved | 328 | raw_spatial_preferred_speed_mae_mps | 0.195957748 | 0.195957748 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | straight | 2159 | total | 0.050733369 | 0.050766714 | +0.000033346 |
| legacy_TRAIN2487_resubstitution_raw_only | straight | 2159 | raw_curvature | 0.004722057 | 0.004855439 | +0.000133382 |
| legacy_TRAIN2487_resubstitution_raw_only | straight | 2159 | raw_preferred_speed | 0.049552854 | 0.049552854 | +0.000000000 |
| legacy_TRAIN2487_resubstitution_raw_only | straight | 2159 | raw_spatial_preferred_speed_mae_mps | 0.146567503 | 0.146567503 | +0.000000000 |
