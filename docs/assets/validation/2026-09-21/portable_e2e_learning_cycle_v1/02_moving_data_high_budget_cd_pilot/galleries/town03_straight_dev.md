<!-- HH_260906 - Pair the same straight DEV samples for the high-budget C/D offline comparison. -->

# Town03 직진 DEV — 고예산 C/D 궤적 갤러리

동일한 Town03 직진 DEV episode 328개를 C와 D의 final checkpoint로 평가한
**offline fixed-DEV open-loop** 자료다. C는 기존 TRAIN 1,147개, D는 Town07 직진 328개를
추가한 TRAIN 1,475개이며 둘 다 seed `20260906`, 7,380 optimizer step이다.

이 그림은 CARLA/Autoware 라이브 주행이나 RViz 전체화면이 아니다. 회색은 route, 초록색은
expert trajectory, 굵은 색 선은 selected trajectory, 파란 상자는 화면 하단 중앙의 ego다.

## Episode metrics

단위는 m이고 속도 MAE만 m/s다. 낮을수록 좋으며 `Δ = D - C`다.

| metric | C — old TRAIN | D — augmented TRAIN | D - C |
| --- | ---: | ---: | ---: |
| loss | 7.176940127 | 1.997684832 | -5.179255295 |
| ADE @ 1.0 s | 0.283963843 | 0.067356766 | -0.216607077 |
| ADE @ 3.0 s | 2.213999163 | 0.693466213 | -1.520532950 |
| ADE @ 6.4 s | 8.090865013 | 3.764740188 | -4.326124825 |
| FDE @ 6.4 s | 19.325329769 | 11.027441244 | -8.297888525 |
| 속도 MAE @ 6.4 s | 3.048794996 | 1.787750496 | -1.261044500 |

D는 C보다 ADE 6.4초 53.47%, FDE 42.94% 개선해 사전 고정된 직진 gate 두 항목을
통과했다. 하지만 우회전 gate가 실패했으므로 D 전체 승격 근거가 되지 않는다.

## Paired plots

| C — `C_old_train_7380_steps` | D — `D_augmented_train_7380_steps` |
| --- | --- |
| ![C straight sample 0000](../visuals/town03_straight_dev/C_old_train_7380_steps/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) | ![D straight sample 0000](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) |
| ![C straight sample 0001](../visuals/town03_straight_dev/C_old_train_7380_steps/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) | ![D straight sample 0001](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) |
| ![C straight sample 0002](../visuals/town03_straight_dev/C_old_train_7380_steps/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) | ![D straight sample 0002](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) |
| ![C straight sample 0003](../visuals/town03_straight_dev/C_old_train_7380_steps/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) | ![D straight sample 0003](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) |
| ![C straight sample 0004](../visuals/town03_straight_dev/C_old_train_7380_steps/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) | ![D straight sample 0004](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) |
| ![C straight sample 0005](../visuals/town03_straight_dev/C_old_train_7380_steps/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) | ![D straight sample 0005](../visuals/town03_straight_dev/D_augmented_train_7380_steps/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) |

[상위 보고서](../README.md) ·
[원시 C episode metrics](../metrics/C_old_train_7380_steps/town03_straight_dev_metrics.json) ·
[원시 D episode metrics](../metrics/D_augmented_train_7380_steps/town03_straight_dev_metrics.json)
