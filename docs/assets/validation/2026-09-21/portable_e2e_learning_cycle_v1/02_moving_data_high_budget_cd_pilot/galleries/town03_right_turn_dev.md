<!-- HH_260906 - Pair the same right-turn DEV samples for the high-budget C/D offline comparison. -->

# Town03 우회전 DEV — 고예산 C/D 궤적 갤러리

동일한 Town03 우회전 DEV episode 337개를 C와 D의 final checkpoint로 평가한
**offline fixed-DEV open-loop** 자료다. C는 기존 TRAIN 1,147개, D는 Town07 직진 328개를
추가한 TRAIN 1,475개이며 둘 다 seed `20260906`, 7,380 optimizer step이다.

이 그림은 CARLA/Autoware 라이브 주행이나 RViz 전체화면이 아니다. 회색은 route, 초록색은
expert trajectory, 굵은 색 선은 selected trajectory, 파란 상자는 화면 하단 중앙의 ego다.

## Episode metrics

단위는 m이고 속도 MAE만 m/s다. 낮을수록 좋으며 `Δ = D - C`다.

| metric | C — old TRAIN | D — augmented TRAIN | D - C |
| --- | ---: | ---: | ---: |
| loss | 3.612647089 | 4.002403618 | +0.389756529 |
| ADE @ 1.0 s | 0.180320740 | 0.195691674 | +0.015370934 |
| ADE @ 3.0 s | 0.853928309 | 0.921544922 | +0.067616613 |
| ADE @ 6.4 s | 2.935314375 | 3.367132111 | +0.431817736 |
| FDE @ 6.4 s | 7.793866114 | 9.697629961 | +1.903763847 |
| 속도 MAE @ 6.4 s | 1.107615423 | 1.486372467 | +0.378757043 |

D는 C보다 ADE 6.4초 14.71%, FDE 24.43%, 속도 MAE 34.20% 악화해 사전 고정된
우회전 비퇴행 gate 세 항목을 모두 실패했다. 아래 대응 그림은 방향성 확인 자료이며 폐루프
주행 성공률이나 실차 안전성을 나타내지 않는다.

## Paired plots

| C — `C_old_train_7380_steps` | D — `D_augmented_train_7380_steps` |
| --- | --- |
| ![C right-turn sample 0000](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0000_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000000.png) | ![D right-turn sample 0000](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0000_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000000.png) |
| ![C right-turn sample 0001](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0001_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000001.png) | ![D right-turn sample 0001](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0001_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000001.png) |
| ![C right-turn sample 0002](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0002_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000002.png) | ![D right-turn sample 0002](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0002_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000002.png) |
| ![C right-turn sample 0003](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) | ![D right-turn sample 0003](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) |
| ![C right-turn sample 0004](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0004_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000004.png) | ![D right-turn sample 0004](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0004_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000004.png) |
| ![C right-turn sample 0005](../visuals/town03_right_turn_dev/C_old_train_7380_steps/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) | ![D right-turn sample 0005](../visuals/town03_right_turn_dev/D_augmented_train_7380_steps/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) |

[상위 보고서](../README.md) ·
[원시 C episode metrics](../metrics/C_old_train_7380_steps/town03_right_turn_dev_metrics.json) ·
[원시 D episode metrics](../metrics/D_augmented_train_7380_steps/town03_right_turn_dev_metrics.json)
