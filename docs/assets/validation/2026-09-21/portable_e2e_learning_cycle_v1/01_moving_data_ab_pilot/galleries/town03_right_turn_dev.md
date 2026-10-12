<!-- HH_260906 - Document the paired fixed-DEV trajectory evidence without implying live-driving validation. -->

# Town03 right-turn DEV — A/B trajectory gallery

이 문서는 동일한 Town03 우회전 DEV 에피소드 337개 샘플에 대해 `A_baseline_train`과 `B_town07_augmented_train`을 비교한 **오프라인 고정 DEV(open-loop) 결과**다. 아래 그림은 CARLA 또는 Autoware 실시간 주행 화면이 아니며, 폐루프 주행 성능이나 실제 차량 제어 가능성을 입증하지 않는다.

그림 범례는 다음과 같다.

- 회색: route
- 초록색: expert trajectory
- 굵은 선: selected trajectory
- 파란색: ego vehicle

## Episode metrics

단위는 m이며 낮을수록 좋다. 표의 A와 B는 각 episode `metrics.json`의 원값이고, delta는 정확히 `B - A`로 계산했다. 따라서 양수는 B의 오차 증가, 음수는 B의 오차 감소를 뜻한다.

| Metric | A — baseline | B — Town07 augmented | B - A |
|---|---:|---:|---:|
| ADE @ 1.0 s | 0.22343706034091415 | 0.23218842593604805 | +0.00875136559513390 |
| ADE @ 3.0 s | 1.1069322577304586 | 1.243243026406192 | +0.1363107686757334 |
| ADE @ 6.4 s | 3.4226633787862624 | 4.355225788644938 | +0.9325624098586756 |
| FDE @ 6.4 s | 8.42707781338904 | 11.993511646366969 | +3.566433832977929 |

이 한 seed의 고정 DEV 결과에서는 Town07 직진 데이터를 추가한 B가 Town03 우회전의 네 오차 지표 모두에서 A보다 나빴다. 이는 방향성 관찰이며 모델 채택 또는 일반화 성능 판정이 아니다.

## Paired plots

각 행은 동일한 DEV 샘플을 좌측 A와 우측 B로 대응시킨 것이다.

| A — `A_baseline_train` | B — `B_town07_augmented_train` |
|---|---|
| ![A right-turn sample 0000](../visuals/town03_right_turn_dev/A_baseline_train/0000_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000000.png) | ![B right-turn sample 0000](../visuals/town03_right_turn_dev/B_town07_augmented_train/0000_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000000.png) |
| ![A right-turn sample 0001](../visuals/town03_right_turn_dev/A_baseline_train/0001_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000001.png) | ![B right-turn sample 0001](../visuals/town03_right_turn_dev/B_town07_augmented_train/0001_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000001.png) |
| ![A right-turn sample 0002](../visuals/town03_right_turn_dev/A_baseline_train/0002_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000002.png) | ![B right-turn sample 0002](../visuals/town03_right_turn_dev/B_town07_augmented_train/0002_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000002.png) |
| ![A right-turn sample 0003](../visuals/town03_right_turn_dev/A_baseline_train/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) | ![B right-turn sample 0003](../visuals/town03_right_turn_dev/B_town07_augmented_train/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) |
| ![A right-turn sample 0004](../visuals/town03_right_turn_dev/A_baseline_train/0004_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000004.png) | ![B right-turn sample 0004](../visuals/town03_right_turn_dev/B_town07_augmented_train/0004_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000004.png) |
| ![A right-turn sample 0005](../visuals/town03_right_turn_dev/A_baseline_train/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) | ![B right-turn sample 0005](../visuals/town03_right_turn_dev/B_town07_augmented_train/0005_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000005.png) |
