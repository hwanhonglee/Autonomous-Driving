<!-- HH_260906 - Document the paired fixed-DEV trajectory evidence without implying live-driving validation. -->

# Town03 straight DEV — A/B trajectory gallery

이 문서는 동일한 Town03 직진 DEV 에피소드 328개 샘플에 대해 `A_baseline_train`과 `B_town07_augmented_train`을 비교한 **오프라인 고정 DEV(open-loop) 결과**다. 아래 그림은 CARLA 또는 Autoware 실시간 주행 화면이 아니며, 폐루프 주행 성능이나 실제 차량 제어 가능성을 입증하지 않는다.

그림 범례는 다음과 같다.

- 회색: route
- 초록색: expert trajectory
- 굵은 선: selected trajectory
- 파란색: ego vehicle

## Episode metrics

단위는 m이며 낮을수록 좋다. 표의 A와 B는 각 episode `metrics.json`의 원값이고, delta는 정확히 `B - A`로 계산했다. 따라서 양수는 B의 오차 증가, 음수는 B의 오차 감소를 뜻한다.

| Metric | A — baseline | B — Town07 augmented | B - A |
|---|---:|---:|---:|
| ADE @ 1.0 s | 0.2598971395140014 | 0.11538044460935563 | -0.14451669490464577 |
| ADE @ 3.0 s | 1.9073732506211212 | 0.8477864894652511 | -1.0595867611558701 |
| ADE @ 6.4 s | 7.531365476003507 | 4.16186288471629 | -3.369502591287217 |
| FDE @ 6.4 s | 18.787311472543855 | 12.266843402894532 | -6.520468069649323 |

이 한 seed의 고정 DEV 결과에서는 Town07 직진 데이터를 추가한 B가 Town03 직진의 네 오차 지표 모두에서 A보다 좋았다. 이는 방향성 관찰이며 모델 채택 또는 일반화 성능 판정이 아니다.

## Paired plots

각 행은 동일한 DEV 샘플을 좌측 A와 우측 B로 대응시킨 것이다.

| A — `A_baseline_train` | B — `B_town07_augmented_train` |
|---|---|
| ![A straight sample 0000](../visuals/town03_straight_dev/A_baseline_train/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) | ![B straight sample 0000](../visuals/town03_straight_dev/B_town07_augmented_train/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) |
| ![A straight sample 0001](../visuals/town03_straight_dev/A_baseline_train/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) | ![B straight sample 0001](../visuals/town03_straight_dev/B_town07_augmented_train/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) |
| ![A straight sample 0002](../visuals/town03_straight_dev/A_baseline_train/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) | ![B straight sample 0002](../visuals/town03_straight_dev/B_town07_augmented_train/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) |
| ![A straight sample 0003](../visuals/town03_straight_dev/A_baseline_train/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) | ![B straight sample 0003](../visuals/town03_straight_dev/B_town07_augmented_train/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) |
| ![A straight sample 0004](../visuals/town03_straight_dev/A_baseline_train/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) | ![B straight sample 0004](../visuals/town03_straight_dev/B_town07_augmented_train/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) |
| ![A straight sample 0005](../visuals/town03_straight_dev/A_baseline_train/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) | ![B straight sample 0005](../visuals/town03_straight_dev/B_town07_augmented_train/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) |
