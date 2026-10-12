<!-- HH_260906 - Pair identical Town03 straight DEV samples for the C/E offline trajectory comparison. -->

# Town03 직진 DEV — C 기준 모델 / E 회전 재생 모델

동일한 Town03 직진 sample `0000`–`0005`를 C reference와 E turn replay로 각각
추론한 **offline fixed-DEV trajectory render** 비교다. 이 이미지는 CARLA/Autoware
라이브 화면, RViz 전체화면, closed-loop 주행 결과 또는 GIF가 아니다.

범례: ego 차량은 화면 하단 중앙에 있다. 회색 선은 route, 초록색 선은 expert
trajectory, 굵은 색 선은 모델이 선택한 selected trajectory다.

## 동일 sample 대응 비교

| C reference | E turn replay |
| --- | --- |
| ![C reference straight sample 0000](../visuals/town03_straight_dev/C_reference/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) | ![E turn replay straight sample 0000](../visuals/town03_straight_dev/E_turn_replay/0000_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000000.png) |
| ![C reference straight sample 0001](../visuals/town03_straight_dev/C_reference/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) | ![E turn replay straight sample 0001](../visuals/town03_straight_dev/E_turn_replay/0001_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000001.png) |
| ![C reference straight sample 0002](../visuals/town03_straight_dev/C_reference/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) | ![E turn replay straight sample 0002](../visuals/town03_straight_dev/E_turn_replay/0002_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000002.png) |
| ![C reference straight sample 0003](../visuals/town03_straight_dev/C_reference/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) | ![E turn replay straight sample 0003](../visuals/town03_straight_dev/E_turn_replay/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) |
| ![C reference straight sample 0004](../visuals/town03_straight_dev/C_reference/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) | ![E turn replay straight sample 0004](../visuals/town03_straight_dev/E_turn_replay/0004_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000004.png) |
| ![C reference straight sample 0005](../visuals/town03_straight_dev/C_reference/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) | ![E turn replay straight sample 0005](../visuals/town03_straight_dev/E_turn_replay/0005_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000005.png) |

