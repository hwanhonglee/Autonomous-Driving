# 2026-09-27 소형 표지판용 다음 경로 탐색 — 실행 전 후보

<!-- HH_260906 - Treat screenshots and route geometry as scouting evidence, never as semantic-label admission. -->

**결론:** 기존 원본 중 Town01 DEV와 겹치지 않으면서 640×360 전방 카메라의 `traffic_sign` 라벨이 **1–4픽셀로 실제 관측된** 다른 Town 경로는 아직 없습니다. 아래는 표지판이 보였던 과거 RGB 화면과 기존 경로 기하를 함께 본 **수집 우선순위**이지, 검증된 학습 데이터가 아닙니다. CARLA·SSH 실행과 모델 변경은 하지 않았습니다.

1. **Town06 직진 p06 — 1순위 정찰.** [2026-09-02 Town06 expert 전체 화면](../../2026-09-02-all-towns-camera-source-5hz/town06/expert_overview_1920x1080.png)의 `FRONT_RIGHT`에 제한속도 60 표지판이 보입니다(표시 시점 진행 약 24.2m). 기존 Town06 직진 p00 경로는 대략 `(-260.2, 247.2)`에서 동쪽으로 시작합니다. p06은 주행 방향 기준 약 30m 상류인 `(-290.2, 240.3)`에서 같은 방향으로 시작하여, p00 시작점 근처를 경로 30m 지점에 지나갑니다. 두 경로의 횡방향 차이는 약 7m입니다. 따라서 p06은 표지판이 옆으로 지나가기 전에 더 먼 거리에서 촬영할 기회를 줄 *가능성*이 있습니다. 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town06/catalog/straight/routes/town06/straight/town06_straight_s0000_p06.json`; SHA256 `e27e95cb60d40657638183f7f78ce2e1f4f070067707ac4dd3f2f89dd1c52283`.
2. **Town07 직진 p04 — 2순위 정찰.** [2026-09-02 Town07 expert 전체 화면](../../2026-09-02-all-towns-camera-source-5hz/town07/expert_overview_1920x1080.png)의 `BACK`에 정지 표지판이 보입니다. p04는 기존 Town07 직진 p00보다 같은 진행방향에서 약 30m 상류에서 출발하여 p00 시작점 근처를 경로 30m 지점에 지납니다(평면 거리 0.32m). 앞쪽 촬영 가능성은 있지만, 기존 사진에서는 표지판이 **후방**에 보였으므로 전방 90° 화각·표지판 앞뒷면 방향은 불확실합니다. 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town07/catalog/straight/routes/town07/straight/town07_straight_s0000_p04.json`; SHA256 `68752a219a12f7f88df65aa885021db874f091dce6b97ef68274c3734ee93fbf`.

과거 화면은 여러 카메라를 배열한 RGB 주행 요약으로, 현재 지도 probe의 **단일 전방 640×360·90° 카메라와 렌즈 위치·방향이 다릅니다.** 표지판이 CARLA semantic tag 8로 나오거나 1–4픽셀 크기라는 증거는 아닙니다. 다른 Town이므로 Town01 TRAIN/DEV와 장소가 다르고 p02/p04의 0.017m 상호 중복도 피하지만, **같은 Town 안에서 향후 TRAIN·독립 TEST 경로를 다시 나누어야** 합니다. 과거 화면으로 후보를 골랐으므로 이 장면 자체를 맹검 TEST라고 부를 수 없습니다.

현재 `tools/e2e/collect_carla_mapping_probe.py`에는 `Town01` 전용 가드가 있어 위 경로를 그대로 넣으면 사전 검사는 통과해도 수집 단계에서 거부됩니다. 별도 검토·테스트를 거친 Town 확장 전에는 실행하지 않습니다. 확장 이후에도 첫 실행은 학습이 아닌 소유권 있는 **원본 정찰**로 제한하고, 각 프레임의 tag 8 픽셀 `0 / 1–4 / 5–64 / >=65` 히스토그램, 0.25m 중복 제거 결과, RGB/라벨 대응, 충돌·재생 일치, 경로 SHA를 확인해야 합니다. 소형 표지판 TRAIN 편입은 중복 제거 후 1–4픽셀 최소 24장과 기존 분리·고정 평가 조건을 통과할 때만 별도 결정합니다.

비교 기준: [p02·p04 실제 원본 감사](../map_sign_probe/README.md) — 두 경로 모두 1–4픽셀 0장. DEV의 1–4픽셀 31장은 학습에 넣지 않습니다.
