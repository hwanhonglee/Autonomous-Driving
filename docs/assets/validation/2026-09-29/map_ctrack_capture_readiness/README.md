# C_track 지도 TRAIN 촬영 사전 점검

<!-- HH_260929 - Freeze a read-only C-track capture design without opening the mapping site or running CARLA. -->

**이 문서는 촬영 전 읽기 전용 사전 점검 기록이다.** 당시에는 촬영 경로와 센서 계약만 설계했고 CARLA, GPU, SSH 및 새 카메라 촬영을 사용하지 않았다. 이후의 실제 실행 상태는 아래 후속 결과가 우선한다.

<!-- HH_260929 - Preserve the earlier preflight while linking the subsequently rejected owned capture. -->
## 후속 결과: Epic 좌회전 1회 촬영, 지도 학습용 QA 실패

[전용 V2 수집기](../../../../../tools/e2e/collect_carla_mapping_ctrack_v2.py)와 [소유 실행 결과](LEFT_OWNED_01_RESULT.md)는 좌회전 240프레임·93.63m·충돌 0 및 경로 전/중/후 이동을 기록했다. 그러나 RGB에 보이는 도로/차선의 semantic 정답 ID `1`·`24`는 전 프레임에서 각각 0픽셀이다. [전수 QA](left_owned_01_qa.json)는 `FAIL`이고 이 원본은 TRAIN에 넣지 않았다. 커스텀 맵 자산의 태깅 원인을 확인하기 전에는 같은 맵의 직진 반복 촬영도 보류한다. 아래의 나머지 항목은 촬영 **전** 판단의 출처·계획으로 보존한다.

## 설치 자산과 후보 경로

`CARLA_ROOT=/home/a/carla-autoware-universe/CARLA_0.9.15`의 packaged `C_track_1_0_7.umap` SHA256은 `d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7`, `OpenDrive/C_track_1_0_7.xodr`는 `3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27`이다. 둘 다 [맵 목록](../../../../../tools/e2e/carla_expert_suite.yaml)의 packaged level과 일치한다. 목록의 기존 주행은 Epic에서 통과했고 Low에서 렌더 충돌을 기록한다. 이 과거 결과는 새 세 센서의 화질을 보증하지 않는다.

| 사전 등록 후보 | 파일 SHA256 | 정적 경로 정보 | 용도와 남은 확인 |
| --- | --- | --- | --- |
| [직진 p03](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/c_track_1_0_7/catalog/straight/routes/c_track_1_0_7/straight/c_track_1_0_7_straight_s0000_p03.json) | `cb8632fa48598389a948fe552826ab05a0d2c737126dd3f39e4dde12d5e3b70e` | 214.6 m, `STRAIGHT`가 누적 64.4–88.2 m | 240개 10 Hz 영상 안에 교차로 전후가 실제 들어오는지 측정해야 한다. |
| [좌회전 확장 경로](../../../../../storage/experiments/training/2026-09-08/turn_launch_matrix_v1/c_track_left/turn_launch_013_v1/run_002/episode/route.json) | `2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b` | 206.3 m, `LEFT`가 28.3–48.9 m | 같은 정확한 경로의 [과거 expert episode](../../../../../storage/experiments/training/2026-09-08/turn_launch_matrix_v1/c_track_left/turn_launch_013_v1/run_002/episode/manifest.json)는 완료·충돌 0·목표 도달을 기록했다. 지도용 센서 동기화와 라벨은 아직 검증되지 않았다. |

두 후보 모두 `coordinate_reference=base_link`, `town=C_track_1_0_7` 및 `start_carla_transform`/`road_option_value`가 있어 기존 BasicAgent 경로 형식에 맞는다. 직진과 회전을 **별도 수집**하기 위한 후보이며, 지금 표의 길이와 조향 구간은 JSON의 정적 값이다. 24초 동안의 실제 이동 거리나 해당 구간 통과는 측정하지 않았다. 짧은 [기존 좌회전 p07](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/c_track_1_0_7/catalog/turn/routes/c_track_1_0_7/left/c_track_1_0_7_left_s0000_p07.json)도 73.9 m·좌회전 28.3–48.9 m로 보존돼 있지만, 위 확장 경로가 회전 후 영상 구간을 더 확보할 수 있는 1차 후보이다.

## 수집기가 할 수 있는 일과 현재 막힌 곳

[지도 probe](../../../../../tools/e2e/collect_carla_mapping_probe.py)는 동일 위치·화각의 전방 RGB/깊이/시맨틱 카메라 3개를 각각 640×360, 수평 90°, 차체 기준 `x=1.5 m, z=2.0 m, pitch=-12°`로 붙인다. CARLA를 고정 `0.1 s` tick으로 실행하며 `sensor_tick=0`이므로 프레임별 RGB PNG, 원본 depth/semantic·카메라 세계 pose NPZ, records와 live/replay 일치 결과를 남길 수 있다. BasicAgent의 240프레임은 물리 주행 기록이지만 학습 모델 주행은 아니다. 기존 [expert episode](../../../../../storage/experiments/validation/2026-08-31/c_track_394_to_290_expert/manifest.json)는 6개 RGB JPEG, 전방 70° 등 다른 센서 계약이며 같은 프레임의 depth/semantic이 없다. 이를 7클래스 지도 학습쌍으로 재사용할 수 없다.

[과거 CAM_FRONT 예시](../../../../../storage/experiments/validation/2026-08-31/c_track_394_to_290_expert/images/CAM_FRONT/00073462.jpg)를 눈으로 보면 검은 포장면과 흰 차선이 보여 Town05 실패 자료의 *외관 다양성 후보*로는 의미가 있다. 하지만 **다른 화각·센서 위치·경로**의 RGB 한 장이고 의미 정답도 없으므로, Town05 오류 개선을 입증하거나 학습 자료로 채택할 근거는 아니다. 새 90° 세 센서 촬영의 같은 구간·라벨을 별도로 검사해야 한다.

1. [통합 사이트 분할 원장](../../../../../learning/config/site_split_guard_20260929.json)은 C_track의 `drive` 기존 TRAIN만 허용하고 `map` 역할은 `[]`이다. `carla_expert_suite.yaml`의 `split: train`만으로 지도 TRAIN 권한이 생기지 않는다. 지도 TRAIN 촬영과 편입 범위를 원장에서 먼저 검토해야 한다.
2. 수집기의 기본 route 결합은 Town01이고 연구용 추가 scope도 Town02/03/05/06/07/13의 해시만 허용한다. C_track 두 경로를 입력하면 현재 검증은 **Town 불일치로 거절**한다. 정확한 두 경로의 바이트 해시, `C_track_1_0_7` world, Epic 품질, 출력 경로를 고정한 좁은 scope와 거절 테스트가 필요하다.
3. 현재 BasicAgent 완료 조건은 총 이동 `>3 m`와 충돌 0 및 240개 저장·replay 일치다. 실제 직진/좌회전 구간 통과, 0.25 m 간격의 충분한 서로 다른 위치, 노면·차선 라벨 존재, 하늘·노면 렌더 품질, depth 유효 픽셀, 교통 신호 반응은 별도 QA가 필요하다. `--ignore-traffic-lights`는 기존 연구 scope에 요구되므로 새 scope에서도 사전에 교통 정책을 명시해야 한다.
4. 설치된 `.umap`·`.xodr` 파일 해시는 확인했지만 현재 실행 중인 CARLA world의 자산 동일성은 이 읽기 전용 점검으로 확인할 수 없다. 소유 Epic cold start에서 world 이름·route start/goal waypoint·렌더/semantic/depth 정합을 확인하고 원본 SHA를 보관해야 한다.

다음 단계는 **사이트 역할 및 정확한 두 route를 사전 등록한 뒤**, C_track 전용 좁은 수집기 변경과 실패 테스트를 만드는 것이다. 그 다음 다른 사용자의 CARLA 작업과 겹치지 않는 소유 Epic 세션에서 직진·좌회전을 각 1회 촬영하고, 프레임 전수 디코드·타임스탬프/pose 동일성·이동/회전 구간·충돌·라벨 분포·전체 화각 PNG·live/replay를 감사한다. 하나라도 실패하면 원본은 보존하되 TRAIN에는 넣지 않는다. 이 보고서 자체는 `capture_authorized=false`, `dataset_admitted=false`, `independent_dev_approved=false`이다.
