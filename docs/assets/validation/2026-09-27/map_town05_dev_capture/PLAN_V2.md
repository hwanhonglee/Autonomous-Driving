# Town05_Opt 지도 DEV 후보 — Epic 단회 재수집 계획

<!-- HH_260906 - Predeclare the matching rendering quality and immutable output before a second owned capture. -->

첫 [Low 회차의 사후 불일치](V1_DEVIATION.md) 때문에 별도 단회 재수집한다. 목표는 기존 Town03 지도 TRAIN과 동일한 **CARLA `quality=Epic`** 조건에서 원본 Town05_Opt DEV 후보를 만드는 것이다. 모델 예측·점수는 수집·자격 검사 전에 열지 않는다.

- source route: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/straight/routes/town05_opt/straight/town05_opt_straight_s0000_p00.json`의 정확한 SHA256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c`.
- 새 output: `storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2`; 기존 Low output과 겹치거나 덮어쓰지 않는다.
- 소유 CARLA, `mapping-probe`, `--quality Epic`, BasicAgent·신호 무시·240프레임, 640×360·90° 전방 RGB/깊이/시맨틱·10 Hz. 프레임별 원본 oracle은 모델 입력으로 사용하지 않는다.
- 사전 자격: owner exit 0, 실행 소스 보관/불변, 정확한 route/world, lifecycle ready/after/stopped PASS, 충돌 0, live/replay 정확 일치, 240 RGB/시맨틱 전수 디코드, 동일 Town 내 0.25 m 보존 ≥120장. 미달이면 DEV 거절하고 추가 자동 재시도하지 않는다.
- 지도 TRAIN Town01+Town03, 지도 최종 Town07 TEST, 주행 TEST 예약 Town04·Town06·Town10HD_Opt의 원본을 열지 않는다. 주행 Town05_Opt right는 development VAL로서 같은 site의 다른 route이다. 향후 통합 모델에서는 site를 개발 분할로만 취급한다.
- 결과물은 수집 자격/라벨 분포·전체 시야 oracle PNG까지만 만든다. 학습, 모델 A/B 점수, Autoware GUI/제어, 정밀지도 승인으로 해석하지 않는다.
