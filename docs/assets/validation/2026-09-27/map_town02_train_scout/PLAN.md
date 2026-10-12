# Town02_Opt 표지판 TRAIN 원본 정찰 — 실행 전 계획

<!-- HH_260906 - Pin a TRAIN-site scouting route without qualifying its unseen semantic labels. -->

**목적:** 지도 후보 C는 표지판 오검출이 늘었지만, 현재 추가 Town03 TRAIN의 표지판 정답은 416장에 140픽셀뿐이다. 주행·지도 통합 분할에서 TRAIN site인 Town02_Opt에서 새 전방 640×360 표지판 정답이 실제 존재하는지 **원본만** 확인한다. 이 정찰만으로 데이터 TRAIN 편입·학습·성능 개선을 선언하지 않는다.

- 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p00.json`, SHA256 `520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b`, ClearNoon, 원본 route length 약 177.13m.
- 새 출력: `storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1`; 기존 자료를 덮어쓰지 않는다.
- 수집: 소유 CARLA Town02_Opt, `quality=Epic`, 물리 BasicAgent·명시적 신호 무시, 240 동기 프레임, 전방 RGB/깊이/시맨틱 640×360·90°·10 Hz. 신호 무시는 교통규칙 기능의 검증 근거가 아니다.
- 원본 품질: owner 종료 코드 0·소스 불변·route/world SHA 일치·CARLA 수명주기 PASS·충돌 0·live/replay 일치·240쌍 전수 디코드·동일 Town 0.25m 보존 ≥120장이 아니면 정찰 실패. 자동 재시도나 기준 완화 없음.
- 정답 분포: 보존 프레임에서 CARLA tag 8 `traffic_sign` 전체 프레임 크기 `0 / 1–4 / 5–64 / ≥65` 구간 수와 픽셀 합계를 기록한다. 작은 표지판을 보강했다는 주장은 **1–4픽셀 보존 프레임 ≥24장**과 RGB/라벨 물체 대응 확인 전까지 금지한다. 양성량이 부족하면 TRAIN 후보를 거부한다.
- Town05_Opt는 지도 DEV 후보, Town07은 지도 최종 TEST, Town04·Town06·Town10HD_Opt는 통합 TEST 예약으로 보호한다. 이들 원본·정답·모델 점수는 이 정찰에서 열지 않는다. AI v2/B/C 추론·추가 GPU 학습·Autoware/실차 제어는 실행하지 않는다.
