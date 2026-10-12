# Town02_Opt p02 표지판 TRAIN 원본 정찰 — 실행 전 고정 계획

<!-- HH_260906 - Preregister the second Town02 sign scout without admitting data or changing any TEST split. -->

**상태: 아직 CARLA 미실행.** [Town02 p00 정찰](../map_town02_train_scout/README.md)은 원본 무결성에는 합격했지만 소형 표지판 조건은 **3/232 < 24장**으로 실패했다. 다음은 같은 TRAIN site의 약 197m 떨어진 다른 도로축을 **원본만** 정찰한다. 표지판 양성이나 성능 개선을 미리 주장하지 않는다.

- 원본 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p02.json`
- 고정 SHA256: `cad81f4cdfbc9dbcad8c3cc53092e6fd80f6d84dc4766b6d73758fa88dec6810`; world `Town02_Opt`, weather `ClearNoon`, route length 약 173.46m. 반대 방향 p03은 **이 계획에 포함되지 않는다.**
- 새 출력: `storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1`. 기존 p00 또는 다른 출력은 덮어쓰지 않는다.
- 소유 수집: CARLA `quality=Epic`, 전방 RGB/깊이/시맨틱 640×360·90°·10Hz, 물리 `BasicAgent`, 240 동기 프레임. 명시적 신호등 무시는 지도 정찰용이며 교통규칙 준수나 학습 모델 제어의 증거가 아니다. 포트·소유 잠금이 비어 있지 않으면 타 실행에 간섭하지 않고 중단한다. 자동 재시도하지 않는다.
- 무결성 합격: owner 종료 코드 0, 경로·수집 소스 SHA 일치, 계획/시작/종료의 CARLA 월드 동일, lifecycle 준비·캡처 후·종료 PASS, 충돌 0, 저장 원본/라이브 지도 일치, 정리 오류 0, RGB/semantic 240쌍 전수 640×360 디코드 및 10Hz 프레임·타임스탬프 확인. 차량 XY는 이전 **모든 보존 위치**에서 0.25m 이상 떨어진 프레임만 보존하여 **120장 이상**이어야 한다. 240프레임 내 약 173m 목적지 도달은 요구하지 않으며 실제 포착 구간만 보고한다.
- 표지판 정답 합격: 보존 프레임의 CARLA `traffic_sign` 태그 8 **프레임 전체 픽셀 수** `0 / 1–4 / 5–64 / ≥65`와 총 픽셀 수를 기록한다. **1–4픽셀 보존 프레임 24장 이상**이 아니면 *소형 표지판 TRAIN 후보로 거부*한다. 24장 이상이어도 해당 픽셀이 RGB의 독립된 실제 표지판과 맞는지 원본 전체 화각과 확대 화면에서 확인하고, 객체·시간 중복을 별도 보고하기 전에는 TRAIN 편입하지 않는다. 작은 연결 성분 개수를 프레임 전체 기준으로 바꿔 세지 않는다.
- 분할 보호: Town02_Opt는 이 정찰의 TRAIN 후보일 뿐이다. Town05_Opt 지도 DEV, Town04·Town07·Town15 held-out 및 Town06·Town10HD_Opt 예약 TEST의 RGB/라벨/예측을 열거나 선택 근거로 쓰지 않는다. 과거 Town03 지도를 TRAIN으로 쓴 후보 C의 통합 분할 결함은 별도 기록으로 남긴다. 추론·새 지도 학습·Autoware 주행·실차 시험·SSH/GPU 사용은 이 계획의 범위 밖이다.

실행 담당자는 다른 CARLA 테스트가 없는지 소유 잠금·포트를 먼저 확인하고, 다음 **정확한** 명령을 한 번만 실행한다. 이 문서 작성 시점에는 명령을 실행하지 않았다.

```bash
bash tools/e2e/run_owned_carla_expert_trial.sh \
  storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1 \
  storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town02_opt/catalog/straight/routes/town02_opt/straight/town02_opt_straight_s0000_p02.json \
  --port 2100 --quality Epic --wall-timeout-sec 900 --capture-mode mapping-probe -- \
  --frames 240 --motion-mode basic-agent --ignore-traffic-lights \
  --town02-sign-train-scout-p02
```

판정 실패 시 기준을 낮추거나 p03을 자동 실행하지 않는다. 성공하더라도 별도 원본 감사·DEV 고정 평가·사전 등록 A/B가 끝나기 전에는 데이터셋과 모델을 변경하지 않는다.
