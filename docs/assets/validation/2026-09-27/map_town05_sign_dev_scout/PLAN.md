# Town05_Opt 우회전 p00: 표지판 DEV 후보 정찰 계획

<!-- HH_260906 - Preregister one exact-route, sign-capable DEV scout before CARLA capture or label inspection. -->

**상태: 준비만 완료, CARLA 미실행.** [Town05 직진 Epic 원본](../map_town05_dev_capture/README.md)은 도로·차선 DEV 후보로 원본 감사를 통과했지만 표지판 태그 8이 보존 프레임당 1–2픽셀이어서 표지판 평가에는 부적합하다. 이번 우회전은 [다음 지도 학습 설계](../map_next_training_design/README.md)에서 사전에 선택한 **같은 Town05_Opt 사이트의 별도 DEV 경로**다. 표지판이 존재하거나 기준을 통과한다고 미리 주장하지 않는다. 직진과 우회전을 서로 독립적인 두 사이트로 세지 않는다.

- 원본 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/turn/routes/town05_opt/right/town05_opt_right_s0000_p00.json`
- 파일 SHA256: `2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3`. 소유 CARLA 월드와 경로의 `town`은 정확히 `Town05_Opt`여야 한다. `--town05-sign-dev-scout`는 **이 해시 하나만** 허용하며 기존 직진 DEV 플래그와 상호 배타적이다.
- 신규 출력: `storage/experiments/training/2026-09-27/map_town05_right_sign_dev_scout_epic_v1`. 기존 직진 Epic/Low 결과나 다른 실험을 덮어쓰지 않는다. 경로명에 `training`이 있어도 이 원본의 연구 분할은 **DEV 후보**이며 TRAIN 편입을 허가하지 않는다.
- 소유 수집 조건: 포트·잠금이 비어 있을 때만 시작한다. CARLA `--quality Epic`, ClearNoon, 앞쪽 RGB/깊이/시맨틱 640×360·90°, 10 Hz 고정, 물리 `BasicAgent` 240 동기 프레임, 지도 정찰용 명시적 신호등 무시. 이 교통규칙 우회는 자율주행 품질의 증거가 아니다. 실패하면 자동 재시도하거나 출력 경로를 재사용하지 않는다.
- 원본 무결성 합격: owner 종료 코드 0, 실행 시작·종료 source archive 해시 불변, 계획·실제 route/world·품질 일치, lifecycle `ready/after/stopped` 모두 PASS와 포트 해제, 충돌 0, 정리 오류 0, 저장 원본 live/replay 정확 일치, RGB/semantic **240쌍 전수** 640×360 디코드, 연속 프레임·0.1초 간격 확인. 0.25 m 이상 떨어진 이전 **모든 보존 위치**에 대해 프레임을 보존하며 **120장 이상**이어야 한다. 240프레임 내 경로 목적지 완주는 요구하지 않고 실제 이동 거리와 관측된 구간만 보고한다.
- 표지판 DEV 자격: 보존 원본에서 CARLA `traffic_sign` 태그 8의 프레임별 픽셀 수와 `0 / 1–4 / 5–64 / ≥65` 크기 분포를 기록한다. 태그 8의 픽셀 덩어리나 여러 프레임의 같은 표지판을 **서로 다른 표지판 객체**로 세지 않는다. 전방 **전체 화각** RGB와 oracle 시맨틱을 시작·중간·끝 및 각 후보 객체의 접근 전·중·후 시점에서 나란히 확인하고, 시간/위치가 이어지는 중복 관측을 묶는다. 서로 다른 실제 표지판 객체 **3개 이상**이 RGB에서 확인되고, 그 확인된 객체 중 **적어도 하나의 의미 라벨이 5픽셀 이상인 보존 프레임 24장 이상**이어야 표지판 DEV 후보로 인정한다. 24장은 독립 객체 수가 아니다. 해석이 불명확하면 보수적으로 실패한다. 전체 화각과 원본 프레임 ID를 함께 보고하고 확대 자른 그림만으로 합격시키지 않는다.
- 분할 보호: Town05_Opt는 통합 모델의 개발 사이트이며 지도·주행 TRAIN 또는 held-out TEST가 아니다. Town02 TRAIN 후보, Town01 기존 TRAIN/DEV, Town04·Town07·Town15 held-out, Town06·Town10HD_Opt 예약 TEST의 데이터를 이 정찰에서 열지 않는다. 표지판 자격 실패 시 직진 DEV의 1–2픽셀 점수로 대신하거나 다른 경로를 사후 선택하지 않는다. 성공해도 객체·분할 감사와 사전 등록 A/B 평가 전에는 학습/모델 승격을 하지 않는다.

실행 담당자가 소유 잠금·포트·출력 부재를 확인한 **뒤에만** 아래 명령을 단 한 번 실행한다. 이 계획 작성 단계에는 실행하지 않았다.

```bash
bash tools/e2e/run_owned_carla_expert_trial.sh \
  storage/experiments/training/2026-09-27/map_town05_right_sign_dev_scout_epic_v1 \
  storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/turn/routes/town05_opt/right/town05_opt_right_s0000_p00.json \
  --port 2100 --quality Epic --wall-timeout-sec 900 --capture-mode mapping-probe -- \
  --frames 240 --motion-mode basic-agent --ignore-traffic-lights \
  --town05-sign-dev-scout
```

이는 정밀지도·교통표지 개체 인식 모델·Autoware 실제 주행 검증이 아니라, oracle 원본을 이용한 **지도 모델 표지판 DEV 자격 정찰**이다.
