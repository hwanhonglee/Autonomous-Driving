# Town05_Opt 지도 독립 DEV 후보 — 수집 전 계획

<!-- HH_260906 - Freeze a map DEV source without opening driving or map holdouts. -->

**목적:** 지도 AI v2/B/C에서 반복 확인한 Town01 DEV의 과적합 위험을 줄이기 위해, 현재 지도 TRAIN Town01+Town03과 다른 Town05_Opt의 새 RGB·CARLA 시맨틱 원본을 **지도 DEV 후보**로 별도 수집한다. 이는 결과를 미리 알거나 새 모델 채택을 허용하는 계획이 아니다. 기존 AI v2/B/C를 이 원본으로 바로 점수화하지 않고, 소유·센서·split 자격을 먼저 확인한다.

- 고정 경로: `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/straight/routes/town05_opt/straight/town05_opt_straight_s0000_p00.json`, SHA256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c`, 원본 route length 209.77275 m.
- 고정 수집: 소유 CARLA Town05_Opt·ClearNoon, BasicAgent 물리 주행, 전방 RGB/깊이/시맨틱 각 640×360·90°·10 Hz, 240 동기 프레임. 신호 무시 설정은 기존 지도 수집과 동일하며 교통규칙 평가로 사용하지 않는다.
- 자격 조건: 소유 run 종료 코드 0, 수집 전후 실행 소스 바이트 동일, 세계/경로 SHA 일치, 다른 차량·센서와 충돌 없음, 원본 240쌍 디코드와 프레임/시각 일치, collision 0, 동일 Town 내 0.25 m 위치 중복 제거 뒤 ≥120장. 결과가 미달이면 **DEV 채택 불가**; 기준 완화·프레임 선택 재시도 없이 별도 보고한다.
- split 경계: 이 경로를 지도 TRAIN이나 TEST에 넣지 않는다. 주행 모델의 미사용 Town04 straight p00 TEST와 지도 Town07 straight/right 봉인을 열지 않는다. 현재 주행 계획의 Town05_Opt right는 development VAL이므로, 같은 site의 다른 route라는 사실을 기록하고 향후 통합 모델의 VAL site로만 다룬다. 다른 site/route 분리 요구가 더 엄격해지면 채택 전에 재검토한다.
- 수집 중 oracle 정답은 **센서/라벨 품질 확인에만** 쓰고, AI v2/B/C 예측·IoU를 계산하지 않는다. 후속 모델 A/B의 기준과 평가 분모를 고정한 뒤에만 DEV 점수를 연다. 이번 자료 수집은 학습·정밀지도 생성·Autoware 실행·실차 승인이 아니다.
