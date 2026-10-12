# Town01 긴 우회전 A/B: 정규화 조향 상한 0.80 → 0.45

<!-- HH_260906 - Preregister one bounded expert-controller factor without relaxing raw-motion or lane gates. -->

**사전 상태: `NOT_RUN_DEVELOPMENT_ONLY`.** 기준 A는 [9 km/h 단회 원본 QA](../drive_long_route_p06_capture/QA_v3/QA.md)의 Town01 `s0002_p06`이다. 30–120m 고정 선택의 360개 anchor 중 63개 미래 창이 우회전 한 0.1초 tick의 XY 곡률 **0.229889 rad/m > 원래 0.200000**을 포함한다. 그 구간의 정규화 조향 명령은 0.542까지 올라갔다. **가설**은 BasicAgent의 `max_steering`만 0.80에서 0.45로 낮추면 실제 곡률의 단발 최고값이 줄 수 있다는 것이다. 방향·크기·차체 차로 유지의 결과는 모른다. 더 약한 조향으로 경로 오차·차로 이탈이 악화될 수 있다.

| 고정 계약 | B 값 |
| --- | --- |
| 목적·분할 | 같은 Town01 TRAIN 물리 교차로의 **제어 A/B 진단 1회**. 다른 독립 site나 학습 모델 주행으로 세지 않음 |
| route | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json`, SHA256 `49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142` |
| 입력 환경 | A와 동일한 packaged Town01, Epic, ClearNoon, seed `2026092702`, Prius, 9 km/h, 20Hz physics/10Hz 여섯 카메라, 3.5초 warmup/6.5초 tail, ACK, after-bootstrap BasicAgent |
| 유일한 제어 요인 | A는 기본 `--basic-agent-max-steering 0.8`; B는 **명시적 `--basic-agent-max-steering 0.45`**. PID·차로 offset·goal-stop·차량/센서 설정은 동일 |
| 증거 수집 변경 | B collector는 향후 QA를 위해 실제 actor 바운딩박스와 live OpenDRIVE를 manifest/episode에 보관한다. **제어 로직은 변경하지 않았다.** collector SHA256 `5dd1654afe2c8357db1c1b44c81bef180ce675c727d4bc021483e73ce07ba24e` |
| wrapper | `tools/e2e/run_owned_carla_expert_trial.sh` SHA256 `0723336ac4b7649004e5cc2f4bfce95e7c0f12caf941a03839096d04cd9f507b` |
| 출력/소유 | fresh `storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/`; user service `autoware-e2e-town01-right-steer045-v1`; port 2100–2102 단독 |
| 실행 제한 | sim 180초, wall 300초, 단일 시도, 자동 재시도·DATASET 편입·GPU/SSH/학습 없음 |

**평가를 먼저 고정한다.** 원본 해시와 owner lifecycle·goal/collision/ACK/여섯 카메라 확인 뒤, A와 같은 물리 경로/같은 seed에서 *실측* route progress 30–120m의 **모든** 주행 camera anchor를 선택한다. 첫 native >120m에서 닫고 30초 연속 >0.5m/s, 20/10Hz 동기, 각 anchor의 정지 tail 전 **64점/6.4초 전체 미래**, 원래 아홉 motion bound(특히 XY 곡률 **≤0.2 rad/m**), 차량 전체 의도 차로 포함을 그대로 검사한다. 위반 창을 자르거나 한 tick을 제외해 통과시키지 않는다. A 대비 최고 곡률과 위반 tick/anchor, turn progress/yaw, 최대 CTE, 조향·속도 및 목표 도달을 나란히 기록한다. B가 곡률 0이어도 차체 차로/다른 원본 기준이 미검증이면 편입 불가다. 기존 A도 조향 외 source/instrumentation 차이가 있어 통계적으로 독립인 다회 검증은 아니다.

시작 전 출력 부재, lock/포트 타 사용자 부재, 소스/route 해시, wrapper 옵션 parse 및 이 사전 계획을 확인한다. 실패나 종료 신호는 그대로 보존하고 다른 speed/상한으로 즉시 재시도하지 않는다. 학습과 독립 Town03 DEV는 별도 관문이다.
