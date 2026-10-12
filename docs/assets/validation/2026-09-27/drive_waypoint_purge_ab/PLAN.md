# Town01 긴 우회전 B/C: BasicAgent waypoint purge 3.0 → 1.5 m

<!-- HH_260906 - Preregister a single expert-tracking factor while keeping curvature and full-body lane gates unchanged. -->

**사전 상태: `NOT_RUN_DEVELOPMENT_ONLY`.** [B 원본 QA](../drive_steering_cap_ab/QA_v1/QA.md)는 9 km/h·조향 상한 0.45에서 고정 360개 anchor의 6.4초 미래와 원래 9개 운동 기준을 모두 통과했지만, [차체 검사](../drive_capture_geometry_provenance/LANE_GATE.md)에서 의도 차로 밖의 차체를 확인해 **학습 편입 실패**했다. [경로 자체 정적 진단](../drive_turn_route_intrinsic/README.md)은 완벽한 명목 경로를 따를 경우 차량 외곽이 차로 안에 있을 가능성을 보이는 진단이지 실제 주행 승인이나 인과 증명은 아니다. 다음 가설은 BasicAgent가 약 4.23m 앞의 waypoint까지 제거하며 코너를 안쪽으로 자를 수 있다는 것이다. `base_min_distance`만 3.0→1.5m로 낮춰 약 2.73m로 만들면 추종 오차가 줄 수 있다. 반대로 곡률·조향 진동이 나빠질 수 있다.

| 계약 | C 값 |
| --- | --- |
| 목적 | 같은 Town01 TRAIN 경로의 전문가 추종기 B/C 진단 **1회**. 새 독립 site나 학습된 모델 주행으로 세지 않음 |
| route | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json`, SHA256 `49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142` |
| 고정 조건 | packaged Town01, Epic, ClearNoon, seed `2026092702`, Prius, 9km/h, 20Hz physics·10Hz 여섯 카메라, warmup3.5s/tail6.5s, ACK, after-bootstrap BasicAgent, normalized steering cap0.45, lane offset0, lateral PID1.95/0.05/0.2, distance ratio0.5s, goal-stop disabled |
| 유일한 제어 요인 | B `base_min_distance=3.0m` → C `--basic-agent-base-min-distance-m 1.5`; 동일 속도에서 총 purge 거리 약 4.23→2.73m. 다른 옵션/route 변경 금지 |
| 출처 | collector SHA256 `5dd1654afe2c8357db1c1b44c81bef180ce675c727d4bc021483e73ce07ba24e`; owner wrapper SHA256 `0723336ac4b7649004e5cc2f4bfce95e7c0f12caf941a03839096d04cd9f507b` |
| 단일 출력/소유 | `storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/`; user service `autoware-e2e-town01-right-purge15-v1`; ports2100–2102 및 runtime lock 단독 |
| 한계 | sim≤180s, collector wall≤300s, 한 번만 실행. 자동 재시도, 데이터 편입, GPU/SSH/학습, 모델 승격 없음 |

**평가 기준은 C를 보기 전에 고정한다.** B와 동일하게 실제 원본 좌표로 경로 30–120m의 모든 camera anchor를 첫 120m 초과 시점 전까지 선택한다. 30초 연속 이동>0.5m/s, 20/10Hz·ACK·여섯 카메라 동기, 각 anchor 실제 64점/6.4초 미래(정지 tail 전), 기존 아홉 운동 bound—특히 최고 XY 곡률≤0.2rad/m—, 충돌0, 목표 도달, spawned actor 치수와 동일 실행 live OpenDRIVE에 대한 **차체 전체 의도 차로 포함**을 모두 요구한다. 실패 구간/anchor를 제거하거나 threshold를 바꾸지 않는다. B/C 최고 곡률·위반 anchor/tick, 차체 raw/inset 침범 프레임·면적, 독립 CTE, 조향·회전·속도·목표 도달을 나란히 기록한다. C가 통과하더라도 이 한 번의 expert capture만으로 학습 모델 품질·일반화를 승인하지 않는다.

시작 직전 route/collector/wrapper SHA, 출력 폴더 부재, 포트·다른 CARLA 실행 부재와 lock을 확인한다. 소유 wrapper의 lifecycle·source 아카이브를 보존한다. 실패 시 다른 lookahead 값을 즉시 재시도하지 않는다.
