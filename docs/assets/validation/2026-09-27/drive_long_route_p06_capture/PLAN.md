# Town01 신규 교차로 s0002_p06: 9 km/h 원본 우회전 1회 사전 선언

<!-- HH_260906 - Permit one bounded BasicAgent raw capture while denying moving-task admission until a separate immutable raw audit passes. -->

**사전 상태: `NOT_RUN_NOT_ADMITTED`.** 목적은 학습 모델 주행이 아니라, 기존 우회전 정답 부족을 해결할 *새 물리 교차로*의 BasicAgent 전문가 원본을 한 번 수집하는 것이다. 기존 Town01 `s0001_p00` TRAIN과 V4 `s0002_p06`의 회전 중심은 66.24m 이상 떨어져 있고 최단 경로 거리는 38.71m이다. V4 경로는 229.671m, 단일 RIGHT, 회전 전 39.980m/후 173.999m이며 정적 자체 루프가 관측되지 않았다. `p06`라는 숫자는 기존 짧은 Town01 p06 경로와 우연히 같은 pair 번호일 뿐 **그 경로의 연장이 아니다.** 카탈로그/형상 점검만으로 새 학습 episode가 승인되지는 않는다.

| 고정 입력·조건 | 값 |
| --- | --- |
| split/site | Town01 `TRAIN`의 신규 교차로 후보; 다른 seed·파일명을 독립 표본 수로 세지 않음 |
| route | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json`, SHA256 `49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142` |
| 카탈로그/owner | SHA256 `0c0cade80ec797de39320157e35c90661ac172caef5fa7a21368036a814c5897`, 완료된 [V4 소유 결과](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/owner_result.json) |
| 소유 wrapper | `tools/e2e/run_owned_carla_expert_trial.sh`, SHA256 `0723336ac4b7649004e5cc2f4bfce95e7c0f12caf941a03839096d04cd9f507b` (`CARLA_READY` 검색만 서비스 환경용 `grep -a`로 수정) |
| collector | `tools/e2e/collect_carla_vad_expert.py`, SHA256 `d22af16f051ee7835f41363d85934a3060ed9bd6dab61b727a8458ed7c30a08a` |
| mapping/calibration | `autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml` SHA256 `9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136`; `autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml` SHA256 `5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea` |
| Common10 미래 계약 | `learning/portable_e2e/config/common_10hz_v1.contract.json` SHA256 `6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642`, 10Hz·64점/6.4초·최소 연속 이동 30초 |
| 서버와 출력 | packaged CARLA Town01, Epic, 127.0.0.1:2100 단일 소유; 새 루트 `storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_trial_v1/`만 사용 |
| 출처·제어 | 20Hz 물리/상태, 10Hz 여섯 카메라, 목표 9km/h, ClearNoon, seed 2026092702, Prius, ACK batch, BasicAgent만 제어, `client.load_world` 금지 |
| 실행 한계 | sim 180초, wall 300초, 정지 warmup 3.5초·goal 뒤 정지 tail 6.5초, **단일 시도/자동 재시도 없음** |

**실제 명령:** 아래 명령을 사용자 서비스 `autoware-e2e-town01-right-p06-9kph-v1`로 시작하여 터미널 세션 수명과 분리한다. 시작 전 출력 부재·workspace lock·2100–2102 포트의 타 작업 부재·route/소스 SHA를 확인한다. launcher 성공은 수집 성공이 아니고, `owner_result` 및 세 lifecycle 영수증을 끝까지 확인한다.

```bash
bash tools/e2e/run_owned_carla_expert_trial.sh \
  storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_trial_v1 \
  storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json \
  --port 2100 --quality Epic --wall-timeout-sec 300 --capture-mode expert -- \
  --target-speed-kmh 9 --max-duration-sec 180 --physics-hz 20 --capture-hz 10 \
  --stationary-warmup-sec 3.5 --stationary-tail-sec 6.5 \
  --control-transport acknowledged_batch --agent-initialization after_bootstrap \
  --weather ClearNoon --seed 2026092702 --vehicle-type vehicle.toyota.prius \
  --spawn-z-offset-m 2 --wheelbase-m 2.85 --goal-stop-profile disabled \
  --mapping autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml \
  --calibration autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml
```

**수집 후 편입 관문:** 일반 `expert` collector는 목표까지 운전하고 전체 raw를 보존하지만, 이 우회전의 30초 이동·6.4초 미래를 직접 승인하지 않는다. 봉인된 15km/h 직진 전용 `--moving-task-plan`을 이 경로에 넘기지 않는다. 독립 QA는 원본 route와 실제 ROS base_link 위치에서 진행거리를 다시 계산한 뒤, 속도/모델 결과를 보고 구간을 고르지 않고 **모든 주행 카메라 anchor 중 진행거리 30–120m에 든 것 전부**를 선택한다. 첫 native progress >120m에서 선택을 영구 종료한다. 그 집합이 실제 시간 기준 최소 30초 연속이고 전부 >0.5m/s인지, 회전 시작/끝이 포함되는지, 각 anchor 뒤 정지 tail 이전의 0.1초 간격 64개 실제 미래 점이 빠짐없이 있는지 검사한다. RGB 여섯 방향/타임스탬프·ACK/native state·충돌·차체 전체 의도 차로·progress 단조·원래 9개 motion bound도 전체 선택 anchor에 적용한다. 위반 anchor를 제거하거나 미래 경로를 가상 재생성해 통과시키지 않는다. 기존 straight 전용/comfortable-stop 전용 감사 CLI는 이 원본의 최종 승인기로 사용하지 않는다.

한 조건이라도 실패하면 출처·화면·실패 원본을 보존하고 `dataset_admitted=false`, `training_started=false`, `learned_control=false`로 마감한다. 합격하더라도 이것은 TRAIN 후보 1 episode이지 모델 성능이나 30km/h/60km/h 운전 증거가 아니다. 별도 Town03 DEV와 동일 조건의 학습 A/B, 미사용 TEST·폐루프까지 필요하다.
