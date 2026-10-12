# 다음 우회전 주행 자료: Town01 새 진입 방향 1개를 오프라인 사전 고정

<!-- HH_260928 - Keep a deterministic offline route separate from owned driving and model approval. -->

**후속 결론: 정적 경로 1개를 찾아 소유 CARLA에서 BasicAgent 우회전 1회를 촬영했으나 원본 수집은 실패했고, 데이터 편입·모델 학습은 0이다.** [실패 원인·전방 화면·궤적](FAILED_TRIAL.md)을 먼저 본다. [고정 경로](provisional_route.json)는 이전 BasicAgent C/p06과 다른 Town01 우회전 진입 방향이다. [검사 수치](feasibility.json)는 30–120m 전체의 *명목* Prius 차체와 보수적 sweep이 기존 **의도 차로 안쪽 10cm** 경계에 들어감을 보인다. 이 정적 통과가 실제 주행의 목표·연속 이동 기준을 보장하지 않는다는 것을 후속 단회 실행에서 확인했다. 원래 판정 `STATIC_FEASIBLE_OWNER_REQUIRED`와 경로 파일의 `capture_authorized=false`는 소급 변경하지 않는다.

| 고정 항목 | 값 |
| --- | --- |
| 경로 재구성 | 보관된 C 원본 **live Town01 XODR** SHA256 `97a7f6ac...709`에서 junction 87, road 103/lane −1의 우회전 connector를 정확히 1개 선택. 진입점 `previous(60m)[0]`, 출구 `next(80m)[0]`, 1m 오프라인 GlobalRoutePlanner. 랜덤 seed 재검색이 아님 |
| 출력 | `provisional_route.json` SHA256 `6484a43d04a98b9d01d27994e8347545b7c350aab39ece542e186bd28d73b13e`; 길이 **152.106m**. 파일 자체에 `capture_authorized=false` 표기 |
| 우회전 | **59.970–74.231m**, road/lane `103/-1 → 5/+1`, 회전 중심 ROS `(85.763, −324.067)m`. 기존 `speed_30kph`는 *형상* profile로 통과했으며 30km/h 주행 승인이 아님 |
| 고정 평가창/미래 | 30–120m, 9km/h × 6.4초 = 16m, 추가 경로 guard 10m. 경로 끝은 146m 필요치보다 **6.106m** 더 멂. 실제 30초 이동과 실제 64개 미래 관측은 아직 없음 |
| 명목 몸체 | 0.1m 간격 **차체 0/901, sweep 0/900** raw 및 10cm inset 침범. 최소 inset sweep 여유 **0.091992m**는 10cm 안쪽 경계까지의 *추가* 여유이고, raw 차로 경계까지의 최소 sweep 거리는 0.191992m. 연속시간 또는 실제 주행 보증은 아님 |
| 중복 | 기존 TRAIN/V4/C 18개 기록과 V5/복합 경로의 18개 route-site 표기를 SHA 고정해서 비교. 같은 connector·5m 회전 중복은 없음. 그러나 V5의 **다른 진입 방향** 회전 중심이 약 **9.816m** 떨어져 있어 같은 교차로일 수 있다. 이를 독립 교차로 2개로 세지 않음 |

기존 [소유 전문가 실행기](../../../../../tools/e2e/run_owned_carla_expert_trial.sh)는 이미 `OUTPUT_ROOT ROUTE_JSON` 형태의 **정확한 고정 경로 파일**을 받는다. 새 카탈로그 seed 검색은 필요 없다. 그러나 이 provisional JSON을 곧바로 학습 데이터로 쓰지 않는다. 다음 단회 진단의 출력은 부재가 확인된 `storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1`로 사전 고정한다. Town13 소스 에디터와 같은 포트/잠금을 동시에 소유하지 않는다. 실행 직전 경로 SHA·source 해시·포트 2100–2102·CARLA 프로세스·공용 runtime lock을 확인한다. 현재 서버의 `world.get_map().to_opendrive()` SHA가 위 보관 XODR과 다르면 정적 판정은 무효이며 수집을 중단한다. owner wrapper 단독으로는 그 동일성을 보장하지 않으며, **아래 명령의 collector 해시 옵션**이 차량 생성 전에 불일치를 거부한다.

2026-09-28에 collector에 선택형 `--expected-live-xodr-sha256`를 추가했다. 맵 이름을 확인한 뒤, 물리 설정·날씨·차량 생성 전에 live XODR의 **정확한 bytes**를 검사하고, 같은 bytes를 아카이브한다. 불일치·OpenDRIVE 부재는 해당 실행의 `episode.partial/manifest.json`에 실패로 남으며 차량을 생성하지 않는다. 옵션을 생략한 과거 실행에는 영향이 없다. 이 장치의 오프라인 단위·fake-world 테스트와 후속 [live 서버 일치](FAILED_TRIAL.md)는 확인했다. 그러나 그 주행의 원본 수집은 미달로 끝났다.

아래는 소유자·포트·기존 C 제어 계약을 고정한 **이미 실행한 v1 단회 진단 명령**의 재현 기록이다. **다시 실행하면 기존 증거와 충돌하므로 그대로 재사용하지 않는다.** 이 문서 자체는 `capture_authorized=false`이며 새 카탈로그나 학습 작업을 자동 시작하지 않는다.

```bash
cd /home/a/autoware_e2e
sha256sum docs/assets/validation/2026-09-28/drive_town01_j87_prereg/provisional_route.json
# 기대값: 6484a43d04a98b9d01d27994e8347545b7c350aab39ece542e186bd28d73b13e
bash tools/e2e/run_owned_carla_expert_trial.sh \
  storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1 \
  docs/assets/validation/2026-09-28/drive_town01_j87_prereg/provisional_route.json \
  --port 2100 --quality Epic --wall-timeout-sec 900 --capture-mode expert -- \
  --expected-live-xodr-sha256 97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709 \
  --target-speed-kmh 9 --max-duration-sec 180 --physics-hz 20 --capture-hz 10 \
  --stationary-warmup-sec 3.5 --stationary-tail-sec 6.5 \
  --control-transport acknowledged_batch --agent-initialization after_bootstrap \
  --weather ClearNoon --seed 2026092702 --vehicle-type vehicle.toyota.prius \
  --spawn-z-offset-m 2 --wheelbase-m 2.85 --goal-stop-profile disabled \
  --basic-agent-max-steering 0.45 --basic-agent-base-min-distance-m 1.5 \
  --mapping autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml \
  --calibration autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml
```

단회 진단을 열게 되면 기존 C와 같은 Epic/ClearNoon, Prius, 9km/h, physics20Hz·여섯 카메라10Hz, ACK, warmup3.5s/tail6.5s, after-bootstrap BasicAgent, 정규화 조향 상한0.45, waypoint purge1.5m, wheelbase2.85m, **waypoint road-Z0의 actor spawn만 +2m**를 고정한다. [C의 원본 owner 계획](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/owner_plan.json)과 같은 collector flag 조합이 현재 파서에 통과함을 확인했다. route 파일의 base_link 도로 고도 자체는 올리지 않는다. 새 경로이므로 이는 A/B 성능 비교나 학습된 모델 제어가 아니다.

계획했던 **전체 원본** 기준은 고정 30–120m의 30초 이상 연속 `>0.5m/s` 이동, 모든 10Hz anchor의 실제 64점/6.4초 미래, 9개 원래 운동 한계(특히 XY 곡률≤0.2/m), raw·10cm inset 차체/sweep 침범0, 충돌0·목표 도달, 여섯 카메라/ACK/20Hz 차체 상태, live XODR/actor bbox 전량 검사다. v1은 [연속 이동 28.35초와 목표 거리 2.552m에서 실패](FAILED_TRIAL.md)했으므로 나머지 부분을 성공 판정으로 돌리지 않는다. 미달 anchor를 제거하거나 기준을 완화하지 않는다. 다음 수집이 통과하더라도 **Town01 한 Town의 한 새 진입 방향 전문가 원본**일 뿐이다. 다른 DEV site와 반복 TRAIN episode가 있어야 PRO6000 물리 GPU0에서 새 가중치 학습을 시작한다.

## 데이터 분할과 다른 지도 검색의 경계

<!-- HH_260928 - Prefer the integrated research split over the older global inventory labels. -->

전역 `inventory_carla_training_maps.py`는 Town06/Town10HD_Opt를 TRAIN으로 표시하지만 [통합 연구 분할](../../../../portable-e2e-data-expansion-2026-09-15.md)과 [최신 지도·주행 분할](../../2026-09-27/map_next_training_design/README.md)은 두 Town을 **예약 TEST**로 보호한다. 이번 오프라인 탐색 초기에 Town10HD_Opt의 과거 p05 경로 형상을 읽고 진단했지만 분할 충돌 확인 즉시 TRAIN 제안을 철회했다. 해당 Town의 새 촬영·모델 점수/라벨 조회·훈련 데이터 사용은 **하지 않았고**, 읽힌 경로 기하를 향후 미사용 TEST의 새로운 독립 경로 선정 증거로도 쓰지 않는다. Town04·07·15 held-out는 열지 않았다.

Town02_Opt의 보관 우회전 3개는 같은 물리 회전이었고, 10cm inset 회전 중심부 명목 차체 **64개**, sweep **70개**가 차로 밖이었다. 추가 오프라인 topology 탐색은 오른쪽 connector 16개에서 고정 upstream50m/downstream90m endpoint 조합 **60쌍**을 검토했으며, 전체 물리형상/30–120m/6.4초 길이 조건을 넘은 것은 1쌍이었지만 그 쌍도 위 10cm 명목 기준에 실패했다. 이 *고정 탐색 범위* 안에서는 Town02 새 수집 후보가 없다. Town02의 과거 확장 계획에는 DEV 제안이 있었고 최신 통합 지도 계획에는 TRAIN 후보이므로, 실제 편입 전 site split을 기계 판독형으로 잠가야 한다. Town03은 주행 연구 DEV이나 과거 지도 C의 TRAIN 노출이 있어 새 통합 독립 DEV로 자동 인정할 수 없다. Town05_Opt도 전역 TRAIN/새 DEV 충돌과 긴 경로 검색 0개 때문에 아직 독립 DEV가 아니다. Town13은 전역 validation이지만 기존 짧은 우회전 8개가 9km/h의 120m+6.4초 미래 예산보다 모두 짧고, 실제 DEV 운전·site exposure 검사가 없다. **독립 DEV 합격 경로는 현재 0개**다.

재현(서버·GPU·SSH 없음):

```bash
python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_preregister_town01_j87_right.py
# 새 출력 경로를 명시하지 않으면 기존 증거를 덮지 않으므로 아래는 재실행 시 거부된다.
python3 tools/workspace_python.py tools/e2e/preregister_town01_j87_right.py
```

[선별 코드](../../../../../tools/e2e/preregister_town01_j87_right.py)와 사전등록 테스트 5개, collector 테스트 46개가 통과했다. 이 오프라인 단계에서 CARLA 서버, GPU, SSH, 패키지 설치, 사진/GIF/Autoware 구동, 데이터 편입 및 모델 학습은 없었다.
