# 자연스러운 종점 정지 — 수집기 보정과 데이터 채택 절차

<!-- HH_260906 - Explain expert calibration separately from learned-model control and keep raw failures reproducible. -->

이 절차는 **로컬 CARLA PC**에서 학습 정답을 만드는 수집기를 고치는 작업입니다.
Pro6000에서 CARLA나 Autoware를 빌드·실행하지 않습니다. 기존 Portable 모델을 교체하거나
실차를 움직이는 명령도 아닙니다. 원격은 검증된 새 데이터를 받은 뒤 개인 venv·GPU0에서
학습·평가하는 역할입니다.

## 왜 먼저 수집기를 고치나

기존 주행은 위치상 목표에 도달하면 움직이는 상태에서도 강한 제동으로 꼬리 구간을
시작했습니다. [정답 진단](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/01_target_feasibility/README.md)에서
일부 정답의 급감속이 모델 디코더의 ±2.9 m/s² 한계를 넘는 것을 확인했습니다.
종점 이외 구간에도 충돌이 있으므로, 꼬리 제동 하나만 바꿔 전부 해결됐다고 하지 않습니다.

`comfortable_v1`은 route arc 기반 목표 속도와 실제 정지 유지 조건을 추가한 최초 시험입니다.
출발 가속이 과했고 BasicAgent가 목표보다 일찍 종료했습니다. `comfortable_v2`는 실제
지도상의 종점 waypoint를 연결하고 가속·제동 입력을 제한한 후속 보정입니다. 목표 앞 정지는
됐지만 저속 급감속이 남았고 실제 최고 속도도 16.7 km/h였습니다. 둘 다 수집 품질 FAIL로
학습 데이터에 넣지 않았습니다. 이는 여러 제어 설정을 함께 바꾼 수집기 보정이며, 단일 변수의
모델 A/B 실험이 아닙니다.

기존 수집기 기본값은 `--goal-stop-profile disabled`로 유지합니다. 보정 profile은 명시적으로
선택할 때만 사용하며, 결과를 보고 기존 원본 라벨을 잘라내거나 속도를 바꿔 적지 않습니다.

## 로컬에서 실행 전 확인

1. 로컬 터미널에서 이 저장소의 최상위 폴더로 이동합니다. `pwd`와 `git status`로 위치와
   변경 사항을 확인합니다. 현재 켜 둔 다른 CARLA/Autoware 작업이 있으면 종료 여부를
   먼저 직접 판단해야 합니다. 이 helper는 다른 작업을 대신 종료하지 않습니다.
2. 이미 준비된 CARLA 0.9.15·ROS 환경을 사용합니다. 패키지 설치 명령은 없습니다.
3. 출력 경로는 **새 폴더 이름**으로 정합니다. 이전 결과 폴더를 주면 덮어쓰지 않고 거부합니다.
4. 먼저 가속·제동 응답을 계측한 뒤 결과를 읽습니다. 이 계측 자료 자체를 학습 데이터로
   변환하지 않습니다.

```bash
# HH_260906 - Run only on the local CARLA PC, from the repository root.
bash scripts/e2e/run_owned_carla_expert_trial.sh \
  artifacts/training/2026-09-08/low_speed_response/example_run_001 \
  docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json \
  --port 2100 --quality Low --wall-timeout-sec 900 \
  --capture-mode actuation-response
```

위 명령은 이 실행이 소유하는 CARLA 서버 한 개를 시작하고, 같은 Prius 차량의 짧은
가속·제동 응답을 순서대로 기록한 뒤 자신이 만든 프로세스만 정리합니다. workspace lock과
포트 점유를 먼저 확인합니다. CARLA가 이미 쓰이고 있으면 실패하므로 다른 프로세스를
`kill`해서 우회하지 않습니다. 원격 SSH 터미널에서 실행하면 안 됩니다.

## 계측과 파일 읽는 방법

- 가속 6개 입력: 0.05 / 0.10 / 0.15 / 0.20 / 0.30 / 0.40을 각각 8초 적용합니다.
- 제동 6개 입력: 0.02 / 0.04 / 0.06 / 0.08 / 0.10 / 0.12를 각각 15초 적용합니다.
  제동 전에는 실제 속도가 3 m/s 이상이 될 때까지 가속합니다. 정확히 같은 초기 속도로
  인위적으로 덮어쓰지 않고, 각 시험의 실제 제동 시작 속도를 별도로 기록합니다.
- 각 시험마다 새 차량을 생성하며, 출발 전 3.5초 안정화 기록도 남깁니다. 출발·정지 직전의
  순간값을 fitting 편의상 제외하지 않습니다. 시뮬레이션 시간과 실제 대기 시간은 다릅니다.
- 물리 20 Hz의 실제 상태와 요청·적용 pedal, 기어, 실제 차량 물리 설정을 저장합니다.
  10 Hz 요약은 이 상태를 두 가지 offset으로 추출한 것이며, 카메라 10 Hz 수집을 뜻하지
  않습니다. 이 시험은 카메라·Autoware·학습 모델 제어를 실행하지 않습니다.

출력 폴더의 `owner_plan.json`은 실행 전 설정과 소스 hash, `provenance/`는 실행 소스의
private 원본 사본, `owner_result.json`은 종료·소스 불변성 확인 결과입니다.
<!-- HH_260906 - Distinguish newly archived bound bytes from older captures that retained only their hashes. -->
새 실행은 물리 한계를 정의한 `portable_e2e/model.py`와 `runtime_contract.py`의 원본도
보존하고 `bounds_source_bytes_archived: true`를 기록합니다. 과거 실행에는 이 사본이
없을 수 있으므로, 현재 소스가 달라졌다는 이유만으로 과거 값을 무효화하거나, 반대로
검증 없이 당시 소스가 보존됐다고 주장하지 않습니다.
`lifecycle/ready.json`과 `stopped.json`은 시작·정리 증거입니다. `collector.log`에는 측정
worker의 출력이 저장됩니다. 성공 시 측정 자료는 `actuation/`, 실패 시 `actuation.partial/`에
남습니다. `.partial`을 성공 폴더로 이름만 바꾸지 않습니다.

`complete`는 **선언한 계측을 완료했다**는 뜻입니다. 입력을 받은 차량이 항상 편안하게
움직였거나 자율주행 기능에 합격했다는 뜻이 아닙니다. 오히려 이번 계측은 순간 가감속을
찾기 위한 시험입니다. 실제 RPM·타이어 내부 마찰 상태처럼 API로 측정하지 못한 값은
측정했다고 주장하지 않습니다.

## 타력 주행·출발 반복·스로틀 ramp 후속 계측

<!-- HH_260906 - Keep the second identification matrix distinct from a qualified goal-stop controller. -->

첫 12개 실험에서는 일정한 제동만으로도 저속 급감속이 재현됐습니다.
[실제 결과와 그래프](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/04_pedal_response_calibration/README.md)를
확인한 뒤, 다음 9개 조건을 실행 전에 고정했습니다. 기본 명령은 여전히 첫 12개 실험입니다.
후속 조건은 `--matrix low_speed_v2`를 명시해야 합니다.

```bash
# HH_260906 - Select the nine-case identification matrix explicitly; do not reuse an output directory.
bash scripts/e2e/run_owned_carla_expert_trial.sh \
  artifacts/training/2026-09-08/low_speed_response/example_v2_run_001 \
  docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json \
  --port 2100 --quality Low --wall-timeout-sec 900 \
  --capture-mode actuation-response -- --matrix low_speed_v2
```

- 타력 주행 2회: 기존과 같은 throttle 0.30 준비 후, 측정 속도가 처음 3 m/s 이상이 되면
  throttle·brake를 모두 0으로 하고 20초 기록합니다.
- 출발 반복 3회: throttle 0.15를 8초 유지합니다. 서로 다른 차량으로 시작하지만 같은
  초기 조건의 반복이므로, 서로 독립적인 경로·환경 검증으로 세지 않습니다.
- ramp 4회: throttle을 초당 0.01 / 0.025 / 0.05 / 0.10씩 올려 8초 기록합니다.
  상한은 0.40이지만 마지막 입력은 각각 **0.08 / 0.20 / 0.40 / 0.40**입니다.
  느린 ramp까지 모두 같은 최대 입력에 도달한 비교라고 설명하지 않습니다.

실제 9개 계측은 완료됐습니다. 타력 구간은 스칼라 가감속 한계 안이었으나, 20초 후에도
0.1864 m/s로 움직여 정지 기준 0.1 m/s를 충족하지 않았습니다. 준비 구간의 출발 초과도
그대로 남습니다. throttle 0.15 반복 세 번의 최대 가속은 모두 2.893276 m/s²였지만
2.9 한계까지 여유가 작습니다. 실제로 움직인 ramp 세 조건은 모두 순간 가속 한계를
넘었습니다. 천천히 pedal을 올리면 반드시 해결된다는 가정은 채택하지 않습니다.
[9개 전체 결과·그래프·검증 명령](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/07_coast_ramp_identification/README.md)을
한 카테고리에 모았습니다.

따라서 다음 단계는 더 긴 타력 정지 거리와, 측정 속도에 따라 전환하는 두 단계 출발을
별도 계측하는 것입니다. 이것은 아직 새 주행 profile이나 학습 데이터의 승인이 아닙니다.
물리 엔진 내부 원인은 이 API 기록만으로 확정하지 않으며, 차량 물리 설정·QA 기준을
바꾸거나 출발·정지 표본을 잘라 통과시키지 않습니다.

## Town07 개발용 세 번째 보정 시험

<!-- HH_260906 - Predeclare a bounded empirical pilot and distinguish its nominal speed from measured cruise and dataset admission. -->

`comfortable_v3`는 아래 **정확한 Town07 직진 경로·Prius·ClearNoon에만** 허용하는
개발용 profile입니다. 목표는 28.8 km/h(8 m/s)이며, 30 km/h급 시험이지 명령 속도가
30 km/h인 시험은 아닙니다. 실제 속도 30 km/h 초과는 실패로 기록합니다. 같은 revision은
소유 실행 기록 기준 최대 두 번만 시도하며, 수집기가 자동으로 재시도하지 않습니다.

출발은 throttle 0.15에서 측정 속도 0.5 m/s에 도달하면 기존 정상 PID로 넘기고,
정상 throttle 상한을 초당 0.05씩 최대 0.40까지 높입니다. 정상 brake 상한은 0.10이며
기존 조향과 비상 제동은 유지합니다. 접근 감속 뒤에는 약 3 m/s에서 두 pedal을 0으로
유지하는 타력 정지를 시험합니다. 타력 거리는 단일 초기 조건의 계측에 근거한 값으로,
다른 경사·노면·차량에서 보장되는 제동 거리라고 해석하지 않습니다.

```bash
# HH_260906 - Run one new local expert pilot; this does not start a learned controller or remote training.
bash scripts/e2e/run_owned_carla_expert_trial.sh \
  artifacts/training/2026-09-08/comfortable_goal_stop_v3/town07_straight_calibration/example_run_001 \
  docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json \
  --port 2100 --quality Low --wall-timeout-sec 900 \
  --capture-mode expert -- \
  --physics-hz 20 --capture-hz 10 --target-speed-kmh 28.8 --max-duration-sec 180 \
  --stationary-warmup-sec 3.5 --stationary-tail-sec 6.5 --spawn-z-offset-m 0.5 \
  --weather ClearNoon --seed 0 --goal-stop-profile comfortable_v3 --goal-tolerance-m 1.0 \
  --mapping autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml \
  --calibration src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml \
  --basic-agent-base-min-distance-m 3.0 --basic-agent-distance-ratio 0.5 \
  --basic-agent-lateral-kp 1.95 --basic-agent-lateral-ki 0.05 --basic-agent-lateral-kd 0.2 \
  --basic-agent-max-steering 0.8 --basic-agent-lane-offset-m 0.0
```

합격 조건에는 실제 속도 7.8–8.2 m/s의 **5초 연속 순항**, 기존 가감속 한계,
목표 앞 1 m 이내·0.1 m/s 이하 2초 정지 유지·정지 후 6.5초 기록이 모두 포함됩니다.
출발 제한은 8초, 타력 제한은 45초입니다. 비상 제동은 그대로 적용하고 다음 물리 tick을
기록한 뒤 해당 개발 시험을 실패로 끝냅니다. 정지 실패를 숨기기 위한 뒤늦은 정상 제동이나
재출발 보정은 하지 않습니다. 성공적으로 기록하더라도 `training_data_approved: false`이며,
독립 수치·영상·미래 XY 품질 검사와 별도 데이터 채택이 필요합니다.

## 명령 수신 확인과 프레임 정합성 비교

<!-- HH_260906 - Distinguish acknowledged server acceptance and coherent observations from proven physical actuator timing. -->

기존 수집의 기본 전송 방식은 `legacy_async`입니다. 실제 두 번의 v3 기록에서 직전
제어 요청과 다음 상태의 API 제어값이 일치하지 않는 경우가 발견됐습니다. CARLA 후보
소스는 비동기 제어 RPC를 사용하지만, 이 사실만으로 관측된 모든 차이를 실제 물리 지연으로
확정하지 않습니다. 기록 시점과 actuator 내부 적용 시점은 별도 문제입니다.

별도 전송·계측 비교에는 위 수집 명령의 마지막 `--` 뒤에
`--control-transport acknowledged_batch`를 추가합니다. 매 명령을 한 차량에 대한
동기 batch로 보내고 서버 응답을 확인하되, batch 자체는 world tick을 진행하지 않습니다.
그다음 소유 수집기가 물리 tick을 정확히 한 번 진행합니다. bootstrap·warmup·주행·tail·
실패 정지 모두 같은 전송 경로를 사용하고 기존 비동기 호출로 자동 전환하지 않습니다.

이 모드에서는 pose·속도·가속도·각속도를 해당 frame의 immutable actor snapshot에서
읽고, 제어 요청 순번·응답·보고된 제어값과 frame 관계를 추가 기록합니다. 현재 값이
직전 수신 확인 명령과 맞지 않으면 해당 관측을 남기고 실패 종료합니다. 자동 변속기가
정하는 실제 gear는 기록하되 요청 gear와 같도록 강제하지 않습니다. 서버 응답은 **명령
수신 확인**이지 실제 토크가 같은 순간에 작용했다는 증명이 아닙니다.

이 비교는 전송과 관측 일관성을 함께 개선하는 실험이며, 물리 설정·정답 라벨·모델·
속도 및 가감속 한계는 바꾸지 않습니다. raw 3D 값 추가는 가속도 기준점을 보정한 것도
아닙니다. 실제 비교 결과와 독립 감사가 끝나기 전에는 학습 데이터로 채택하지 않습니다.

## 작업 종료 시각을 넘기지 않도록 새 실행 제한

<!-- HH_260906 - Explain the explicit UTC admission budget without promising hard real-time process termination. -->

소유 실행 helper의 `--finish-before-utc`는 작업 경계를 **UTC**로 받습니다. 예를 들어
2026-09-08 오전 10시 한국 시간은 `2026-09-08T01:00:00Z`입니다. 위 명령에서
`--capture-mode`와 같은 helper 옵션 위치, 즉 마지막 `--`보다 앞에 넣습니다.

```bash
# HH_260906 - This is an additional owner option, not a command or a collector option.
--finish-before-utc 2026-09-08T01:00:00Z
```

서버를 띄우기 전에는 수집 wall timeout 외에 330초를, 시작된 후 수집 직전에는 120초를
더 확보할 수 있어야 합니다. 여유가 부족하면 새 수집을 거절하고 자신이 만든 서버만
정리합니다. 지정 시각·예약 정책은 `owner_plan.json`에 기록합니다. 옵션을 생략하면
기존 시간 제한 방식이 유지됩니다. 이것은 시작 전 시간 예산 검사이며, 운영체제 지연까지
막는 하드 실시간 종료 보장은 아닙니다. 다른 프로세스를 종료하지 않습니다.

## 정답 데이터로 채택하기까지

### 도로 표면·화질만 별도로 비교하기

<!-- HH_260906 - Isolate stationary visual quality from moving control experiments and never export the probe as training data. -->

도로가 체크무늬처럼 보이는 등 시각 자료에 문제가 있으면 주행 제어 시험과 분리해
`stationary-camera`를 사용합니다. 정확한 Town07 경로의 시작점에 Prius를 만들고,
스로틀·조향은 0, 브레이크는 1로 유지한 채 8초의 simulation 시간을 관찰합니다.
저장 시점은 5·6·7·8초이며 각 시점의 실제 6개 카메라를 모두 저장합니다.
이 진단은 **주행도 Common10 학습 데이터 수집도 아닙니다.**

```bash
# HH_260906 - Compare Low and Epic in separate new owned worlds; no shared package settings are edited.
bash scripts/e2e/run_owned_carla_expert_trial.sh \
  artifacts/training/2026-09-08/stationary_camera_quality/example_low \
  docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json \
  --quality Low --wall-timeout-sec 300 --capture-mode stationary-camera -- \
  --mapping autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml \
  --calibration src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml
```

Epic 비교는 출력 폴더를 새 이름으로 바꾸고 `--quality Epic`만 변경합니다. CARLA·
Autoware가 이미 실행 중이면 helper가 거부합니다. 두 결과의 `camera_audit/manifest.json`,
`camera_frames.jsonl`, `images/`에서 실제 상태·이미지·출처를 비교하며, 렌더링 품질을
바꾼 결과를 제어 알고리즘의 개선으로 세지 않습니다. 카메라는 이 진단에서 물리 tick마다
관찰하고 일부 시점만 저장하므로, 저장 이미지 수가 10 Hz 학습 수집률을 뜻하지 않습니다.

### 주행 데이터 채택 검사

<!-- HH_260906 - Render diagnostic evidence directly without turning a failed raw episode into validated training samples. -->

종료된 실제 수집의 6개 카메라·차량 중심 경로를 보려면 로컬에서 다음을 실행합니다.
출력은 새 폴더여야 하며, 실패한 `episode.partial`도 원래 실패 상태로 표시합니다.

```bash
# HH_260906 - This is a recorded-data dashboard, not a live Autoware screenshot or learned driving run.
python3 -m scripts.e2e.render_carla_raw_trial \
  artifacts/training/2026-09-08/comfortable_goal_stop_v3/town07_straight_calibration/run_001 \
  artifacts/training/2026-09-08/comfortable_goal_stop_v3_visual/example_run_001 \
  --camera-stride 5 --playback-fps 10
```

원본 시야를 자르지 않은 6개 카메라, 항상 중앙에 놓인 차량과 전체 경로 inset,
관측 속도·가감속·제어값·주행 phase를 함께 표시합니다. `Reported` 제어값은 API가
보고한 값이며 해당 프레임의 물리 적용 시점이 별도로 입증됐다는 뜻은 아닙니다.
GIF는 10 Hz 원본 카메라의 다섯 장마다 한 장을 10 fps로 재생하는 약 5배속 미리보기입니다.
실제 simulation timestamp를 함께 보여주며 실시간 성능 계측으로 사용하면 안 됩니다.
주황색 미래 궤적은 기록에서 관측한 위치이고 모델 예측이 아닙니다. 마지막 부분에 없는
미래 위치를 만들거나 학습 sample로 내보내지 않습니다. 표시한 원본 이미지와 metadata의
SHA, PNG/GIF에 사용한 frame index는 `visual_provenance.json`에서 확인합니다.

계측 결과로 새 수집 제어 profile을 정하고 변경 이유·설정을 고정한 뒤, 우선 동일 Town07
직진 경로에서 다시 수집합니다. 이때 목표 속도 30 km/h와 실제 도달 속도, 자연스러운 접근,
목표 1 m 이내 정지·0.1 m/s 이하 2초 유지·정지 후 6.5초 기록을 따로 확인합니다.
기준점은 현재 `base_link`이며 전방 범퍼의 정지선 안전 판정을 대신하지 않습니다.

소스·route·센서 hash, camera coverage/cadence, 실제 가감속, 충돌·차선 침범을 확인하고
Common10 변환·라벨의 속도 및 독립 XY 진단을 통과해야 새 데이터 후보가 됩니다.
scalar 가감속 검사만으로 전체 decoder의 표현 가능성이나 주행 안전을 보장하지 않습니다.

<!-- HH_260906 - Enforce an explicit native restriction without treating absent legacy markers as a formal approval proof. -->
Common10 변환기는 이제 원본에 명시된 `training_data_approved: false` 또는
`development_only: true`가 있으면 변환을 거부합니다. 수집 완료와 학습 데이터 승인은
다르기 때문입니다. 이 값을 원본에서 지워 우회하지 않습니다. 과거에 해당 표시가 없던
데이터는 기존 호환성을 유지하지만, 표시가 없다는 사실 자체를 새 승인 증거로 간주하지
않습니다. 이미 변환된 데이터의 학습 단계에 외부 승인 기록을 연결하는 검사는 별도
후속 과제이며, 이번 원본 변환기 검사만으로 전체 경로가 보호된다고 주장하지 않습니다.

그 후에 회전·독립 episode를 늘리고 별도 버전으로 전송합니다. 기존 train/val/test를
덮어쓰거나 같은 주행을 잘라 split하지 않습니다. 새 학습은 원격 개인 venv·GPU0에서
고정된 조건으로 수행하고 모든 seed 결과를 비교합니다. 실패 raw 자료는 원인과 분모를
남기되, 공개 영상이나 학습 데이터에 성공으로 섞지 않습니다.

[현재 결과 모음](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/README.md)
