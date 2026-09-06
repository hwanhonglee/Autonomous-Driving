# Portable E2E 10 Hz shadow runtime 초보자 가이드

> 기준일: 2026-09-06
>
> 현재 단계: **physical-v1 학습·val337·runtime gate v8·CARLA 3장면 10 Hz shadow 계측 완료**
>
> 승인 상태: **연구용 관찰 전용. 차량 제어 연결 금지**

이 문서는 10-epoch 30 km/h Portable E2E physical-v1 checkpoint를 Autoware 옆에서 **관찰만** 하는 방법을
설명한다. 현재 node는 학습 모델의 출력을 `/planning/portable_e2e/` 아래에만 발행한다.
정식 제어 입력 `/planning/trajectory`를 발행하지 않으며 throttle, brake, steering도 발행하지
않는다. 각 publisher가 정확한 고정 topic으로 resolve되는지도 시작 때 검사해 remap을
거부한다. shadow 결과를 relay 또는 mux로 제어 계통에 연결하면 안 된다.

기능 전체의 단계와 합격 기준은
[9개 상위·30개 하위 기능 로드맵](portable-e2e-feature-roadmap.md), 학습·전송 절차는
[Portable E2E 학습·운용 가이드](portable-e2e-training.md)를 본다. checkpoint의
학습·open-loop 수치는
[2026-09-05 Common10 보고서](validation-2026-09-05-portable-e2e-common10-30kph.md),
최신 Autoware 동시 live 10 Hz shadow 결과는
[2026-09-06 3장면 보고서](validation-2026-09-06.md)에 분리돼 있다.

## 1. 지금 무엇이 있고 무엇이 없는가

현재 저장소에는 다음 요소가 있다.

| 요소 | 구현 상태 | 실제 실행 증거 |
|---|---|---|
| 6-camera/ego/route tensor 변환 | 구현 | CARLA 3장면 연속 입력 검증 완료 |
| contract·runtime bundle·source checkpoint·corpus·model config·rig·route SHA-256 고정 | 구현 | 원격 export와 로컬 strict-load 통과 |
| live 6-camera CameraInfo와 pinned rig의 causal/non-stale 일치 검사 | 구현 | 단위검사와 ROS startup 단계 |
| live `base_link`→6 optical-frame TF와 pinned `T_base_from_camera` 일치 검사 | 구현 | CARLA 3장면 모든 accepted anchor 검증 완료 |
| 10-frame causal ego history와 stale/skew 검사 | 구현 | 단위검사 단계 |
| camera 우선 도착 시 50 ms bounded causal-state settle | 구현 | CARLA 3장면 settle timeout 0 |
| 최고 logit 후보 선택과 geometry/speed/acceleration/curvature/lateral-acceleration gate v8 | 구현 | offline 검사와 CARLA 3장면 live 검증 |
| `val`/`test` 전체 후보 geometry 감사 CLI | 구현 | Town03 337개에 실행 |
| 격리된 Autoware `Trajectory`, `Path`, status 발행 | 구현 | CARLA 3장면 output correlation과 topic 격리 검증 완료 |
| 10 Hz용 CARLA 640×360 six-camera mapping | 구현 | 모든 인접 source period 100 ms, 위반 0 |
| canonical planner 대체, fallback/MRM, 제어 승인 | 없음 | 실행 금지 |

`10 Hz runtime`이라는 이름은 입력 bundle 목표 주기가 100 ms이고 inference deadline이
100 ms라는 뜻이다. steady-clock timer가 10 ms마다 준비된 bundle을 확인하고 1초마다 상태를
발행하므로 CARLA `/clock`이 멈춰도 camera-bundle timeout 감시는 계속된다. 최신 세 장면은
모든 인접 source anchor, accepted output correlation, wall gap과 drop/reject/timeout 분모를
검사해 10 Hz `ESTABLISHED_FOR_SHADOW_ONLY`를 통과했다. 이 판정은 Portable 모델의
canonical control이나 실차 10 Hz 승인이 아니다.

## 2. 가장 중요한 현재 결과

### physical-v1 결과

2026-09-06에 Pro6000 개인 venv와 허용된 physical GPU 0 하나만 사용해
`perspective_trajectory.physical.v1`을 10 epoch, 정확히 1,540 step 학습했다. GPU 1, 시스템
환경, 타 사용자 process는 건드리지 않았다. 원격 종료 뒤 검증된 비실행 NPZ bundle만 로컬로
복사해 manifest와 strict loader를 다시 확인했다.

| 항목 | 결과 |
|---|---:|
| 학습 sample / 독립 val sample | 613 / 337 |
| val selected ADE / FDE | 4.2615 m / 10.9172 m |
| val speed MAE | 1.3933 m/s |
| historical offline gate v6 selected PASS | 326 / 337 |
| 하나 이상 candidate PASS | 326 / 337 |
| 모든 candidate PASS | 325 / 337 |
| selected candidate | c2, 337 / 337 |
| 실제 Common10 입력 로컬 CPU bundle smoke | total 42.404 ms, forward 20.258 ms, PASS |

위 표의 gate v6 수치는 학습 직후 보존한 historical offline audit이며 현재 live runtime의
gate ID를 뜻하지 않는다. 현재 runtime은 `portable_e2e.runtime_geometry_gate.v8`이고,
reverse-speed jitter와 전체 증거 계약이 추가됐다. historical JSON의 이름만 v8로 바꾸지
않는다.

geometry gate 통과율은 v0의 `0/337`에서 크게 개선됐다. 거부된 11개는 기록된 현재 속도가
30 km/h 입력 상한을 이미 넘은 sample이다. 그러나 candidate가 c2 하나로 붕괴했고 selected
ADE/FDE도 크므로 이는 **실제 제어 승인 결과가 아니다**. 이번 세 장면에서는 기존 Autoware
VAD가 계속 제어를 소유하고 physical-v1은 shadow 출력과 지연만 기록한다.

### physical-v1 CARLA live shadow 결과

2026-09-06에는 같은 runtime·rig·contract로 Town07 직진, C-track 좌회전,
Town03 좌회전을 각각 owned CARLA cold-start로 실행했다. 차량은 Autoware VAD hybrid가
제어했고 Portable physical-v1은 격리 shadow topic만 발행했다.

| 장면 | route | accepted / source 구간 | source rate | inference p99 / 최대 | source period 위반 |
|---|---:|---:|---:|---:|---:|
| Town07 직진 | PASS | 547 / 54.6 s | 9.999999851 Hz | 40.408 / 49.778 ms | 0 |
| C-track 좌회전 | PASS | 508 / 50.7 s | 9.999999851 Hz | 38.482 / 47.436 ms | 0 |
| Town03 좌회전 | PASS | 497 / 49.6 s | 9.999999851 Hz | 36.862 / 46.283 ms | 0 |

세 장면 모두 분석 schema v2 `EVIDENCE_VALID`, 10 Hz
`ESTABLISHED_FOR_SHADOW_ONLY`, 요구 조건 `17/17`, inference `>100 ms` 0이었다.
input reject, stale drop, bundle eviction/expiry, settle timeout과 camera timeout도 모두
0이다. 모든 인접 source period는 `100,000,001~100,000,002 ns`였으며 목표
`100,000,000 ns ±5,000 ns`를 벗어난 구간이 없다. 전체 Autoware 화면·GIF·경로 분석은
[2026-09-06 검증 보고서](validation-2026-09-06.md)와
[정리된 발행 폴더](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/)를
본다.

### historical v0 결과

이전에 Pro6000 개인 작업공간에서 학습한 10-epoch v0 checkpoint를 비실행 `.runtime.npz` bundle로
export하고 원격과 로컬 CPU에서 모든 SHA-256 pin 및 strict state load를 검증했다. 이어 같은
Town03 우회전 `val` 337개를 로컬에서 변경하지 않고 read-only replay해 당시 runtime geometry
gate v6를 전체 재감사했다. 이 절은 current gate v8 live 판정이 아닌 historical 비교다.

| 항목 | 결과 |
|---|---:|
| candidate `c0` geometry PASS | 0 / 337 |
| candidate `c1` geometry PASS | 0 / 337 |
| candidate `c2` geometry PASS | 0 / 337 |
| candidate `c3` geometry PASS | 0 / 337 |
| candidate `c4` geometry PASS | 0 / 337 |
| candidate `c5` geometry PASS | 0 / 337 |
| 한 개 이상의 gate-pass candidate가 있던 sample | 0 / 337 |
| 최고 logit으로 선택된 candidate index `1` | 337 / 337 |
| selected geometric-speed / speed-disagreement / step 실패 | 326 / 337, 각각 |
| selected reported·geometric speed-rate / distance-disagreement 실패 | 326 / 337, 각각 |
| selected curvature / lateral-acceleration 실패 | 326 / 337, 각각 |
| selected heading / backward-step / speed 실패 | 310 / 337, 94 / 337, 47 / 337 |

gate v6는 `1e-4 m/s` 이하만 명시적인 zero-speed claim으로 판단한다. 별도의 `0.1 m/s`
저속 tolerance는 heading deadband와 짧은 정지 궤적에만 사용한다. 따라서 속도와 변위가
일치하는 정상 crawl을 stationary drift로 오판하지 않으면서, 속도 0을 주장하고 누적 이동하는
궤적은 계속 거부한다. 보고 속도와 XY 역산 속도의 차이는 각 점에서 `0.25 m/s` 이하여야
하고, XY 역산 속도에도 30 km/h와 종·횡가속 한계를 똑같이 적용한다. 각 step의
`|distance - speed × 0.1 s|`도 전체 horizon에 걸쳐 절댓값으로 누적하며, `0.02 m`와 전체
이동거리의 `5%` 중 큰 허용치 밖이면 거부한다. 반대 부호 오차 상쇄, 2 mm보다 큰 역방향
step, 진행 없는 저속 왕복 우회도 허용하지 않는다.

여기서 `safe candidate`는 현재 numeric/speed/geometry runtime gate를 통과했다는 제한된
용어다. drivable area, 장애물, 신호 또는 충돌 안전성을 확인했다는 뜻이 아니다.

즉, 모든 sample에서 여섯 후보 각각이 현재 speed-displacement, step, acceleration/deceleration
등의 gate 조합을 만족하지 못했다. 11개 sample은 기록된 현재 속도가 엄격한 30 km/h 입력
상한을 넘어서 candidate-independent speed reject가 먼저 발생했고, 나머지 326개는 후보별
실패 코드를 끝까지 분해했다.
선택도 전 sample에서 index 1로 고정되어 candidate ranking 또는 diversity 붕괴를 의심해야
한다. 현재 runtime은 먼저 최고 logit 후보를 고른 뒤 그 후보가 gate를 위반하면 결과를
거부한다. 다른 후보로 자동 교체하지 않는다. 이번 사전감사에서는 다른 후보도 전부
부적격이므로 교체했어도 safe candidate가 없다.

동일한 입력 변환과 gate가 유지된다는 조건에서는 Town03 337개가 모두
`SHADOW_REJECTED`가 될 것으로 예상한다. 이것은 **오프라인 결과에 근거한 예상**이며 실제
ROS/CARLA 실행 측정은 아니다. sanitized 결과와 전체 분모는
[Sep06 local CPU gate v6 evidence](assets/validation/2026-09-06/portable_e2e_v0_duration_ab_v1/local_cpu_runtime_smoke_gate_v6.json)에
보존했다. 같은 폴더의 `runtime_gate_audit.json`은 speed-disagreement와 speed-rate gate가
추가되기 전 historical 자료이므로 최신 승인 판정에 사용하지 않는다.

로컬 CPU의 30개 sampled public-runtime 호출은 모두 gate에서 먼저 거부됐다. 최신 관찰 wall
latency는 p50 `46.733 ms`, p95 `91.469 ms`, p99 `99.808 ms`였고 `1/30`이 100 ms를
넘었다. geometry 거부가 deadline 검사보다 먼저 발생하므로 이것은 formal deadline PASS/FAIL도,
ROS sensor-to-plan 측정도 아니다. 동시에 실행 중인 작업의 영향을 받는 진단 수치이며 화면
끊김 원인을 카메라 Hz 하나로 단정할 수 없음을 보여준다.

감사를 다시 실행할 때는 Pro6000의 개인 venv에서 기존 dataset/checkpoint를 읽고 존재하지
않는 새 output만 지정한다. 아래는 GPU를 전혀 노출하지 않는 재현 예시다.

```bash
: "${PORTABLE_E2E_ROOT:?set the personal project root first}"
: "${REPO_ROOT:?set the checked-out repository root first}"
source "$PORTABLE_E2E_ROOT/venvs/py312/bin/activate"
export PIP_REQUIRE_VIRTUALENV=true
export PYTHONNOUSERSITE=1
cd "$REPO_ROOT"

dataset_root="$PORTABLE_E2E_ROOT/datasets/prepared/carla-common10-30kph-town07-ctrack-town03-20260905-v2"
checkpoint_file='<regular-checkpoint-file>'
audit_json='<new-runtime-audit-json>'
checkpoint_sha256='370f12dbfa15cc17fa29931bc3c9dd3140dbd7c0a61d976af296fd223b2becf0'

test -d "$dataset_root"
test -f "$checkpoint_file" && test ! -L "$checkpoint_file"
test ! -e "$audit_json" && test ! -L "$audit_json"

CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
python -m portable_e2e.audit_runtime "$dataset_root" \
  --checkpoint "$checkpoint_file" \
  --checkpoint-sha256 "$checkpoint_sha256" \
  --output-json "$audit_json" \
  --split val \
  --device cpu \
  --batch-size 4
```

실제 GPU0 재실행이 필요하면 [학습·운용 가이드](portable-e2e-training.md)의 단일 장치
allowlist 절차를 그대로 적용한다. GPU를 자동 선택하거나 GPU 1로 바꾸지 않는다.

## 3. 두 환경의 역할을 섞지 않기

| 환경 | 담당 작업 | 금지·비담당 |
|---|---|---|
| 로컬 CARLA·Autoware PC | branch build, map/route 준비, 6-camera와 차량 상태 발행, shadow node 실행, ROS bag·RViz·지연·reject 수집 | open-loop 숫자만으로 제어 승인, shadow topic을 canonical control에 연결 |
| SSH Pro6000 | 개인 venv에서 immutable dataset 검증, 학습, val/test, model A/B, read-only runtime 사전감사 | CARLA/Autoware build·실행, 실차 제어, 시스템/Conda/타 사용자 환경 변경 |

Pro6000에서는 개인 `venvs/py312`만 사용한다. GPU 작업은 실행 직전에 사용 가능 여부를
확인한 physical GPU 0 하나만 단일 allowlist로 노출하고, process 내부에서는 logical
`cuda:0`으로 사용한다. GPU 1, 다른 사용자의 process, driver와 kernel을 건드리지 않는다.

checkpoint는 Pro6000에서 학습하되, 로컬 live runtime에는 `.pt`를 직접 넣지 않는다. 개인
작업공간에서 비실행 `.runtime.npz` bundle로 export하고 source checkpoint hash를 함께 묶은 뒤
그 bundle만 **복사**해 SHA-256 동일성을 다시 확인한다. 원격 venv를 로컬로 복사하지 않는다.
코드 변경은 로컬 Git에서 검토·commit·push하고 Pro6000은 `pull --ff-only`로 받는다.

## 4. runtime의 입력과 출력

### 입력

| 종류 | topic/파일 | 필수 조건 |
|---|---|---|
| Camera 6종 | `/sensing/camera/<CAMERA>/image_raw` | 아래 순서, 640×360, 동일 timestamp bundle |
| CameraInfo 6종 | `/sensing/camera/<CAMERA>/camera_info` | Reliable, 미래 값 금지, 250 ms 이내, frame·640×360·K/D/R/P·binning/ROI가 rig와 일치 |
| Odometry | `/localization/kinematic_state` | 미래 state 금지, 100 ms 이내, `map`→`base_link` frame |
| Acceleration | `/localization/acceleration` | 미래 state 금지, 100 ms 이내, `base_link` frame |
| Steering | `/vehicle/status/steering_status` | 미래 state 금지, image anchor 기준 100 ms 이내 |
| Route | launch의 `route_file` | 명시적 `coordinate_reference=base_link`, 현재 scene과 같은 route, regular file과 exact hash |
| Contract | launch의 `contract_file` | Common10 JSON regular file과 exact hash; symlink 거부 |
| Calibration | launch의 `rig_file` | Common10 validator를 통과한 regular JSON과 exact hash |
| Model | launch의 `runtime_bundle_file` | 비실행 `.runtime.npz`, bundle/source checkpoint/corpus/config hash 모두 일치 |

카메라 ABI 순서는 `CAM_FRONT`, `CAM_BACK`, `CAM_FRONT_LEFT`, `CAM_BACK_LEFT`,
`CAM_FRONT_RIGHT`, `CAM_BACK_RIGHT`다. ROS callback 순서가 아니라 이 이름 순서로 model
tensor를 만든다.

live `/camera_info`는 image anchor보다 미래가 아닌 최신 값을 카메라별 bounded buffer에서 고르고,
250 ms를 넘으면 거부한다. 선택된 6개 모두 optical frame, 해상도, K, zero-D rectified 상태,
identity R, K와 일치하는 P, 무 binning/full ROI를 pinned rig와 대조한다. 이어 매 image anchor에서
live TF tree의 `base_link`→각 optical frame을 독립 조회해 pinned `T_base_from_camera`와 비교한다.
translation 또는 rotation 오차가 각각 0.005 m, 0.005 rad를 넘거나 TF가 없으면 결과를 거부한다.
CameraInfo와 TF가 모두 맞아도 이것만으로 차량 제어 승인이 되지는 않는다.

### 출력

| topic | 의미 |
|---|---|
| `/planning/portable_e2e/shadow_trajectory` | gate를 통과했을 때만 나오는 격리 trajectory |
| `/planning/portable_e2e/shadow_path` | 위 trajectory의 RViz 확인용 path |
| `/planning/portable_e2e/status` | `SHADOW_WAITING`, `SHADOW_OK` 또는 `SHADOW_REJECTED`, 누적 수와 사유 |
| `/planning/portable_e2e/latency_ms` | 통과한 inference의 tensorization 포함 total latency |
| `/planning/portable_e2e/selected_candidate` | 통과한 최고-logit candidate index |

status JSON의 `vehicle_control_approved`는 항상 `false`다. `SHADOW_OK`는 입력·추론 gate만
통과했다는 뜻이며 `inference_inputs_healthy_now=true`로 기록된다. 같은 anchor에서 6개 camera
TF까지 모두 맞아야 `tf_extrinsic_parity=VERIFIED`, `calibration_extrinsics_verified=true`,
`healthy_now=true`가 된다. node는 최근 상태를 wall/steady
clock 기준 1초마다 heartbeat로 다시 발행한다. node는 recorder가 다섯 shadow output에 연결되기
전에는 disarmed 상태로 입력을 버리며, 명시적인 `arm_measurement` 뒤의 pristine zero status부터
계측한다. 종료 때는 `seal_measurement`가 추가 추론을 먼저 막은 뒤 단 하나의 terminal status를
남긴다. 이 arm/seal은 계측 경계일 뿐 차량 제어 기능이 아니다. 완성된
6-camera bundle이 300 ms 동안 오지 않으면 이전 `SHADOW_OK`를 `camera_bundle_timeout`의
`SHADOW_WAITING`으로 철회한다. exact camera bundle이 상태 callback보다 먼저 오면 최대 50 ms만
보존해 causal state 도착을 기다린다. 이 동작은 `input_settle_deferred_count`와
`input_settle_timeout_count`로 구분된다. `anchor_attempt_count = accepted_count +
anchor_rejected_count`가 항상 성립하고 input callback 거부는 `input_event_rejected_count`로
따로 센다. status heartbeat가 끊기거나 bundle이 계속 `WAITING`이면 외부 watchdog에서도
실패로 취급해야 한다.

현재 startup 범위에는 두 가지 운용 제한이 남아 있다. node는 `declared_map_id`와 pinned route
hash를 status provenance에 싣지만 CARLA RPC를 직접 호출하지 않는다. 따라서 각 scene은 CARLA와
shadow node를 함께 cold-start하고, 별도 read-only CARLA probe의 `observed_map_id`가 declared map과
일치하며 aligned route hash도 같은지 evidence analyzer에서 확인해야 한다. CUDA status는 물리 GPU
UUID를 공개·기록하지 않으므로 GPU timing evidence에는 실행 직전 단일-device allowlist 검증과
비식별 장치 provenance를 별도로 남긴다.

## 5. 최초 실행 전 점검

정식 evidence는 [Quick Start 12.6](BEGINNER_QUICKSTART_KO.md#126-2026-09-06-exact-10-hz-shadow-3장면)의
`run_owned_carla_route_trial.sh`로 만든다. 이 wrapper가 fresh CARLA ownership,
map/route binding, runtime health, arm/seal, recorder 연결, 분석과 cleanup을 한 계약으로
검증한다. 아래 5~9절의 여러 Terminal 방식은 node 하나를 진단할 때만 사용하며 공식
v3 재현을 대신하지 않는다.

아래 절차는 **로컬 CARLA·Autoware PC**에서만 한다. 새 clone이면 먼저
[한국어 Quick Start](BEGINNER_QUICKSTART_KO.md)에 따라 CARLA, map, Autoware dependency와
workspace를 준비한다. 이 절차는 package를 임의 설치하지 않는다. import가 실패하면 멈추고
승인된 로컬 dependency 준비 절차를 먼저 수행한다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
source scripts/e2e/env.sh

python3 - <<'PY'
import numpy
import PIL
import rclpy
import torch
print("runtime Python imports: PASS")
PY

test -f autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml
test -f autoware_e2e_vad_launch/launch/portable_e2e_shadow.launch.xml
```

기존 workspace를 갱신했다면 package를 다시 빌드하고 새 shell에서 환경을 다시 읽는다. 처음부터
전체 stack을 만드는 경우에는 Quick Start의 full build를 사용한다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
export AUTOWARE_E2E_SKIP_INSTALL=1
source scripts/e2e/env.sh

colcon build \
  --base-paths autoware_e2e_vad_launch \
  --symlink-install \
  --packages-select autoware_e2e_vad_launch

unset AUTOWARE_E2E_SKIP_INSTALL
source scripts/e2e/env.sh
ros2 pkg executables autoware_e2e_vad_launch | \
  grep 'portable_e2e_shadow_node.py'
```

## 6. 모델·rig·route를 안전하게 고정하기

현재 연구 checkpoint와 private runtime bundle에 연결되는 공개 provenance 값은 다음과 같다.

| 값 | SHA-256 |
|---|---|
| private physical-v1 runtime bundle | `bdd3daf605e269a8d90ea8e56ab1691e7e49db50e3f2100c7b66469813569d4e` |
| physical-v1 source checkpoint | `df2a4b75a978f35c213894c56cf3a905f547734ee8099e8142133bdc9bd3ae09` |
| training corpus fingerprint | `17c248440efca864e6c322ca5a1602d08cd1a0eabfa71e10545049181a86e073` |
| physical-v1 canonical model config | `cbf591196084509fa96eda35a197c0bc7eb2c8a251cd812d71a109441710d65d` |
| matching CARLA rig JSON | `9c41a11824585a73c639a9d10b5be032dd578ea0e97f140418a692d41a2390da` |
| Common10 contract JSON | `6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642` |

checkpoint와 runtime bundle 자체는 Git에 포함되지 않는다. matching CARLA rig와 final
3개 source route는
[2026-09-06 발행 asset](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/)에
exact byte로 포함했다. 공개 저장소만 clone하면 소스와 입력 계약은 확인할 수 있지만 learned
shadow inference에는 matching private bundle이 별도로 필요하다. 승인된 bundle 복사본을
로컬에 준비한 뒤 아래 placeholder를 실제 **로컬** 파일로 바꾼다. symlink는 runtime이
거부한다. `.pt` checkpoint를
`runtime_bundle_file`에 넣으면 의도적으로 실패한다. aligned route 경로를 아직 모르면 7절에서 stack을 먼저 시작해
`Map alignment: route=...` 출력을 확인한 뒤 이 절로 돌아온다. 이 명령을 실행한 Terminal 3은
8절까지 닫지 않는다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
contract_file="$REPO_ROOT/portable_e2e/config/common_10hz_v1.contract.json"
runtime_bundle_file='<local-private-regular-runtime-npz-file>'
rig_file="$REPO_ROOT/docs/assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/config/carla_common10_rig.json"
shadow_route_file='<local-map-aligned-route-json>'
declared_map_id='<normalized-map-id-such-as-town07>'

contract_sha256='6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642'
runtime_bundle_sha256='bdd3daf605e269a8d90ea8e56ab1691e7e49db50e3f2100c7b66469813569d4e'
source_checkpoint_sha256='df2a4b75a978f35c213894c56cf3a905f547734ee8099e8142133bdc9bd3ae09'
corpus_fingerprint_sha256='17c248440efca864e6c322ca5a1602d08cd1a0eabfa71e10545049181a86e073'
model_config_sha256='cbf591196084509fa96eda35a197c0bc7eb2c8a251cd812d71a109441710d65d'
rig_sha256='9c41a11824585a73c639a9d10b5be032dd578ea0e97f140418a692d41a2390da'

for input_file in "$contract_file" "$runtime_bundle_file" "$rig_file" "$shadow_route_file"; do
  test -f "$input_file" && test ! -L "$input_file" || exit 2
done

actual_contract_sha256="$(sha256sum -- "$contract_file" | awk '{print $1}')"
actual_runtime_bundle_sha256="$(sha256sum -- "$runtime_bundle_file" | awk '{print $1}')"
actual_rig_sha256="$(sha256sum -- "$rig_file" | awk '{print $1}')"
route_sha256="$(sha256sum -- "$shadow_route_file" | awk '{print $1}')"
test "$actual_contract_sha256" = "$contract_sha256"
test "$actual_runtime_bundle_sha256" = "$runtime_bundle_sha256"
test "$actual_rig_sha256" = "$rig_sha256"
test "${#route_sha256}" -eq 64
[[ "$declared_map_id" =~ ^[a-z0-9_]{1,64}$ ]]

CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 \
python3 -m portable_e2e.runtime_weight_bundle verify \
  --bundle "$runtime_bundle_file" \
  --bundle-sha256 "$runtime_bundle_sha256" \
  --source-checkpoint-sha256 "$source_checkpoint_sha256" \
  --model-config-sha256 "$model_config_sha256" \
  --corpus-fingerprint-sha256 "$corpus_fingerprint_sha256"
```

`shadow_route_file`은 `run_route_vad_full.sh`가 출력하는 `Map alignment: route=...`의 route와
같아야 한다. source route와 aligned route가 다르면 source 파일을 shadow node에 넣지 않는다.
`declared_map_id`는 사용자가 임의로 붙이는 별칭이 아니라, 같은 CARLA server에
`probe_carla_server.py`를 실행해 얻은 실제 map 이름을 소문자로 정규화한 값이어야 한다.

## 7. CARLA와 Autoware를 먼저 실행하기

Terminal 1에서 해당 Town의 CARLA server를 cold-start한다. 아래 `<Town>`을 route의 town으로
바꾼다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
scripts/e2e/run_carla_map.sh '<Town>' \
  --port 2100 --quality Epic \
  -- -windowed -ResX=1280 -ResY=720 -nosound
```

`C_track_1_0_7`은 Low renderer의 known skeletal-mesh crash 때문에 project wrapper가
Low 실행을 fail-fast로 거부한다. 세 v3 장면의 exact 비교는 모두 `Epic`을 쓴다.

`CARLA_READY` 뒤 Terminal 2에서 기존 VAD/Autoware stack을 실행한다. 이것이 차량 제어를 계속
소유하고 Portable E2E는 아직 실행하지 않는다. `<source-route-json>`은 같은 Town의 검증된
route로 바꾼다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
source scripts/e2e/env.sh

scripts/e2e/run_route_vad_full.sh '<source-route-json>' \
  sensor_mapping_file:="$REPO_ROOT/autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml"
```

출력된 aligned route를 6절의 `shadow_route_file`에 사용한다. 동일 CARLA server에 다른 tick
owner를 실행하지 않는다. 다른 CARLA/VAD 작업과 GPU를 동시에 사용하면 화면 끊김과 deadline
miss가 섞이므로 timing 증거를 채택하지 않는다.

## 8. shadow node 실행하기

Terminal 3에서 6절의 변수를 다시 설정·검증하고 아래 launch를 실행한다. 최초 wiring은
`device:=cpu`로 시작한다. `research_acknowledged:=true`는 안전 승인이 아니라
“미승인 연구 checkpoint임을 이해했다”는 명시적 확인이다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
source scripts/e2e/env.sh

: "${runtime_bundle_file:?repeat section 6 in this terminal}"
: "${contract_file:?repeat section 6 in this terminal}"
: "${contract_sha256:?repeat section 6 in this terminal}"
: "${runtime_bundle_sha256:?repeat section 6 in this terminal}"
: "${source_checkpoint_sha256:?repeat section 6 in this terminal}"
: "${corpus_fingerprint_sha256:?repeat section 6 in this terminal}"
: "${model_config_sha256:?repeat section 6 in this terminal}"
: "${rig_file:?repeat section 6 in this terminal}"
: "${rig_sha256:?repeat section 6 in this terminal}"
: "${shadow_route_file:?repeat section 6 in this terminal}"
: "${route_sha256:?repeat section 6 in this terminal}"
: "${declared_map_id:?repeat section 6 in this terminal}"

ros2 launch autoware_e2e_vad_launch portable_e2e_shadow.launch.xml \
  contract_file:="$contract_file" \
  contract_sha256:="$contract_sha256" \
  runtime_bundle_file:="$runtime_bundle_file" \
  runtime_bundle_sha256:="$runtime_bundle_sha256" \
  source_checkpoint_sha256:="$source_checkpoint_sha256" \
  corpus_fingerprint_sha256:="$corpus_fingerprint_sha256" \
  model_config_sha256:="$model_config_sha256" \
  rig_file:="$rig_file" \
  rig_sha256:="$rig_sha256" \
  declared_map_id:="$declared_map_id" \
  route_file:="$shadow_route_file" \
  route_sha256:="$route_sha256" \
  input_settle_timeout_s:=0.05 \
  maximum_tf_translation_error_m:=0.005 \
  maximum_tf_rotation_error_rad:=0.005 \
  device:=cpu \
  use_sim_time:=true \
  research_acknowledged:=true
```

contract hash, bundle/source checkpoint pin, model ABI, rig, route 또는 research 확인이 틀리면 node가
시작되지 않는 것이 정상이다. 시작 후에도 CameraInfo parity, stale/skew, image shape, history, non-finite output,
speed/step/heading/extent/acceleration/deceleration 또는 100 ms deadline을 위반하면 해당 결과는 발행하지 않는다.

## 9. 관찰과 판정

Terminal 4에서 먼저 canonical topic 소유권과 shadow node/service가 존재하는지 확인한다. 이
시점의 node는 아직 disarmed이고 status QoS는 volatile이므로, 여기서 status를 기다리면 안 된다.

```bash
export REPO_ROOT="${REPO_ROOT:-$(git rev-parse --show-toplevel)}"
cd "$REPO_ROOT"
source scripts/e2e/env.sh
ros2 node info /portable_e2e_shadow
ros2 topic info /planning/trajectory --verbose
```

historical v0 checkpoint라면 Town03에서 `SHADOW_REJECTED`가 반복되고 trajectory가 나오지 않는
것이 당시 사전감사와 일치한다. current physical-v1은 실제 세 CARLA 장면에서 accepted
`547/508/497`, reject/drop/timeout 0으로 live 측정을 마쳤다. 이 수치는 해당 exact
route·rig·runtime의 shadow 결과이며 다른 map이나 실제 차량으로 일반화하지 않는다. 어떤
모델이든 threshold를 늘리거나 status를 무시해 path를 만드는 것은 합격이 아니다.

새 run directory만 만들어 status, selected candidate, latency, shadow path와 차량 상태를
기록한다. 성공 장면만 선택하지 않고 reject와 무출력 구간도 전체 분모에 포함한다.

```bash
evidence_root="${REPO_ROOT}/artifacts/validation/$(date +%Y-%m-%d)/portable_shadow"
run_id='town03_right_30kph_run_001'
run_dir="$evidence_root/$run_id"
test ! -e "$run_dir"
mkdir -p "$run_dir"

ros2 bag record --output "$run_dir/rosbag" \
  /planning/portable_e2e/shadow_trajectory \
  /planning/portable_e2e/shadow_path \
  /planning/portable_e2e/status \
  /planning/portable_e2e/latency_ms \
  /planning/portable_e2e/selected_candidate \
  /sensing/camera/CAM_FRONT/camera_info \
  /sensing/camera/CAM_BACK/camera_info \
  /sensing/camera/CAM_FRONT_LEFT/camera_info \
  /sensing/camera/CAM_BACK_LEFT/camera_info \
  /sensing/camera/CAM_FRONT_RIGHT/camera_info \
  /sensing/camera/CAM_BACK_RIGHT/camera_info \
  /localization/kinematic_state \
  /localization/acceleration \
  /vehicle/status/steering_status
```

node는 처음에 disarmed이므로, 위 recorder의 다섯 shadow topic subscription을 확인한 다음 별도
terminal에서 아래 arm을 한 번 호출해야 입력을 처리한다. 공식 trial wrapper는 이 endpoint
확인과 exact arm status 보존을 자동으로 수행한다.

```bash
ros2 service call /portable_e2e_shadow/arm_measurement std_srvs/srv/Trigger '{}'
timeout 5 ros2 topic echo /planning/portable_e2e/status \
  std_msgs/msg/String --once --no-daemon --qos-reliability reliable
```

arm 호출의 `success=True` 응답 안 JSON이 정확한 zero baseline이다. 그 뒤의 bounded `topic echo`가
5초 안에 끝나지 않으면 정상 status heartbeat를 받지 못한 것이므로 계속 기다리거나 합격으로
간주하지 말고 해당 trial을 실패 처리한다.

종료할 때는 recorder가 살아 있는 동안 먼저 seal을 호출하고, shadow node를 종료한 뒤 마지막으로
recorder를 `Ctrl+C`로 정상 종료한다. seal response의 JSON과 bag의 terminal status가 정확히
같아야 한다.

```bash
ros2 service call /portable_e2e_shadow/seal_measurement std_srvs/srv/Trigger '{}'
```

입력 camera cadence와 exact bundle coverage는 별도 계측 보고서에 남긴다. 원격 full-val
model-forward `2.415211581 ms/sample`이나 단일 bundle smoke의 forward `20.257685 ms`를 로컬
ROS 전체 runtime latency로 복사하지 않는다.

## 10. 다음 합격 순서

1. acceleration-bounded `perspective_trajectory.physical.v1`의 10-epoch 학습, val337,
   안전 bundle export와 current runtime gate v8 배선은 완료했다. v0 weight를 v1 결과처럼
   재사용하지 않았다.
2. 로컬 CARLA/Autoware shadow를 Town07 직진, C-track 좌회전, Town03 좌회전 순으로 각각
   cold-start했고 3/3 route PASS, shadow `EVIDENCE_VALID`, source-native 10 Hz를 기록했다.
3. C-track correction이 첫 반복 `15.966 m` FAIL, 선택 반복 `12.622 m` PASS였으므로
   다회 cold-start 반복과 geometry/controller A/B를 먼저 수행한다.
4. candidate c2 고정과 큰 selected ADE/FDE를 별도 A/B로 개선하고 독립 `test` split을 동결한다.
   Town03 `val`을 test로 재사용하지 않는다.
5. drivable/collision selector와 deterministic fallback/MRM을 별도로 구현·검증한다.
6. 그 뒤에만 30 km/h learned closed-loop를 검토한다. 60 km/h와 실차 actuator 연결은
   별도의 후속 gate다.

현재 단계에서 가능한 결론은 “부적격 model output을 제어에서 격리하고, 세 CARLA 장면에서
10 Hz shadow 경로를 연속 출력했다”까지다. “Portable 모델이 10 Hz로 차량을 제어한다”,
“Autoware canonical planner로 사용할 수 있다” 또는 “실차 투입 가능”이라는 결론은
허용되지 않는다.
