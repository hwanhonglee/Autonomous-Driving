# Portable E2E Model v0: 구현 상태와 확장 설계

## 0. 문서 상태

이 문서는 2026-09-06 현재 저장소에 구현된 최소 trajectory baseline과 앞으로 비교할
모델 설계를 함께 기록한다. 다음 네 문장을 먼저 구분해야 한다.

- **모델·trainer·evaluator와 격리된 Autoware shadow runtime 코드는 구현되어 있다.**
- **현재 연구 기준선은 v0 10-epoch checkpoint와 Town03 open-loop 결과다.**
- **실제 ROS/CARLA shadow 연속 실행과 learned closed-loop 결과는 아직 없다.**
- **현재 코드는 실차 또는 Autoware 차량 제어에 사용할 수 있는 완성 모델이 아니다.**

구현된 baseline은 특정 VAD repository에 의존하지 않는 PyTorch planning core다. 여섯
카메라, calibration, ego history와 route를 받아 6개의 미래 trajectory 후보를 낸다.
throttle, brake, steering을 직접 출력하지 않는다.

현재 기본 model의 parameter 수는 **1,053,278개**다. 이 값은
[`perspective_trajectory_v0.model.json`](../portable_e2e/config/perspective_trajectory_v0.model.json)의
현재 기본 설정과 [`parameter_count`](../portable_e2e/model.py) 결과다. config나 layer를
바꾸면 다시 측정해야 한다.

Pro6000의 개인 프로젝트 venv에는 PyTorch `2.13.0+cu130`, NumPy `2.5.2`, Pillow `12.3.0`을
고정해 CPU 1-step과 physical GPU 0만 사용한 v0 10-epoch 학습을 완료했다. 이는 개인 venv의
확인 결과이며 시스템 Python, Conda base 또는 다른 GPU를 사용해도 된다는 뜻이 아니다.
checkpoint는 연구용 artifact이고 Git에는 포함하지 않는다.

현재 실행 수치와 자료 경계는
[2026-09-05 Common10 보고서](validation-2026-09-05-portable-e2e-common10-30kph.md)의
1-epoch 역사적 기준선과
[2026-09-06 duration A/B evidence](assets/validation/2026-09-06/portable_e2e_v0_duration_ab_v1/README.md),
shadow 실행 절차는 [10 Hz shadow runtime 가이드](portable-e2e-shadow-runtime.md), 전체
기능의 단계는 [9개 상위·30개 하위 기능 로드맵](portable-e2e-feature-roadmap.md)을 기준으로
한다.

## 1. 현재 구현된 M1 wiring baseline

구현 위치는 다음과 같다.

| 역할 | 파일 |
|---|---|
| 모델 | [`portable_e2e/model.py`](../portable_e2e/model.py) |
| tensor dataset | [`portable_e2e/torch_dataset.py`](../portable_e2e/torch_dataset.py) |
| loss | [`portable_e2e/losses.py`](../portable_e2e/losses.py) |
| trainer/checkpoint | [`portable_e2e/train.py`](../portable_e2e/train.py) |
| open-loop evaluator | [`portable_e2e/evaluate.py`](../portable_e2e/evaluate.py) |
| trajectory PNG | [`portable_e2e/visualize.py`](../portable_e2e/visualize.py) |
| A/B report 검사 | [`portable_e2e/compare.py`](../portable_e2e/compare.py) |
| shadow 입력·출력 gate | [`portable_e2e/runtime_contract.py`](../portable_e2e/runtime_contract.py) |
| non-executable bundle inference | [`portable_e2e/runtime.py`](../portable_e2e/runtime.py) |
| safe runtime bundle export/load | [`portable_e2e/runtime_weight_bundle.py`](../portable_e2e/runtime_weight_bundle.py) |
| full-split runtime 감사 | [`portable_e2e/audit_runtime.py`](../portable_e2e/audit_runtime.py) |
| Autoware shadow node | [`portable_e2e_shadow_node.py`](../autoware_e2e_vad_launch/scripts/portable_e2e_shadow_node.py) |
| 10 Hz CARLA sensor profile | [`sensor_mapping_portable_e2e_10hz.yaml`](../autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml) |

이 baseline의 목적은 첫 대규모 모델이 아니라 실제 image decode → forward → loss →
backward → checkpoint → 고정 split 평가가 정확히 연결되는지 검증하는 것이다.

### 1.1 실제 입력 ABI

| 입력 | 현재 tensor | 현재 동작 |
|---|---|---|
| Camera | `[B, 6, 3, 180, 320]` | 원본 640×360 JPEG를 RGB decode, bilinear resize, 고정 mean/std 정규화 |
| Calibration | `[B, 6, 16]` | 정규화된 `fx, fy, cx, cy`와 `T_base_from_camera` 회전·이동 성분 |
| Ego history | `[B, 10, 13]` + bool mask | 최근 최대 1초의 속도·가속도·yaw rate·steering과 6-way command one-hot |
| Route | `[B, 128, 2]` + bool mask | anchor `base_link`의 local `(x,y)` polyline을 약 1 m 간격으로 재표본화 |

카메라 순서는 다음 ABI로 고정한다.

```text
0 CAM_FRONT
1 CAM_BACK
2 CAM_FRONT_LEFT
3 CAM_BACK_LEFT
4 CAM_FRONT_RIGHT
5 CAM_BACK_RIGHT
```

현재 모델 route 입력은 `(x,y)`뿐이다. heading, curvature, speed limit, lane boundary,
object 또는 traffic-light state를 별도 tensor로 받는다는 설계 목표를 현재 구현 사실처럼
표현하면 안 된다.

mask에도 방향 규칙이 있다. ego history는 현재 frame이 반드시 유효한 contiguous suffix,
route와 future target은 contiguous prefix다. shape, dtype, device와 NaN/Inf가 맞지 않으면
학습 전에 중단한다.

### 1.2 image와 route의 공간·순서 보존

초기 구현의 전체 평균 pooling과 route 평균은 서로 다른 장면을 비슷하게 만들 위험이 있어
현재 다음처럼 바뀌었다.

```text
각 camera RGB
  → 공유 4-stage stride-2 Conv + GroupNorm + ReLU
  → 겹치지 않는 고정 평균 3×5 spatial grid
  → grid 전체를 96차원으로 projection
  → camera ID embedding + 16차원 calibration embedding
  → 여섯 view를 flatten하여 192차원 camera feature

route (최대 128×2)
  → point MLP 96차원
  → 순방향 GRU 96차원
  → 마지막 valid route point의 hidden state
```

따라서 image encoder는 최소 3×5 공간 배열을 보존하고, route는 점 순서를 바꾸면 같은 평균이
나오는 구조가 아니다. 다만 이것은 dense BEV projection이나 cross-view attention이 아니다.
3×5 grid를 96차원으로 압축하고 여섯 view를 flatten하는 작은 wiring baseline이다.
기본 180×320 입력은 네 번의 stride-2 convolution 뒤 12×20 feature map이 되고, 이를
4×4 cell의 3×5 grid로 평균낸다. grid 크기는 downsampled feature 크기를 정확히 나눠야 하며
그렇지 않은 config는 시작 전에 거부한다. 이 고정·비중첩 pooling은 엄격한 CUDA 결정론 모드의
backward를 지원하기 위한 계약이다.

ego history는 13→64 projection 뒤 64차원 GRU의 마지막 causal state를 사용한다. camera 192,
ego 64, route 96을 이어 붙인 352차원 feature를 256차원 fusion MLP에 통과시킨다.

### 1.3 실제 출력 ABI

| 출력 | 형상 | 구현 |
|---|---|---|
| `trajectory_xy_m` | `[B, 6, 64, 2]` | 0.1~6.4초 후보별 XY; bounded step을 누적해 생성 |
| `speed_mps` | `[B, 6, 64]` | softplus로 음수가 아닌 후보별 목표 속도 생성 |
| `candidate_logits` | `[B, 6]` | 후보 선택 score의 정규화 전 값 |

각 XY step은 config의 `maximum_step_m=3.0`으로 제한한 뒤 누적한다. 이 값은 물리적
안전성을 인증하지 않는다. 출력 yaw, uncertainty, occupancy, object, signal 또는 명시적
health head는 아직 없다. yaw는 XY step에서 계산해 학습 metric에 사용한다.

후보 수 6은 command enum 수와 우연히 같을 뿐, 후보 하나를 command 하나에 고정하지 않는다.
같은 lane-follow에서도 정지·추종·좌우 회피 같은 여러 mode가 필요하기 때문이다.

### 1.4 실제 loss와 metric

현재 loss는 다음 항목을 결합한다.

- valid future point의 XY smooth-L1
- target speed smooth-L1
- XY step에서 계산한 기하학적 속도와 출력 speed의 consistency
- XY step yaw와 target yaw의 circular error
- 마지막 valid point의 displacement
- best-of-K oracle candidate를 target으로 한 candidate-score cross entropy

trainer는 loss, regression/score loss, selected ADE/FDE, speed/yaw/kinematic speed MAE와
gradient norm을 `metrics.jsonl`에 기록한다. evaluator는 별도 `val` 또는 `test` split에서
1.0/3.0/6.4초 horizon metric과 차량 중심 trajectory PNG를 만든다. 유효 label이 해당
horizon까지 없는 sample은 그 horizon의 분모에 포함하지 않고 count를 함께 기록한다.
전체 평균과 별도로 `carla`/`real` sample 수, metric, metric별 분모도 기록한다. A/B
comparison은 이 domain 구성이 같지 않거나 domain metric 구조가 잘못되면 거부하고 두 domain을
별도 표로 출력한다.

이 metric이 finite라는 사실은 주행이 좋다는 뜻이 아니다. random initialization이나
one-step checkpoint도 finite metric과 PNG를 만들 수 있다.

### 1.5 재현성과 방어 경계

현재 trainer/evaluator는 다음을 코드로 강제한다.

- trainer는 `train` split만, evaluator는 `val`/`test`만 허용
- 학습 split과 평가 split fingerprint가 같으면 거부
- checkpoint의 train episode ID와 평가 episode ID가 하나라도 겹치면 거부
- corpus fingerprint, model config hash와 image SHA-256 확인
- image를 symlink 추종 없이 열고 읽는 중 inode/size/time 변경 검사
- run directory 중복·동시 writer 거부, metrics/checkpoint 원자적 기록
- checkpoint resume 시 model/train/loss config, cursor, sample count, runtime/device ABI 검사
- sampling plan hash, weighted-interleave algorithm, domain별 실제 누적 노출 수 검사
- checkpoint load는 `weights_only=True`를 지원하는 PyTorch 요구
- report 비교 시 split/sample/domain/device/runtime/hardware와 metric 분모가 다르면 거부

sampling과 report schema가 추가된 현재 trainer/checkpoint/evaluation/comparison ID는 v1이다.
여기서 model ID의 `perspective_trajectory.v0`와 checkpoint schema v1은 서로 다른 version
축이다. 구형 checkpoint schema를 같은 형식이라고 가장해 재개하지 않고 명시적으로 거부한다.
현재 v0 10-epoch 연구 checkpoint는 v1 schema로 생성됐으며 공개 보고서에 byte SHA-256을
고정했다. 자동 migration은 제공하지 않는다.

`.pt` checkpoint는 trainer, evaluator와 read-only auditor에서만 제한적으로 읽는다. live
shadow runtime은 `.pt`를 직접 역직렬화하지 않고, 승인된 source checkpoint에서 내보내고
SHA-256으로 고정한 non-executable `.runtime.npz`만 받는다.

이 검사는 데이터 오염과 실수 가능성을 낮추지만 모델의 안전성 또는 보안 인증을 대신하지
않는다. 신뢰할 수 없는 외부 checkpoint를 실행하거나 배포 대상으로 사용하지 않는다.

## 2. 현재 baseline이 할 수 없는 것

현재 M1은 다음 기능을 학습했다고 주장할 근거가 없다.

- 장애물 검출·추적·회피
- 선행차 ACC와 cut-in 대응
- 안전한 좌우 차선 변경
- 신호등·정지선·보행자 대응
- drivable area, lane, occupancy 또는 dynamic flow 이해
- unseen real rig/domain 일반화
- 30 km/h 또는 60 km/h closed-loop 안정성
- Autoware controller에 바로 전달 가능한 trajectory 승인

그 이유는 코드에 head만 부족해서가 아니다. 해당 행동을 포함한 데이터, 정확한 label,
scenario-balanced split, closed-loop 평가와 독립 safety gate가 아직 없기 때문이다.

또한 현재 image baseline은 anchor의 여섯 frame만 본다. ego state에는 1초 history가 있지만
과거 image sequence나 object memory는 없다. map의 절대 Town ID는 입력하지 않으며, 위치를
암기하는 것을 막기 위해 route는 anchor frame local polyline을 사용한다.

## 3. calibration과 sensor 배치가 중요한 이유

TF만 정확히 바꾼다고 서로 다른 camera rig가 같은 입력이 되지는 않는다. 최소한 다음이
동시에 맞아야 한다.

- 카메라 위치와 회전 `T_base_from_camera`
- intrinsic `fx, fy, cx, cy`, 실제 horizontal/vertical FOV
- distortion/rectification 방식과 해상도·crop·resize
- exposure, motion blur, rolling/global shutter와 시간 동기
- camera order와 optical-axis convention

현재 baseline은 normalized intrinsic과 extrinsic 16개 성분을 실제로 입력한다. 하지만
calibration embedding이 잘못된 영상 crop, 보이지 않는 영역 또는 큰 sim-to-real appearance
gap을 자동으로 복구하지는 못한다. CARLA 값으로 실차 rig 값을 채우지 않고, 실차별 K/D/TF와
측정 증거를 dataset manifest에 고정해야 한다.

## 4. 다음 A/B 모델

현재 구현은 아래 Candidate B 아이디어의 **최소 wiring version**이다. 완전한 sparse-token
attention 모델이 아니다. Candidate A는 아직 구현하지 않았다.

### Candidate A — Geometry-BEV Planner (미구현)

1. 공유 multi-scale image encoder
2. K와 `T_base_from_camera`를 사용하는 calibration-aware projection
3. 작은 local BEV latent
4. ego/route query와 mode decoder
5. trajectory와 geometry/dynamic auxiliary head 공유

장점은 multi-rig metric geometry와 occupancy/collision 분석 연결이 명시적이라는 점이다.
위험은 projection 오류, BEV 메모리와 TensorRT/10 Hz latency다.

### Candidate B — Perspective-Token Planner (부분 구현)

다음 버전에서는 3×5 spatial grid를 단일 view embedding으로 바로 압축하지 않고 camera ID,
calibration과 2D 위치가 있는 sparse token으로 유지한다. ego/route query가 이 token을
cross-attention하고 mode query가 후보 trajectory를 decode한다.

장점은 dense BEV보다 초기 연산량을 제한하기 쉽다는 점이다. 위험은 view 사이 metric
geometry와 overlap이 적은 급회전 영역을 암묵적으로 배워야 한다는 점이다.

비교 순서는 다음과 같다.

1. 현재 작은 baseline으로 32~128 sample overfit과 data/checkpoint/eval 경로 검증
2. full Perspective-Token 구현
3. Geometry-BEV 구현
4. 같은 train/val/test, augmentation, step, seed budget과 target inference PC에서 A/B

parameter 수만 맞추지 않는다. 품질, p50/p95/p99 latency, peak memory, failure mode와
closed-loop를 함께 비교한다. public pretrained weight를 쓰면 license, source hash와 변환
과정을 기록하고 random-init 결과와 분리한다.

## 5. 기능을 추가하는 단계

label과 validator가 준비된 head만 추가한다. 없는 label을 0으로 채우면 “장애물 없음” 같은
잘못된 정답을 학습하므로 valid mask로 loss 전체에서 제외한다.

### M0 — control-flow smoke: 구현됨

- synthetic metadata/trajectory
- batch, masked loss, checkpoint/resume fingerprint 검사
- 실제 image decode, PyTorch model 또는 GPU 없음

### M1 — trajectory baseline: 첫 CARLA-only open-loop 실행 완료, 품질 미달

- 6 candidate `(x,y,speed)`와 score
- image/calibration/ego/route fusion
- best-of-K, yaw/speed/kinematic consistency loss
- 실제 optimizer/checkpoint/evaluator/PNG
- Town07 직진 309개와 CTrack 좌회전 304개 학습, Town03 우회전 337개 validation
- 1 epoch 154 step은 2026-09-05 역사적 기준선이고 현재 연구 결과는 10 epoch 1,540 step
- 현재 6.4초 ADE/FDE `6.567681/16.171295 m`, speed MAE `1.872983 m/s`, yaw MAE
  `0.379922 rad`, kinematic speed MAE `1.697376 m/s`로 개선됐지만 초기 gate 미달
- model-forward `0.702218 ms/sample`은 전처리·ROS·selector를 제외한 open-loop 참고값
- runtime geometry gate v6의 Town03 전체 감사에서 여섯 후보 각각 PASS `0/337`이며,
  learned selector는 candidate 1을 `337/337` 선택
- 완료된 최신 full-split 재감사의 selected failure는 geometric-speed·speed-disagreement·
  step·reported/geometric-speed-rate·distance-disagreement·curvature·lateral-acceleration이
  각각 `326`, heading `310`, backward-step `94`, speed `47`이다.

별도 `perspective_trajectory.physical.v1` decoder도 구현하고 단위검사를 통과했다. 이 decoder는
100 ms 속도 적분, 30 km/h 상한, bounded acceleration과 route-relative heading을 구조적으로
제약하지만 아직 학습·open-loop·CARLA 실행을 하지 않았으므로 v0보다 낫다고 주장하지 않는다.

### M2 — static geometry: 미구현

- drivable-area 또는 low-resolution BEV occupancy
- lane/road edge와 route corridor
- 선택적 depth consistency

### M3 — dynamic scene: 미구현

- dynamic occupancy/flow 또는 object motion representation
- lead vehicle distance/relative speed
- ego footprint를 사용하는 collision proxy

### M4 — behavior와 risk: 미구현

- stop/keep/follow/change-left/change-right likelihood
- 검증된 label이 있을 때 signal/stop-line condition
- candidate risk/uncertainty calibration

head 출력은 설명과 candidate ranking을 돕지만 AEB, command gate 또는 MRM을 대체하지 않는다.

## 6. CARLA와 실제 데이터를 같이 학습하는 방법

두 domain을 같은 common10 schema로 변환하는 것은 가능하다. 그러나 파일을 한 폴더에 넣고
frame 단위 random split하는 방식은 사용하지 않는다.

1. CARLA 10 Hz로 M1 배선과 closed-loop scenario를 빠르게 반복한다.
2. 공개/자체 실차 자료는 license, timestamp, calibration, ego/route/future pose 적합성을
   검증한 뒤 별도 provenance로 변환한다.
3. 구현된 `domain_balanced_without_replacement` sampler로 CARLA와 real의 비율을 명시한다.
   각 domain 안에서는 epoch별로 섞고, domain 사이에는 weighted interleave를 적용해 학습이
   epoch 중간에 끝나도 목표 누적량과의 차이가 1 sample 이내가 되게 한다.
4. 구현된 domain별 validation을 따로 보고, real fine-tuning 때 CARLA replay를 남겨 망각을
   측정한다.
5. final test는 model/threshold 선택에 쓰지 않는다.

기본 `uniform_without_replacement`도 모든 sample을 한 번씩 사용한다는 의미의 uniform이며,
전역 permutation을 균등하게 뽑는 정책은 아니다. 두 domain이 있으면 원래 dataset 비율로
domain 순서를 고르게 interleave한다. balanced 정책은 그 비율을 명시적으로 교체하고 큰
domain의 초과분을 기록한 뒤 제외한다.

예를 들어 1:1 학습은 다음 옵션을 사용한다. 두 domain이 모두 planning validation을 통과한
corpus에 있어야 하며, 작은 domain을 복원 추출하지 않는다. 따라서 epoch 크기는 가능한 최대
1:1 subset으로 정해지고 큰 domain의 남는 sample 수가 sampling plan에 기록된다.

```bash
python -m portable_e2e.train <common10-dataset> \
  --run-dir <new-run-directory> \
  --split train \
  --device cpu \
  --sampling-policy domain_balanced_without_replacement \
  --domain-ratio carla=1 \
  --domain-ratio real=1
```

split 최소 group은 다음과 같다.

- CARLA: map + 실제 route geometry hash + seed + weather/traffic scene
- Real: vehicle/rig + site + collection date + continuous drive/session

같은 image, 연속 drive, episode 또는 route group이 여러 split에 나오면 누수다.

### 6.1 지금 확보된 자료의 역할

현재 학습에 실제로 사용한 CARLA corpus는 Town07 직진 `train` 309개, CTrack 좌회전
`train` 304개, Town03 우회전 `val` 337개로 총 950개다. 세 episode는 native 20 Hz
physics와 10 Hz six-camera로 새로 수집해 `common_10hz_v1` planning validation을
통과했다. 하지만 train 2 episode와 val 1 episode뿐이고 독립 `test`가 없으므로 모든 Town,
일반화 또는 closed-loop 성능을 대표하지 않는다.

원격 학습 서버에는 legacy CARLA raw/export 11쌍이 아래 경로에 정확히 전송되어 있다.

```text
$PORTABLE_E2E_ROOT/
  datasets/legacy/carla_2026-08-31/source_tree
```

regular file 7,805개, payload 646,642,112 bytes이며 checksum dry-run 차이가 없었다. 그러나
4/5 Hz이므로 `common_10hz_v1` 학습 자격은 없다. adapter/schema와 작은 비성능 smoke에만
사용한다. frame 복제로 10 Hz를 만들지 않는다.

Bench2Drive legacy Mini archive 10개도 다음 경로에서 hash 검증을 마쳤다.

```text
$PORTABLE_E2E_ROOT/
  tmp/downloads/bench2drive/legacy-mini-10-research-only/archives
```

추출 없는 구조 검사도 10/10 PASS하여 총 2,295 frames와 주변 camera 13,770장, 1600×900
JPEG/calibration 일치를 확인했다. 그러나 검사한 annotation 최상위의 인식 가능한 native
timestamp가 10/10 없고, 8개 archive의
553 annotation frames에서 `bounding_boxes/*/brake` NaN 788건을 확인했다. explicit timestamp와
non-finite 처리 정책이 없고 변환 후 canonical validation도 실행되지 않았으므로 converter
readiness는 0/10 `BLOCKED_FAIL_CLOSED`다.

첫 real schema/adapter smoke 후보는 nuScenes mini이며, 공식 원본 4,167,696,325 bytes의
다운로드와 streaming archive audit까지 마쳤다. CAN expansion도 exact byte, local SHA-256과
전체 ZIP payload CRC 검사를 통과했다. 추출 없는 adapter audit 결과 10 scenes/404 samples,
6-camera 14,008 frames, CAN scene/route 대응 10/10은 구조 PASS였다. 하지만 404/404 camera
bundle이 20 ms skew gate를 넘고 최대 43.251 ms, selection-only 10 Hz pooled p99 gap
150.012 ms와 stream별 p99 최댓값 250.0 ms,
최소 scene 19.149566초여서 planning은 `NOT_QUALIFIED`다. 두 원본 모두 미해제·미변환·미학습
상태다.

장시간 10 Hz 실세계 route source 후보인 nuPlan v1.1은 mini DB, map v1.0과 camera group
0의 제한 subset만 raw 격리 경로에 반입했다. DB, map과 camera archive는 모두 exact byte 및
전체 ZIP payload CRC를 통과했고, archive-level 검사에서 camera JPEG 242,320장·7개 log·8개
channel과 mini DB 대응도 통과했다. 세 archive 모두 미해제·미변환·미학습이며, raw archive
확보는 checkpoint 또는 성능 결과가 생겼다는 뜻이 아니다. nuPlan dataset terms는 devkit
code의 Apache-2.0 license와 별개다. 정확한 byte·local SHA-256·CRC·license 상태는
[데이터셋 조사 문서](portable-e2e-datasets.md)에 있다.

nuScenes mini를 복제·합성 없이 selection-only 10 Hz로 thinning한 실측은 pooled p99
150.012 ms, stream별 p99 최댓값 250.0 ms라 현재 `common_10hz_v1`의 p99 150 ms planning
cadence gate를 통과하지 못한다. 따라서 schema/adapter smoke 전용이며, 이 실측값을 바탕으로
별도 profile/contract를 검토·고정하기
전에는 정식 common10 학습 corpus에서 제외한다. 4/5 Hz image도 복제하거나 interpolation해서
정식 10 Hz 관측이라고 표시하지 않는다.

## 7. 30 km/h와 60 km/h의 차이

6.4초 동안 이동 거리는 대략 다음과 같다.

- 30 km/h: 53.3 m, 0.1초당 약 0.83 m
- 60 km/h: 106.7 m, 0.1초당 약 1.67 m

같은 64-point output ABI는 유지할 수 있지만 60 km/h에서는 최소 약 120 m route coverage,
더 먼 장애물 가시성, 제동거리, camera-to-plan latency와 차량별 동역학 검증이 필요하다.
30 km/h 결과를 60 km/h 안전성으로 외삽하지 않는다.

속도 dataset도 정지, 0~10, 10~20, 20~30, 30~45, 45~60 km/h와 직진/좌회전/우회전/
차선변경을 episode와 주행거리 기준으로 집계한다. 저속 trajectory의 시간축만 줄여 60 km/h
label처럼 만드는 것은 금지한다.

## 8. 평가 gate

### 8.1 Open-loop

- selected/oracle ADE와 FDE: 1.0, 3.0, 6.4초
- speed/yaw/acceleration/jerk와 kinematic consistency
- candidate recall과 선택 score calibration
- off-road, route-corridor 이탈과 collision proxy (head/label 추가 후)
- command, speed bin, straight/turn/stop/lane-change, map/site, weather와 domain별 분해
- 전처리, model, 후처리와 camera-to-plan latency p50/p95/p99

현재 evaluator는 이 중 기본 trajectory/speed/yaw/kinematic metric과 forward time, PNG까지만
구현했다. 나머지를 이미 제공한다고 해석하면 안 된다.

현재 v0 10-epoch Town03 `val` 337개의 6.4초 ADE/FDE는
`6.567681/16.171295 m`이고 speed/yaw/kinematic speed MAE는 각각
`1.872983 m/s`, `0.379922 rad`, `1.697376 m/s`다. model-forward
`0.702218 ms/sample`은 decode, tensorization, ROS transport, selector와 controller를 포함하지
않는다. runtime geometry gate v6 전체 감사에서는 candidate `c0`부터 `c5`까지 각각 PASS
`0/337`이고 최고 logit 선택은 index 1에 `337/337` 고정됐다. selected failure는
geometric-speed·speed-disagreement·step·reported/geometric-speed-rate·distance-disagreement·
curvature·lateral-acceleration이 각각 `326`, heading `310`, backward-step `94`, speed `47`이다.
이는 합격이 아니라 output scale/shape와 candidate ranking을 고쳐야 한다는 실패 신호다.

### 8.2 CARLA closed-loop

Town/custom map마다 straight와 turn을 분리하고 고정 seed 및 unseen seed의 전체 분모를
보존한다.

- route completion/goal success
- collision 종류별 및 km당 비율
- lane/road departure, 반대 차선, red-light/stop-line violation
- fallback/MRM/intervention, stuck/oscillation
- TTC/headway, lateral acceleration, yaw rate, accel/jerk
- model timeout, camera bundle miss와 10 Hz deadline miss

성공한 화면만 고르지 않고 실패 run, 로그와 영상도 같은 실행 ID 아래 보존한다.

### 8.3 실차

순서는 offline replay → actuator에 연결하지 않는 shadow mode → safety driver가 있는 폐쇄
시험장 저속 주행이다. public-road 자동제어와 검증되지 않은 60 km/h 실차 주행은 v0의
승인 범위가 아니다.

## 9. Autoware adapter의 경계

학습 core는 ROS message를 import하지 않는 순수 tensor API로 유지한다.

```text
6-camera / ego / route
        ↓ input adapter + timestamp/calibration 검사
portable model core
        ↓ candidate trajectories + scores
constraint/risk checker + candidate selector
        ↓ selected trajectory 또는 reject
Autoware Trajectory adapter
        ↓
existing controller → vehicle_cmd_gate → vehicle
```

adapter와 safety layer가 맡아야 할 항목은 다음과 같다.

- input timestamp/TF/rig/camera order/stale data 검사
- output finite/horizon/frame/monotonic time 검사
- vehicle-specific acceleration/deceleration/jerk와 kinematic limit
- drivable corridor와 collision 검사
- timeout·낮은 confidence·제약 위반 시 기존 planner 또는 MRM fallback
- AEB, actuator saturation, watchdog, heartbeat와 command authorization
- input/output hash, model version, latency와 reject reason 기록

2026-09-06의 shadow adapter는 이 목록 중 제한된 배선만 구현한다. 여섯 카메라의 exact
timestamp bundle과 frame ID, 미래 state를 쓰지 않는 causal buffer, state freshness/skew,
runtime bundle·source checkpoint·corpus·model config·rig·route hash, 입력 shape,
non-finite output, 선택 후보의
speed/step/heading/extent와 100 ms deadline을 검사한다. 통과한 결과도
`/planning/portable_e2e/` 아래의 고정 격리 topic에만 발행하고 output remap을 거부한다.
1초 status heartbeat에는 항상 `vehicle_control_approved=false`를 기록한다.

live CameraInfo의 intrinsic·rectification parity는 구현됐다. live TF extrinsic parity는
**UNIMPLEMENTED_BLOCKED**이며 drivable corridor, object collision,
uncertainty, 기존 planner fallback/MRM, 독립 AEB 연동 또는 canonical trajectory authorization을
구현하지 않았다. 실제 ROS/CARLA shadow도 아직 실행하지 않았다. 따라서 shadow adapter 코드가
생겼다는 사실은 `P3_RUNTIME` 통과나 Autoware 제어 통합 완료가 아니다. 상세한 시작·관찰 절차는
[10 Hz shadow runtime 가이드](portable-e2e-shadow-runtime.md)에 있다.

모델 checkpoint가 이 계층을 대신하지 않는다. 비 Autoware 시스템은 같은 core ABI에 별도
adapter를 붙인다.

## 10. 다음 실제 실행 순서

현재까지 Common10 3-episode corpus, CPU 1-step, GPU0 v0 10 epoch와 Town03 `val` 337개
open-loop를 완료했다. source checkpoint SHA-256
`370f12dbfa15cc17fa29931bc3c9dd3140dbd7c0a61d976af296fd223b2becf0`에서 내보낸
non-executable runtime bundle SHA-256
`b9b10e1604ac59eb4375b233d80b8f7ea04d983c0b841d6afddbc39e008c292c`은 로컬 strict load를
통과했다. gate v6 전체 감사에서
candidate `c0~c5`와 selected trajectory는 각각 `0/337`이고 selector는 candidate 1에
`337/337` 고정됐다. speed-rate·curvature·lateral-acceleration을 포함한 최종 재감사는 완료했으며 실제 ROS/CARLA shadow는
아직 실행하지 않았다. 상세 명령은
[학습·운용 가이드](portable-e2e-training.md)와
[shadow runtime 가이드](portable-e2e-shadow-runtime.md)에 있다.

1. 현재 checkpoint의 step/heading/curvature 실패와 candidate index 1 고정을 재현·원인 분해
2. 기존 train/val을 자르지 않은 unseen straight/left/right/stop `test` episode 신규 수집
3. fixed split·seed·budget으로 Perspective baseline 고도화와 Geometry-BEV 후보 A/B
4. offline absolute geometry·candidate-diversity gate를 통과한 checkpoint만 shadow 후보로 승격
5. 로컬 Autoware/CARLA shadow에서 exact bundle, 전체 sensor-to-plan 지연, reject 분모 계측
6. drivable/collision selector, deterministic fallback/MRM과 독립 safety boundary 구현
7. 30 km/h CARLA learned closed-loop를 모든 지원 Town의 직진·회전·정지로 확대
8. 권리·provenance가 확인된 실제 데이터 replay/shadow 뒤 폐쇄 시험장
9. 위 기준선을 통과한 뒤 별도 60 km/h dataset·제동·곡률·지연 gate

Bench2Drive와 nuScenes는 기존 archive 감사 결과에 따라 Common10 planning 부적격을 유지한다.
nuPlan은 약관과 intended-use 승인 전 학습에 넣지 않고, legacy 4/5 Hz 자료도 10 Hz로
복제하지 않는다.

## 11. v0 완료 정의

다음이 모두 있어야 v0가 완료되었다고 부른다.

- version이 고정된 model A/B와 입출력 schema
- 재현 가능한 dependency lock, seed, config와 source commit
- provenance와 license가 확인된 train/val/test manifest
- 실제 학습 checkpoint와 중단/재개 증거
- open-loop 및 CARLA closed-loop 원본 로그·PNG/GIF·요약 보고서
- target inference PC의 10 Hz latency와 peak memory
- Autoware adapter reject/fallback와 independent safety boundary 테스트
- unseen real replay/shadow와 폐쇄 시험장 단계 결과
- 알려진 실패, ODD, 30/60 km/h 범위를 쓴 model card

이 조건 전에는 어떤 checkpoint도 “실제 차량에서 사용할 수 있는 E2E 모델”이라고 표현하지
않는다.
