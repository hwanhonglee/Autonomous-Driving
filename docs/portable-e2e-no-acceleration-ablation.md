# 가속도 입력값 제외 모델 — 연구용 A/B 학습

<!-- HH_260906 - Keep the supplied-acceleration value ablation separate from physical correction, model training and deployment approval. -->

모델 ID `portable_e2e.perspective_trajectory.physical_no_accel.v1`는 기존 physical-v1과 같은 구조에서 **입력 가속도 값의 영향만 제외**하는 비교 실험용입니다. 2026-09-09에 기존 데이터로 실제 A/B 전체 모델 학습 6회를 완료했습니다. 가속도 기준점 문제를 물리적으로 고친 모델이나 자율주행 승인 모델이 아닙니다.

<!-- HH_260906 - Link the actual exploratory learning campaign without reclassifying earlier code-only work or denied new captures. -->
[9월 9일 실제 진행 기록](assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/01_no_accel_development_ab/README.md)에
기존 physical-v1과 새 ID의 fresh 3-seed 비교 계획·완료 수치·실제 곡선·72장 예측 PNG를 기록했습니다.
상대 기준은 FAIL/PASS/FAIL이고 B의 절대 품질은 모두 FAIL이라 **채택하지 않았습니다.**
학습을 앞당기기 위해 기존 v3 데이터의 한계를 명시한 탐색적 비교부터 진행했으며,
새 검증 데이터에서의 품질 확인은 별도로 남습니다.

설정은 [perspective_trajectory_physical_no_accel_v1.model.json](../portable_e2e/config/perspective_trajectory_physical_no_accel_v1.model.json)입니다. 기존 physical-v1 설정에서 모델 ID만 다릅니다.

## 무엇이 같고 다른가

- 입력은 동일한 13개 ego feature와 같은 history 길이를 유지합니다. 구조·파라미터 이름/개수(기본 설정 954,590개)·같은 seed의 초기 가중치는 physical-v1과 같습니다.
- 새 ID의 공유 `forward` 안에서만 모든 history 시점의 `acceleration_x_mps2`, `acceleration_y_mps2`(인덱스 3, 4)를 0으로 바꾼 복사본을 사용합니다. 호출자가 준 텐서나 원본 데이터는 바꾸지 않습니다.
- 원시 입력의 shape·dtype·device·유한성 검사는 먼저 그대로 수행합니다. 제외할 값이라도 NaN/Inf를 허용하지 않으며, history padding 위치도 예외가 아닙니다.
- physical-v1 decoder, 후보 점수 head, loss, 속도/가감속·경로 안전 한계는 바꾸지 않습니다. 기존 v0·physical-v1·candidate-rank ID와 기본 설정은 그대로 동작합니다.

따라서 이 모델은 가속도 값 대신 속도 history·yaw rate 등 나머지 정보를 사용하게 됩니다. 이번 고정 데이터·예산에서는 일관된 개선을 보이지 않았으며, 다른 데이터·실차 성능까지 판정한 것은 아닙니다. 원래 데이터의 `ax/ay`나 XY/속도 정답을 수정하거나, 새 물리 가속도를 계산한 것이 아닙니다.

## 학습과 배포 경계

새 ID는 `ModelConfig`를 사용하는 연구 학습/평가 코드에서 구분됩니다. checkpoint의 전체 모델 설정과 해시가 달라지므로 기존 C/D/E checkpoint를 이 ID로 이어 학습하는 exact resume는 거부됩니다. 파라미터 모양이 같다는 이유로 ID만 바꿔 과거 학습 결과를 새 실험으로 간주하면 안 됩니다.

**현재 secure runtime weight bundle의 허용 ID 목록은 변경하지 않았습니다. 이 새 ID는 해당 bundle로 내보내거나 현재 portable ROS 런타임에 배포할 수 없습니다.** 공유 forward에서 정책을 구현한 것은 계산 방식이 일치할 수 있다는 뜻이지, 실제 런타임 지원·주행 승인이 완료됐다는 뜻이 아닙니다. 기존 고정 campaign 계획도 자동으로 새 ID를 허용하지 않습니다.

또한 가속도 숫자의 영향만 제외했습니다. 현재 ROS 가속도 topic 구독, 시간 정합성·유한성·stale 입력 검사는 제거하지 않았습니다. 가속도 메시지가 없거나 잘못돼도 구동 가능한 모델이라고 주장하지 않습니다.

## 다음 비교의 조건

원래 계획은 수집 데이터의 경로·속도·정지 품질을 통과시킨 뒤 동일한 새 corpus에서 비교하는
순서였습니다. 사용자의 학습 우선 요청으로 기존 v3의 개발용 A/B를 먼저 고정·실행했습니다.
이는 새 데이터 품질 검증을 대신하지 않으며, 기존 실패 자료·정답을 바꾸지 않습니다.
두 모델의 seed·optimizer budget·loss·gate는 같고, test로 설정을 고르거나 자동 승격하지 않습니다.
전체 corpus 계약·해시 검사는 test 파일도 읽지만 test는 학습·추론·평가지표·모델 선택에 사용하지 않습니다.

## 코드 검증

CPU의 작은 합성 텐서로 모든 history의 가속도 값 변화에 대한 출력 불변성, 해당 입력 gradient 0, 다른 입력 gradient 유지, 원본 텐서 불변성, 물리 모델에 가속도 0을 명시한 경우와의 출력 일치, 기존 세 ID의 이전 forward 대비 bit-exact 회귀, 같은 파라미터/decoder, NaN/Inf 거부와 config/resume/bundle 경계를 검사합니다.

```bash
CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  python3 -m pytest -q tests/test_portable_e2e_no_accel_model.py \
  tests/test_portable_e2e_candidate_rank_model.py
```

이는 합성 입력 단위 테스트이며 실제 학습 모델 성능, 10 Hz 실시간 구동이나 주행 촬영 결과가 아닙니다. 실행에는 기존 환경의 PyTorch가 필요하고, 이 작업에서 패키지를 설치하지 않았습니다.
