# 반복 학습·검증 운용 안내

<!-- HH_260906 - Separate actual learning, expert data capture, and learned driving evidence. -->

이 문서는 **새 Portable E2E 모델**의 반복 개발 안내다. 기존 VAD가 CARLA에서 운전한
영상, BasicAgent로 모은 정답 데이터, 새 모델의 학습 결과를 서로 구분한다.
현재 새 모델에는 실제 차량 제어 승인이 없으며, 30개 기능이 완성된 상태도 아니다.

## 두 PC에서 하는 일

| 환경 | 실행하는 일 | 실행하지 않는 일 |
|---|---|---|
| 로컬 `$REPO_ROOT` | CARLA 정답 데이터 수집, 변환·검증, 모델 결과 분석, Autoware shadow | 원격 GPU1 사용, 데이터 수집 영상을 learned 모델 주행으로 표시 |
| SSH `$TRAINING_HOST` | 개인 `py312` venv에서 GPU0 학습 → val 평가 → 경로 형상 검사 | 시스템/Conda 설치, 재부팅, 타 사용자 작업 종료, CARLA·Autoware 빌드 |

원격 작업공간은 `$PORTABLE_E2E_ROOT`, 실제 데이터는 `$PERSONAL_DATASET_ROOT` 안에 둔다.
이번 개인 폴더의 상대 배치는 `~/personal/hwanhong/{portable_e2e,dataset}`이다.
작업공간의 `datasets`는 이 개인
데이터 폴더를 가리키는 링크다. 프로젝트용 패키지는 개인 venv에만 설치하며, 이번 실험은
기존 패키지만 사용한다.

### 명시적으로 미승인된 데이터는 학습에 넣지 않는다

<!-- HH_260906 - Preserve explicit denial through converted Common10 metadata without treating legacy absence as approval. -->

Common10의 dataset·episode·source manifest 루트와 해시로 연결된 collection config를 검사한다.
collection config에 보존된 native capture/result의 정해진 경로에서
`training_data_approved: false` 또는 `development_only: true`가 발견되면 거부한다.
두 표시가 Boolean이 아닌 경우도 오류다. 선택한 split을 로드할 때 collection config의
해시와 표시를 다시 확인하므로, 최초 검증 뒤 파일을 바꿔도 통과하지 않는다.

표시가 없는 기존 자료는 호환성을 위해 계속 읽을 수 있지만 **미기재는 승인이라는 뜻이 아니다**.
이 호환성은 정상적인 JSON 객체 형식의 메타데이터에 대한 것이다. 해시만 맞는 임의의
바이너리·잘못된 JSON·중복 키·NaN 표기는 collection config로 허용하지 않는다.
원본 JSON의 표시를 지우거나 `true`로 바꿔 우회하면 안 된다. 별도 데이터 승인 절차가 필요하다.
현재 이 제한은 `planning`·`runtime`·`schema` 모두에 적용되어, 같은 Common10 로더를 쓰는
읽기 전용 평가도 미승인 자료를 거부한다. 실패 원인 분석은 원본 CARLA 진단 도구로 수행한다.
임의의 중첩 진단 객체나 `dataset_admission: false`를 재귀적으로 승인/거절로 추정하지 않는다.

## 2026-09-07에 시작한 학습

설정은 [고정 실험 계획](../config/portable_e2e_lr_ab_20260907.json)에 있다.
첫 학습은 11:10:08 KST에 시작했다. 시작 기록은 완료·합격 기록이 아니다.

- 기존 Common10 v2: Town07 직진 + C-track 좌회전 **train 613개**, Town03 우회전 **val 337개**.
- physical-v1 모델을 매번 처음부터 학습한다. 기존 checkpoint에 새 학습률을 억지로 적용하지 않는다.
- A: 학습률 `0.0001`, B: `0.00003`. 그 외 모델·데이터·배치 4·1,540 step 조건은 같다.
- seed `20260903`, `20260904`, `20260905` 각각 A/B를 짝지어 **총 6회** 실행한다.
- 각 실행 뒤 같은 val 전체를 평가하고 같은 runtime geometry **v8**로 검사한다.
- 각 프로세스는 GPU0 UUID만 보게 한다. 이미 점유되어 있으면 새 단계를 시작하지 않는다.
- SSH 연결이 끊어져도 실행 프로세스가 유지되도록 분리 실행했다. 실패 시 기록을 보존하고
  정지하며, 실패 결과를 보고 자동으로 기준을 낮추거나 모델을 제어용으로 교체하지 않는다.

각 실행은 시작 시 복사한 runner로 실행했다. 이후 저장소의 runner 안전성 개선을
이미 실행한 프로세스에 소급 적용했다고 표시하지 않는다. `status.json`의 `runner_sha256`과
`source_commit`이 해당 실행의 근거다.

### 지금 실행 상태 보기

로컬 터미널에서 본인이 설정한 SSH 별칭을 입력해 접속한다. 실제 서버 주소·계정·key 경로는
공개 문서에 기록하지 않는다.

```bash
read -r -p 'Training SSH alias: ' TRAINING_HOST
ssh "$TRAINING_HOST"
```

접속된 원격 터미널에서 개인 작업공간으로 이동한다.

```bash
cd "$HOME/personal/hwanhong/portable_e2e"
export PORTABLE_E2E_ROOT="$PWD"
```

다음은 조회 명령이다. 학습을 새로 시작하거나 다른 작업을 종료하지 않는다.

```bash
nvidia-smi
venvs/py312/bin/python -m json.tool \
  runs/campaigns/hh260907-physical-v1-lr-ab-3seeds-v1/status.json
```

진행 중인 seed/arm은 `status.json`의 마지막 `stages` 항목에서 확인한다. 예를 들어 첫 A의
최신 optimizer step은 다음과 같이 읽는다.

```bash
tail -n 1 runs/campaigns/hh260907-physical-v1-lr-ab-3seeds-v1/seed_20260903/A_baseline/training/metrics.jsonl
```

1차 캠페인은 11:31:15 KST에 6개 모델의 18개 단계를 완료했다. 3개 seed 중 2개만 개선되어
학습률 변경은 채택하지 않았다. 새 데이터 학습은 11:31:17 KST에 이어서 시작해 3개 모델의
9개 단계를 완료했다. 이후 같은 v3 데이터에서 선택 손실 가중치 0.1 → 0.5를 비교한
3개 모델의 9개 단계도 **12:02:16 KST에 완료**했다. 후속 기록은 위 경로의 campaign ID를
각각 다음 값으로 바꾸면 볼 수 있다.

- 데이터 확장: `hh260907-physical-v1-data-expansion-3seeds-v1`
- 선택 손실 변경: `hh260907-physical-v1-selector-weight-3seeds-v1`
- 후보 경로 인지 선택 구조: `hh260907-candidate-rank-3seeds-v1`

후보 경로 인지 구조는 23:23:04 KST에 시작해 23:34:14 KST에 세 모델의 9개 단계를
완료했다. 총 **4개 캠페인 / 15개 모델 / 45개 단계**이며 이 캠페인들의 GPU 학습과
후속 CPU 진단은 끝났다. 학습률·선택 손실·새 선택 구조 모두 세 seed 중 두 개만 상대 개선을 보였고, 절대 목표는 모두
미달했다. 실험 완료와 성능 합격을 구분해 기존 주행용 checkpoint는 유지한다.
자료는 [전체 실험 결과](validation-2026-09-07-portable-learning.md)에 있다.

최신 E 상태와 마지막 seed의 실제 학습 기록은 원격 작업공간에서 아래처럼 확인한다.

```bash
venvs/py312/bin/python -m json.tool \
  runs/campaigns/hh260907-candidate-rank-3seeds-v1/status.json
tail -n 1 runs/campaigns/hh260907-candidate-rank-3seeds-v1/seed_20260905/E_candidate_rank/training/metrics.jsonl
```

이 실험의 [고정 계획](../config/portable_e2e_candidate_rank_20260907.json),
[모델 설계·실행 결과](portable-e2e-candidate-ranking.md),
[최신 코드 테스트](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/13_candidate_rank_code_validation/README.md)를 함께 확인한다.
기존 campaign ID는 덮어쓰기 재실행용이 아니라 완료 기록 조회용이다.

`TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED`는 학습·평가·검사가 끝났다는 뜻이지 주행 승인이
아니다. `STOPPED_FAILURE_NO_PROMOTION`이면 마지막 단계의 `train.log`, `evaluate.log`,
`audit.log`를 확인한다. 같은 폴더를 덮어쓰거나 무조건 재시작하지 않는다.

## 무엇을 보고 개선으로 판단하나

`selected ADE`는 모델이 **실제로 선택한** 경로의 평균 위치 오차다. `FDE`는 예측 마지막
위치 오차다. `oracle ADE`는 정답을 알고 후보 중 가장 좋은 것을 골랐을 때의 오차다.
실제 운전에서는 미래 정답을 모르므로 oracle를 실제 모델 성능으로 대신 표시하면 안 된다.

기존 모델은 selected ADE `4.2615 m`, oracle ADE `1.7607 m`였다. 차이 `2.5009 m`는
후보를 고르는 쪽에도 개선 여지가 있다는 뜻이다. c2 선택 337/337은 선택 인덱스 고정의
증거이며, 여섯 후보 경로가 모두 똑같다는 증거는 아니다.

고정한 1차 선별 기준은 세 seed 모두에서 A 대비 B의 ADE/FDE가 개선되고, 선택 경로의
형상 PASS 개수가 줄지 않으며, 속도 오차가 5% 넘게 악화하지 않는 것이다. 별도로
1/3/6.4초 ADE `0.5/1/2 m`, 6.4초 FDE `4 m` 이하라는 절대 목표도 확인한다.
상대 개선만으로 절대 목표를 통과 처리하지 않는다.

후속 C/D는 같은 v3 corpus·seed·1,540 step·학습률을 유지하고 후보 선택 손실 가중치만
바꿨다. 세 seed 모두의 FDE는 개선됐으나 첫 seed ADE는 3.8743 → 4.3049 m로 나빠져
채택하지 않았다. 선택 index가 다양해지는 것과 좋은 경로를 고르는 것은 별개다.
또한 ADE oracle은 XY·speed·yaw 등을 포함하는 학습의 복합 loss oracle과 다르다.
후속 C/D 6개 모델 진단에서 이 차이를 train/val로 분리 계측했으며, val의 두 oracle
ADE 차이보다 모델 선택 오차가 훨씬 컸다. 학습 목표 정의 차이가 유일한 원인이라는
가설은 지지되지 않았다. CPU/GPU 진단 수치가 다른 경우 원본을 구분해 보존한다.

이후 구현·학습한 E는 실제 예측 후보의 XY·속도와 공통 특징으로 점수를 계산한다.
C → E의 ADE는 seed 순서대로 3.8743 → 4.7357 / 6.2585 → 4.5391 / 4.7895 →
4.2284 m였다. 상대·절대 기준에 모두 미달해 채택하지 않았다. 새 ID는 runtime bundle
허용 목록에도 추가하지 않았다. E의 10 Hz 성능이나 learned CARLA 주행 성공은 아직
측정하지 않았다. 다음은 선택기 분리 통제 실험과 더 다양한 독립 회전·정지 episode다.

`compare.py`의 `FAIR_OPEN_LOOP_COMPARISON_READY`는 보고서를 공정하게 비교할 조건이
맞다는 뜻이지 모델 품질 PASS가 아니다. geometry PASS 역시 도로 이탈·충돌 안전을
보증하지 않는다. 과거 v6 검사 결과와 현재 v8 결과를 동일 기준인 것처럼 비교하지 않는다.

## 새 데이터는 어떻게 섞나

이번 로컬 수집은 6개 실제 CARLA 카메라 프레임과 ego 상태·경로·미래 궤적을 함께
기록하는 BasicAgent expert 수집이다. 화면만 녹화해서 바로 학습하는 방식이 아니다.

- Town01 새 우회전은 v3 **train**에 추가했다.
- Town04 새 직진은 독립 **test**로 고정한다. 현재 학습률 선택에 쓰지 않는다.
- 기존 Town03 **val**은 유지한다. 한 주행의 앞뒤를 나누어 train/test로 위장하지 않는다.
- 센서 시간·캘리브레이션·실제 이미지·정답 범위가 맞아야 Common10 변환을 통과한다.
- 원격 전송은 새 staging 폴더에서 파일 수·크기·SHA-256을 비교하고, 검증 후 새 prepared
  경로로 이동한다. 기존 v2 폴더는 변경하지 않는다.

test 데이터의 형식·센서·수집 품질 검사는 가능하다. 하지만 모델 결과를 계속 보며
조건을 고르는 데 사용하면 더 이상 미사용 test가 아니다. 최종 후보를 고정한 뒤 평가한다.

## 30개 기능까지 확장하는 순서

전체 정의는 [9개 상위·30개 하위 기능표](portable-e2e-feature-roadmap.md)와
[기계 판독용 기능표](../config/portable_e2e_feature_matrix.yaml)에 있다.

1. 직진·좌/우회전·목적지 정지: 데이터 다양성, 후보 선택, 속도 오차를 먼저 개선한다.
2. 신호·정지선·교차로: 신호 상태·우선권·정지선 label과 위반 판정을 추가한다.
3. ACC·정체·끼어들기: 수집기가 소유하는 선행차와 상대 상태·시간 간격 label을 추가한다.
4. 장애물·보행자·위험 정지: 공간 점유·움직임·충돌 평가와 독립 안전 정지 계층을 추가한다.
5. 차선변경·합류: 의도·합법성·주변 공간·완료 판정을 포함한 시나리오를 추가한다.
6. 저속 정밀 주행·주차·후진, 실차 센서 replay/shadow: 필요한 입력·출력 설계와 별도 데이터,
   안전 검증을 갖춘다. 현재 전진·30 km/h 모델을 그대로 주차/후진 모델이라고 부르지 않는다.

각 단계에서 **데이터 → 학습 → 고정 검증 → 실패 원인 분석 → 수정·재학습**을 반복한다.
개선 후보만 export parity, 로컬 10 Hz shadow, 시뮬레이터 learned closed-loop 검증으로
진행한다. 현재 빈 도로 수집기는 다른 차량이 있는 world를 거부하므로, 외부에서 차량만
추가한다고 ACC 학습 데이터가 되는 것은 아니다. 의도된 차선변경도 기존 차선 침범 기준을
무조건 완화하지 않고 별도 label과 합법성 판정을 먼저 만든다.

실제 데이터 혼합은 센서·시간·정답·이용조건이 확인된 변환 자료부터 시작한다. 승인되지
않은 데이터 이용조건, 실제 차량 제어, 외부 환경 변경은 이 반복 실행에 포함하지 않는다.
