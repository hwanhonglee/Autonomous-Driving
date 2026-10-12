# S — 실제 모델 추가 학습과 후보 생성·선택 병목 진단

<!-- HH_260906 - Separate memorization gains on repeated TRAIN examples from unseen-route driving performance. -->

## 실제 수행

원격 PRO6000의 승인된 GPU0 UUID만 노출하고 기존 개인 py312 venv에서 K1 전체 모델을 추가 학습했다. 공유 GPU0 lease를 취득했으며 시작 전 GPU0 idle을 확인했다. 새 설치, GPU1 사용, ROS 빌드, CARLA 실행은 하지 않았다.

- 시작점: 완료된 K1 checkpoint. 원래 파일은 수정하지 않았다.
- TRAIN 12개: 직진/좌회전/우회전 episode에서 각4개. 전체 TRAIN으로 history를 구성한 뒤 선택하여 history를 잘라내지 않았다.
- 선택: 미래가 완전하고 이동 속도가0.5m/s 이상이며 기존 runtime target gate를 통과한 후보에서 회전량 순위의 네 지점을 결정적으로 선택했다. DEV/TEST 결과로 고르지 않았다.
- 학습: 기존 모델·손실·AdamW 설정, lr1e-4, batch4, 1,500step, 총6,000회 표본 노출. 세 group batch를 반복, 각 샘플500회 노출. optimizer는 새로 생성했다.
- 전체 모델 파라미터 업데이트. 변경하지 않은 optimizer/모델 코드의 원격 K frozen source hash 검증.
- 12개 이미지 tensor를 캐시해 학습했다. 학습+최종 저장 시간 약92.25초이며 데이터 사전 검증/전송 시간은 별도다. 전체 데이터 학습 시간으로 외삽하지 않는다.
- 연구용 checkpoint만 저장. 배포용 schema/export, 기존 모델 교체, DEV/TEST 평가, closed-loop 주행은 하지 않았다.

## 같은 TRAIN 장면에서의 결과

ADE는 미래6.4초 평균 위치 오차. 각 group 4개 평균이며 학습한 바로 그 장면에 대한 결과다.

| 선택 장면 | ADE 전→후(m) | 속도 MAE 전→후(m/s) | 일정 속도 경로 기준 ADE(m) |
|---|---:|---:|---:|
| 직진 episode | 0.03860 → 0.02088 | 0.09103 → 0.05182 | 0.04073 |
| 좌회전 episode | 0.04975 → 0.03683 | 0.13319 → 0.12168 | 0.13181 |
| 우회전 episode의 거의 직진 구간 | 0.04986 → 0.02678 | 0.11238 → 0.10520 | 0.02743 |

세 group에서 학습 장면의 선택 후보 ADE가 감소했다. 해당12개에서는 전/후 모두 선택 후보 ADE와 oracle 후보 ADE가 같았다. 따라서 이 작은 TRAIN 집합에서는 후보 선택이 이미 작동했다. 이것으로 전체 TRAIN이나 DEV 선택이 잘 된다고 확대 해석하지 않는다.

일정 속도 기준은 입력 route를 현재 전진 속도 × 시간으로 보간한 위치 예측이다. 정답 미래 속도를 입력하지 않았으며 실제 controller/안전 검증된 계획기가 아니다. oracle은 정답을 사용해 가장 가까운 후보를 고르는 **진단용** 수치이며 배포할 수 없다.

## 우회전 데이터의 결정적인 범위 제한

| 해당 TRAIN episode | 이동 장면 | runtime target gate 통과 | 통과 중 미래 회전량≥15도 |
|---|---:|---:|---:|
| 선택 직진 episode | 328 | 328 | 0 |
| 기존 좌회전 episode | 304 | 131 | 131 |
| 기존 우회전 episode | 414 | 133 | **0** |

선택된 좌회전의 최대 미래 yaw는17.06~58.26도다. 우회전 episode의 선택 구간은0.02~0.07도밖에 회전하지 않는다. **우회전 경로 이름을 근거로 이번 학습을 큰 우회전 성능 개선이라고 부르면 안 된다.**

여기서0개라는 것은 원래 우회전 데이터가 없거나 K1 학습에서 제외됐다는 뜻이 아니다. 기존 K1은 gate 불합격 장면도 포함한 기존 TRAIN을 학습했다. 이번 별도 진단의 물리 통과 subset에서는 실제 우회전 구간이 사라진다는 뜻이다. 또한 runtime 통과가 decoder 표현 가능성의 완전한 증명은 아니다.

## 기존 고정 DEV 근거와 연결

이번에는 DEV를 다시 실행하지 않고 보존된 K 평가 결과를 읽었다.

| 모델/DEV | 실제 선택 ADE(m) | 후보 중 oracle ADE(m) |
|---|---:|---:|
| K0 우회전 | 3.27010 | 1.42484 |
| K1 우회전 | 3.83253 | 1.29909 |
| K1 직진 | 0.51127 | 0.51127 |

K1은 우회전 oracle은 개선됐지만 실제 선택은 악화됐다. 선택 일반화 문제가 크다는 근거이며, oracle에도1.30m 오차가 남아 생성부 문제가 없다고 할 수 없다. 이 값은 현재S 체크포인트의 DEV 성능이 아니다.

## 이번 판단

1. **실제 추가 학습으로 같은 장면의 fitting은 개선됐다.** 학습 전부터4~5cm 수준이었으므로, 적어도 선택된 유효12개를 못 배우는 전반적인 학습 구현 실패는 관측되지 않았다.
2. 동일한 적은 TRAIN을 더 오래 반복하는 것만으로 우회전 일반화가 해결된다는 근거는 없다.
3. 선택 일반화와 유효한 회전 supervision/decoder 표현 가능성을 다음 실험의 대상으로 삼아야 한다. 기능 head를 늘리거나 새로운 수집 brake 설정을 무작정 반복하지 않는다.
4. S 모델은 과적합 진단이므로 승격하지 않는다. 실차·전체 Town·30km/h·closed-loop 성능 향상을 주장하지 않는다.

## 다음 단위

우선 기존 큰 우회전 TRAIN 표본에서 **이미지 encoder/선택기와 분리한 decoder 표현 진단**을 수행한다. 고정 ego/route에서 궤적 latent만 최적화하여 기존 모델 후보와 비교한다. 불합격 원본은 연구 진단으로만 사용하고 학습용 승인 데이터로 바꾸지 않는다. 수치 최적화 결과는 전역 최적/오차 하한의 증명이 아니라 관측된 도달 오차다.

- decoder도 가까이 못 맞추면: 좌표/시간/속도 의미 및 물리 제약과 정답 궤적의 불일치를 구체적으로 수정·검증한다.
- decoder는 맞추는데 네트워크가 못 만들면: encoder/경로 표현 및 회전 학습 분포를 변경하는 대조 실험으로 간다.
- 좋은 후보는 있으나 선택이 틀리면: 같은 조건의 oracle regret와 선택 일반화를 별도 평가한다.

그 뒤 고정 DEV에서 개선/회귀를 비교하고, 별도 미사용 경로에서 실제 모델 주행으로 확인한다. S를 반복해 TRAIN 숫자만 더 줄이는 작업은 다음 목표가 아니다. 다음 진단 구현/검산은 대략30~60분 예상 범위이며 정확한 과금 토큰·잔액은 확인하지 못한다.

전체9영역/30기능 중 기본 주행 모델의 학습·일반화 병목을 검증한 단계다. 자체 모델 승인 closed-loop 기능0/30은 변함없다.

## 파일 및 환경

- 로컬 결과: `artifacts/training/2026-09-22/train_memorization_s_v1/`
- 원격 결과: `/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/train_memorization_s_20260922_v1/`
- 개인 venv: `/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312/`
- 소스: `scripts/e2e/run_train_memorization_s.py`, `summarize_train_memorization_s.py`
- before/after/selection/progress/summary/checkpoint/source 회수 완료. checkpoint 및 실행 소스 SHA256 독립 검산, paired sample identity 검사 완료.
- 결과 검산 단위 테스트5개 통과. 실제 GPU 학습은1,500step 완료 후 종료했고 GPU0 메모리0MiB를 확인했다.
- checkpoint SHA256: `6d837818b0743be40cd96beb8aabe408e972c02a582b09376577793618d19c83`

재검산: `python3 -m scripts.e2e.summarize_train_memorization_s --run artifacts/training/2026-09-22/train_memorization_s_v1 --output NEW_REPORT.json`

학습은 백그라운드에서 계속되고 있지 않다. 이번 실행은 완료 상태다. 설치·삭제·commit/push는 수행하지 않았다.
