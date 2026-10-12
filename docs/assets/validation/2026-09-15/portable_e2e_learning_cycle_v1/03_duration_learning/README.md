# 03 — 전체 모델 이어학습과 epoch별 검증 곡선

<!-- HH_260906 - Prioritize actual whole-model optimization and per-epoch validation; preserve all historical data and deployment boundaries. -->

상태: **9월 15일 19:29:38 → 21:30:27 KST, GPU 0 실제 전체 모델 이어학습·평가 완료.**
6개 계보가 모두 20 epoch에 도달했고 90개 epoch 종점·282개 실행 단계를 독립 검산했다.
최종 판정은 `PASS_NOT_PROMOTED`: 실행·증거 검증 완료이며 모델 채택은 아니다.

[전체 성능표·학습 곡선·고정 144개 예측 경로](results/README.md) ·
[정지·재가속 선택 진단](results/behavior_decisions.md) · [모든 행동별 오차](results/behavior_metrics.md).

## 실제로 얼마나 좋아졌나?

동일 개발 VAL 337개에서 부모 대비 6.4초 경로 평균 오차는 세 seed 단순 평균으로
**A 5.4327→4.2282m(22.2% 감소), B 4.5539→3.2918m(27.7% 감소)**였다.
6개 중 5개는 이 오차가 줄었지만, B/seed 20260903은 **3.6452→3.8871m로 악화**했고
끝점·속도 오차도 악화했다. A의 1초 오차는 세 seed 모두 부모보다 커졌다.
반복 노출 증가에 효과가 있었지만 모든 지표가 일관되게 좋아진 것은 아니다.

최종 6.4초 평균 오차는 2.7490–4.5743m, 끝점 오차는 7.2355–10.3018m다.
네 가지 경로 목표를 모두 만족한 모델은 **0/6**이다. 학습 부족만으로 모든 문제를 설명하거나,
20 epoch만으로 충분히 학습했다고 판단하지 않는다. 독립 장면과 정지 검증의 부족도 그대로다.

### 평균 개선으로 가려지면 안 되는 행동 결과

- 가장 낮은 전체 ADE를 낸 B/seed05도 감속 후 정지 정답 51개 중 예측 종점 속도 ≤0.1m/s가
  **41/51**이었다. epoch 10의 51/51보다 줄었다. 정지군 경로 오차가 좋아져도 정지 시점까지 좋아졌다는 뜻은 아니다.
- 같은 epoch 20의 B/seed03은 A보다 계속 이동 그룹의 끝점 오차 **31.8%**, 속도 오차 **31.8%**가 커졌다.
  A/seed05의 계속 이동 속도 오차도 epoch 10보다 **31.2%** 커졌다.
- VAL 정지 유지 표본은 **0개**다. TRAIN 정지 유지 118개를 맞춘 것을 독립 검증으로 세지 않는다.
- 미래 속도 0 뒤 재가속 예측도 남았다. 대부분 기타 움직임 그룹이며 명시적 의도 정답이 없어
  모든 경우를 불필요하거나 위험한 재출발로 판정하지 않는다. 실제 차량 사건 수가 아닌 예측 진단이다.
- 기존 geometry 검사 거부도 모든 모델에서 TRAIN 6/1,147·VAL 1/337로 남았다.

## 다음에 성능을 올릴 작업

**이번 캠페인은 종료됐으며 아래 새 학습이나 구조 변경은 아직 실행하지 않았다.**

1. 새 moving 데이터 변환·수신 근거를 유지하면서 서로 다른 TRAIN/DEV 직진·회전과 정지·재출발
   장면을 확보한다. 특히 현재 비어 있는 독립 VAL 정지 유지 장면이 필요하다. 새 328개는 별도
   Town07 TRAIN pilot이며 이번 1,147개 학습에 넣었다고 소급 표시하지 않는다.
2. 다음 모델 개입 가설은 **12개 후보의 실제 XY·속도를 점수 선택기에 입력하고 전체 모델과 공동 학습**하는 것이다.
   현재 STOPMIX 점수 head는 공유 문맥에서 직접 점수를 낸다. 최종 B의 평균 선택 ADE 3.2918m와
   후보 중 최소 ADE 1.3562m 사이의 1.9356m 차이는 선택 개선 여지를 보여주지만, 개선 보장은 아니다.
   기존 선택기 전용 실패 실험과 구분하고, 같은 크기·같은 초기 잔차·동일 데이터 A/B로 검증해야 한다.
   데이터 변경과 선택기 변경을 한 비교에 섞지 않는다.
3. 절대 경로 품질·행동별 퇴행 기준을 충족한 후보에 한해 export 일치 → 10 Hz shadow →
   제어 소유권·fallback을 갖춘 CARLA 제한 폐루프 검증으로 진행한다. 장애물·ACC·차선 변경에는
   해당 상호작용을 담은 추가 장면과 평가가 필요하다. 현재 짧은 경로 데이터를 반복하는 것만으로
   그 기능들이 완료되지는 않는다. 실제 차량 적용 승인은 별도다.

VAD/METEOR와 같은 데이터·평가기로 직접 비교한 결과는 없다. 동급·우월한 성능이라고 주장하지 않는다.

## 이번에 확인할 질문

기존 모델을 더 오래 학습하면 독립 개발 VAL의 경로 오차도 줄어드는가?
TRAIN 손실만 감소하는지, 단기/장기 경로와 정지/지속 주행 사이에 퇴행이 생기는지 확인한다.
이전 수집 제어기 진단과 별개인 **실제 전체 모델 학습 캠페인**이다.

## 고정 범위

- 부모: 9월 14일 STOPMIX 손실 A/B × seed 3개의 전체 모델 checkpoint 6개.
- 각 부모의 1,540 step·6,155회 표본 노출에서 같은 가중치·optimizer·RNG 상태를 이어간다.
- 목표: 모델마다 20 epoch = 5,740 step·22,940회 누적 노출.
- 실제 추가 학습량: 모델마다 4,200 step·16,785회 노출, 총 25,200 step·100,710회 노출.
- epoch 6부터 20까지 매 epoch 고정 VAL 337개 평가·기존 geometry gate 검사.
- 모든 모델을 epoch 6까지 진행한 뒤 epoch 7로 가는 순서다. 중간 결과도 같은 학습량으로 비교한다.
- epoch 10·20에서 TRAIN/VAL 행동별 분석과 고정된 예측 경로 그림을 남긴다.
- 20 epoch는 충분한 학습 보장이 아니다. 이번 고정 구간의 검증 곡선을 보고 다음 예산·변경을 판단한다.
- 무조건 개선될 때까지 재시도하거나 가장 좋은 seed만 선택하지 않는다. 이번에는 자동 모델 채택이 없다.

데이터는 기존 TEST 제외 연구 뷰 그대로다. TRAIN 3 episode·1,147표본(원본 약 134초),
개발 VAL은 Town03 우회전 1 episode·337표본이며 정지 유지 표본은 0개다.
반복 노출 수를 독립 주행 수로 세지 않는다. 최근 미승인 CARLA 수집은 넣지 않는다.

이전 연구 뷰 생성 증명서의 `fresh_fits_only`/`optimizer_resume_allowed=false` 표기는 변경하지 않는다.
이번 새 실행 계획은 **그 뷰에서 새로 학습했던 정확히 6개의 동일 fingerprint 계보만** 이어학습하도록
별도로 선언한다. 원래 corpus나 다른 데이터로 학습한 checkpoint의 이어학습을 허용하는 것이 아니다.

## 두 환경과 결과 해석

| 환경 | 이번 역할 | 제한 |
| --- | --- | --- |
| PRO6000 | physical GPU 0, 기존 개인 py312 venv에서 이어학습·VAL·행동 분석 | 설치·GPU 1·시스템/Conda·Autoware 빌드 없음 |
| 로컬 | 학습 결과 회수·곡선/비교 보고서, TRAIN 30/DEV 10 데이터 확대 준비 | 이번 예측 그림을 CARLA/Autoware 직접 주행 영상으로 표시하지 않음 |

새 데이터 확대 계획은 [TRAIN 30 / DEV 10 제안](../../../../../portable-e2e-data-expansion-2026-09-15.md)을 본다.
이동 전용 수집 모드를 구현하고 [Town07 seed 0 pilot 1회](../04_moving_data_pilot/README.md)를
완료했다. [TRAIN 328개 변환·원격 수신 검증](../05_moving_train_dataset/README.md)까지 마쳤지만,
cohort seed 101 슬롯이나 40개 수집을 완료한 것이 아니다. 이번 캠페인에는 넣지 않았다.

학습 결과는 경로 평균 오차(ADE), 끝점 오차(FDE), 속도 오차, 후보 최선 오차, 물리 검사,
정지/주행 subgroup의 개선·악화를 함께 본다. TRAIN 곡선은 **가중치가 변하면서 측정한 batch 손실**이며
고정 checkpoint의 TRAIN 재평가와 다르다. A/B 총 손실은 목적함수가 달라 서로 직접 비교하지 않는다.

## 재현 코드

- [학습 실행기](../../../../../../tools/e2e/run_stopmix_duration_learning.py)
- [곡선 분석기](../../../../../../tools/e2e/summarize_stopmix_duration_learning.py)
- [부모 학습 결과](../../../2026-09-14/portable_e2e_learning_cycle_v1/05_stopmix_regret_learning/README.md)

서버의 `$PORTABLE_E2E_ROOT/runs/diagnostics/stopmix_duration_20260915_v1`에
새 폴더를 사용하며 부모·기존 운용 모델은 바꾸지 않는다. 실행 중에는 `progress.json`, 종료 뒤에는
최종 실행 증명서와 각 epoch의 report/checkpoint를 기준으로 판단한다. 이미 실행 중인 같은 폴더에
명령을 다시 실행하지 않는다.

실행기 SHA-256: `9d18ba37ddf0b7a790987e12cd7bd16268a61bd895b1eeb4c161343e02b9e9e0`.
사전 계획 SHA-256: `5b7c00d86868aaf7e51d36300f653ed2d4acb4df4b76fafae334ce8fbd179bf8`.
실행 당시 관련 로컬 검증은 75 passed였고, 후속 moving 데이터 변환·전송·발행을 포함한
최종 관련 회귀검사는 257 passed다. 학습 성능 통과 수와 다르다. 선언 준비 중 상대 `../` 경로가
canonical 검사에서 한 번 거부됐고 절대 경로로 바로잡았다. 해당 거부 때는 계획 생성·학습 실행 전이었다.

## 진행 상태 확인 — 새 학습을 시작하는 명령이 아니다

로컬에서 `ssh pro6000-training`으로 접속한 뒤 다음처럼 조회한다.

```bash
cd "$HOME/personal/hwanhong/portable_e2e"
venvs/py312/bin/python -B benchmarks/stopmix_duration_20260915_v1/inspect_stopmix_duration_run.py \
  --run-root "$PWD/runs/diagnostics/stopmix_duration_20260915_v1" --gpu0
tail -n 10 benchmarks/stopmix_duration_20260915_v1/runner.log
```

`epoch/run/stage`는 진행 위치이고 `current_training_tail.global_step`은 마지막 기록 갱신이다.
새 프로세스 시작·데이터 검사·평가 사이에는 GPU 사용률이 잠깐 0%일 수 있다. 끝난 것은
`summary.json`의 종료 상태와 282개 완료 단계, 부모/소스/데이터 보존 검증까지 확인해야 한다.
`tmux` 세션 이름은 `hh260915-stopmix-duration`이며 SSH 종료와 독립적으로 실행된다.
전체 실행 상한은 6시간이고 단계별 timeout·소유 프로세스 정리가 적용된다.

실제 종료 뒤 원격 checkpoint 90개를 포함한 전체 파일 inventory와 부모 파일을 다시 해시하고,
로컬에 회수한 비-checkpoint 파일, exact-resume 연결, 모든 epoch report, 행동별 17,808개 행과
PNG를 검산했다. 원본 source·데이터 뷰·부모 checkpoint의 전후 보존도 통과했다.
원격 inventory SHA는 `a021244ba9da0efd7264f31d5989dc34ab58d977003a2e1cece5de926cc7cd6f`,
독립 검산 report SHA는 `8b1cecf2a9dbc54653aac47a8721561e83045635243e85ea01eb534b178d1da7`다.
발행 결과는 [자체 SHA256SUMS](results/SHA256SUMS)의 168개 항목을 모두 검증했다.

**모델 직접 주행 통과 0/30과 실차 미승인은 그대로다.** 학습 완료와 기능 통과를 구분한다.
