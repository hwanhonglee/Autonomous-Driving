# AD — 기존 K1 전체 모델 추가 학습 실행

<!-- HH_260906 - Supersede the historical running snapshot with completed non-promoted results. -->
## 완료 결과 — 2026-09-22 16:30:53 KST

추가 15,000 step 학습과 DEV 665개 평가가 완료됐다. 아래 실행 중 기록은 과거 조회다.
원격 원본은 [completed_summary.json](completed_summary.json)으로 회수했다.

| 지표 | 기존 K1 | 추가 학습 AD |
|---|---:|---:|
| 우회전 평균 궤적 오차 (m) | 3.8325 | 3.4242 |
| 우회전 마지막 위치 오차 (m) | 11.4235 | 9.9124 |
| 우회전 속도 오차 (m/s) | 1.7461 | 1.4624 |
| 직진 평균 궤적 오차 (m) | 0.5113 | 0.5991 |
| 직진 속도 오차 (m/s) | 0.2573 | 0.2693 |

기존 K1 대비 비퇴행 7/9, 별도 strict gate 6/9로 미채택. 형상 통과 664/665는
궤적 정확도나 자율주행 성공률이 아니다. 우회전 평균 오차는 약 10.7% 줄었지만
직진 퇴행과 우회전 품질 부족이 남는다. 주행 기능 승인 0/30 유지.
학습은 끝났으며 자동으로 다음 학습을 시작하지 않는다. 실행 모델 교체도 하지 않았다.
다음 주행 개선은 우회전 후보 생성/선택 오차와 데이터 다양성을 확인해 정하고,
직진·정지 회귀를 함께 비교해야 한다. 지도 기능 준비는 별도 트랙으로 진행한다.

<!-- HH_260906 - Quantify aggregate candidate and selection effects without using future truth for driving. -->
### 후속 오차 분해

[재현 가능한 분석](selection_gap.json)은 기존 평가 원본 두 개만 읽는다. 새 학습·추론은 없다.

| 우회전 평균 오차 | K1 | AD |
|---|---:|---:|
| 실제 선택 | 3.8325 m | 3.4242 m |
| 정답을 보고 고른 최선 후보 (oracle) | 1.2991 m | 1.3237 m |
| 선택에 따른 추가 오차 | 2.5334 m | 2.1005 m |

이번 우회전 개선은 평균 분해상 후보 최저 오차 개선이 아니라 선택 gap 감소에서 나왔다.
그러나 여전히 선택 gap 2.10m가 남고 최선 후보도 1.32m 오차다. oracle은 미래 정답을
사용하는 분석용 수치로 운용 불가하며, 선택기 수정만으로 그 수치에 도달한다는 보장이 없다.
직진 oracle도 0.5113→0.5989m로 악화하여 선택기만 고쳐서는 직진 퇴행을 복구할 수 없다.
다음 학습은 후보 생성 품질과 후보 선택을 함께 비교하고 직진/정지 보존을 확인해야 한다.
반복 관측한 동일 DEV 결과이며 새로운 경로 일반화나 기능 완료를 주장하지 않는다.

<!-- HH_260906 - Resume weights, not optimizer state; keep running work separate from completed results. -->

## 목표와 실행 범위

미세 경계 검사를 모델 학습의 선행 조건에서 분리하고, 기존 K1 가중치에서 전체 모델 추가 학습을 시작했습니다. **추가 학습만으로 기존 데이터의 회전 다양성 부족이 해결된다고 가정하지 않습니다.** 더 낮은 학습률로 학습을 이어갔을 때 기존 모델 대비 평가 성능이 개선되는지 확인하는 실험입니다.

- 원격 개인 venv: `/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312`
- 허용 GPU: 물리 GPU 0, UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`만 노출. 소유 lock 사용.
- 기존 TRAIN 2,787개, 데이터 fingerprint 검증. 새 합성 경로·목표점 변경 후보·DEV 라벨을 학습에 추가하지 않음.
- 기존 K1 전체 가중치에서 추가 15,000 step, batch 4, 학습률 `3e-5`.
- 기존 K1은 15,000 step 학습된 모델. 정상 완료 시 가중치 기준 누적 30,000 step. AdamW optimizer 상태는 원본에 없어 새로 시작함.
- 원본 손실·샘플링·모델 구조 유지. 기능 헤드 추가 없음.
- 학습 종료 후 기존 Town03 DEV 665개에서 한 번 평가하고 원래 K1과 비교하도록 연결함. 학습 도중 DEV 기반 선택/조기종료 없음.

이 DEV는 학습에는 쓰지 않지만 이전에 반복 관측한 연구 평가 자료입니다. 새 독립 평가가 아니고 좌회전 subgroup도 없습니다. 비교 결과가 좋아도 실차·폐루프·모든 맵 검증 완료로 주장하지 않습니다.

## 실행 상태 확인

수정본 v2 시작 확인: **2026-09-22 15:59:08 KST** (`started.json`). 이 문서는 실행 시작 기록이며 완료 보고서가 아닙니다. 완료 여부는 원격 `summary.json`과 로그로 확인해야 합니다.

이번 턴 마지막 확인: **500/15,000 step, 2,000 sample exposures**. 프로세스 PID 1190746이 허용 GPU 0에서 1,108 MiB를 사용 중이었습니다. 이는 조회 시점 기록이며 실시간 상태 표시가 아닙니다.

첫 v1은 기존 평가 요약이 실제 지표가 아닌 파일 참조임을 발견해, 제가 시작한 프로세스만 초기에 중단했습니다. 비교 대상 파일의 SHA를 검증하고 읽도록 수정 후 v2를 시작했습니다. v1 실패 로그는 보존했습니다. 다른 작업은 중단하지 않았습니다.

기존 실행 속도 기준 예상은 약 30–40분이며 보장값이 아닙니다. 추가 토큰 사용량·계정 잔액은 확인할 수 없습니다. 학습은 원격 프로세스로 실행되며 GPU 계산 시간과 대화 토큰 사용량은 별개입니다.

## 결과 확인 명령

```bash
ssh pro6000-training
tail -n 10 ~/personal/hwanhong/portable_e2e/runs/diagnostics/full_continuation_ad_20260922_v2.launch.log
nvidia-smi
```

산출물 루트:
`/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/full_continuation_ad_20260922_v2`

- `declaration.json`: 단계 수·학습률·baseline hash·제약
- `started.json`: 실제 학습 루프 진입 시각
- `K1_current_plus_successful4/training/epoch_history.jsonl`: 750 step마다 기록
- 같은 training 폴더의 `final_checkpoint.pt`: 15,000 step 완료 후 생성
- `K1_current_plus_successful4/final_dev_evaluation.json`: 학습 종료 후 평가
- `summary.json`: 학습·평가·원래 K1 대비 비교 완료 시 생성

로그의 `K_TRAIN_PROGRESS`가 250 step마다 증가해야 합니다. `summary.json`이 없다는 사실만으로 실행 중인지 실패했는지 구분할 수 없으므로 프로세스와 로그를 함께 확인해야 합니다.

## 다음 판단

직진·우회전의 ADE/FDE·속도 오차와 기존 안전 검사를 원래 K1에 비교합니다. 개선되지 않으면 그대로 채택하지 않습니다. 데이터 다양성/후보 선택 문제와 단순 추가 최적화 문제를 구분하는 증거로 사용합니다. 좌회전·정지 보존·독립 경로·CARLA 주행 검증은 이 실행에 포함하지 않습니다. 모델 자동 교체·배포는 금지합니다.

로컬에서는 실행 코드와 기록만 추가했습니다. 패키지 설치나 CARLA 실행은 하지 않았습니다.
