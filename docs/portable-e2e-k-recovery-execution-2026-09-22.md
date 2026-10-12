# K0/K1 전체 모델 성능 복구 — 실행 기록

<!-- HH_260906 - Record actual execution milestones separately from plans and model acceptance. -->

## 최신 상태: 학습·평가·사후 검증 완료, 모델 미채택

2026-09-22 03:38:57 KST에 두 모델 각각 15,000 step과 최종 DEV 평가를 완료했다.
K1은 직진 ADE를 2.696 → 0.511 m로 줄였지만 우회전 ADE는 3.270 → 3.833 m로 악화했다.
두 모델 모두 고정 지표 6/9, 형상 3/3이고 K1의 K0 대비 비퇴행도 6/9여서 복구 실패다.
최종 checkpoint·소스·평가 hash와 판정 재계산을 확인했고, 별도 사후 재생에서도 수치가 재현됐다.

[최종 결과·후보 선택 분석·PNG](assets/validation/2026-09-22/portable_e2e_learning_cycle_v1/01_full_model_recovery_k/README.md)

아래 진행 기록은 당시 관측 이력이며 현재도 학습 중이라는 뜻이 아니다.

## 목표와 범위

전체 9영역·30기능 중 기본 이동 경로·속도 성능 복구를 수행한다. 새 기능 헤드, NuRec,
실차 제어, TEST 평가는 포함하지 않는다. 기존과 추가 데이터의 전체 모델 학습을 비교한다.
GPU 0 단독·개인 py312 venv·새 경로 생성 원칙을 유지한다.

## 이번 재개에서 실제 완료한 일

- 성공 수집 4개에 recovery v2 증명서를 발급했다. 이전 v1 증명서는 보존했다.
- 원본 weather·route bytes를 보존하고 봉인 수집 당시 weather를 검증해 Common10으로 변환했다.
- 각 328개 TRAIN, 합계 1,312개. 각 표본의 64개 미래 target에 대한 기존 물리 한계와 이미지 해시를 검사했다.
- 실패한 Town01 S02는 포함하지 않았다. 직진 C-track 두 경로는 같은 물리 그룹으로 취급한다.
- 변환/qualification/K runner 관련 테스트 60개, 새 데이터 통합 도구 방어 테스트 5개 통과.
- 신규 prepared 데이터 약 800 MB를 원격 개인 dataset 폴더로 전송했다.
- 원격에서 기존 1,475 TRAIN + 신규 1,312 TRAIN + 기존 665 DEV 통합, 전체 planning validator 및 실제 loader 검증을 완료했다.
- 통합 dataset SHA: `8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957`.
- 학습 소스 11개와 모델 설정 1개의 로컬/원격 SHA 동일성을 확인했다.
- K0/K1 사전 검사 통과. plan SHA: `4c52532bedca9e146be0d06fb98f9e27b7263ce8cd13840426f81fa88dce4075`.

원본/변환 산출물:
`artifacts/training/2026-09-15/moving_cohort_v1/prepared_recovery_{t07_s02,ct_s01,ct_s02,t01_s01}_common10_v1`.

| 변환본 | dataset.json SHA-256 |
| --- | --- |
| Town07 S02 | ce55c26cbee3ac1d486d13cb361803bd46b68eff3c29a3910224147da93d1994 |
| C-track S01 | d126a8ca509bfed4b8716a332692681386af011e26c4249d2c50372a0a4bdc20 |
| C-track S02 | f301fd92ea701f72b6f1d1b187a5656ff07d60b4b804d1aceed47383f47d0056 |
| Town01 S01 | 3715697a0ca4652d7fbb4ffa2749089876c037f6ec68da955b94986da4e3a974 |

## 원격 실행 위치

- 소스: `/home/pro6000/personal/hwanhong/portable_e2e/source_worktrees/moving_recovery_k_20260922_v1`
- 입력: `/home/pro6000/personal/hwanhong/dataset/prepared/moving-recovery-k-inputs-20260922-v1`
- K1 통합: `/home/pro6000/personal/hwanhong/dataset/prepared/moving-recovery-k1-20260922-v1`
- 결과: `/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/moving_recovery_k_20260922_v1`
- 실행 로그: 위 결과 경로 뒤 `.launch.log`
- 실행 plan: 위 결과 경로 뒤 `.plan.json`

통합은 episode와 label bytes, split을 그대로 두고 dataset의 상대 참조 위치만 바꾼다.
원본은 수정하지 않는다. hash/validator/실제 loader를 모두 통과해야 학습을 실행한다.

## 학습 계약

K0/K1 각각 약 106만 파라미터 전체 fresh fit, 같은 초기 가중치, 15,000 step, batch 4.
두 군 합계 30,000 step·120,000회 sample exposure. K1 추가 직진의 물리 그룹을 균형 샘플링한다.
양쪽 학습 완료 후에만 final checkpoint로 DEV 665개 평가. 정지 유지·독립 seed·새 경로 확인은 후속이다.

2026-09-22 02:38 KST에 SSH 종료에도 유지되는 단일 작업을 제출했다. 시작 전 데이터 재검사를
수행한 뒤 K0 → K1 → 양쪽 final DEV 평가 → decision/summary 저장 순으로 실행한다.
작업 제출과 optimizer step 완료는 다르며, 진행 로그로 실제 학습 시작을 별도 확인한다.

## 실제 학습 시작 확인

2026-09-22 **02:38:52 KST** 시작 기록을 확인했다. 원격 PID는 `1018206`이며 승인된
GPU 0 UUID만 노출되었다. 02:39:48 KST에는 `K0_current1475`의 **250/15,000 step**,
1,000 sample exposure 로그와 GPU 메모리 1,117 MiB를 관측했다. GPU 1은 0 MiB이고
실패 기록은 없었다. 이는 진행 관측이며 최종 학습/평가 완료 또는 성능 복구 판정이 아니다.

02:40:29 KST: K0 **750/15,000 step**, 첫 logical epoch 완료, 평균 TRAIN loss
`2.7574932593`, `dev_observed=false`. 초기화 시간을 포함한 시작 후 약 96초 관측에서
학습은 약 7.8 step/s 수준이다. 이후 속도가 비슷하면 두 군 학습·평가에 대략 60–90분을
예상하지만, K1 이미지 decode·평가 비용에 따라 달라질 수 있다. TRAIN loss만으로 주행 성능
향상 여부를 판단하지 않는다.

로컬 보존: `artifacts/training/2026-09-22/moving_recovery_k_v1/{plan,started}.json`.

사용자 확인 명령(읽기 전용):

```bash
ssh pro6000-training
tail -n 10 ~/personal/hwanhong/portable_e2e/runs/diagnostics/moving_recovery_k_20260922_v1.launch.log
nvidia-smi
```

같은 작업을 다시 실행하지 않는다. 종료되면 결과 폴더의 `summary.json`과 `decision.json`을
확인하고, 실패 시 `failed.json` 또는 실행 로그를 확인한다. 결과를 보기 전에 TEST를 열거나
현재 실행의 epoch/손실/기준을 변경하지 않는다. 두 군이 완료되기 전에는 DEV 성능 판정이 없다.
