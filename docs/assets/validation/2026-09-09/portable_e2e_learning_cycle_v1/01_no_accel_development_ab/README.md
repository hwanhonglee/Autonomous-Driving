# 가속도 입력 사용/제외 — 기존 데이터로 실제 모델 A/B 학습

<!-- HH_260906 - Declare an exploratory legacy-data ablation before training and retain the requirement for later qualified-data validation. -->

## 완료 결과부터 보기

<!-- HH_260906 - Separate successful training execution from failed model-selection criteria and preserve every paired result. -->

**2026-09-09 00:36:22–00:58:53 KST, GPU 0에서 전체 모델 6회 학습·6회 평가·6회 경로 검사를 완료했습니다.**
약 22분 30초 동안 총 9,240 optimizer step, 36,930 sample exposure를 수행했습니다.
이는 준비·합성 테스트가 아니라 개인 venv에서 실제 영상 데이터로 파라미터를 갱신한 결과입니다.
6개 모델 모두 1,540 step을 끝냈고, 평가·감사까지 18단계 모두 exit 0입니다.
완료 후 이 캠페인의 학습 프로세스는 종료됐습니다. 무한 반복 학습이 실행 중이라는 뜻은 아닙니다.

**가속도 입력 제외 B는 채택하지 않습니다.** 세 seed 중 하나만 상대 기준을 통과했고,
세 후보 모두 절대 품질 기준에 미달했습니다. 학습 실행 성공과 모델 성능 합격은 다릅니다.

| Seed | 경로 평균 오차 ADE: A → B (m) | 경로 끝점 오차 FDE: A → B (m) | B 상대 기준 |
|---|---:|---:|---|
| 20260903 | 3.874 → 5.586 | 9.828 → 12.535 | FAIL |
| 20260904 | 6.259 → 4.621 | 12.897 → 11.115 | PASS |
| 20260905 | 4.790 → 6.244 | 11.866 → 13.173 | FAIL |

전체 숫자·선택 후보 빈도·seed별 기준은 [최종 결과표](evidence/results/README.md)와
[검증된 JSON](evidence/results/summary.json)에 있습니다. 여섯 실행 모두 selected geometry는
336/337입니다. 공통 거절 1개는 현재 속도 약 30.02757 km/h가 30 km/h 입력 제한을 초과한
val index108이며, 미검사 경로까지 형상이 통과했다고 해석하거나 해당 sample을 삭제하지 않습니다.

### 학습 곡선과 예측 경로

<!-- HH_260906 - Label offline plots as measurements, not simulator screenshots or learned vehicle-control footage. -->

![전체 여섯 모델의 실제 학습 곡선](evidence/plots/01_training_curves.png)

![같은 validation에서 seed별 A/B 최종 비교](evidence/plots/02_paired_validation.png)

학습 곡선은 9,240개 실제 batch 기록을 모두 사용한 100-step 가중 평균입니다.
마지막 40-step 구간도 포함했고 중간 epoch의 validation 곡선을 새로 만든 것은 아닙니다.
별도의 예측 경로 PNG는 모델마다 val 첫 12개를 고정 출력한 **총 72장**입니다.
차량 원점은 중앙이고 임무 경로·정답·후보 경로를 함께 표시합니다. 초기 warmup 표본이므로
전체 주행·회전 구간을 대표하는 이미지가 아니며, CARLA/Autoware 실행 화면이나 GIF가 아닙니다.

| Seed | A 예측 경로 12장 | B 예측 경로 12장 |
|---|---|---|
| 20260903 | [A 경로 폴더](evidence/runs/seed_20260903/A_physical_input/evaluation/trajectories) | [B 경로 폴더](evidence/runs/seed_20260903/B_no_accel_input/evaluation/trajectories) |
| 20260904 | [A 경로 폴더](evidence/runs/seed_20260904/A_physical_input/evaluation/trajectories) | [B 경로 폴더](evidence/runs/seed_20260904/B_no_accel_input/evaluation/trajectories) |
| 20260905 | [A 경로 폴더](evidence/runs/seed_20260905/A_physical_input/evaluation/trajectories) | [B 경로 폴더](evidence/runs/seed_20260905/B_no_accel_input/evaluation/trajectories) |

### 이번 결과로 알게 된 것과 다음 순서

제공된 가속도를 0으로 제외하는 것만으로는 이번 데이터·학습량에서 일관되게 개선되지 않았습니다.
반대로 제공 가속도가 물리적으로 정확하다거나 실차에서도 반드시 필요하다는 증명도 아닙니다.
세 seed 평균 selected ADE는 A 4.974m → B 5.484m로 악화한 반면, 정답으로 사후 고른
oracle ADE는 약 1.529m → 1.526m였습니다. 좋은 후보를 고르는 문제는 남아 있지만,
이 수치만으로 선택기 하나가 유일한 원인이라거나 후보 경로의 모양이 같다고 단정하지 않습니다.

다음은 정답 수집의 가감속·곡률 품질을 통과시키고, 새 corpus의 경로 선택·정지 의도 라벨을
검증한 뒤 고정 조건으로 재학습하는 순서입니다. 이번 실패를 숨기고 같은 실험을 좋은 seed가
나올 때까지 반복하지 않습니다. 장애물·ACC·차선 변경 등을 학습할 입력·정답·시나리오는
별도 단계이며, 기존 30개 기능의 learned closed-loop PASS 수는 여전히 0개입니다.

## 이번에 학습한 것

사용자는 GPU가 비어 있는 동안 실제 학습부터 빠르게 진행해 달라고 요청했습니다.
이미 구현·단위 검증된 `physical_no_accel.v1`을 사용해 입력 가속도 값의 영향부터 비교합니다.
기존 physical-v1도 같은 환경에서 새로 학습하여 비교 기준을 맞춥니다. 선택기만 학습하는
것이 아니라 영상 인코더를 포함한 전체 모델 파라미터를 학습하는 실험입니다.

**시작 당시 확인:** 개인 venv·GPU 0에서 첫 A 모델의 optimizer step 339→340→341 증가와
유한한 loss를 확인했습니다. 이후 여섯 실행의 완료 기록을 확인한 최종 결과는 위에 정리했습니다.
[고정 계획](../../../../../../config/portable_e2e_no_accel_development_20260909.json)은 실행 전
커밋 `3147718`, 실제 서버 코드는 `213cfc6`으로 고정했습니다. [실행 연결 기록](execution_binding.json)에
시작 시각과 계획 해시를 남겼습니다. 계획은 저장소 밖 개인 plans 폴더에 두고, 실행 중인
서버 소스는 바꾸지 않았습니다. 학습은 SSH 세션과 분리해 접속 종료에도 유지되도록 실행했습니다.

| 조건 | A: 기존 입력 | B: 가속도 값 제외 |
|---|---|---|
| 모델 ID | `portable_e2e.perspective_trajectory.physical.v1` | `portable_e2e.perspective_trajectory.physical_no_accel.v1` |
| 영상·경로·ego history | 기존 입력 | 동일 |
| ego history의 가속도 두 채널 | 기록된 값을 사용 | 모델 내부에서 모든 시점의 값만 0으로 제외 |
| 구조·파라미터 수 | 954,590개 | 동일 |
| 초기화 | seed별 새 초기화 | 같은 seed·공유 초기화 코드; 실제 초기 텐서 해시는 별도 저장하지 않음 |
| 학습률·batch·optimizer step | 0.0001·4·1,540 | 동일 |
| seed | 20260903·20260904·20260905 | 동일 |
| 평가 | Town03 validation 337개 | 동일 |

완료 범위는 총 6회 학습·6회 평가·6회 기존 runtime geometry 감사입니다. 1,540 step은
이 데이터에서 전체 epoch 5회와 부분 epoch 1회(약 5.37 epoch)이며, 10 epoch가 아닙니다.
6개 모델 모두 마지막 고정 step의 checkpoint를 사용합니다. 가장 좋은 validation 시점이나
seed만 고르지 않으며, 원래 C 모델 학습 결과를 이번 A의 새 실행처럼 재사용하지 않습니다.

## 왜 지금 기존 데이터로 하는가?

<!-- HH_260906 - Explain the changed scheduling assumption without relabeling legacy data as newly admitted data. -->

[이전 계획](../../../../../portable-e2e-no-acceleration-ablation.md)은 새 수집 데이터의 품질을
확인한 뒤 비교하는 순서였습니다. 그 품질 검증 단계는 남겨두되, 학습을 앞당기기 위해
**기존 v3 데이터의 한계를 명시한 개발용 실험을 먼저 병행**합니다. 이 결과로 새 수집 데이터의
문제가 해결됐다고 판정하거나 실차·주행 모델을 채택하지 않습니다.

기존 학습용 v3는 train 1,147개(Town07 직진·기존 C-track 좌회전·Town01 우회전),
val 337개(Town03 우회전)입니다. 여기서 기존 C-track episode는 9월 8일 새로 수집했다가
미승인된 8회와 다릅니다. 신규 8회는 추가하지 않습니다. 기존 데이터의 라벨·split·sample·
warmup 표시는 그대로 두며, 명시적 사용금지 표시가 없다는 사실을 새 품질 승인으로 해석하지 않습니다.

기존 정답의 감속 한계 초과·warmup 포함 문제는 남아 있습니다. 따라서 입력 가속도의
영향을 같은 조건에서 비교하는 데 의미가 있으며, 좋은 결과가 나와도 정지 데이터 품질·
독립 test·새 도로 일반화·실제 제어 성능을 입증하지 않습니다.

## 입력의 시간 경계와 test 사용 범위

<!-- HH_260906 - Distinguish corpus integrity reads from held-out test inference and prevent future target leakage into model inputs. -->

모델에는 현재 6카메라 영상·보정값·현재와 과거 ego 상태·현재 위치에 투영한 사전 임무 경로만
들어갑니다. 미래 XY·속도·방향 정답은 loss 계산 전용입니다. 원본의 goal·phase는 이 모델의
입력이 아니며, 이번 실험은 정지 의도를 학습하는 모델이나 정지 primitive 연결 학습이 아닙니다.

**Town04 test는 학습 loss·모델 추론·평가지표·모델 선택에 사용하지 않습니다.** 다만 기존
로더의 전체 corpus 무결성·계약 검사는 split을 고르기 전에 test JSON/JPEG 바이트도 읽어
형식과 해시를 확인합니다. 따라서 “test 파일을 전혀 열지 않는다”고 표현하지 않습니다.
이 검증을 건너뛰거나 로더를 수정하지 않습니다.

## 사전 비교 기준

<!-- HH_260906 - Retain fixed relative and absolute gates while reporting every paired seed, including unsuccessful ones. -->

세 seed 모두에서 B의 selected ADE와 FDE가 A보다 작고, selected geometry 통과 수는
줄지 않으며, 속도 MAE는 A 대비 5%를 넘게 악화하지 않아야 상대 기준을 통과합니다.
절대 기준은 1초 ADE ≤0.5m, 3초 ≤1m, 6.4초 ≤2m와 6.4초 FDE ≤4m입니다.
좋은 평균으로 실패 seed를 가리지 않으며 물리 한계·runtime gate·loss는 변경하지 않습니다.

원격 개인 venv와 physical GPU 0만 사용합니다. 기존 shadow checkpoint와 runtime bundle
허용 목록은 그대로 유지하며, 패키지 설치·GPU 1 사용·다른 작업 종료·재부팅·서버의
CARLA/Autoware 빌드·차량 제어는 하지 않습니다.

## 서버에서 진행 상태와 모델 파일 확인하기

<!-- HH_260906 - Offer read-only operational commands for the existing SSH alias and keep repeated dispatch separate from inspection. -->

현재 설정해 둔 SSH 별칭 기준입니다. 아래는 **상태 확인용**이며 새 학습을 시작하는 명령이 아닙니다.

```bash
ssh pro6000-training
cd ~/personal/hwanhong/portable_e2e
nvidia-smi -i 0
venvs/py312/bin/python - <<'PY'
import json
from pathlib import Path
root = Path('runs/campaigns/hh260909-no-accel-development-ab-3seeds-v1')
state = json.loads((root / 'status.json').read_text())
print('Campaign:', state['status'])
print('Completed stages:', sum(s['status'] == 'COMPLETE' for s in state['stages']), '/ 18')
for stage in state['stages']:
    print(stage['run'], stage['stage'], stage['status'])
PY
```

모델 가중치는 개인 작업공간의 아래 형식으로 모델마다 따로 저장됩니다.

```text
runs/campaigns/hh260909-no-accel-development-ab-3seeds-v1/
  seed_20260903/
    A_physical_input/training/checkpoints/latest.pt
    B_no_accel_input/training/checkpoints/latest.pt
  seed_20260904/...
  seed_20260905/...
```

각 `training/metrics.jsonl`은 step별 실제 학습 수치, `training/run.json`은 설정·완료 상태,
`evaluation/metrics.json`은 val 평가, `gate_v8.json`은 기존 출력 제한 검사입니다.
checkpoint는 모델·optimizer 상태를 담은 연구 파일이며 Git에 넣거나 현재 차량에 연결하지 않습니다.
같은 campaign ID로 다시 실행하면 기존 결과 보호를 위해 거절합니다. 재시험은 새 계획을 먼저
고정해야 하며, 학습 도중 원격 저장소에 `git pull`하거나 코드를 수정하면 소스 검증에서 중단될 수 있습니다.

실행 중 SSH 조회가 한 차례 연결 리셋됐으나, 재접속에서 9단계 완료와 다음 모델 학습 진행을
확인했습니다. 이것은 학습 실패가 아니라 상태 조회 연결의 일시적 문제였습니다.

## 로컬 코드 검증

<!-- HH_260906 - Report full local regression counts once, distinct from six actual GPU fits and their validation performance. -->

최종 로컬 코드 `6a1b56f`에서 **4,884 passed / 6 skipped / 0 failed**였습니다.
일반 테스트 4,511개와 런치 테스트 373개의 합계이며, 집중 테스트 수를 다시 더하지 않습니다.
[실제 명령·검사 범위](verification.json), [일반 원본 로그](tests/main.log),
[런치 원본 로그](tests/launch.log), [실행 전후 소스 493개 SHA](tests/source_sha256.json)를 보존합니다.

건너뛴 5개는 기존 로컬 Torch의 안전한 `weights_only` 로딩 기능 부재, 나머지 1개는
고정 Woraksan 맵 묶음 부재 때문입니다. 검사를 우회하거나 패키지를 새로 설치하지 않았습니다.
원격 실제 학습 소스 `213cfc6`과 로컬 결과 요약·표시를 추가한 검사 소스를 구분합니다.
이 코드 검사 통과는 새 모델의 주행 성능·실차 안전 통과를 뜻하지 않습니다.

## 자료 보관과 재검증

<!-- HH_260906 - Keep exact raw training evidence private while making public derivatives and byte-identical plots unambiguous. -->

로컬 원본은 `artifacts/training/2026-09-09/no_accel_development_ab_v1/`에 있습니다.
모델 가중치는 앞서 적은 원격 개인 runs 경로에 남겨뒀으며 로컬 다운로드·모델 로딩은 하지 않았습니다.
[원격 완료 확인](evidence/provenance/remote_completion.json)에는 원격 파일·최종 checkpoint의
바이트 SHA와 GPU 0·소스 상태 확인을 기록합니다. 요약기의 `checkpoint_bytes_locally_verified=false`와
원격 바이트 해시 검증은 서로 다른 검사입니다.

[공개 manifest](evidence/publication_manifest.json)는 원본 SHA와 공개 파일 SHA를 함께 기록합니다.
원격 원본 116개·5,521,583 bytes와 로컬 수집본의 경로·크기·SHA가 모두 일치했습니다.
[발행 후 검사 기록](publication_verification.json)에 그래프 입력·PNG 해독·문서 링크·공개 해시 검사를 남깁니다.
`runs`의 JSON과 `provenance/plan.json`, `status.json`은 개인 경로를 치환한 **명시적 파생본**입니다.
원본 학습 JSONL·72장 예측 PNG·2장 학습 그래프·최종 요약은 바이트 그대로 보존했습니다.
원본 stage 로그와 실행 runner는 private 자료에 남겼고, checkpoint는 Git에 넣지 않았습니다.
그래프의 입력 SHA는 private 원본을 가리키므로 치환된 JSON으로 독립 재실행했다고 주장하지 않습니다.

이 저장소에서 공개 파일 무결성은 다음 명령으로 확인할 수 있습니다. 추가 패키지는 필요 없습니다.

```bash
cd docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/01_no_accel_development_ab/evidence
sha256sum -c SHA256SUMS
```
