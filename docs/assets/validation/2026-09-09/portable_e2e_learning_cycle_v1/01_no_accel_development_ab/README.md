# 가속도 입력 사용/제외 — 기존 데이터로 실제 모델 A/B 학습

<!-- HH_260906 - Declare an exploratory legacy-data ablation before training and retain the requirement for later qualified-data validation. -->

## 이번에 학습하는 것

사용자는 GPU가 비어 있는 동안 실제 학습부터 빠르게 진행해 달라고 요청했습니다.
이미 구현·단위 검증된 `physical_no_accel.v1`을 사용해 입력 가속도 값의 영향부터 비교합니다.
기존 physical-v1도 같은 환경에서 새로 학습하여 비교 기준을 맞춥니다. 선택기만 학습하는
것이 아니라 영상 인코더를 포함한 전체 모델 파라미터를 학습하는 실험입니다.

**현재 상태: 실행 준비.** 이 선언 자체가 학습 시작·완료를 뜻하지 않습니다.

| 조건 | A: 기존 입력 | B: 가속도 값 제외 |
|---|---|---|
| 모델 ID | `portable_e2e.perspective_trajectory.physical.v1` | `portable_e2e.perspective_trajectory.physical_no_accel.v1` |
| 영상·경로·ego history | 기존 입력 | 동일 |
| ego history의 가속도 두 채널 | 기록된 값을 사용 | 모델 내부에서 모든 시점의 값만 0으로 제외 |
| 구조·파라미터 수 | 954,590개 | 동일 |
| 초기화 | seed별 새 초기화 | 같은 seed의 동일 초기 가중치 |
| 학습률·batch·optimizer step | 0.0001·4·1,540 | 동일 |
| seed | 20260903·20260904·20260905 | 동일 |
| 평가 | Town03 validation 337개 | 동일 |

계획은 총 6회 학습·6회 평가·6회 기존 runtime geometry 감사입니다. 1,540 step은
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
