# 실제 좌우회전 균형 학습 진행 상황

<!-- HH_261002 - Separate the actual fresh GPU0 B5 fit from data preparation, offline diagnostics and capability approval. -->

**19시 38분 확인: PRO6000 물리 GPU0에서 실제 학습 중이다.** 19:05:51에 서버 전용 로그를 쓰는 독립 프로세스를 시작했고, 19:06:29에 실제 자료의 CUDA 역전파·영상 검사 뒤 optimizer 학습에 들어갔다. 현재 **4,638/20,000회·23,190/100,000개 입력 노출**을 확인했다. 소스 31개는 그대로이며 오류 로그는 없다. 시작 영수증만으로 학습을 주장하지 않고 실제 업데이트 기록을 확인했다.

메인 **0/9**, 하위 **0/30**이다. 현재 집중하는 하위 목표는 직진·좌회전·우회전·목적지 정차·속도 추종 5개이며, 이번 학습에는 정상 자료가 있는 직진·복귀·좌우회전을 넣었다. 정차 실패 원본은 여전히 학습하지 않는다. 이번 학습 완료만으로 정차·ACC·회피·차선변경 또는 기능 승인을 주장하지 않는다.

## 무엇을 바꾸고 왜 학습하는가

이전 좌회전 학습은 실제 약 89도 회전·차체와 지연 기준을 통과했지만, 차로 중심 오차 p95가 0.413m로 기존 0.35m에 미달했다. 같은 모델의 추종 거리 A/B에서 0.414→0.386m로 줄어도 미달은 남았다. 기존 저속·복귀·우회전 보호 진단도 14개 실패했다. 새로운 head를 추가하지 않고, **실제 저속 우회전 697개를 좌회전 347개와 균형 학습하면서 기존 직진·복귀 보호 손실을 유지**한다.

직전의 좌회전 전용 20,000회 모델을 이어 쓰지 않는다. 원래 6,000회 공동 학습 모델에서 새 AdamW로 고정 20,000회를 시작했다. 기존 101개 가중치 구조·학습 54개 텐서·고정 47개 텐서를 유지한다. 한 업데이트에 복귀·기존 저속·이전 주행·좌회전·우회전 5개 입력을 사용한다. 이전 보호 손실 계수는 그대로이고 좌·우회전이 기존 회전 항을 절반씩 나눈다. 자료·일정·균형 손실을 함께 바꾸므로 한 가지 변화만의 효과라고 주장하지 않는다.

| 구분 | 실제 범위 |
| --- | --- |
| 학습 장소 | PRO6000의 개인 `hwanhong` 폴더와 기존 `venvs/py312` |
| 학습 장치 | 물리 GPU0만, 지도 학습 종료·잠금과 장치 비어 있음 확인 뒤 실행 |
| 학습 자료 | 20개 주행 기록·전체 6,876개 기준 시점, 기존 규칙상 eligible 6,576개 |
| 새 회전 정답 | LEFT 347개·RIGHT 697개, 실제 64점 미래·속도·차체 검사 완료 자료 |
| 학습 일정 | 고정 20,000회·B5·100,000개 노출, 좌·우회전 각각 20,000개 노출 |
| 최종 선택 | 고정 마지막 모델만, 중간에 좋은 체크포인트 선택 없음 |
| 평가 | 최종 가중치·전체 업데이트 검증 뒤 부모와 최종 모델 전량 비교 |

우회전 70초 실패 자료와 정차 실패 자료는 제외한다. 미래를 늘리거나 정지 곡률을 0으로 만들지 않았다. 학습에 넣은 자료와 앞으로 독립 평가할 자료를 섞지 않는다. 새 RIGHT 경로는 실제 개인 `datasets/prepared`이고 LEFT·기존 자료는 원래 `dataset/prepared`이다. 경로 별칭이나 데이터 중복 복사로 문제를 숨기지 않았다.

## 실제 확인한 것과 아직 남은 것

**완료:** ROOT의 새 학습 코드 127개·실행 코드 51개 CPU 검사, 독립 검토자의 새·기존 학습 코드 216개·실행 코드 51개 검사, 서버 소스 31개 동일성, 실제 20개 TRAIN CPU 검증, 20,000개 B5 입력 순서, 전체 고유 카메라 원본 41,256개 해시, 실제 5개 도메인 CUDA 유한 gradient·사전 검사 중 가중치 업데이트 0회, 지도 모델 종료와 GPU0의 프로세스 0개·사용률 0%·정확한 UUID 확인이다. 합성 검사 수를 실제 기능 성공 수로 세지 않는다.

**실행 중:** 실제 GPU0 optimizer 학습이다. 처음 456회는 189.8초였다. 단순 환산 약 139분이고 저장·전체 평가 시간이 추가되므로 **학습 약 2~3시간과 추가 평가**로 예상한다. 확정 종료 시각·토큰 청구량·크레딧 잔액을 알 수는 없다.

**다음:** 고정 20,000회와 100,000개 입력 기록·101개 최종 가중치를 검증한다. 그 뒤 기존 74개 진단의 같은 수치 기준과 실패 14개를 유지하고 RIGHT 3개를 추가해 전체 77개를 비교한다. 이 77개는 알려진 오프라인 연구 진단이지 독립 주행 승인 수가 아니다. 새로운 최종 export의 실행기를 별도로 검토한 후, 로컬에서 같은 조건 좌우회전·직진 보호와 독립 경로를 실제 주행한다. 원본과 모든 실패 꼬리·차량 중심 화면을 남긴다.

**미구현 또는 미승인:** 정상 목적지 정차 학습 연결, 30km/h 확대, 교통규칙·ACC·회피·차선변경 등 추가 기능의 독립 검증이다. 자동 모델 교체·실차 구동은 하지 않는다. 다른 테스트의 CARLA·GPU·프로세스는 건드리지 않는다.

## 실행과 재현 근거

서버 학습 결과는 `/home/pro6000/personal/hwanhong/portable_e2e/runs/training/temporal_left_right_joint_20261002_v2`, 일반 파일 로그는 `runs/launches/temporal_left_right_joint_20261002_v2`에 보존한다. 이 문서는 마지막 확인 시점의 상태이며, 프로세스가 끝난 뒤에도 계속 실행 중이라는 뜻은 아니다.

- [ROOT 고정 학습 범위](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_fixed20k_GOOD6000_balanced_authorization.json)
- [실제 전체 TRAIN CPU 검증 원문](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_all20_TRAIN_CPU_preflight.json)
- [앞선 지도 전체 완료와 350장 독립 재검산](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_previous_MAPV3_completed.json)
- [새 분리 실행 ROOT 범위](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_detached_launch_authorization.json)
- [실제 실행 직전 GPU0 사전 검사](../../../../../../storage/experiments/training/2026-10-02/temporal_left_right_joint_dispatch_v2/ROOT_actual_detached_launch_preflight.json)
- [실제 소유 프로세스 영수증](../../../../../../storage/experiments/training/2026-10-02/temporal_left_right_joint_dispatch_v2/ROOT_actual_detached_launch_receipt.json)
- [실제 학습 업데이트와 CUDA 검사](../../../../../../storage/experiments/training/2026-10-02/temporal_left_right_joint_dispatch_v2/ROOT_actual_GPU_progress_1938.json)
- [이전 좌회전 오차 A/B와 전체 실제 화면](../24_actual_left_joint_lateral_lookahead_pair/OUTCOME.md)
- [정상 LEFT·RIGHT 자료와 실패 원자료 구분](../18_low_speed_turn_data_readiness/README.md)
