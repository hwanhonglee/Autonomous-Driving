# 생성기 고정 선택기 A/B — 주행·정지 후보를 고르는 부분만 학습

<!-- HH_260906 - Declare the fixed six-head experiment and actual start without claiming completed training or deployment. -->

2026-09-09 **05:34:23–05:40:53 KST 실제 실행 완료**. 기존 10-epoch STOPMIX 3개 모델의
카메라 인코더·경로 생성기·12개 후보는 고정하고, 후보를 선택하는 점수 헤드만 새로 학습합니다.
전체 모델 학습 6회가 아니라 **선택기 전용 학습 6회**입니다. 종료·무결성 검사 오류는 0개이고,
9,240 step·36,930회 표본 노출을 완료했습니다. 모델은 미채택이며 상세 자료 수집을 진행 중입니다.

| 고정 조건 | 내용 |
|---|---|
| 부모 모델 | seed 20260903 / 20260904 / 20260905, 각각 2,870 update |
| 비교 A | 기존 형태의 별도 Linear(256,6) 두 개, 3,084 parameters, 새 초기화 |
| 비교 B | context와 후보 XY·속도를 함께 보는 공유 MLP 448→256→1, 115,201 parameters, 새 초기화 |
| 학습량 | 각 1,540 step·6,155회 표본 노출, batch 4, 동일 seed별 순서 |
| 정답·손실 | 원래 composite cost의 hard argmin과 CE × 0.1; 동률·가중치 변경 없음 |
| 데이터 | 기존 TRAIN 1,147개 / 개발 VAL 337개, episode 분리 유지 |
| 평가 | 원래 선택기 및 A/B의 전체 TRAIN·VAL, 동일 VAL 12개 시점 PNG |
| 실행 환경 | 원격 개인 venv·physical GPU 0만, 고정 b478 저장소는 읽기 전용 |
| 제한 | 실시간·closed-loop·실차 검증 아님, 새 데이터 승인·자동 승격 없음 |

A/B는 파라미터 수도 다르므로 차이가 나타나도 **후보 정보의 효과와 모델 크기의 효과가
분리됐다고 해석하지 않습니다.** 미래 정답으로 나누는 정지/이동 그룹은 사후 진단이며
모델 입력에 넣지 않습니다. 원래 두 헤드의 logits를 비트 단위로 재현한 뒤에만 학습합니다.

계획 SHA-256: `a11c5c94e94077fbbd6eb3d270fe54e62ccec5590082c28b9f76c8552479cdfa`.
선언 시각 05:31:32 KST, 실제 실행 전 고정했습니다. worker `469bf474…`, core `4716c22d…`는
local commit `cf3bfc2`에 보존했고, 원격 b478 소스를 덮어쓰지 않고 개인 benchmarks에서 실행합니다.

첫 실행 요청(05:33:08 KST)은 잘못 지정한 `runs/research` 경로가 소유 helper의 허용 범위를
벗어나 **GPU lease·데이터 로딩·optimizer 생성 전에 거절**됐습니다. 두 번째 launcher는
허용된 `runs/diagnostics` 경로를 사용합니다. 실험 소스·계획·모델·학습 조건은 동일하며,
첫 오류 로그·실행/종료 receipt도 별도로 보존합니다. 이를 실패한 학습 1회로 세지 않습니다.

05:35 KST: 첫 A 선택기의 실제 step 1,540·표본 노출 6,155를 확인했습니다.
05:40:53 KST: 3개 부모 × 2개 선택기 학습과 전체 TRAIN·VAL 진단이 종료됐습니다.
후보 인지 B는 세 seed의 VAL 평균 경로 오차가 원래 모델보다 줄었지만, TRAIN의 정지 유지
118개에서 STOP 선택은 모두 0개로 퇴행했습니다. VAL에는 정지 유지 표본이 0개이므로
VAL 평균만으로 정지 성능이 좋아졌다고 결론내릴 수 없습니다. 전체 결과·그림·원본 해시를 게시합니다.

[앞선 TRAIN oracle 진단](../09_train_oracle_diagnosis/README.md) ·
[부모 10-epoch 이어학습](../08_stopmix_ten_epoch_continuation/README.md) ·
[야간 작업 전체](../README.md)
