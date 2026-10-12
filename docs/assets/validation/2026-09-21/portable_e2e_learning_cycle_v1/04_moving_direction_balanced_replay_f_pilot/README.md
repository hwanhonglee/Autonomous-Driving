# 04 — 성능 우선 F 방향 균형 replay 진단

<!-- HH_260906 - Record the failed Pareto recovery before any feature-head expansion or confirmation-seed work. -->

## 결론

F는 우회전 성능을 C보다 개선했지만 직진과 전체 성능을 D보다 크게 악화시켰다. 따라서 최종
판정은 **`FAIL_SINGLE_SEED_DIAGNOSTIC`**, checkpoint 미승격이다. ACC·차선 변경·장애물 회피
등 신규 기능 head 추가와 3-seed 확증, export, 폐루프 주행은 계속 동결한다.

F는 PRO6000 물리 GPU 0에서 seed `20260906`으로 fresh fit했다. 모델 구조, 1,056,362개
parameter, loss, optimizer, TRAIN 1,475개는 D/E와 동일하게 유지했다. high-curvature 기준은
`max(abs(target_yaw_rad)) > 0.2`이고, `scenario_tags`의 exact `left/right`만 사용했다.
좌회전 254개는 replay하지 않고 우회전 73개의 부족분 181개/epoch만 보충하여 좌/우 노출을
254:254로 맞췄다.

- 20 epoch, 8,280 optimizer step, 총 33,120 exposure
- base 29,500 + right replay 3,620
- schedule 33,120행 SHA-256 `d340a10c42f2a733988b69ae6571ec3c37b7e8fa1601c43e6111192cd35c55b0`
- sampling plan SHA-256 `abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b`
- 같은 dataset index가 한 batch에서 충돌한 횟수 0
- TEST 접근 0, resume 0, retry/seed shopping 0, GPU 1 사용 0

## 고정 DEV 결과

아래 ADE/FDE/속도 MAE는 모두 6.4초 지평이며 낮을수록 좋다. 우회전 gate는 C, 직진과 전체
gate는 D의 정확한 수치를 상한으로 사용했다.

| 범위 | 기준 | 기준 ADE / FDE / 속도 | F ADE / FDE / 속도 | 결과 |
| --- | --- | --- | --- | --- |
| Town03 우회전 337 | C | 2.935 / 7.794 m / 1.108 m/s | **2.867 / 7.177 m / 0.917 m/s** | 3/3 PASS |
| Town03 직진 328 | D | 3.765 / 11.027 m / 1.788 m/s | **7.210 / 17.484 m / 2.772 m/s** | 0/3 FAIL |
| 전체 VAL 665 | D | 3.563 / 10.354 m / 1.635 m/s | **5.009 / 12.261 m / 1.832 m/s** | 0/3 FAIL |
| geometry | C/D | selected/any/all ≥664/665 | 664/665, 664/665, 664/665 | PASS |

F는 C 대비 우회전 ADE `-2.31%`, FDE `-7.92%`, 속도 MAE `-17.18%`로 좋아졌다. 그러나 D
대비 직진 ADE `+91.51%`, FDE `+58.55%`, 속도 MAE `+55.06%`로 퇴행했다. 전체도 D 대비
ADE `+40.58%`, FDE `+18.42%`, 속도 MAE `+12.06%` 나빠졌다. F는 E보다 우회전은 좋지만
직진 ADE가 `+28.55%`, 전체 ADE가 `+10.05%` 나쁘다.

이는 우회전 데이터 부족이 실제 문제 중 하나였음을 보여 주지만, 같은 Town01 우회전 73개를
반복해 균형을 맞추면 직진 표현을 희생한다는 뜻이다. F의 우회전 숫자만 골라 모델을 채택하지
않는다.

후보 생성 자체는 유지됐다. 직진 oracle ADE는 D `0.544720 m`에서 F `0.536597 m`로 소폭
개선됐지만 selected ADE는 `3.764740→7.209749 m`로 무너졌다. 전체 oracle ADE도 D
`1.028224 m`, F `1.024447 m`로 사실상 같은 반면 candidate regret은
`6.839871→9.186236`으로 증가했다. 따라서 즉시 병목은 trajectory 후보 생성보다
candidate selector/ranking이다. 새 기능 head를 넣거나 core를 키우기 전에 이 선택 실패부터
분리해서 고친다.

## 시각 자료

- [C↔F Town03 우회전 같은-frame 비교](galleries/town03_right_turn_dev.md)
- [D↔F Town03 직진 같은-frame 비교](galleries/town03_straight_dev.md)
- [C/D/E/F 전체 수치와 증감률](evidence/comparison_C_D_E_F.md)
- [원격→로컬 exact 전달 검증](evidence/transfer_verification.json)
- [실행 후 provenance 검증](evidence/postrun_provenance_verification.json)

모든 PNG는 저장된 고정 DEV 입력에 대한 offline 예측이다. `900×700` 화면 중앙 `(450,350)`에
ego를 두고 route, expert, 전체 candidate, selected trajectory를 표시한다. 실제 CARLA 주행
화면, Autoware/RViz 화면, learned closed-loop 성공 증거 또는 GIF가 아니다.

## 다음 성능 회복 단계

oracle/selected 분리 결과에 따라 H selector 재보정을 먼저 수행한다. F의 encoder·fusion·후보
trajectory/stop 생성부를 모두 동결하고, 새 구조가 아닌 기존 두 Linear selector head 3,084개
parameter만 F weight에서 이어 학습한다. paired H0/H1은 batch 순서, 8,280 step, 33,120 exposure,
loss와 optimizer를 같게 유지하고, 추가 181개/epoch의 selector 학습 identity만 우회전 반복 대
전체 TRAIN 균등으로 바꾼다. Town03 DEV는 마지막 평가 한 번에만 쓰며 early stopping이나
checkpoint 선택에는 쓰지 않는다.

H가 Pareto gate를 회복하지 못하면 G는 replay 횟수를 더 늘리지 않는다. Town04/Town15 TEST와
Town03 DEV를 TRAIN에 섞지 않고, 기존 TRAIN 역할인 Town01·Town07·C-track의 서로 다른 물리
우회전 group에서 신규 BasicAgent episode 6개를 수집한다. 고정된 구조·loss·optimizer로 신규
고유 우회전 데이터 효과만 분리한다. 어느 단계든 C 우회전과 D 직진·전체 Pareto 기준을 동시에
통과해야만 3개 unseen seed로 넘어간다. 성능 기준선을 회복하기 전에는 기능 head를 추가하지
않는다.

## 경계

이 단계는 동일한 Town03 두 DEV episode를 반복 관찰한 single-seed open-loop 진단이다. 모든
Town, 30개 기능, 30/60 km/h 폐루프, 장애물 회피, ACC, 차선 변경, 실제 차량 성능을 검증하지
않았다. 모델 직접 제어 승인과 기능 head 추가 승인은 모두 0으로 유지한다.

[exact artifact](../../../../../../storage/experiments/training/2026-09-21/moving_direction_balanced_replay_f_pilot_v1/README.md) ·
[직전 E 단계](../03_moving_turn_replay_e_pilot/README.md) ·
[cycle 목차](../README.md)
