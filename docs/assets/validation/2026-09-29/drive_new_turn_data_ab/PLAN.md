# 신규 우회전 자료의 주행 모델 A/B 학습 설계

<!-- HH_260929 - Freeze a data-only driving comparison before any new turn episode is admitted. -->

**상태: `DESIGN_ONLY_NO_EXECUTION_AUTHORIZATION`.** 새 우회전 TRAIN episode는 아직 없다. Town12 정적 경로 조사, 소유 CARLA 원본 수집, 전체 QA, 통합 split 잠금이 끝나기 전에는 이 계획으로 GPU 학습을 시작하지 않는다. 이 문서 작성에는 CARLA·SSH·GPU를 사용하지 않았다.

<!-- HH_260929 - A static right-turn connector with a dynamic signal cannot be treated as signal-free training data. -->
후속 [Town12 교차로 3742 XML 분류](../drive_town12_right_static/README.md)는 정적 차체 기준으로 지지되는 후보의 접근부에 `StopLine2`와 동적 `Signal_3Light_Post01` 참조를 찾았다. 이는 신호등 상태나 법적 적용의 실시간 판정이 아니다. 해당 후보를 실제로 수집한다면 신호 상태·정지선·운행 정책을 사전에 고정하고 신호 준수 및 원본 기록을 검사해야 한다. **신호를 무시하는 BasicAgent 주행을 정상 우회전 학습 정답으로 간주하지 않는다.** 현재 정적 PASS 8개는 한 물리 장소의 경로 변형이고, 그중 production 차로 검사가 지원하는 것은 4개뿐이다.

## 해결하려는 측정 실패

[AH 최종 평가](../../2026-09-26/dual_model_recovery_ah/results/driving/summary.json)의 반복 관측 Town03 DEV 우회전 337개에서 6.4초 ADE/FDE/속도 MAE는 **3.523050 m / 10.194983 m / 1.485901 m/s**다. 사전 절대 상한 **2.935314 m / 7.793866 m / 1.107615 m/s**를 모두 넘었다. 우회전 최선 후보 ADE 1.324224 m와 실제 선택 ADE 사이에는 **2.198826 m**의 선택 gap이 있다. 최선 후보는 미래 정답을 본 분석용 하한이다. 같은 DEV 직진 328개 ADE는 0.364104 m라 회전 개선 과정에서 보존해야 한다. AH는 절대 지표 6/9이고 학습 모델 폐루프 승인 기능은 9개 영역·30개 중 **0/30**이다.

가설은 한 물리 우회전 TRAIN episode만 되풀이한 자료 구성이 회전 후보 선택과 속도 일반화를 제한한다는 것이다. 이 가설을 검증할 때 모델 구조·손실·안전 한계를 함께 바꾸지 않는다. 과거 [선택기 단독 J 실험](../../2026-09-21/portable_e2e_learning_cycle_v1/07_moving_causal_route_ranker_j/README.md)은 고정 성능 지표 4/9로 실패했다.

## 고정 입력과 출처

| 항목 | 값과 역할 |
| --- | --- |
| 동일 시작 가중치 | 로컬 [K1 최종 체크포인트](../../../../../storage/experiments/training/2026-09-22/moving_recovery_k_v1/K1_current_plus_successful4/training/final_checkpoint.pt), 직접 확인한 SHA256 `8ec01ccf83583b09a557e643e687a9ae683a65b35a33f5a14cefc880f323d832`. A/B가 이 정확한 model state에서 각각 새 optimizer로 시작한다. |
| 기존 비교 모델 | 로컬 [AH 최종 체크포인트](../../../../../storage/experiments/training/2026-09-26/dual_recovery_ah/driving/final_checkpoint.pt), 직접 확인한 SHA256 `4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91`. 현재 미채택 모델이며 A/B의 출발 가중치는 아니다. |
| A의 기존 TRAIN | 원격 개인 자료 `/home/pro6000/personal/hwanhong/dataset/prepared/moving-recovery-k1-20260922-v1`: [동결 K 계획](../../../../../storage/experiments/training/2026-09-22/moving_recovery_k_v1/plan.json)에 기록된 manifest SHA256 `8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957`, 2,787개. 기존 우회전은 Town01 `right_s0001_p00` 534개다. 원격 manifest 바이트와 loader fingerprint는 실행 전 다시 검사한다. |
| 고정 연구 DEV | 같은 K 계획에 기록된 Town03 `right_s0000_p00` 337개와 `straight_s0000_p07` 328개, 합계 665개. 연구 DEV manifest SHA256 `70fceaf0a6fea1b6ac455154bdc7b1113cfef15abc1a26ca7327ac2a186799e6`. 기존 [연구용 4-episode manifest](../../../../../storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json)는 로컬 직접 확인 SHA256 `c73900cedce870a0f55663fde08b3a64b90fae139506fbe2099fdc8bf991c4a9`다. |
| AH 평가 원본 | [summary.json](../../2026-09-26/dual_model_recovery_ah/results/driving/summary.json), 직접 확인한 SHA256 `d19681d74fac425fc726b100c4b713a84b9c6ee01cc9be7ff56a80c07faabe28`. |

현재 로컬 참조 소스의 **전체 SHA256**은 `run_driving_near_horizon_ah.py` `2e80960f368266edcc322b4295f770b9c40b6af7d8a5d9e5083cb4e21764d632`, `dual_recovery_objectives.py` `d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15`, `model.py` `fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e`, `train.py` `c5ee8ab4533c3feedb0c60fe270fcb4a5e42247e186cfc8fe1ac9ec2fb7e0247`, `losses.py` `cdf37262024435c3a04888bf816c24db1b769b87034f08e1541c02d22da80973`, `evaluate.py` `9d30d3be7e114ee80f31355b2c2780c840ef48328e358935ec33944dc8d14887`, `torch_dataset.py` `a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9`, `contract.py` `e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09`, 모델 설정 JSON `29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602`다. 이는 **현재 로컬 파일**의 핀이다. 새 자료 sampler를 구현한 실행기와 원격 전달본의 전체 source manifest는 실행 전에 별도로 고정·대조해야 한다. 기존 AH 실행기가 새 자료를 읽을 수 있다고 가정하지 않는다.

## 새 자료 편입 전 필수 조건

1. [Town12 최소 정적 조사 계획](../drive_other_site_right_static/NEXT.md)의 비-TEST 물리 우회전이 단일 RIGHT, 30–120 m 회전 창, 최소 146 m 경로, 신호 진입부, 명목 Prius 전체 차체와 보수적 sweep의 의도 차로 10 cm inset을 통과한다. `speed_30kph` 이름은 경로 **형상** profile이며 실제 30 km/h 주행 승인이 아니다.
2. 통합 지도/주행 원장에 Town12를 TRAIN으로 잠그고 기존 체크포인트·route·source session·물리 교차로와의 중복을 검사한다. 후보 route ID, endpoint, live OpenDRIVE SHA, actor bbox, 9 km/h 기준 속도 계획, seed, rig와 수집 코드를 원본 실행 전에 고정한다. packaged XODR 결과를 live 지도 일치 증명으로 대체하지 않는다.
3. 소유 BasicAgent 원본의 여섯 카메라·10 Hz 영상, 20 Hz 상태/명령, 보정·시간 동기·ACK·warmup/tail을 보존한다. 연속 30초 실제 `>0.5 m/s` 이동, 진입·회전·이탈 이동 표본, 각 선택 anchor의 **64점/6.4초** 미래, 원래 9개 운동 한계, 충돌·차선 침범과 별도로 **실제 차체 전체 및 sweep의 의도 차로 포함**, 단조 progress와 목표 ≤2.5 m를 전체 원본으로 검사한다. 모든 미래 창의 원래 한계 위반이 0이어야 한다.
4. 실패 창 삭제, 저속 정지 표본의 이동 재명명, target clipping, 사후 속도·기준·기준점 변경은 금지한다. 원본 QA와 새 TRAIN manifest의 해시·split 역할·표본 수를 고정한 뒤에만 아래 A/B를 별도로 실행 계획으로 승인한다. Town01 p02 정지 자료, AN/AO 실패 원본, Town03 DEV, 예약/TEST 자료는 B에 넣지 않는다.

## 편입이 승인될 경우의 단일변수 A/B

<!-- HH_260929 - Match optimizer, direction slots, and old-source sampling while changing only right-turn data exposure. -->

| 조건 | A: 기존 자료 | B: 새 우회전 자료 |
| --- | --- | --- |
| 시작 상태와 학습 | 동일 K1 가중치, 새 AdamW optimizer, 동일 seed `20260906`, 전체 약 106만 파라미터, 15,000 step, batch 4, LR `3e-5`, 2초 보조 손실 가중치 `0.5`와 AH의 원래 6.4초 손실 | A와 동일 |
| 배치 방향 슬롯 | 매 batch `직진 2·좌회전 1·우회전 1`, 같은 그룹/표본 순서 seed | A와 동일 방향 슬롯 및 직진·좌회전 표본 ID 순서 |
| 우회전 슬롯 | 15,000개 모두 기존 Town01 우회전에서 선택 | 사전 고정한 우회전 슬롯의 25%인 3,750개만 QA 승인된 새 Town12 물리 route에서 선택; 나머지 11,250개는 기존 Town01에서 선택. 같은 슬롯의 선택 규칙·seed·샘플 중복 빈도를 기록한다. |
| 평가 | 최종 체크포인트만 고정 Town03 연구 DEV 665개에 한 번 평가 | A와 같은 DEV, preprocessing·metric code·순서 |

기존 K1 TRAIN 2,787개를 B에서 삭제하지 않는다. 우회전 25%는 독립 새 자료의 **노출 비율**이지 25%의 새 기능 승인이나 목표 성능을 뜻하지 않는다. 구현 전에는 새 자료가 실제로 몇 개의 합격 anchor를 제공하는지 알 수 없다. 두 arm의 소스, 입력 manifest, 초기 model state, optimizer reset, 방향별 slot schedule, 이미지 SHA 검사, 물리 route group을 기계 판독형 수령증에 기록하고, 비교 조건이 하나라도 어긋나면 paired A/B 판정을 내리지 않는다. 자동 재시작·중간 DEV 선택·checkpoint 승격은 하지 않는다.

## 사전 평가 기준과 사용량

1. A/B 모두 우회전·직진·전체 DEV의 ADE/FDE/속도 MAE **9개**, 우회전 선택 gap과 oracle ADE, 후보 분포 및 1·3·6.4초 오차를 보고한다. B가 A보다 우회전 세 오차와 선택 gap을 모두 낮추고, 직진 세 오차를 높이지 않는지를 우선 판정한다. 후보 oracle은 원인 분석일 뿐 운용 결과가 아니다.
2. 별도 절대 관문은 기존 [K 계획](../../../../../storage/experiments/training/2026-09-22/moving_recovery_k_v1/plan.json)의 9개 고정 상한 **9/9**, 후보/선택 경로 형상 통과 **3/3**, 비퇴행과 전체 자료 QA다. 우회전 ADE/FDE/속도 상한은 각각 `2.9353143748791353 m`, `7.793866114022117 m`, `1.1076154233086357 m/s`로 유지한다. 사후 threshold 조정은 하지 않는다.
3. PRO6000의 **물리 GPU 0 하나만** 유휴 확인·lease 확보 후 `/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312` 안에서 사용한다. GPU 1, 시스템 설치, 다른 사용자의 작업 중단은 없다. AH 실측 15,000 step은 29분 41초였으므로 이 A/B 학습 두 회는 **약 60–80분 GPU 시간**으로 추정한다. 새 자료 검사·전송·평가와 오류 해결 시간은 별도이며 추정치를 실행 약속으로 보지 않는다.

Town03은 여러 번 본 연구 DEV이고 지도 계열의 과거 TRAIN 노출도 있다. **새 독립 DEV는 0개**다. A/B 한 seed의 9/9 통과도 배포·Autoware·실차·10 Hz·30/60 km/h·기능 승인 근거가 아니다. 이후 별도 독립 DEV와 반복 seed, 미사용 TEST, 소유 CARLA shadow와 제한 폐루프 검증이 필요하다. 현재 결론은 `TRAINING_NOT_STARTED`, `NEW_TURN_DATA_NOT_ADMITTED`, `PROMOTION_FORBIDDEN`이다.
