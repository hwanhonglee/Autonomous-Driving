# HH_261001 — Town07 방향 경사 A/B 실제 주행 비교

같은 TRAIN 직진 경로에서 미리 지정한 A 1회, B 1회를 각각 20초 실행했다. A는 W1과 모델 텐서가 같지만 새 물리 실행이다. 과거 W1 20초는 변동 폭을 보기 위한 별도 참고 기록이다.

![동일 경로의 차체 시작점 기준 실제 궤적과 속도](vehicle_centered_comparison.png)

![측정 속도, 선택된 원시 0.3초 속도 예측, 제동](speed_preview_brake.png)

| 항목 | A 새 실행 | B 새 실행 | 과거 W1 20초 |
| --- | ---: | ---: | ---: |
| 상태 / 6카메라 묶음 | 400 / 200 | 400 / 200 | 400 / 200 |
| 추론 / 학습 제어 ACK | 183 / 366 | 183 / 366 | 183 / 366 |
| 순 경로 진행 (m) | 25.35 | 25.05 | 29.76 |
| 마지막 경로 위치 / 전체 길이 (m) | 25.67 / 210.60 | 25.37 / 210.60 | 30.07 / 210.60 |
| 실측 절대 속력 최소/중앙/최대 (m/s, 400상태) | 0.00/1.42/2.33 | 0.00/1.43/2.33 | 0.00/1.81/2.33 |
| 학습 ACK가 다음 상태에 연결된 365개 중 abs(vx)≤0.5m/s (초) | 2.65 | 2.75 | 2.50 |
| 같은 365개 중 abs(vx)≤0.05m/s (초) | 1.85 | 1.90 | 1.55 |
| 첫 실측 정지 시각 (초) | 12.05 | 11.85 | 13.60 |
| 최저 20Hz 이산 vx 변화율 (m/s²) | -18.56 | -18.53 | -21.07 |
| 최대 중심 경로 오차 (m) | 0.121 | 0.127 | 0.143 |
| 추론 p95 (ms) | 31.45 | 32.94 | 29.13 |
| 50ms 추론 지연 초과 / 후보 runtime gate 실패 | 0 / 0 | 0 / 0 | 0 / 0 |
| 후보 runtime gate 통과 / 검사 (12×183) | 2196 / 2196 | 2196 / 2196 | 2196 / 2196 |
| 충돌 / 차선 침범 이벤트 | 0 / 0 | 0 / 0 | 0 / 0 |
| 차체 10cm inset 침범 (400) | 0 | 0 | 0 |
| bounded sweep 10cm inset 침범 (399) | 0 | 0 | 0 |
| bounded sweep inset 최소 안쪽 여유 (m) | 0.325 | 0.326 | 0.308 |

새 B−A 순 진행: -0.30m. 과거 W1 대비 A도 -4.41m 차이가 난다. 한 번씩의 물리 실행으로 학습 변경의 인과 효과를 단정할 수 없다.

### 원시 예측과 초반 제동

원시 선택 후보의 0.3초 속도는 추론 183회당 한 번만 집계했다. 실제 속도는 같은 추론 프레임의 차량 상태다. 제동은 제어기 명령이며 실측 감속과 구분한다.

| 측정값 | A | B | 과거 W1 |
| --- | ---: | ---: | ---: |
| 후보 1 첫 선택: 캡처 시각 (초) | 3.10 | 2.70 | 6.20 |
| 초반 10초 선택 0.3초 속도 평균 (m/s) | 1.866 | 1.765 | 2.494 |
| 초반 10초 선택 2.0초 속도 평균 (m/s) | 2.126 | 1.765 | 4.532 |
| 초반 10초 평균 스로틀 | 0.103 | 0.098 | 0.177 |
| 실측 vx>1.0인데 선택 예측≤0.5m/s | 0 | 0 | 0 |
| 그중 첫 학습 5초 | 0 | 0 | 0 |
| 실측 vx>1.0에서 제동>0.05 추론 | 3 | 1 | 2 |
| 첫 해당 제동: 캡처 시각 (초) | 15.90 | 18.20 | 11.20 |
| 첫 선택 0.3초 속도≤0.05: 시각 (초) | 12.10 | 12.00 | 13.60 |

첫 제동 사건과 선택 예측의 방향은 사건별로 다를 수 있다. 선택 후보 번호만으로 정지 뜻을 부여하지 않았고, 미선택 후보를 실제 주행으로 해석하지 않았다.

초반 10초(각 83회 추론)의 후보1 선택은 A/B/과거 W1 각각 69/73/26회다. 같은 구간 실제 양의 제동 명령은 1/0/11회다. B의 초반 진행 감소를 실제 제동 명령으로 설명할 수 없다. 선택 후보의 낮은 목표속도와 스로틀이 관측되지만 이 단일 물리 시험만으로 원인을 확정하지 않는다.

### 원본과 결과 해석

A/B 동일 경로 해시: `8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9`; 활성 OpenDRIVE 해시: `be730652769e993e27f748ba5793dba9e49d299205146a7a8db0bcd17af877bf`. A/B 각 95개 실행 소스 보관본의 핀 일치: `True`. 고정 이중 export binding 해시: `a6b674a471f2bec5c1c153d912a61fac5ecffae72ef797f6cbc9a86574efac8e`. A의 모델 텐서 해시는 과거 W1과 정확히 같다.

두 실행 모두 원래 수집기 상태는 `failed`이고 소유자 결과는 `DIAGNOSTIC_BUDGET_REACHED`다. 400상태/200카메라를 완전히 보관했으나 목표 경로는 완주하지 못했다. 소유 프로세스·포트·소스 정리는 두 팔 모두 통과했다. 학습 자료 편입, 모델 승격, 실차 제어 승인은 없다.

A/B에서 최저 이산 vx 변화율은 각각 약 −18.56/−18.53m/s²다. 이는 인접 20Hz 상태의 속도 차분이며 별도 승차감 기준을 통과했다는 뜻이 아니다. 초반 둔화와 급정지 순간의 원인을 제동 명령 하나로 환원하지 않는다.

### 추가 원본 대조 — 같은 위치에서 전문 에이전트는 통과

<!-- HH_261001 - Distinguish a spatial road defect from the learned low-speed control regime. -->

둘째 급감속은 A에서 경로25.13m·B에서25.03m로 거의 같은 위치였다. 두 실행 모두 약1.46→0m/s로 두 20Hz 틱 안에 떨어졌고 당시 저장된 브레이크는 A0.023, B0.011 수준이었다. 첫 정지는 A16.40m, B15.71m이며 브레이크는 각각0.0001/0.0013 수준이었다. 센서 충돌 이벤트는 0개였다. 이 숫자만으로 실제 타이어 접촉력, 제어 명령의 물리 적용 시각 또는 숨은 충돌을 증명할 수는 없다.

별도 [Town07 저속 BasicAgent 원본](../../../../../../storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1/episode/states.jsonl)은 같은 경로 근처를 통과했다. 둘째 정지 좌표에서 가장 가까운 전문 에이전트 상태는0.106m 떨어진 경로25.05m·속도2.484m/s·스로틀0.20·브레이크0이었다. 첫 정지들의 중간 좌표에서는0.055m 떨어진 경로16.09m·속도2.421m/s·스로틀0.061·브레이크0이었다. 인접 상태에서도 약2.42–2.49m/s로 이동했다. 따라서 **그 지도 좌표 자체가 모든 차량을 멈추게 한다는 가설은 지지되지 않는다.** 하지만 전문 에이전트의 속도·스로틀도 달라서 원인을 모델 출력, 추종 제어, 저속 물리 중 하나로 확정할 수 없다. 다음에는 동일 속도·동일 구간의 제어/물리 분리 계측이 필요하다.

차체 검사는 보관된 actor bounding box와 활성 OpenDRIVE를 이용한 20Hz 정적 다각형 및 두 상태 사이 bounded snapshot-linear sweep의 10cm 안쪽 차로 검사다. 연속 시간의 실제 물리 충돌 부재를 증명하지는 않는다. 충돌/차선 이벤트 0개와 차체 여유는 별도 근거다.

원본 전면 카메라 JPEG (복사하지 않은 보관 파일):

- A: [시작 frame 834](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_A_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00000834.jpg) · [중간 frame 954](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_A_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00000954.jpg) · [마지막 frame 1232](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_A_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00001232.jpg)
- B: [시작 frame 1273](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_B_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00001273.jpg) · [중간 frame 1393](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_B_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00001393.jpg) · [마지막 frame 1671](../../../../../../storage/experiments/validation/2026-10-01/directional_gradient_B_town07_sparse20_v1/episode.partial/images/CAM_FRONT/00001671.jpg)
- W1_history: [시작 frame 1396](../../../../../../storage/experiments/validation/2026-10-01/warmup_supervision_W1_warmup_anchor_off_town07_v1/episode.partial/images/CAM_FRONT/00001396.jpg) · [중간 frame 1516](../../../../../../storage/experiments/validation/2026-10-01/warmup_supervision_W1_warmup_anchor_off_town07_v1/episode.partial/images/CAM_FRONT/00001516.jpg) · [마지막 frame 1794](../../../../../../storage/experiments/validation/2026-10-01/warmup_supervision_W1_warmup_anchor_off_town07_v1/episode.partial/images/CAM_FRONT/00001794.jpg)

전체 원본: `comparison.json`의 각 run 경로에서 `episode.partial/`을 확인한다. 읽기 전용 집계이며 시뮬레이터 실행, 새 추론 또는 제어 변경이 없다.
