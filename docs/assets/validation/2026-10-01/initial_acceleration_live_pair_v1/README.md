# 주행 A/B — 추가 학습으로 정지 문제가 해결되지는 않았다

<!-- HH_261001 - Report the full failed closed-loop pair, including low-speed chatter and delayed cleanup evidence. -->

**결론: 두 새 모델 모두 채택하지 않는다.** 기존 AH를 같은 조건에서 시험했을 때 약 15.22m를 진행했지만, 추가 3,000 step을 학습한 A와 B는 더 일찍 주행이 무너졌다. B가 잠깐씩 움직였다는 사실을 정지 문제 해결로 해석하면 안 된다.

## 실제로 무엇을 비교했나

PRO6000 물리 GPU 0에서 같은 AH 가중치·같은 2,787개 TRAIN anchor·같은 12,000회 표본 노출로 각각 3,000 step 학습했다. A는 기존 손실함수, B는 초기 가속도 보조 손실을 추가했다. 두 최종 가중치를 저장한 뒤에만 고정 DEV 평가를 수행했다. 학습 시간 합계 약 12분 39초이며 평가 시간은 별도다.

로컬 CARLA에서 두 최종 모델을 **같은 Town07 TRAIN 경로·20초·9km/h 상한·10Hz 추론·같은 추종기**로 각각 한 번 실행했다. 전문가는 초기 출발 11개 명령만 담당했고, 이후 각 모델이 366개 명령을 실제 ACK로 전달했다. 전체 경로 완주를 위한 시간 예산이 아니므로 종점 도달 시험으로 세지 않는다.

| 측정 | 기존 AH 참고 | A: 기존 방식 추가 학습 | B: 초기 가속도 보조 손실 |
| --- | ---: | ---: | ---: |
| 실제 진행 거리 | 15.22m | 3.09m | 5.31m |
| 모델 제어 상태 18.25초 중 속력 ≤0.5m/s | 별도 기준으로 미집계 | 16.40초 | 15.10초 |
| 역방향 vx <−0.05m/s | 별도 기준으로 미집계 | 0초 | 4.85초 |
| 추론 p95 | 31.90ms | 29.53ms | 26.75ms |
| 충돌/차선 이벤트 | 0 / 0 | 0 / 0 | 0 / 0 |
| 새 모델 채택 | 원본 보존 | 하지 않음 | 하지 않음 |

충돌·차선 이벤트 0개는 전체 차체 침범 검사가 통과했다는 뜻이 아니다. 이 경로는 이미 알려진 TRAIN 경로이고, 한 번씩만 실행했다. 반복 재현성·독립 지도 일반화·실차 안전 승인을 의미하지 않는다.

## B의 ‘마지막 정지 시간 0.05초’가 좋은 결과가 아닌 이유

기존 보고서는 마지막부터 연속으로 `abs(vx) ≤ 0.05m/s`인 시간을 계산한다. B는 아주 작은 전후 속도 진동 때문에 이 연속 조건이 자주 끊겼다. 전체 기록을 추가로 확인하면 마지막 **15.10초 내내 속력 0.5m/s 이하**였다. 따라서 기존 숫자는 그대로 보존하되, [별도 저속/진동 감사](B_preclamp_acceleration/low_speed_audit.json)를 함께 해석한다. 새 설명 기준으로 원래 합격 기준을 바꾸지 않았다.

## 확인한 원인과 다음 조치

- A의 저속 추론 164회 중 157회: 기존 궤적 런타임 검사를 통과하고 0.3초 후 속도가 0.5m/s를 넘는 DRIVE 후보가 있었지만, 선택기는 0.05m/s 이하의 후보를 골랐다. **후보 선택 문제를 의심할 직접 근거**다. 다른 후보가 장애물·차선까지 안전하거나 전문가 정답이라는 뜻은 아니다.
- B의 저속 추론 151회: 위 조건의 움직이는 DRIVE 후보가 하나도 없었다. 선택기뿐 아니라 생성된 궤적도 약해졌다.
- TRAIN 고정 출발 probe에서도 선택 후보와 전문가 미래에 가장 가까운 후보가 불일치한다. 다음 실험은 충분한 정지/출발 자료를 확보하면서, **기존 궤적 생성부를 고정한 선택기 보정**이 이 불일치를 줄이는지 확인하는 방향이다. 무조건 빠른 후보를 선택하도록 바꾸지 않는다.
- 고정 DEV의 우회전 ADE/FDE/속도 오차도 B가 A보다 나빴다. 품질 미달을 숨기거나 기준을 낮추지 않는다.

## 화면과 원본

아래 GIF는 유리한 일부만 잘라낸 것이 아니라 **전체 20초**를 5Hz로 표시한다. 6카메라·차량 중심 경로·12개 후보·실제 속도·추종기 목표 속도가 함께 나온다.

- A: [전체 GIF](A_ah_continuation/vehicle_centered_full.gif), [마지막 PNG](A_ah_continuation/final.png), [수치](A_ah_continuation/analysis.json), [저속 감사](A_ah_continuation/low_speed_audit.json).
- B: [전체 GIF](B_preclamp_acceleration/vehicle_centered_full.gif), [마지막 PNG](B_preclamp_acceleration/final.png), [수치](B_preclamp_acceleration/analysis.json), [저속 감사](B_preclamp_acceleration/low_speed_audit.json).
- [원래 비교 보고서](pair_report.json), [학습 결과 원본](../../../../../storage/experiments/training/2026-10-01/ah_initial_acceleration_ab_20261001_v1/summary.json).

## 종료 확인의 별도 실패

A는 즉시 종료 검사를 통과했다. B는 촬영·소스 동일성 검사를 마쳤지만 즉시 독점 포트 바인드 검사에서 `Err98`이 나와 원래 `OWNER_FAILURE`를 보존했다. 뒤이은 읽기 전용 조사에서 프로세스/리스너는 없고 TCP TIME-WAIT가 남은 것을 확인했으며, 이후 같은 원래 바인드 검사까지 통과했다. [후속 종료 영수증](../../../../../storage/experiments/validation/2026-10-01/initial_acceleration_B_preclamp_acceleration_town07_v1_post_release_v1/release_observation.json)은 원래 실패를 덮어쓰지 않는다. 따라서 원래 비교 보고서의 `clean_owned_runs=false`도 유지한다.

현재 이 시험으로 새로 승인된 기능은 **0개**다. 실패 원인 분리·실제 A/B 가중치·원본 주행 증거를 확보한 단계다.
