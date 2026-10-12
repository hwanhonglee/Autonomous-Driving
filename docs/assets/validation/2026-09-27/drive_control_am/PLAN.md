# AM 사전 선언 — AL 기준점 정렬 후 횡방향 P 계수 한 변수 시험

<!-- HH_260906 - Predeclare one controller-only A/B and retain all data-admission gates. -->

목적: Town01 p03 우회전의 실제 고속 회전 구간 곡률 초과를 줄이되, AL에서 회복된 경로 추종을 잃지 않는지 확인한다. 이것은 **CARLA BasicAgent 정답 후보** 시험이며 Portable E2E 학습·모델 제어·차량 승인이 아니다.

- 대조군: `storage/experiments/training/2026-09-27/drive_control_al/rear_axle`의 AL, 횡방향 PID `K_P=1.95`.
- 후보: 정확히 같은 우회전 경로·seed·9 km/h·20/10 Hz·Epic·정지 정책·1.425 m rear-axle controller proxy에서 `--basic-agent-lateral-kp 1.70`만 변경한다. `K_I=0.05`, `K_D=0.2`, waypoint purge, 종방향 제어, 비상 제동, 물리 계약은 그대로 둔다.
- 관찰 기준: 회전 구간 route progress 42.25–58.67 m에서 실제 `base_link` 중심선 오차의 평균 절댓값 ≤0.10 m, 최대 ≤0.30 m를 넘기지 않고, 2 m/s 이상에서 0.2/m 곡률 초과 시점이 0개인지 본다. 전체 6.4초 미래 창의 곡률·yaw·속도·가감속 위반은 별도 원래 기준으로 모두 0이어야 학습 적합성 검토를 시작할 수 있다. 충돌·차선 센서 0도 필수이나 차량 몸체 전체 차로 포함을 증명하지는 못한다.
- 위 기준이 좋아도 개발 수집을 즉시 TRAIN으로 편입하지 않는다. 독립 경로·미사용 TEST·사전 우회전 admission 계약이 필요하다. 실패 창 삭제, 시간축 변경, 곡률 한계 완화, 눈으로 본 장면만 선별하는 사후 승인은 금지한다.

AL에서는 해당 회전의 실측 곡률 초과가 4개 독립 100 ms 시점, 최대 0.20586/m였고, 출발과 목표 부근 저속 위치 잡음까지 포함한 전체 초과는 26개 시점이었다. 그러므로 이번 P 계수 시험은 회전 원인을 분리하는 데 의미가 있지만 저속 문제를 자동으로 해결할 것으로 가정하지 않는다.
