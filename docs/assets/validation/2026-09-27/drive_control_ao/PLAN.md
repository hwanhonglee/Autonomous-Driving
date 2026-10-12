# AO 사전 선언 — AN 기준 출발 정상 조향 한 변수 시험

<!-- HH_260906 - Freeze the launch-only normal-steering guard before any owned CARLA execution. -->

**상태: 코드·단위 검사 준비 단계, CARLA 미실행.** 대조군은 봉인된 [AN](../drive_control_an/README.md) Town01 p03 우회전 9 km/h BasicAgent 주행이고, 후보 AO는 AN의 모든 경로·환경·rear-axle 제어·`K_P=1.70`·종방향 coast/정지 정책·목표 근처 0.20 m/s 조향 게이트를 그대로 둔다. **유일한 추가 요인은 출발 1 m에서 측정 속도가 0.20 m/s 미만일 때 정상 조향 출력을 0으로 보류하는 것**이다. Portable E2E 학습 모델의 운전이나 차량 승인이 아니다.

원본에서 마지막 `stationary_warmup` 프레임의 **첫 BasicAgent 명령**은 속도 0인데 `steer≈+0.0681`이고, 다음 첫 `driving` 관측부터 앞바퀴 각은 약 `−0.0813 rad`다. 이후 속도 0인 약 35개 20 Hz 관측 동안 조향 명령은 `+0.0691`, 바퀴 각은 `−0.0824 rad`까지 유지된다. 첫 실제 이동 관측은 약 0.098 m/s이고 출발 미래 곡률 초과 tick 53·54·59는 AN과 AM에서 동일하다. 따라서 주행 phase가 된 *뒤*에만 게이트를 걸면 선행 조향 명령을 놓친다. AO는 마지막 준비→첫 주행 핸드오프의 정상 BasicAgent 출력까지 같은 관측 조건으로 검사한다.

## 고정한 한 변수와 보존 조건

<!-- HH_260906 - A physical near-start guard changes only non-braking output steering after hazard selection. -->

- 경로: Town01 `right_s0000_p03`, SHA256 `3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a`, 직렬화된 총 길이 `76.16005405260421 m`. 목표 9 km/h, ClearNoon, seed `2026092601`, 물리/카메라 20/10 Hz, Epic, 동일 6카메라·정지 및 `2.85 m` wheelbase.
- **AO 추가 조건:** 같은 물리 관측의 남은 경로 길이 `> 76.16005405260421 − 1.0 m`, 측정 속도 `<0.20 m/s`, `brake=0`, `hand_brake=False`, 정상 BasicAgent 제어 출력에만 `steer=0`. 차가 움직이기 전 마지막 준비 프레임의 *실제* 첫 BasicAgent 명령도 포함한다. 속도 `≥0.20 m/s` 또는 시작 1 m 밖에서는 즉시 기존 AN 출력을 유지한다. `throttle`, `brake`, PID 내부 상태, waypoint, route, 미래 라벨은 손대지 않는다.
- 적용 위치는 BasicAgent 위험 판단 이후, 기존 AN 목표 조향 처리 이전이다. 긴급 제동(`brake>0`)·정지 유지 제동·핸드브레이크는 AO가 수정하지 않으며, AN의 기존 목표 1 m 저속 조향은 그대로 실행한다. 설치된 CARLA BasicAgent/LocalPlanner/controller 소스 바이트를 실행 전후 봉인해야 한다.
- 기존 route/admission 검사, 속도·가감속·yaw·**전체 6.4초 XY 곡률 ≤0.2/m** 한계는 그대로 둔다. AO 출력은 개발 자료로만 기록하고 학습 TRAIN에 자동 추가하지 않는다.

## 사전 판정과 예상 위험

AN 출발의 서로 다른 100 ms 위반 tick 53·54·59가 줄어드는지 동일 코드로 본다. 동시에 출발 첫 5 m의 `base_link` 최대 중심선 오차는 **0.50 m 이하**, 회전 본구간 평균/최대 CTE는 **0.10/0.30 m 이하**, 2 m/s 이상 회전 곡률 위반 0, 목표 앞 1 m 정지·2초 유지, 충돌·차선 센서 0, 원래 전체 미래 속도·가감속·yaw 한계를 요구한다. 출발 기준점이 경로 중심에서 이미 약 **0.409 m** 벗어나 있으므로 조향 지연이 경로 오차를 늘리거나 0.20 m/s 해제 순간에 더 큰 급선회를 만들 수 있다. 이 경우 AO는 폐기한다. 차선 센서 0만으로 차체 전체 차로 포함을 주장하지 않는다.

AN의 목표 근처 **3개 반복 초과**는 AO 조작 범위 밖이다. 출발 3개가 없어져도 전체 미래 계약이 남으면 학습 자료로 승인하지 않는다. 처음부터 끝까지 원본 20 Hz 상태·카메라/명령 정렬·실제 10 Hz 미래 창·소스 해시와 차량 중앙 경로 PNG/GIF를 보존한다. 실패 창 삭제, 한계 완화, 재시도 후 좋은 결과만 선별하는 행위는 금지한다.
