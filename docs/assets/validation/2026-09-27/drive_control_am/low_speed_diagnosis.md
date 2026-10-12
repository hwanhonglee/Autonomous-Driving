# AK·AL·AM 저속 곡률 원본 대조

<!-- HH_260906 - Distinguish measured low-speed XY geometry from body yaw, steering, and overlapping future windows. -->

**AM의 우회전 본구간은 기존 곡률 한계를 통과했지만, 출발 3개·목표 근처 27개의 실제 100 ms 시점에서 미래 XY 곡률이 초과됐습니다.** 원본 20 Hz 위치·yaw·휠 조향과 10 Hz 미래 감사 결과를 같은 시점에 놓아 보니, 목표 근처 27개 중 **22개는 해당 시점을 미래 궤적의 첫 점으로 삼은 창 하나에서만 실패**했습니다. 나머지 5개 시점은 여러 중첩 창에서도 실패합니다. 따라서 모든 저속 실패를 하나의 제어 문제나 단순한 보간 오류로 묶을 수 없습니다.

| 동일 Town01 p03 우회전 | AK B | AL | AM |
| --- | ---: | ---: | ---: |
| 출발 부근 곡률 초과 실제 시점 | 2 | 5 | 3 |
| 실제 회전 부근 곡률 초과 실제 시점 | 4 | 4 | **0** |
| 목표 근처 곡률 초과 실제 시점 | 3 | 17 | 27 |
| 그중 미래 첫 점에서만 초과 | 1 | 16 | **22** |
| 그중 여러 창에서 반복 초과 | 2 | 1 | **5** |
| 전체 곡률 초과 실제 시점 | 9 | 26 | **30** |

여기서 ‘실제 시점’은 서로 겹치는 6.4초 미래 창의 `episode_relative_100ms_tick`을 중복 제거한 수입니다. 원본 감사의 위반 *시작점* 수 191/191/151이나 위반 *단계* 수 487/551/505와 같은 의미가 아닙니다. AM의 회전 구간 2 m/s 이상 곡률 초과 0개는 [사전 선언](PLAN.md)의 국소 목표를 충족했지만, 위 전체 30개 때문에 기존 **전체 미래 궤적 계약은 실패**합니다.

## 무엇을 비교했나

[미래 감사 코드](../../../../../tools/e2e/audit_portable_target_feasibility.py)는 매 100 ms의 두 XY 위치로 이동 방향을 구하고, 직전 이동 방향과의 각도 차이를 **이번 이동 거리로 나눠** 곡률을 계산합니다. 이동 거리가 0.0001 m보다 커야 검사합니다. 각 6.4초 창의 첫 점에서는 직전 이동 방향을 차량의 현재 전방인 0 rad로 시작합니다. 그러므로 첫 점 하나만 실패하면, 그 100 ms의 위치 이동 방향이 현재 차량 yaw에서 조금 벗어난 것일 수 있습니다. 연속된 두 이동 구간에서 급하게 방향이 바뀌었다는 증거로 바로 해석할 수 없습니다. 하지만 현재 모델의 시작 상태와 목표 경로의 연결이 맞지 않는다는 **실제 학습 계약 문제**로 남습니다.

아래 표의 `ΔXY/100 ms`는 원본 [AK](../../../../../storage/experiments/training/2026-09-27/drive_control_ak/b_coast_27_hold/episode/states.jsonl)·[AL](../../../../../storage/experiments/training/2026-09-27/drive_control_al/rear_axle/episode/states.jsonl)·[AM](../../../../../storage/experiments/training/2026-09-27/drive_control_am/lateral_kp_170/episode/states.jsonl)에서 두 20 Hz 간격을 합쳐 직접 확인한 값입니다. `yaw 변화/거리`는 같은 두 원본 상태의 몸체 yaw 차이를 거리로 나눈 **별도 진단값**이며 XY 경로 곡률이나 물리적 타이어 반경의 대체 검사가 아닙니다.

| AM 대표 지점 | 원본 상태·속도 | ΔXY/100 ms | 미래 XY 곡률 | 몸체 yaw 변화/거리 | 조향·해석 |
| --- | --- | ---: | ---: | ---: | --- |
| 출발 tick 53 | frame 1900, 0.212 m/s | 9.10 mm | 4.269/m | 0.0449/m | 휠 −0.082 rad. 정지 전부터 조향 입력이 있어 실제 출발 조향도 관여할 수 있음. |
| 출발 tick 59 | frame 1912, 0.527 m/s | 55.59 mm | 0.2018/m | 0.0234/m | 휠 −0.075 rad. 한계를 근소하게 넘는 위치 방향 변화. |
| 목표 tick 527 | frame 2848, 0.155 m/s | 16.89 mm | 0.215–0.389/m | 0.00074/m | 64개 중첩 창에서 반복. 휠 −0.0032 rad, yaw rate −0.0052 rad/s. |
| 목표 tick 549 | frame 2892, 0.109 m/s | 12.31 mm | 0.2925/m | 0.00101/m | **첫 미래 점 하나만** 초과. 약 0.206°의 이동 방향 차이가 작은 거리로 나뉨. |
| 제동 직후 tick 556 | frame 2906, 약 0 m/s | **1.63 mm** | **110.45–112.74/m** | 0.1008/m | 64개 중첩 창에서 반복. 당시 휠·yaw rate는 0. |

AM 목표 접근의 초과 시점에서 100 ms 이동량은 **1.63–18.69 mm**, 실측 속도는 **0–0.173 m/s**입니다. 해당 시점의 몸체 yaw 변화/거리는 최대 **0.1008/m**로 XY 곡률 한계 0.2/m보다 작습니다. 정상적인 감속 부분의 휠 조향은 최대 약 **0.00466 rad**였습니다. 특히 원본 frame 2904에서 0.09924 m/s를 관측한 뒤 **다음 제어 명령**에 저속 유지 `brake=1`이 설정됐습니다. frame 2905는 0.00011 m/s, frame 2906은 약 0.00002 m/s입니다. 이 100 ms의 `base_link` 이동은 x 약 −0.295 mm, y 약 −1.602 mm이고 몸체 yaw 변화는 약 0.0094°입니다. 큰 `112.74/m` 수치는 **작은 위치 이동의 방향 변화/1.63 mm**이며 큰 몸체 선회 관측값은 아닙니다.

목표 직전 frame 2901–2904에서는 50 ms마다 x 이동이 대략 −0.064∼+0.081 mm로 번갈아 나타나고, y는 매번 약 −5.5∼−5.9 mm 전진합니다. 이 크기의 횡방향 변화는 100 ms의 전체 이동이 약 12 mm일 때 계산한 이동 방향에 영향을 줍니다. 출발에서는 바퀴가 이미 약 −0.08 rad 돌아가 있었고, 첫 100 ms 이동 거리가 9 mm였으므로 실제 조향, 초기 위치 흔들림, 차량 기준점 이동이 각각 얼마나 기여했는지 이 기록만으로 분해할 수 없습니다.

10 Hz 미래 시각과 가장 가까운 원본 20 Hz 상태 시각의 차이는 **위반 시점 모두 96 ns 이하**였습니다. 미래 생성기는 계약상 연속 20 Hz 상태 사이 선형 보간 경로를 탔지만, 이 시간 차이로 인한 보간 자체가 여기의 0.2–112/m 초과를 설명하지는 못합니다. 위치 방향, 100 ms 이동 거리, 각 미래 창의 **첫 점 초기화**가 더 직접적인 검사 대상입니다.

![AK·AL·AM 출발/목표 저속 곡률과 원본 100 ms 이동 거리](low_speed_curvature_diagnosis.png)

그림의 원은 한 창의 첫 점에서만 실패한 시점, ×는 여러 창에서 실패한 시점입니다. 오른쪽 검은 +는 AM의 별도 몸체 yaw 변화/거리 진단이며, 빨간선은 **바꾸지 않은** XY 곡률 한계 0.2/m입니다. [그림 생성 코드](../../../../../tools/e2e/plot_low_speed_curvature_am.py)는 봉인된 원본과 미래 감사 JSONL을 읽어 위반 시점만 중복 제거합니다.

## 다음 제어·측정 판단

<!-- HH_260906 - Keep original data and gates intact while separating source-frame, motion, and finite-displacement hypotheses. -->

1. 출발은 정지 상태에서 이미 생긴 조향과 첫 이동의 횡방향 흔들림을 따로 재현해 봅니다. 출발 위치·경로 첫 waypoint와 actor/body 프레임을 함께 계측해야 합니다.
2. 목표 접근은 동일 제어의 20 Hz `base_link`와 actor 중심 궤적, 연속 yaw·휠 조향을 추가로 비교해 측정 해상도·기준점 변환·실제 횡이동의 기여를 분리합니다. 특히 AM의 첫 점 전용 22개와 반복 5개를 따로 봅니다.
3. 원인이 확인되기 전에는 **실패 프레임 제거, 라벨 평활화, 곡률 한계 완화 또는 사후 학습 편입을 하지 않습니다.** 이 세 회차는 개발 진단이며 CARLA BasicAgent 제어 기록입니다. Portable E2E 주행 모델을 새로 학습하거나 제어에 사용한 결과가 아닙니다.

원본 미래 기록: [AK](final_audit/b_coast_27_hold.future.jsonl.gz) · [AL](final_audit/rear_axle.future.jsonl.gz) · [AM](final_audit/lateral_kp_170.future.jsonl.gz). [전체 감사 요약](final_audit/summary.json).

<!-- HH_260906 - Bind this comparison to the unchanged original state and audit-source bytes. -->

재현용 SHA256: 세 회차의 `route.json`은 모두 `3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a`입니다. `states.jsonl`은 AK `1014a8d612a60b32f329b0f30fdbe6e9bd22fc612eb7caa330278a9417e4ad70`, AL `6ebb4d8b15414a0d30b934f4f3168222148e5b987954aebf2e0a1df4e9f44bf2`, AM `7ff801eb56ea39e79568bf99b4e91213e1cdbdb4114fdc9fd297c5daffe1c89f`입니다. 미래 생성 감사 소스 `audit_carla_raw_pre_admission.py`는 `0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00`, 곡률 검사 소스 `audit_portable_target_feasibility.py`는 `8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894`입니다. AM [소유 실행 결과](../../../../../storage/experiments/training/2026-09-27/drive_control_am/lateral_kp_170/owner_result.json)는 수집 소스와 설치된 CARLA 제어 소스가 실행 전후 같았다고 기록합니다.
