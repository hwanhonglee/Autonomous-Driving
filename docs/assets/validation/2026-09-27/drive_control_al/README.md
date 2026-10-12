# 주행 개선 AL — 회전 경로 오차는 줄었지만 학습 조건은 미달

<!-- HH_260906 - Report measured CARLA expert behavior separately from learned-model performance and data admission. -->

Town01 p03 우회전에서 CARLA BasicAgent의 경로 추종 기준점을 차량 중앙에서 라벨의 `base_link`(뒷차축, 1.425 m 뒤)로 맞췄습니다. **회전 중 평균 경로 중심 오차는 0.397 m에서 0.061 m로 줄었습니다.** 목표 앞 정지와 2초 유지도 유지됐습니다. 그러나 실제 회전 곡률이 기존 0.2/m 한계를 네 시점에서 넘었으므로 이 회차는 **학습에 넣을 수 없습니다.**

| 같은 경로·조건 | AK B: 기존 기준점 | AL: 뒷차축 기준점 |
| --- | ---: | ---: |
| 회전 구간 평균 / 최대 경로 중심 오차 | 0.397 / 0.698 m | **0.061 / 0.154 m** |
| 회전 구간 경로 중심 오차 p95 | 0.690 m | **0.150 m** |
| 회전 구간 평균 속도 | 2.209 m/s | 2.208 m/s |
| 회전 중 곡률 초과 실제 100 ms 시점 | 4개, 최대 0.24246/m | **4개, 최대 0.20586/m** |
| yaw 범위 초과 실제 100 ms 시점 | 1개 | **0개** |
| 전체 곡률 초과 실제 100 ms 시점 | 9개 | **26개** |
| 목표 앞 정지·2초 유지 | 0.46864 m 앞, 성공 | 0.46866 m 앞, 성공 |
| 20 Hz 가속 최대 / 감속 최소 | +2.350 / −2.732 m/s² | +2.325 / −1.995 m/s² |
| 6.4초 미래 전체 시작점 | 571 | 575 |
| 6.4초 미래 곡률 위반 시작점 | 191 | 191 |
| 원본 JPEG 전수 디코딩 | 3,816장 | 3,840장 |
| 학습 자료 승인 | 아니요 | 아니요 |

회전 구간은 직렬화된 경로의 RIGHT 명령이 있는 route progress **42.25–58.67 m**입니다. 오차는 각 주행 상태의 `base_link` 위치에서 같은 경로 선분까지 잰 `route_cte_m`입니다. AK B는 141개, AL은 149개 주행 상태로 평균·최대·p95를 계산했습니다. 두 회차의 경로 원본 SHA256은 `3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a`로 같습니다.

전체 곡률 초과 시점이 9개에서 26개로 늘어난 것은 출발 저속 구간(2→5개)과 목표 근처 저속 구간(3→17개)의 영향이 큽니다. AL의 회전 중 초과는 여전히 **4개**이며 최고값은 낮아졌습니다. 저속의 위치 차분으로 계산한 곡률은 작은 이동량에 민감합니다. 그렇다고 실패 창을 지우거나 기존 판정 기준을 완화하지 않았습니다. 191개 미래 시작점은 서로 겹치는 6.4초 창이므로 191번의 독립 사건으로 해석하지 않습니다.

두 회차 모두 충돌·차선침범 **센서 이벤트 0**, 제어/관측 정렬 불일치 0이었습니다. 이 수치와 중심선 오차만으로 **차량 몸체 전체가 의도 차로 안에 있었다고 증명할 수는 없습니다.** AL의 속도·가감속 검사에는 위반이 없고 yaw 위반은 없어졌지만, 곡률 검사 때문에 전체 미래 궤적 계약은 실패합니다.

## 실제 기록과 재생 화면

<!-- HH_260906 - Identify these visuals as sampled CARLA expert replay, not an Autoware GUI or model inference. -->

- [AK B와 AL 속도·가감속·경로·정지 비교 PNG](final_audit/control_comparison.png)
- [AL 원본 6카메라, 차량 중심 경로 PNG](final_audit/rear_axle.png) / [GIF](final_audit/rear_axle.gif)
- [AK B 원본 6카메라, 차량 중심 경로 PNG](final_audit/b_coast_27_hold.png) / [GIF](final_audit/b_coast_27_hold.gif)
- [전체 원본·카메라·6.4초 미래 감사 JSON](final_audit/summary.json) / [AL 상세](final_audit/rear_axle.json) / [AK B 상세](final_audit/b_coast_27_hold.json)

PNG/GIF는 수집한 **CARLA 원본 6카메라와 차량 중심 경로를 표본 재생**한 것입니다. Autoware/RViz 실행 화면, 실시간 FPS, 학습 모델의 예측 또는 학습 모델 제어 화면은 아닙니다.

AL [소유 실행 결과](../../../../../storage/experiments/training/2026-09-27/drive_control_al/rear_axle/owner_result.json)는 종료 코드 0, 수집 소스 바이트 동일·보관, 설치된 CARLA `local_planner.py`와 `controller.py`의 실행 전후 해시 일치를 기록합니다. [원본 상태](../../../../../storage/experiments/training/2026-09-27/drive_control_al/rear_axle/episode/states.jsonl)와 [manifest](../../../../../storage/experiments/training/2026-09-27/drive_control_al/rear_axle/episode/manifest.json)에는 20 Hz 물리 상태, 10 Hz 카메라, 경로·정지·정리 결과가 있습니다. 이 실험의 제어 주체는 **CARLA BasicAgent**이며, Portable E2E 가중치는 바꾸지 않았고 주행 모델 학습도 실행하지 않았습니다.

## 다음 시험과 현재 위치

[AM 사전 선언](../drive_control_am/PLAN.md)은 이 AL 조건에서 횡방향 PID의 `K_P`만 1.95에서 1.70으로 낮춰 실제 회전 곡률과 경로 오차를 함께 확인합니다. AM이 회전 곡률을 통과해도 출발·정지 주변을 포함한 **전체** 미래 창, 독립 우회전 자료 승인, 미사용 검증 경로가 필요합니다. 현재 자체 주행 모델의 학습 기반 폐루프 승인 기능은 계획한 30개 세부 기능 중 **0/30**입니다. 30/60 km/h, 모든 Town, 실차 성능을 이 결과로 주장할 수 없습니다.

[앞선 AK A/B](../drive_control_ak/README.md)
