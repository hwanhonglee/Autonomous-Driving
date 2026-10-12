# 실제 좌회전 개선 — 회전·침범은 개선, 중심 정확도는 미달

<!-- HH_261002 - Lead with measured behavior and the remaining unchanged accuracy gate; disclose unequal complete/early-failure denominators. -->

**새 모델은 실제 좌회전을 완료했습니다.** 기존 모델은 회전 중 차선을 침범해 수집이 끝났지만, 새 모델은 약 89° 회전 후 10m 이상 진행하고 3초 안정 주행했습니다. **다만 차로 중심 오차 p95 0.413m가 기존 0.35m 기준을 넘어서 종합 판정은 여전히 FAIL입니다.** 기준을 낮추거나 기능 완료로 바꾸지 않습니다.

같은 TRAIN 사용 경로·CPU4·PI2.4·0.5초 조건의 GOOD6000/LEFT20k 비교입니다. 독립 평가·목적지 정차·정식 기능 승인이 아니며 메인 0/9·하위 0/30입니다. 학습 자료·학습 기간을 함께 바꾼 결과이지 한 요인의 효과를 분리한 실험이 아닙니다.

| 실제 측정 / 원본 판정 유지 | 기존 GOOD6000 | 새 LEFT20000 |
| --- | ---: | ---: |
| 회전 후 10m·3초 안정 주행 | 미완료 | **89.13°·10.70m·3.00초 완료** |
| 모델 제어 이후 진행 | 51.26m | **91.89m** |
| 충돌 / 차선 센서 침범 | 0 / 1 | **0 / 0** |
| 차체·이동 영역 10cm 여유 | 실패 | **전량 통과** |
| 같은 처음 513개 모델 제어 상태의 중심 오차 p95 | 0.787m | **0.427m** |
| 각 시도의 전체 중심 오차 p95 | 0.787m / 513상태 | 0.413m / 838상태 — 기준 0.35m 미달 |
| 관측 묶음→명령 ACK 최대 | 80.55ms | **75.11ms** |
| 실제 상태 / 6카메라 묶음 / ACK | 600 / 300 / 605 | 1,055 / 528 / 1,060 |

기존 시도는 `recovery_collision_or_lane_event`로 약 30초에서 실제 수집 종료됐습니다. 이후 상태를 만들거나 45초 완료로 가정하지 않습니다. 두 시도 모두 **존재하는 모든 원본·실패·마지막 화면**을 남겼습니다. 그림의 `FAIL retained`는 회전 자체가 전혀 안 됐다는 뜻이 아니라 **전체 정확도·안전·완료 기준 중 하나 이상이 미달**이라는 뜻입니다.

**다음:** 같은 새 가중치로 기존 추종 거리와 가까운 추종 거리의 두 조건을 사전에 고정하고 다시 주행합니다. 더 정확한 회전을 기대하지만 조향 진동·차체·가감속·100ms 기준 악화도 함께 확인합니다. 속도·합격 기준·가중치는 그대로 유지합니다.

## parent6000 — FAIL_RETAINED

실제 분모 `{'native': 600, 'camera': 300, 'ACK': 605, 'JPEG': 1800}`; 원래 전체 완료 `False`. 실패와 원본 판정을 그대로 보존합니다.

![전체 실제 기록](parent6000/full_recorded_interval_six_camera_ego_centered_2hz.gif)

[01_first.png](parent6000/01_first.png) · [02_quarter.png](parent6000/02_quarter.png) · [03_middle.png](parent6000/03_middle.png) · [04_three_quarter.png](parent6000/04_three_quarter.png) · [05_last.png](parent6000/05_last.png) · 

## LEFT20000 — FAIL_RETAINED

실제 분모 `{'native': 1055, 'camera': 528, 'ACK': 1060, 'JPEG': 3168}`; 원래 전체 완료 `True`. 실패와 원본 판정을 그대로 보존합니다.

![전체 실제 기록](LEFT20000/full_recorded_interval_six_camera_ego_centered_2hz.gif)

[01_first.png](LEFT20000/01_first.png) · [02_quarter.png](LEFT20000/02_quarter.png) · [03_middle.png](LEFT20000/03_middle.png) · [04_three_quarter.png](LEFT20000/04_three_quarter.png) · [05_last.png](LEFT20000/05_last.png) · 

파랑은 원 경로, 초록은 현재까지 측정 궤적, 보라는 현재 상태보다 앞서 ACK된 모델 계획입니다. 긴급제동·진단 경계·coast에서는 오래된 보라를 지웁니다. 페달은 실제 관측값이며 교사 출발/무페달 coast를 NN 제어나 STOP 성공으로 세지 않습니다. 실제 CARLA 기록 기반 화면이지 Autoware/RViz 촬영이 아닙니다. 전체 SHA·실패·시계는 [provenance](publication_provenance.json)에 남겼습니다.

두 GIF는 실제 카메라 전 구간 29.9초·52.7초와 마지막 표시 유지 0.1초를 포함합니다. 화면 게시 뒤 모든 원본·소스 및 GIF 2개·PNG 10개를 다시 검증하고 세 PNG를 직접 확인했습니다. 차량 중앙·6카메라 전체 화각·경로·이미 적용된 계획을 확인했으며 제목 잘림은 없었습니다. [직접 화면 확인·전량 SHA 근거](ROOT_actual_gallery_review.json).
