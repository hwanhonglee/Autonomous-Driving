# 실제 NN 좌회전 비교 — 두 시도 전체 기록

같은 TRAIN 사용 경로·CPU4·PI2.4·0.5초 조건의 같은 LEFT20k의 lateral1.5m/0.8s 대조군과1.0m/0.5s 후보 비교입니다. 독립 평가·목적지 정차·정식 기능 승인이 아니며 메인0/9·하위0/30입니다.

## baseline — FAIL_RETAINED

실제 분모 `{'native': 1055, 'camera': 528, 'ACK': 1060, 'JPEG': 3168}`; 원래 전체 완료 `True`. 실패와 원본 판정을 그대로 보존합니다.

![전체 실제 기록](baseline/full_recorded_interval_six_camera_ego_centered_2hz.gif)

[01_first.png](baseline/01_first.png) · [02_quarter.png](baseline/02_quarter.png) · [03_middle.png](baseline/03_middle.png) · [04_three_quarter.png](baseline/04_three_quarter.png) · [05_last.png](baseline/05_last.png) · 

## short_lookahead — FAIL_RETAINED

실제 분모 `{'native': 1055, 'camera': 528, 'ACK': 1060, 'JPEG': 3168}`; 원래 전체 완료 `True`. 실패와 원본 판정을 그대로 보존합니다.

![전체 실제 기록](short_lookahead/full_recorded_interval_six_camera_ego_centered_2hz.gif)

[01_first.png](short_lookahead/01_first.png) · [02_quarter.png](short_lookahead/02_quarter.png) · [03_middle.png](short_lookahead/03_middle.png) · [04_three_quarter.png](short_lookahead/04_three_quarter.png) · [05_last.png](short_lookahead/05_last.png) · 

파랑은 원 경로, 초록은 현재까지 측정 궤적, 보라는 현재 상태보다 앞서 ACK된 모델 계획입니다. 긴급제동·진단 경계·coast에서는 오래된 보라를 지웁니다. 페달은 실제 관측값이며 교사 출발/무페달 coast를 NN 제어나 STOP 성공으로 세지 않습니다. 실제 CARLA 기록 기반 화면이지 Autoware/RViz 촬영이 아닙니다. 전체 SHA·실패·시계는 [provenance](publication_provenance.json)에 남겼습니다.
