# 실제 목적지 정차 교사 주행 v7

이 화면은 실제 기록된 6카메라·측정 차량 위치·경로를 보여줍니다. BasicAgent 교사 제어이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다.

원래 결과: `Owner: PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA | future64: FAIL 158/1665 windows`. 실패·대기 상태와 전체 출발·정지 유지·후속 기록을 보존하며 학습 승인으로 해석하지 않습니다.

![실제 전체 기록](full_recorded_interval_six_camera_ego_centered_2hz.gif)

마지막 실제 카메라: [PNG](05_last.png). 화면의 페달은 현재 측정값이고, 같은 프레임의 교사 의도 표시는 이미 적용됐다는 증거가 아닙니다.
