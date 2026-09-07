<!-- HH_260906 - Bind observed expert evidence to its declared dataset split. -->
# Town04 / 독립 직진 테스트 데이터

이 화면은 CARLA **BasicAgent expert**가 주행하면서 수집한 실제 camera/state 자료입니다. 학습한 Portable E2E 모델이나 Autoware가 이 경로를 제어한 화면은 아닙니다.

사전에 지정한 map-held-out TEST episode입니다. 이 지표로 학습 설정·threshold·checkpoint를 고르지 않습니다. Town03 전체 episode는 기존 VAL로 유지합니다.

[전체화면 PNG](01_centered_expert_overview.png) · [차량 중심 GIF](02_centered_expert_drive_5fps.gif)

PNG와 GIF는 원본 bytes를 그대로 복사했습니다. 둘 다 1920×1080이고 GIF는 200 frames / 5 fps입니다. 여섯 camera와 중앙 ego, 기준 경로, 실제 궤적, 0.1–6.4초 future label을 함께 표시합니다. GIF는 검토용 sampling이며 wall-clock 실행 속도나 camera 입력 Hz를 재는 자료가 아닙니다.

| 항목 | 결과 |
|---|---|
| 경로 / 원본 수집 길이 | 210.010 m / 37.300 s |
| 목표 / 최고 속도 | 30 km/h / 30.257 km/h |
| 원본 bundle / state | 374 / 747 |
| native camera / physics | 9.999999851 Hz / 20 Hz |
| 최대 camera skew | 0.000 ms |
| goal / 충돌 / 차선침범 | PASS / 0 / 0 |
| 최대 절대 CTE | 1.269 m |
| 64-horizon export / Common10 선택 | 310 / 309 samples |

두 sample 수 차이 1개는 Common10이 stationary-tail anchor를 제외하기 때문입니다. tail state는 앞선 sample의 future context로 보존하며 프레임을 복제하거나 시간을 바꾸지 않았습니다.

수집 조건은 640×360 six-camera, JPEG95, ClearNoon, seed1, 3.5초 brake warmup, 6.5초 brake tail, Prius wheelbase2.850m입니다. Expert waypoint purge는 2.0m + 0.2s × speed(m/s), lateral PID 1.95/0.05/0.20, steering max0.8, lane offset0m를 사용했습니다. 외부 traffic actor는 추가하지 않았습니다.

원본 경로·manifest·수집/전송 SHA는 [metadata](06_native_camera_and_split_summary.json)에 있습니다. JSON 표시본은 machine root를 placeholder로 바꿨으며 각 wrapper에 원본 SHA를 남겼습니다. 발행본 무결성은 `sha256sum -c SHA256SUMS`로 확인합니다.

원본 episode: `artifacts/training/2026-09-07/independent_common10_v1/town04_straight_test/run_001/episode`

새 장애물·ACC·차선변경 label, 학습 모델 closed-loop, 실차 운용 승인을 입증하지 않습니다.
