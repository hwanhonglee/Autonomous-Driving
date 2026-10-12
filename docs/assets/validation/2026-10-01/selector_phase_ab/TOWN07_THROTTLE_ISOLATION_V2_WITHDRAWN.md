# HH_261001 — Town07 스로틀 분리 v2: 실행 금지 기록

**v2 계획은 선언만 되었으며 실행하지 않는다.** 계획 SHA256 `3719e63e839cd4d2440c8acc075e5df014fcbf31be53f9896960ef6ef3e035a2`와 관련 소스는 수정·삭제하지 않는다.

정적 대조에서 v2 `warmup_final_failures()`가 준비 종료 시 절대 `route_progress_m ≤ 0.25`를 요구한다. 같은 Town07 route SHA256 `8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9`를 사용한 V3 원본 `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1/episode/states.jsonl`의 정지 준비 첫 프레임 1147은 이미 `route_progress_m=0.31809989301818486`이다. 따라서 정상적으로 정지한 차량도 v2 종료 조건에서 실패할 가능성이 높다. 원인은 모델/차량 성능이 아니라 **절대 시작 좌표와 정지 중 이동량을 혼동한 검증식**이다.

v1·v2 모두 CARLA 미실행이며, 디스크 시작 예산도 아직 부족하다. v2의 성공·실패 주행 결과는 없다. 별도 [v3 계획](TOWN07_THROTTLE_ISOLATION_V3.md)은 `bootstrap` 대비 진행 변화량 `≤0.25 m`와 절대 시작 통로 `0–0.5 m`를 각각 검사하고, 원본 V3의 `0.3181 m` 회귀 검사를 포함한다. v3 역시 자원·소스 사전점검 통과 전에는 실행하지 않는다.
