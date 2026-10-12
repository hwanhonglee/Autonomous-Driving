# 다음 주행 원본의 차체 형상 출처 보강

<!-- HH_260906 - Record the spawned actor's geometry without retroactively approving earlier captures. -->

**상태: 수집기 코드·단위시험 완료, 새 원본 미수집, 차로 QA 미승인.** Town01 신규 우회전 `s0002_p06`은 실제 9 km/h BasicAgent 원본을 남겼지만 그 episode의 manifest에는 해당 순간 생성된 Prius actor의 바운딩박스가 없다. 과거 다른 Prius 계측값을 대입하면 차량 asset 동일성과 frame offset을 독립 입증할 수 없으므로 차체 전체의 *의도 차로 포함*을 PASS로 소급하지 않는다. 차선침범 sensor 이벤트 0건도 차체 포함의 충분조건이 아니다.

[수집기](../../../../../tools/e2e/collect_carla_vad_expert.py)는 **앞으로** 차량을 생성한 직후 `carla.Vehicle.bounding_box`의 actor-local 위치·반길이·회전과 `type_id`·actor ID를 `capture_contract.actor_body_geometry`에 기록한다. 실제 주행에 사용한 `carla.Map.to_opendrive()`의 원본도 해당 episode의 `active_map_opendrive.xodr`로 보관하고 SHA256을 `capture_contract.active_map_opendrive`에 묶는다. 유한한 양수 반길이가 아니거나 맵 XML이 비정상이면 수집을 실패시키고, API 자체가 없는 시험 actor/map이라면 `UNAVAILABLE`을 남긴다. `RECORDED`도 형상·지도 *측정*일 뿐 `lane_body_containment_verified=false`로 남는다. 실제 차로 판정에는 같은 episode의 actor pose, 매 시점의 **의도** road/section/lane 대응, 차체 polygon의 경계/연속 시간 검사가 추가로 필요하다. 주변의 아무 주행 차로 하나에 닿는지만 보는 검사는 대체물이 아니다.

이 변경은 기존 [우회전 단회 수집 계획](../drive_long_route_p06_capture/PLAN.md)의 소스 해시나 수집 원본을 수정하지 않는다. [기존 원본의 독립 QA](../drive_long_route_p06_capture/QA_v3/QA.md)에는 형상 결손 **외에도** 실제 곡률 기준 초과가 있어, 형상만 보강해 같은 조건으로 재촬영하는 것은 학습 자격을 보장하지 않는다. 새 episode의 기본 입장도 `training_data_approved=false`, `learned_model_control=false`이며 재촬영 성공을 모델 성능으로 세지 않는다.

검증: collector, ACK, 시간, 이동 capture, agent 초기화의 단위·모의 통합 테스트 → **161 passed**. CARLA 재촬영·PRO6000 학습·Autoware 구동은 이번 변경에서 실행하지 않았다.
