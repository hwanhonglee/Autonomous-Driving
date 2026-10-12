# Town01 j87 우회전 소유 원본 단회 진단

<!-- HH_260928 - Authorize one bounded expert diagnostic without admitting an offline route as training data. -->

이 계획은 [정적 경로 선별](README.md)의 `OFFLINE_PROVISIONAL_ONLY` 및 `capture_authorized=false`를 **학습 편입 승인으로 바꾸지 않는다.** 사용자의 주행 모델 개선 요청 범위에서 이 고정 경로의 BasicAgent **진단 촬영 1회**만 수행한다. 실제 CARLA가 보관된 OpenDRIVE와 다르면 차량 생성 전에 중단한다. 출력은 `storage/experiments/training/2026-09-28/drive_town01_j87_right_9kph_expert_trial_v1` 하나이며, 실패해도 같은 출력에 재시도하지 않는다.

고정 입력 route SHA256은 `6484a43d04a98b9d01d27994e8347545b7c350aab39ece542e186bd28d73b13e`, live XODR 기대 SHA256은 `97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709`다. [조건부 owner 명령](README.md)은 이전 C 전문가 설정(Epic/ClearNoon, Prius, 9km/h, 20/10Hz, 6카메라, ACK, 조향0.45, waypoint purge1.5m, 고도 spawn+2m)과 신규 `--expected-live-xodr-sha256` 한 옵션으로 잠근다. 이것은 새로운 route의 전문가 주행이므로 C와 모델 성능 A/B라고 부르지 않는다.

실행 전 문서/route와 collector/owner 현재 소스 검사를 끝내고 **실행 중 소스 편집을 중지**한다. 공용 lock, 기존 CARLA 프로세스, 2100–2102 점유, 새 출력 부재, 로컬 여유 공간을 확인한다. owner는 소유 프로세스만 정리한다. 이후 고정 30–120m 원본 window에 대해 30초 이동·6.4초 미래·9개 운동 한계·차체 raw/10cm inset/sweep·충돌/목표·여섯 카메라/ACK/lifecycle/출처를 전량 QA한다. 실패한 구간을 잘라내지 않는다.

단회 진단이 통과해도 이 Town01의 한 진입 방향은 독립 DEV가 아니고 이전 V5의 가까운 다른 진입 방향과 같은 junction일 수 있다. 자료 편입·PRO6000 GPU 학습·30개 기능 승인은 별도의 결정이다.
