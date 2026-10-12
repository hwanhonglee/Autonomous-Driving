# 실제 저속 좌회전 교사 v2 — 전체 기록

실제 6카메라·측정 차량 위치·경로입니다. BasicAgent/manual1 교사이며 모델 자율주행이나 Autoware/RViz 화면이 아닙니다. rolling 좌회전과 진단 coast 기록이며 목적지 GO/HOLD/STOP 학습이 아닙니다.

<!-- HH_261002 - Explain the measured teacher result and separate display sampling from the original sensor rate. -->

**교사가 약 89.88° 좌회전하고 출구 방향으로 복귀한 실제 기록입니다.** 이 자료 347개 구간을 [새 주행 모델의 20,000회 학습](../20_actual_low_speed_left_joint_learning/README.md)에 사용 중입니다. 학습한 모델이 같은 회전을 해냈다는 뜻은 아닙니다.

| 확인한 항목 | 결과 |
| --- | --- |
| 전체 보존 시간 | 약 52.7초, 출발·좌회전·후속 coast 포함 |
| 원본 센서와 상태 | 10Hz 6카메라 528묶음, JPEG 3,168장, 20Hz 상태 1,055개 |
| 실제 명령 확인 | ACK 1,060개 |
| 차체·이동 중 여유 | 전체 차체 1,055개·연속 기록 간 sweep 1,054개 검사 통과 |
| 학습 정답 검사 | 전체 347구간·미래 22,208점, 실패 0구간 |
| 화면 | GIF 107프레임·PNG 5장, 차량 중심에 원 경로와 실제 궤적 표시 |

GIF는 보기·저장용으로 약 2Hz 간격을 사용하지만 **원본 카메라가 2Hz로 수집된 것은 아닙니다.** 마지막 `stationary_tail`은 원래 기록의 단계명이며 실제 차량은 coast 중입니다. 정상 목적지 정지 또는 STOP 학습 자료로 해석하지 않습니다. 화면의 긴 첫 상태 문구는 오른쪽 끝이 일부 잘리므로, 판정은 위 표와 아래 전체 원문을 따릅니다.

[원본·화면 전량 재검산](ROOT_actual_gallery_review.json): 입력·소스 3,421개와 미디어 6개 모두 SHA 변화가 없으며, 중간·마지막 화면도 직접 확인했습니다.

원래 결과: `Owner: PASS_ACTUAL_TEACHER_PENDING_TRUE_FUTURE_QA | true64: CHECKS_CLEAR 0/347 windows | existing research TRAIN qualification; NO model approval`. 전체347 구간·실제 미래22,208점의 기존 연구 TRAIN 자격은 외부 봉인된 별도 영수증을 재검증했습니다. 이 화면 자체는 모델·차량·MAIN9/sub30 승인이나 새 학습 승인이 아닙니다. 전체 출발·rolling·coast 기록을 보존합니다.

![실제 전체 기록](full_recorded_interval_six_camera_ego_centered_2hz.gif)

마지막 실제 카메라: [PNG](05_last.png). 화면의 페달은 현재 측정값이고, 같은 프레임의 교사 의도 표시는 이미 적용됐다는 증거가 아닙니다.
