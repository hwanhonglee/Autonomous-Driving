# 실제 Town03 209m 전체 경로 V4 원본 버퍼·복원 재시험

<!-- HH_261004 - Preserve actual whole-interval evidence and distinguish catalog cleanup from learned stopping. -->

원본 판정: **FAIL_KNOWN_DEV_FULL209_RESEARCH**. 알려진 DEV 경로의 연구 시험이며 정식 완료는 메인0/9·하위0/30입니다. 새 terminal decoder와 전체 폴더 용량 계산 개입이 포함된 비교로, 모델만의 성능 향상이라고 주장하지 않습니다.

실제 상태 **1836개**, 실제6카메라 묶음 **918개**와 실패 구간을 모두 보존했습니다. 첫→마지막 실제 상태 간격은 **91.75초**이며, 상태 개수×50ms 표시 구간은 91.80초입니다.

GIF는 실제 카메라 타임스탬프 간격과 마지막 카메라의 사전 선언100ms 표시 유지로 총 91.80초입니다. 이 마지막 표시 유지는 새 카메라·차량 상태·제어 관측이 아닙니다. 마지막 실제 카메라는 프레임 7158, 마지막 실제 차량 상태는 프레임 7159이며 둘을 합성하지 않았습니다.

GIF는 첫 카메라부터 매5번째와 정확한 마지막을 선택하며, PNG는 전체 실제 카메라 수의0/¼/½/¾/마지막입니다. 결과나 실패 위치에 따라 좋은 장면만 고르지 않았습니다. 원래2.5m 목표 도착 뒤 정리 제동 ACK는 학습한 정차·정차 유지·실차 안전 증거가 아닙니다.

![전체 실제6카메라 기록](full_actual_six_cameras_2hz.gif)

![01_start.png](01_start.png)

![02_quarter.png](02_quarter.png)

![03_half.png](03_half.png)

![04_three_quarters.png](04_three_quarters.png)

![05_last.png](05_last.png)

차량 중심 진단 지도에서 파랑은 원래 경로, 초록은 그 시점까지의 실제 이동, 보라는 현재 상태에 앞서 ACK된 학습 모델 계획입니다. 교사 출발·긴급제동을 모델 주행으로 세지 않으며 긴급제동 이후 모델 계획을 지웁니다. 실제 CARLA 입력을 이용한 기록 기반 진단으로, Autoware/RViz 화면 캡처가 아닙니다. 전체 원본 판정·분모·원본SHA·선택 규칙은 [publication_provenance.json](publication_provenance.json)에 있습니다.

<!-- HH_261004 - Keep V3's missing-JPEG failure immutable and distinguish fresh source ownership from learned driving. -->

V4는 기존 주행 모델·제어를 유지하고, 실제 카메라 원본 버퍼의 불변 복사본을 확보한 뒤 3개 작업자로 인코딩하고 모두 종료한 뒤 원래 순서로 기록하는 별도 연구입니다. 신호등은 설치 전 실제 전체 목록을 기준으로 캡처 밖 2개 실제 틱에서 복원 상태를 검증합니다. V3의 JPEG0개 실패를 변경하거나 누락 이미지를 만들어 넣지 않았습니다. 이 자료는 기록 기반 진단이며, 정식 기능 완료·실차 안전·실제 지연 개선은 별도 검증 전까지 주장하지 않습니다.
