# 실제 Town03 직진과 카메라 수신 시간 측정

FAIL(실패 구간 전부 보존)입니다. 독립 평가나 정식 RTE-01 승인이 아니며 현재 메인 0/9·하위 0/30은 변하지 않습니다.

이번에는 실제 카메라 callback의 첫 문장부터 계획 생성·명령 ACK까지 같은 단조 시계로 기록했습니다. 최대 수신→계획 미확인, 수신→ACK 미확인입니다. 센서 하드웨어 노출 시각이나 실제 벽시계 10Hz 보장을 측정한 것은 아닙니다.

실제 상태 900개, 6카메라 묶음 450개를 전부 확인했습니다. 최초→마지막 상태 간격은 44.95초입니다. GIF는 고정 0.5초 표본과 정확한 마지막 실제 카메라를 포함하고, PNG는 실행 전 선언한 최초/25%/50%/75%/마지막 규칙만 사용합니다. 짧게 끝났다면 전체 가용 실패 꼬리를 유지하며 PASS로 바꾸지 않습니다.

원본 오류(그대로 보존): `{"original_capture_error": "CollectionError: BasicAgent did not finish within the driving budget reserved by the maximum total duration and stationary tail", "callback_timing_diagnostic_error": "CollectionError: duplicate, incomplete or malformed actual callback receipt"}`

![실제 전체 기록](full_actual_six_cameras_2hz.gif)

![01_start.png](01_start.png)

![02_quarter.png](02_quarter.png)

![03_half.png](03_half.png)

![04_three_quarters.png](04_three_quarters.png)

![05_last.png](05_last.png)

지도 패널은 모든 장면에서 측정 차량을 중심에 두고 파랑 원 경로, 초록 실제 주행 궤적, 보라색 실제 이전 ACK가 적용된 NN 계획만 표시합니다. 여섯 영상은 기록된 CARLA JPEG입니다. 이는 기록 기반 진단 렌더링이며 실제 Autoware 또는 RViz 화면이 아닙니다. 원본 판정·SHA·표본 인덱스는 [publication_provenance.json](publication_provenance.json)에 보존합니다.
