# 실제 Town03 우회전 시도: 실패를 포함한 전체 기록

NN 제어 효과 관측 0개 — 출발/수집 실패, NN 우회전 판정 불가입니다. 목적지 정차나 정식 RTE03 성공이 아닙니다. 이미 사용한 DEV 경로 한 조건으로 신규 독립 평가 0개·메인0/9·하위0/30입니다.

이 기록은 실제 NN 제어 효과를 관측하기 전에 끝났습니다. **NN 우회전 성능의 실패나 성공으로 세지 않습니다.** 교사·위험 감지의 제동과 모델 명령을 구분하며, 수신→ACK 처리 시간을 없던 자료로 만들지 않습니다.

실제 183개 상태·92개 6카메라 묶음을 전량 확인했습니다. 상태 최초→마지막은 9.10초이며, GIF는 마지막 상태의 50ms 표시 구간까지 포함합니다. 사전 110초 상한을 채웠다고 주장하지 않습니다. 관측된 NN 효과 0개·ACK된 NN 명령 0개·신규 NN 제안 0개입니다. 고정 0.5초 간격과 정확한 마지막 카메라를 포함하며 PNG는 최초/¼/½/¾/마지막입니다. 실패 꼬리나 좋은 회전 장면만 고르지 않습니다.

원본 오류(변경 없이 보존): `{"original_capture_error": "ValueError: fixed low-speed launch did not reach 0.5 m/s within 8 seconds", "diagnostic_error": "RuntimeError: complete unique camera/ACK receipts required"}`

전체 정식 주행·제어 전송·가감속 판정을 새로 PASS로 바꾸지 않습니다. 관측 뒤 cleanup ACK가 있어도 없는 미래 차량 자세나 카메라를 만들지 않습니다. 별도 원본 감사가 있으면 그 FAIL도 provenance에 그대로 보존합니다.

![실제 전체 기록](full_actual_six_cameras_2hz.gif)

![01_start.png](01_start.png)

![02_quarter.png](02_quarter.png)

![03_half.png](03_half.png)

![04_three_quarters.png](04_three_quarters.png)

![05_last.png](05_last.png)

차량 위치 중심 지도에서 파랑은 원 경로, 초록은 현재까지 실제 궤적, 보라는 현재 상태에 앞서 실제 ACK가 확인된 모델 계획입니다. 긴급제동 이후 보라를 지우며 교사 출발은 모델 제어로 세지 않습니다. 실제 CARLA 카메라와 기록 기반 진단으로 Autoware/RViz 촬영이 아닙니다. 전체 owner 판정·실패·원본 SHA·현재 프레임과 과거ACK는 [publication_provenance.json](publication_provenance.json)에 남겼습니다.
