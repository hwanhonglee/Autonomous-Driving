# 실제 Town03 우회전 시도: 실패를 포함한 전체 기록

교사만 주행한 NN 통합 실패 — NN 우회전 성능 판정 불가입니다. 목적지 정차나 정식 RTE03 성공이 아닙니다. 이미 사용한 DEV 경로 한 조건으로 신규 독립 평가 0개·메인0/9·하위0/30입니다.

이 기록의 전체 주행은 교사 제어이며 NN 제어 효과는 관측되지 않았습니다. 출발 직후 중단한 v1과 달리, 긴 교사 주행을 했더라도 **NN 우회전 성공으로 세지 않습니다.** NN 통합 실패·원본 owner FAIL·실제 차체 FAIL이 있다면 모두 그대로 유지합니다. 교사가 목적지에 갔다는 사실을 모델 주행이나 신호 준수 성능으로 바꾸지 않습니다.

실제 1868개 상태·934개 6카메라 묶음을 전량 확인했습니다. 상태 최초→마지막은 93.35초이며, GIF는 마지막 상태의 50ms 표시 구간까지 포함합니다. 사전 110초 상한을 채웠다고 주장하지 않습니다. 관측된 NN 효과 0개·ACK된 NN 명령 0개·신규 NN 제안 0개입니다. 고정 0.5초 간격과 정확한 마지막 카메라를 포함하며 PNG는 최초/¼/½/¾/마지막입니다. 실패 꼬리나 좋은 회전 장면만 고르지 않습니다.

원본 오류(변경 없이 보존): `{"diagnostic_error": "AttributeError: module 'scripts.e2e.collect_temporal_route_joint_town03_right_v2_20261002' has no attribute 'RightTurnObserver'"}`

전체 정식 주행·제어 전송·가감속 판정을 새로 PASS로 바꾸지 않습니다. 관측 뒤 cleanup ACK가 있어도 없는 미래 차량 자세나 카메라를 만들지 않습니다. 별도 원본 감사가 있으면 그 FAIL도 provenance에 그대로 보존합니다.

![실제 전체 기록](full_actual_six_cameras_2hz.gif)

![01_start.png](01_start.png)

![02_quarter.png](02_quarter.png)

![03_half.png](03_half.png)

![04_three_quarters.png](04_three_quarters.png)

![05_last.png](05_last.png)

차량 위치 중심 지도에서 파랑은 원 경로, 초록은 현재까지 실제 궤적, 보라는 현재 상태에 앞서 실제 ACK가 확인된 모델 계획입니다. 긴급제동 이후 보라를 지우며 교사 출발은 모델 제어로 세지 않습니다. 실제 CARLA 카메라와 기록 기반 진단으로 Autoware/RViz 촬영이 아닙니다. 전체 owner 판정·실패·원본 SHA·현재 프레임과 과거ACK는 [publication_provenance.json](publication_provenance.json)에 남겼습니다.

발행 v2는 문구와 정확한 v1/v2/v3 출력 구분만 추가했습니다. 제어·차체·시간·원본 실패 판정을 완화하지 않았으며, 별도 감사 원문은 provenance의 auxiliary_root_audits에만 연결합니다.
