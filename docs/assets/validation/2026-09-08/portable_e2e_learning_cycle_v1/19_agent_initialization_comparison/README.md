<!-- HH_260906 - Preserve both initialization-order outcomes and the native launch failures without causal or admission claims. -->
# C-track 초기화 순서 비교: 초기 조향 정상화, 출발 가속도는 여전히 실패

실제 CARLA BasicAgent 전문가 주행입니다. 학습 모델이나 Autoware 자율주행 화면이 아닙니다. 목표 14.4 km/h, Epic, 동일 원본 좌회전 경로·센서·제어 설정이며 학습 데이터 승인은 하지 않았습니다.

| 구분 | 이전: bootstrap 이전 생성 | 새 시험: 검증된 bootstrap 후 생성 |
|---|---:|---:|
| 첫 API 관측 조향 | −0.800000 | −0.000218737 |
| native 20Hz 최대 가속도 | +3.654933 m/s² | +3.467811 m/s² |
| 카메라 10Hz 최대 가속도 | +2.450543 m/s² | +2.715636 m/s² |
| native 상태 / 카메라 묶음 | 1,907 / 954 | 1,939 / 970 |
| 종점 오차 | 0.938601 m | 0.952436 m |
| 저속 안정 구간 | 39.45초 | 39.50초 |
| native 품질 결과 | FAIL | FAIL |

이전 결과는 별도 시점의 **비무작위 역사 비교**입니다. 새 시험은 **1회 / 최대 2회** 실행했으며 두 번째는 미실행·한도 미소진입니다. 유리한 반복만 고른 결과가 아니며 동일 조건의 독립 경로 표본도 아닙니다.

새 시험은 bootstrap frame 2863에서 생성 전후 pose·시간·control이 일치하고 PID 조향 이력이 0임을 확인했습니다. 기존 warmup 70틱 후 frame 2933의 첫 제안·송신 명령이 receipt 73으로 승인됐으며 다음 frame 2934의 관측 control까지 일치합니다. ACK 1,944개 모두 확인, 불일치·충돌·차선 침범 0개입니다. 초기 조향이 거의 0으로 바뀌었지만 **출발 가속도 초과는 해결되지 않았습니다**.

BasicAgent 생성자는 현재 위치와 cached control 등을 읽으므로 이것은 전체 생성 시점 변경 실험이지 조향 이력만의 원인 증명이 아닙니다. API 관측과 서버 ACK는 실제 물리 적용 시각의 증명도 아닙니다. 추가 read-only 관측의 시간 비용도 0이라고 주장하지 않습니다.

[이전/신규 전체 속도·20Hz 가속도와 출발 확대](01_initialization_and_native_acceleration_comparison.png) · [새 전체 경로와 위반 구간](02_new_full_route_and_native_launch_failure.png) · [시작 실제 6카메라](actual_camera_route_evidence/01_start.png) · [좌회전 중앙](actual_camera_route_evidence/07_catalog_left_midpoint.png) · [종점 정지](actual_camera_route_evidence/04_goal_dwell.png) · [전체 구간 GIF](actual_camera_route_evidence/whole_recording_accelerated.gif)

10Hz 결과나 두 가지 대체 10Hz offset은 진단용이며 **20Hz +2.9 디코더 / +3.0 runtime 가속도 실패를 대체하지 않습니다**. 전체 warmup·driving·정차 tail과 위반 구간을 제거·평활화하지 않았습니다. 2초 정지 유지 및 130개 tail 상태를 보존했습니다. 전체 XY·픽셀 품질·30 km/h 검증·실차·학습 데이터 승인과는 별개입니다.

GIF는 10Hz 묶음을 5개 간격으로 고른 10fps 가속 재생으로 실제 FPS 계측이 아닙니다. 카메라 합성은 1600×900, 전체 FOV와 차량 중심 경로를 유지합니다. 07 장면은 원본 LEFT 구간 중간 진행거리에 가장 가까운 카메라로 선정했습니다. 기존 renderer의 02는 7.8m/s 조건이므로 이 저속 기록에는 없습니다.

[독립 감사·사전 계획·원본 해시](audit.json) · [전체 5,820개 이미지 해시](run_001_image_hashes.json) · [발행 출처](publication_manifest.json) · [파일 검증값](SHA256SUMS) · [이전 실패 원본 결과](../18_c_track_low_speed_turn/README.md)

원본 실패 기록·카메라 데이터는 private artifacts에 그대로 있습니다. 기존 렌더 PNG/GIF는 정확한 원본 바이트로 복사했고 JSON은 계정 경로를 가린 메타데이터 뷰입니다. 원본 JSON 해시를 보존합니다. 새 검증기는 dfccaa8 소스를 따로 고정하여 이전 c4f40fc 감사기의 소스 제한을 변경하지 않습니다.
