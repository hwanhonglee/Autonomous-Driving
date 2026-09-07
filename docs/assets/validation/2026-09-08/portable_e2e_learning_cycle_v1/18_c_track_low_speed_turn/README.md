<!-- HH_260906 - Preserve the first low-speed turn failure with exact raw provenance and no admission claim. -->
# C-track 저속 좌회전: 경로·정차 완료, 출발 가속도 실패

실제 CARLA BasicAgent 전문가 주행입니다. 학습 모델이나 Autoware 자율주행 화면이 아닙니다. 목표 14.4 km/h의 별도 개발 시험이며 30 km/h 검증·학습 데이터 승인은 하지 않았습니다.

- 실행 **1회 / 허용 최대 2회**. 첫 실행 실패를 그대로 보존했습니다. 두 번째는 **미실행(NOT_RUN)**이며 횟수 한도가 소진된 것도, 반드시 재시도해야 하는 것도 아닙니다.
- native 1,907개 상태, 954개 10Hz 카메라 묶음(5,724개 이미지), ACK 1,912개 모두 확인; 불일치·충돌·차선 침범 0개.
- 실제 최대 14.4086 km/h, 저속 안정 구간 39.45초. 종점 오차 0.9386m, 2초 정지 유지와 130개 정차 후속 상태 확인.
- **20Hz 출발 한 구간 +3.654933 m/s²**가 물리 디코더 +2.9 및 runtime +3.0 기준을 초과했습니다. 10Hz 최대 +2.450543만 보고 통과시키지 않습니다. 원본 구간을 제거·평활화하지 않았습니다.
- 기존 C-track lateral 설정 2.0m/0.2s를 사용했습니다. Town07의 3.0m/0.5s와 동일 제어 조건의 맵 간 A/B가 아닙니다. 원본 경로의 예전 30km/h PASS는 새 시험의 승인 근거가 아닙니다.

[전체 경로·속도·출발 위반 확대](01_full_route_and_native_launch_failure.png) · [좌회전 중앙 실제 6카메라/차량 중심 경로](actual_camera_route_evidence/07_catalog_left_midpoint.png) · [전체 구간 GIF](actual_camera_route_evidence/whole_recording_accelerated.gif) · [정차 장면](actual_camera_route_evidence/04_goal_dwell.png)

GIF는 10Hz 카메라에서 5개 간격으로 고른 10fps **가속 재생**입니다. 센서 FPS나 GUI 끊김 측정이 아닙니다. 1600×900 카메라 합성 화면은 전체 FOV를 보존하고 차량을 경로 패널 중앙에 표시합니다. 07 장면은 원본 LEFT 태그 구간의 중간 진행거리와 가장 가까운 카메라로 고정 선정했습니다. 기존 renderer의 02는 7.8m/s 이상을 조건으로 하므로 이 저속 기록에는 생성하지 않았습니다.

[정확한 원본 경로 JSON](original_c_track_left_route.json) · [독립 계측과 소스·사전 계획 검증](audit.json) · [전체 이미지 SHA 목록](run_001_image_hashes.json) · [발행 출처 증거](publication_manifest.json) · [파일 검증값](SHA256SUMS)

원본 C-track 경로 끝에는 같은 위치·진행거리의 점 두 개가 있습니다. 독립 계측기의 접선 선택만 직전의 다른 위치를 찾도록 수정했습니다. 원본 경로 점·배열·파일과 실제 차량 높이는 변경하지 않았습니다. downstream Autoware z−15 정렬은 native CARLA spawn/goal/상태/센서 TF에 적용하지 않았습니다.

원본 이미지와 실패 기록은 private artifacts에 유지합니다. 이 폴더의 PNG/GIF 기존 렌더본과 경로는 원본 바이트이며 JSON은 계정 경로를 가린 메타데이터 뷰입니다. 원본 해시를 보존합니다. 카메라 바이트 무결성은 전체 픽셀·미래 XY·학습 데이터 품질 승인과 다릅니다.
