<!-- HH_260906 - Keep all four actual outcomes and distinguish scalar improvement from data admission. -->
# 정상 브레이크 제거 V4: 기존 실패 2건과 반복 시험 비교

Town07 직진의 같은 초기 조건에서 정상 브레이크 상한만 0.1→0으로 변경했습니다. 정상 조향, 비상 제동, 정지 확인 후 꼬리 구간의 전제동은 유지했습니다. 기존 V3 ACK 두 건과 실행된 V4 전부를 포함하며 실패 구간을 자르지 않았습니다. **학습 모델 주행·데이터셋 승인·실차 검증은 아닙니다.**

| 조건 | 종료 코드 | 전체 속도 변화율·목표정지 QA | 20Hz 최솟값 / 최댓값 (m/s²) | 마지막 주행 목표 오차 (m) |
|---|---:|---|---:|---:|
| V3 ACK run_001 | 1 | FAIL | -8.860458 / 2.890151 | 0.831987 |
| V3 ACK run_002 | 1 | FAIL | -7.730855 / 2.803371 | 0.836505 |
| V4 brake=0 run_001 | 0 | PASS | -1.601662 / 2.688967 | 0.843325 |
| V4 brake=0 run_002 | 0 | PASS | -1.746143 / 2.803371 | 0.823259 |

![모든 시도의 실제 속도 변화율](01_all_four_speed_rate_traces.png)

그래프는 준비·주행·정지 꼬리와 모든 경계 샘플을 포함합니다. 패널별 Y축 범위가 다르며 기존 실패의 큰 감속도 그대로 표시했습니다. 물리20Hz, 카메라10Hz, 명목28.8km/h, 실제 상한30km/h, 목표1m·0.1m/s 이하2초 유지, decoder ±2.9 및 runtime +3/−6m/s² 기준은 변경하지 않았습니다.

모든10개 실행 소스의 보관 바이트는 사전 검토 커밋과 실행 당시 HEAD 양쪽에 일치합니다. 두 번째 HEAD는 공개 문서 커밋 때문에 달라졌으므로 전체 커밋이 같다고 주장하지 않습니다. 사전 계획·첫 결과·두 번째 검토·실행의 기록된 시간과 원본 SHA를 검증했으며, 역사적 파일 생성 시점의 불변 증명으로 확대하지 않습니다.

제어 API 보고 값과 서버 ACK는 물리 페달·토크 적용 시점의 증명이 아닙니다. 같은 경로 반복은 독립 경로 검증이 아니며, 전체 future XY/영상 품질의 후속 검사와 데이터 승인은 별도입니다. Low의 체크무늬 노면을 포함한 실제 영상은 미화하지 않았습니다.

### V4 run_001

![실제 원시 카메라와 경로의 가속 재생](run_001/whole_recording_accelerated.gif)

[순항 PNG](run_001/02_measured_cruise.png) · [정지 유지 PNG](run_001/04_goal_dwell.png) · [원시 영상 출처](run_001/visual_provenance.json)

### V4 run_002

![실제 원시 카메라와 경로의 가속 재생](run_002/whole_recording_accelerated.gif)

[순항 PNG](run_002/02_measured_cruise.png) · [정지 유지 PNG](run_002/04_goal_dwell.png) · [원시 영상 출처](run_002/visual_provenance.json)

영상은 원본 카메라를 사용한1600×900 진단 합성 화면이며 Autoware/RViz 라이브 화면이 아닙니다. GIF는 명시된 간격의 가속 재생으로 카메라 FPS 측정값이 아닙니다.

[독립 재계산 결과](summary.json) · [원본/공개 해시 출처](provenance.json) · [전체 공개 SHA256](SHA256SUMS)

원본: `artifacts/training/2026-09-08/brake_free_goal_stop_v4`. 이전 V3: `artifacts/training/2026-09-08/acknowledged_control_v1`. 원본·학습/검증/테스트 데이터셋은 변경하지 않았습니다.

재현: `python3 scripts/e2e/audit_carla_brake_free_goal_stop.py --campaign <V4 원본> --baseline-root <V3 town07_straight_calibration> --visual-root <V4 시각화 원본> --output <새 폴더>`. 기존 출력은 덮어쓰지 않습니다.
