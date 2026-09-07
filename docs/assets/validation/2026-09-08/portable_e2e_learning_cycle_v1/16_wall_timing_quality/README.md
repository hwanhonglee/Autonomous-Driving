<!-- HH_260906 - Separate actual simulation cadence, wall throughput, visual observations and unapproved training data. -->
# Low/Epic 화면 품질과 실제 처리 시간: Town07 직진 각 1회

동일 V4 정상 브레이크 0·ACK 제어·6카메라·Town07 직진에서 렌더링 품질만 Low→Epic으로 변경했습니다. 사전에 각 1회만 선언했고 Low 검토 후 Epic을 실행했습니다. **두 실행 모두 종료·전체 속도변화/정지 QA·제어 응답·시간 기록 검증을 통과했지만, 학습 데이터나 실제 차량 승인은 아닙니다.**

| 품질 | native / 카메라 묶음 수 | 벽시계 native / 카메라 묶음 Hz | 시뮬레이션 진행 배율 | tick p99 / 최대 ms | >50 / >100 ms 횟수 | 최고 km/h | 마지막 주행 목표 오차 m |
|---|---:|---:|---:|---:|---:|---:|---:|
| Low | 1462 / 731 | 72.534 / 36.508 | 3.627× | 30.347 / 146.709 | 1 / 1 | 28.551 | 0.823 |
| Epic | 1462 / 731 | 80.723 / 40.601 | 4.036× | 27.332 / 119.022 | 1 / 1 | 28.551 | 0.823 |

![모든 native tick과 단계별 실제 지연](01_wall_timing_all_attempts.png)

## 끊김에 대해 이번 시험에서 확인한 범위

시뮬레이션 시간 기준 native 20 Hz·카메라 10 Hz는 둘 다 누락 없이 유지되었습니다. 벽시계 처리량은 별개이며, 이번에는 Epic이 더 빨랐습니다. 각각 한 번이고 실행 순서가 고정되어 있어 캐시·시스템 변동·순서 효과를 분리하지 못합니다. Epic이 항상 빠르다는 결론은 아닙니다. 각 실행에 100 ms를 넘는 tick 1개가 있었고 모두 그래프에 남겼습니다. GUI 화면 재생 FPS, Autoware, 학습 모델 추론 또는 CPU/GPU 부하는 측정하지 않았으므로 기존 화면 끊김 전체의 원인이 해결되었다고 주장하지 않습니다.

카메라 대기 시간은 순서대로 읽을 때 남아 있던 큐 대기이며 센서 생성 지연이 아닙니다. RPC 단계는 각 기록 뒤의 다음 제어 명령만 포함합니다. 부트스트랩·구간 경계·종료 정리 ACK는 별도 단계로 계측하지 않았습니다. tick 자체는 자기 journal 저장 시간을 제외하며, tick 사이 간격과 전체 관측 처리량은 중간 작업을 포함합니다. **journal 저장 지연은 원 실행 요약의 집계만 보존했고 개별 저장 구간이 없으므로 독립 재계산한 값이 아닙니다.**

## 실제 6카메라·차량 중심 경로

Low와 Epic 원본을 같은 전체 화각·1600×900 배치로 표시합니다. 확인한 순항 PNG에서 Low 노면은 체크무늬이고 Epic은 노면 텍스처가 보입니다. 이 관찰만으로 전체 영상 품질을 승인하지는 않습니다. 경로 그림의 미래 궤적은 실제 나중에 기록된 차량 위치이며 모델 예측이 아닙니다.

### Low

![Low 실제 카메라와 경로](low/whole_recording_accelerated.gif)

[순항 PNG](low/02_measured_cruise.png) · [정지 PNG](low/04_goal_dwell.png) · [영상 출처](low/visual_provenance.json)

### Epic

![Epic 실제 카메라와 경로](epic/whole_recording_accelerated.gif)

[순항 PNG](epic/02_measured_cruise.png) · [정지 PNG](epic/04_goal_dwell.png) · [영상 출처](epic/visual_provenance.json)

각 GIF는 전체 구간에서 카메라 5개 간격으로 147장 선택한 10 fps 가속 미리보기입니다. 위 벽시계 Hz 측정이나 실제 라이브 화면의 FPS와 같지 않습니다. 차량은 경로 패널 중앙에 고정되며 전체 경로 미니맵도 함께 보입니다. Autoware/RViz 라이브 캡처·학습 모델 주행 영상이 아닙니다.

## 근거와 재현

실행 소스 11개는 보관본·실행 HEAD·별도 검토 커밋 `3a069256046dfb8e06a8d068678dc3f88ed3d1c0` 모두 바이트 일치를 확인했습니다. 현재 작업 파일이 바뀌어도 역사적 보관본으로 검증합니다. 계획·Low 종료·중간 검토·Epic 시작의 기록된 시간을 검증했으며 불변 파일 생성시각의 증명으로 확대하지 않습니다. 각 단계·6개 카메라·프레임·시뮬레이션 시각·위상·ACK·준비/주행/정지 꼬리 전부를 검사했습니다. 물리 ±2.9 m/s², runtime +3/−6, 실제 30 km/h 상한은 바꾸지 않았습니다. 전체 future XY, 센서 의미, 영상 품질, 다중 경로 일반화와 학습 데이터 승인은 별도입니다.

[독립 검증 결과](summary.json) · [원본/공개 출처](provenance.json) · [Low 모든 카메라 해시](low_image_hashes.json) · [Epic 모든 카메라 해시](epic_image_hashes.json) · [공개 SHA256](SHA256SUMS)

재현: `python3 scripts/e2e/audit_carla_wall_timing_quality.py <wall_timing_quality_v1 원본> --visual-root <wall_timing_visual_v1> --output-dir <새 폴더>`
원본 자료와 이미 존재하는 출력은 덮어쓰지 않습니다. 공개 JSON의 개인 경로는 가렸으며 원본 SHA는 유지했습니다.
