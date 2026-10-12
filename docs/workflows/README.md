# 작업별 시작점

<!-- HH_260906 - Keep human-facing categories separate from canonical runtime paths. -->

<!-- HH_261008 - Link the maintained current status and actual model scope ahead of historical category inventories without creating aliases. -->
현재 작업은 [최신 상태 색인](../assets/validation/2026-10-02/README.md)에서 먼저 확인하세요. [두 모델의 구조와 실제 학습 범위](../assets/validation/2026-10-08/model_scope/README.md), [학습 없이 확인한 결과](../assets/validation/2026-10-08/README.md), [실제 종료 상태 확인 명령](../assets/validation/2026-10-08/workspace_status/README.md)을 구분했습니다. 과거 보고서의 완료와 지금 작업이 실행 중인지는 다릅니다.

<!-- HH_261008 - Link real limited batch implementation without presenting nonlive sensor callbacks as genuine CARLA capture. -->
새로 완료한 [실제 RGB·정답 배치 연결과 모의 센서 수집 검사](../assets/validation/2026-10-08/nontraining_batch_capture/README.md)는 입력 처리의 검증입니다. 새 학습이나 실제 촬영·주행 기능 승인이 아닙니다.

| 폴더 | 목적 |
|---|---|
| [01_driving](01_driving/README.md) | 자체 주행 모델·전체 가중치 학습 |
| [02_mapping](02_mapping/README.md) | 도로 인식·지도 생성·실시간/재생 |
| [03_simulation](03_simulation/README.md) | CARLA·Autoware·센서 수집 |
| [04_results](04_results/README.md) | 최신 결과·과거 자료·영상 |
| [05_maintenance](05_maintenance/README.md) | 환경·테스트·정리 기록 |

[전체 실행 파일 목록](EXECUTABLES.md) · [전체 작업공간 설명](../../WORKSPACE.md)

<!-- HH_260906 - Filesystem shortcuts were retired; category guides now point directly to canonical files. -->
각 폴더의 README에서 실제 코드·결과 파일로 직접 이동합니다. 중복 파일시스템 바로가기 12개는 제거했습니다.
학습 스크립트는 원격 전용 실행 조건이 있으므로 임의 실행하지 마세요.
