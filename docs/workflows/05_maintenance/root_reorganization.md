# 루트 구조 정리 — 실제 이동 내역

<!-- HH_260906 - Separate a navigation shortcut from an actual evidence relocation. -->

`workflows/`는 별도 저장소가 아니라 기존 코드·결과를 연결하는 안내입니다.
이번에는 아래 루트 항목도 실제로 이동했습니다.

| 이전 위치 | 지금 위치 | 이유 |
|---|---|---|
| `SUMMARY.md` | `artifacts/validation/2026-08-31/legacy_root_matrix/SUMMARY.md` | 과거 테스트 요약이 현재 상태처럼 보이지 않도록 분리 |
| `aggregate.json` | 위 폴더의 `aggregate.json` | 같은 실행의 집계 결과 |
| `matrix_plan.json` | 위 폴더의 `matrix_plan.json` | 같은 실행의 테스트 계획 |
| `maps/` | 위 폴더의 `maps/` | 실제 지도 원본이 아닌 Town별 과거 테스트 상태 |

총 30개 파일을 같은 파일시스템에서 이동하고 SHA256 일치를 확인했습니다.
기록 안의 과거 경로 문자열은 이력 보존을 위해 수정하지 않았습니다. 새로운 실행 입력으로 사용하지 마세요.
기존 matrix 도구는 `--output-root`를 요구하며, 현재 실행 소스에서 옛 절대 `maps/` 경로 참조가 없음을 확인했습니다.
파일이 없던 `files/imagery/`, `files/`는 `rmdir`로 제거했습니다. 데이터 삭제는 없습니다.

검증 기록: [root_reorganization.json](root_reorganization.json).
복구는 이전 위치에 새 파일이 없는지 확인한 뒤 위 네 항목을 원래 위치로 옮기면 됩니다.
빈 폴더는 필요하면 다시 생성할 수 있습니다.

`src/build/install/log`, 모델·데이터셋·설치 환경은 이름을 바꾸지 않았습니다.
이들은 ROS 환경과 학습 manifest의 경로 참조가 있어 단순 폴더 이동 대상이 아닙니다.
