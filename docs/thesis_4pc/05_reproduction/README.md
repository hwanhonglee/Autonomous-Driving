# 논문 자료 재현과 원본 보존

<!-- HH_261007 - Document read-only reproduction and fixed-source-path constraints without enabling ROS replay or actuation. -->

원본 bag·manifest·소스는 수정하지 않습니다. PC1 분석 스크립트는 기본 출력 이름으로 다시 실행하면 기존 분석을 덮어쓸 수 있으므로, 새 분석은 별도 작업 사본과 새 출력 이름에서 진행해야 합니다.

1. 먼저 [PC1 원본 분석 안내](../../vils/result/PC1/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/README.md)와 [PC3 복구 안내](../../vils/result/PC3/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/README.md)를 읽습니다.
2. 각 run의 checksum은 해당 run 폴더를 기준으로 검사합니다. 검사는 파일을 읽는 작업이며 재생·발행을 하지 않습니다.
3. `run_id`·PC별 역할·부분 기록·복구 구간을 유지한 새 결과를 만듭니다. 나중 부팅의 기록을 중단된 run 뒤에 이어 붙이지 않습니다.

<!-- HH_261007 - Separate current browsing paths from frozen PC4 consumer paths and require a new reviewed runtime revision. -->

2025 자료는 현재 [홈의 VILS_Test](/home/a/thesis_4pc/VILS_Test)에 있습니다. 과거 PC4 runner에는 이전 `/home/a/VILS_Test`와 기존 `pc4_vils_runs` 입력을 가리키는 절대경로가 남아 있습니다. `--run-directory`로 출력만 바꾸면 입력 경로 문제가 해결되지는 않습니다. 재현하려면 새 실행용 코드에서 현재 입력 경로와 원본 해시를 다시 선언하고 별도로 검증해야 합니다. 과거 스크립트·manifest·보존 소스와 이전 위치를 대신하는 심볼릭 링크는 변경하거나 복원하지 않습니다.

CARLA·Unreal·Autoware·RoadRunner, 센서 보정·지도·모델·venv를 보존합니다. ROS replay·CAN 송신·차량 engage·공유 CARLA map 변경은 이 정리에 포함되지 않습니다.

[새 목록 생성 코드](../../../tools/maintenance/catalog_thesis_4pc_v1_20261007.py)는 파일 위치·크기만 조사합니다. 같은 출력 폴더를 덮어쓰지 않으며, 새 조사에는 새 버전과 새 출력 경로가 필요합니다.

[전체 분류로 돌아가기](../README.md)
