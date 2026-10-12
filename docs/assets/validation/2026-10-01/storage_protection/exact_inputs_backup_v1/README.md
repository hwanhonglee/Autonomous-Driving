# 입력 원본 백업과 승인된 로컬 중복본 정리

<!-- HH_261001 - Separate a verified redundant backup from authority to remove local evidence. -->

완료된 W0/W1 20초 시험의 `exact_runtime_inputs` 두 폴더를 개인 서버의
`/home/pro6000/personal/hwanhong/dataset/carla_exact_input_backup_20261001_v1/raw/`
아래에 원래 workspace 상대 경로 그대로 복사했다. 각 폴더는 입력183개와
메타데이터1개다. 합계 **368파일·1,519,696,023바이트(약1.42GiB)**다.

로컬 복사 전, 원격 복사 후, 로컬 복사 후의 모든 상대 경로·크기·SHA256이
일치했다. 링크·비정상 파일을 거부하는 기존 검산 도구도 원격에 함께 보관했다.
[검증 결과](backup_result.json), `local_W0/1_manifest.json`,
`remote_W0/1_manifest.json`은 같은 서버 `receipts/`에도 보관했다.

최초 백업 영수증에는 **로컬 삭제 0개**로 남아 있다. 이후 사용자가 정확히 이 두
중복 폴더의 삭제를 승인했다. 삭제 직전 로컬·서버의368파일과 원래 영수증의
경로·크기·SHA256을 전부 다시 대조한 후, 로컬의 `exact_runtime_inputs` 두 폴더만
삭제했다. [별도 정리 영수증](local_cleanup_result.json)의 삭제 전후 실제 여유 공간
증가는 **1,520,652,288바이트**다. 서버 원본, GIF·PNG·추론/제어/경로 기록,
기존 실패·실험 소스는 유지했다. 후속 검증에 쓰는 W1 150초 입력149개도 유지했다.

복원 시에는 원격 `raw/` 아래의 해당 전체 `storage/.../exact_runtime_inputs`
폴더를 먼저 새로운 로컬 임시 복원 폴더에 복사하고, `verify_tree_manifest.py`로
이 문서의 원래 manifest와 파일별 해시를 대조해야 한다. 그다음 원래 위치가
비어 있는지 확인해 배치한다. 기존 파일 위에 무조건 덮어쓰지 않는다.

이전20초 시험의 원본 입력을 다시 재생하려면 위 절차로 원본을 복원해야 한다.
“원격에 해시만 있다”가 아니라 모든 입력 파일의 실제 복사본이 검증되어 보관된
상태다. 삭제 직전 대조와 삭제 후 상태는 새 영수증에 기록하며 초기 백업 영수증을
덮어쓰지 않는다.
