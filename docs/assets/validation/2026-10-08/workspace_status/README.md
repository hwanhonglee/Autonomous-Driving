# 학습과 실험이 지금 실행 중인지 확인하는 방법

<!-- HH_261008 - Document the first actual bounded read-only status CLI and distinguish curated snapshots from live global discovery. -->

**최신 실제 확인에서는 네 작업의 종료 자료를 확인했습니다. 학습을 새로 시작하지 않았습니다.** [최신 확인 결과 V2](../../../../../storage/experiments/diagnostics/2026-10-08/workspace_status_v4_snapshot_v2_report.json)는 10월8일 08:44에 고정한 목록을 대상으로 로컬에서 읽기 전용 명령을 실행해 얻었습니다. 실행 전후 시계 관측은 모두08:50:31 KST이며, 초보다 정밀한 시작·종료 시각이나 전체 서버 상태를 뜻하지 않습니다.

| 확인한 작업 | 실행 상태 | 결과 |
| --- | --- | --- |
| 주행 CNN 실행 비교 | 종료 자료와 권한 연결 확인 | 100ms 지연 기준 미달 |
| 지도 정답 픽셀 분석 | 종료 자료와 권한 연결 확인 | 분석 완료. 후보 모델 미채택 유지 |
| 기록 영상 전체 렌더 V9 | 종료 자료와 권한 연결 확인 | 947프레임·3PNG·1,836행 완료. 새 주행·기능 합격 아님 |
| 지도 crop 보존 비교 | 종료 자료와 권한 연결 확인 | 정답 완전 잘림3,626→1,821회. 학습·모델 개선 미검증 |

최신 명령은28개 지정 파일의 SHA와 원본 JSON 연결을 읽었습니다. 모든 모델·입력·영상 픽셀을 다시 검증한 명령은 아닙니다. `STORED_COMPLETED_WITH_VERIFIED_METADATA`는 **고정된 종료 자료가 맞다는 뜻**이며, 주행 기능 합격이나 서버 전체 프로세스 조사 결과가 아닙니다. 이 목록 확인의 학습·NN·ROS 새 실행은0회, 정식 승인 기능은 메인0/9·하위0/30입니다.

[V2 독립 지정 메타데이터 검산](../../../../../storage/experiments/diagnostics/2026-10-08/workspace_status_v4_snapshot_v2_independent_postcheck_v1.json)도 완료했습니다. 지정28개 파일의 앞뒤 해시·세대가 실제 보고서 관측과 일치했고, 선언한28개 값도 타입까지 일치했습니다. 도구 작성자가 별도로 저장 자료를 검산한 것이므로 독립 구현·두 번째 시간 실험이라고 주장하지 않습니다. 서버 전체·현재 PID·영상 픽셀을 다시 조사하지 않았고 각 작업의 현재 실행 여부는 이 고정 자료만으로는 UNKNOWN입니다.

이전 [V1 확인 결과](../../../../../storage/experiments/diagnostics/2026-10-08/workspace_status_v4_snapshot_v1_report.json)와 [V1 독립 검산](../../../../../storage/experiments/diagnostics/2026-10-08/workspace_status_v4_snapshot_v1_independent_postcheck_v1.json)은 그대로 보존합니다. V1은07:56 고정 목록·08:04:35–08:04:47 관측 사이에 실행했고, 세 작업·24개 지정 파일·15개 선언값을 확인했습니다. 당시 영상V6는509프레임 용량 실패였습니다. 새V2로 과거 실패를 덮어쓰지 않습니다.

## 터미널에서 같은 자료를 확인하기

아래 명령은 로컬 `/home/a/autoware_e2e`에서 실행합니다. 패키지를 설치하지 않습니다. 이 스냅샷은 현재 PC의 절대 경로를 포함하므로 다른 PC에 clone했다고 같은 경로의 과거 결과가 자동으로 생기는 것은 아닙니다.

```bash
cd /home/a/autoware_e2e
flock -n storage/assets/locks/autoware_e2e_runtime.lock \
  env -i PATH=/usr/bin:/bin USER=a LANG=C.UTF-8 \
  PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' \
  /usr/bin/python3 -B tools/maintenance/report_workspace_status_20261008_v4.py \
  --manifest /home/a/autoware_e2e/storage/experiments/plans/2026-10-08/workspace_status_v4_snapshot_v2/ROOT_curated_manifest.json \
  --manifest-sha256 e830ace7fb34f4088ebabe1e4d6a5c44e7fbef22b5f56a010cd242af4529091a
```

다른 소유 작업이 잠금을 사용 중이면 실행하지 않습니다. 잠금을 지우거나 다른 작업을 종료하지 마세요. 결과는 JSON으로 출력하며, 도구 자체는 파일을 쓰거나 SSH·학습·추론·CARLA·ROS를 실행하지 않습니다. 이번 저장 파일은 ROOT가 출력 문자열을 그대로 보존한 것입니다.

긴 결과에서 상태만 읽으려면 이미 저장된 결과를 다음처럼 확인할 수 있습니다.

```bash
/usr/bin/python3 -B -c 'import json; from pathlib import Path; v=json.loads(Path("storage/experiments/diagnostics/2026-10-08/workspace_status_v4_snapshot_v2_report.json").read_bytes()); [print(e["name"], e["status"], e.get("outcome", e.get("reason"))) for e in v["entries"]]'
```

## 표시의 뜻

- `PREPARED_NOTRUNNING`: 실행 준비만 됐습니다. 학습 중이 아닙니다.
- `RUNNING`: 그 시점의 정확한 PID 세대·부팅 식별자·명령·잠금 보유를 관찰한 상태입니다. 학습 진척이나 계속 실행 중임을 보장하지 않습니다.
- `UNKNOWN_STORED_EVIDENCE`: 지정한 파일 연결이나 값이 맞지 않습니다. 성공으로 해석하지 마세요.
- `STORED_COMPLETED_WITH_VERIFIED_METADATA`: 고정된 결과·독립 검산 기록을 확인했습니다. `outcome`의 실패 여부도 함께 읽으세요.

현재 목록은 ROOT가 수동으로 고른 네 작업입니다. 자동으로 최신 폴더를 탐색하거나 서버 학습 여부를 알아내지 않습니다. 목록을 갱신할 때는 기존 선언을 수정하지 않고 **새 스냅샷과 SHA**를 만들며, 실제 접속 실패나 종료 결과를 실행 중으로 바꾸지 않습니다. 이전 [workspace 관리 명령](../../../../../tools/workspace.py)의 고정된 9월 로그와도 구분합니다.
