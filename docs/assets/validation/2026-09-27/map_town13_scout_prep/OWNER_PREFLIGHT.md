# Town13 source-editor 사전 점검 결과

<!-- HH_260906 - Keep source-editor asset readiness separate from owned runtime and camera evidence. -->

**카메라 정찰 판정: `STOP_NOT_EXECUTABLE`.** [사전 점검 도구](../../../../../tools/e2e/audit_town13_source_editor_preflight.py)는 파일을 읽고 SHA256을 확인할 뿐 CARLA·UE4Editor를 시작하거나 기존 서버에 접속하지 않는다. 이 정적 도구의 종료 코드 2는 그대로다. 별도 [소유 카탈로그 실행](CATALOG_OWNER.md)은 이후 실제 Town13을 시작해 32개 경로를 만들었지만, 이 도구 자체가 실행 권한이나 DEV 자료 승인서를 발급하는 것은 아니다.

```bash
python3 tools/e2e/audit_town13_source_editor_preflight.py
# exit code 2 = STOP_NOT_EXECUTABLE; JSON is printed on stdout.
python3 -m pytest -q learning/tests/test_audit_town13_source_editor_preflight.py
```

2026-09-27 로컬 읽기 전용 실행에서 suite의 `town13`은 `validation`, `source_editor_0915_4ws`, `/Game/Carla/Maps/Town13/Town13`으로 일치했다. UE4Editor, 프로젝트, 소스 CARLA Python egg, Town13 `.umap` 및 `.xodr` **5개 파일의 SHA256이 계획에 고정한 값과 일치**했다. 이는 자산 무결성만 뜻한다. 맵 로드·렌더·표지판 RGB 가시성·semantic tag 8 정합은 검증되지 않았다. 단위 테스트는 **5개 통과**했다. 변조 자산, 심볼릭 링크, 다른 Town/해시 경로, validation→test 분할 변경을 거부한다.

카탈로그 전용 [소유 실행기](../../../../../tools/e2e/run_owned_town13_route_catalog.py)는 별도로 구현하고 정적·음성 테스트를 마쳤다. 기본 호출의 source egg 확인도 읽기 전용으로 통과했다. 이후 실제 UE4Editor 시작·Town13 live RPC·카탈로그와 정상 종료까지 통과했다. **카메라 정찰 차단 판정은 계속 유지한다.** [실행 계약과 결과](CATALOG_OWNER.md)에 고정 인자와 결과 영수증을 기록했다.

남은 독립 차단 조건은 네 가지다.

1. Town13 카탈로그 32개 중 카메라 촬영에 사용할 정확한 한 route JSON/SHA256이 아직 별도 사전 등록되지 않았다. 정적 XODR의 표지판 이름만으로 보이는 경로를 증명할 수 없다.
2. 카탈로그 소유 실행의 `ready/after_catalog/stopped` 실제 영수증은 통과했다. 그러나 Town13 **카메라 수집용** owner의 촬영·센서 동기·종료 영수증은 아직 없다. 카탈로그 owner는 촬영하지 않는다.
3. `collect_carla_mapping_probe.py`에는 Town13 전용 연구 scope와 사전 고정 route 해시가 없다. Town01 기본값이나 다른 Town 플래그를 빌려 사용할 수 없다.
4. 에디터 cold-start와 `Town13` live world는 확인됐다. 실제 RGB·semantic·depth 카메라의 240-frame 정합과 표지판 물체별 가시성은 모두 미확인이다.

다음 순서는 **완료된 무카메라 Town13 live route catalog → 경로/SHA 사전 고정 → Town13 전용 mapping probe 소유 실행기·scope 검증 → 단회 촬영 → 원본·객체별 화면 QA**다. 각 단계가 실패하면 중단하고 TRAIN/TEST 경로를 DEV로 바꾸지 않는다. 이 문서와 사전 점검은 새 지도 모델 학습, 독립 DEV 평가, 자동 지도 완성, 실차 사용 증거가 아니다.
