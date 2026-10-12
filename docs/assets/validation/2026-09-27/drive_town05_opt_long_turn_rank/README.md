# Town05_Opt 긴 우회전: 완료 후 오프라인 선별 준비

<!-- HH_260906 - Refuse unfinished owner output and keep packaged-map geometry provisional. -->

**상태: `OWNER_BLOCKED_NO_ROUTE`, 선별 미실행.** Town05_Opt 소유 경로 검색은 [0개 수락·199,536회 추적](../drive_long_route_town05_dev_prep/PLAN.md)으로 끝나 owner exit 1 및 catalog `blocked`를 기록했다. [검사기](../../../../../tools/e2e/audit_town05_opt_long_turn_catalog.py)는 `owner_result.json`, 카탈로그 SHA, source 13개 archive/check ledger, ready/after_catalog/stopped 영수증과 포트 해제까지 모두 확인한 다음에만 right route를 읽는다. 실패한 owner에는 `rank.json`을 생성하지 않고 오류로 종료한다. 따라서 아래 조건은 **추후 새 소유 카탈로그가 정상 완료된 경우에만 적용**된다.

소유 검색이 성공하더라도 **패키지 OpenDRIVE의 명목 경로 형상**만 선별한다. 사용할 `Town05_Opt.xodr` SHA256은 `7edc53b0be840e177598da7a4ed9571146d7b2595ce280d154c90f92e8a48851`로 고정했다. 이것이 실제 주행 때의 `carla.Map.to_opendrive()`와 동일하다는 증거는 아직 없다. Prius 차체는 [Town01 원본의 기록된 actor bbox](../drive_waypoint_purge_ab/QA_v1/C_lane_gate_30_120.json)를 예비 계산에 재사용하며, 실제 Town05_Opt 차량·spawn·rig 일치도 별도로 검증해야 한다.

후보 필터는 한 개 RIGHT block, 회전 전체가 고정 30–120m 창 안에 있음, 경로 선행 30m 이상, 120m 이후 9km/h × 6.4초 = 16m 이상, 0.10m 간격의 명목 차체 및 보수적 swept-body가 원시 sampled lane과 10cm 안쪽 lane에 들어감, 동일 road/lane 회전과 중심 5m 이내인 경로 중복 제거다. [이전 Town05_Opt 우회전 p00](../../../../../storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town05_opt/catalog/turn/routes/town05_opt/right/town05_opt_right_s0000_p00.json)와 같은 물리 회전도 별도 표시한다. p00의 SHA256은 `2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3`다.

카탈로그 소유가 정상 종료된 뒤 재현 명령:

```bash
source tools/e2e/env.sh
python3 tools/workspace_python.py tools/e2e/audit_town05_opt_long_turn_catalog.py \
  --output docs/assets/validation/2026-09-27/drive_town05_opt_long_turn_rank/rank.json
```

`rank.json`의 `capture_authorized=false`, `independent_dev_approved=false`, `dataset_admitted=false`는 오프라인 형상이 통과해도 변하지 않는다. 특히 기존 전역 split에서 Town05_Opt를 TRAIN으로 다룬 기록이 있어 독립 DEV 노출은 해결되지 않았다. 다음 단계는 완료된 owner 자료 선별 → 후보의 live XODR/차체·spawn 확인 → 전체 과거 TRAIN/DEV site exposure 확인 → 필요 시 별도 사전 선언한 단일 BasicAgent 원본 → 실제 30초/64-step/차체 QA다. 이 단계에서 CARLA 서버·GPU·SSH·지도/주행 모델 학습은 실행하지 않았다.

검사기의 순수함수/미완성 owner 거부 [테스트](../../../../../learning/tests/test_audit_town05_opt_long_turn_catalog.py)는 6개 통과했고, 실제 차단 출력은 `STOP_OWNER_BLOCKED`/`rank_created=false`였다.
