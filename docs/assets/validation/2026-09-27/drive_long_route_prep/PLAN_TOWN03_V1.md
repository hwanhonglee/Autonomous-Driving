# Town03 DEV 장거리 회전 경로 카탈로그 V1 사전 선언

<!-- HH_260906 - Keep this Town03 search catalog-only and preserve its DEV split without admitting an episode. -->

**사전 상태: `NOT_RUN`.** 기존 prepared 연구 데이터의 Town03 right `s0000_p00`는 `val`이며 `site_id=Town03`이다. 새 seed 1,2를 쓰더라도 같은 Town 안의 물리적 교차로·도로가 겹칠 수 있고, 파일 이름만으로 독립 DEV가 되지 않는다. 이 실행은 새 긴 좌/우회전 **경로 후보 검색**일 뿐 촬영, 데이터 편입, 모델 학습, learned-model 주행이 아니다.

| 고정 항목 | 값 |
| --- | --- |
| 출력 | `storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1/` — 사전 확인 시 부재, 덮어쓰기 금지 |
| 실행 | `systemd-run --user` transient service `autoware-e2e-town03-turn-catalog-v1`, 작업 디렉터리 `/home/a/autoware_e2e`, `KillMode=control-group` |
| 소유 범위 | 기존 `tools/e2e/run_owned_carla_turn_route_catalog.sh town03` 단독; `Town03` packaged map cold start, Epic, 127.0.0.1:2100–2102, workspace runtime lock |
| 검색 | 좌·우회전, seeds 1,2, 각 8 pair quota, 직렬화 경로 200–230 m, 선호 210 m, 1 m sampling, `speed_30kph` 물리 회전 preflight, `max-traces=100000` |
| 제한 | CARLA startup 180초, catalog worker 1200초; `--allow-map-load` 사용하지 않음 |
| wrapper SHA256 | `d6233d9881ebda58d7331f779519ea4bcdb2664c5d852bd3b79d5e00e79455d7` |
| generator SHA256 | `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857` |
| physical turn SHA256 | `94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd` |

실행 전 포트 2100–2102, CARLA/카탈로그 프로세스, workspace lock, 출력 루트를 다시 확인한다. 점유가 나타나면 타인의 프로세스를 종료하지 않고 중단한다. wrapper가 보관하는 source byte archive와 해시를 실행 후 현재 source와 각각 대조한다. `owner_result.exit_code=0`, `route_catalog.status=complete`, Town03/map-load 없음, right preflight PASS, 좌/우 파일 수 및 SHA, `ready`·`after_catalog`·`stopped` 영수증, 포트 해제를 점검한다. 실패하면 원본을 남기고 같은 루트에 재시도하지 않는다.

후보가 있더라도 기존 Town03 right `s0000_p00`의 route geometry, road/lane 연결 및 교차점과 비교한다. 중복·자체 루프·회전 전후 거리·신호/endpoint 위험을 별도 감사할 때까지 **`STATIC_REVIEW_ONLY`, `capture_authorized=false`, `dataset_admission_granted=false`**로 둔다. Town03을 DEV로 유지하고 봉인된 TEST 경로·이미지·라벨은 열지 않는다. 카탈로그 성공은 9/15 km/h에서 30초 실제 이동과 6.4초 미래 라벨을 보증하지 않는다.
