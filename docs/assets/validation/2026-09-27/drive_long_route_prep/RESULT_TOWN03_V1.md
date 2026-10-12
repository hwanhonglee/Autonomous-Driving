# Town03 DEV 장거리 회전 경로 카탈로그 V1 결과

<!-- HH_260906 - Report exhaustive Town03 route-search coverage without mistaking a new seed for an independent DEV trajectory. -->

**결론: `CATALOG_COMPLETE_NO_NOVEL_DEV_ROUTE`; 수집·데이터 편입·학습 금지.** [사전 선언](PLAN_TOWN03_V1.md)대로 Town03 카탈로그 전용 소유 실행을 완료했다. [owner 결과](../../../../../storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1/owner_result.json)는 `exit_code=0`, 13/13 source 원본/보관본 SHA 일치, [catalog](../../../../../storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1/route_catalog.json) `complete`, `ready`·`after_catalog`·`stopped` 모두 PASS다. CARLA 2100–2102 포트와 workspace runtime lock도 해제됐다. 검색기 측 map load는 없었고, `capture_executed=false`, `training_executed=false`, `dataset_admission_granted=false`, `learned_model_control=false`다.

| 시나리오 | 시도한 spawn pair | 물리 조건을 통과한 경로 | 의미 |
| --- | ---: | ---: | --- |
| 좌회전 | 73,048 | 0 | 선언한 200–230 m 및 `speed_30kph` 단일 회전 조건을 만족한 후보 없음 |
| 우회전 | 73,047 | 1 | 새 ID 하나가 생성됐지만 기존 DEV 경로와 동일 |

우회전 [새 `s0001_p00`](../../../../../storage/experiments/validation/2026-09-27/drive_long_route_town03_dev_catalog_v1/routes/town03/right/town03_right_s0001_p00.json)의 경로 길이는 208.985201610 m, spawn 11→48, 회전 선행 150.493362 m·후행 40.859277 m다. 기존 prepared DEV `town03_right_s0000_p00`의 원본 [route.json](../../../../../storage/datasets/raw/carla/common10_v1/2026-09-05/30kph/town03/turn/qualified/b1_compact_lookahead/ClearNoon/seed_0000/run_002/episode/route.json)과 **213개 직렬화 경로 점 전체, 각 XY(최대 차이 0 m), start/goal spawn, `option_counts`, `physical_turn_preflight`가 같다.** 경로 ID/seed 및 JSON 전체 SHA가 달라도 같은 궤적이다. 기존 원본 route SHA256 `47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee`; 새 route SHA256 `eaafd85fedb792433efed7e6f6446f84ed0888a7cacea0b71b23b20fe1778fb9`다. 새 파일 자체 SHA와 catalog 기록, catalog 자체 SHA `678f60a0fb5a096bcc3da19facc5d39ac2d483c69fc81e05cdcc9f58ea1ff4e7`도 독립 재검산 일치했다.

따라서 quota 8/seed가 채워지지 않은 `complete`는 **탐색이 끝났다는 뜻**이지 새 DEV 데이터가 준비됐다는 뜻이 아니다. 같은 경로를 다시 촬영하면 새 session일 수는 있어도 새로운 교차로/경로 검증은 되지 않는다. Town03에서 다른 후보가 필요하면 endpoint 생성 전략 또는 거리·회전 제약을 별도로 사전 설계해 새 출력 루트로 검색해야 한다. 그 전에는 기존 DEV와의 중복을 이유로 이 후보의 `capture_authorized=false`와 `dataset_admission_granted=false`를 유지한다. 봉인 TEST는 열지 않았다.
