# Town01 긴 우회전 catalog-only 재시도: V2 사전 선언

<!-- HH_260906 - Retain the failed V1 owner generation and run the exact same search only in a new output root. -->

**사전 상태:** `NOT_RUN`. 이 계획은 v2 CARLA 소유 세션 전에 기록했다. [v1의 import 실패 원본](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/catalog.log)을 수정하거나 지우지 않는다. v1은 route 0개/exit 1이지만 CARLA ready·after_catalog·stopped는 PASS였다. [V1 결과](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v1/owner_result.json)를 성공 기록으로 재분류하지 않는다.

| 고정 항목 | V2 값 |
| --- | --- |
| 소유 진입점 | `bash tools/e2e/run_owned_carla_turn_route_catalog.sh town01` |
| 출력 | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v2/` — 사전 확인 시 부재 |
| 서버 | packaged CARLA `Town01`, profile `packaged_0915`, `127.0.0.1:2100`, `Epic`, 단일 소유/잠금, `client.load_world` 금지 |
| 검색 | `left,right`, seeds `2,3`, seed당 후보 quota `8`, 직렬화 거리 `200–230 m`, 선호 `210 m`, 1 m 샘플, `speed_30kph` physical turn, scenario당 최대 trace `100000` |
| 실행 제한 | CARLA 시작 전에 helper import 검증, catalog worker 1200초 제한, no capture, no training, no TEST payload |
| wrapper SHA256 | `ba78dbc293ad9d4f7fba51e9b458a6aa8bf33e377ed4dbaec5c2962ef9a307a8` |
| 생성기 SHA256 | `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857` |
| turn geometry SHA256 | `94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd` |

수정 범위는 생성기의 top-level `prepare_carla_route` import 실패 시 canonical `scripts.e2e.prepare_carla_route`로 돌아가는 **좁은 fallback**과 wrapper의 시작 전 import 확인뿐이다. 거리·turn profile·seed·quota와 split intent는 v1에서 바꾸지 않았다. 이 가정은 [sourced-env 무서버 회귀 테스트](../../../../../learning/tests/test_prepare_carla_expert_route_catalog.py)가 통과해야 유지한다.

**실행 전/후 판정:** 시작 전에 v2 출력 부재와 workspace lock/port 2100–2102의 외부 소유 부재를 확인한다. 실행 후 `owner_plan.json`, `catalog.log`, `route_catalog.json`, `owner_result.json`, `lifecycle/{ready,after_catalog,stopped}.json`를 읽고 오른쪽 후보 존재·생성기/회전 계약 PASS·source 해시 일치·CARLA 세대 종료를 확인한다. 하나라도 빠지면 또 다른 새 출력 루트를 사전 등록하기 전까지 중단한다. 카탈로그 성공도 수집, 독립 TRAIN episode, 학습 모델 성능을 의미하지 않는다. Town03는 이번 v2 진단과 분리하며 자동으로 시작하지 않는다. 후보의 site/loop/중복/30초 이동+6.4초 미래는 [상위 경로 설계](README.md)의 후속 게이트다.
