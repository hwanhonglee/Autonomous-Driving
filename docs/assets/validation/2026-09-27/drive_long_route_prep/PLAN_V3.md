# Town01 긴 우회전 카탈로그 V3 사전 선언

<!-- HH_260906 - Keep the interrupted V2 immutable and move the same bounded catalog search into a user-owned service. -->

**사전 상태: `NOT_RUN`.** [V2 원본](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v2/owner_result.json)은 `exit_code=1`, `route_catalog.json` 없음, 우회전 후보 0개이다. 8개 좌회전 JSON은 완성된 카탈로그가 아니다. `catalog.log`는 시작 후 약 146초에 `KeyboardInterrupt`가 CARLA 경로 탐색 중 들어온 것을 기록한다. 내부 검색 제한 1200초에 도달한 것은 아니며, 소유 wrapper가 `stopped` PASS로 CARLA를 종료했다. 정확한 외부 중단 주체는 확인되지 않아 실행 세션의 수명 제한은 **추정**이다. V1/V2 출력은 덮거나 학습 데이터로 편입하지 않는다.

| 고정 항목 | V3 값 |
| --- | --- |
| 출력 | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v3/` — 실행 전 부재 |
| 실행 방식 | 현재 터미널 세션과 분리한 `systemd-run --user` 일회성 서비스. 소유 wrapper 자체의 lock·포트 검사·종료 처리 유지 |
| 서버·검색 | V2와 동일: packaged `Town01`, Epic, `127.0.0.1:2100`, seeds `2,3`, `left,right`, quota 8/seed, 200–230m, `speed_30kph`, 1200초 worker 제한 |
| 범위 | 경로 후보 검색만. 수집·학습·TEST 열람·학습 모델 제어 없음 |
| wrapper SHA256 | `e2acfd1f400cb6ad77ac6d1f2e02d2b4f482b0339c9711d5c3912b21e6060410` |
| 생성기 SHA256 | `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857` |
| turn geometry SHA256 | `94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd` |

실행 전 `systemctl --user` 사용 가능, v3 출력 부재, 2100–2102 포트 부재, 다른 소유 작업 부재, 정적 테스트 통과를 확인한다. 일회성 서비스 이름은 `autoware-e2e-town01-turn-catalog-v3`로 고정한다. `systemd-run`은 패키지를 설치하지 않는다. 종료 후 `ActiveState/Result/ExecMainStatus`, `owner_result.json`, `route_catalog.json`, `ready/after_catalog/stopped` 영수증, 우회전 후보와 source SHA를 모두 확인한다. **서비스 시작 자체 또는 좌회전 부분 파일은 성공이 아니다.** 후보가 생겨도 [경로 독립성·원본 motion QA](README.md)를 통과하기 전에는 수집하거나 TRAIN에 넣지 않는다.
