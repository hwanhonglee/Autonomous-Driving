# Town01 긴 우회전 카탈로그 V4 사전 선언

<!-- HH_260906 - Preserve V3's service-environment failure and repair only the CARLA-ready log check. -->

**사전 상태: `NOT_RUN`.** [V3 `owner_result`](../../../../../storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v3/owner_result.json)은 `exit_code=1`, 경로/카탈로그/준비 영수증 없음, CARLA `stopped` PASS다. 소유 사용자 서비스의 기본 PATH에는 VS Code 확장에 묶인 `rg`가 없어 wrapper의 `CARLA_READY` 검색이 실패했다. 실제 `server.log` 첫 줄에는 준비 신호가 있었으므로 CARLA 자체 준비 실패로 분류하지 않는다. **V3를 덮지 않는다.**

V4에서 wrapper의 단일 로그 준비 검사 두 곳만 시스템 기본 `/usr/bin/grep -a -q`로 바꿨다. `-a`는 CARLA 로그의 NUL 바이트에도 텍스트 패턴을 검사하도록 한다. CARLA, 생성기, 카탈로그 검색 플래그·timeout·잠금·종료·TRAIN/DEV 분할 계약은 바꾸지 않는다. 49개 정적/생성기 테스트가 통과했으며 `grep -a`가 실제 V3 `server.log`에서 `CARLA_READY`를 찾는 것을 별도 확인했다.

| 고정 항목 | V4 값 |
| --- | --- |
| 출력 | `storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/` — 실행 전 부재 |
| 서비스 | `autoware-e2e-town01-turn-catalog-v4`, `systemd-run --user`, 작업 디렉터리 `/home/a/autoware_e2e`, `KillMode=control-group` |
| 검색 | V2/V3와 동일한 Town01 Epic, `127.0.0.1:2100`, `left,right`, seeds 2,3, quota 8/seed, 200–230m, `speed_30kph`, 1200초 worker 제한 |
| wrapper SHA256 | `d6233d9881ebda58d7331f779519ea4bcdb2664c5d852bd3b79d5e00e79455d7` |
| 생성기 SHA256 | `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857` |
| turn geometry SHA256 | `94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd` |

실행 후 서비스 상태, `owner_result`, `route_catalog`, 세 lifecycle 영수증, 우회전 후보의 물리 회전 계약, source 해시, 포트 해제를 확인한다. 소유 검색 결과는 **수집·학습·모델 주행·데이터 편입이 아니다.** 성공해도 회전 위치·전후 길이·기존 TRAIN route와 지리 중복을 검사하고 독립 episode 및 30초 이동/6.4초 실제 미래 QA가 뒤따라야 한다. 검색 실패나 1200초 제한 시 즉석으로 같은 출력에서 재시도하지 않는다.
