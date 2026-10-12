# Town13 카메라 진단 v2 단회 재시도 등록

<!-- HH_260928 - Preserve the source-race failure and preregister one new immutable diagnostic attempt. -->

[첫 시도](../../../../../storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v1/owner_result.json)는 `FAIL`이다. Town13 맵 기동과 정확한 full map 확인까지 성공했으나, 별도 테스트 보강이 실행 중 owner 파일을 변경했다. 소스 아카이브 SHA256 `9ede84204d0e10bc3a60203fefefa05fecc98a86e4fdbe2d2f71cc27f305ecfd`와 당시 작업 파일 SHA256 `5aefef683f23da69d19f6796ec512d7acd4677f453c9cda26464862608a10a6c`가 달랐다. owner가 worker 시작 **전에** 변경을 감지해 RGB/depth/semantic 촬영은 0장이고, 소유 UE4Editor 정지·포트 해제 영수증은 `PASS`다. v1 출력은 삭제·수정·승격하지 않는다.

v2는 [원래 계획](PLAN.md)의 맵·route `town13_right_s0001_p00`·SHA256 `6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b`·240 frame·BasicAgent 설정·진단 전용 목적·판정 기준을 **그대로** 유지한다. 변경점은 소스 아카이브 검사에 카탈로그 JSON과 성공 owner 영수증까지 포함한 것과 **새 출력** `storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v2`뿐이다. 이 결과도 서로 다른 표지판 3개가 없어 지도 DEV 승인에 사용할 수 없다.

재시도 전에 새 owner 코드/collector 코드 및 테스트를 완료하고 편집을 중지한다. 기본 read-only preflight, 고정 owner 테스트, 잠금·2100–2102·기존 CARLA 프로세스와 출력 부재를 확인한다. `--execute`는 그 뒤 단회만 호출하며, 동시에 다른 파일 편집 또는 다른 CARLA 실행을 하지 않는다. 실패해도 v2에서 자동 재시도하지 않는다.
