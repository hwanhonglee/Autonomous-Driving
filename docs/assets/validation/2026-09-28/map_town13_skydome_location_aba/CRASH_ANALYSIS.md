# Town13 A→B→A v1 충돌 분석과 다음 위치 비교 조건

<!-- HH_260928 - Preserve a read-only source and log diagnosis; no simulator replay or global UE setting change. -->

## 확인된 결과

[v1 실행](README.md)은 A1 정지 RGB 10장만 저장했다. B와 A2는 0장이다. A1 10장 모두 상단의 노란 skydome 경고가 남았으므로 위치에 따른 화질 차이를 판정할 수 없다. [원본 서버 로그](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/server.log)의 2026-09-28 06:30:03 UTC에서 `ActorsToRemove 0 ActivesToRemove 1` 직후 주소 `0x18`을 읽다 `SIGSEGV`가 발생했다. 이번 분석은 로그와 소스만 읽었으며 CARLA를 다시 실행하지 않았다.

## 가장 유력한 코드 경로

[기존 worker](../../../../../tools/e2e/capture_town13_skydome_location_aba.py)는 월드에 직접 만든 RGB 카메라 하나를 A에서 30 tick 안정화와 10장 저장 후 `camera.set_transform(...)`으로 약 4,829.175m 떨어진 B로 이동시키고 첫 tick을 요청한다(196–209행). Town13의 `ALargeMapManager::CheckActiveActors`는 센서에 대해 거리 초과 조건을 제외하지만, 센서가 올라간 타일이 로드되지 않았으면 휴면 대상으로 넣는다(`LargeMapManager.cpp` 761–803행). 실제 스택은 `ConvertActiveToDormantActors` 817행 → `FActorRegistry::PutActorToSleep` 198행 → `FCarlaActor::PutActorToSleep` 159–161행 → 카메라 `ASensor::EndPlay` 120행이다.

소스 `/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorData.cpp` 252–257행은 센서가 잠들 때 `Sensor->MoveDataStream()`으로 스트림을 휴면 레코드에 옮긴다. 이어서 `CarlaActor.cpp` 161행이 원래 센서를 `Destroy()`한다. 그런데 `Sensor.cpp` 118–121행의 `EndPlay`는 이동된 센서 스트림에 무조건 `Stream.GetToken()`을 호출한다. `/home/a/carla/LibCarla/source/carla/streaming/detail/Stream.h` 35–36행의 `token()`은 내부 `_shared_state`를 역참조한다. 이 이동 후 역참조가 `0x18` SIGSEGV와 일치하는 가장 유력한 직접 원인이다. 충돌 전에 개별 휴면 actor ID를 출력하지 않아 타일 ID나 정확한 상태 변화는 확정할 수 없지만, 스택의 종료 대상은 카메라 센서다. sky dome이 이 충돌을 직접 일으켰다는 증거는 없다.

## Epic 자동 전송과 실행 단위 제어

같은 서버 로그에는 시작 중 `ensure` 보고서와 위 `SIGSEGV` 보고서가 각각 Epic datarouter로 전송되고 `bSucceeded=1`을 받은 기록이 있다(06:29:00, 06:30:05 UTC). UE4 4.26의 `/home/a/UnrealEngine_4.26/Engine/Config/BaseEditorSettings.ini` 13–14행은 `[/Script/UnrealEd.CrashReportsPrivacySettings]`의 `bSendUnattendedBugReports=True`를 기본값으로 둔다. `UnixPlatformCrashContext.cpp` 448–501행은 `-unattended` 실행에서 이 값을 `GEditorSettingsIni`로 읽고, 값이 false이며 editor 이외의 업로드 동의도 없으면 CrashReportClient 시작을 건너뛴다.

향후 소유 Town13 source-editor 실행의 [공통 명령 생성기](../../../../../tools/e2e/run_owned_town13_route_catalog.py)에만 `-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:bSendUnattendedBugReports=False`를 추가했다. UE4 `ConfigCacheIni.cpp` 1163–1206행은 이 형식의 명령줄 값을 config cache에 적용하고, 1550–1557행은 그 임시 값을 ini 파일에 저장하지 않도록 한다. `LaunchEngineLoop.cpp` 5369–5374행은 실행 초기에 config를 초기화한다. 이는 소스상 전송 차단 경로이며, 변경 후 실제 UE4Editor 실행 또는 고의 충돌로 검증한 결과는 아니다. CARLA/UE4 설치 파일이나 사용자 전역 설정은 수정하지 않았다.

## 다음 진단 실험의 차단 조건

<!-- HH_260928 - Keep the replacement capture offline until target-tile residency can be observed. -->

현재 CARLA Python API에는 목표 카메라 위치에 대한 타일 거주 상태를 조회하는 경로가 확인되지 않았다. C++ `LargeMapManager.h` 127–140행의 `IsTileLoaded`는 내부 `CurrentTilesLoaded` 집합을 읽고, 124–125행의 `IsLevelOfTileLoaded`는 Blueprint 호출용이다. Python `World.cpp`는 `spectator_as_ego`, `get_spectator`, `spawn_actor`, tick을 제공하지만 이 두 조회 함수를 노출하지 않는다. `CarlaServer.cpp` 539–557행에서 `spectator_as_ego`는 manager에 전달되지만, `world.get_settings()`와 tick 성공만으로 B 위치 타일이 실제로 존재하고 로드되었다고 확인할 수 없다. 따라서 B 위치 센서를 생성할 수 있는 새 실행 runner는 아직 만들지 않았다. 이 조회를 신뢰할 수 있는 소유 실행 경로에서 제공하거나 동등한 직접 증거를 확보하기 전에는 아래 실험을 실행하지 않는다.

## 다음 진단 실험 조건

A, B, A2를 각각 새 source-editor 소유 프로세스와 새 출력 경로에서 수집한다. 각 프로세스는 카메라를 만들기 전에 그 위치로 spectator를 놓고, `spectator_as_ego`와 대상 타일 로드 상태를 확인한 다음 카메라를 해당 위치에 처음부터 생성한다. 카메라가 살아 있는 동안 장거리 `set_transform`을 호출하지 않는다. 매 실행의 맵 이름, 고정 입력 해시, 카메라 자세, 10Hz/날씨/화각, 안정화 tick, PNG 수, 시작·종료 영수증을 사전 등록하고 검증한다. 타일 거주 상태를 확인할 수 없으면 해당 위치 촬영을 중단한다. 세 위치가 모두 정상 종료되어야 위치별 화질 비교를 하고, 정상 하늘과 표지판 객체 검수 전에는 학습·독립 DEV 자료로 승인하지 않는다.

[명령 생성 테스트](../../../../../learning/tests/test_run_owned_town13_route_catalog.py)는 소유 명령의 기존 플래그와 위 인자 하나, 실행 없는 static preflight를 검사한다. 실시간 전송 차단과 B 위치 타일 거주 여부는 다음 소유 실행의 별도 관문으로 남는다.
