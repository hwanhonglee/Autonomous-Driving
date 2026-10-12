# Town13 정지 카메라 위치 A→B→A 진단: A1 기록 후 source-editor 충돌

<!-- HH_260928 - Preserve the one-run crash and reject an incomplete location contrast. -->

**결론:** [사전 고정 계획](PLAN.md)에 따른 단 한 번의 소유 실행은 `FAIL`이다. A1에서 차량 없는 정지 RGB 카메라 10장을 저장했고 **10/10장에 노란 skydome 경고**와 거의 검은 상부가 남았다. B 위치로 옮긴 직후 Town13 source-editor가 SIGSEGV로 종료되어 **B 0장, A2 0장**이다. 따라서 위치별 화질 차이는 판정하지 못했다. 지도 학습 데이터·독립 DEV·HD map·주행 모델 성능으로 승인한 결과는 없다.

| 관찰 항목 | 실제 결과 |
| --- | --- |
| 원본 프레임 | A1 10/10, B 0/10, A2 0/10; [원본 기록](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/scan/records.json) |
| A1 노란 경고 | 모든 10장 상단 32px에서 조건 충족, 각 2,644 pixel |
| A1 중앙 상부 검은 영역 | 계획의 고정 ROI 평균 **99.4167%** (`max(R,G,B)<20`) |
| 동일 카메라 자세 | A1 10장의 측정 camera transform을 기준 행렬과 비교했음; 차량 액터를 만들지 않았고 phase 전후 차량 없음 검사 |
| 소유 결과 | [owner_result.json](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/owner_result.json) `FAIL`; source·입력 바이트와 source 자산은 불변 `true` |
| 종료 상태 | [stopped 영수증](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/lifecycle/stopped.json)은 `PASS`, 2100–2102 포트는 현재 비어 있음. 다만 owner의 **전체 프로세스 그룹 정리 판정은 `false`**이므로 전체 cleanup `PASS`로 적지 않는다. |

![A1 마지막 원본 카메라: 노란 skydome 경고와 어두운 도로](A1_last_observed_frame.png)

이 PNG는 [저장된 A1_09 원본](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/scan/frames/A1_09.png)을 재인코딩한 같은 640×360 장면이다. 원본 SHA256은 `34b12657a16f3aa453b93d696cad9a94965bf0740b4e36d568da8282e5678d47`이다. A1 시작의 30 tick 안정화 후 10 tick을 10Hz로 기록했다. 이 카메라는 월드에 직접 놓였으므로 차량 부착·이동 카메라였던 기존 v2와 픽셀 단위 동일한 장면은 아니다. A1/B/A2의 **내부 비교**만 목적이었으나 B와 A2가 없어 성립하지 않았다.

## 실패와 다음 판단

[센서 스트림 이동·파괴 순서 및 자동 보고 설정 분석](CRASH_ANALYSIS.md)을 후속으로 추가했다. CARLA/Unreal 설치는 수정하지 않았고, 다음 소유 실행의 *프로세스별* 자동 보고 차단 인자만 준비했다. 소스상 차단 경로는 확인했지만 실제 런타임 효과는 아직 검증하지 않았다.

[source-editor 로그](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/server.log)의 06:30:03 UTC 부분에 `ActorsToRemove 0 ActivesToRemove 1` 직후 `SIGSEGV`가 기록됐다. 스택은 `ASensor::EndPlay` → `FActorRegistry::PutActorToSleep` → `ALargeMapManager::ConvertActiveToDormantActors`를 지나며, B 전환 시 첫 tick에 발생했다. 4.83km 카메라 위치 이동과 대규모 맵의 actor 활성/휴면 처리가 충돌한 **가능성**이 있으나, 정확히 어떤 actor 또는 sky dome 자체가 원인인지는 확인하지 못했다. worker는 10초 RPC timeout과 카메라 destroy/설정 복원 실패를 기록했고, 원본과 [요약](../../../../../storage/experiments/validation/2026-09-28/map_town13_skydome_location_aba_v1/scan/summary.json)은 모두 보존했다. 자동 재실행하지 않았다.

엔진의 CrashReportClient가 크래시 자료를 Epic datarouter로 **자동 전송**했고, 로그에는 전송 성공 `bSucceeded=1`이 기록됐다. 이는 이 실행에서 발생한 외부 전송이며 별도로 요청해 수행한 결과 게시가 아니다.

다음 위치 비교가 필요하다면 먼저 source-editor의 actor 휴면·센서 EndPlay 충돌을 조사하고, A/B를 각각 독립적인 새 소유 실행으로 시작하는 방식을 새로 사전 등록해야 한다. 재실행 전에는 크래시 리포트 자동 전송 설정도 확인해야 한다. 현재 Town13의 원본 화면 품질은 지도 모델 데이터 승인 기준에 미달한다.
