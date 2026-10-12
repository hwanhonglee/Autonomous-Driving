# Town12 source editor 첫 실시간 프로브: 렌더 스레드 충돌

<!-- HH_260929 - Separate observed Town12 render failure from the earlier Town13 sensor crash. -->

**판정: FAIL.** [소유 실행 결과](../../../../../storage/experiments/validation/2026-09-29/town12_source_editor_probe_v1/owner_result.json)는 UE4Editor 시작 중 종료 코드 `-11`을 기록했다. 배우(actor) 생성과 촬영은 모두 0건이며 live Town12 OpenDRIVE를 읽지 못했다. 따라서 [정적 우회전 후보](junction3742_derivative.json)가 실제 CARLA 월드에서 재현된 것이 아니다. 소유 프로세스 종료와 2100–2102 포트 해제는 통과했고, 고정한 소스 자산은 변경되지 않았다. 이 분석은 저장된 로그와 소스를 읽기만 했으며 서버를 다시 시작하지 않았다.

## 로그에서 확인한 범위

[고정 실행 계획](../../../../../storage/experiments/validation/2026-09-29/town12_source_editor_probe_v1/owner_plan.json)은 `/Game/Carla/Maps/Town12/Town12`, source editor 프로필, Vulkan·Epic·화면 밖 렌더를 지정했다. 정적 자산 해시와 XODR 해시는 계획에서 모두 일치했다. [ready 영수증](../../../../../storage/experiments/validation/2026-09-29/town12_source_editor_probe_v1/lifecycle/ready.json)은 10:09:37 UTC에 CARLA RPC 4초 시간 초과를 기록했고, 그 뒤 프로세스 종료를 소유자가 감지했다. 월드의 전체 맵 이름이나 live XODR 일치 영수증은 생성되지 않았다.

[서버 로그](../../../../../storage/experiments/validation/2026-09-29/town12_source_editor_probe_v1/server.log)에는 10:09:34–36 UTC에 Town12 타일 `2_3`, `2_4`, `1_4`를 스트리밍하며 정적 메시를 만들었다고 기록되어 있다(17215–17239행). 10:09:36.363에 거리장 atlas가 `153×128×128`, 5개 객체, backing data 4.1 MB로 할당되었다(17240행). 약 0.18초 뒤 렌더 스레드가 `SIGSEGV`를 냈고, 맨 위 스택은 `FDistanceFieldVolumeTexture::IsValidDistanceFieldVolume()` (`DistanceFieldAtlas.cpp:868`) → `FPrimitiveSceneShaderData` → `UpdateGPUSceneInternal` → `Render_CARLA` 순이다(17241–17255행). UE 소스 `StaticMeshRender.cpp:1940`의 `FStaticMeshSceneProxy::HasDistanceFieldRepresentation()`가 `DistanceFieldData->VolumeTexture.IsValidDistanceFieldVolume()`을 호출한다. 이는 **거리장 데이터가 렌더 장면에 반영되는 경로에서 메모리 접근이 실패했다**는 위치 근거다. 어떤 메시·타일의 어떤 포인터가 잘못되었는지는 로그에 없다. 서로 다른 게임/렌더 스레드의 마지막 출력 시각만으로 특정 타일이나 atlas의 5개 객체 중 하나를 범인으로 확정할 수 없다.

프로젝트의 `Config/DefaultEngine.ini:33`에는 `r.GenerateMeshDistanceFields=False # HH_250321`이 있고 실행 로그 239행은 **인라인 주석까지 포함한 값** `False # HH_250321`을 출력한다. 16643–16654행 등에는 실제 mesh 거리장 빌드, 8991행과 17240행에는 atlas 할당이 기록되어 있다. 따라서 주석이 포함된 설정값의 해석 또는 다른 품질 CVar와의 상호작용은 확인할 만한 **가설**이다. 해당 CVar의 유효 정수값이 실행 중 실제로 무엇이었는지는 로그만으로 확인되지 않는다. 같은 설정을 가진 Town13에서는 작은 atlas를 생성하고도 카메라 수집까지 진행했으므로 이 문자열만으로 충돌 원인을 단정할 수도 없다. 로그는 RTX 3090 Ti를 Vulkan `Device 0`으로 선택했다고 기록한다(595, 1240행). 소프트웨어 `llvmpipe`와 `VendorId` ensure도 시작 초기에 보이지만(854–874행), 선택한 장치는 `Device 0`이며 그 뒤 3분 이상 타일을 읽었다. 로그에 메모리 부족·Vulkan device lost 메시지는 없다. 현재 자료로 GPU 드라이버, 메모리 부족, 특정 mesh asset, atlas 재할당 경쟁 중 하나를 직접 원인으로 확정하지 못한다.

## Town13과 다른 충돌

[Town13 이전 사고 분석](../../2026-09-28/map_town13_skydome_location_aba/CRASH_ANALYSIS.md)은 이미 살아 있던 카메라를 약 4.83 km 옮긴 직후 `ASensor::EndPlay`에서 발생한 `0x18` 역참조다. Town12는 **카메라가 없고 시작 중** `FDistanceFieldVolumeTexture`에서 발생했다. Town13 센서 이동 방지나 sky dome 변경으로 이 Town12 충돌이 해결된다는 근거는 없다. 반대로 Town13은 같은 source editor·Vulkan·Epic 계열 실행에서 거리장 atlas `17×42×26`, 2개 객체까지 생성하고 카메라 수집을 진행한 기록이 있다. 이는 source editor 전체가 항상 시작 실패하는 것은 아님을 보여줄 뿐, Town12의 원인을 특정하지 않는다.

## 다음 진단 순서

1. **실행 없이** Town12 `2_3`, `2_4`, `1_4` 하위 레벨의 정적 mesh 참조와 거리장 데이터/Derived Data Cache 기록을 확인하고, atlas 할당 직전의 메시 목록과 맞춘다. 원본 `.uasset`·UE/CARLA 설치·공용 캐시를 수정하거나 삭제하지 않는다. 현재 로그에 특정 충돌 객체 ID가 없으므로 이 단계가 후보를 좁힐 수는 있어도 직접 원인을 증명하지는 못한다.
2. 정확한 프로세스 PID `1877806`의 crash context/core가 이미 보존되었는지 조회하여 object 주소와 전체 호출 스택을 확보한다. 저장된 `Saved/Crashes` 파일 목록에서는 해당 PID를 찾지 못했고 이 환경에는 `coredumpctl` 실행 파일이 없다. 기존 증거가 없다면 별도 로그/메모리 계측을 사전 등록해야 한다.
3. 추가 실시간 시험이 필요하면 **새 출력 경로와 소유 프로세스**로 한 번에 하나의 렌더 설정만 바꾸는 시작 진단을 설계한다. 첫 후보는 공유 ini를 편집하지 않는 격리된 설정 사본 또는 검증된 실행 인자로 `r.GenerateMeshDistanceFields`의 **실제 유효값**을 읽고, 인라인 주석 제거 전후의 시작 차이를 보는 것이다. 같은 Town12 source 자산 해시, 실제 선택된 GPU, 타일 로드·거리장 atlas, 시작 RPC와 종료 영수증을 비교해야 한다. 화면 품질·semantic 태그가 달라질 수 있으므로 렌더 우회 설정으로 시작만 성공해도 촬영/학습 승인은 별개다. 공유 UE/CARLA 프로젝트를 직접 고치거나 현재 실패 결과를 덮어쓰는 재시도는 하지 않는다.

이후에도 **live 전체 맵 이름 + live XODR 정확 해시**를 먼저 통과해야 한다. 다음으로 신호등/정지선 연계와 적색 준수 계측, 주행·영상 품질을 [신호 인지 수집 점검](SIGNAL_AWARE_CAPTURE_GAP.md)에 따라 검증한다. 현재 `capture_authorized=false`, `training_data_admitted=false`, learned closed-loop 검증 0이다.

고정 원본 SHA-256: 서버 로그 `e60ed068a79e738aeb5a599a53c1a3cc8200d1450cf842b8a484744505cf3d7c`, 계획 `fbf8845a8e9108a9e8357bc740a20fec4beb17803ac0e190ce55c83614817040`, 결과 `9df5a622dd17ef01db07944e3e48e0f279f65fc3afbbf03fa5c4c133e11468a1`.
