# CTrack semantic 태그 복구: 격리 빌드 가능성 감사

<!-- HH_260929 - Bound repair options to versioned copies and preserve the shared CARLA installation. -->

**결론:** 노면·차선이 전 프레임에서 semantic ID 0으로 나온 [실측 실패](../map_ctrack_left_semantic_failure/README.md)를 설명할 강한 [자산 경로 가설](SEMANTIC_ASSET_DIAGNOSIS.md)이 있다. 원본 프로젝트에는 올바른 semantic 폴더로 분류된 CTrack 자산도 남아 있다. 다만 **현재 로컬 디스크 21 GB 여유로는 검증 가능한 전체 프로젝트 복사본(최소 46 GB)이나 설치 패키지 복사본(38 GB)을 만들 수 없다.** 공용 `/home/a/carla` 또는 설치 패키지를 직접 고치지 않는 조건에서 지금 즉시 빌드·실행할 준비는 안 됐다. 이번 감사는 읽기만 했고 CARLA·Unreal 빌드나 실행은 하지 않았다.

## 실제로 있는 자산

| 위치 | 읽기 전용 확인 결과 | 태거의 폴더 토큰 |
| --- | --- | --- |
| 원본 프로젝트 `Content/map_package/Maps/C_track_1_0_7/C_track_1_0_7.umap` | 15,638,436바이트. `/Game/map_package/Static/Road/...` 등 참조가 인쇄 가능한 패키지 문자열에 존재 | `Road` 등 정상 후보 |
| 원본 프로젝트 `Content/map_package/Static/{Road,RoadLine,SideWalk}/C_track_1_0_7` | 각각 10개 StaticMesh `.uasset`, 디스크 2.3/2.3/5.1 MB | 각각 `Road`/`RoadLine`/`SideWalk` |
| 원본 프로젝트 `Content/Carla/Maps/C_track_1_0_7.umap` | 3,927,871바이트. `/Game/Carla/Maps/C_track_1_0_7/...` 메시 참조 | `C_track_1_0_7` |
| 현재 실행 패키지 `CarlaUE4/Content/Carla/Maps/C_track_1_0_7.umap` | 1,473,547바이트. 세 노면 계열 각 10개 모두 같은 map 폴더의 StaticMesh를 참조 | `C_track_1_0_7` |

`Tagger.h:77–81`은 `GetPathName()`을 `/`로 나눈 **인덱스 4**의 폴더를 사용하고, `Tagger.cpp:27–34`는 `Road`, `RoadLine`, `SideWalk`만 해당 라벨로 분류한다. 이 원본 코드와 현재 바이너리의 컴파일 시점 동일성은 증명되지 않았다. 다만 원본 프로젝트와 설치 패키지의 `CarlaUE4-Linux-Shipping` 실행 파일은 SHA-256 `34a97404e1dbf61d2ebfb6dae18970ec5684f65c08c934cc7f65a5bd2b7f5192`로 **바이트 동일**하다. staging 맵의 올바른 폴더 참조와 현재 실행 맵의 잘못된 참조는 분명하지만, 어느 import/cook 단계에서 갈라졌는지는 감사만으로 확정하지 못한다. `MoveAssetsCommandlet.cpp:80–160`은 본래 `Static/<semantic>/<map>`으로 자산을 분류하고 `PrepareAssetsForCookingCommandlet.cpp:544–574`는 그 폴더를 읽도록 설계되어 있다.

<!-- HH_260929 - Bind staging and shipping road graphs without claiming level-transform equivalence. -->
추가 읽기 전용 대조에서 staging의 `Content/map_package/Maps/C_track_1_0_7/OpenDrive/C_track_1_0_7.xodr`와 현재 설치 패키지의 `Content/Carla/Maps/OpenDrive/C_track_1_0_7.xodr`는 SHA-256 `3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27`로 **바이트 동일**했다. 따라서 도로 그래프 입력은 같은 버전이다. 그러나 serialized Unreal level의 actor 배치·월드 변환·spawn, 태거를 실제 실행할 때의 메시 바인딩, live 지도 이름까지 이 해시만으로 동일하다고 할 수는 없다. staging level을 촬영 대체재로 쓰기 전 그 항목을 별도 소유 런타임에서 확인해야 한다.

## 두 복구 경로와 필요한 격리

1. **자산/맵 재생성:** 별도 복사한 Unreal 프로젝트에서 기존 `map_package` staging 맵과 세 `Static` 폴더를 우선 점검한다. 사용할 맵이 현재 CTrack route와 같은 XODR·지형·spawn을 제공하는지 확인한 다음, Unreal AssetTools로 참조를 고치고 새로운 이름으로 level 저장·cook한다. 원본 `.uasset` 30개를 파일시스템에서 단순 이동하거나 설치 패키지에 복사하면 serialized package path와 map 참조가 그대로 남아 깨질 수 있다. staging 경로가 이미 적합하더라도 곧바로 주행 가능한 packaged 맵이라고 주장할 수 없다.
2. **태거 코드 분기:** 별도 버전의 `Tagger.cpp`에서 CTrack의 **정확한 세 StaticMesh 이름 계열과 전체 package prefix**만 허용해 Road=1, RoadLines=24, Sidewalk=2를 반환한다. 폴더 전체를 도로로 묶거나 ID 0을 사후 재라벨링하지 않는다. 변경 소스·빌드 ID·실행 바이너리를 고정해야 하며, 설치 패키지는 monolithic `CarlaUE4-Linux-Shipping` 실행 파일을 사용한다. 소스 `.cpp`나 새 editor `.so`만 설치 폴더에 넣는 방법으로 shipping 동작이 바뀐다고 기대할 수 없다. 별도 빌드와 새 소유 실행 폴더가 필요하다.

원본 `/home/a/carla/Unreal/CarlaUE4`는 **83 GB**이고 `Saved` 30 GB, `Intermediate` 5.8 GB, `DerivedDataCache` 161 MB를 빼도 **46 GB**다. `Content`만 44 GB이며 공통 `Content/Carla/Static` 19 GB, CTrack staging Maps 약 929 MB, 배포 맵의 메시 폴더 약 1.7 GB다. 설치 패키지 `/home/a/carla-autoware-universe/CARLA_0.9.15`도 **38 GB**다. 같은 ext 계열 파일시스템의 남은 공간은 `df -h` 기준 **21 GB**다. 하드링크 복사본은 수정 시 원본 inode를 건드릴 수 있어 격리 방법으로 쓰면 안 된다. 얇은 복사본·overlay·공유 UE4 엔진에 의존하는 방식은 의존 자산과 모든 쓰기 경로를 아직 검증하지 못했다. 빌드 도구 `BuildCarlaUE4.sh`는 프로젝트 설정·Binaries·Intermediate를 쓰고, `Package.sh`는 release 출력물을 삭제/재생성하므로 공용 트리에서 직접 실행할 수 없다. 별도 저장 공간 또는 쓰기 경계가 증명된 격리 환경이 먼저 필요하다.

## 최소 검증 게이트

추가 공간에 버전 복사본을 만든 뒤, 바뀐 파일·원본 및 새 실행 바이너리·맵·30개 메시의 SHA를 기록한다. 새 소유 프로세스를 별도 출력 경로에서 시작해 정확한 CTrack 전체 맵 이름과 live OpenDRIVE 해시를 확인한다. 런타임 태거가 실제로 본 mesh `GetPathName()`과 지정한 semantic ID를 로그로 묶어, 경로 규칙이 **실행 바이너리에서** 적용됐는지 증명한다.

그다음 기존 [좌회전 원본](../map_ctrack_left_semantic_failure/README.md)의 고정 카메라 자세·640×360 전체 화각에서 원본 RGB/semantic 한 쌍을 먼저 촬영한다. 화면 속 도로와 차선 위치에 각각 원본 ID `1`, `24`가 나타나고 다른 물체가 잘못 도로가 되지 않는지 확인한다. Town05_Opt를 양성 대조군으로 확인한 뒤에야 직진과 좌회전의 별도 240프레임 10 Hz 촬영을 새 파일에서 수행하고 [촬영 QA](left_owned_01_qa.json)를 전부 다시 적용한다. 현재 실패 캡처는 그대로 `FAIL`이며 새로운 영상/학습 데이터는 아직 없다.

이번 숫자는 2026-09-29 로컬 `du -sh`·`df -h` 읽기 결과다. staging 맵 SHA-256 `775739b54fe5b6eecf2b2cae6f463d3c9583336e89911b882b9a9281d27407fe`, 원본 프로젝트의 배포 맵 SHA-256 `d820d489ae2f1875223b4c65b57c2f98d26b56ec728bb3d267ca79f6d6a0b617`, `MoveAssetsCommandlet.cpp` SHA-256 `ccb9c3c13872aab3287c530a2ba459b3fa096c341e7d9aac64e1a43ede1254f8`.
