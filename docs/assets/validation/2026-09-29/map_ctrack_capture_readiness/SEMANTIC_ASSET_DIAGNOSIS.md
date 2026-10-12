# CTrack 노면·차선 시맨틱 라벨 0: 자산 경로 진단

<!-- HH_260929 - Record a read-only, hash-pinned asset-path diagnosis and its runtime limits. -->

**판정:** 설치된 CTrack 노면 메시의 자산 경로는 이 호스트의 CARLA 태거가 요구하는 폴더 규칙과 맞지 않는다. 이번 검사는 해당 불일치를 30개 패키지 메시에서 재현했다. 따라서 [좌회전 원본 촬영](../map_ctrack_left_semantic_failure/README.md)의 `Roads=1`, `RoadLines=24`가 전 240프레임에서 0인 현상을 설명하는 **강한 원인 가설**이다. 실행 바이너리와 이 소스의 동일성 및 화면 속 각 메시의 실제 바인딩은 아직 증명하지 않았다. 원본의 ID 0을 도로·차선 정답으로 바꾸지 않았으며 `dataset_admitted=false`이다.

## 재현한 경로 규칙

`Tagger.h`의 `GetLabelByPath`는 `GetPathName()`을 `/`로 나누고 `StringArray[4]`를 폴더명으로 사용한다. 맨 앞 `/` 때문에 첫 토큰은 빈 문자열이다. `Tagger.cpp`의 `TagActor`는 static mesh에 이 규칙을 적용한다. `GetLabelByFolderName`은 `Road`, `RoadLine`, `SideWalk`와 **정확히 일치할 때만** 각각 도로·차선·보도 라벨을 반환한다. 그 밖에는 `None`이고 `SetStencilValue`가 해당 컴포넌트의 semantic custom-depth 렌더링을 끈다. 원본 위치: `/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/Tagger.h:77`, 같은 폴더의 `Tagger.cpp:27`.

| 패키지의 메시 참조 예시 | `StringArray[4]` | 태거가 요구하는 값 | CTrack 고유 참조 / 확인된 StaticMesh 자산 | 폴더 일치 |
| --- | --- | --- | ---: | ---: |
| `/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Road_Layer0Node` | `C_track_1_0_7` | `Road` | 10 / 10 | 0 |
| `/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Marking_Layer0Node` | `C_track_1_0_7` | `RoadLine` | 10 / 10 | 0 |
| `/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Sidewalk_Layer0Node` | `C_track_1_0_7` | `SideWalk` | 10 / 10 | 0 |

`C_track_1_0_7.umap`의 인쇄 가능한 패키지 참조를 고유 경로로 센 결과다. 각 참조와 이름이 같은 packaged `.uasset` 30개가 모두 존재하며 `StaticMesh` 마커가 있다. 한편 실제 양성 대조군 촬영이 사용한 `Town05_Opt.umap`은 `T05_Ground` 하위 레벨을 참조한다. 그 하위 레벨의 참조는 `/Game/Carla/Static/Road/...` 1개, `/Game/Carla/Static/RoadLine/...` 1개, `/Game/Carla/Static/SideWalk/...` 2개이고 해당 `StringArray[4]`가 전부 태거 규칙에 맞는다. 같은 수집 방식의 Town05_Opt 원본에는 도로 26,269,899픽셀, 차선 1,335,355픽셀이 있다. 이는 경로 규칙과 실제 라벨 분포의 일관성을 보여주는 비교이며, CTrack 바이너리 내부 원인을 단독으로 확정하지는 않는다.

## 고정한 입력과 확인 범위

| 입력 | SHA256 |
| --- | --- |
| `/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/Tagger.h` | `ba5dd4bc118e06a4e595c4295d9ca4cb53c984ff97da7e5de342e53d1d58cd17` |
| 같은 위치의 `Tagger.cpp` | `fb71f676e987506e41edcbc6309323d1e2dbb5408fb8ee20addeb0b5708b372b` |
| packaged `C_track_1_0_7.umap` | `d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7` |
| packaged `Town05_Opt.umap` | `89596dd951846a719a7dd71a342a40f090119ff32941337b2f4cf0d4f06a0a82` |
| packaged `Sublevels/Town05_Opt/T05_Ground.umap` | `49f21962c5bcd7b604d00e62c8d9ac2f79936859f8e313a6ba713c48cbaa7893` |
| CTrack `Roads_Road_Layer0Node.uasset` | `c8cd7d0940d073ed912815d740832f2f91406761038d8477b76c2a1febbe8106` |
| CTrack `Roads_Marking_Layer0Node.uasset` | `271403687def81cd3cb828d0dbaafa51692121b7e9aebe47f0e9301e2157bdaa` |
| CTrack `Roads_Sidewalk_Layer0Node.uasset` | `f56e6950e9161805de757ad0cca931aa62cedfe4e20fc96934c0ad6a05114874` |

재현: `python3 tools/e2e/audit_ctrack_semantic_asset_paths.py`. 해시가 바뀌거나 패키지 참조·StaticMesh 파일이 사라지면 진단은 실패한다. [진단 스크립트](../../../../../tools/e2e/audit_ctrack_semantic_asset_paths.py)와 [4개 단위 테스트](../../../../../learning/tests/test_audit_ctrack_semantic_asset_paths.py)는 CARLA 서버·GPU·SSH 없이 실행했다. 테스트 명령: `python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_audit_ctrack_semantic_asset_paths.py` → `4 passed`.

이 검사는 `.umap` 문자열과 packaged `.uasset`를 읽는다. 문자열은 참조 존재의 증거이지, 실행 시 각 화면 픽셀이 어느 컴포넌트에서 왔는지의 증거는 아니다. `/home/a/carla` 소스가 현재 설치된 packaged 실행 파일을 빌드한 정확한 소스라는 증명도 없다. 그러므로 **실측 사실은 라벨 부재**, **소스와 자산으로 지지되는 가설은 폴더 불일치**라고 구분한다.

## 안전한 수정과 검증 순서

1. 현재 packaged CARLA와 CTrack 원본을 그대로 보존하고, 별도 소유 복사본의 Unreal 프로젝트에서 세 노면 메시 계열을 `Carla/Static/Road`, `Carla/Static/RoadLine`, `Carla/Static/SideWalk`에 맞춰 이동한 다음 모든 레벨 참조를 Unreal Editor에서 다시 저장·cook한다. 파일시스템의 `.uasset`만 수동으로 이름을 바꾸면 Unreal 참조가 깨질 수 있다. 새 패키지 이름과 SHA를 기록한다.
2. 자산 이동이 어렵다면, **별도 버전의 CARLA 태거**에 CTrack 전용 메시 경로 매핑을 추가해 빌드하는 방법도 있다. 이는 실행 코드가 달라지므로 빌드 식별자와 수정 범위를 별도로 고정해야 한다. 어느 방식을 택해도 현재 공유 설치본을 덮어쓰지 않는다.
3. 새 소유 세션에서 같은 카메라 위치·화각으로 짧은 정지 촬영을 수행한다. 눈에 보이는 포장면과 차선 픽셀에서 원본 `Roads=1`, `RoadLines=24`를 확인하고 Town05_Opt를 양성 대조군으로 재확인한다. 그다음 별도 직진·좌회전 240프레임에 [촬영 QA](left_owned_01_qa.json)를 적용한다. 새 자료의 TRAIN 편입과 독립 DEV 승인 여부는 이 검사 이후에만 별도 판정한다.

현재 자산은 그대로이며 원본 수집의 `ROAD_LABEL_COVERAGE`, `LANE_LABEL_COVERAGE` 실패도 그대로다.
