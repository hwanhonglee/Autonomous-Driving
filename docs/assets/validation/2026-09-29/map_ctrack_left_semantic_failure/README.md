# C-track 좌회전 지도 촬영: 의미 라벨 부재 진단

<!-- HH_260929 - Separate a successful owned sensor recording from failed map-label eligibility. -->

**결론: 지도 학습 자료로 편입 불가.** 소유 Epic CARLA 세션의 BasicAgent 좌회전은 240개 10 Hz RGB/깊이/시맨틱 프레임을 저장했고, 93.63 m 이동·충돌 0·live/replay 일치·정리 완료를 기록했다. 그러나 전방 영상에서 도로와 흰 차선이 보이는데도 원본 시맨틱의 `Roads=1`, `RoadLines=24` 픽셀은 **240프레임 전체에서 각각 0개**다. 지도 투영 셀도 0개다. 이는 학습 모델 또는 Autoware 주행 화면이 아니라 **CARLA BasicAgent가 수집한 센서 원본**이다.

[차량 중앙 고정 경로·실제 궤적 포함 PNG](full_fov_with_centered_route_and_actual.png) · [차량 중앙 고정 경로·실제 궤적 포함 GIF](sampled_with_centered_route_and_actual.gif) · [원본 RGB/시맨틱만 비교 PNG](full_fov_rgb_vs_raw_semantic.png) · [원본 RGB/시맨틱만 비교 GIF](sampled_full_fov_rgb_vs_raw_semantic.gif) · [전 240프레임 수치 JSON](metrics.json)

PNG는 0·120·239번 프레임의 640×360 전체 RGB와 동일 프레임의 원본 시맨틱 ID를 나란히 표시한다. 시맨틱의 분홍색은 *원본 ID 0*일 뿐 도로로 새로 라벨링한 색이 아니다. GIF도 화면 전체를 유지한 13개 표본을 300 ms 간격으로 재생한 진단용 요약이며, 실제 주행 시간이나 10 Hz 재생 속도를 나타내지 않는다. 입력 240프레임은 [원본 촬영 폴더](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/mapping)에 그대로 있다.

차량 중앙 고정 보조 화면에서는 원본 `records.json`의 BasicAgent 실제 위치가 항상 가운데 흰 삼각형이고 진행 방향이 위다. 하늘색은 소유 촬영에 고정된 `route.json`의 계획 경로, 연두색은 해당 시점까지 실제로 지나간 궤적이다. 이것은 경로·차량 위치를 보기 위한 **기하 정보**만 표시하며, 시맨틱 지도나 학습 모델 출력을 생성·표시하지 않는다. 각 영상 아래의 10 m 원이 시각적 거리 척도다.

## 전수 비교

두 촬영 모두 240×640×360, 총 55,296,000 픽셀의 원본 `semantic[...,2]`을 검사했다. 현재 설치된 `carla.CityObjectLabel`의 `Roads=1`, `RoadLines=24`, `Sidewalks=2`를 코드 실행으로 확인했다.

| 항목 | C-track 좌회전 | 기존 Town05_Opt 기술 비교 자료 |
| --- | ---: | ---: |
| 도로 ID 1 픽셀 / 포함 프레임 | **0 / 0** | 26,269,899 (47.51%) / 240 |
| 차선 ID 24 픽셀 / 포함 프레임 | **0 / 0** | 1,335,355 (2.41%) / 240 |
| 무라벨 ID 0 픽셀 | 48,615,964 (87.92%) | 0 |
| 유효 0.1–40 m 깊이 픽셀 비율, 전 프레임 평균 | 79.31% | 78.55% |
| 수집기 지도 셀 | 0 | 94,574 |

C-track의 나머지 원본 ID는 3이 1,757픽셀, 9가 3,791,764픽셀, 11이 2,886,515픽셀이다. 0·3·9·11 외 ID는 없었다. 깊이는 대체로 유효하므로 **세 카메라 전체가 죽은 상황은 아니다**. 비교용 [Town05_Opt 캡처](../../../../../storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping)는 같은 수집기의 기술적 양성 대조군이지, 새로운 독립 DEV 승인을 뜻하지 않는다.

120번 RGB 프레임의 눈에 보이는 좌측 흰 경계 `(x=150,y=300)`, 우측 흰 경계 `(450,300)`, 검은 도로 중앙 `(320,300)`은 시맨틱 원본에서 모두 ID 0이다. 해당 RGB 값과 모든 프레임별 ID·깊이 비율은 [metrics.json](metrics.json)에 있다. 임의의 RGB 색을 라벨로 바꾸거나 가짜 정답을 생성하지 않았다.

## 원인 단서와 한계

<!-- HH_260929 - Preserve the leading empty path token in the local tagger's folder-index reasoning. -->
이 호스트에 있는 `/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/Tagger.h`의 `GetLabelByPath`는 static mesh `GetPathName()`을 `/`로 나눈 뒤 `StringArray[4]`를 라벨 폴더 이름으로 사용한다. 맨 앞 `/`의 빈 토큰을 포함하면 설치된 `/Game/Carla/Maps/C_track_1_0_7/C_track_1_0_7_Roads_Road_Layer0Node...`에서 이 값은 자산명이 아니라 **`C_track_1_0_7` 폴더**다. 같은 위치의 `Tagger.cpp`는 `Road`, `RoadLine`, `SideWalk`와 **정확히 같은 문자열**만 각각 도로·차선·보도로 분류한다. [30개 자산 경로 감사](../map_ctrack_capture_readiness/SEMANTIC_ASSET_DIAGNOSIS.md)는 노면·표시·보도 메시 모두 이 폴더 규칙을 만족하지 않음을 확인했다. 이는 커스텀 메시가 ID 0으로 렌더링되는 현상과 부합한다.

이는 **소스·자산 경로에 근거한 원인 가설**이다. 읽은 `/home/a/carla` 소스가 실행 중인 packaged CARLA 바이너리와 동일 빌드인지, 모든 도로 메시가 이 경로로 태깅되는지는 아직 확인하지 않았다. 화면과 원본 태그가 함께 보여주는 사실은 라벨 부재이며, 특정 Unreal 수정 지점은 추가 검증이 필요하다. 위 호스트 소스의 SHA256: `Tagger.h` `ba5dd4bc118e06a4e595c4295d9ca4cb53c984ff97da7e5de342e53d1d58cd17`, `Tagger.cpp` `fb71f676e987506e41edcbc6309323d1e2dbb5408fb8ee20addeb0b5708b372b`.

## 판정과 다음 관문

[소유 결과](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/owner_result.json)의 종료 코드 0은 촬영·정리의 성공만 의미한다. 별도 [촬영 QA](../map_ctrack_capture_readiness/left_owned_01_qa.json)는 `ROAD_LABEL_COVERAGE`와 `LANE_LABEL_COVERAGE`로 **FAIL**이다. `dataset_admitted=false`, `train_eligible=false`, `independent_dev_approved=false`를 유지한다. 이 실패를 본 뒤 직진 경로만 바꾸어 재촬영해도 같은 맵의 라벨 문제를 해결한다는 근거는 없다.

다음에는 packaged C-track 메시의 실제 semantic stencil/폴더 규칙을 소유된 빌드에서 확인하고, 수정된 지도 자산을 **별도 버전·새 해시**로 만든 뒤 소량의 정지 카메라 원본에서 Road=1·RoadLines=24를 먼저 확인해야 한다. 그 전에는 직진·회전 지도 TRAIN 촬영을 반복하거나 원본을 학습에 편입하지 않는다. 자산 수정·재빌드·실차 사용은 이번 CPU 진단에서 하지 않았다.

재현 명령(현재 2개 원본을 읽고 새 결과 폴더에만 쓰며 덮어쓰기는 거부): `python3 tools/workspace_python.py tools/e2e/diagnose_ctrack_semantic_capture.py`. 중앙 고정 보조 화면: `python3 tools/workspace_python.py tools/e2e/render_ctrack_left_centered_path.py`(기존 결과 폴더에 새 2개 파일만 추가하며 덮어쓰기는 거부). 이번 실행은 CARLA 서버·GPU·SSH를 사용하지 않았다.
