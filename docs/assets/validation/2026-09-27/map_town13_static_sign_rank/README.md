# Town13 정적 표지판 경로 순위 — 카메라 정찰 전 고정 규칙

<!-- HH_260906 - Lock a geometry-only route ranking before viewing any Town13 RGB or model output. -->

이 순위는 Town13 source-editor 소유 카탈로그가 **완료·정상 종료된 뒤에만** 계산한다. OpenDRIVE의 `Sign_Stop`/`Sign_Yield` 기록과 경로 좌표로 **정적 표지판 기회**를 정렬하는 작업이다. 표지판이 카메라에 보이는지, 특정 차량의 진행 방향에 적용되는지, 픽셀이나 객체로 학습할 수 있는지는 확인하지 못한다. 이 결과만으로 RGB 촬영, 독립 DEV 편입, 모델 승격 또는 실차 사용을 승인하지 않는다.

## 완료된 정적 결과

Town13 v2 소유 카탈로그는 `PASS`이며 32개 route, spawn 12,478개, 소유 source·XODR/`.umap` 해시와 `ready/after_catalog/stopped` 영수증이 모두 통과했다. 이 도구는 [v2 결과](rank_v2.json)를 생성했다. **정적 우선 검토 경로는 `town13_right_s0001_p00`**로, road 886의 XODR `Sign_Stop` ID 20662가 경로 시작 후 약 20.876 m, 중심선에서 약 4.599 m에 매핑된다. 같은 고정 규칙에서 정적 표지판 1개를 가진 경로는 총 6개이며, 어떤 경로도 2개 이상을 갖지 않는다. XODR의 `Sign_*` 이름은 `Sign_Stop` 1,062개와 `Sign_Yield` 22개뿐이다. 네 직진 후보는 같은 물리 XODR ID 20436을 재사용하므로 독립 표지판 네 개가 아니다. 이는 앞서 계획한 **한 주행에서 서로 다른 실제 RGB 표지판 ≥3개** 검증을 충족할 근거가 없다. XODR에 없는 렌더 객체가 있을 가능성까지 배제한 것은 아니다.

[경로 개요 PNG](route_overview.png)는 직렬화 경로와 XODR 신호 좌표만 그린 오프라인 그림이다. CARLA/Autoware 주행 화면이나 전방 카메라 영상이 아니다. 현재 고정 32개 경로 중에서는 표지판 DEV 정찰을 바로 실행하지 않고, 별도의 새 카탈로그 탐색 규칙과 촬영 사전 등록을 먼저 정하는 것이 맞다.

## 카탈로그 결과를 보기 전에 고정한 규칙

1. [전용 도구](../../../../../tools/e2e/rank_town13_static_sign_routes.py)는 소유 실행의 `owner_result.json`이 `PASS`, `catalog.status=complete`, 출처 코드 보관 SHA, Town13 소스 OpenDRIVE SHA256 `5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f`, `ready/after_catalog/stopped` 영수증, 포트 해제, 카탈로그 및 **모든** 경로 파일 SHA가 맞지 않으면 결과를 만들지 않는다. 실패한 첫 v1 소유 시도나 v2 부분 파일은 입력으로 쓰지 않는다.
2. 각 XODR stop/yield 신호의 도로 `s,t`를 `line`/`arc` 기하로 XY에 놓는다. XODR의 `y`는 직렬화된 ROS `base_link` 경로와 같은 방향이며, CARLA transform의 `y`만 반대 방향이다. 실제 Town13 road 7638에서 route 시작 `y=-422.453`과 XODR 기준선 `y=-423.296`으로 교차 확인했다. 경로가 밟는 **같은 road ID**의 인접 노드 선분에만 투영한다. 횡방향 중심선 거리 ≤8 m, 경로 시작·끝에서 각각 ≥20 m인 XODR 고유 ID만 센다. 다른 `Signal_*`이나 `StopLine*`은 표지판 수에 포함하지 않는다.
3. 기회 수 내림차순 → `Sign_Stop`/`Sign_Yield` 종류 수 내림차순 → 경로 ID 사전순으로 정렬한다. 기회가 하나도 없으면 `STOP_NO_STATIC_SIGN_OPPORTUNITY`이며 우선 후보는 없다. 하나 이상이면 맨 앞 경로는 **정적 우선 검토 후보**일 뿐 촬영 경로의 승인된 확정본이 아니다.
4. RGB·semantic·depth, 지도 모델 출력 및 주행 성공 여부는 이 순위에 입력하지 않는다. 후보 선정 후에도 별도의 전용 Town13 collector scope/owner와 경로 SHA를 촬영 **전에** 사전 등록하고, 실제 full-FOV RGB 표지판 객체/프레임/라벨 정합을 검증해야 한다.

도구는 완성된 입력을 읽어 같은 폴더의 `rank_v2.json`을 **한 번만 새 파일로** 만든다. 기존 결과는 덮어쓰지 않는다. `rank.json`은 첫 계산에서 XODR `y`를 잘못 반전한 **`INVALID_COORDINATE_FRAME_DO_NOT_USE` 증거**이며 선정에 사용하지 않는다. 실행은 `python3 tools/workspace_python.py -m scripts.e2e.rank_town13_static_sign_routes`이며 CARLA/UE4Editor를 시작하지 않는다. 검증: `python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_rank_town13_static_sign_routes.py`.

표지판의 실제 방향·차로 적용성, 차체 카메라 위치·화각, 3D 가림, 낮/밤 렌더, 표지판 메쉬와 XODR ID 대응, CARLA semantic 픽셀 수는 모두 **미확인**이다. 반대 차로의 가까운 표지판도 정적 기회로 포함될 수 있다. 따라서 이 순위의 수치를 가시 표지판 수로 읽으면 안 된다. 지도 모델 학습 및 30개 주행 기능의 검증 성과로도 계산하지 않는다.
