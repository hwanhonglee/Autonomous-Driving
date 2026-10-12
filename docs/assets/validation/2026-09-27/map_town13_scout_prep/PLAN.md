# Town13 표지판 DEV 정찰: 실행 전 사전 점검과 단일 시도 계획

<!-- HH_260906 - Stop a Town13 camera scout until a source-editor owner, exact route, and map-specific collector binding are proven. -->

**카메라 DEV 정찰 판정: `STOP_NOT_EXECUTABLE`.** Town13은 지도 모델의 *잠정 독립 DEV 사이트*이지 검증된 표지판 DEV가 아니다. 소스 자산과 [카탈로그 전용 소유 실행](CATALOG_OWNER.md)은 검증되어 Town13 live map에서 경로 32개를 만들었다. 그러나 RGB·시맨틱 영상을 수집하지 않았고, Town13 전용 mapping-probe scope도 없다. 학습·모델 평가·승격은 하지 않았다. 파일 존재 및 OpenDRIVE 이름은 화면 가시성의 증거가 아니다.

## 고정된 출처와 아직 없는 것

| 항목 | 정적 확인 결과 |
| --- | --- |
| suite 역할 | [`carla_expert_suite.yaml`](../../../../../tools/e2e/carla_expert_suite.yaml): `town13`, `split: validation`, `status: source_editor_required`, `source_editor_0915_4ws`, `/Game/Carla/Maps/Town13/Town13`. suite SHA256 `d11ddf689cb22f5db2946d1fd3941460e6cd84eb9dacba04d52d553dfeb0ff1f` |
| 지도 파일 | `/home/a/carla/Unreal/CarlaUE4/Content/Carla/Maps/Town13/Town13.umap` 59,723 bytes, SHA256 `390179369aa348ab97a7dc6a98e31123ecba23bfdd9b4a34f2d87d6061c097b7`; 같은 Town13 디렉터리의 `OpenDrive/Town13.xodr` 89,418,305 bytes, SHA256 `5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f` |
| 실행 의존성 | `/home/a/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor` 존재·실행 가능, SHA256 `331f8ece41d8e09c85e927046a9afa522de2dc0a56affa560849d677b9ccb9e9`; `CarlaUE4.uproject` SHA256 `11b5ca8a37d550f45fcf5b93d15e450b7df944746ea599518cca02d6e8eddde2`; 소스 CARLA Python egg SHA256 `ee7209c5570cb9201e9fbd3cbcefc5b176dd4f6b2289df52bb0bd7dc2c56800b`, suite 명세와 일치. **로드·렌더 성공은 미확인.** |
| 정적 OpenDRIVE 목록 | `<road` 8,381개, `<signal` 4,431개, 그중 `name="Sign_Stop"` 1,062개와 `name="Sign_Yield"` 22개. 이는 XML 항목 수일 뿐 서로 다른 RGB 표지판 객체 수, 카메라 시야, 올바른 CARLA 태그 8을 보장하지 않는다. |
| 경로 | [소유 v2 카탈로그](../../../../../storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2/route_catalog.json)에 32개 경로가 있다. 다음 카메라 정찰에는 영상/모델 출력을 보기 전에 **정확한 한 경로·SHA256을 별도로 사전 고정**해야 한다. |

## 즉시 실행을 막는 세 가지 독립 조건

1. [`run_owned_carla_expert_trial.sh`](../../../../../tools/e2e/run_owned_carla_expert_trial.sh)는 서버를 [`run_carla_map.sh`](../../../../../tools/e2e/run_carla_map.sh)로만 시작한다. 후자는 `${CARLA_ROOT}/CarlaUE4/Content`의 패키지 `.umap`을 요구하여 Town13 소스 `.umap`에 맞지 않는다. 별도 [source-editor 카탈로그 owner](../../../../../tools/e2e/run_owned_town13_route_catalog.py)는 실제 cold-start와 `ready/after_catalog/stopped`를 통과했지만 **카메라 capture owner는 아직 없다**.
2. [`collect_carla_mapping_probe.py`](../../../../../tools/e2e/collect_carla_mapping_probe.py)의 연구 scope는 Town13을 포함하지 않는다. scope가 없으면 `validate_route_binding`은 `Town01`을 요구한다. 다른 Town scope 플래그를 빌려 쓰거나 검증을 꺼서 실행하지 않는다.
3. 카탈로그는 생성됐지만 촬영용 한 route와 SHA256은 아직 잠그지 않았다. XODR 신호 목록만으로 표지판이 보이는 카메라 경로를 증명할 수 없고, 수집 후 결과에 맞춰 경로를 바꿀 수도 없다.

기존 패키지 owner의 SHA256은 `0723336ac4b7649004e5cc2f4bfce95e7c0f12caf941a03839096d04cd9f507b`, packaged launcher `a01c032fc18e0fc8ed9cb00ce7d933adc89495251f25eaa4078438be5d57432e`, mapping probe `ef1248008f30fa2f3ad52751e36ad31813596704d3322e1aef4c9c65d12fd6fb`, route cataloger `7954d90f27e750a9de183aacca5136fef660ed756ee477f4304f792494e43857`이다. 새 카탈로그 owner는 실행 시 원본 바이트를 별도 `provenance/`에 보관한다. 코드가 수정되면 새 SHA를 수집 전에 다시 고정해야 한다.

## 차단 해제 후의 **한 번만** 수행할 정찰 계약

이 절은 **카메라 실행 명령이 아니다.** 카메라 없는 Town13 카탈로그는 별도 owner 계약으로 완료했다. 정확한 경로 파일·SHA256, 최종 collector/owner 코드 SHA256, 신규 출력 폴더를 **카메라 수집 전** 이 계획의 별도 실행 개정본에 기입해야 비로소 촬영 사전 등록이 성립한다. 그 전에는 포트가 비어 있어도 카메라 정찰을 실행하지 않는다.

1. 다른 CARLA 사용자 작업과 겹치지 않는지를 확인한다. source-editor 전용 owner가 workspace lock 및 포트 2100–2102를 독점하고, 이미 점유된 포트·잠금·기존 출력이 있으면 **중단**한다. 타인의 CARLA/UE4Editor를 종료하지 않는다. 소유한 child process group만 정리하고 카탈로그의 `ready/after_catalog/stopped`, 이후 별도 카메라 실행의 `ready/after_capture/stopped` lifecycle, 세 포트 해제, source/archive 해시를 남긴다. 에디터 cold-start 실패나 실제 world 이름 불일치 시 `STOP`하며 다른 맵으로 바꾸지 않는다. `client.load_world`로 운행 중 서버를 전환하지 않는다.
2. **카탈로그 단계 완료:** owner로 시작한 Town13 live map에서 카메라 없이 32개 route를 만들고, 관측 world 이름 `Carla/Maps/Town13/Town13`, client/server `0.9.15-dev-dirty`, spawn 12,478개, `.umap`·`.xodr` 해시 및 종료를 검수했다. [사전 고정한 정적 stop/yield 순위](../map_town13_static_sign_rank/README.md)에서 각 경로의 기회는 최대 1개라, 현재 32개 중 한 주행의 서로 다른 RGB 표지판 ≥3개를 뒷받침할 후보는 없다. **따라서 3번 카메라 단계로 바로 넘어가지 않는다.** 새 경로 탐색 규칙·정확한 한 route SHA/출발·종료 ID를 영상이나 모델 예측 전에 별도 계획에 고정해야 한다.
3. 개정 계획의 정확한 route 및 새 Town13-only collector scope를 검증하는 소유 capture를 **단 한 번** 실행한다. 예정 조건은 Epic, ClearNoon, Prius, 전방 RGB/semantic/depth 각 640×360·90°, 10 Hz, 물리 BasicAgent 240 동기 프레임, 포트 2100–2102, wall timeout 900 s다. 신호등 무시를 선택하면 실행 전에 명시하고 준법성 주장은 금지한다. 출력은 `storage/experiments/validation/2026-09-27/map_town13_sign_dev_scout_epic_v1`처럼 새롭고 사용 전 존재하지 않는 경로로 고정한다. 한 번 실패하면 자동 재시도·기준 완화·다른 route 선택은 하지 않는다.
4. **먼저 수집 무결성:** owner 종료 0, source/route/map SHA 불변, live map/route 일치, lifecycle 세 단계 PASS, 충돌 0, 센서 timestamp·frame ID 연속, RGB/semantic/depth 240쌍 전수 디코드, 모든 보존 XY와 0.25 m 이상 떨어진 프레임 ≥120장, 원본 저장과 replay 일치, 정리 오류 0 및 포트 해제를 요구한다. 실제 주행하지 않은 경로 구간이나 BasicAgent를 학습 모델 주행으로 세지 않는다.
5. **그 뒤 표지판 DEV 자격:** 전방 *전체 화각*의 동일 frame ID RGB·oracle semantic·depth를 시작/중간/끝과 모든 후보 물체 접근 전·중·후 시점에서 확인한다. CARLA `TrafficSigns` 태그 8의 프레임 전체 픽셀 `0 / 1–4 / 5–64 / ≥65` 분포와 객체별 연결/추적을 별도로 기록한다. 중복 시점은 한 객체로 묶고, RGB에서 **서로 다른 물리 표지판 ≥3개**를 확인하며, 그 확인 객체 하나 이상이 semantic 단일 객체 **≥5 px**인 보존 프레임 **≥24장**이어야 한다. 합쳐진 tag-8 픽셀, XODR 이름 1,084개, 같은 물체의 24프레임을 서로 다른 물체 24개라고 세지 않는다. RGB/label 정합이 불확실하거나 반사/스텐실이면 실패로 둔다. 전체 화각 예시 PNG와 원본 frame ID·SHA를 남기고 확대 crop만으로 판정하지 않는다.

성공해도 **독립 DEV 자료의 표지판 자격**만 생긴다. 그 자료를 TRAIN에 섞지 않고, 기존 Town01/Town05 반복 DEV나 Town02 TRAIN과 분리한다. Town04·Town07·Town15 held-out 및 Town06·Town10HD_Opt 예약 TEST를 route 선택·DEV 탐색에 사용하지 않는다. 별도 고정 A/B 모델 평가, sign/road/lane 비퇴행, 미사용 TEST 없이는 새 지도 모델 승격이 불가하며, 이 정찰만으로 자동 정밀지도 생성이나 실차 사용을 주장하지 않는다.
