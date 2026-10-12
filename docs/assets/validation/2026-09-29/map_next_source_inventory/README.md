# 다음 지도 학습용 맵·영상 출처 점검

<!-- HH_260929 - Separate physically present map assets from admitted TRAIN and independent DEV data. -->

**판정: 현재 즉시 편입할 새 지도 TRAIN 또는 독립 DEV 데이터는 없다.** 이 점검은 로컬 파일과 [통합 사이트 분할 원장](../site_split_guard/README.md), [기존 DEV 적격성 기록](../map_new_dev_feasibility/README.md)만 읽었다. CARLA·UE4·GPU·SSH 실행, 새 촬영, 봉인 TEST 영상·라벨·예측 열람은 없었다. 원장의 독립 DEV 승인 수는 **0개**다.

| 후보 | 현재 확인한 출처 | 지도 모델에 넣기 전 관문 |
| --- | --- | --- |
| `C_track_1_0_7` | 설치된 packaged level SHA256 `d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7`, XODR `3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27`. [suite](../../../../../tools/e2e/carla_expert_suite.yaml)는 주행 TRAIN으로 등록한다. | **새 지도 TRAIN 후보**다. 현재 [원장](../../../../../learning/config/site_split_guard_20260929.json)의 지도 역할은 빈 배열이다. 보관된 [expert episode](../../../../../storage/experiments/validation/2026-08-31/c_track_394_to_290_expert/manifest.json)의 여섯 카메라는 `sensor.camera.rgb` JPEG이고, 이 episode에는 짝지은 semantic/depth 카메라가 없다. 따라서 RGB를 기존 7클래스 학습쌍으로 간주할 수 없다. |
| `Town12` | source-editor root level SHA256 `a5d427ba29e69a4f7cb13d776bb4b0accdd5b840f01d6f1b7643c0beca2f2f7d`, XODR `26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f`; 타일 level 파일도 존재한다. | [suite](../../../../../tools/e2e/carla_expert_suite.yaml)의 주행 TRAIN 후보이며 현재 지도 역할은 비어 있다. source-editor의 정확한 live 맵, 타일·하늘·카메라 렌더 품질과 RGB/semantic/depth 정합을 소유 실행에서 확인해야 한다. 오프라인 우회전 경로 탐색만으로 지도 영상 자격이 생기지 않는다. |
| `Town13` | source-editor root level SHA256 `390179369aa348ab97a7dc6a98e31123ecba23bfdd9b4a34f2d87d6061c097b7`, XODR `5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f`. | [기존 240장](../../2026-09-28/map_town13_render_sensor_scout/README.md)이 전량 skydome 경고 영상이다. [타일·센서 crash 관문](../map_town13_tile_residency_gate/README.md)을 통과한 새 소유 촬영, 정상 전체 화각과 라벨 QA, 분할 사전 등록 전에는 DEV가 아니다. |
| `Woraksan_v1_0_3_parking_lot_hegiht_fit` | [suite](../../../../../tools/e2e/carla_expert_suite.yaml)는 packaged `ready`로 적지만, 이 컴퓨터의 기본 `CARLA_ROOT=/home/a/carla-autoware-universe/CARLA_0.9.15` 아래 명시된 `.umap`과 `.xodr`가 모두 현재 없다. | 과거 주행 기록만으로 현재 맵 로딩 가능성을 추정하지 않는다. 활성 런타임의 실제 자산 위치·해시부터 다시 확인해야 한다. 지도 역할도 비어 있다. |

<!-- HH_260929 - Keep alternate levels of one physical site out of a fresh-site validation claim. -->

설치된 `C_track_1_0_7_2`, `_TL`, `_object` 등 변형 level은 같은 C-track 계열이다. 다른 파일명만으로 미사용 독립 DEV 사이트가 되지 않는다. Town04·06·07·10HD_Opt·15는 [원장](../site_split_guard/README.md)에 따라 새 TRAIN/DEV 후보 탐색에서 제외한다. Town03은 과거 지도 TRAIN과 주행 DEV가 충돌하고, Town05_Opt는 이미 반복 평가됐다. Town11은 장식 없는 POC이며 Town08·09 자산은 없다.

## 실행 가능한 다음 순서

1. 지도 **TRAIN** 후보 한 곳을 먼저 선택한다. C-track이라면 정확한 packaged level/XODR와 새 직진·회전 경로, 소유 Epic 프로세스, 카메라 위치·640×360/90°/10 Hz 조건, RGB·semantic·depth·pose의 프레임별 결합, 실패 처리 및 원본 SHA를 촬영 전에 고정한다. 기존 [mapping probe](../../../../../tools/e2e/collect_carla_mapping_probe.py)는 이 세 센서를 지원하지만 허용 Town/경로 해시가 고정돼 있어 C-track에 그대로 실행할 수 없다. 범위를 명시한 별도 collector 변경과 사이트 원장 검토가 필요하다.
2. 새 원본에서 하늘·노면·차선의 RGB와 semantic 라벨을 전후·회전 전체 화각으로 대조하고, live/replay와 이동 거리·프레임 수·타임스탬프·심도·충돌·중복 경로를 감사한다. `Roads`·`RoadLines` 라벨이 실제 C-track 재질에서 유효한지 먼저 확인한다. 관문 실패 영상은 TRAIN에 넣지 않는다.
3. **독립 DEV**는 별도 장소로 확보한다. Town13을 고치려면 위 타일/crash/경고 관문과 새 영상 QA를 먼저 통과하고, 촬영 및 모델 평가 전에 주행·지도 통합 분할을 잠근다. 그렇지 않으면 기존 19곳 밖의 새 장식 맵과 provenance를 확보해 사전 등록해야 한다. 새 DEV 없이 추가 학습을 하더라도 결과는 TRAIN 내부 비교/반복 DEV 탐색으로만 기록한다.

이번 점검은 파일 존재와 과거 기록의 대조다. 새 센서 프레임이나 모델 예측을 만들지 않았고, `capture_authorized`, `dataset_admitted`, `independent_dev_approved`를 승인하지 않는다.

<!-- HH_260929 - Preserve the inventory timestamp while linking later collection and repair findings. -->
후속 실제 C-track 좌회전 수집은 RGB 240장을 만들었으나 [노면·차선 semantic 라벨 0으로 QA 실패](../map_ctrack_capture_readiness/LEFT_OWNED_01_RESULT.md)했다. [격리 복구 가능성 감사](../map_ctrack_capture_readiness/REPAIR_FEASIBILITY.md)는 원본 staging 맵의 올바른 semantic 자산과 실행 맵의 다른 참조를 확인했지만 공유 프로젝트를 수정하지 않았다. Town12는 [첫 source-editor 시작이 렌더 스레드 충돌](../drive_town12_right_static/LIVE_PROBE_FAILURE.md)로 끝나 live 지도/영상이 없다. 별개로 Town02 p00 보관 원본을 이용한 [한정 A/B 연구](../map_town02_p00_route_holdout/AUDIT.md)는 실행됐으나 Town01 퇴행으로 미채택이며 독립 DEV 수는 여전히 0이다.
