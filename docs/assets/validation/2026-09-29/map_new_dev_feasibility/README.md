# 다음 지도 A/B의 새 DEV 장소: 현재 자료 적격성

<!-- HH_260929 - Keep repeatedly inspected development data and sealed test maps out of a new independent validation claim. -->

**결론: 현재 보관된 영상 중 새롭게 미사용·품질 합격·통합 분할이 잠긴 DEV는 확인되지 않았다.** 이번 조사는 문서와 manifest만 읽었다. CARLA/UE4, GPU, SSH, 모델 추론, 새 수집 및 봉인 TEST 접근은 하지 않았다. 이 판정은 새 지도 모델의 *승격 관문*에 대한 것이며, 기존 Town05 영상을 진단 화면으로 보는 것까지 금지한다는 뜻은 아니다.

| 장소 | 기존 영상/역할 | 새 독립 DEV 판정 |
| --- | --- | --- |
| Town05_Opt | 도로·차선 DEV 232장과 우회전 195장이 있으나 이미 지도 A/B와 후보 비교에 반복 사용. [계획·후속 기록](../../2026-09-27/map_next_training_design/README.md) | **새 미사용 DEV 아님.** 표지판 DEV 기준도 실패. 같은 점수로 다음 학습 설정을 고르면 평가 오염이 커진다. |
| Town03 | 직진·우회전 416장 원본은 [지도 TRAIN 후보](../../2026-09-27/map_generalization_town03/README.md)였고 과거 C 학습에 노출. 주행 연구에서는 DEV로 사용 | **통합 독립 DEV 아님.** 새 seed/route만으로 이전 Town 노출이 없어지지 않는다. |
| Town13 | suite의 validation/source-editor 지도. 240장 촬영은 [전량 skydome 경고](../../2026-09-28/map_town13_render_sensor_scout/README.md), 후속 카메라 이동은 [엔진 충돌](../../2026-09-28/map_town13_skydome_location_aba/CRASH_ANALYSIS.md) | **현재 영상 품질 불합격.** [타일 관문](../map_town13_tile_residency_gate/README.md)과 crash/전송 통제가 해결되기 전 새 카메라 수집 금지. |
| Town01·Town02_Opt·Town12 | 현재 지도 A/B의 Town01/Town02는 TRAIN, Town12는 [expert suite](../../../../../tools/e2e/carla_expert_suite.yaml)의 TRAIN 후보 | 데이터 역할을 결과를 보고 DEV로 바꾸지 않는다. Town12의 주행 정적 우회전 탐색과 지도 DEV 선정은 별개다. |
| Town04·Town06·Town07·Town10HD_Opt·Town15 | 후속 프로젝트 계획에서 held-out/예약 TEST. expert suite의 초기 `train` 표기와 다른 이력이 있음 | **열지 않는다.** 최신 프로젝트 분할 잠금 없이 suite의 예전 필드만 따라 DEV로 쓰면 누출 위험이 있다. |

## 다음 관문

1. 주행·지도 모델, source session, route family와 이미 평가에 열람한 Town을 포함한 **기계 판독형 통합 분할 원장**을 먼저 고정한다. 초기 suite와 후속 계획이 다를 때는 더 엄격한 held-out 판정을 따른다.
2. Town13을 DEV로 쓰려면 안전한 source-editor 재빌드·무센서 타일 상태·crash 자동 전송 차단을 먼저 증명한다. 그다음 **새 소유 프로세스와 새 출력**에서 640×360 RGB/semantic 전체 화각, 주행 거리, 라벨 정합, 경고 0, 종료/재생 일치 및 서로 다른 물리 구간을 사전 등록해 촬영한다. 기존 경고 240장을 고쳐 쓴 것으로 간주하지 않는다.
3. 그 전의 차선 sampler/손실 실험은 TRAIN-only 내부 비교 및 이미 반복된 Town01/Town05 **탐색 DEV**로만 보고한다. 개선되어도 독립 일반화·HD map·실차·폐루프 승인으로 승격하지 않는다. 다음 A/B 조건은 결과를 보기 전에 등록하고 같은 DEV를 보며 여러 후보를 선택하지 않는다.

출처 고정: [분할 계획](../../2026-09-27/map_next_training_design/README.md) SHA256 `656d38b18d39b1847c2b5ced606140edf97456c530128a1cc965a2358aafe630`; [Town03 원본 감사](../../2026-09-27/map_generalization_town03/README.md) `71137fb1b7980e38dda62a1f5435eca235d6e97f5ecca34e2c801552de93a7fe`; [Town13 촬영 기록](../../2026-09-28/map_town13_render_sensor_scout/README.md) `46ea326dc770642d4dea77c68a113817eb3506298c3bcb99f94f90763fad5351`; [suite](../../../../../tools/e2e/carla_expert_suite.yaml) `d11ddf689cb22f5db2946d1fd3941460e6cd84eb9dacba04d52d553dfeb0ff1f`.
