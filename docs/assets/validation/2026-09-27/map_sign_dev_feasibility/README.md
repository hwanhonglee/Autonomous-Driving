# 표지판 DEV 확보 가능성: 현재 실행 가능한 경로의 한계

<!-- HH_260928 - Keep this historical feasibility reading linked to later Town13 capture and render failures. -->
후속 현황: [Town13 실제 카메라와 렌더 품질](../../2026-09-28/map_town13_render_sensor_scout/README.md)에서 240쌍을 수집했지만 skydome 경고로 학습·DEV 영상 품질이 미달했고, [위치 A→B→A 진단](../../2026-09-28/map_town13_skydome_location_aba/README.md)은 엔진 충돌로 완료되지 않았다. 아래의 “미실행” 설명은 이 문서 작성 당시의 상태다.

<!-- HH_260906 - Separate observed sign visibility from OpenDRIVE sign inventory and protect sealed TEST towns. -->

**판정: 지금 당장 표지판 성능을 공정하게 확인할 신규 DEV는 없음.** 사전 기준은 다른 site의 RGB로 확인한 서로 다른 표지판 3개 이상과, 확인 객체가 5픽셀 이상인 보존 프레임 24장 이상이다. 아래는 모델 예측을 보고 경로를 고른 결과가 아니라, 소유 원본의 라벨 크기와 비-TEST 원본 지도 자산에 대한 정찰이다.

| 지도 | 현재 근거 | DEV 판정 |
| --- | --- | --- |
| Town05_Opt 직진 p00 | [Epic 원본](../map_town05_dev_capture/README.md) 232장, 태그 8은 장당 1–2픽셀 | 도로·차선 DEV에는 유용하지만 표지판 DEV 실패 |
| Town05_Opt 우회전 p00 | [Epic 원본](../map_town05_sign_dev_scout/README.md) 195장, 태그 8 최대 3픽셀, ≥5픽셀 0장 | 표지판 DEV 실패 |
| Town03 | [기존 독립 캡처](../map_generalization_town03/README.md) 직진·우회전 보존 416장에 태그 8 총 140픽셀, 장당 최대 1픽셀 | 현재 캡처는 실패. 나머지 카탈로그는 형상만 있어 RGB 근거 없음. 과거 C가 Town03을 TRAIN에 사용했으므로 C의 독립 DEV로도 불가 |
| Town13 | [suite 역할](../../../../../tools/e2e/carla_expert_suite.yaml)은 `validation`, `source_editor_required`. Town13.xodr에서 `Sign_Stop` 1,062개, `Sign_Yield` 22개 이름을 확인 | **향후 후보만.** 주행 경로, 640×360 카메라 가시성, 태그 8 정합, 소유 source-editor 실행 모두 미검증 |

Town05의 다른 경로 파일들은 카메라 캡처가 없어 표지판 3개가 보인다고 추정하지 않는다. Town03 XODR의 `Speed_30/60/90` 객체는 도로 스텐실이라 표지판 얼굴 3개로 세지 않는다. Town08·09는 로컬 배포 자산이 없고 Town11은 장식 없는 POC, Town12는 기존 suite TRAIN이다. Town04·07·15 held-out, Town06·Town10HD_Opt 예약 TEST의 RGB·라벨·예측을 DEV 선택에 쓰지 않는다. TEST를 열어 DEV를 찾으면 그 TEST는 더 이상 최종 독립 검증이 아니다.

다음 선택지는 (1) Town13 source-editor 실행 안전성·소유 경로를 **별도** 사전 등록한 뒤 표지판 카메라 정찰을 하거나, (2) 표지판 성능 승격을 보류하고 Town05 도로·차선만을 목표로 한 별도 학습 실험을 사전 등록하는 것이다. 어느 쪽도 현재 표지판 DEV 관문을 통과시킨 것은 아니다. Town02 p02 TRAIN 후보의 [객체 검수](../map_town02_p02_object_qa/README.md)가 완료되어도 DEV 부재는 독립적인 차단 사유다.
