# C-track 좌회전 Epic 지도 수집 01: 기술 QA 실패

<!-- HH_260929 - Separate a clean owned driving capture from unusable map labels. -->

결론: [CPU 전수 QA](left_owned_01_qa.json)는 `FAIL`이다. 소유 Epic 실행은 종료 코드 0, 240프레임·시뮬레이션 24초·93.63 m 이동·충돌 0으로 끝났고, 실제 좌회전 구간 전/중/후를 지났다. 그러나 지도 모델의 정답으로 필요한 `road=1`과 `lane_marking=24` semantic 픽셀이 **240프레임 전체에서 각각 0개**다. 수집기의 지도 `observed_cells`도 0이다. 이 수집본은 TRAIN에 넣지 않는다 (`dataset_admitted=false`, `train_eligible=false`).

| 검사 | 결과 |
| --- | --- |
| 소유 실행/출처 | `mapping-ctrack-v2`, Epic, `owner_result.exit_code=0`, 보관 소스 18개 해시 일치, live XODR SHA-256 `3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27` |
| 센서/주행 | RGB·semantic·depth 240쌍, 10 Hz, live/replay 일치, 유효 깊이 79.31%, 서로 다른 차량 위치 231개, 충돌 0 |
| 좌회전 구간 | 고정 경로의 `LEFT` 28.29–48.88 m; 실제 차량 위치 기준 전 76프레임·안 48프레임·후 107프레임, 구간 내 진행 20.46 m, 최대 경로 이탈 1.08 m |
| 지도 정답 | `ROAD_LABEL_COVERAGE`, `LANE_LABEL_COVERAGE` 실패: road 비율 0, lane 비율 0, lane 양성 프레임 0 |

RGB에는 포장도로와 흰 차선이 선명하다. 예를 들어 [프레임 0000](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/mapping/frames/0000.png), [회전 후 프레임 0120](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/mapping/frames/0120.png), [마지막 프레임 0239](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/mapping/frames/0239.png)을 보면 된다. 반면 저장된 native semantic BGRA의 태그 채널 분포는 전체 55,296,000픽셀 중 `0: 48,615,964`, `3: 1,757`, `9: 3,791,764`, `11: 2,886,515`뿐이다. 프레임 0120의 화면 아래 중앙 포장도로 픽셀도 태그 `0`이다. 즉 **영상에 도로가 없는 문제가 아니라, 이 커스텀 맵에서 카메라 semantic 출력에 필요한 도로/차선 ID가 나오지 않는 문제**다. 원인이 특정 메쉬의 semantic 태그, 수입 파이프라인, CARLA 클래스 매핑 중 어디인지는 아직 확정하지 않았다. `0`을 통째로 road로 바꾸면 보도·미분류 물체까지 오염시킬 수 있으므로 금지한다.

원본은 [소유 수집 폴더](../../../../../storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/)에 그대로 보존한다. [QA JSON](left_owned_01_qa.json)에 원본 route·summary·records·owner plan/start/ready/result·pre-actor provenance·보관 소스 18개·프레임 파일 480개의 SHA-256이 들어 있다. 주요 매니페스트 SHA-256은 route `2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b`, summary `406b0ef14512e1c0d4fb100f36a2988352dfa12ce23034d99b5b999af43a2678`, records `66c6c6c4360b94de78abd0f9908db0b6aaa2d1238555c053fe24f2783f04b56f`, owner result `240771da5eaf0bf3deaab5d7df55f277e6002c4a53d8f755c1aedfa89468ca6d`다.

다음 결정은 수집 반복이 아니라 **커스텀 맵의 semantic 태깅/메쉬 할당 진단**이다. 같은 설정으로 직진을 촬영해도 학습용 도로·차선 정답이 생긴다고 가정할 수 없다. 태그 원인을 수정한 뒤 별도의 새 소유 수집으로 재검증해야 한다. 이 QA는 소유 출처와 기술적 일관성까지 확인하지만, 외부 서명, 개별 센서 timestamp·pose의 독립 재생, 실차 적합성 또는 지도 TRAIN 편입 권한을 증명하지 않는다.
