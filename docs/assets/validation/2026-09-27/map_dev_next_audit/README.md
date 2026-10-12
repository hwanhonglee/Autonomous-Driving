# 지도 모델: 다음 독립 표지판 DEV 관문

<!-- HH_260928 - Distinguish the archived planning stage from later Town13 raw camera failures. -->
후속 현황: [Town13 실제 카메라 정찰](../../2026-09-28/map_town13_render_sensor_scout/README.md)은 원본 240쌍을 남겼지만 렌더 경고로 DEV 품질에 실패했고, [정지 위치 비교](../../2026-09-28/map_town13_skydome_location_aba/README.md)는 B 이동 중 엔진 충돌로 실패했다. 아래의 “아직 실행하지 않았다”는 표현은 이 문서 작성 시점의 기록이다. 독립 sign-capable DEV는 여전히 없다.

<!-- HH_260906 - Keep observed camera evidence separate from map-object inventory, split roles, and model promotion. -->

**현재 판정: `NO_SIGN_CAPABLE_INDEPENDENT_DEV`.** 지도 모델의 도로·차선 진단은 진행할 수 있지만, 표지판 성능 비퇴행을 판정하거나 새 모델을 승격할 독립 DEV는 아직 없다. CARLA 시맨틱 `traffic_sign` 태그 8의 픽셀 수는 RGB에서 확인한 물리 표지판 개수와 다르다. 아래 자료는 새 모델의 학습·추론 결과도, 자동 정밀지도 생성 결과도 아니다.

| 후보 | 실제 카메라·분할 근거 | 이번 판정 |
| --- | --- | --- |
| `Town05_Opt` 직진 p00 | [Epic 640×360 소유 원본](../map_town05_dev_capture/README.md)에서 0.25m 보존 232장, 태그 8은 장당 1–2px | 도로·차선 DEV만 가능. 표지판 관문 실패 |
| `Town05_Opt` 우회전 p00 | [소유 원본·사전 관문 감사](../map_town05_sign_dev_scout/README.md)에서 보존 195장, 태그 8 최대 3px, ≥5px 가능 프레임 **0/195** | 같은 Town05 site의 표지판 DEV 실패. 직진과 독립 site 둘로 세지 않음 |
| `Town03` 보관 직진·회전 | [보존 416장](../map_generalization_town03/README.md)의 태그 8 합계 140px, 프레임당 최대 1px. 과거 지도 후보 C는 Town03을 TRAIN에 사용함 | 표지판 가시성 실패. C의 독립 DEV로 소급 지정 금지 |
| `Town02_Opt` p02 | [중·대형 객체 QA](../map_town02_p02_object_qa/README.md)에서 RGB 확인 표지판 4개·≥5px 보존 프레임 111장, 기존 소형 표지판 조건 0/232장 | **TRAIN 후보만**. 가시성이 좋아도 같은 자료를 독립 DEV로 전용하지 않음 |
| `Town13` | [suite](../../../../../tools/e2e/carla_expert_suite.yaml)는 `validation`, `source_editor_required`. 로컬 XODR의 `Sign_Stop` 문자열 1,062개·`Sign_Yield` 22개는 정적 자산 목록뿐 | 다음 **정찰 후보만**. 경로·렌더링·RGB 가시성·태그 8 정합·소유 실행 미검증 |

이 표의 기존 관문은 **RGB로 확인한 서로 다른 표지판 ≥3개**와 **그 확인 객체가 단일 5px 이상인 보존 프레임 ≥24장**이다. 여러 프레임에서 같은 표지판을 3개로 중복 세거나, 한 프레임의 태그 8 전체 합계를 단일 물체 크기로 대체하지 않는다. Town04·Town07·Town15 held-out 및 Town06·Town10HD_Opt 예약 TEST의 영상·라벨·예측을 DEV 탐색이나 후보 선택에 쓰지 않는다. 과거 C가 Town03을 학습에 사용한 사실은 새 분할 문서로 지워지지 않는다.

## 다음 소유 수집에 앞선 최소 안전 절차

1. **분할·출력 잠금:** Town13을 *잠정* validation site로 고정하고, 소유 source-editor 프로파일, 고유 출력 폴더, 맵/경로·수집 코드 SHA, seed, 날씨, 렌더 품질, 센서 설정, 최대 시간과 종료 조건을 실행 전에 별도 계획에 기록한다. 지도 모델 A/C의 예측을 보며 경로를 고르지 않는다. 정적 XODR의 표지판 이름만으로 카메라 가시성을 보증하거나, 결과가 나쁠 때 Town13 경로를 무한히 재선택하지 않는다.
2. **기동 가능성부터 확인:** suite가 요구하는 로컬 Town13 `.umap`·`.xodr`와 UE4Editor 파일은 존재하지만, 이것은 맵 로드 성공이 아니다. 다른 CARLA 소유 실행과 포트·workspace lock이 겹치지 않을 때만 전용 wrapper로 editor/world/route preflight를 수행한다. 실패 시 기존 서버나 타인 작업을 종료하지 않고 STOP한다.
3. **한 번의 고정 카메라 정찰:** preflight를 통과한 *고정 경로* 하나에서 Epic, 전방 RGB/semantic/depth **640×360, 90°, 10Hz, 240 동기 프레임**을 BasicAgent로 수집한다. 소유 `exit 0`, 실행 전후 source·route SHA, ready/after/stopped lifecycle, 충돌 0, 저장 원본·replay 일치, 240쌍 전수 디코드, 0.25m XY 보존 **≥120장**을 먼저 확인한다. 차량이 도달하지 않은 경로 부분을 관측으로 세지 않는다. 신호등 무시를 사용했다면 준법 주행 증거로 쓰지 않는다.
4. **고정 DEV 판정:** 보존 RGB·시맨틱·깊이를 객체별로 대조해 실제 물리 표지판 ≥3개 및 단일 확인 객체 ≥5px인 보존 프레임 ≥24장을 확인한다. 불명확한 semantic 표지판 픽셀은 양성으로 추정하지 않는다. 실패하면 `NOT_QUALIFIED`로 남기고 기준을 낮추거나 TEST를 대체 DEV로 열지 않는다. 통과해도 이것은 *표지판 DEV 자격*이며, 모델 비퇴행은 같은 고정 평가 코드로 별도 측정해야 한다.

Town13 source-editor 경로는 아직 **실행하지 않았다**. 이 문서는 저장 자료와 로컬 파일 존재만 읽었다. CARLA·GPU·SSH·패키지 설치·새 학습·TEST 열람은 수행하지 않았다. DEV 관문 전에는 [잠긴 D0/D1 지도 학습 제안](../map_next_training_design/README.md)의 표지판 개선 학습과 승격을 보류한다. 대안으로 Town05의 도로·차선만 개선하는 실험을 별도 사전 등록할 수 있으나, 그것을 표지판 개선 또는 자동 지도 완성으로 표현하지 않는다.
