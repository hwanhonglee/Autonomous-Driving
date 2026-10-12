# Town12 여섯 scope 정적 우회전 탐색 — 완료

<!-- HH_260929 - Keep static route geometry separate from live and dataset gates. -->

**결과: `OFFLINE_STATIC_COMPLETE`, 수집·편입 승인 없음.** [원래 계획](PLAN.md)과 [V4 사전 등록](preregister_v4.json)은 source Town12 OpenDRIVE SHA256 `26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f`, 1m planner, 우회전 driving connector, `u={60,70,80}m × d={100,120}m`의 여섯 scope를 고정했다. 30–120m 창, 최소 146m, 원래 `carla_vad_30kph_v2` 물리 *형상* profile, 명목 Prius 차체와 10cm inset, 5mm 차로 연결 허용 간격을 변경하지 않았다. 이 profile 이름은 30km/h 주행 승인이 아니다.

[최종 V4 보고서](report_v4.json)는 628/628 junction, 오른쪽 connector 1,542개, endpoint 15,716쌍을 끝까지 검사했다. 원래 물리 형상 profile **PASS 4,707건** 중 명목 Prius 10cm 차체·sweep 결과는 **PASS 30 / FAIL 4,300 / UNSUPPORTED 377**이다. 30개 PASS는 독립 장소 30곳이 아니라 **서로 다른 물리 junction 3곳의 endpoint 변형**이다. 아래 `gate supported`는 명목 PASS 안에서 pose-identical zero-arc 차로 전환이 생산 차로 게이트에 지원되는 수이고, `진입부 신호 기록 없음`은 OpenDRIVE 직접 신호·참조가 없는 수이다.

| 물리 junction | 명목 PASS 경로 변형 | gate supported | 진입부 신호 기록 없음 | 지원 경로 최고 10cm-inset sweep 여유 |
| --- | ---: | ---: | ---: | ---: |
| 3742 | 8 | 4 | 0 | 0.01959m |
| 15824 | 16 | 10 | 0 | 0.13261m |
| 19928 | 6 | 6 | 0 | 0.12620m |

생산 차로 게이트가 지원되는 경로의 **정적 inset 여유순**은 15824 > 19928 > 3742이며, 신호 기록과 live QA를 통과한 승인 순위는 아니다.

30개 모두 30–146m 구간의 다른 junction은 없지만, **진입부와 전체 경로 모두 OpenDRIVE 신호 기록이 있다.** 따라서 신호 기록이 없는 정적 후보는 0개이고, 기록만으로 실시간 신호 상태·정지 정책·법적 적용성을 판정할 수 없다. 최고 여유의 *지원된* 설명용 경로는 junction 15824/connector 15851, u=80m/d=120m, route SHA256 `98ea2396ebb5e89382f218ea4d388816735002bc4b0f21c0344d8fbcf05a229b`, 여유 0.132614m, 길이 221.760m다. 역시 진입부 신호 기록이 있어 승인 경로가 아니다. V4 보고서에는 이 경로의 전체 2D polyline이 없고 추가 CARLA 재구성을 하지 않았으므로, 정확한 경로 개요 PNG를 만들지 않았다.

Junction 3742 **한 곳만** [별도 고정 입력](preregister_junction3742_derivative.json)으로 [파생 재구성](junction3742_derivative.json)했다. 그 [오프라인 2D 경로 예시](junction3742_supported_route_overview.png)와 [시각화 해시](junction3742_visual_provenance.json)는 전체 세 지점 중 한 지점의 특정 경로만 보여준다. [원본 XML 신호 분류](junction3742_approach_signal_classification.json)는 road 1062의 비동적 `StopLine2`와 connector 3745의 오른쪽 회전·lane -1 참조가 road 3743의 동적 `Signal_3Light_Post01` 정의를 가리킴을 기록한다.

[완료 진척 장부](progress_v4.jsonl) SHA256 `22538ac958051c5b3c9baded8344b56a9a30d01adbe3c1ae190dbd8a01af6a62`와 최종 보고서 SHA256 `e4ebf3298c6a625126b98f78039dec3acc4cddbab7c2166883045c95d160c2c9`는 독립 장부/보고서 감사에서 일치했다. V4 source SHA256 `dcdb8a6c654eba3607973b81d61c078abffb667b0004b2eea37b14aeee493a8d`, 사전 등록 SHA256 `50dab0ed7dd87dd486679fed2793bf96c26c5488ce9a6692cc1748bb2dcde9b6`이다. 원래 V1은 차로 연결 판정 불가 예외를 분류하지 못해 보고서 없이 끝났고, V2·V3는 계산시간 때문에 각각 [명시적 종료 기록](v2_stop.json), [명시적 종료 기록](v3_stop.json)을 남겼다. V4는 같은 XODR의 순수 section-bounds XML 파싱만 한 번 재사용했고, 지도 객체도 한 번 생성했다. 이 변화는 범위나 임계값을 바꾸지 않았다.

기존 expert suite의 Town12 `train/source_editor_required` 표기는 정적 TRAIN 후보 역할일 뿐이다. [새 통합 분할 원장](../../../../../learning/config/site_split_guard_20260929.json)은 Town12 주행에 **`offline_train_scout`만** 허용한다. Live XODR·실차체 일치, 연속 주행, 64점/6.4초 미래, 아홉 운동 한계, 실제 충돌·차로·목표 QA, 신호 정책 및 closed-loop는 검증되지 않았다. `capture_authorized=false`, `dataset_admitted=false`, `independent_dev_sites=0`, 학습·TEST 실행 없음. 이 **V4 정적 탐색** 자체는 로컬 XODR 오프라인 계산만 수행해 live CARLA/UE4·SSH·GPU·주행 수집을 하지 않았다. 같은 날 별도로 [source-editor 소유 시작을 한 번 시도](OWNER_PROBE.md)했지만 [렌더 스레드 충돌](LIVE_PROBE_FAILURE.md)로 서버 식별 전 종료됐고 actor·촬영은 0건이었다.
