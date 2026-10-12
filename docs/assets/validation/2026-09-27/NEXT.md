# 다음 개선 순서 — 주행 모델과 지도 모델

<!-- HH_260928 - Point to the newer owned Town13 camera and Town01 turn-candidate outcomes. -->

이 문서 이후의 실행 결과는 [2026-09-28 주행·지도 관문](../2026-09-28/README.md)에 있다. 아래 2026-09-27 결과는 당시 판정으로 보존한다. Town13 카메라는 센서 수집에 성공했으나 렌더 품질이 미달이고, 후속 A→B 위치 비교는 서버 충돌로 중단됐다. Town01 새 우회전도 소유 BasicAgent 촬영 후 연속 이동·종점 기준에 실패했다. 학습 승인 상태는 바뀌지 않았다.

<!-- HH_260906 - Make the next data, training, and release gates explicit without promoting failed candidates. -->

**현재 위치:** [이번 결과 한눈에 보기](README.md). 주행은 학습된 AH가 직진에서는 개선됐지만 우회전 절대 기준을 통과하지 못했고, 새 정답 후보 AO도 전체 6.4초 미래 곡률 기준을 실패했습니다. 지도는 학습 후보 B·C가 기존 AI v2의 Town01 DEV 차선·표지판 보호 기준을 실패했습니다. 따라서 **현재 채택·배포할 새 모델은 없고, 9개 상위/30개 세부 주행 기능 중 학습 모델 폐루프 승인은 0/30**입니다. 지도 의미 분할은 정밀지도 자동 생성의 한 입력일 뿐, 완성된 AMG가 아닙니다.

<!-- HH_260906 - Record the next data gates as completed or failed without advancing model approval. -->

**이번 후속 판정:** [기존 우회전 카탈로그 50개](drive_next_data_plan/README.md)는 9/15 km/h에서 계획한 선행 거리·30초 이동 anchor·6.4초 미래의 정적 길이 예산을 만족하는 파일이 **0개**라 경로 연장 설계 전 수집을 멈췄습니다. [AN/AO 가상 라벨 진단](drive_label_geometry/README.md)은 저속뿐 아니라 실제 회전 실패도 없애므로 원래 거부 판정을 바꾸지 않습니다. 지도는 [Town05 Epic DEV 원본](map_town05_dev_capture/README.md)의 앞 94.31m를 감사했고, [고정 AI v2/C 비교](map_town05_dev_eval/README.md)에서 C 도로·차선 IoU가 +56.73/+53.14pp 높았습니다. **C의 Town01 DEV 실패는 그대로**이며 Town05 표지판은 장당 1–2픽셀이라 판정용이 아닙니다. Town02의 [p00](map_town02_train_scout/README.md)과 [p02](map_town02_p02_train_scout/README.md)는 소형 표지판 프레임이 각각 3/232·0/232장이라 사전 기준 24장에 실패했습니다. p02에는 큰 표지판 정답이 보이지만 별도 목적·물체 QA·표지판 DEV 없이는 편입하지 않습니다. 아직 이번 후속 학습은 하지 않았습니다.

과거 지도 B/C는 Town03을 TRAIN에 사용했지만 기존 주행 연구 분할에서는 Town03이 DEV입니다. 지도 단독 당시 평가 결과는 보존하되, **향후 통합 주행·지도 모델의 site 독립성으로 소급 인정하지 않습니다.** 새 지도 TRAIN은 [Town02처럼 TRAIN으로 잠근 site](map_sign_next_route/README.md)에서 별도로 확인하고, Town05는 DEV로 남겨야 합니다.

지도 모델의 다음 **학습 직전 계약**은 [Town02 객체 검수와 Town05 sign-capable DEV 확보 후 D0/D1 단일 요인 비교](map_next_training_design/README.md)입니다. 두 자료 조건이 통과하지 않으면 표지판 개선 학습을 이미 시작한 것처럼 표시하지 않습니다.

**객체 검수 갱신:** [Town02 p02의 중·대형 표지판 사전 QA](map_town02_p02_object_qa/README.md)는 241개 성분/113개 원본 해상도 프레임, 태그 8 픽셀 95.0012% 검토, 확인 객체 4개·유효 프레임 111장으로 자체 관문을 통과했습니다. 하지만 소형 표지판 기준은 여전히 0/232장으로 실패했고, Town05 표지판 DEV는 아래와 같이 실패해 **TRAIN 편입·표지판 개선 학습·모델 승격은 아직 0**입니다.

<!-- HH_260906 - Keep later sign-DEV and long-route feasibility outcomes visible without promoting data. -->

**후속:** [Town05 우회전 p00 표지판 DEV 정찰](map_town05_sign_dev_scout/README.md)도 원본 무결성은 통과했지만, 보존 195장 중 ≥5px 확인 표지판이 가능한 프레임 **0장**으로 자격 실패했습니다. 이 경로와 기존 Town05 직진을 표지판 DEV로 쓰지 않습니다. 주행 우회전은 [200–230m 신규 카탈로그 소유 검색 계획](drive_long_route_prep/README.md)을 준비했지만, 검색 성공과 별개로 기존 경로 중복·회전 전후 길이·실제 30초 이동/6.4초 미래 QA를 통과해야 합니다.

**표지판 DEV 현실 점검:** [비-TEST 지도 가능성 감사](map_sign_dev_feasibility/README.md)에서도 현재 촬영된 Town05·Town03 경로는 사전 픽셀 기준을 충족하지 못했습니다. Town13은 XODR에 표지판 객체가 많은 *향후* source-editor 후보이지, 현재 실행·가시성이 검증된 DEV가 아닙니다. 이 부족분을 숨기고 기존 TRAIN/TEST를 DEV로 옮기지 않습니다.

**주행 경로 후속:** [Town01 V4 소유 카탈로그](drive_long_route_prep/README.md)는 좌·우회전 각 16개와 lifecycle/source 검사를 완료했습니다. [우회전 정적 감사](drive_long_route_prep/v4_candidate_audit.json)에는 9/15 km/h 길이 예산을 만족하면서 기존 학습 회전과 겹침 플래그가 없는 후보가 있지만, 후보 간 동일 회전 중복도 있어 독립 개수로 세지 않습니다. 다음은 **한 경로의 자체 루프·교차점·신호·site 독립성 확인 후** 원본 BasicAgent 촬영과 전체 미래/차체 차로 QA입니다. 카탈로그 성공은 학습 데이터나 learned closed-loop가 아닙니다.

<!-- HH_260906 - Keep completed raw qualification failures ahead of any new training claim. -->

**최신 실제 우회전 판정:** 그중 Town01 `s0002_p06`을 [사전 등록](drive_long_route_p06_capture/PLAN.md)하고 BasicAgent로 9 km/h **한 번** 수집했습니다. [독립 원본 QA](drive_long_route_p06_capture/QA_v3/QA.md)에서 30–120m 360개 카메라 anchor/35.9초 연속 주행, 여섯 카메라·ACK, 매 anchor 실제 6.4초 미래와 충돌 0은 통과했습니다. 그러나 우회전의 한 0.1초 구간에서 실측 XY 곡률 **0.229889 > 0.200000 rad/m**로 겹치는 63개 미래 anchor가 원래 기준을 위반했고, 당시 actor 바운딩박스가 없어 **차체 전체 의도 차로 포함도 미판정**입니다. 자료 편입·주행 학습·모델 승격은 모두 하지 않았습니다. 향후 원본에 actor 바운딩박스와 실제 OpenDRIVE를 보관하도록 [수집기를 보강](drive_capture_geometry_provenance/README.md)했지만, 기존 실패를 소급 PASS로 바꾸지 않습니다. 저속만 더 낮추면 곡률이 자동 해소된다는 가정도 하지 않습니다. 다음 한 요인 제어/경로 실험에는 원래 9개 motion bound와 차체 차로 기준을 그대로 사용해야 합니다.

**독립 DEV 경로:** [Town03 200–230m 소유 카탈로그](drive_long_route_prep/RESULT_TOWN03_V1.md)는 좌회전 0개, 우회전 1개였으며 우회전은 기존 DEV의 213개 route 점과 완전히 동일했습니다. 새 seed가 새 장면이라는 주장 없이 Town03 추가 촬영은 보류합니다. 따라서 다음 주행 A/B 학습 전 다른 물리적 DEV 교차로나 별도 site를 확보해야 합니다.

<!-- HH_260906 - Record the steering-cap experiment as expert-data progress, not a promoted learned model. -->

**우회전 제어 A/B 후속:** 같은 Town01 `s0002_p06`에서 BasicAgent 정규화 조향 상한만 0.80→0.45로 바꾼 [사전 등록 단회 B](drive_steering_cap_ab/PLAN.md)를 수집하고 [독립 원본 QA](drive_steering_cap_ab/QA_v1/QA.md)를 수행했습니다. 고정 360개 anchor의 최고 XY 곡률은 **0.229889→0.185850 rad/m**, 곡률 위반 미래 창은 **63→0**으로 줄었고 회전·목표 도달도 유지됐습니다. 하지만 저장된 spawned actor 바운딩박스·실제 OpenDRIVE와 720개 native 상태를 사용한 [차체 차로 검사와 확대 그림](drive_capture_geometry_provenance/LANE_GATE.md)은 도로 ID 경계의 초기 오탐 가능성을 분리한 뒤에도 회전 중 의도 차로 샘플 영역 밖 차체를 확인했습니다(최악의 20 Hz 보간 sweep에서 약 **1.105 m²**). **곡률은 통과, 차체 차로는 실패**이므로 데이터 편입·새 주행 학습·기능 승인은 없습니다. 먼저 경로 자체의 차량 치수 적합성과 BasicAgent 추종 오차를 분리해 다음 한 요인을 고릅니다. 같은 site의 단회 BasicAgent A/B는 모델 성능 개선이나 일반화 증거가 아닙니다.

<!-- HH_260906 - Separate the successful expert C qualification checks from independent-data and learned-model approval. -->

**이번 C 후속:** [명목 경로 차체 정적 검사](drive_turn_route_intrinsic/README.md)는 30–120m에서 1,820 위치·1,819 sweep 침범 0이었으나 10cm 안쪽 차로까지 최소 보수 여유가 **2.4cm**뿐이었습니다. 이에 [사전 등록 B/C 단일 요인](drive_waypoint_purge_ab/PLAN.md)으로 BasicAgent `base_min_distance`만 3.0→1.5m로 줄여 한 번 더 수집했습니다. [C 원본 QA](drive_waypoint_purge_ab/QA_v1/QA.md)는 최대 독립 CTE **1.370→0.703m**, 최고 곡률 **0.19146<0.2rad/m**, 차체 원본·10cm 안쪽 차로 검사 **727프레임·726 sweep 모두 위반 0**을 확인했습니다. C는 *필요한 원본 품질 검사에 처음 통과한 단일 전문가 주행*이지, 독립 교차로·site 반복 자료나 학습된 E2E 모델이 아닙니다. 따라서 아직 데이터 패키지 편입·PRO6000 재학습·기능 승인을 하지 않았습니다. 다음은 물리적으로 다른 우회전 TRAIN과 별도 DEV를 확보해 같은 기준을 반복 적용하고, 최소한의 충분한 자료가 갖춰졌을 때 같은 초기 가중치·seed 예산으로 주행 모델을 다시 학습·평가하는 것입니다.

**지도 후속:** [독립 표지판 DEV 재감사](map_dev_next_audit/README.md)에도 검증 가능한 sign-capable DEV는 없었습니다. Town13의 정적 표지판 목록은 카메라에서 보였다는 증거가 아니므로, 별도 소유 원본과 물체 단위 RGB/semantic 검수 없이 표지판 학습·지도 모델 승격을 하지 않습니다.

**Town13 실행 관문:** [source-editor 전용 카탈로그 소유 실행](map_town13_scout_prep/CATALOG_OWNER.md)은 v2에서 통과해 32개 경로를 만들었습니다. [고정 XODR 정적 순위](map_town13_static_sign_rank/README.md)에서 `Sign_Stop`/`Sign_Yield` 기회가 있는 경로는 6개지만 각 1개뿐이고, 첫 좌표계 오류 순위는 무효로 보존했습니다. 이 정적 정보는 카메라 가시성이나 한 주행의 서로 다른 실제 표지판 ≥3개 기준을 증명하지 못합니다. 기존 촬영 owner는 packaged CARLA만 지원하고 지도 probe에도 Town13 범위가 없습니다. 새 경로 검색 규칙·정확한 한 route/SHA·Town13 전용 수집 scope/owner를 **촬영 전에** 잠그고, 그 뒤 단회 RGB/semantic/depth 원본·객체 QA를 해야 합니다. 따라서 아직 독립 표지판 DEV 또는 지도 모델 재학습 자격이 없습니다.

<!-- HH_260906 - Reconcile the completed V4/V5 and Town05 searches with the training gate. -->

**이번 경로 재검증:** [Town01 V4 차체 선별](drive_next_wide_turn_rank/README.md)은 우회전 파일 16개 가운데 새 물리 회전 1곳을 찾았으나 10cm 안쪽 차로의 명목 sweep **70/900 실패**로 수집 허가 0이었습니다. 다른 시작·도착점 분포를 사전 등록한 [Town01 V5](drive_town01_v5_rank/README.md)는 소유 검색에 성공해 우회전 16개/물리 회전 7곳/기존과 다른 2곳을 만들었지만, 한 곳은 고정 30–120m 창 밖이고 다른 곳은 동일 포즈 차로-ID seam을 보존해 재검사해도 10cm sweep **38/900 실패**했습니다. seam 검사기 자체도 아직 실제 episode를 지원하지 않습니다. V4/V5 어느 경로도 새 TRAIN 촬영·학습에 넣지 않았습니다. [Town05_Opt 긴 DEV 검색](drive_long_route_town05_dev_prep/PLAN.md)은 199,536 trace에서 수락 경로 **0개**로 `blocked` 종료, 원본·종료 영수증은 통과했습니다. 따라서 충분한 독립 TRAIN/DEV를 갖췄다고 말할 수 없고, 무작정 같은 조건을 반복하기보다 물리적으로 넓은 경로 탐색 또는 다중 조작 장거리 주행에서 사전 고정한 단일 회전 구간을 추출하는 새 데이터 계약이 필요합니다. 기존 10cm 차체/6.4초 미래/미사용 TEST 기준은 낮추지 않습니다.

**복합 경로 실행 결과:** 그 새 계약의 [Town01 catalog-only 모드와 고정 90m 창](drive_multi_maneuver_window/README.md)을 구현해 소유 CARLA에서 right 경로 16개를 확보했습니다. [전체 정적 검사](drive_town01_multi_rank/README.md)에서 2개 파일이 창 조건을 통과했지만 같은 물리 회전 한 곳이고, 실제 보관 Town01 XODR/Prius를 이용한 10cm 안쪽 차로의 명목 몸체 **25/901**, sweep **42/900**이 밖으로 나와 두 파일 모두 수집 후보에서 제외했습니다. 기존 30–120m 감사의 출력·판정은 유지한 채 [새 창의 원본 SHA 바인딩 선택기](drive_multi_maneuver_window/QA_WINDOW.md)만 준비했습니다. 전체 새 episode QA는 아직 소유 trial/출처/차체 바인딩이 없고 주행·학습도 하지 않았습니다. 다음은 같은 회전을 seed만 달리해 늘리는 것이 아니라 다른 물리 회전과 별도 DEV site를 찾는 일입니다.

**지도 정적 실행 확인:** [Town13 자산 5개 해시 점검](map_town13_scout_prep/OWNER_PREFLIGHT.md)과 소유 카탈로그는 통과했습니다. 정적 점검 도구의 카메라 정찰 `STOP_NOT_EXECUTABLE` 판정은 유지되며, 정확한 한 촬영 route/SHA, Town13 mapping-probe scope/owner, live RGB/semantic 가시성이 아직 비어 있습니다. 이것은 표지판 DEV, 새 지도 학습 또는 완성된 자동 지도 생성의 증거가 아닙니다.

| 순서 | 로컬 PC에서 할 일 | PRO6000에서 할 일 | 다음 단계로 넘어가는 조건 |
| --- | --- | --- | --- |
| 1. 실패 원인 분리 | [AO 실측 조향/휠각/미래 XY](drive_control_ao/MECHANISM_AUDIT.md)와 [지도 C 차선·표지판 오검출](map_town_rehearsal_c/diagnosis/README.md)을 보존하고 재현 | 없음 | 이 단계는 완료. 관측 상관관계를 원인으로 확정하지 않음 |
| 2. 주행 데이터 자격 | 기존 K1/AH와 다른 **이동 중 우회전**의 독립 episode를 소유 CARLA로 수집. 원본 6카메라/20 Hz 상태·10 Hz 동기, 차체 차로 포함, 충돌, 속도·가감속·yaw, **6.4초 전체 미래**를 원래 기준으로 검사. TRAIN·DEV·미사용 TEST는 연속 주행/route/site 단위로 분리 | 미승인 원본 전송·학습 없음 | 위반 0인 충분한 독립 우회전 episode가 있어야 TRAIN 후보. [AO](drive_control_ao/README.md)는 여전히 불합격 |
| 3. 지도 데이터 자격 | 표지판이 실제 픽셀로 충분히 보이는 여러 Town/거리·배경의 RGB+시맨틱을 모으고 라벨을 물체 단위로 검토. Town01 반복 DEV와 분리된 새 DEV를 모델 예측 보기 전에 고정. Town07은 최종 TEST로 계속 봉인 | 허가된 개인 dataset 위치에 해시 검증 후에만 복사 | 새 TRAIN/DEV의 클래스별 픽셀·객체 크기·경로 중복 검사가 통과. Town03의 표지판 140픽셀만으로 일반화 주장 금지 |
| 4. 각각 단일 요인 학습 | 사전에 A/B 데이터·초기 가중치·seed·절대/비퇴행 gate·화면 선택 기준을 문서화 | 개인 `venvs/py312`와 **물리 GPU 0만** 사용. 같은 시작점·데이터/시간 예산으로 주행 전체 모델 및 지도 모델을 각각 비교; GPU 1·시스템 패키지·타 사용자 프로세스 불변 | 주행 우회전 ADE/FDE/속도와 직진 보존, 지도 road/lane/sign 및 새 DEV가 모두 사전 gate 통과. **최소 3 seed와 미사용 TEST** 없이 채택 불가 |
| 5. 통합 | 후보가 위 조건을 통과하면 export 일치·센서→출력 10 Hz·거부/fallback을 검증하고 Autoware **shadow** 화면/로그를 수집. 그 뒤 별도 소유 CARLA에서 학습 출력이 실제 ego를 제어하는 폐루프 시험 | 검증된 checkpoint만 회수; 학습 서버에서 차량 제어 안 함 | expert 주행 영상이나 CPU forward를 learned closed-loop 성공으로 세지 않음. 실차 제어는 추가 폐쇄 시험장·안전 절차 뒤 별도 결정 |

주행 모델의 이번 개선 대상은 주로 **RTE-01 직진 차로 유지, RTE-03 우회전, 종점 정지와 속도 추종의 기본 궤적 품질**입니다. ACC·회피·차선변경·교통규칙 같은 나머지 기능의 새 head는 이 기본 성능과 기능별 정답/독립 검증을 먼저 갖춘 뒤에 진행합니다. 지도는 차선·도로·표지판·신호등 기구의 픽셀 인식 다음에 공간 투영·시간 누적·차선 연결·정지선/신호 상태 관계가 필요합니다. 두 모델을 같은 것처럼 승인하지 않습니다.

**시간·사용량을 읽는 법:** 완료된 실측 학습 시간은 지도 B/C 각각 약 **76초/75초**, 주행 AH **29분 41초/1회**입니다. 새 데이터 양·수집 충돌·검사 실패·GPU 점유에 따라 다음 회차 시간은 달라집니다. 새 장면 수집·라벨 감사에는 최소 수시간이 들 수 있고, 그 전에는 여러 번 학습만 반복해도 성능이 보장되지 않습니다. 대화 토큰 잔량·청구액은 이 저장소에서 측정할 수 없어 숫자로 약속하지 않습니다. 이번에 시작한 GPU 학습은 종료됐으며 후속 학습을 이미 실행 중인 것처럼 표시하지 않습니다.
