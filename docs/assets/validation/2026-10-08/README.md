# 10월 8일 비학습 개선: 주행 지연과 지도 정답 확인

<!-- HH_261008 - Keep the requested overnight nontraining work, actual blockers and formal feature counts readable and distinct. -->

**현재 정식 완료는 메인 0/9, 서브 0/30입니다.** 첫 목표는 직진 차로 유지(RTE-01)입니다. 모델 가중치·기능 head를 늘리지 않고, 기존 모델이 사용하는 입력과 실행 경로부터 개선합니다. [원래 9개 영역·30개 기능·합격 기준](../../../portable-e2e-feature-roadmap.md)은 유지합니다.

사용자 요청에 따라 오늘 **오전 9시 KST까지의 비학습 작업을 마친 뒤**, 추가 진행 요청으로 아래의 영상·정답 연결부 구현을 이어갔습니다. 다른 테스트의 CARLA에는 연결하지 않았으며, 새 학습·GPU 작업·실차 구동·자동 모델 승격은 하지 않았습니다. [10월 7일까지의 완료 결과](../2026-10-07/README.md)는 그대로 보존합니다.

## 현재 결론부터 보기

<!-- HH_261008 - Report the ended FIRST16 serial host-value and actual ACK verification ahead of historical cache A/B. -->

**후속 입력 형식·수신 확인 연결을 실제 검증까지 완료했습니다.** 원본 crop과 표지 보존 방식 각각 첫16배치에서 생성·모든 실제 값 검사·두 기록 저장·ACK가 모두16회 성공했고 실패/미실행은0입니다. 후보 복구2회·전체75.26초·관측 최대약130MiB·종료seal41개 일치입니다. 별도 코드의 지정 파일57개 저장 검산도3.11초에 완료했습니다.

[이번 결과·입력 연결·완료와 미구현·다음 순서](nontraining_batch_capture/03_serial_training_input/README.md)를 먼저 보세요. **실제 학습기 연결과 새 학습은 아직 없습니다.** Torch 수치 등가성·전체20,000배치 처리량도 미검증입니다. 로컬CPU·학습/GPU/PRO6000/CARLA/ROS0회이며 정식 승인0/9·0/30과 주행SAF-01 실패는 그대로입니다. 다음은 Torch/trainer 경계 확인과 동일 구조 crop A/B 학습 계약이며 기존 frozen trainer와 실패 근거는 유지합니다. 아래는 이전 종료 시점 기록입니다.

<!-- HH_261008 - Report the ended actual rescue/cache experiment before the historical byte-connection checkpoint. -->

**13:09 KST: 실제 표지판 정답 복구와 제한 캐시 읽기 비교를 완료했습니다.** 같은 첫 16배치에서 캐시 없음/사용 두 설정 모두 16개 통과·실패/미실행 0·RGB/정답 전부 일치이며 실제 복구는 같은 2회입니다. 설정별 시간 **237.06→34.28초**, 반복 압축 해제량 약 **86.1% 감소**, 전체 실행 274.11초·관측 최대 약 60MiB입니다. 실제 입력·잘림·복구 PNG를 생성·열람했고, 별도 코드로 지정 파일 68개·전체 기록의 저장 검산도 3.09초에 마쳤습니다.

[새 결과와 PNG·개선 수치·한계·다음 입력 연결](nontraining_batch_capture/02_actual_rescue_and_reader_ab/README.md)를 먼저 확인하세요. **입력 처리 개선이지 새 학습·인식/주행 성능 합격은 아닙니다.** 한 번의 순차 A/B로 전체 학습 속도를 보장하지 않으며 나머지 19,984배치의 RGB 검증은 범위 밖입니다. 모두 로컬 CPU이고 PRO6000·GPU·CARLA·ROS·학습은 사용하지 않았습니다. 첫 주행 목표 RTE-01·SAF-01 지연 미달·공식 승인 0/9·0/30은 유지합니다. 다음은 기존 frozen trainer를 보존한 새 입력 어댑터 연결과 비학습 검증입니다. 아래는 이전 완료 시점 기록입니다.

<!-- HH_261008 - Make the completed limited real-data integration current while preserving the earlier nine-o-clock checkpoint as history. -->

**12:23 KST: 실제 RGB·정답 배치 연결부 구현과 검증을 완료했습니다.** reader·crop worker·실제 바이트 journal writer를 구현하고, 실제 8배치 모두 원래 RGB·정답 해시를 재현했습니다. 실행 74.84초·실패/미실행 0·관측 최대 메모리 약 55MB이며, 독립 저장 검산은 지정 파일 50개·원래 20,000행에서 선택한 8행·새 8행의 64개 원본 해시 witness를 확인했습니다. 나머지 19,992배치의 RGB 검증은 범위 밖입니다.

**이번 64회 노출의 복구 조건 발생은 0회였습니다.** 배치 연결이 실제로 동작한 것은 확인했지만 실제 RGB 복구 효과·모델 성능 향상으로 세지 않습니다. 최종 합성 검사 185개와 별도 스칼라 검산 검사 20개는 통과했고 최초 18개 실패도 보존했습니다. 센서 수집 어댑터는 모의 정상·응답 중단·정리 실패 등을 검사했으며 실제 촬영·커널 소유권·파일 저장 인증은 아직 없습니다.

[이번 구현·실제 결과·분명한 한계·다음 순서](nontraining_batch_capture/README.md)를 먼저 보세요. **이번 작업은 로컬 CPU이며 새 학습·NN·CARLA·GPU·PRO6000 접속은 0회**입니다. 정식 승인 0/9·0/30과 주행의 SAF-01 지연 실패는 유지합니다. 다음은 실제 복구 배치 검증과 제한 메모리 loader 처리량 비교이며, trainer 연결·원격 학습은 미실행입니다.

## 오전 9시 마감 시점의 기록

<!-- HH_261008 - Replace superseded present-tense claims with ended measurements and explicitly prepared nontraining follow-ups. -->

**09:00 KST 마감: 이번에는 새 학습을 시작하지 않았습니다.** 실제 실행·독립 저장 검산과 문서 정리를 완료했습니다. 다음 RGB·정답 배치 worker는 설계 단계이며 구현 완료로 세지 않습니다. 현재 중요한 결론은 다음 세 가지입니다.

1. **주행 지연은 아직 기준 미달입니다.** CNN 컴파일 비교도 종료·독립 검산 완료했지만, 같은12쌍의 전체 ACK 중앙값104.03→104.31ms여서 채택하지 않습니다. 직접918개 원시 출력은 동일했습니다. 계산 방식 변경이 새로운 주행 능력 학습은 아닙니다.
2. **지도 학습 입력에서 실제 개선을 확인했습니다.** 같은20,000배치·1,209개 정답 검산 후 새 crop 정책이 완전 잘림3,626→1,821회를 줄였습니다. 독립 전체 저장·집계 검산도 마쳤습니다. 표지판 픽셀 총량 증가는1.10%로 작고, 실제 모델 인식·학습 개선은 아직 미검증입니다. [그룹별 결과와 향후 동일 구조 A/B 학습 설계](mapping_review/crop_preservation/README.md)를 따릅니다. 새 journal의59개 스칼라 검사도 완료했지만 실제RGB/trainer 연결은 아직 없습니다. 기존350장 평가의23/27·4개 실패·후보 미채택은 유지합니다.
3. **차량 중심 전체 기록 영상과 상태 명령을 완료했습니다.** [947프레임GIF·고정PNG3장·전체1,836행과 설명](recorded_visuals/README.md)을 확인하세요. 독립 저장 검산과 ROOT고정PNG3장 열람을 마쳤고, 과거 실패와ROS889개 미실행도 보존했습니다. [상태 확인 방법](workspace_status/README.md)은 고정된 종료 자료를 읽습니다. 실제새주행·RViz화면·모델 성능 합격은 아닙니다.

## 오전 9시까지 완료한 주행·화면 작업

**CNN 실행 비교 V21d는 지연 실패로 종료했고 독립 검산도 완료했습니다.** 사전 검사76개 후 실제105.41초 실행했습니다. 직접918개는 완료했고 ROS27개 관측·891개 미실행입니다. 비교 가능한 궤적12쌍의100ms 통과는0쌍이며, 마지막 Path 진입103.58ms·전체 ACK116.88ms를 보존했습니다. 로컬 CPU 작업이며 학습·GPU·CARLA·차량 구동은 아닙니다. [읽기 쉬운 측정 결과](driving_ros/README.md), [새 실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v21d/result.json), [원문 계획](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v21d/ROOT_plan.json)을 따릅니다.

차량 중심 영상 V9는218개 사전 검사와 별도5장 실검사 뒤 실제38.39초에 전체947프레임을 완료했습니다. GIF합11,336,666바이트·PNG3장·1,836행CSV이며 용량 기준은 그대로입니다. 앞선V6의509프레임 용량 실패도 보존했습니다. PNG에는 차량몸체 대신 실제rear-axle 기준점을 중심으로 원래 경로·실제 과거 흔적·저장된 모델 선택을 표시합니다. 최신 상태 읽기 도구는103개 사전 검사·실제 명령·독립 지정 메타데이터 검산을 완료했습니다.

[현재 모델이 실제로 학습한 범위와 다음 학습 순서](model_scope/README.md)도 추가했습니다. 최신 주행 학습은54개 상태를 갱신·47개 동결했으며, 정지 의도는 미학습이고 런타임은 명시된 GO입니다. 모델에 head가 존재하는 것과 기능이 되는 것을 구분합니다.

## 정리·재현 검사에서 남은 것

<!-- HH_261008 - Preserve the failed historical link and distinguish actual usable disk from raw free space without unauthorized cleanup. -->

08:54:43 KST [자원·경로 확인 기록](../../../../storage/experiments/diagnostics/2026-10-08/resource_and_navigation_readonly_v1.json)에서 쓰기 가능한 공간은 약3.28GB였습니다. 원시 빈 공간 약37.58GB와 다르며, CPU 기준은 충족하지만 GPU 시험의5.97GB 기준은 미달입니다. 디스크 예약 설정을 바꾸거나 시험·모델·설치 환경을 삭제하지 않았습니다. 승인된 캐시 정리는 서버 개인 폴더 백업 검증이 전제인데, SSH가 연결되지 않아 삭제0개입니다.

레거시 별칭·workflow 바로가기는0개입니다. 이는 전체 Autoware 실행 검증이 모두 끝났다는 뜻은 아닙니다. [링크 검사 V5](../../../../storage/experiments/diagnostics/2026-10-08/navigation_links_readonly_v5.json)는681개 문서·7,930개 로컬 링크에서 레거시0·과거 고정 자료 누락1건을 확인해 종료값1을 보존했습니다. 누락은10월1일 `WEIGHT_PROVENANCE_CORRECTION.md`이며 원본을 추정해서 만들어 넣지 않았습니다. 검사 범위 밖 홈 경로18건은10개 실제 대상이며 [존재 확인](../../../../storage/experiments/diagnostics/2026-10-08/navigation_outside_home_destinations_v1.json)에서 모두 있었습니다. 파일 내용·재현 가능성까지 검증한 것은 아닙니다.

기록 결과는 `storage/experiments/`에 있고 Git에서 제외됩니다. 코드·문서 clone만으로 과거 데이터·가중치·결과가 생기지는 않습니다. 이번에는 commit·push·서버 전송을 하지 않았습니다. 기존 결과를 삭제하고 같은 출력 경로로 다시 실행하는 것도 금지하며, 다음 실제 실험은 새 revision·출력·권한·자산 해시와 자원 확인이 필요합니다.

<details>
<summary>오전 9시까지의 조건별 실제 실행·사전 검사·실패 내역</summary>

| 대상 | 현재 상태 | 목적과 남은 조건 |
| --- | --- | --- |
| 주행: ROS 기록 입력 비교 | V13/V14 모두 종료·지연 실패 | V14 직접918개 완료·ROS28개 관측·890개 미실행. 완전한 궤적13쌍 중 전체 ACK 적시0쌍 |
| 주행: 상세 로그 저장 | V13/V14/V15 실제 압축 기록·독립 저장 검산 완료 | 각8종의 기록을 원문 바이트까지 복원 검증. 보존 검증이며 지연 개선이나 10Hz 통과로 세지 않음 |
| 주행: 감사 해시 복사 제거 | V15 실제 실행 종료·지연 실패 | 직접918개 입력·원시5출력 전량 V14와 동일. 공통 완전 출력11개 감사 평균10.58→8.49ms, 전체 ACK108.59ms로 기준 미달. [읽기 쉬운 결과·실제 그래프](driving_ros/README.md) |
| 주행: 추론 전 중복 복사 | V16b 실제 실행 종료·지연 실패 | 모의 검사344개 통과 후 직접918개·ROS31개 관측,887개 미실행. 완전15쌍의 전체 ACK 적시0. [실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v16b/result.json) |
| 주행: CPU 실행 위치 | V17b 실제 실행 종료·지연 실패 | 자체 스레드 설정16개 checkpoint 통과. 직접918개 완료·ROS10개 관측·908개 미실행, 완전4쌍의 전체 ACK 적시0. [실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v17b/result.json) |
| 주행: 입력 준비 후 타이머 대기 | V18c 종료·독립 검산 완료, 지연 미달 | 직접918개 완료·ROS146개 관측·772개 미실행. 완전129쌍의 ACK 적시0. 같은 입력의 공통 완전15쌍은 ACK 중앙값109.91→106.20ms. [전체 실제 검산](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v18c_independent_postcheck_v1.json) |
| 주행: 같은 시작 파일 반복 읽기 | V19c 종료·독립 검산 완료, 미채택 | 직접918개·ROS29개 관측,889개 미실행. 완전14쌍의 ACK100ms 통과0. 공통14쌍의 중앙값105.67→108.76ms로 퇴행. [전량 검산](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v19c_independent_postcheck_v1.json) |
| 주행: CNN 메모리 배치 | V20 종료·독립 검산 완료, 미채택 | 직접918개 완료·ROS43개 관측·875개 미실행, 비교 가능한 완전27쌍의 적시ACK0. 마지막 실패 입력도 두 출력은 실제 도착했으나 완료100.01ms로 늦음. [전량 검산](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_v20_independent_postcheck_v1.json) |
| 주행: 원래 CNN의 TorchScript 실행 | V21d 종료·독립 검산 완료, 미채택 | 직접918개 동일·ROS27개 관측·891미실행. 공통12쌍 ACK 중앙값104.03→104.31ms, 적시0 |
| 화면: 차량 중심 기록 영상 | V9 전체 렌더·독립 검산·고정PNG3장 열람 완료 | GIF918+29프레임·3PNG·전체1,836행. 원래ROS889미실행·실패·과거 용량 실패 유지. RViz 캡처나 새 주행 아님 |
| 주행: 최신 지연 그림 | V4 생성·전량 검산 완료 | 실제V16b/V18c의 전체1,836행·실패·미실행을 보존. 각실행19항목 검증과 교차실행17입력 비교를 구분. [실제 그림](driving_ros/README.md)·[전량 검산](../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_comparison_v4_independent_postcheck_v1.json) |
| 주행: 전체 유한값 검사 비용 | 실제 합성 비교816회 완료·미채택 | 768회 A/B·48회 overwrite 결과 동일, 실패0·미실행0. 뚜렷한 속도 이득이 없어 모델에 추가하지 않음. [실제 결과와 범위](../../../../storage/experiments/diagnostics/2026-10-08/cpu_finite_scratch_v1_ROOT_scope_v1.json) |
| 지도: 실제 학습 정답 crop | V2 실제 재생·독립 검산 완료 | 20,000개 배치 전량 일치. 표지판 양성 정답 노출의4.69%가 crop으로 완전히 잘림. [상세 결과](mapping_review/README.md) |
| 지도: 완전 잘림 보존 정책 | 실제20,000배치·독립 전체 저장 검산 완료 | 잘림3,626→1,821회. 학습 입력 품질 개선이며 새RGB배치·학습·모델 개선 미검증 |
| 지도: 새 journal 계약 |59개 스칼라 검사 완료, 실제writer/trainer 미연결 | 제안/선택box·새RGB/GT해시 구분. 합성20,000행 전량·마지막실패 보존 검사이며 실제 픽셀 인증 아님 |
| 지도: 센서별 촬영 헤더 | V2 helper·모의 검사53개 완료, 실제 collector 연결 미구현 | 다음 새 촬영부터 각 센서의 frame·시각·위치·원시 바이트 출처 보존. 실제 촬영0이며 옛 촬영에 추정값을 넣지 않음 |
| 지도: 새 촬영 소유권 | V2 helper 모의 검사 완료, 실제 연결 미구현 | 최초 RPC 소켓과 다른 소켓을 같은 프로세스가 열어도 거절. 제공된 메타데이터 검사이며 실제 커널·collector 인증은 별도 |
| 지도: 지원 분류 표시 | 새V2 helper·모의 검사93개 완료 | 여섯 정적 분류·other=미확인 보존. 원본88개 검사 유지, 번호 범위가 출처 인증은 아닌 점을 수정. 실제 지도·인식 개선 아님 |
| 지도: 같은 정답 영역과 픽셀의 누락·회복 | V3·V4 실제 완료·독립 저장 검산 완료 | 각350장·세 군1,050건 전량 완료. 후보의 누락 증가를 확인했으며 모델 개선 아님. [분석 결과](mapping_review/README.md) |
| 상태 표시 | V4 새 스냅샷V2 실제 명령 검증 완료 | 08:44 고정 목록·08:50:31 실행 관측. 지정28개 메타데이터·4개 종료 작업 연결. 자동 서버 전체 조사나 기능 승인 아님 |
| PRO6000 | 접속 불가 | 08:31–08:32 KST 재확인도 `No route to host`. 원격 GPU·디스크 상태 미확인, 새 학습 없음 |

</details>

## 9개 영역 중 지금 어느 부분인가요?

<!-- HH_261008 - Keep formal approvals separate from partial implementation, offline checks and the additional mapping model. -->

| 메인 영역 | 세부 수 / 정식 승인 | 현재 위치 |
| --- | ---: | --- |
| 센서·상태 이해 | 4 / 0 | 기록된 영상·상태·TF 연결 검사. 실제 rig 검증은 별도 |
| 경로·차로 주행 | 4 / 0 | 저속 직진·좌우회전 근거는 있으나 독립 경로·반복까지 합격한 기능 없음. 첫 목표RTE-01 |
| 속도·승차감 | 3 / 0 | 자체 모델의 30km/h 기능 승인 전. 곡률 감속·승차감도 별도 검증 필요 |
| 교통규칙·교차로 | 4 / 0 | 교사 행동·지도 정적 분류와 모델의 신호 준수 성공을 구분 |
| 선행차·ACC | 3 / 0 | 선행차 인식·간격 유지·cut-in의 독립 폐루프 합격 근거 없음 |
| 장애물·충돌 위험 | 4 / 0 | 위험 head나 점수 실험만으로 회피·독립 AEB를 승인하지 않음 |
| 차선변경·합류 | 3 / 0 | 좌우회전과 다른 기능. 합법 경계·후측방 간격·복귀 검증 필요 |
| 저속 정차·주차 | 2 / 0 | 갓길정차·주차/후진 검증 전. 경로 종점 정지는 경로 영역의RTE-04 |
| 런타임 안전·실차 전환 | 3 / 0 | **현재 집중: SAF-01 처리 지연.** 100ms 기준 미달이며 실차 승인 없음 |
| 합계 | 30 / 0 | 메인도0/9. 검사 개수나 학습 횟수를 완료 기능 수로 세지 않음 |

자동 지도는 이9개와 병렬인 별도 모델입니다. 현재7종 최종 분류와 누적 코드가 있지만 횡단보도·정지선·동적 객체·신호 상태와 정밀지도 연결 관계가 모두 구현·검증된 것은 아닙니다. 주행 모델이 이 지도를 필수로 요구하도록 바꾸지 않습니다.

<details>
<summary>앞선 시점의 주행 문제와 수정 과정 전체 기록</summary>

아래의 “준비·실행 중·다음”은 각 기록 당시의 상태입니다. 지금의 상태는 위쪽 최신 결론을 따릅니다.

## 앞선 주행 문제와 계측 과정

직전 V11은 직접 입력 918개를 모두 처리했지만, ROS 연결에서는 43개만 관측하고 종료했습니다. 완전한 출력 26쌍 중 전체 ACK 100ms 충족은 6쌍입니다. 마지막 출력 진입이 100.65ms를 넘어 거절됐고, 나머지 875개는 미실행입니다. [실제 결과·실패·계측 그림](../2026-10-07/driving_ros/README.md)을 유지하며 개선됐다고 말하지 않습니다.

새로 확인한 것은 **상세 기록의 중복 저장 문제**입니다. V11 실제 43행을 기준으로 918행의 평균 크기를 계산하면 수신 기록 약50.91MB, 송신 기록 약37.23MB입니다. 각각 기존 파일 한도32MiB를 넘습니다. 송신 콘솔에도 전체 ACK가 반복되므로 전체128MiB 한도와 마감용16MiB 여유까지 지키기 어렵습니다. 이것은 실제 전체 실행 결과가 아니라 관측된 행 크기의 사전 추정입니다.

먼저 V12 실행기 변경을 순수 검사하고, **정상적인 원래 기록을 무손실 보존하는 별도 저장 revision**을 준비한 뒤 실행합니다. 원래 918개 분모·실패 꼬리·모든 실제 계측 단계·100ms 관문은 바꾸지 않습니다. 제한보다 큰 임의 출력은 전체 바이트를 저장할 수 없으며, 그 경우 기존 접두 바이트·정확한 크기·SHA·미보존 사실을 남기고 실패로 처리합니다. 원래 호출 위치의 기록 비용을 다른 곳으로 옮기지 않습니다.

**ACK 시각의 범위:** 원래 ACK 완료는 실제 메시지 publish 직후이고, 그 뒤의 파일 기록·콘솔 출력까지 포함하지 않습니다. 숫자 감사와 ACK 인코딩은 그 시각 전에 있지만, 전체 기록 비용까지 포함됐다고 주장하지 않습니다. 새 비교에서는 원래 시각을 바꾸지 않고 로그까지 끝난 시각을 별도 항목으로 구분합니다.

01:49 KST까지 [소스·모의 검사 기록](../../../../storage/experiments/diagnostics/2026-10-08/nontraining_source_checks_v1.json)을 보존했습니다. 실행기276개·저장 모듈59개·지도 검산102개는 각각 다른 파일의 검사이며 기존 검사를 포함하므로 새 기능 수로 합치지 않습니다. 교차 검토에서 확인한 실패 경계는 원래 소스·검사를 남긴 채 새 revision으로 수정합니다. 실제 ROS 비교는 저장 방식만 바꾼 V13→같은 저장 방식에 노드 등록 유지를 추가한 V14 순서로 준비합니다. 두 조건의 같은 가중치·입력·정리·100ms를 유지해 효과를 구분합니다.

**02:18 KST:** [V13 통합 모의 검사295개](../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v13_source_test_v1.json)와 독립 소스 검토 후 실제 실행을 시작했습니다. [원문 계획](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_plan.json)·[별도 실행 권한](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_execution_authority.json)·[실행 전 검토](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v13/ROOT_execution_review_v1.json)에서 전체106개 소스·5,900개 입력·42개 생성 메시지 경로를 고정했습니다. 현재는 기록 입력의 직접 CPU 추론 단계이며 차량 구동·CARLA·GPU·학습은 아닙니다. 처음부터 끝까지 실행됐다는 결론이나 10Hz 개선 판정은 아직 없습니다.

**02:20 KST 이후 종료 확인:** [V13 실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13/result.json)는103.85초 실행 뒤 지연 관문에서 실패했습니다. 직접918개는 완료했지만 ROS는26개만 관측하고 나머지892개는 미실행입니다. 관측26개의 입력 텐서·비교 가능한 원시 출력은 같았고, 완전한 궤적11쌍 중 전체 ACK100ms 충족은0쌍입니다. 마지막의 일부 Trajectory는 실제 송신·수신됐지만 뒤의 Path 진입이100.67ms라 거절됐으며 전체 ACK도120.20ms입니다. 부분 출력을 완전한 성공으로 세지 않습니다. 소스·입력 종료 해시는 동일하고 정리 오류는 없습니다. 압축 기록의 종료 검사는 모두 손상 없음으로 반환했지만, 별도 독립 검산 전 전량 검증 완료로 표현하지 않습니다. 같은 압축·계측·정리 조건의 V14 실행기 변경을 별도로 비교할 예정입니다. 과거 V11보다 주행이 개선됐다는 결론은 없습니다.

**02:33 KST:** [V14 모의 검사298개·동일 조건 확인](../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v14_source_test_v1.json)을 완료하고 실제 실행을 시작했습니다. [V14 계획](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_plan.json)·[실행 전 검토](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v14/ROOT_execution_review_v1.json)에서 같은 가중치·입력·저장 방식·계측·정리를 유지하고 수신 노드의 실행기 등록 유지 방식만 바꿨습니다. 현재는 기록 입력의 직접 CPU 추론이며, 완료·주행 개선·10Hz 통과로 표현하지 않습니다.

**02:35 KST 이후 종료 확인:** [V14 실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14/result.json)는109.99초 실행 후 실패했습니다. 직접918개 완료, ROS28개 관측·890개 미실행입니다. 완전한13쌍의 궤적·Path는 직접 결과와 같았지만 전체 ACK100ms 충족은0쌍입니다. 마지막 일부 Trajectory는 실제 전달됐고, 뒤의 Path 진입100.51ms가 거절됐으며 전체 ACK115.62ms입니다. 종료 소스·입력 해시 일치와 정리 오류0은 유지합니다. V13보다2개 더 관측했다는 이유로 지연 개선을 입증하거나 기능을 승인하지 않습니다.

다음 후보는 **감사 해시의 바이트 복사 한 가지**입니다. 현재 `array.tobytes()`가 입력 이미지 텐서20,736,000바이트를 새로 복사합니다. 동일한 바이트를 `memoryview`로 읽어 SHA를 계산하되 dtype·shape·원본 입력·모든 출력·변경 검사·100ms는 그대로 유지하는 새 V15 소스를 준비합니다. 실제 절감 시간은 미측정이며, 이 변경을 모델 학습이나 인식 능력 향상으로 표현하지 않습니다.

**03:01 KST:** [V15 소스·모의 검사309개·실제 합성 NumPy 검사](../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_v15_source_and_numpy_test_v1.json)와 독립 소스 검토 후 실제 실행을 시작했습니다. [원문 계획](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_plan.json)·[실행 전 검토](../../../../storage/experiments/plans/2026-10-08/temporal_recorded_ros_shadow_v15/ROOT_execution_review_v1.json)를 보존했습니다. 합성 검사14개는 현재9입력·5출력의 양수 크기 형식에서 dtype·shape·SHA를 확인한 것이며, 실제 영상이나 NN 실행이 아니었습니다. 빈2차원 배열 `(0,3)`의 호환성 차이는 범위 밖 오류로 남겼고 모든 텐서 형식의 동등성·동시 버퍼 변경에 대한 snapshot 동등성을 주장하지 않습니다. 지금 실제 기록 입력에서 직접 추론→ROS를 측정 중이며 최종 결과는 아직 없습니다.

<!-- HH_261008 - Preserve the completed adverse V15 interval, complete direct parity and the limited paired timing observation. -->

**03:03 KST 종료 확인:** [V15 실제 결과](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v15/result.json)는103.26초 후 실패로 종료했습니다. 직접918개는 완료했고, V14와 입력9종·원시5출력·상태가 전량 동일했습니다. ROS는26개 관측·892개 미실행입니다. 완전한11쌍 중 전체 ACK100ms 충족은0쌍이며 마지막 부분 Trajectory는 실제 전달됐지만 Path 진입100.59ms가 거절됐습니다. 마지막 전체 ACK116.15ms·부분 출력·실패 꼬리를 모두 보존합니다. 종료 소스·입력 해시 일치, 정리 오류0입니다.

워밍업·거절을 포함한 순번14–24의11개 집계와 **실제 완전 출력의 공통11개(순번5–9·19–24)**를 구분했습니다. 후자의 감사 평균은 V14의10.58ms→V15의8.49ms, 전체 ACK는110.57ms→108.59ms입니다. 한 번씩 측정한 사후 비교라 지속적·인과적 향상을 입증하지 않으며 기준 미달로 채택하지 않습니다. [원래 표기 오류·두 집계·실패 보존](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14_v15_clock_label_correction_v2.json)을 따릅니다. 이 감사는 Path 출력 **뒤**, ACK **앞**에 있어 같은 입력의 Path 진입 지연을 직접 줄이지 못합니다. 다음 V16b는 추론 **전**의 과거 입력 중복 복사 한 가지이며, 모델·입력 값·검사·100ms는 유지합니다.

독립 저장 검산은 [V13](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v13_independent_postcheck_v1.json)·[V14](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v14_independent_postcheck_v1.json)·[V15](../../../../storage/experiments/diagnostics/2026-10-08/temporal_recorded_ros_shadow_v15_independent_postcheck_v1.json) 모두 완료했습니다. 원본·보존 소스·입력·생성 메시지·기록의 세대를 두 번 확인했고 각8종 압축 기록의 전체 CRC·원문 SHA·색인을 검증했습니다. 이것은 다른 실행자가 같은 주행 시간 실험을 다시 했다는 뜻이 아닙니다. V14/V15 검산의 공통 구간 통계에 이전 버전명 `V13`이 남은 표기 오류는 원본을 보존하고 별도 정정 중이며, 그 표기를 V13 측정값으로 사용하지 않습니다.

[그래프 V1 준비 실패](../../../../storage/experiments/diagnostics/2026-10-08/recorded_ros_clock_v1_prepare_failure_v1.json)는 실행 전 소스 보존량3,511,520바이트가2MiB 한도를 넘어 파일 쓰기 전에 중단된 경우입니다. 전체16MiB 한도와 마감용2MiB는 그대로 두고, 소스4MiB·PNG당2MiB로 내부 배분만 바꾸는 새 V2를 준비합니다. 실제 추론·ROS 실패 분모와 섞지 않습니다.

</details>

## 지도 쪽: 왜 정답을 다시 확인하나요?

직전 검사에서는 원본 영상·semantic과 저장 package의 **350장 픽셀이 전량 일치**했습니다. 저장 오류가 없다는 뜻이지 작은 표지가 잘 인식된다는 뜻은 아닙니다. 후보는 오탐을 줄였지만 누락을 늘려 미채택입니다.

**실제 학습에서 표지판 정답이 crop·확대 후 얼마나 남았는지 검산했습니다.** 정답 재생 V2는 군당20,000개 배치·160,000회 노출이며 두 군 합계320,000회입니다. 두 군의 실제 기록은 바이트까지 같습니다. 한 번 재생한 모든20,000개 증강 후 정답 해시가 과거 기록과 일치했고, 원본1,209개 정답의 해시·클래스별 픽셀 수도 일치했습니다. 계산160,000회와 두 군의 원래 분모320,000회를 구분합니다.

승인된 TRAIN의 정답 두 개만 배열로 해독했습니다. RGB·TRUE view·모델 가중치·DEV/HOLDOUT·학습 금지 CTrack의 배열은 해독하지 않았습니다. package 파일의 전량 해시는 검증하며, 그 안의 압축된 바이트를 읽는 것은 영상 배열 해독과 구분합니다. 로컬 환경은 과거 학습 환경과 달라, 환경 동등성이 아니라 **실제 정답 바이트의 전량 일치**만 확인합니다. 실제 재생은145.97초, 관측 최대 VmHWM은약310.52MiB였으며 양 군 각각 실패0·미실행0입니다. 최대 메모리·시간은 관측치이고 OS 강제 차단 보장은 아닙니다.

표지판 정답이 있던77,291회 중3,626회는 crop으로 모두 잘렸습니다. crop 후에도 정답이 있었는데 resize 때문에 사라진 경우는0입니다. 전체 resize 정답에서 표지판 픽셀은약0.034%로 매우 드뭅니다. **두 비교 모델이 같은 정답을 사용했으므로 crop만으로 후보 모델의 퇴보 원인이 입증되지는 않습니다.** 이번 결과는 다음 학습 설계를 위한 근거이며 가중치 변경·모델 개선·기능 승인으로 세지 않습니다. [실제 결과와 독립 사후 검산](mapping_review/README.md)을 함께 보존했습니다.

센서 헤더는 [새 V2 helper](../../../../learning/portable_e2e/mapping_sensor_provenance_20261008_v2.py)와 [모의 검사53개](../../../../storage/experiments/diagnostics/2026-10-08/mapping_sensor_provenance_source_test_v2.json)를 완료했습니다. V1의52개 검사는 보존하며, 후속 검토에서 찾은 저장 경로 `.` 허용 오류는 새 V2에서 거절합니다. NaN/Inf·센서 frame 건너뜀·시각 차이·ID 중복·잘못된 위치 행렬도 검사합니다. helper는 이미 받은 객체만 확인하고 CARLA에 연결하지 않습니다. 객체 출처 인증과 실제 파일 해시 검증은 향후 별도 owner·collector가 해야 합니다. **실제 새 촬영·옛 헤더 복원은0**입니다.

## 오전 9시에 정리한 다음 순서

<!-- HH_261008 - Preserve the old handoff sequence as history and route current work to the completed implementation report. -->

아래는 오전 9시 시점의 순서입니다. 이후 실제 RGB·정답 연결부를 구현했으므로 [현재의 다음 순서](nontraining_batch_capture/README.md#다음-순서)를 따릅니다. 기존 결과와 실패 기록은 유지합니다.

1. 로컬 완료: 전체947프레임·3PNG·1,836행을 생성·검산했습니다. 원래 실패 구간·용량 기준을 유지했습니다.
2. 로컬 완료: TRAIN 정답만 사용한 보존 정책의 실제20,000배치 재생·독립 검산과 새journal59개 검사를 마쳤습니다. RGB·모델·학습·실제writer/trainer 연결은 실행하지 않았습니다.
3. 로컬 마무리: 최신 종료 결과의 상태 목록·재현 명령·링크·구조·공간을 갱신하고 점검합니다. 종료된 학습을 실행 중이라고 표시하지 않습니다.
4. 서버: 접속만 정해진 간격으로 재확인합니다. 접속이 되더라도 이번 비학습 요청에서 학습을 시작하지 않습니다. 백업·검증 전 승인 캐시도 삭제하지 않습니다.
5. 후속 학습과 실제 폐루프: 별도 허용 TRAIN·독립 평가·GPU0 가용성·소유 시뮬레이터·기존 공간 기준이 충족된 뒤 진행합니다. [모델 구조와 구체적인 학습 순서](model_scope/README.md)를 따릅니다.

[08:31–08:32 실제 SSH 재확인 기록](../../../../storage/experiments/diagnostics/2026-10-08/ssh_readonly_recheck_v8.json). 서버 백업 없이 승인된 캐시를 삭제하지 않고, 추가 로그 삭제도 새 권한 없이 실행하지 않습니다.

## 파일 보관과 재현의 주의사항

<!-- HH_261008 - Disclose ignored local experiment assets and frozen-path rerun limitations instead of promising evidence availability after clone. -->

원본 그림·GIF·JSON은 현재PC의 `storage/experiments/diagnostics/2026-10-08`에 있고, 이 위치는Git에서 제외돼 있습니다. 안내문과 코드만 clone하면 이 과거 실행 자료까지 자동으로 생기지 않습니다. 이번 작업에서 commit·push·서버 전송은 하지 않았습니다. 공유할 때는 별도 검증된 증거 패키지와 필요한 모델/데이터가 필요합니다.

기존 실험의 `execute` 명령은 이미 존재하는출력을 덮어쓰지 않도록 거절합니다. 같은 폴더를 지워 재실행하지 마세요. 읽기 전용 상태 명령은 저장 자료를 확인할 수 있지만, 새 실험은 새revision·고정입력·권한·자원검사·출력 경로가 필요합니다. 현재 재현 설명은이PC의 실제 기록이며 모든환경의 즉시 실행을 보장하지 않습니다.
