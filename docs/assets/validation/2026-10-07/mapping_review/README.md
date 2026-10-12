# 지도 7분류: 완료한 검산과 아직 남은 실패

<!-- HH_261007 - Report the actual whole stored-mask diagnostic and independent seals without implying new inference, training or model approval. -->

**후보는 표지 오탐을 줄였지만 누락이 늘어, 여전히 미채택이다.** 10월 7일에는 저장된 350장의 정답·예측 마스크를 부모·대조군·후보 세 방식으로 전량 분석했다. **1,050/1,050 처리 완료**는 진단을 끝냈다는 뜻이지, 인식 성능이나 기능이 합격했다는 뜻이 아니다. native-fusion v3의 기존 판정은 **23/27 통과·4개 실패**, 정식 완료는 계속 **메인 0/9·하위 0/30**이다.

이번 지도 검산에서는 새 추론·학습·GPU·ROS·CARLA 실행이 없었다. 이번 표지 분석과 아래의 기존 좌표 검산은 서로 다른 검사이며, 어느 쪽도 모델 가중치를 개선하거나 자동 교체하지 않았다.

## 무엇이 전량 완료됐고, 무엇이 실패했나

| 범위 | 전량 측정 | 기존 판정 |
| --- | --- | --- |
| Town01 `dev_town01` | 118장, 7종 모두 정답 지지 픽셀과 지표 존재 | 관련 19개 기준 중 15개 통과·4개 실패 |
| Town02 `holdout_town02_p02` | 232장, 7종 모두 지표 존재 | 관련 8개 기준 통과·0개 실패 |
| 전체 | 고정 350장, 후보의 분류×집합 14칸; 부모·대조군 포함 42칸 | 기존 20개는 18/20, 추가 7개는 5/7, 합계 23/27 |

두 집합은 이미 반복 사용한 진단 자료이며 **새 독립 장소는 0개**다. Town02의 기존 상대 보존 기준 통과는 도메인 전체 승인이나 7개 클래스의 절대 정확도 합격을 뜻하지 않는다. [원래 전체 27개 판정·고정 화면](../../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/README.md), [실제 수치](../../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/results.json)를 기준으로 했다.

## 표지 전량 분석에서 확인된 것

<!-- HH_261007 - Preserve all six cells, size/border partitions and the failed V1 denominator; correlations are not causal proof. -->

[실제 V2 결과](../../../../../storage/experiments/diagnostics/2026-10-07/map_sign_components_v2/result.json)는 **3.56575초**, 현재 프로세스의 최대 RSS **325,582,848바이트(약 310.5MiB)**로 1,050개를 모두 처리했다. 실패·미실행은 0개다. RGB를 디코딩하거나 모델을 다시 실행하지 않고, 저장된 정답·예측 라벨만 분석했다.

[독립 후검산](../../../../../storage/experiments/diagnostics/2026-10-07/map_sign_components_v2_independent_postcheck_v1.json)은 현재 소스 11개·입력 16개·소스 사본 11개, 전체 처리 순서와 기록된 혼동표·집계를 확인했다. 원본 마스크에서 연결영역을 다시 계산한 두 번째 실행은 아니다. [V1의 실행 전 RSS 검사 실패](../../../../../storage/experiments/diagnostics/2026-10-07/map_sign_components_v1/result.json)와 **1,050개 미실행 분모**도 보존했다.

오탐(FP)은 표지가 아닌 픽셀을 표지라고 판정한 수, 누락(FN)은 실제 표지 픽셀을 놓친 수다. 아래 숫자는 표지판 개수가 아니다.

| 데이터 | 방식 | 오탐 픽셀 FP | 누락 픽셀 FN | precision | recall |
| --- | --- | ---: | ---: | ---: | ---: |
| Town01 118장 | 부모 | 25,976 | 1,055 | 6.68% | 63.81% |
| Town01 118장 | 대조군 | 20,884 | 1,379 | 6.85% | 52.69% |
| Town01 118장 | 후보 | 16,758 | 1,551 | 7.53% | 46.79% |
| Town02 232장 | 부모 | 15,022 | 3,017 | 78.56% | 94.80% |
| Town02 232장 | 대조군 | 9,637 | 5,057 | 84.62% | 91.29% |
| Town02 232장 | 후보 | 8,072 | 5,275 | 86.74% | 90.92% |

즉, 후보는 두 집합 모두에서 **덜 잘못 표시하지만 실제 표지도 더 많이 놓친다.** Town02의 기존 보존 기준 통과도 이 recall 저하를 없애지는 않는다. 오탐·precision 개선만으로 누락 손실을 상쇄했다고 판단하지 않는다.

크기 분석의 단위는 같은 표지 라벨이 8방향으로 붙은 **픽셀 연결영역**이다. 한 영역이 실제 표지판 하나인지는 확인하지 않았다.

| 관찰 | Town01 | Town02 |
| --- | --- | --- |
| 1–16픽셀 정답 영역의 완전 누락: 부모 → 대조군 → 후보 | 136/136 → 136/136 → 131/136 | 332/463 → 406/463 → 403/463 |
| 후보의 정답 경계 영역 누락 / 전체 누락 픽셀 | 19 / 1,551 | 26 / 5,275 |

작은 영역 누락은 세 방식 모두 심하다. 다만 후보의 손실이 작은 영역에만 있는 것은 아니다. 17–64픽셀 정답 영역의 TP도 Town01 **1,097 → 772 → 593**, Town02 **4,174 → 3,306 → 3,140**픽셀로 줄었다.

Town01의 **257픽셀 이상 예측 영역은 세 방식 모두 정답과 겹치는 TP가 0**이다. 이 큰 영역의 FP는 부모 16,893·대조군 15,540·후보 7,932픽셀이다. 후보에서 줄었지만 큰 오인식 영역이 여전히 남았다.

경계에 닿는 정답 영역의 누락은 전체 누락 중 일부뿐이다. 현재 결과로는 **대부분의 누락을 화면 경계나 crop 탓으로 단정할 수 없다.** 크기·경계와 오류의 관련성을 관찰했을 뿐, 데이터·crop·모델 구조 중 무엇이 원인인지는 입증하지 않았다.

<!-- HH_261007 - Describe exact sign/other confusion from whole archived ledgers, not a visual guess or a new model run. -->
추가 [오류 라벨 집계](../../../../../storage/experiments/diagnostics/2026-10-07/map_sign_error_transitions_v1.json)에서는 후보가 놓친 표지 픽셀 중 Town01 **1,481/1,551(95.49%)**, Town02 **4,987/5,275(94.54%)**를 `other`로 분류했다. Town01 표지 오탐도 **16,702/16,758(99.67%)**가 `other`에서 나왔다. 따라서 우선 확인할 것은 표지와 배경의 구분이며, 신호등 상태 head를 추가하면 이 누락이 해결된다는 근거는 아니다. 이 집계는 이미 저장된 전체 혼동표를 읽은 것이며 마스크·추론을 다시 실행하지 않았다.

기존 공개 고정 인덱스 그림 중 [Town01 0번](../../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/dev_town01_0000.png)과 [58번](../../2026-10-02/portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/dev_town01_0058.png)도 직접 확인했다. 원래 영상·정답·부모·대조군·후보를 나란히 보여주는 저장 예측 그림이며 새 RViz 화면이나 새 추론은 아니다. 두 그림 확인을 전체 350장 시각 검토나 원인 입증으로 세지 않는다.

## 최신 후보의 7종 실제 수치

단위 %, 각 칸은 **IoU / recall**이다. 영상 픽셀 지표이며 누적 지도 셀이나 실제 객체 개수의 지표가 아니다.

| 라벨 | Town01 | Town02 |
| --- | ---: | ---: |
| 기타 `other` | 99.03 / 99.50 | 96.18 / 97.53 |
| 도로 `road` | 99.87 / 99.88 | 99.43 / 99.69 |
| 보도 `sidewalk` | 98.76 / 99.49 | 95.41 / 99.09 |
| 차선 표시 `lane_marking` | 99.33 / 99.86 | 99.77 / 99.91 |
| 신호등 기구 `traffic_light_fixture` | 76.48 / 83.69 | 44.78 / 61.76 |
| 표지 `traffic_sign` | 6.93 / 46.79 | 79.82 / 90.92 |
| 기둥 `pole` | 68.22 / 78.91 | 52.10 / 67.72 |

## 실패 유형

<!-- HH_261007 - Keep every failed screen and distinguish observed errors from unproven causal explanations. -->

1. **Town01 표지 누락 증가:** recall은 부모 63.81% → 대조군 52.69% → 후보 46.79%로 떨어졌다. 아래 네 기준이 모두 실패했지만, 같은 recall을 두 기준·두 참조와 비교한 결과이지 서로 독립된 네 사고나 네 클래스 실패가 아니다.

   - `Town01_sign_recall_vs_parent`
   - `Town01_sign_recall_vs_control`
   - `ADDED_Town01_sign_recall_strict_gain_vs_parent`
   - `ADDED_Town01_sign_recall_strict_gain_vs_control`

2. **오탐 감소와 누락의 교환:** Town01 배경→표지 오탐은 25,548 → 16,702픽셀로 감소했지만 후보 표지 precision은 7.53%뿐이다. 표지 전체 FP 16,758·FN 1,551픽셀이 남으며, 실제 정답 표지 픽셀은 2,915개다. 오탐 감소만으로 채택하지 않는다. 이 숫자는 표지 객체 수가 아니다.
3. **장소·클래스별 편차:** Town02 표지 IoU 79.82%와 Town01 6.93%는 크게 다르며, Town02 신호등 기구 IoU 44.78%·기둥 52.10%도 충분한 지도 품질 승인 근거가 아니다. 작은 대상·crop·외관·표현 중 어느 원인이 지배적인지는 이 요약만으로 입증하지 못한다.
4. **인식 밖의 미구현·미검증:** 신호등 기구는 빨강/노랑/초록 상태가 아니고, 표지는 세부 종류·규칙 판독이 아니다. 현 7종에는 횡단보도·정지선의 전용 라벨, 차선 ID·연결 관계·주행 방향이 없다. `other`나 미관측 영역을 자유 공간으로 바꾸면 안 된다.

[라벨·입력 manifest](../../../../../storage/experiments/training/2026-10-02/map_native_fusion_pair_v3/inputs/manifest.json)는 RGB만 모델 입력으로 쓰며 depth·pose는 입력이 아님을 명시한다. TRAIN은 기존 729개와 추가 FALSE 뷰 480개의 1,209개 논리 슬롯이다. DEV/holdout을 fit에 넣지 않았고 CTrack은 계속 TRAIN 금지다. 반복 노출이나 같은 장소의 추가 장면을 독립 지역 증가로 세지 않는다. [고정 구조·자료 계약](../../../../../tools/e2e/map_native_fusion_contract_20261002.py)도 같은 경계를 유지한다.

## 좌표 검증 완료 ≠ 실제 AMG 완료

<!-- HH_261007 - Separate historical oracle-assisted fusion integrity from latest-model perception and deployable localization. -->

[10월 7일 V2 실제 결과](../../../../../storage/experiments/diagnostics/2026-10-07/mapping_no_training_v2/result.json)는 **기존 geometry/control 모델**의 저장 예측으로 480프레임·1,440개 누적 상태·최종 격자 6개를 대조했다. 최대 역투영 오차는 약 2.39×10⁻¹²pixel이며, 전치 대신 실제 저장 행렬을 풀어 역변환했다. [V1 실패](../../../../../storage/experiments/diagnostics/2026-10-07/mapping_no_training_v1/result.json)와 [별도 2,482개 해시·전량 기록 후검산](../../../../../storage/experiments/diagnostics/2026-10-07/mapping_no_training_v2_independent_postcheck_v1.json)은 그대로 보존했다.

이 검사는 **최신 native-fusion v3를 좌표 지도에 적용한 시험이 아니다.** CARLA 정답 depth·pose를 공유하며, oracle semantic은 기준에만 쓴다. 실제 센서의 깊이·측위 오차, 드리프트, lane topology, 실제 온라인 100ms 지연은 검증하지 않았다. 원래 LEFT owner FAIL도 유지한다.

[기존 CTrack 누적 지도](../../2026-10-01/map_geometry_full_projection/README.md)의 geometry 차선 셀 IoU는 LEFT 9.34%·STRAIGHT 21.88%이며, oracle 셀 밖 learned 셀 8,630·4,433개와 road FP 3,316·2,818개가 남아 있다. 이는 위의 최신 모델 영상 픽셀 IoU와 모델·분모가 다르다. 좌표 수치 일치나 셀 증가가 인식 개선·물리적 자유 공간·정밀지도 완성을 뜻하지 않는다.

## 다음 우선순위: 비학습 확인과 향후 학습을 구분

<!-- HH_261007 - Report completed source/metadata alignment review without claiming raw-pixel alignment or new inference. -->

[추가 소스·메타데이터 정렬 검토](../../../../../storage/experiments/diagnostics/2026-10-07/map_alignment_source_review_v1.json)는 **완료**했습니다. 원래 V3 소스 15개의 현재/보존 해시와 Town01 118장·Town02 232장 전체의 순서·native frame·시각·RGB/semantic 파일 해시 참조를 대조했습니다. 라벨 7종 순서나 이 메타데이터 연결에서 구체적인 불일치는 발견하지 못했습니다.

현재 평가는 **640×360 전체 영상**을 사용하며 RGB의 HWC→CHW 변환과 float32/255만 수행합니다. 평가에서 crop/resize를 하지 않으므로, 이번 350장의 표지 누락을 평가 crop 때문이라고 설명할 근거는 없습니다. 학습 crop은 RGB와 정답에 같은 영역을 적용하고 RGB에는 BILINEAR, 정답에는 NEAREST를 사용하지만 **최신 V3의 실제 crop 노출에서** 작은 표지 지지 픽셀의 손실량은 이번 검사에서 측정하지 않았습니다. 이전 다른 학습군의 [128,000개 노출 분석](../../2026-10-02/portable_e2e_learning_cycle_v1/13_map_effective_training_exposure/README.md)은 보존하되 V3의 새 측정으로 바꾸어 세지 않습니다.

이 검토는 **실제 원본 PNG/semantic 배열을 디코딩하지 않았습니다.** 과거 수집 코드는 같은 world frame·시각 차이 10⁻⁶ 이내·pose 일치를 확인하지만, 해당 옛 파일에는 센서별 frame/time 헤더가 따로 저장되지 않았습니다. 따라서 소스와 참조 일치만으로 영상·정답 픽셀 정렬이나 실제 객체 라벨의 정답성을 확인했다고 말하지 않습니다.

자료 설명에도 구분이 필요합니다. 입력 package의 과거 balanced FALSE/TRUE 설명과 달리 **실제 V3 양쪽 방식은 FALSE 입력만 사용**했습니다. 추가 TRAIN p02·p04는 Town01이며, 이름이 비슷한 HOLDOUT p02는 별도의 Town02 자료입니다. Town 이름이나 같은 장소에서 멀리 떨어진 포즈만으로 새로운 독립 지역이라고 세지 않습니다. 원래 권한·manifest·소스·TRAIN 금지 조건은 변경하지 않았습니다.

<!-- HH_261007 - Preserve a real pre-pixel quota denial and distinguish a new storage-only revision from actual consistency evidence. -->

원본-패키지 픽셀 검사 V1의 [순수 테스트 50개](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_consistency_source_test_v1.json)는 통과했지만, [실제 준비](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_consistency_prepare_attempt_v1.json)는 **픽셀을 열기 전에 저장 한도로 중단**됐습니다. 결과를 저장하지 못한 채 전량 실행하지 않습니다. [용량 원인 검토](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_storage_readonly_v1.json)에서 소스 경로 36개의 사본 572,499byte 중 동일 해시 내용의 중복 228,920byte를 확인했습니다. 새 V2는 원래 36개 경로의 봉인은 모두 유지하면서 서로 다른 내용 21개를 한 번씩만 보관하는 수정입니다. 이는 모델·입력·기준을 바꾸는 작업이 아니며, V1 코드는 수정하지 않았습니다.

## 원본-패키지 실제 V2: 350장 모두 일치

<!-- HH_261007 - Report genuine whole350 one-frame pixel comparisons without claiming sensor-header truth, perception improvement or a second independent replay. -->

[V2 순수 테스트 55개](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_consistency_source_test_v2.json)와 별도 소스 검토를 통과한 뒤, [새 ROOT 선언·권한·검토](../../../../../storage/experiments/plans/2026-10-07/map_raw_package_consistency_v2/ROOT_review.json) 아래 **실제 전량 비교를 완료**했습니다. [실제 결과](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_consistency_v2/result.json)는 **350/350 PASS·실패0·미실행0**, 실제 소요 **4.56129초**입니다. Town01 118장·Town02 232장 순서와 시각을 그대로 사용했습니다.

| 비교한 것 | 실제 결과 |
| --- | --- |
| 원본 RGB↔package RGB | 전체 350장, RGB 바이트 241,920,000개 중 불일치0 |
| 원본 semantic의 red 채널에서 원래 코드로 복원한 라벨↔package 정답 | 전체 정답 픽셀 80,640,000개 중 불일치0 |
| 두 집합의 7종 정답 픽셀 합계 | 원래 manifest·평가 기록의 값과 동일 |
| 프레임·종료 기록의 최대 관측 VmHWM | 52,494,336byte, 약50.1MiB. 원래512MiB 관측 한도 유지. 종료 때의51,642,368byte만 최대값으로 취급하지 않음 |
| 새 모델 계산·학습·ROS·GPU | 모두0. 저장 prediction도 해시만 확인하고 배열은 해독하지 않음 |

전체 입력 717개·현재/원래 소스 36개·고유 소스 사본 21개와 ROOT 권한은 종료 때도 해시가 일치했습니다. 계획·결과 합산 2MiB 한도와 종료용 512KiB 예약을 유지하고 한 프레임씩만 읽었습니다. 기존 V1 중단·원본 package·영상·정답·예측·가중치·평가 기준은 변경하지 않았습니다. 결과 SHA-256은 `f09675f82c3bd276608498960607fe8e8e774b218fb7a1b0c8f004d1975cd283`입니다.

[별도 전량 후검산](../../../../../storage/experiments/diagnostics/2026-10-07/map_raw_package_consistency_v2_independent_postcheck_v1.json)도 **완료**했습니다. 현재 소스36개·입력717개·고유 사본21개·ROOT 원본/사본을 두 번 확인하고 전체350행·두 집합의7종 합계·기존6개 평가 참조가 일치했습니다. 계획25파일+결과3파일의 실제 논리 크기는 **1,686,832byte**로2MiB 안입니다. 검산 중 ordinal을 global 순번으로 잘못 가정한 오류 등도 실제 픽셀 실패와 구분해 공개했습니다. 이는 별도 기록·해시 검산이지 두 번째 픽셀 재생이나 센서 헤더 정합 실험이 아닙니다. 실제 작업은 exit0으로 종료했으며 사전에 PID 세대를 저장하지 않아 PID 현재 부재를 독립 세대 종료 증명으로 쓰지 않습니다.

**해석:** 이번 두 집합의 표지 오류를 package 영상 변환이나 7종 정답 복원 불일치로 설명할 근거는 없습니다. 다만 저장된 RGB와 semantic 사이의 센서별 시각 헤더 정합, 실제 표지 객체 정답성, 학습 crop의 손실이나 모델 구조가 원인인지까지 입증한 것은 아닙니다. 원본과 package가 같은 잘못된 라벨을 공유할 가능성을 이 검사만으로 배제하지 못합니다. 따라서 원래 23/27 판정과 4개 recall 실패·후보 미채택·정식0/9·0/30은 그대로입니다.

<!-- HH_261007 - Prioritize evidence-led checks; future training needs separate authorization and cannot trade away recall or redefine failed criteria. -->

| 순서·상태 | 할 일과 지켜야 할 경계 |
| --- | --- |
| 완료 | 기존 480프레임 좌표·누적 검산과 최신 세 방식의 1,050개 표지 크기·경계 진단. 모델 가중치 변화 없음 |
| 1. 다음 비학습 확인 | 새 선언 후, 동일한 전체 자료에서 작은 영역·17–64픽셀 손실·큰 TP0 오탐을 원본 영상과 나란히 확인한다. 유리한 장면만 골라 결과를 대체하거나 연결영역을 실제 객체 수로 부르지 않는다. 현재 표지 진단은 RGB를 디코딩하지 않았으므로 원인 확인용 영상 검토는 별도 작업이다 |
| 2. 소스·참조·픽셀 전량 확인 완료 | 라벨 순서·전체 350장 참조·평가 crop 부재와 원본↔package 픽셀 일치를 확인했다. 학습 crop의 정답 지지 손실, 센서별 헤더 정합·실제 객체 정답성은 아직 별도 확인 대상이다. 정답을 출력에 맞춰 고치거나 자료를 재생성하지 않는다 |
| 3. 별도 실제 평가 필요 | 최신 모델의 지연, 실제 센서의 깊이·측위·보정, 새로운 독립 장소를 확인한다. 기존 좌표 일치·노출된 두 집합의 픽셀 지표로 이를 대신하지 않는다 |
| 향후 학습: 아직 실행하지 않음 | 원인 확인 후 별도 승인 아래 TRAIN만 사용해 작은 표지·17–64픽셀 누락을 보호하는 표본/손실 설계와 큰 오탐 억제를 함께 비교한다. 부모·대조군과 같은 자료·예산으로 검증하고, 오탐 감소를 얻으려고 recall 손실을 허용하거나 기존 27개 기준을 낮추지 않는다. 현재 자료·가중치·head는 변경하지 않는다 |
| 추가 기능: 미구현·미검증 | 실사용 수준의 실시간·기록 주행 AMG, 실제 센서 기반 정밀지도, 신호 상태·차선 ID/연결·횡단보도/정지선 등은 별도 입력·정답·누적·평가가 필요하다. 기록 재생·oracle 보조 누적 진단은 위의 480프레임 검사에 있지만, 완성된 AMG는 아니며 현 7분류와 좌표 검산만으로 완료 처리하지 않는다 |

가중치 자동 교체·평가 기준 완화·실패 프레임 제거·새로운 head만으로 기능 완료 처리는 하지 않는다. 위 다음 단계는 우선순위이며 새 촬영·추론·학습을 실행했다는 영수증이 아니다. [현재 비학습 작업 상태](../README.md)를 따른다.

근거 수치 파일 SHA-256: `a367c1f594cc5518ac6d6a0df8bff8aa51cd946395108967870fc948fe7fcd7b`; 입력/라벨 manifest SHA-256: `614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b`. V2 표지 결과 SHA-256: `bdf4fa1583e73a74eaec8121895416e0f2539f4ed3e7f75cddd42c214d3009b6`; 독립 후검산 SHA-256: `d998f84ecde70c1bfb3abc0656b8f18f8e4d963fb8bd7c5380bee5809675dbe5`.

이번 추가 작업은 새 비학습 도구·선언·실제 결과와 현재 안내 문서에 반영했다. 모델 가중치·원본 영상·저장 예측·이전 동결 PLAN/코드·과거 실패 영수증은 수정하지 않았다.
