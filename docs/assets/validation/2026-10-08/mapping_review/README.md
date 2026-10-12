# 지도 모델: 정답이 학습 과정에서 얼마나 보였나요?

<!-- HH_261008 - Explain actual TRAIN-label support without calling crop evidence a model gain, new training or independent driving approval. -->

이번에는 **학습을 다시 하지 않고, 과거 학습에 사용된 정답을 실제로 재생해 확인했습니다.** 20,000개 배치가 모두 과거 정답 해시와 일치했습니다. 정답 저장·증강 재현은 확인했지만 모델 인식 성능이 좋아졌다는 결과는 아닙니다. 정식 기능 완료는 메인0/9·서브0/30을 유지합니다.

<!-- HH_261008 - Route readers to the later actual byte-worker integration while retaining the original mask and target-only analyses. -->
**이후 RGB·정답 배치 연결부도 구현·검증했습니다.** 실제 8배치·64회 노출에서 원래 RGB와 정답 해시를 재현했고, 새 journal과 지정 파일 50개의 독립 저장 검산을 완료했습니다. 복구 조건 발생은 0회이며 모델 성능 개선으로 세지 않습니다. [최신 구현·모의 센서 수집 검사·다음 순서](../nontraining_batch_capture/README.md)를 먼저 보세요. 아래는 그보다 앞선 정답·예측 마스크 분석입니다.

## 지금 이어서 하는 일

<!-- HH_261008 - Separate the feasible paired-mask diagnostic from unavailable confidence and incompatible live capture ownership. -->

**07:49 KST: 실제 픽셀의 위치별 겹침 수까지 확인한 V4를 완료했습니다.** 350장·세 군1,050건,6.38초·실패0·미실행0이며 독립 저장 집계 검산도 완료했습니다. 같은 정답 표지판 픽셀 중 양쪽이 맞춘 것, 부모만 맞춘 것, 후보만 맞춘 것, 둘 다 놓친 것을 네 부분으로 나눴습니다.

| 정답 픽셀 기준 | 둘 다 맞춤 | 부모만 맞춤 → 후보가 놓침 | 후보만 새로 맞춤 | 둘 다 놓침 |
| --- | ---: | ---: | ---: | ---: |
| Town01 DEV,118장 | 1,344 | 516 | 20 | 1,035 |
| Town02 노출된 진단,232장 | 52,494 | 2,563 | 305 | 2,712 |

예를 들어 Town01의 실제 누락516개와 새 회복20개의 차이는496개입니다. 앞선 V3의 순감소496개와 맞지만, 순감소만으로516개·20개를 따로 알 수는 없었습니다. 이 네 값은 실제 같은 정답 픽셀의 분할 **개수**이며, 좌표·객체 수·bbox를 새로 저장했다는 뜻은 아닙니다. 표지판이 아닌 영역의 오탐 위치 변화도 이 표에 포함하지 않습니다.

대조군→후보는 Town01 양쪽1,338·대조군만198·후보만26·둘 다 누락1,353개, Town02는51,969·1,048·830·4,227개입니다. 기존350행의 모든 V3 지표·27개 판정은 동일하며 여전히23개 통과·4개 실패·후보 미채택입니다. [V4 실제 결과](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v4/result.json), [독립 집계·파일 검산](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v4_independent_postcheck_v1.json), [실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/map_paired_sign_regions_v4/ROOT_review.json)를 보존합니다. 독립 검산은 저장된350행·집계·해시 검증이며, 마스크 픽셀을 두 번째로 재실행한 것은 아닙니다. 새NN·학습은0입니다.

<!-- HH_261008 - Record actual target-only preservation gains without equating them to learned recognition or an integrated trainer. -->
**TRAIN 정답 보존 정책의 실제20,000배치 재생과 독립 저장 검산을 완료했습니다.** 원래 정답 해시와1,209개 슬롯 통계가 전량 동일한 것을 먼저 확인했고, 새 정책은 완전 잘림3,626→1,821회(양성 노출의4.691→2.356%)로 줄였습니다. 복구1,805회이며 실패0·미실행0, 계산192.69초입니다. 표지판 정답 픽셀 총량 증가는1.10%로 작고, 다른 클래스 기여도 일부 바뀌었습니다. **학습 입력 품질 개선이지 모델 정확도 향상은 아닙니다.** RGB/trainer 연결·실제 학습은 아직 전입니다. 새 journal59개 스칼라 검사도 완료했습니다. [읽기 쉬운 결과·그룹별 수치·다음 학습 설계](crop_preservation/README.md)를 확인하세요.

고정350장의 **같은 정답 연결영역**을 부모·대조군·후보 사이에서 비교하는 V3를 실제로 완료했습니다. 전체 세 군1,050건 통과·실패0·미실행0, 계산6.31초이며 관측 최대 VmHWM은68,354,048바이트입니다. 로컬 CPU3 하나·스레드1개에서 저장 정답과 최종 분류 마스크만 읽었습니다. RGB 해독·NN 추론·새 학습은0입니다. 실행 성공은 분석 절차의 완료이지 인식 성능 합격이 아닙니다. [독립 사후 검산](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v3_independent_postcheck_v1.json)도 완료했습니다. 저장350행·1,050개 분모·원래 지표·집계·파일 해시는 별도로 검산했지만, 픽셀 연결영역과 CRC 계산을 두 번째로 재실행했다는 뜻은 아닙니다. 검산 코드의 첫 assertion 실수도 별도 실패 원본으로 보존했습니다.

| 같은 정답 영역의 비교 | 부모가 잡았지만 후보가 완전히 놓침 | 부모가 놓쳤지만 후보가 새로 잡음 | 실제 맞춘 정답 픽셀: 부모 → 후보 |
| --- | ---: | ---: | ---: |
| Town01 기존 DEV,118장·199영역 | 15 | 5 | 1,860 → 1,364 |
| Town02 노출된 진단,232장·731영역 | 91 | 9 | 55,057 → 52,799 |

후보는 부모보다 오탐을 줄였지만 **기존에 인식하던 정답도 더 많이 놓쳤습니다.** Town01은 대조군→후보에서 완전 누락3개·새 회복6개, Town02는16개·16개입니다. 같은 누락 개수라도 놓치는 영역이 같다는 뜻은 아닙니다. 이 결과로 다음 학습에서 단순히 오탐 억제만 강화하면 안 된다는 근거가 추가됐지만, 원인 전체를 확정하거나 후보를 교체하지는 않습니다.

연결영역은 실제 표지판 객체 수가 아닙니다. 영역별 맞춘 픽셀 수의 순증감과 실제 픽셀 위치 변화를 구분하며, bbox·새로 맞힌 픽셀 위치를 복원했다고 부르지 않습니다. 전체 정답·마스크8개의 EOF/CRC·해시를 검증했고 기존 **23/27 통과·4개 실패·후보 미채택**도 그대로 재계산했습니다. [실제 결과·전량350행·세군 집계](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v3/result.json), [원문 준비 계획](../../../../../storage/experiments/plans/2026-10-08/map_paired_sign_regions_v3/prepared_candidate.json), [ROOT 실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/map_paired_sign_regions_v3/ROOT_review.json)을 보존합니다.

[V1에서 발견한 표현 문제](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v1_source_finding_v1.json)는 소스 단계에서 발견했습니다. V2의 사전 검사는72개 통과·5개 실패여서 배열 실행 전에 중단했고 [실패 원본](../../../../../storage/experiments/diagnostics/2026-10-08/map_paired_sign_regions_v2_source_test_failure_v1.json)을 남겼습니다. V3는 모의 CPU 설정과 AST 비교 검사만 바로잡고109개 검사를 통과했습니다. 계산식·1,050개 분모·기존27개 판정·출력2MiB 한도는 유지했습니다. 더 엄격한 행 크기 상한2,560바이트의 전체 정상행 증명으로 원래 마감 여유를 지켰습니다. 중간 프레임은 압축 파일 끝까지 CRC와 전체 정답 해시를 확인하기 전에는 통과로 바꾸지 않았습니다.

최신 6개 저장 예측 파일에는 최종 `uint8` 분류 마스크만 있습니다. 원래 평가 코드가 `argmax` 뒤 저장하므로 **확률·confidence를 정직하게 복원할 수 없습니다.** 다른 모델·과거 촬영의 confidence를 대신 쓰거나 확률 보정 파라미터를 학습하지 않습니다.

새 촬영 연결도 모의 helper 통과만으로 실행 가능한 상태는 아닙니다. 기존 paired p02/p04의480개에는 이미 네 센서의 frame·시각·pose·원시 해시가 보존돼 있습니다. 반면 새 ownership V2의 직접 자식·상속 lease·정확한 loopback listener 조건은 기존 shell 실행기와 맞지 않습니다. CARLA0.9.15 공식 소스의 포트 전용 서버 생성은 IPv4 wildcard를 사용하므로, client 주소가 `127.0.0.1`이라는 이유로 서버 bind도 같다고 간주하면 안 됩니다. 실제 설치 바이너리의 현재 bind는 관측하지 않았습니다. 안전 조건을 낮추거나 기본 소프트웨어를 변경하지 않습니다. [CARLA 설정 소스](https://raw.githubusercontent.com/carla-simulator/carla/0.9.15/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Settings/CarlaSettings.cpp), [서버 소스](https://raw.githubusercontent.com/carla-simulator/carla/0.9.15/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp), [의존성 pin](https://raw.githubusercontent.com/carla-simulator/carla/0.9.15/Util/BuildTools/Setup.sh), [rpclib 구현](https://raw.githubusercontent.com/carla-simulator/rpclib/v2.2.1_c5/lib/rpc/server.cc).

## 중요한 결과 세 가지

1. 표지판 정답이 있었던77,291회 중3,626회(4.69%)는 crop으로 완전히 잘렸습니다. 이후 resize만으로 사라진 경우는0입니다.
2. 새 p02/p04 FALSE 촬영은 전체 정답 노출의39.71%이지만, 증강 후 표지판 정답 픽셀의2.43%만 제공합니다. 학습 샘플 수가 많다는 것과 어려운 클래스의 정답이 많이 보인다는 것은 다릅니다.
3. 전체 증강 정답 픽셀 중 표지판은약0.034%입니다. 다만 **표지판 클래스의 정답 픽셀**이지 개별 객체 수·신호등 상태·모델 재현율이 아닙니다.

| 정답 출처 | 전체 노출 | 표지판 정답 양성 노출 | crop으로 완전히 잘림 | 양성 중 잘림 | 증강 후 표지판 정답 픽셀 기여 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 기존 Town01,497개 | 65,766 | 36,124 | 1,395 | 3.86% | 78.49% |
| 기존 Town02,232개 | 30,705 | 30,705 | 80 | 0.26% | 19.08% |
| 새 Town01 p02 FALSE,240개 | 31,766 | 6,753 | 1,319 | 19.53% | 1.60% |
| 새 Town01 p04 FALSE,240개 | 31,763 | 3,709 | 832 | 22.43% | 0.83% |

이 표의 잘림 비율은 **양성 정답 노출을 분모**로 합니다. 한 이미지의 여러 표지판을 구분한 객체별 통계가 아닙니다. 부분적으로 잘린2,616회는 완전히 잘린3,626회와 별도 집계입니다.

## 무엇을 실제로 실행했나요?

- 실행 장소: 로컬, 논리 CPU3 하나·스레드1개. CARLA·ROS·GPU·추론·optimizer는 실행하지 않았습니다.
- 배열 입력: 승인된 TRAIN의 원본729개와 paired FALSE480개 `target.npy`만. RGB·TRUE view·DEV/HOLDOUT·CTrack·가중치 배열은 해독하지 않았습니다. 파일 전량 SHA 검증의 압축 바이트 읽기는 배열 해독과 구분합니다.
- 분모: 두 과거 군의 배치 기록이 바이트까지 같으므로160,000회 정답 노출을 한 번 계산했습니다. 양 군의 원래 분모는 합계320,000회로 보존했고 각 군20,000개 배치가 전량 일치했습니다.
- 검산: 원본1,209개 정답의 해시와7개 클래스 픽셀 수도 전량 일치했습니다. 배치 실패0·미실행0, 종료 뒤 소스·입력 변경0입니다.
- 실제 재생 시간:145.97초. 관측 최대 VmHWM:325,603,328바이트(약310.52MiB). 최종 결과 파일 쓰기 이후의 추가 메모리는 측정하지 않았고 OS 강제 메모리 제한을 주장하지 않습니다.
- 로컬 Python/NumPy/Pillow 버전은 과거 원격 학습과 다릅니다. 환경 동등성이 아니라 실제 정답 바이트 일치를 확인했습니다.

## 이 결과로 무엇을 바꿀 수 있나요?

다음 학습 때는 총 프레임 수만 늘리기 전에, 표지판이 실제로 보존되는 crop 비율·샘플별 정답 기여·희귀 클래스 손실을 함께 설계할 근거가 생겼습니다. 표지판이 없는 정상 배경도 필요하므로 이를 무조건 빼거나, 원하는 결과가 나오도록 안전·평가 기준을 낮추지는 않습니다.

**현재 후보의 성능 퇴보 원인은 아직 확정되지 않았습니다.** 두 군이 동일한 crop 정답을 사용했으므로 이 검사만으로 두 모델 차이를 설명할 수 없습니다. 가중치·손실 구성·오탐/누락·평가 출처를 별도로 비교해야 합니다. 과거 기준27개 중23개 통과·4개 실패, 후보 미채택이라는 판정도 바꾸지 않습니다.

## 학습과 평가 자료가 섞였나요?

후속 [촬영 출처 메타데이터 검산](../../../../../storage/experiments/diagnostics/2026-10-08/mapping_cohort_provenance_readonly_v1.json)도 완료했습니다. 현재1,209개 정답과 평가350개의 **capture 폴더+frame stem으로 구분한 기록 출처 교집합은0**이며, 같은 형식의 기록된 PNG/NPZ 해시 교집합도0입니다. 둘 다232개인 Town02 자료는 서로 다른 촬영입니다. 현재 학습은p00, 기존 진단 평가 쪽은p02입니다.

하지만 기존p02 평가232개가 과거 D1 학습 자료와 같은 기록 출처였다는 점도 확인했습니다. 따라서 현재 학습에 섞인 것은 아니어도, **그 평가 자료를 새 독립 TEST라고 부를 수 없습니다.** 기존의 노출된 진단 자료라는 구분을 유지합니다. 이번 검산은 실제 PNG/NPZ/가중치를 열지 않은 메타데이터 검사이며, 모든 과거 모델의 학습 이력이나 픽셀 비중복까지 재증명하지는 않습니다.

과거 원본 capture의 학습 금지·미승인 상태와 별도 파생 연구 허가도 구분해 보존했습니다. 이번 검사는 원래 연구 기록을 확인한 것이지, 실패 원본을 새 TRAIN으로 승인하거나 실제 학습에 추가한 것이 아닙니다. 다음 학습은 정답 기여·희귀 클래스·오탐/누락 설계를 함께 검토하고, 별도 독립 평가 자료가 필요한 상태입니다. 새 학습과 새 CARLA 촬영은 이번 작업에 포함하지 않습니다.

## 지금 인식 가능한 분류와 아직 없는 기능

<!-- HH_261008 - Separate learned output vocabulary from future fusion labels and unknown sensor/geometry authentication. -->

현재 RGB 모델의 출력은 **7종**입니다. 도로·보도·차선 표식·신호등 구조물·표지판·기둥의6종과 `other`입니다. `other`는 **미확인**이며 빈 공간이나 물체 없음 판정이 아닙니다. 이 목록은 출력 이름만 설명하며 인식 정확도 합격을 뜻하지 않습니다.

누적 지도 코드는 횡단보도·정지선·도로 경계와 차량·보행자·자전거·신호 상태도 받을 수 있게 되어 있지만, **현재 모델이 이 항목을 출력하는 것은 아닙니다.** 특히 신호등 구조물을 찾았다고 빨강/노랑/초록을 알아낸 것이 아닙니다. CARLA semantic의 차선 tag24와 모델 출력 index3도 서로 다른 번호 체계입니다.

이를 혼동하지 않도록 [새V2 준비 코드와 모의 검사93개](../../../../../storage/experiments/diagnostics/2026-10-08/mapping_baseline_capability_contract_source_test_v2.json)를 확인했습니다. 전체 입력·미확인 점을 보존하고 잘못된 분류 순서·범위 밖index·좌표·프레임·선언한 출처는 거절합니다. [원본V1의88개 검사](../../../../../storage/experiments/diagnostics/2026-10-08/mapping_baseline_capability_contract_source_test_v1.json)도 보존했습니다. 원본 오류 문구의 “모든 oracle ID를 거절”은 과장이었습니다. 숫자1·2·6은 두 번호 체계에서 겹치므로, 올바른 범위와 `learned_prediction` 선언만으로 실제 모델 출처를 인증할 수 없습니다. 새V2에서 이 한계를 명시했습니다.

기존 모델·누적 지도는 수정하지 않았으며, 실제 센서 출처·깊이·3D 보정·차로 연결 관계·HD-map 인증은 여전히 미검증입니다. 불변 객체도 임의 생성된 객체의 진실성을 인증하는 보안 봉인은 아닙니다. 실제 새 지도 생성0회·추론0회·학습0회입니다.

## 결과와 재현 근거

- [실제 재생 결과: 모든 배치·샘플·그룹·실패 분모](../../../../../storage/experiments/diagnostics/2026-10-08/map_native_fusion_crop_support_v2/result.json)
- [독립 사후 검산: 91개 경로·전체 기록·파일 세대·실행 종료 확인](../../../../../storage/experiments/diagnostics/2026-10-08/map_native_fusion_crop_support_v2_independent_postcheck_v1.json)
- [원문 ROOT 실행 계획](../../../../../storage/experiments/plans/2026-10-08/map_native_fusion_crop_support_v2/ROOT_plan.json), [별도 실행 권한](../../../../../storage/experiments/plans/2026-10-08/map_native_fusion_crop_support_v2/ROOT_authority.json), [실행 전 검토](../../../../../storage/experiments/plans/2026-10-08/map_native_fusion_crop_support_v2/ROOT_execution_review_v1.json)
- [실제 CPU 재생 소스 V2](../../../../../tools/e2e/audit_map_native_fusion_crop_support_20261008_v2.py), [모의 검사120개·로그 저장 모의 검사66개 기록](../../../../../storage/experiments/diagnostics/2026-10-08/nontraining_source_checks_v2.json)

독립 검산은 모든 저장 기록·집계·정답 해시 체인을 다시 확인한 **메타데이터 검산**입니다. 픽셀을 두 번째로 재생했거나 모델을 독립 평가했다고 표현하지 않습니다. 과거 실패 revision·소스·결과는 삭제하거나 덮어쓰지 않았습니다.

[오늘 전체 진행 상황으로 돌아가기](../README.md)
