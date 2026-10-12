# 지도 학습 정답 보존: 완전히 잘린 표지판 노출이 약 절반 감소

<!-- HH_261008 - Report actual target-only counterfactual improvement separately from model accuracy, future trainer integration and independent validation. -->

**같은20,000배치를 실제로 재생한 결과, 표지판 정답의 완전 잘림은3,626→1,821회로 줄었습니다.** 줄어든1,805회는 정답 입력 품질 개선입니다. 모델 가중치를 바꾸거나 인식 성능을 평가한 결과는 아닙니다. 정식 완료는 메인0/9·하위0/30입니다. 08:17에 실행 종료를 확인했고 독립 전체 저장·집계 검산도 완료했습니다.

<!-- HH_261008 - Distinguish the later real RGB worker implementation from this frozen target-only counterfactual result. -->
**이후 RGB·정답 worker와 writer도 구현했습니다.** 실제 8배치는 양쪽 원래 해시를 재현했고 독립 저장 검산도 완료했습니다. 다만 선택한 64회 노출의 복구는 0회라 실제 RGB 복구 효과는 아직 입증하지 않았습니다. [이후 구현·실제 결과·다음 순서](../../nontraining_batch_capture/README.md)를 확인하세요. 아래 20,000배치 수치는 **정답 전용 실험**이며, trainer 연결·학습·모델 개선은 아직 아닙니다.

## 무엇을 고쳤나요

기존 학습은 사진을 자르고 확대하는 증강을 사용했습니다. 원래 화면에 표지판 정답이 있어도 자른 사진에서는 전부 사라지는 경우가 있었습니다. 새 정책은 처음부터 정한 **짝수 노출 순번**에서만, 그런 경우 원본 전체화면으로 되돌립니다.

표지판이 없던 정상 배경, 홀수 순번, 자른 뒤에도 표지판이 남는 경우는 원래 제안 box를 그대로 유지합니다. 표지판을 만들어 넣거나 다른 사진을 뽑지 않습니다. 이 정답 전용 실행에서는 승인된 TRAIN의 정답만 재생했고, RGB나 모델을 실행하지 않았습니다. 이후 worker 구현과 별개로 실제 trainer 연결은 아직 미구현입니다.

| 확인 항목 | 기존 | 새 정책 |
| --- | ---: | ---: |
| 표지판 정답이 있는 노출 | 77,291 | 77,291 |
| crop 뒤 정답이 완전히 없어짐 | 3,626 | 1,821 |
| 양성 노출 중 완전 잘림 | 4.691% | 2.356% |
| 증강 뒤 표지판 정답 픽셀 | 12,492,557 | 12,629,819 |
| 전체 증강 픽셀 중 표지판 | 0.033888% | 0.034261% |

완전 잘림은49.78% 줄었지만 **표지판 정답 픽셀 총량은1.10% 증가**에 그칩니다. 아직 전체에서 매우 드뭅니다. “정답이 남았다”와 “모델이 작은 표지를 잘 인식한다”는 다른 결과입니다. 부분 잘림2,616회는 그대로이며 resize만으로 완전히 없어진 경우는 두 조건 모두0입니다.

## 어느 자료에서 달라졌나요

| TRAIN 출처 | 정답 양성 노출 | 기존 완전 잘림 | 새 완전 잘림 | 보존된 노출 |
| --- | ---: | ---: | ---: | ---: |
| 기존 Town01,497개 | 36,124 | 1,395 | 714 | 681 |
| 기존 Town02,232개 | 30,705 | 80 | 35 | 45 |
| Town01 p02 FALSE,240개 | 6,753 | 1,319 | 688 | 631 |
| Town01 p04 FALSE,240개 | 3,709 | 832 | 384 | 448 |
| 합계 | 77,291 | 3,626 | 1,821 | 1,805 |

보호 순번은 전체80,000/160,000회입니다. 각 출처의 절반을 무조건 맞춘 것은 아닙니다. 실제 보호/미보호는 위 순서대로32,754/33,012·15,491/15,214·15,912/15,854·15,843/15,920회입니다. 이 고정 순번 선택을 독립 무작위 표본이나 최적 정책으로 주장하지 않습니다.

다른 클래스의 기여도 같이 달라졌습니다. 도로 정답 픽셀은약0.185%, 차선 표식0.156%, 신호등 구조물0.362%, 기둥0.133% 감소했습니다. 보도는0.048%, 미확인other는0.186% 증가했습니다. 정책이 다른 인식 기능에 도움이 되는지까지는 별도 학습·회귀 평가가 필요합니다. class weight를 자동 변경하지 않았습니다.

## 실제 실행과 검산 범위

- 로컬 CPU3·스레드1, 실제192.69초. 관측 최대 VmHWM325,926,912바이트.
- 원본729개와 paired FALSE480개의 TRAIN `target.npy`만 해독. 전체 package 바이트의 해시는 확인하지만 RGB·TRUE·DEV/HOLDOUT/TEST·가중치 배열은 열지 않음.
- 원래20,000개 B8 정답 해시·해시 체인·1,209개 슬롯/그룹/분류 통계를 전량 재계산했고 과거 결과와 동일.
- 계산한 공통 노출은160,000회. 원래 두 군의 분모320,000회와 각20,000 PASS·실패0·미실행0을 그대로 보존.
- 후보 target/box는 새 해시로 구분. 기존 결과·source·trainer·데이터 registry 변경0, 종료 seal 오류0.
- 모델 추론·학습·ROS·CARLA·GPU·새 실제 지도 생성0회. 메모리/시간은 관측 검사이며 OS 강제 차단 보장 아님.

[실제 결과와 모든1,209개 후보 통계](../../../../../../storage/experiments/diagnostics/2026-10-08/map_train_crop_preservation_counterfactual_v1/result.json) · [원문 실행 계획](../../../../../../storage/experiments/plans/2026-10-08/map_train_crop_preservation_counterfactual_v1/ROOT_plan.json) · [별도 ROOT 권한](../../../../../../storage/experiments/plans/2026-10-08/map_train_crop_preservation_counterfactual_v1/ROOT_authority.json) · [실행 전 검토](../../../../../../storage/experiments/plans/2026-10-08/map_train_crop_preservation_counterfactual_v1/ROOT_execution_review_v1.json).

[독립 전체 저장 검산](../../../../../../storage/experiments/diagnostics/2026-10-08/map_train_crop_preservation_counterfactual_v1_independent_postcheck_v1.json)은106개 지정 파일의 앞뒤 전체SHA·파일 세대, 양쪽 원래20,000행, 원본1,209개 메타데이터, 전체/4그룹/1,209슬롯의1,214개 통계행과7클래스 분모를 확인했습니다. 정답 픽셀을 두 번째로 재생하거나 후보의 새픽셀 해시를 독립 계산한 것은 아닙니다.

## 다음 학습에 어떻게 연결하나요

<!-- HH_261008 - Preserve the one-factor training design and actual RGB/GT journal requirements; no trainer mutation is authorized by this plan. -->

1. 기존 파일을 고치지 않고 새 TRAIN 전용 wrapper를 만듭니다. 연결 지점은 [CropBatches.__getitem__](../../../../../../tools/e2e/train_map_native_fusion_pair_20261002.py)의 배치 생성 호출입니다. 전역 crop 함수를 바꾸면 다른 실험과 평가까지 영향을 줄 수 있어 피합니다.
2. 원래 배치·RNG·FALSE 선택을 유지하고, 선택한 **같은 box**를 [원래 RGB BILINEAR·정답 NEAREST 함수](../../../../../../tools/e2e/map_appearance_matched_contract_20261001.py)에 전달합니다. NumPy 정수와 Python 정수 타입 차이·RGB 바이트·worker 순서까지 새로 검증해야 합니다.
3. 원래 제안 box, 실제 선택 box, rescue 여부, RGB/정답 해시를 전부 새 journal에 저장합니다. crop이 바뀌므로 기존 “두 군 입력 바이트가 동일” 검사를 그대로 통과시키거나 동일했다고 부르면 안 됩니다.
4. **같은 모델 구조·초기 가중치·손실·optimizer**로 crop 한 가지만 A/B 비교합니다. 기존 native-fusion 실험은 군별 구조가 달랐으므로 그 architecture 비교에 crop 변경까지 섞으면 원인을 분리할 수 없습니다.
5. 평가의 원래 full-FOV·27개 판정·실패 분모를 유지하고 recall·오탐·기존 기능 퇴행을 확인합니다. 과거 노출된 Town02 p02를 새 독립 TEST라고 부르지 않습니다.
6. 필요하다면 희귀 클래스 손실·샘플 설계를 별도의 다음 실험으로 비교합니다. 이번 데이터 품질 개선을 이유로 추가 head·학습 횟수·자동 모델 승격을 바로 적용하지 않습니다.

원격 환경·GPU0 가용성의 현재 확인과 실제 trainer 연결은 아직 전입니다. 이후 로컬 RGB·정답 worker의 제한 8배치 검증은 완료했지만 학습은 시작하지 않았습니다. 독립 인식 평가와 지도 품질 검증까지 좋아져야 실제 모델 개선으로 판단합니다.

## 이번에 준비한 새 journal 검사

<!-- HH_261008 - Keep scalar preparation and full synthetic tests distinct from a real trainer, byte authentication or learned gains. -->

[새 scalar journal 계약](../../../../../../learning/portable_e2e/map_crop_preservation_journal_20261008_v1.py)은 원래6필드 노출 기록을 보존하면서 선택 box·보호/복구 여부·새 RGB/정답 해시를 별도 schema에 남길 수 있게 검증합니다. 변경 없는 배치는 원래 두 해시가 같아야 하고, 복구 배치는 새 실제 해시를 원래와 같다고 강요하지 않습니다. 제공된 GT count·SHA 문자열만으로 실제 픽셀이나 RNG 진위를 인증하지는 않습니다.

독립 소스 검토 후 원래58개 검사와 전체20,000행 합성 추가 검사를 합친 **59개 검사 통과·4.54초**를 확인했습니다. 추가 검사는 stub 없이 실제 배치/정책 검사 함수를 전부 호출했고 마지막 위조 행도19,999 PASS·1 FAIL로 남겼습니다. 원래58개는59개 안에 포함되며117개라고 합산하지 않습니다. **합성 메타데이터이지 실제RGB/정답 노출 journal이 아닙니다.** [실제 검사 명령·결과·소스 해시](../../../../../../storage/experiments/diagnostics/2026-10-08/map_crop_preservation_journal_source_test_v1.json)를 확인하세요.

위 59개 스칼라 검사 당시에는 파일 writer·RGB/정답 worker가 없었습니다. 이후 새 revision과 별도 제한 권한에서 worker·writer를 구현하고 실제 8배치를 검증했습니다. [최신 구현 결과](../../nontraining_batch_capture/README.md)를 따르며, 원격 학습·기존 trainer 수정·모델 승격은 여전히 하지 않았습니다.

[지도 전체 분석](../README.md) · [현재 두 모델의 역할](../../model_scope/README.md) · [오늘 전체 현황](../../README.md).
