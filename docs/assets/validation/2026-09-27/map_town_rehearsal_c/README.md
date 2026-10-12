# 지도 모델 C: B보다 회복했지만 AI v2 보호 관문 실패

<!-- HH_260906 - Separate a successful GPU0 optimization from model-quality and holdout approval. -->

**결론: 후보 C를 채택하지 않습니다.** Town01:Town03 TRAIN 배치 노출을 **7:1**로 바꾼 뒤 물리 GPU 0에서 2,000-step 학습은 정상 완료했습니다. 같은 Town01 DEV에서 B보다 좋아졌지만, 미리 고정한 AI v2 대비 **차선 IoU 하락 최대 2%p**와 **표지판 IoU·정밀도 비퇴행**을 충족하지 못했습니다. [Town07 보류 결정](town07_holdout_decision.json)에 따라 이 후보로 Town07의 RGB·의미 라벨·예측을 열거나 점수를 만들지 않았습니다. C 체크포인트는 `deployable=false`, 실행 결과는 `PILOT_COMPLETE_NOT_PROMOTED`이고 모델을 승격하지 않았습니다.

## 무엇이 달랐나

[사전 계획](PLAN.md)의 단일 변경은 학습용 프레임 **선택 확률**입니다. A는 기존 AI v2(Town01 TRAIN 497장), B는 Town01 497장+Town03 416장 총 913장을 균등 추출, C는 **B와 같은 913장·같은 7클래스 모델·loss/class weight·seed·optimizer·crop·2,000 step**을 쓰면서 각 8장 배치에서 Town01 7장과 Town03 1장을 TRAIN index로만 추출했습니다. C는 B 체크포인트를 이어 학습하지 않고 처음부터 초기화했습니다. DEV/TEST는 샘플러에 사용하지 않았습니다. 공통 Town01 DEV 118장 NPZ는 `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3`으로 동일합니다.

실행기는 v1과 v2를 혼동하면 안 됩니다.

| 실행 | 결과 | GPU 최적화 | 증거 |
|---|---|---:|---|
| v1 | CPU `--validate-only`에서 `CUDA_VISIBLE_DEVICES`의 고정 GPU0 UUID 계약 오류로 종료 코드 1 | **0회** | [실패 영수증](mapping_town_rehearsal_20260927_v1.exit.json) · [실패 로그](mapping_town_rehearsal_20260927_v1.launch.log) |
| v2 | CPU 사전검사 통과 후 물리 GPU0에서 단회 학습, 종료 코드 0 | **1회, 75.48초** | [성공 영수증](mapping_town_rehearsal_20260927_v2.exit.json) · [학습 로그](mapping_town_rehearsal_20260927_v2.launch.log) |

v2는 개인 경로 `/home/pro6000/personal/hwanhong/portable_e2e`의 Python 3.12 venv를 사용했습니다. 물리 GPU0 UUID는 `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`; GPU1은 사용하지 않았습니다. v2의 원격 체크포인트와 소스는 새 경로에 보존했고 v1 결과를 덮어쓰지 않았습니다. v1 오류는 [실행기 수정 기록](V2_EXECUTION_NOTE.md)에 설명되어 있습니다.

## 고정 Town01 DEV 118장의 비교

단위는 %이며 괄호 안은 C가 AI v2 A에서 달라진 **퍼센트포인트(pp)**입니다. 이 DEV는 이미 반복 확인한 개발 자료로, 새로운 Town 일반화나 독립 TEST 결과가 아닙니다.

| 지표 | A: AI v2 | B: 균등 913장 | C: 7:1 복습 | C의 사전 기준 판정 |
|---|---:|---:|---:|---|
| 도로 IoU | 98.751 | 98.565 | 98.375 (−0.377) | 하락 ≤1pp: 통과 |
| 차선 표시 IoU | 91.233 | 77.089 | **88.705 (−2.528)** | 하락 ≤2pp: **실패** |
| 7클래스 평균 IoU | 60.121 | 55.967 | 59.328 (−0.794) | 하락 ≤2pp: 통과 |
| 표지판 IoU | 3.157 | 0.856 | **2.269 (−0.888)** | A보다 낮지 않아야 함: **실패** |
| 표지판 정밀도 | 3.293 | 0.886 | **2.341 (−0.952)** | A보다 낮지 않아야 함: **실패** |
| 신호등 기구 IoU | 15.180 | 5.999 | 11.816 (−3.363) | 참고 지표 |

C는 B에 비해 차선 IoU **+11.616pp**, 평균 IoU **+3.361pp**, 표지판 IoU **+1.413pp** 회복했습니다. 그러나 비교 기준을 B로 완화하지 않았습니다. 표지판 정답 2,915픽셀 중 `other`로 예측한 픽셀은 A **1,495**, B **2,113**, C **1,591**개입니다. 표지판 오검출 픽셀은 A **37,061**, B **65,680**, C **51,386**개입니다. C의 전체 픽셀 정확도는 A보다 높지만 큰 면적 클래스의 영향을 강하게 받으므로 사전 IoU/정밀도 관문을 대신하지 않습니다.

## 전체 화각 화면과 원본 증거

같은 Town01 DEV의 사전 선택한 첫 장·중간 장·마지막 장을 **원본 640×360 화각**으로 RGB / CARLA 정답 / A / B / C 나란히 표시했습니다. 아래 화면은 같은 입력과 저장된 예측을 보여주며, 차량 주행 화면이나 Town07 화면이 아닙니다.

- [프레임 000 PNG](town01_dev_000_abc_full_fov.png) · [프레임 059 PNG](town01_dev_059_abc_full_fov.png) · [프레임 117 PNG](town01_dev_117_abc_full_fov.png)
- [세 장 비교 GIF](town01_dev_abc_full_fov.gif)

복사한 [C 체크포인트](remote_run_v2/checkpoint.pt), [시작 조건](remote_run_v2/started.json), [결과·혼동행렬](remote_run_v2/summary.json), [DEV 예측](remote_run_v2/dev_predictions.npz), [loss 계약](remote_run_v2/loss_contract.json), [데이터 manifest](remote_run_v2/data_manifest.json), [20개 기록의 학습 이력](remote_run_v2/history.jsonl)을 보존했습니다. 로컬에서 아래 SHA와 v1 최적화 0회/v2 최적화 1회, 동일한 B·C `loss_contract.json`, 7:1 TRAIN-only 샘플러, 118장 정답과 C 예측으로 재계산한 혼동행렬의 정확한 일치를 확인했습니다.

| 고정 대상 | SHA-256 |
|---|---|
| B와 C의 동일 TRAIN 데이터 manifest | `e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d` |
| TRAIN NPZ / 변하지 않은 DEV NPZ | `1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440` / `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3` |
| 부모 A / 비교 B / 후보 C 체크포인트 | `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380` / `12c63095e1a6c401e7772ddf9441c727e3e96ee523623995f1368bf818826fff` / `864de725655ef7219218f2b4a9da059630e6e3c30a18a010f74238938aa4a7f6` |
| 동결 C trainer / 모델 구조 코드 | `e1808cb77df8bc7c28d5019eb46c2d40d7e756d2f45ce59d01b43d944ab04de2` / `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b` |

**다음 판단:** C가 B보다 좋아졌다는 사실만으로 Town07 봉인을 해제하거나 추가 학습을 자동 반복하지 않습니다. Town01 차선·희소 표지판 보존 실패의 원인을 분리한 다음 한 요인씩 사전 등록해 검증해야 합니다. 지도 의미 분할의 이 단계는 완성된 정밀지도도, learned-control 주행도, 실차 안전 승인도 아닙니다.
