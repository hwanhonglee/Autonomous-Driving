# Town03 자료 추가 지도 모델: 학습은 완료, 성능 관문은 실패

<!-- HH_260906 - Keep the trained candidate, repeated DEV regression, and sealed Town07 decision distinct. -->

**결론: 모델 B는 채택하지 않습니다.** 개인 PRO6000의 물리 GPU 0에서 2,000-step 학습은 정상 종료했지만, 동일한 Town01 DEV 118장에서 기존 AI v2보다 차선·표지판·평균 분할 성능이 낮아졌습니다. 따라서 [사전 계획](../map_next_factor/PLAN.md)의 DEV 보존 기준을 통과하지 못했습니다. [Town07 봉인 결정](../map_next_factor/town07_freeze.json)에 따라 이 후보로 Town07 원본·정답·예측 화면을 열거나 평가하지 않았습니다. 체크포인트는 `deployable=false`, 결과는 `PILOT_COMPLETE_NOT_PROMOTED`이며 자동 승격하지 않았습니다.

## 무엇을 학습했나

기존 AI v2 지도 의미 분할 모델과 **같은 구조(130,503 파라미터), seed 20260922, RGB 640×360, batch 8, 2,000 step, optimizer/증강/loss 공식**으로 처음부터 다시 학습했습니다. 단일 실험 요인은 TRAIN 자료 구성입니다. 기존 Town01 497장에 소유 CARLA BasicAgent가 수집한 Town03 직진 232장·회전 184장을 추가해 TRAIN 913장으로 만들었습니다. 프레임은 같은 Town03 내에서 0.25m 중복 제거했고, 기존 Town01 TRAIN 배열 497장은 동일한 prefix, Town01 DEV 118장 NPZ는 동일한 바이트입니다. 이 모델의 입력은 RGB이고 CARLA 의미 라벨은 정답으로만 썼습니다. **학습 모델이 주행한 자료가 아니며**, 자동차 제어·정밀지도·실차 성능의 검증이 아닙니다.

실행 기록은 개인 venv `/home/pro6000/personal/hwanhong/portable_e2e/venvs/py312`, 물리 GPU **0** (`GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`), CPU 사전검사 1회·최적화 1회, 종료 코드 0, 최적화 시간 **75.62초**입니다. 재시도·자동 승격은 없었습니다. [로컬 포장 기록](../map_generalization_town03/TRAIN_PACKAGE.md)과 [원격 실행 영수증](mapping_generalization_20260927_v1.exit.json), [학습 로그](mapping_generalization_20260927_v1.launch.log)를 분리해 보존했습니다.

## 같은 Town01 DEV의 결과

수치는 IoU/정밀도를 %로 표시했고 변화는 **퍼센트포인트(pp)**입니다. DEV는 앞서 여러 번 확인한 개발 자료이지 새로운 독립 TEST가 아닙니다.

| 지표 | 기존 AI v2 A | Town03 추가 B | 변화 | 사전 기준 판정 |
|---|---:|---:|---:|---|
| 도로 IoU | 98.751 | 98.565 | −0.187 pp | 하락 ≤1 pp: 통과 |
| 차선 표시 IoU | 91.233 | 77.089 | **−14.144 pp** | 하락 ≤2 pp: **실패** |
| 7클래스 평균 IoU | 60.121 | 55.967 | **−4.154 pp** | 하락 ≤2 pp: **실패** |
| 표지판 IoU | 3.157 | 0.856 | **−2.301 pp** | 기존 이하 금지: **실패** |
| 표지판 정밀도 | 3.293 | 0.886 | **−2.407 pp** | 기존 이하 금지: **실패** |
| 신호등 기구 IoU | 15.180 | 5.999 | −9.181 pp | 별도 참고 지표 |

표지판 정답 2,915픽셀 중 `other`로 잘못 분류한 픽셀은 A **1,495개 → B 2,113개**였고, 표지판으로 예측했으나 오검출인 픽셀은 **37,061개 → 65,680개**였습니다. 이는 저장된 118장 예측과 정답에서 혼동행렬을 다시 계산해 원격 결과 JSON과 일치시킨 값입니다. 절대적으로 A의 표지판 정밀도도 낮으므로, B의 실패가 A의 충분한 품질을 뜻하지 않습니다.

새 Town03 416장에 포함된 표지판 의미 라벨은 **140픽셀**뿐입니다. TRAIN 전체에서 표지판 픽셀 비율은 기존 **67,794/114,508,800 (0.0592%)**에서 **67,934/210,355,200 (0.0323%)**로 낮아졌습니다. 반대로 차선 표시 픽셀은 **287,660 → 2,834,271**로 늘었고, 기존 class weight `lane_marking` **1.4623 → 0.8016**, `traffic_sign` **2.1720 → 2.6512**로 TRAIN 빈도 공식에 따라 변했습니다. 이 변화는 재현 가능한 진단 사실이지, 하락 원인을 한 가지로 확정하는 증거는 아닙니다. 서로 다른 Town 외관·라벨 분포·고정 2,000-step 예산 등의 영향을 분리하려면 새로 사전 등록한 A/B가 필요합니다.

## 전체 화면 근거

동일 Town01 DEV의 사전 선택한 첫 장·중간 장·마지막 장을 원본 화각 그대로 RGB / CARLA 정답 / A / B 네 화면으로 표시했습니다. 이 장면들은 독립 일반화 증거가 아니라 결과 해석용입니다.

- [프레임 000 PNG](town01_dev_000_full_fov.png) · [프레임 059 PNG](town01_dev_059_full_fov.png) · [프레임 117 PNG](town01_dev_117_full_fov.png)
- [세 장 비교 GIF](town01_dev_ab_full_fov.gif)

## 무결성 및 다음 관문

| 고정 항목 | SHA-256 또는 확인 결과 |
|---|---|
| Town03 직진 경로 | `aee6b0fd6e5a6dfa882cd172b4cb10c2e0865d2729d3233f5f121bb4be0674d4` |
| Town03 회전 경로 | `8aa4040617f733e5df11910d3ac8113307e05a53334f29bc3387eb1370e2bf3c` |
| 직진/회전 소유 실행 결과 | `830e47ff4c95a380d747d7c50512c4556f7e152736d620db434e8470d90aefc2` / `7e85478b64b4651ae3a0d4f28dbcc02d46ee4e1868d4437c4350cfae75abaa16` |
| 직진/회전 240쌍 원본 센서 해시 목록 | `a062b366c6fc4ce727f246b342d9356b44c90e62ac88cd1294e3cab90f436841` / `10e6b8f8743b2fd8b49c4c2d86ecedfd77c82d9e54a9b29239cec4de79614030` |
| 새 학습 데이터 manifest / TRAIN NPZ / 그대로 둔 DEV NPZ | `e5f4ec6ab46f16f2f2167f4b453abe4bea829ac9bce7a1215a64d7a41444de0d` / `1328dd608eb642cc8c4b0d9be9d07cb391639a6a67b426fe2bbdb703e300e440` / `417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3` |
| AI v2 A / 새 B 체크포인트 | `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380` / `12c63095e1a6c401e7772ddf9441c727e3e96ee523623995f1368bf818826fff` |
| B 훈련 코드 / 모델 구조 코드 | `86e583aacf654d78ce913a95ba537e0025317aa5a737aefacc2162aa7e2adceb` / `1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b` |

원격에서 복사한 [checkpoint](remote_run/checkpoint.pt), [시작 조건](remote_run/started.json), [결과·혼동행렬](remote_run/summary.json), [loss 계약](remote_run/loss_contract.json), [데이터 manifest](remote_run/data_manifest.json), [DEV 예측](remote_run/dev_predictions.npz), [훈련 이력](remote_run/history.jsonl)을 보존했습니다. 로컬에서 위 해시, GPU 0·2,000 step, 원래 DEV 정답과 B 예측의 혼동행렬 일치, 저장된 A 결과와 원격 재평가 A의 정확한 일치를 재검산했습니다.

**다음은 B를 더 학습시키는 자동 반복이 아닙니다.** Town07 봉인을 유지한 채 Town03 추가 자료의 클래스 분포·라벨 정합성과 도메인 차이를 진단하고, 하나의 변경 요인만 둔 새 실험을 미리 등록해야 합니다. Town01 DEV 보존 관문을 통과한 후보만 향후 Town07 단회 평가 대상으로 삼습니다. 어떤 후보도 추가 Town·날씨·실차·시계열 지도 품질 검증 전에는 지도 모델 또는 주행 모델로 채택하지 않습니다.
