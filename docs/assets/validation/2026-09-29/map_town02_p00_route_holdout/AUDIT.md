# Town02 p00 추가 학습 A/B 독립 결과 감사

<!-- HH_260929 - Recompute saved map predictions before interpreting route-holdout gains. -->

**판정: 저장 결과 재계산 통과, 실험 관문 실패, 모델 승격 없음.** B는 Town02_Opt p02의 차선 표시를 개선했지만, Town01의 차선 표시와 신호등 형상 정확도가 떨어졌다. 이번 B를 기존 AI v2 지도 모델 대신 사용하지 않는다.

## 실험과 독립 검사 범위

[사전 계약](PLAN.md)에 따라 A는 Town01 학습 497장, B는 그 497장을 **동일한 배열 순서로 유지하고** Town02_Opt p00 232장을 추가한 729장이다. 양쪽은 같은 7클래스 모델을 처음부터, 같은 seed와 가중치 계산·증강·batch 8·각 2,000 step으로 학습했다는 실행 기록을 남겼다. GPU 기록은 물리 0번 RTX PRO 6000의 UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`다. 기록상 학습 구간은 A **75.59초**, B **74.39초**로 합계 **149.98초**다.

[독립 검사기](../../../../../tools/e2e/audit_map_town02_p00_route_holdout_result.py)는 학습기 평가 함수를 호출하지 않고 [패키지](../../../../../storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1/manifest.json)의 정답과 [A/B 결과](../../../../../storage/experiments/training/2026-09-29/map_town02_p00_holdout_run_v1/summary.json)의 저장된 예측 배열을 직접 결합했다. 두 팔 × p02 232장/Town01 118장, 총 **700장의 프레임별 7클래스 IoU**와 **4개의 7×7 혼동행렬**을 재계산했다. 프레임별 수치, 혼동행렬의 49개 칸, 클래스별 정답 픽셀 수, 전체 IoU·정밀도·픽셀 정확도가 저장 기록과 모두 일치했다. 7클래스 프레임별 IoU는 이번 700장에서 모두 정의됐다. 분모가 0인 경우를 `0`이나 PASS로 바꾸지 않는 검사도 포함한다.

패키지 manifest SHA-256은 사전 고정한 `f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834`와 일치한다. 네 split NPZ의 해시·개수·형식·라벨 픽셀 수와 B의 A 497장 접두부 동일성을 확인했다. 보관된 학습기·지원 모듈 15개의 해시, A/B 체크포인트 2개의 해시, 각 팔의 100~2,000 step 간격 20개 손실 기록, 부모 재현 기록도 대조했다. 부모 체크포인트의 실제 바이트는 이 로컬 묶음에 없어서, 그 SHA `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380`는 사전 계약과 실행 영수증 간 일치만 확인했다. 이 저장 예측 감사 자체는 새 추론을 하지 않았고, 별도의 [CPU 체크포인트 재추론](CHECKPOINT_CPU.md)이 이후 700프레임의 픽셀 일치성을 검사했다.

## 전체 7클래스 결과

수치는 원본 픽셀을 합친 IoU다. `Δ`는 B−A, 단위는 퍼센트포인트(%p).

| 클래스 | p02 A | p02 B | p02 Δ | Town01 A | Town01 B | Town01 Δ |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 기타 | 90.14% | 88.27% | −1.87 | 95.01% | 95.37% | +0.36 |
| 도로 | 94.12% | 95.49% | +1.37 | 98.75% | 98.99% | +0.24 |
| 보도 | 86.31% | 86.48% | +0.18 | 92.82% | 93.42% | +0.61 |
| 차선 표시 | 92.07% | 95.38% | +3.31 | 91.23% | 88.74% | **−2.50** |
| 신호등 형상 | 17.83% | 12.44% | **−5.39** | 15.18% | 13.02% | **−2.16** |
| 교통 표지 | 34.22% | 42.08% | +7.85 | 3.16% | 3.39% | +0.23 |
| 기둥 | 18.22% | 26.88% | +8.67 | 24.70% | 26.93% | +2.22 |

사전 관문 중 p02 차선 **+2.0%p 이상**과 도로 **−1.0%p 이내**, Town01 도로 **−0.5%p 이내**는 통과했다. Town01 차선 **−0.5%p 이내** 및 Town01 전체 7클래스 각각 **−0.5%p 이내**는 실패했다. 따라서 다섯 조건을 동시에 충족하지 못했다. A의 Town01 7클래스 IoU는 이전 AI v2 기준과 영수증상 정확히 같았다(`a_reproduction.json` 차이 0); 이 검사는 저장된 기준값과의 일치 검산이다.

Town02_Opt p02는 **두 팔에서 학습에 넣지 않은 경로**이지만, 과거 D1 학습과 분석에서 이미 결과를 본 내부 경로다. 새로운 독립 DEV나 봉인 TEST로 부르지 않는다. Town01 DEV도 이미 알려진 장소다. 승인된 독립 DEV 장소는 **0곳**이다. p00과 p02의 보존 ego 위치는 최소 약 **197.04m** 떨어져 있지만 동일 Town의 외관을 공유한다. 이번 숫자는 실제 주행 성능이나 다른 Town 일반화, 정밀지도 완성, 10 Hz 운영 또는 실차 가능성을 증명하지 않는다.

<!-- HH_260929 - Distinguish linked local evidence from files available in a fresh clone. -->
아래 `storage/experiments`의 원본 NPZ·예측·체크포인트와 이 문서에서 연결한 JSON은 **로컬 보관 자료이며 `.gitignore` 대상**이다. 이 저장소를 새로 clone한 것만으로는 373 MB 데이터 패키지나 7.6 MB 학습 결과가 내려오지 않는다. Git에 포함되는 분석 문서·전체 화각 PNG/GIF는 결론 열람용이고, 픽셀 단위 재실행에는 별도 해시 일치 아카이브 전달이 필요하다. 이 차이를 숨기고 'clone 후 바로 동일 학습 재현'이라고 안내하지 않는다.

## 재현 명령과 다음 조치

저장 자료만 읽는 검사다. SSH, GPU, CARLA를 사용하지 않는다.

```bash
cd /home/a/autoware_e2e
python3 tools/workspace_python.py tools/e2e/audit_map_town02_p00_route_holdout_result.py \
  --data storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1 \
  --run storage/experiments/training/2026-09-29/map_town02_p00_holdout_run_v1
python3 tools/workspace_python.py -m pytest -- -q \
  learning/tests/test_audit_map_town02_p00_route_holdout_result.py
```

실측 결과는 `INDEPENDENT_SAVED_ARTIFACT_AUDIT_PASS`, 단위 시험 **4 PASS**다. [화면 영역별 오류 분석](ERROR_ANALYSIS.md)은 Town01 차선 퇴행이 상단 오검출 증가에 집중됨을 확인했다. 실험 실패 후 같은 p02 수치로 임계값을 바꾸거나 B를 승격하지 않는다. 다음 지도 작업은 독립 장소 DEV 원본을 확보하고, 노면/차선과 신호등 형상의 라벨 품질·클래스 불균형을 별도로 진단한 다음 새 실험을 사전 등록하는 것이다. 주행 E2E의 폐루프 기능 승인은 이 지도 A/B와 무관하며 현재 **9개 상위/30개 하위 중 0/30**이다.
