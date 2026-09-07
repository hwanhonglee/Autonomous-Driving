# Portable E2E — 반복 학습·검증과 새 우회전 데이터

<!-- HH_260906 - Keep learned-model validation, expert demonstrations, and transfer proofs in separate categories. -->

2026-09-07 실행 자료. **새 Portable 모델의 실차·차량 제어 승인은 없다.**
이 폴더의 CARLA 카메라 영상은 학습 정답을 수집한 BasicAgent 주행이며, 학습 모델의
closed-loop 주행 성공 영상이 아니다. 기존 Autoware VAD 주행은
[별도 제어·shadow 검증](../control_ab_30kph_and_60kph_readiness_v1/README.md)에서 확인한다.

| 폴더 | 들어 있는 내용 |
|---|---|
| [01 학습률 A/B 완료](01_learning_rate_ab/README.md) | 기존 train613 / val337, 학습률 2개 × seed 3개, 경로·속도 오차와 v8 검사 |
| [02 새 우회전 학습 데이터](02_new_training_town01_right/README.md) | Town01 expert 실제 카메라·차량 중심 경로 PNG/GIF, 충돌·차선침범·수집 실패 기록 |
| [03 독립 직진 test 데이터](03_held_out_test_town04_straight/README.md) | Town04 새 주행의 수집 품질·PNG/GIF; 모델 선택에는 사용하지 않음 |
| [04 후보 선택 진단](04_selection_diagnostics/README.md) | 후보들이 다른데 한 index만 선택하는 문제, 전체 val337 수치와 고정 구간 PNG |
| [05 데이터 전송 검증](05_dataset_transfer/README.md) | train/val/test 분리, 기존 표본 불변성, 로컬·원격 SHA 검증 |
| [06 우회전 데이터 추가 학습](06_data_expansion/README.md) | train1147, 같은 1,540 step으로 3개 seed 완료; 절대 품질 미달 |
| [07 후보 선택 손실 A/B](07_selector_weight_ab/README.md) | 같은 v3 데이터·seed·학습량, 선택 손실 0.1 → 0.5; 3개 seed 완료 |
| [08 선택 손실 변경 경로 분석](08_selector_weight_diagnostics/README.md) | 첫 seed C/D 전체 val337 진단, 차량 중심의 후보·expert 경로 PNG |
| [09 코드 검증](09_code_validation/README.md) | 전체 회귀 테스트 결과, 건너뛴 테스트의 환경 제약 |
| [10 학습 목표·선택 오차 원인 분석](10_objective_alignment/README.md) | C/D 6개 모델, train/val 분리, 복합 loss와 위치 오차 oracle 비교 |
| [11 후보 경로 인지 모델 C/E 비교](11_candidate_rank_ab/README.md) | 새 선택 구조 3개 seed 학습 완료, 실제 학습 곡선·고정 품질 판정 |
| [12 새 모델 차량 중심 경로 분석](12_candidate_rank_route_analysis/README.md) | E의 전체 val337 진단, 고정 6시점 × 3개 seed 원본 PNG 18장 |
| [13 최신 코드 검증](13_candidate_rank_code_validation/README.md) | 새 모델·진단·집계 포함 전체 2,333 passed / 6 skipped 원본 로그 |

## 두 환경의 역할

- **Pro6000:** 개인 py312 venv, GPU0만 사용. 기존 데이터 A/B 6회, 새 데이터 학습 3회,
  선택 손실 변경 3회, 후보 경로 인지 선택 구조 3회를 완료했다. **4개 캠페인 / 총 15회**다.
  모델마다 학습 → val 평가 → 경로 형상 검사를 수행했다. test는 열지 않았다.
  마지막 학습 캠페인 단계는 23:34:14 KST, 후속 CPU 경로 진단은 약 23:34:30 KST에
  끝났다. 현재 이 캠페인들의 학습·진단은 실행 중이 아니다. 자동으로 모든 기능을
  추가하는 무한 학습 작업은 아니다.
- **로컬:** 새 Town01/Town04 expert 수집·Common10 변환, 원격 결과 수집·분석·발행.
  후속 E 구조 실험에서는 추가 주행 촬영 없이 코드 검증과 분석 자료를 발행했다.
  이번에는 새 Portable 모델로 차량 제어를 실행하지 않았다.

## 현재까지 확정된 데이터 결과

| 구분 | Town01 우회전 train 추가 | Town04 직진 test 신규 |
|---|---:|---:|
| 경로 길이 | 228.251 m | 210.010 m |
| 목적지 도달 | PASS | PASS |
| 충돌 / 차선 침범 | 0 / 0 | 0 / 0 |
| Native camera Hz | 9.999999851 | 9.999999851 |
| Common10 표본 | 534 | 309 |

새 v3는 **train 1,147 / val 337 / test 309**, 총 1,793개다. 새 우회전 episode의 534개가
모두 회전 중인 표본은 아니다. 현재 navigation command는 RIGHT 18개, LANEFOLLOW
516개이며, 미래 궤적의 회전 범위와 현재 command는 다른 지표다.

로컬·원격 파일 **10,786개 / 1,033,543,479 bytes**와 tree SHA-256이 일치하고 planning
검증도 통과했다. 기존 v2의 train/val sample JSONL과 route geometry는 그대로 보존했다.

## 결과 해석

첫 A/B 쌍은 selected ADE가 4.2615 → 4.1509 m로 조금 개선됐다. 하지만 c2 고정이 c4
고정으로 바뀌었을 뿐이고, 정답 기준 최선 후보 대비 평균 선택 손실은 2.5009 → 2.5155 m로
조금 증가했다. 후보 간 평균 경로 간격은 약 7.5 m로, 여섯 후보가 동일하다는 뜻은 아니다.

전체 6회는 11:31:15 KST에 완료했다. seed `20260903`, `20260904`에서는 낮은 학습률이
개선됐지만 `20260905`에서는 ADE 4.2580 → 4.5922 m, FDE 11.0934 → 11.5660 m로
악화했다. **상대 선별 FAIL, 절대 목표 FAIL로 학습률 변경은 채택하지 않았다.**
실제 [학습 곡선](01_learning_rate_ab/visuals/01_training_curves.png)과
[세 seed 비교 그래프](01_learning_rate_ab/visuals/02_paired_validation.png)를 함께 확인할 수 있다.

새 데이터 후속 실험은 동일한 **1,540 optimizer step**을 사용했다. train 크기가 달라져 약 5.37 epoch이며,
이 실험을 기존의 10 epoch와 동일 epoch 조건이라고 표시하지 않는다.

우회전 데이터를 추가한 C의 세 seed ADE는 `3.8743 / 6.2585 / 4.7895 m`였다. 일부 결과는
좋아졌지만 절대 목표에는 모두 미달했다. 이어 같은 v3에서 선택 손실 가중치를 높인 D는
`4.3049 / 5.3370 / 3.8286 m`로, 두 seed는 좋아지고 첫 seed는 나빠졌다. D의 FDE는
세 seed 모두 줄었지만 **세 seed 모두의 ADE/FDE 개선**이라는 고정 기준은 통과하지 못했다.
따라서 앞선 세 실험 모두 완료하되 모델을 채택하지 않았다. checkpoint 12개와 실제 실행
runner의 원본 bytes는 private 자료로 보존·검증했고, 공개 metadata는 원본 SHA와
계정 경로 치환 여부를 함께 표시했다.

이어 C/D 6개 모델에서 학습 목표와 위치 오차 oracle의 차이를 train/val로 분리 계측했다.
val에서 두 oracle의 ADE 차이는 0.033–0.103 m인 반면, 모델 선택과 학습 oracle 사이
차이는 2.036–4.742 m였다. 목표 정의 차이만으로 큰 선택 오차를 설명하기 어렵다.
CPU/GPU의 선택 빈도 차이가 나타난 D seed04는 양쪽 수치를 분리 보존했으며 원인을
확정하지 않았다.

후보 XY·속도를 함께 보고 점수를 계산하는 E를 새로 구현·학습했다. E의 selected ADE는
`4.7357 / 4.5391 / 4.2284 m`로 두 seed는 C보다 개선됐지만 첫 seed는 악화됐다.
FDE·속도 오차는 모두 줄고 geometry는 336/337로 유지됐으나 **상대 선별 FAIL,
절대 품질 FAIL**이다. 기존 운용 checkpoint는 유지했다. 새 모델은 parameter 수와
선택 head 깊이도 다르므로 궤적 입력 하나만의 인과 효과라고 해석하지 않는다.
[설계와 판단 근거](../../../../portable-e2e-candidate-ranking.md)를 함께 참고한다.

새 E checkpoint는 원격과 로컬의 private 원본으로 보존했다. 원격 학습·평가·CPU 진단
전후 checkpoint SHA와 로컬 checkpoint bytes를 검증했으며, 집계 JSON의 세 E 모델
local byte verification은 모두 true다. 실제 실행 worker 원본 bytes도 로컬에 보관·검증했다.

다음은 선택기를 분리한 통제 실험과 독립적인 회전·정지 episode 확장이다. 후보 index
빈도를 균등하게 만드는 것만으로 개선 처리하지 않는다. 이후 기능별 데이터·label·판정을
추가하며 반복한다.

두 corpus의 fingerprint가 달라 v2↔v3 비교는 탐색적 데이터 확장 분석이다. 원래
`compare.py`의 동일-corpus 비교 승인으로 대신 표시하지 않는다. geometry PASS도
충돌·도로 안전 PASS가 아니다. 독립 test와 learned closed-loop 검증은 별도 단계다.

## 자료 검증

01–08은 130개 파일, 182,663,804 bytes다. 체크섬 122개, JSON 71개, JSONL 12개
(18,480개 기록), 문서 내부 경로 45개를 검증했다. 공개 자료와 private 원본 66쌍의
hash·크기도 확인했다. PNG 28개를 디코딩했으며 GIF 2개는 앞서 전체 400 frame을
디코딩한 원본과 hash가 같다. 공개 metadata에서는 계정별 경로를 치환하고 원본 SHA를
남겼다. 원본 checkpoint·데이터셋·runner는 공개 자료에 포함하지 않는다.

추가한 10–12는 **57개 파일 / 4,845,588 bytes**다. 체크섬 54개, JSON 27개,
JSONL 3개(4,620개 학습 기록), PNG 21개, 문서 링크 27개와 public/private 원본
manifest 26쌍을 검증했다. C/E 집계를 원본에서 재계산해 공개 결과와 일치함을 확인했고,
E의 세 checkpoint bytes와 실제 실행 worker SHA도 검증했다.

앞선 코드 회귀 결과 **2,155 passed / 6 skipped**는 09에 보존했다. 최신 변경을 포함한
전체 결과는 13의 **2,333 passed / 6 skipped**다. 코드 검증과 모델 성능 판정을 구분한다.
10–12의 새 그래프·경로 PNG는 실제 수치와 예측을 그린 것이며 새 주행 녹화가 아니다.
임시의 미완료 집계와 초기 수집 사전 점검 실패 자료는 private 원본에 남겨 최종 성공
기록과 섞지 않았다.

자세한 운용법은 [반복 학습 안내](../../../../portable-e2e-learning-loop.md), 기능별 다음
조건은 [9개 상위·30개 하위 기능표](../../../../portable-e2e-feature-roadmap.md)를 참고한다.
