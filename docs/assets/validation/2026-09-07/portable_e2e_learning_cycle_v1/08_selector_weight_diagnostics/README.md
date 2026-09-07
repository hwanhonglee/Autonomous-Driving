# 첫 seed C/D — selector loss 가중치와 실제 후보 선택 진단

<!-- HH_260906 - Preserve measured first-seed selector diagnostics without inferring benefit from selection diversity alone. -->

같은 v3 corpus의 train 1,147개·Town03 우회전 val 337개, seed `20260903`, LR `0.0001`,
batch 4, 1,540 step에서 candidate score loss weight를 **C `0.1` → D `0.5`**로 바꾼
checkpoint를 진단했다. 이 확장 corpus에서 1,540 step은 10 epoch가 아니다.
이 폴더는 **첫 seed의 val 진단**이며 전체 3-seed 결론이나 모델 승격을 대신하지 않는다.

| CPU 진단 항목 | C: score weight 0.1 | D: score weight 0.5 |
|---|---:|---:|
| val sample | 337 | 337 |
| 선택 index별 빈도 c0~c5 | 0 / 0 / 294 / 43 / 0 / 0 | 150 / 24 / 115 / 48 / 0 / 0 |
| ADE oracle index별 빈도 c0~c5 | 31 / 32 / 218 / 25 / 14 / 17 | 139 / 35 / 112 / 19 / 15 / 17 |
| selector와 ADE oracle 일치 | 232/337, 68.84% | 175/337, 51.93% |
| selected ADE | 3.874334 m | 4.304883 m |
| oracle ADE | 1.552181 m | 1.590711 m |
| 평균 selection regret | 2.322154 m | 2.714173 m |
| 후보 간 평균 경로 거리 | 7.431474 m | 7.230519 m |
| 후보 간 평균 최종점 거리 | 19.513897 m | 18.319624 m |

D는 선택하는 index가 두 종류에서 네 종류로 늘었지만, ADE oracle 일치율은 낮아지고
selected ADE와 평균 regret는 커졌다. **선택 빈도가 다양해진 것만으로 선택 품질이
개선됐다고 판단할 수 없다.** 후보 경로들은 두 모델 모두 서로 다르므로 이 현상을 후보
geometry가 모두 같아진 것으로 해석하지 않는다. 이 첫 seed에서는 더 큰 score 가중치의
이점을 입증하지 못했으며 운용 모델을 자동 교체하지 않는다.

참고로 원래 GPU 최종 평가의 ADE/FDE는 C `3.874328/9.827706 m`, D
`4.304872/9.448977 m`였다. FDE 한 지표는 낮아졌지만 ADE와 위 선택 진단은 악화됐다.
이 표의 CPU ADE와 GPU 평가 ADE에는 각각 약 `0.000006/0.000011 m`의 작은 수치 차이가
있어 별도 값을 그대로 보존했다. CPU 재분석 결과를 원본 GPU 평가 값으로 덮어쓰지 않았다.

## 숫자의 의미와 전체 경로 그림

ADE oracle은 유효 expert XY에서 ADE가 가장 작은 후보이며 정확한 동률은 앞 index를
선택한다. speed 등을 포함하는 학습 loss의 oracle과는 정의가 다르다.
`selection regret = selected ADE − oracle ADE`다. 후보 간 경로 거리는 15개 unordered
pair의 64개 예측 시점 전체에서 계산한 Euclidean distance 평균이고 최종점 거리는 마지막
예측점 기준이다. 이 거리들은 충돌·주행가능영역 안전 판정이 아니다.

예측을 보기 전에 고정한 그림 index는 `floor(phase × 336 / 5), phase=0..5`에 따른
`0, 67, 134, 201, 268, 336`이다. 화면 중앙은 ego 차량이며 회색은 route, 초록색은 expert,
여섯 색은 모델 후보, 굵은 색은 선택 결과다.

- [C 전체 val337 수치·sample별 진단](C_expanded_data/selector_diagnostic.json)
- [D 전체 val337 수치·sample별 진단](D_selector_weight/selector_diagnostic.json)
- C 그림: [0](C_expanded_data/val_phase_000.png), [67](C_expanded_data/val_phase_067.png), [134](C_expanded_data/val_phase_134.png), [201](C_expanded_data/val_phase_201.png), [268](C_expanded_data/val_phase_268.png), [336](C_expanded_data/val_phase_336.png)
- D 그림: [0](D_selector_weight/val_phase_000.png), [67](D_selector_weight/val_phase_067.png), [134](D_selector_weight/val_phase_134.png), [201](D_selector_weight/val_phase_201.png), [268](D_selector_weight/val_phase_268.png), [336](D_selector_weight/val_phase_336.png)

## 실제 실행·provenance

기존 개인 venv에서 CUDA 장치를 숨기고 CPU만 사용했다. torch intra-op/inter-op,
OMP/MKL은 각각 4 threads였다. C는 `2026-09-07T02:58:08.125933Z`부터
`02:58:20.702109Z`까지 12.576초, D는 `02:59:09.630659Z`부터
`02:59:22.257275Z`까지 12.627초 걸렸다. 이는 자료 검증·CPU 추론·PNG 저장을 포함한
소요 시간으로 **live 10 Hz 또는 GPU inference 성능의 증거가 아니다**.
package 설치, GPU 사용, 진행 중 학습 변경, source 변경은 하지 않았다.

두 진단은 같은 script와 현재 source `27ca6153fa4fdb88022d018ab478fe9eed916a25`의
기존 secure loader·모델 구현으로 실행했다. C의 학습 source는 이전 commit이므로 아래에서
구분한다. 기대 checkpoint SHA, corpus 일치, train/val episode 비중복을 검사했고
Town04 held-out `test`는 열지 않았다.

| 고정 항목 | SHA / revision |
|---|---|
| C 학습 source | `081a71f7fc014d2864b790b0b0cae7378ae18e4c` |
| D 학습 source | `27ca6153fa4fdb88022d018ab478fe9eed916a25` |
| C checkpoint | `cc43adb16d3d2677b5c05235c7563bc71bb267094abf3f1e3378ddfdb79149fd` |
| D checkpoint | `867c7c21c328cb12720f42e99c12550ba10511f9b689c1ebe5dc2421ada41d02` |
| 진단 script | `1524e18a92758a8dc95bf86a4d7c11524a139e5c027fd4e6d9ba0f17a7dfc022` |
| v3 corpus fingerprint | `56d9ff663612a090cd61698b53cf0cf39b0a7ae0de6b42d096957a94b6c92047` |
| val tensor fingerprint | `631be7323f502cafc0dd66766104203bd5e7fee7494436c479bff1e0dddc0285` |

원본 JSON 2개와 PNG 12개에는 계정별 경로가 없어 bytes 그대로 발행했다. JSON의 PNG별
hash와 [SHA256SUMS](SHA256SUMS)로 검증할 수 있다. checkpoint 자체는 private 자료에
보관한다. 전체 반복 과정은 [학습·검증 반복 가이드](../../../../../portable-e2e-learning-loop.md),
도구는 [selector 진단 CLI](../../../../../../scripts/e2e/diagnose_portable_selector.py)에 있다.
