# 첫 paired seed의 경로 선택·후보 다양성 진단

<!-- HH_260906 - Publish measured selector and geometric diagnostics without promoting the model. -->

같은 CARLA train613/val337, seed `20260903`, 10 epoch·1,540 step의 두 checkpoint를
비교했다. A의 learning rate는 `0.0001`, B는 `0.00003`이다. 이 폴더는 **첫 paired seed의
val 진단 완료** 기록이며, 전체 3-seed 캠페인 완료나 모델 승격을 선언하지 않는다.
현재 운용 모델을 자동 교체하지 않으며 Portable learned closed-loop 실적은 여전히 0회다.

| 측정 항목 | A baseline | B lower LR |
|---|---:|---:|
| 전체 val sample | 337 | 337 |
| 선택 index | c2: 337/337 | c4: 337/337 |
| ADE oracle index별 빈도 c0~c5 | 57 / 36 / 156 / 30 / 28 / 30 | 32 / 43 / 36 / 21 / 171 / 34 |
| selector와 ADE oracle 일치 | 156/337, 46.29% | 171/337, 50.74% |
| selected ADE | 4.261500 m | 4.150919 m |
| oracle ADE | 1.760646 m | 1.635451 m |
| 평균 selection regret | 2.500854 m | 2.515469 m |
| 후보 간 평균 경로 거리 | 7.534858 m | 7.545791 m |
| 후보 간 평균 최종점 거리 | 19.852814 m | 19.586472 m |

선택 index는 c2에서 c4로 바뀌었지만 각각 하나의 index에 고정됐다. 후보 간 거리가 0이
아니므로 **선택 index 고정이 여섯 후보 경로의 동일함을 뜻하지는 않는다**. B의 selected ADE는
약 2.59% 개선됐고 ADE oracle 일치율도 증가했지만, 평균 regret는 약 0.0146 m 커졌다.
따라서 선택 빈도, 후보 자체 품질, selector가 놓친 기회를 함께 분석해야 한다.

`ADE oracle`은 각 sample의 유효 expert XY에서 ADE가 가장 작은 후보이며, 정확히 같은
값이면 앞 index를 선택한다. speed 등을 포함하는 학습 loss의 oracle과는 정의가 다르다.
`regret = selected ADE - oracle ADE`이고, 후보 간 경로 거리는 15개 unordered pair의
64개 예측 시점 전체에서 계산한 Euclidean distance 평균이다. 이 숫자는 충돌 안전성이나
주행가능영역 통과 판정이 아니다.

## 실제 경로 그림·원본 수치

그림 index는 예측을 보기 전에 `floor(phase × 336 / 5), phase=0..5`로 고정한
`0, 67, 134, 201, 268, 336`이다. 차량은 화면 중앙, 회색은 route, 초록색은 expert,
여섯 색 경로는 모델 후보이고 굵은 색 경로가 선택 결과다.

- [A 전체 val 수치와 sample별 진단](A_baseline/selector_diagnostic.json)
- [B 전체 val 수치와 sample별 진단](B_lower_lr/selector_diagnostic.json)
- A 그림: [0](A_baseline/val_phase_000.png), [67](A_baseline/val_phase_067.png), [134](A_baseline/val_phase_134.png), [201](A_baseline/val_phase_201.png), [268](A_baseline/val_phase_268.png), [336](A_baseline/val_phase_336.png)
- B 그림: [0](B_lower_lr/val_phase_000.png), [67](B_lower_lr/val_phase_067.png), [134](B_lower_lr/val_phase_134.png), [201](B_lower_lr/val_phase_201.png), [268](B_lower_lr/val_phase_268.png), [336](B_lower_lr/val_phase_336.png)

원본 JSON 2개와 PNG 12개는 실행 산출물의 bytes를 그대로 보존했다. 각 JSON에 PNG별
hash가 있고, 이 폴더의 [SHA256SUMS](SHA256SUMS)에는 README를 포함한 파일 checksum이 있다.

## 실행 환경·재현 경계

기존 Pro6000 개인 venv에서 CPU만 사용하고 CUDA 장치를 노출하지 않았다. torch intra-op,
inter-op, OMP, MKL은 각각 4 threads로 제한했으며 package 설치나 진행 중 GPU 학습 변경은
없었다. A는 `2026-09-07T02:23:03.944611Z`부터 `02:23:15.023016Z`까지 약 11.078초,
B는 `02:23:41.117573Z`부터 `02:23:52.109289Z`까지 약 10.992초 걸렸다. 이 시간은
파일 검증·CPU 추론·그림 저장을 포함한 진단 소요 시간이며 **10 Hz live runtime이나 GPU
추론 성능의 증거가 아니다**.

Town03 우회전의 기존 `val` 337개만 읽었다. 새 Town04 `test`는 학습·선택·진단에
사용하지 않았다. 각 checkpoint는 기대 SHA를 필수로 받고 기존 secure weights-only loader,
corpus 일치, train/val episode 비중복 검사를 통과했다.

| 고정 항목 | SHA / revision |
|---|---|
| A checkpoint | `66f9031b0939c0321dca563400f4e363f6a42f223a364526021902e2d4611a46` |
| B checkpoint | `f6b7e70bde941213e1f7df033bb354e36b2e6aa0414eb42b5684abaa22fb785d` |
| 진단 script | `1524e18a92758a8dc95bf86a4d7c11524a139e5c027fd4e6d9ba0f17a7dfc022` |
| 학습 코어 source commit | `081a71f7fc014d2864b790b0b0cae7378ae18e4c` |
| corpus fingerprint | `17c248440efca864e6c322ca5a1602d08cd1a0eabfa71e10545049181a86e073` |
| val tensor fingerprint | `631be7323f502cafc0dd66766104203bd5e7fee7494436c479bff1e0dddc0285` |

전체 반복 흐름은 [학습·검증 반복 가이드](../../../../../portable-e2e-learning-loop.md),
진단 CLI는 [diagnose_portable_selector.py](../../../../../../scripts/e2e/diagnose_portable_selector.py)에 있다.
