# J 실패 분석 — route 신호는 맞지만 TRAIN 경로 다양성이 부족하다

<!-- HH_260906 - Distinguish a useful route-conditioning signal from evidence sufficient for model promotion. -->

## 판정

J는 I에서 확인한 “candidate geometry만으로는 desired route를 알 수 없다”는 문제를 causal raw
route로 직접 다뤘다. 결과적으로 Town03 straight FDE와 속도 MAE, combined FDE와 속도 MAE는
고정 gate를 통과했다. 그러나 직진 ADE, 우회전 세 지표, combined ADE가 실패해 metric은
`4/9`에 그쳤다.

따라서 route conditioning 방향을 폐기할 결과는 아니지만, 현재 J를 더 튜닝하거나 모델로
승격할 근거도 아니다.

## 왜 학습량 부족으로만 보지 않는가

- trainable parameter는 4개뿐인데 3개 fold와 final fit에서 총 `946,176` row exposure를 썼다.
- final effective weight는 lateral `6.2943`, heading `6.6575`, speed `2.7771`, curvature
  `6.5408`로 유한하며, final optimization loss는 step 1 `1.6698`에서 step 256 `1.3624`로
  감소했다.
- TRAIN route group별 DRIVE 행은 Town07 `625`, C-track `289`, Town01 `318`이지만 straight
  policy가 실제 적용된 행은 각각 `625`, `0`, `156`이다.
- C-track은 전부 turn fallback이라 LOO fold가 있어도 straight route policy 일반화를 전혀
  검증하지 못한다.

즉 optimizer step을 더 늘리는 것보다, 서로 다른 물리 straight route에서 같은 인과 입력이
같은 candidate 선택을 지지하는지 확인할 데이터가 먼저다.

## 개선과 잔여 실패

| 범위 | 기준 대비 J ADE | 기준 대비 J FDE | 기준 대비 J 속도 MAE |
| --- | ---: | ---: | ---: |
| Town03 우회전 | +0.059495 m | +0.214406 m | +0.004317 m/s |
| Town03 직진 | +0.383269 m | -3.818071 m | -0.312238 m/s |
| combined | +0.000360 m | -2.739309 m | -0.343760 m/s |

직진의 장거리 endpoint와 속도는 좋아졌지만 전체 궤적 ADE가 남았다. 우회전은 turn evidence
전의 straight prefix `129`행에 J가 적용될 수 있어 세 지표가 소폭 퇴행했다. causal latch는
현재 시점 이후의 정확한 fallback을 보장하지만 과거 prefix를 소급 변경하지 않으며, 이 제한을
사후 DEV 결과에 맞춰 바꾸지 않는다.

## 다음 실험의 고정 원칙

1. J의 4-feature 구조, seed, C/D 9-metric 상한을 고정한다.
2. Town03 DEV와 Town04/Town15 TEST는 수집·학습에 사용하지 않는다.
3. TRAIN 역할 Town의 물리적으로 독립된 15 km/h straight route를 추가한다.
4. 동일 route의 속도·날씨 변형은 새 route group으로 세지 않는다.
5. C-track straight 후보의 exact corridor 중복은 QA에서 제거하거나 같은 group으로 묶는다.
6. 기존 baseline과 신규-data arm을 같은 exposure로 비교한다.
7. 성능 `9/9`, geometry `3/3`, preservation `2/2`를 모두 통과해도 stationary·새 seed·신규
   route confirmation 전에는 승격하지 않는다.
8. 30 km/h는 최소 `333.333 m` 유효 route가 준비된 뒤, 60 km/h는 그 다음 별도 안전 계약으로
   진행한다.

NuRec/Cosmos 기반 domain 확장은 이 성능 회복과 분리한다. synthetic-to-real 외관 변화는
route-conditioning 오류를 자동으로 고치지 않으며, 실차용 검증에는 실제 센서 로그와 별도
held-out real-world 평가가 필요하다.

[J 결과](../README.md) ·
[exact report](../metrics/J_causal_route_ranker/report.json)
