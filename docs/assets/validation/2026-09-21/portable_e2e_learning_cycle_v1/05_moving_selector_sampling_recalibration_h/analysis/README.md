# H 실패 원인 정밀 분석

<!-- HH_260906 - Diagnose the failed H recovery from immutable caches and logits without opening TEST or changing remote files. -->

## 결론

H가 실패한 주원인은 **학습량 부족이나 우회전 replay identity가 아니다.** F의 `fused` 문맥만
받는 candidate-index linear selector가 Town/수집-domain shortcut에 걸렸고, 실제로 좋은 후보의
XY·속도 geometry는 점수 입력으로 전혀 보지 못한다. H0/H1은 TRAIN을 더 잘 맞혔지만 Town03
DEV 일반화는 더 나빠졌다.

- TRAIN oracle agreement: F `1228/1475 (83.25%)` → H0 `1265 (85.76%)` → H1 `1262 (85.56%)`
- TRAIN CE: `0.5596` → `0.4251` / `0.4554`
- DEV oracle agreement: F `200/665 (30.08%)` → H0 `173 (26.02%)` → H1 `172 (25.86%)`
- DEV CE: `2.9849` → `3.7189` / `3.7276`
- 직진 oracle ADE는 `0.5366 m`인데 H1 selected ADE는 `7.0817 m`이고 selection regret는
  `6.5451 m`다.

즉 후보 생성기는 쓸 수 있는 궤적을 남겼고, 선택기가 높은 확신으로 잘못된 후보를 고른다.
지금 우회전 데이터를 더 넣으면 이미 통과한 방향을 더 강화하면서 직진 실패를 다시 가릴 가능성이
크다. **기존 G 우회전-only 수집은 즉시 다음 단계로 두면 안 된다.**

정확한 파생 수치와 입력 SHA는
[h_failure_root_cause.json](h_failure_root_cause.json)에 고정했다.

## 후보 2 대 후보 5

아래 후보 번호는 0부터 시작한다. Town03 직진 DEV 328개 중 composite oracle은 후보 2가
`312개`, 후보 0이 `16개`다.

| 항목 | F | H0 | H1 |
| --- | ---: | ---: | ---: |
| 후보 2 선택 | 65 | 53 | 53 |
| 후보 5 선택 | 258 | 229 | 226 |
| oracle 2에서 후보 2 선택 | 50/312 | 39/312 | 40/312 |
| oracle 2에서 후보 5 오선택 | 258/312 | 229/312 | 226/312 |
| logit(2−5) 평균 | -2.757 | -2.554 | -2.478 |
| selected ADE (m) | 7.210 | 7.117 | 7.082 |
| selected composite regret | 14.879 | 14.296 | 14.204 |

H1이 후보 5를 고른 226개에서 후보 5 softmax 확률은 평균 `0.901`, 중앙값 `0.981`이다.
그중 176개는 확률 0.9 이상, 78개는 0.99 이상이다. top-1 대 top-2 logit margin도 평균
`4.107`이므로 단순한 동률이나 작은 calibration 오차가 아니다.

그런데 frozen candidate 자체를 보면 정답 정보가 사라진 것이 아니다.

| Town03 직진 15 km/h | 후보 2 | 후보 5 |
| --- | ---: | ---: |
| 후보 평균 속도 (m/s) | 3.920 | 7.365 |
| ADE (m) | 0.558 | 8.547 |
| composite cost | 1.226 | 19.089 |

expert target 평균 속도는 `4.117 m/s`다. 후보 2는 속도와 경로를 잘 보존하지만, 후보 5는 평균
`7.365 m/s`로 과속하며 ADE가 약 `8.0 m` 더 크다. 현재 linear selector는 이 후보별
geometry를 입력으로 받지 않아 이 명백한 차이를 이용할 수 없다.

## 같은 15 km/h 직진인데 Town이 바뀌면 뒤집힌다

TRAIN의 Town07 15 km/h 직진은 328개 모두 oracle 2이고 F/H0/H1 모두 328개 전부 후보 2를
선택한다. H1의 `logit(2−5)` 평균은 `+12.281`이다. 반면 Town03 15 km/h 직진에서는 평균이
`−2.478`이다. 속도와 주행 유형이 같은데 Town만 달라졌을 때 **14.759 logit**이 뒤집힌다.

fused 256차원을 TRAIN 통계로 표준화해 Town03 직진 각 행의 최근접 TRAIN을 찾으면 다음과 같다.

| 최근접 TRAIN episode | 행 수 |
| --- | ---: |
| Town01 30 km/h 우회전 | 172 |
| Town07 30 km/h 직진 | 155 |
| C-track 30 km/h 좌회전 | 1 |
| Town07 15 km/h 직진 | **0** |

최근접 TRAIN oracle이 Town03 oracle과 맞는 비율은 `6.10%`뿐이고 328개 중 259개의 최근접
oracle이 후보 5다. cosine 기준도 Town07 15 km/h 최근접은 0개이고 일치율은 `6.40%`다.

반대로 후보 geometry를 기존 residual ABI와 같이 `XY/120 m + speed/55 m/s`로 비교하면,
Town03 후보 2 centroid와 가장 가까운 동일-index TRAIN centroid는 Town07 15 km/h 후보 2다
(`0.098`). Town07 30 km/h 후보 2까지 거리는 `0.731`이다. 문맥 representation은 map/domain
쪽으로 붙었지만, candidate geometry에는 더 일반적인 속도·형상 단서가 남아 있다는 증거다.

## replay sampling이 주원인이 아닌 이유

H0의 추가 3,620회 노출은 Town01 우회전에 전부 들어갔다. H1은 Town07 30 직진 927,
C-track 좌회전 912, Town01 우회전 1,125, Town07 15 직진 656회로 재분배했다. 그런데 H0와
H1의 최종 argmax 차이는 TRAIN `5/1475`, DEV `8/665`, 직진 DEV `4/328`뿐이다.

또한 H1은 F에서 충분히 멀리 이동했다. F 대비 평균 절대 logit 변화는 TRAIN `2.960`, DEV
`2.690`이고 행별 L2 변화는 각각 `12.786`, `11.732`다. 반면 H1 대 H0 평균 절대 logit
차이는 TRAIN `0.248`, DEV `0.201`이다. 따라서 “head 업데이트가 너무 작았다”나 “우회전
replay identity만 바꾸면 해결된다”는 설명은 데이터와 맞지 않는다.

## 다음 최소 실험

다음은 새 자율주행 기능 head가 아니라 **성능 회복용 selector 교체 A/B**여야 한다.

1. F image/route/ego encoder, fusion, DRIVE/STOP 생성기, candidate XY/speed와 원래 logits를
   모두 고정한다.
2. 두 arm 모두 같은 크기의 shared residual scorer를 쓰고 마지막 층을 0으로 초기화해 시작
   logits가 F와 byte-equal인지 확인한다.
3. 대조군은 `fused + original logit + zero geometry`, 실험군은
   `fused + original logit + candidate XY/speed geometry`를 입력으로 쓴다.
4. DRIVE `[0..5]`, STOP `[6..11]` 각각에서 최종 group maximum이 원래 F group maximum과
   정확히 같도록 정규화한다. 그러면 DRIVE/STOP mode 선택은 바뀌지 않고 mode 내부 순위만
   바뀐다.
5. 원래 F가 선택한 mode를 causal mask로 사용하고, 그 mode 안에서 composite-cost가 가장 낮은
   후보를 CE 및 expected-regret target으로 삼는다. 모든 TRAIN 행은 유지한다.
6. H1 cyclic-uniform 순서·동일 seed·동일 노출량·동일 optimizer를 두 arm에 고정하고 Town03
   DEV는 마지막 step 뒤 한 번만 평가한다.
7. geometry arm이 equal-capacity 대조군을 이기면서 C 우회전과 D 직진/전체의 9개 절대 gate를
   전부 통과해야 단일-seed recovery signal로 인정한다. 그 뒤에만 3 seed와 봉인 stationary
   DEV guard를 진행한다.

과거 다른 cache에서 candidate-aware residual은 세 seed 중 두 개의 VAL ADE를 개선했지만 한
seed는 악화시켰다. 따라서 구조 방향은 타당하지만 성공이 보장된 것은 아니다. 이번 mode-max
보존은 그 실험보다 더 좁게 DRIVE/STOP 동작을 고정하려는 안전장치다.

이 A/B도 실패하거나 TRAIN만 맞히면, 다음 데이터 수집은 우회전이 아니라 **서로 다른 TRAIN
Town·route의 15/30 km/h 직진**이어야 한다. 현재 직진 TRAIN은 물리적으로 Town07 한 route의
두 속도뿐이라 map-invariant representation을 배울 근거가 부족하다.

## 증거 범위

분석 입력은 [exact H artifact](../../../../../../../storage/experiments/training/2026-09-21/moving_selector_sampling_recalibration_h_v1/summary.json)의
고정 TRAIN/DEV cache, F/H0/H1 logits, 두 history와 pinned loss source다. 입력별 SHA-256과 원격
episode/config SHA-256은 JSON에 기록했다. 원격 파일은 읽기만 했고 수정하지 않았다.

이 분석에서는 TEST 파일을 읽거나 추론하지 않았고, CARLA·Autoware·RViz를 실행하지 않았다.
closed-loop, runtime checkpoint export, 모델 승격, 차량 제어, 실차 검증도 없다. Town03은 이미
H에서 사용한 DEV이며 unseen TEST가 아니다. kNN/centroid는 representation 원인을 설명하는
진단일 뿐 배포 가능한 predictor가 아니다. 봉인 stationary DEV도 여전히 없으므로 어떤 결과도
현재 상태에서 승격할 수 없다.
