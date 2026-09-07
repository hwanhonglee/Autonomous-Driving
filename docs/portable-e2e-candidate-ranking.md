# 후보 경로를 보고 선택하는 모델 — 3-seed 비교 완료

<!-- HH_260906 - Freeze a research-only architecture comparison without changing the physical decoder or deployment model. -->

2026-09-07의 학습률·데이터 확장·선택 손실 가중치 실험 뒤에 이어지는 연구다.
[이전 12회 결과](validation-2026-09-07-portable-learning.md)에서 일부 seed의 오차는
줄었지만 모든 seed에 걸친 상대 개선과 절대 경로 품질 목표를 함께 통과하지 못했다.
기존 Autoware 제어와 Portable shadow checkpoint는 유지한다. 새 E 모델도 실제 학습·평가를
완료했지만 **상대 선별 FAIL / 절대 품질 FAIL**이다.

## 실제 결과 — 2026-09-07

Pro6000 개인 venv의 GPU0에서 23:23:04–23:34:14 KST에 세 seed의 학습·평가·v8 검사
9개 단계를 완료했다. source는 `5c50353480a51f11b970c7d0b9ec8dcd91f7b056`이다.
이번 추가 실험에서 시스템·venv 패키지를 설치하지 않았고 GPU1을 사용하지 않았다.

| Seed | C → E 평균 경로 오차 ADE m | C → E 마지막 위치 오차 FDE m | 상대 선별 | E 절대 품질 |
|---|---:|---:|---|---|
| 20260903 | 3.8743 → 4.7357 | 9.8277 → 9.4130 | FAIL | FAIL |
| 20260904 | 6.2585 → 4.5391 | 12.8975 → 9.7093 | PASS | FAIL |
| 20260905 | 4.7895 → 4.2284 | 11.8663 → 9.3487 | PASS | FAIL |

속도 오차와 FDE는 세 seed 모두 줄었고 형상 PASS는 모두 336/337로 유지됐다. 하지만
첫 seed의 ADE가 악화해 전체 상대 기준을 통과하지 못했다. E의 oracle ADE도 C보다
세 seed 모두 높아졌다. 후보 선택 종류가 늘어난 것만으로 성공이라고 할 수 없다.

- [학습 곡선·3-seed 수치·고정 기준 판정](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/11_candidate_rank_ab/README.md)
- [차량 중심 경로 PNG 18장과 전체 val337 분석](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/12_candidate_rank_route_analysis/README.md)
- [전체 코드 테스트: 2,333 passed / 6 skipped](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/13_candidate_rank_code_validation/README.md)

PNG는 학습 모델의 **오프라인 예측 경로**이며 Autoware 실시간 주행 화면이 아니다.
이번에는 새 CARLA 촬영·learned closed-loop 주행·E 모델 10 Hz 계측을 하지 않았다.

## 무엇을 바꾸나

기존 physical-v1은 카메라·차량 이력·route를 합친 공통 특징에서 여섯 후보의 점수를
직접 계산한다. 새 `candidate_rank.v1`은 **각 후보가 실제로 예측한 XY·속도**도 함께 보고
후보마다 같은 작은 MLP로 점수를 계산한다. 특정 후보 번호에만 붙는 embedding이나
별도 점수 bias는 사용하지 않는다.

- 기존 여섯 카메라 입력, 시간·좌표·64-point 출력 계약은 유지한다.
- 경로 생성 decoder와 30 km/h 상한·가속도·곡률·횡가속도 상수는 그대로 둔다.
- XY는 route scale, 속도는 기존 물리 속도 상한으로 정규화한다.
- 점수 계산에 넣는 후보 궤적은 `detach`한다. 점수 loss가 궤적 tensor로 직접 역전파되지
  않지만, 공통 특징을 통한 간접적인 경로 생성 변화는 여전히 가능하다.
- 같은 seed에서 기존 encoder·경로 생성기의 초기 가중치를 유지하도록 기존 초기화 뒤에
  선택 head만 교체한다. 학습 후의 가중치가 계속 같다는 뜻은 아니다.
- 새 구조는 모델 parameter 수와 연산량이 늘어난다. 동일 step 비교이지 동일 FLOP·동일
  wall-clock 예산 비교는 아니다.

E의 parameter 수는 1,068,249개, C는 954,590개다. 경로 입력뿐 아니라 선택 head의 깊이와
용량도 바뀌므로, 결과가 좋아져도 경로 정보를 추가한 것만의 인과 효과라고 단정하지 않는다.
새 선택 구조 전체에 대한 비교로 해석한다. 표본 순서는 별도 seed/epoch generator를
사용하므로 head 초기화가 소비하는 난수량과 분리돼 있다.

새 model ID와 config로 별도 학습하며 이전 checkpoint를 덮어쓰거나 다른 구조에
`resume`하지 않는다. 현재 runtime bundle의 허용 목록에는 새 ID를 추가하지 않는다.
따라서 이 연구용 checkpoint를 기존 live shadow에 바로 끼워 넣는 단계도 아니다.

## 먼저 확인하는 원인

`ADE oracle`은 expert 미래 위치를 알고 위치 오차가 가장 작은 후보를 고른 것이다.
반면 학습의 `composite-loss oracle`은 XY·speed·yaw·kinematic consistency·최종점
오차를 함께 고려한다. 두 oracle을 같은 것으로 간주하면 원인을 잘못 판단할 수 있다.

기존 C와 D의 세 seed checkpoint 각각에서 다음을 **train 1,147 / val 337을 따로** 계산한다.

1. 모델 선택과 composite-loss oracle의 일치율
2. 모델 선택과 ADE oracle의 일치율
3. 두 oracle 사이의 일치율과 후보별 혼동 행렬
4. 선택 경로와 각 oracle 경로의 ADE 차이

원래 `trajectory_loss` 구현과 checkpoint의 loss 설정, XY·speed·yaw·valid mask를 그대로
사용한다. train 결과는 학습 자료 내부 진단이며 일반화 성능으로 사용하지 않는다.
이 진단에서는 GPU를 숨기고 개인 venv의 CPU 4 threads만 사용한다.

이 원인 진단도 C/D 6개 모델에서 완료했다.
[전체 집계와 train/val 비교 그림](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/10_objective_alignment/README.md)에 따르면,
composite oracle과 ADE oracle의 ADE 차이는 val에서 0.033–0.103 m지만 실제 선택과
composite oracle 사이 차이는 2.036–4.742 m다. 목표 정의 차이만으로 큰 선택 오차를
설명하기 어렵다. D는 train의 composite oracle 일치율을 세 seed 모두 높였지만 val
개선은 일관되지 않았다. 이는 선택기의 학습·일반화 문제를 더 조사할 근거이지 단일 원인을
확정한 결과는 아니다. D의 두 번째 seed에서 CPU/GPU 선택 빈도 차이도 관측돼, CPU
진단 값을 GPU 성능 판정에 대신 사용하지 않았다. 그 차이의 원인은 아직 확정하지 않았다.

## 고정 비교 조건

| 항목 | 기준 C | 새 E |
|---|---|---|
| 모델 | physical-v1 | candidate_rank-v1 |
| train / val | 1,147 / 337 | 동일 |
| seed | 20260903, 20260904, 20260905 | 동일 |
| 학습률 / batch / step | 0.0001 / 4 / 1,540 | 동일 |
| 후보 선택 loss weight | 0.1 | 0.1 |
| 나머지 loss·물리 decoder·검사 | 기존 설정·v8 | 변경 없음 |
| 기존 checkpoint에서 이어 학습 | 아니오 | 아니오 |

Town04 test 309개는 모델 선택에 사용하지 않는다. v3에서 1,540 step은 약 5.37 epoch이며
이전 train613의 10 epoch와 혼동하지 않는다. 각 모델을 학습한 뒤 전체 val 평가와 v8
경로 형상 검사를 수행한다. 공개 집계는 stage·checkpoint·source·dataset hash와 분모가
일치할 때만 생성한다.

상대 기준은 **세 seed 모두**에서 ADE와 FDE가 줄고, 선택 경로의 geometry PASS 수가
감소하지 않으며, speed MAE가 5% 넘게 악화하지 않는 것이다. 절대 기준은 1/3/6.4초
ADE 0.5/1/2 m 이하와 6.4초 FDE 4 m 이하다. 한 seed의 최선 결과로 전체 성공을
선언하거나 결과를 본 뒤 기준을 완화하지 않는다.

## 두 환경의 역할과 다음 단계

Pro6000에서는 개인 venv·GPU0만 사용해 이 비교를 실행한다. 시스템·Conda·GPU1·다른
사용자의 프로세스를 변경하지 않는다. 원격에서는 CARLA나 Autoware를 빌드하지 않는다.
로컬에서는 코드 검증, 원격 결과 수집·경로 그림·보고서 정리를 수행한다.

이 변경이 합격해야 별도의 export parity·runtime 지원·로컬 10 Hz shadow 회귀를 검토한다.
이번 E는 합격하지 못했으므로 그 단계로 진행하지 않았다. 다음 반복은 선택기만 별도로
검증하는 통제 실험과 독립적인 회전·정지 episode 확장을 우선 검토한다. 같은 val을
반복해서 본 결과는 개발용 수치이며 미사용 test의 일반화 성능으로 제시하지 않는다.
형상 검사 통과만으로 충돌 회피나 도로 안전이 입증되지는 않는다. 기능별 label·시나리오와
안전 selector·fallback을 갖춘 뒤 learned CARLA closed-loop로 진행한다.
[9개 상위·30개 하위 기능](portable-e2e-feature-roadmap.md)은 그 이후에도 각 기능의 독립
데이터·평가를 차례로 추가해야 하며 이번 선택 구조 변경으로 모두 구현되는 것은 아니다.
