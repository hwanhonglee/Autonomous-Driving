# 경로 생성기를 고정한 선택기 실험

<!-- HH_260906 - Separate selector-only learning from generator changes and from runtime deployment. -->

2026-09-08에 진행하는 다음 통제 실험이다. 이전 [C/E 비교](portable-e2e-candidate-ranking.md)는
선택 head를 바꿨지만 공통 특징의 학습 때문에 경로 생성 결과도 달라졌다. 이번에는 기존 C의
완료된 세 checkpoint를 그대로 두고, 각 모델에서 뽑은 입력 특징·후보 경로·속도를 고정한다.

## 비교하는 것

| 구분 | 선택기 시작 상태 | 새 학습 |
|---|---|---|
| C 기준 | 원래 C가 계산한 점수 | 없음 |
| linear_continue | 원래 C의 Linear 가중치 복사 | 선택기만 |
| linear_reset | 같은 Linear 구조를 새로 초기화 | 선택기만 |
| candidate_reset | 후보 XY·속도를 함께 보는 공유 MLP 초기화 | 선택기만 |

세 parent seed 각각에서 세 선택기를 학습하므로 **새 선택기 학습은 9회**다. 원래 C는
다시 학습하지 않으며, 이것을 전체 모델을 처음부터 9회 학습한 것으로 세지 않는다.
이미 학습한 경로 생성기를 사용하므로 이전 scratch C/E와도 학습 예산이 같은 실험이 아니다.

세 방식 모두 batch 4, 1,540 step, 학습률 0.0001, 새 AdamW optimizer를 사용한다.
기존 복합 경로 loss가 고르는 정답 후보에 대한 분류 loss만 학습한다. 데이터는 기존 v3의
train 1,147 / 개발용 val 337이며 독립 Town04 test는 모델 평가·학습·선택에 사용하지 않는다.
기존 dataset 검증기는 test를 포함한 전체 corpus의 파일·hash·형식 무결성을 확인한다.
따라서 test 파일을 전혀 읽지 않는다는 뜻은 아니며, test 예측이나 성능 분석은 하지 않는다. 세 방식의 표본 순서는
같게 고정하되, 과거 전체 모델 학습의 domain-aware 순서를 재현했다고 표시하지 않는다.

## 무엇을 고정하고 확인하나

- 전체 생성기는 `eval()`과 gradient 비활성 상태를 유지한다. 공통 특징도 분리한다.
- model 파라미터·buffer와 캐시 tensor의 dtype·shape·bytes·sample ID를 hash로 연결한다.
- 각 C 모델의 후보 번호 의미가 다르므로 서로 다른 모델의 캐시를 합치지 않는다.
- 정답 궤적·oracle·오차 값은 선택기의 입력에 넣지 않는다. 학습용 label로만 사용한다.
- 같은 캐시에서는 후보 XY·속도와 oracle ADE가 바뀌면 안 된다. 선택한 경로는 달라질 수
  있으므로 기존 v8 형상 검사를 다시 수행한다.
- 처음부터 새 head를 쓰는 효과와 기존 head를 더 학습하는 효과를 따로 표시한다.
  공유 MLP는 깊이·용량·입력도 달라지므로 궤적 입력만의 인과 효과라고 단정하지 않는다.

[고정 계획](../config/portable_e2e_frozen_selector_20260908.json)은 결과를 보기 전에 정했다.
각 방식은 세 seed 모두에서 같은 캐시의 원래 C보다 ADE/FDE가 개선되고, 형상 PASS가
감소하지 않으며 속도 MAE가 5% 넘게 악화하지 않아야 상대 기준을 통과한다. 별도로
1/3/6.4초 ADE 0.5/1/2 m, 6.4초 FDE 4 m 이하의 기존 절대 기준을 유지한다.
좋은 seed 하나만 골라 성공이라고 하지 않으며 val로 epoch를 고르지 않는다.

## 환경과 종료 경계

원격 Pro6000의 기존 개인 py312 venv·GPU0만 사용한다. 로컬에서는 코드 검증과 결과
분석·문서·PNG를 정리한다. 설치·Conda 변경·GPU1·다른 사용자 프로세스에는 손대지 않는다.
사용자가 요청한 작업 경계는 **2026-09-08 오전 10시 KST**이며, 실제 시작·완료·중단은
각 run의 상태 파일로 확인한다. 이 문서의 계획 자체가 실행 완료 증거는 아니다.

이 결과는 별도 **head-only 연구 artifact**다. 기존 trainer나 runtime checkpoint로
위장하지 않으며, 정상 runtime loader는 이 ID를 거부한다. 운용 모델 교체·차량 제어·
실시간 10 Hz 성공을 의미하지 않는다. 경로 PNG도 오프라인 예측 분석이다.
