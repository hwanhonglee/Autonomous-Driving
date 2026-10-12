<!-- HH_261002 - Propose causal terminal decoding and genuine stop supervision; no source, weight, data or approval mutation. -->

# 목적지 접근·정지를 학습하기 전에 고쳐야 할 구조

현재 상태: **새 종점 정보 계약과 수치 디코더 코드는 준비했지만, 모델·학습·실행기에 아직 연결하지 않았습니다.** 아래는 원래 설계 제안이며 실제 정답 기반 표현 검산을 진행하는 단계입니다. 공식 승인 수는 메인 **0/9**, 서브 **0/30**입니다. 기존 모델·동결 디코더·학습 자료·실패 기록은 변경하지 않습니다.

<!-- HH_261002 - Distinguish prepared numerical code and actual teacher improvements from failed admission and learned goal stopping. -->
후속 교사 v6는 실제 정차 제동·속도 0·2초 유지·전체 가감속을 확인했으나, 요청 조향과 실제 억제 조향의 차이 및 저속 미래 궤적 품질 미달로 학습하지 않습니다. [현재 작업과 모든 결과](assets/validation/2026-10-02/README.md), [준비한 종점 정보 계약](../learning/portable_e2e/terminal_witness_v1_20261002.py), [준비한 수치 디코더](../learning/portable_e2e/path_time_decoder_terminal_v1_20261002.py)를 구분해 읽어주세요. 코드 검사 통과는 실제 모델 정차 성능이 아닙니다.

## 쉽게 말하면

현재 모델에는 이미 속도, 주행/제동/정지 의도, 정지 거리, 가속도를 출력하는 부분이 있습니다. 새 기능용 헤드가 없어서 목적지 정지를 못 하는 상황은 아닙니다.

문제는 이 출력들을 연결하는 방법입니다. 현재 실행기는 의도 출력을 읽지 않고 항상 GO로 주행시킵니다. 목적지까지 18m도 남지 않으면 GO 디코더가 거절합니다. BRAKE를 사용하더라도 속도·가속도 출력을 읽지 않고 일정한 감속만 만듭니다. 이런 연결을 그대로 두고 학습 횟수만 늘리면, 연결되지 않은 출력이 실제 정지 궤적을 개선할 수 없습니다.

또한 정상적인 정답이 필요합니다. 실제 교사 주행은 브레이크 상한 .10과 .01 조건 모두 급감속으로 실패했습니다. 실패 자료를 좋은 학습 자료로 바꾸거나, 실패 구간을 잘라서 학습시키지 않습니다. **먼저 차량의 실제 제동·코스팅 반응을 확인하고 정상 정지 자료를 확보해야 합니다.**

## 코드에서 확인한 문제

| 현재 연결 | 확인된 내용 | 학습을 더 해도 해결되지 않는 이유 |
| --- | --- | --- |
| GO 경로 지원 | 최대 속도 2.5m/s × 미래 6.4초 + 제동 여유를 고정 계산하여 18m 요구 | 실제 목적지 접근은 남은 경로가 18m보다 짧을 수 있음 |
| 현재 학습·실행 의도 | `joint_forward`와 runtime이 GO, stop distance 0을 직접 설정 | 의도·정지 거리 예측은 궤적에 사용되지 않음 |
| 의미 정답 | temporal dataset의 `intent_valid`, `stop_distance_valid`는 항상 false | 현재 자료만으로 정지 의도를 학습했다고 할 수 없음 |
| BRAKE 시간 궤적 | `v0²/(2×stop_distance)` 일정 감속 | 기존 speed/a0 출력에 제동 궤적 손실의 유효한 연결이 없음 |
| 목적지 정보 | Common10 원본에는 `goal_base_m`이 있으나 `TrainingExample`과 `RuntimeInputs`에는 없음 | 경로 묶음의 마지막 점이 목적지인지, 잘린 중간 점인지 명시적으로 구분하지 못함 |

근거 파일: [디코더](../learning/portable_e2e/path_time_decoder_v2.py), [공동 학습 연결](../tools/e2e/train_temporal_v2_low_speed_joint_20261001.py), [현재 런타임](../learning/portable_e2e/temporal_simulation_research_v2_20261001.py), [temporal dataset](../learning/portable_e2e/temporal_dataset_v2.py), [손실](../learning/portable_e2e/temporal_losses_v2.py), [기존 모델 출력](../learning/portable_e2e/temporal_model_v2.py), [Common10 변환](../tools/e2e/prepare_carla_common10_dataset.py), [framework-neutral example](../learning/portable_e2e/dataset.py), [runtime input](../learning/portable_e2e/runtime.py).

v1 실제 검사에서 GO 897개 중 **65개는 남은 경로가 18m 미만**이었습니다. 이 숫자는 급감속에 연관된 실패 앵커 75개와 별개의 검사 결과입니다. [전체 실제 미래 검사 설명](assets/validation/2026-10-02/town07_natural_goal_stop_teacher_v1/FUTURE64_FINDINGS.md)을 참고합니다. v1/v2 교사 실패는 그대로 보존하며 어느 쪽도 TRAIN으로 승인하지 않습니다.

## 최소 변경 제안: 새 버전의 terminal-aware 디코더

### 1. 실제 목적지임을 확인하는 입력 계약

새 `TerminalWitness`에 다음을 연결합니다. 이것은 교사의 미래 정답이 아니라 **이미 주어진 미션 경로와 현재 측정 위치**에서 얻는 정보입니다.

- 미션 경로 ID·전체 경로 SHA, 선언된 목적지와 정지 기준의 좌표·기준 방식.
- 현재 ego/camera/navigation 시각과 현재 측정 자세.
- 현재 경로 투영 위치, 실제 남은 미션 경로 길이, 현재 base 좌표계의 목적지.
- 제공한 경로의 마지막 점이 미션 목적지에 도달하는지에 대한 검증 결과.
- 현재 제공된 실제 경로의 유효 지원 거리와 종점 접근 모드 여부.

현재 canonical route는 최대 **120m**를 잘라 제공합니다. 따라서 단순히 `route_mask`가 짧다는 이유만으로 마지막 점을 목적지로 취급하면 안 됩니다. 알려진 전체 미션 경로·목적지·현재 투영 위치·원본 시각과 일치해야 합니다. 미션을 모르는 짧은 경로는 기존처럼 거절합니다. 경로를 늘리거나 실제 미래 위치로 부족한 지원을 메우지 않습니다. 경로 지원은 장애물 없는 공간이나 차체 안전을 증명하지도 않습니다.

첫 버전은 기존 101개 state tensor와 아홉 종류의 neural input을 유지할 수 있습니다. 모델은 실제 짧은 경로와 mask를 이미 받습니다. 별도 witness는 그 경로가 진짜 목적지까지 제공된 것인지 검증하는 디코더 입력입니다. 새로운 learned head는 추가하지 않습니다. 이 계약은 새 decoder/runtime/export 버전으로 명시하며 기존 loader가 자동으로 받아들이게 하지 않습니다.

### 2. 기존 일반 주행은 정확히 보존

제안 API는 기존 인자를 유지하고 `terminal_witness`와 `initial_acceleration_prediction`을 명시적으로 추가합니다.

`terminal_witness=None` 또는 검증된 일반 주행 모드에서는 **동결된 기존 디코더에 그대로 전달**합니다. 기존 GO 18m 조건과 기존 수치 계산·출력은 그대로 유지합니다. 실제 stop 정보가 없는 legacy replay는 이 일반 주행 경로를 사용합니다.

검증된 목적지 접근 모드만 별도 경로를 사용합니다. 현재 교사 의도가 GO인 경우도 접근 모드가 가능하지만, **GO 정답을 BRAKE로 바꾸지 않습니다.** 현재는 주행 중이면서 미래 6.4초 안에 제동·정지가 시작되는 정상 상황을 표현하는 것입니다.

### 3. 실제 속도 출력으로 변속 감속을 표현

종점 모드는 기존 33개 공간 speed knot와 curvature knot를 사용합니다. 학습한 경로에 대해 다음을 새 수치 계약으로 정의합니다.

- 실제 `v0=hypot(vx,vy)`를 t=0에 그대로 둡니다. 0·작은 양수·상한 초과를 0 또는 상한으로 덮지 않습니다.
- 실제로 제공된 경로 안에 양의 길이를 가진 정지 끝점을 둡니다. 알려진 미션 종점 밖으로 진행할 수 없습니다.
- speed knot의 요청과 곡률 제약, 정지 끝점의 속도 0을 함께 사용하여 공간상의 속도 곡선을 만듭니다. 기존처럼 뒤에서 제동 가능성을 확인하고 앞에서 가속 가능성을 확인합니다.
- BRAKE도 이 가변 속도 곡선으로 시간 궤적을 만듭니다. 더 이상 정지 거리 하나만으로 일정 감속을 고정하지 않습니다.
- 기존 acceleration head는 첫 실제 100ms 구간의 속도 변화에 연결합니다. 첫 구간 경계점을 삽입하는 방법은 별도 수치 테스트를 통과해야 합니다. 지원 거리·속도·가속도에 맞지 않는 초기 경계는 조용히 숨기지 않고 거절하거나, 사전 선언된 인과적 feasible parameterization으로 표현합니다.
- HOLD는 실제 현재 속도가 .05m/s 이하일 때만 허용합니다. 작은 잔여 속도의 제동·이동 거리도 계산합니다. 측정 속도를 0으로 만들지 않습니다. 실제 0일 때만 이후 위치·속도가 일정합니다.
- 실제 정지 뒤 64개 XY·body yaw·속도 정답은 보존합니다. 이동 방향 곡률이 정의되지 않는 항목만 `null/valid=false`로 구분하며 곡률 0 정답으로 바꾸지 않습니다.

첫 제안의 내부 가속/감속 envelope는 기존 **+1.5/−2.0m/s²**를 유지합니다. 원래 실제 자료·전체 상태·미래 검사의 **±2.9m/s²** 한계를 넓히지 않습니다. 제한이 더 보수적인 디코더로 실제 정상 정답도 표현하지 못한다면 oracle 평가에 그대로 실패로 기록하고, 수치 모델 변경 여부를 별도 검토합니다. 곡률·횡가속·속도·차체·정지 위치·시간 게이트도 별도 유지합니다.

### 4. 정지 거리의 뜻을 혼동하지 않기

다음 세 거리는 같은 것이 아닙니다.

| 거리 | 용도 |
| --- | --- |
| 실제 남은 미션 경로 길이 | 현재의 인과적 경로 지원·목적지 경계 |
| 미리 선언한 `goal−.75m` 정지 기준까지 거리 | 교사 목표와 미션 정지 기준 설명 |
| 현재부터 실제 첫 정지/HOLD까지 이동한 미래 arc | 정지 거리 출력의 관측 정답 후보; loss/평가 전용 |

교사가 `.75m` 기준을 지나 아직 움직이고 있으면 기준까지 거리는 음수가 될 수 있습니다. 이를 0으로 잘라 움직이는 BRAKE 정답에 집어넣으면 기존 BRAKE 계약과도 맞지 않습니다. 관측 정지 arc와 기준 오차를 각각 남겨 oracle 적합성을 검사해야 합니다. 실제 미래 arc를 decoder의 **경로 지원 입력**으로 사용하는 것은 금지합니다.

학습한 정지 거리는 실제 현재 속도의 최소 제동 거리 이상이며 현재의 검증된 경로 지원 이하가 되도록 새 변환을 검토할 수 있습니다. 예를 들어 현재 속도로 계산한 최소 거리와 실제 미션 지원 사이를 scalar stop head가 선택하게 합니다. 이는 현재 상태·미션만 사용하는 물리 표현이며 미래 정답을 runtime에 제공하는 것이 아닙니다. 변환·경계·손실 정의는 구현 전에 고정합니다. observed label이 이 범위에 없을 때 정답을 잘라 맞추지 않습니다.

## 어떤 정답으로 학습하는가

정상 정지 수집과 독립적인 full-body/ACK/pixel/cadence/미래 QA가 먼저 필요합니다. 목표를 통과한 교사라도 급감속하면 학습 자료가 아닙니다.

정답은 다음과 같이 만들며 모든 필드의 원본 frame/time/source SHA를 남깁니다.

| 정답 | 실제 근거 | 손실 |
| --- | --- | --- |
| 현재 GO/BRAKE_TO_STOP/HOLD | 같은 프레임의 실제 governor phase와 측정 상태 | 기존 intent cross-entropy, 검증된 valid만 사용 |
| 정지 이동 거리 | 실제 정지/HOLD 관측과 현재 위치; 선언 기준 오차 별도 | 새 의미가 명확한 stop-distance loss |
| 첫 구간 가속도 | `(실제 v(t+.1)−실제 v0)/.1` | 기존 a0 head supervision; API 가속도 벡터 norm과 혼동하지 않음 |
| 공간 속도·곡률 | 관측된 실제 미래 arc의 유효 구간만 | 기존 raw speed/curvature loss와 개별 valid mask |
| 미래 64 XY·순간 속도 | 실제 시각 보간의 원래 정답 | 기존 near/mid/far 시간 손실; chord speed와 순간 속도 분리 |

HOLD의 실제 속도 0은 유효한 속도 정답입니다. 곡률이 무효라고 속도·XY 정답까지 지우지 않습니다. 정지 후 body yaw는 원래 값으로 보존하고, path heading과 동일하게 표현할 수 있는지도 oracle에서 따로 확인합니다.

현재 기존 손실은 raw curvature .25, raw speed 1, intent .2, stop distance .1, a0 .1, decoded XY 1, decoded instantaneous speed 1이며 시간 구간은 0–.5s/.5–2s/2–6.4s에 .4/.3/.3을 사용합니다. 새 첫 버전은 이 가중치를 출발점으로 삼되, 실제로 사용하는 stop-distance 변환과 a0 경계의 뜻을 새 loss/source metadata에 명시합니다. 현재 cruise 전용 far-speed continuation prior를 정지 구간에 무조건 적용하지 않습니다.

학습 시 semantic routing에 검증된 현재 의도 정답을 사용하는 경우를 명시하고, **모델이 예측한 의도로 독립적으로 디코딩한 결과도 전량 평가**합니다. 정답 의도로만 좋은 결과를 낸 것을 실제 모델 정지 성공이라고 보고하지 않습니다. 기존 visual encoder·일반 주행 replay를 보존하고, 이미 존재하는 semantic/stop/a0 파라미터와 필요한 speed/fusion 파라미터의 학습 범위를 사전 선언합니다. 세부 파라미터 이름·개수·학습률·업데이트 수는 실제 승인 자료와 oracle 결과를 보고 별도로 고정합니다.

## 다음 순서와 통과 조건

1. 제동·zero-throttle·manual/automatic gear 등 실제 차량 반응의 원인을 분리합니다. 더 작은 브레이크를 계속 추측해서 실패 자료를 쌓지 않습니다.
2. 정상 교사 주행을 새 조건으로 사전 선언하여 수집합니다. 종점 도달, 실제 `.05m/s` 이하 2초 HOLD, 130 native tail, 전체 ±2.9m/s²·차체·ACK·카메라 검사를 통과해야 합니다.
3. 새 stop-aware QA/변환을 만듭니다. 실제 GO/BRAKE/HOLD 전량, 모든 event-defined 앵커, true64, 원본 실패 여부를 보존합니다. TRAIN/HOLDOUT은 별도 실제 주행으로 정하고 같은 주행의 겹친 프레임을 독립 검증으로 세지 않습니다.
4. **가중치 학습 전에 decoder oracle 평가**를 합니다. 실제 정답으로부터 decoder가 표현할 수 있는 공간 속도·곡률·정지 거리·a0를 구성하되, 실제 미션 경로 지원과 현재 v0는 그대로 유지합니다. 이는 수치 표현의 진단이며 runtime나 학습 입력에 미래 정답을 주는 것이 아닙니다.
5. oracle의 GO 짧은 지원, BRAKE, HOLD별 거절 수·ADE/FDE·순간 speed MAE·첫 구간 a0·정지 arc/시간/위치·yaw residual을 전체 고정 분모로 보고합니다. 이 단계의 오차가 큰데 모델만 오래 학습하지 않습니다. 절대 기준과 기존 주행 보존 기준은 oracle 실행 전에 root 검토로 고정합니다.
6. 정상 자료와 표현이 준비되면 새 혼합 학습을 합니다. 새 stop episode의 의도별 표본과 전체 앵커, 기존 signed recovery/center/true-turn replay를 함께 사용하며 holdout은 optimizer에 넣지 않습니다. 중간 checkpoint 선택 없이 사전 고정 endpoint로 평가합니다.
7. 고정 최종 모델에 대해 동일 조건의 실제 learned goal-stop 시험을 진행합니다. 일반 주행 90m/45s, 회전, 속도·body·시간 회귀와 목적지 정지를 각각 검사합니다. 교사 성공·oracle 성공·offline fit·실제 learned 성공은 별개 결과로 기록합니다.

### 구현 전에 검증할 불변식

- nonterminal dispatch는 동결 V2와 동일 수치·거절·출력이며 기존 source/weights가 바뀌지 않음.
- 같은 causal image/ego/mission 입력에서 미래 정답만 바꾸어도 forward/decoder 결과가 바뀌지 않음.
- 짧은 미션 지원을 허용하는 것은 검증된 종점 모드뿐임. unknown/truncated 경로는 여전히 거절.
- 현재 GO 정답을 거절 회피 목적으로 BRAKE/HOLD로 바꾸지 않음.
- t0 실제 v0는 bit-preserved, 실제 0 및 작은 양수의 시간 적분·gradient가 finite.
- 고정 t=.1..6.4 출력, 단조 arc, 비음수 속도, 원래 속도·곡률·가속도·횡가속 bounds, 실제 경로 지원 및 endpoint 경계 유지.
- 잘못된 HOLD 진입, 물리적으로 불가능한 짧은 정지, NaN/Inf, mask/source/time 오류는 fail-closed.
- stationary null 곡률을 PASS 또는 0 정답으로 바꾸지 않으며 모든 actual XY/speed/native rate 검사를 유지.
- 정상·실패·빈 semantic stratum과 전체 TRAIN/HOLDOUT/replay 분모를 숨기지 않음.
- STOP output 변화와 semantic 학습 여부, 새 decoder/export ABI를 명시하고 기존 GO-only loader를 속이지 않음.

## 작업량과 범위

우선 새 인과적 witness/수치 디코더/합성 불변식/oracle 도구를 준비하는 코드 단계가 필요합니다. 정상 교사 데이터 확보 시간은 실제 물리 원인에 따라 달라 지금 보장할 수 없습니다. 실제 GPU 학습은 그 다음 단계이며 **GPU0 + 개인 venv만** 사용합니다. 업데이트·시간·크레딧 예산은 승인 데이터의 전체 분모와 oracle 결과를 확인한 뒤 별도 제안합니다. 지금 다시 6,000회 학습을 예약하거나 기능 성공을 약속하지 않습니다.

이번 제안은 주행의 기본 경로 유지·목적지 접근·정지를 개선하기 위한 연결 수정입니다. 9개 메인 기능을 동시에 완성했다는 뜻도, 장애물 회피·차선 변경·실차 안전을 검증했다는 뜻도 아닙니다.
