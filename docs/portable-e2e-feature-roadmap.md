# Portable E2E 자율주행 기능 요구사항·로드맵

> 기준일: 2026-09-06
>
> 기계 판독 원본: [`config/portable_e2e_feature_matrix.yaml`](../config/portable_e2e_feature_matrix.yaml)

## 현재 결론

목표 기능을 9개 상위 영역, 30개 하위 기능으로 분해했다. 현재 완료된 것은
`common_10hz_v1` 데이터·학습 배선과 첫 CARLA-only open-loop baseline이다. 학습된
checkpoint가 Autoware 또는 CARLA를 폐루프로 주행한 적은 없으며, 30개 중
`CLOSED_LOOP_PASS`인 기능은 **0개**다.

현재 판정의 근거는 다음과 같다.

- Town07 직진 309개와 CTrack 좌회전 304개를 학습하고 Town03 우회전 337개를
  validation한 총 950개 CARLA sample은 데이터 계약을 통과했다.
- 현재 v0 10-epoch Town03 open-loop 6.4초 ADE/FDE는 `6.567681/16.171295 m`, speed
  MAE는 `1.872983 m/s`, yaw MAE는 `0.379922 rad`, kinematic speed MAE는
  `1.697376 m/s`다. 2026-09-05의 1-epoch 값은 역사적 A/B 기준선이며 현재 값도 아래 초기
  품질 gate를 통과하지 못한다.
- `0.702218 ms/sample`은 Pro6000의 model-forward 한 구간일 뿐 전처리, ROS, TF,
  selector, controller를 포함한 10 Hz runtime 판정이 아니다.
- runtime geometry gate v6의 Town03 전체 감사에서 candidate `c0~c5` 각각 geometry
  PASS가 `0/337`이고 최고 logit 선택은 index 1에 `337/337` 고정됐다. 완료된 최신
  full-split 재감사의 selected failure는 geometric-speed·speed-disagreement·step·reported/
  geometric-speed-rate·distance-disagreement·curvature·lateral-acceleration이 각각 `326`,
  heading `310`, backward-step `94`, speed `47`이다.
- source checkpoint
  `370f12dbfa15cc17fa29931bc3c9dd3140dbd7c0a61d976af296fd223b2becf0`에서 내보낸
  non-executable runtime bundle
  `b9b10e1604ac59eb4375b233d80b8f7ea04d983c0b841d6afddbc39e008c292c`은 로컬 CPU strict
  load를 통과했다. live ROS/CARLA나 10 Hz PASS는 아니다.
- physical v1 decoder는 구현·단위검사를 통과했지만 아직 학습·평가·CARLA 실행을 하지 않았다.
- 격리된 Autoware shadow adapter와 10 Hz 목표 sensor profile의 source/unit 배선은 생겼지만,
  ROS graph startup smoke만 실행했다. sensor-fed trajectory와 CARLA shadow는 아직 실행하지
  않았다. 따라서 `P3_RUNTIME` 단계는 미통과다.
- Town03 expert가 신호를 기다린 기록은 있으나 신호 상태가 현재 모델 입력이나 label은
  아니다. 따라서 교통신호 기능은 `EXPERT_BEHAVIOR_ONLY`다.
- 과거 Autoware VAD 화면은 별도 stack의 역사적 증거다. 이번 portable E2E checkpoint의
  runtime 또는 closed-loop 증거로 사용하지 않는다.

자세한 현재 수치는
[2026-09-05 Common10 학습·검증 보고서](validation-2026-09-05-portable-e2e-common10-30kph.md),
[2026-09-06 duration A/B evidence](assets/validation/2026-09-06/portable_e2e_v0_duration_ab_v1/README.md),
운용 경계는 [학습·운용 가이드](portable-e2e-training.md), 모델 ABI는
[Model v0 설계](portable-e2e-model-v0.md), 로컬 runtime 시작 절차는
[10 Hz shadow runtime 가이드](portable-e2e-shadow-runtime.md)에 있다.

## 상태를 읽는 방법

| 상태 | 의미 |
|---|---|
| `FOUNDATION_PASS_CARLA_ONLY` | CARLA 데이터 계약·배선만 통과. 자율주행 기능 PASS가 아님 |
| `DATA_PARTIAL` | 자격 expert episode 일부만 있음. 독립 train/val/test가 부족함 |
| `EXPERT_BEHAVIOR_ONLY` | expert는 동작했지만 모델 입력·label·평가가 없음 |
| `OPEN_LOOP_MEASURED_BELOW_TARGET` | open-loop 수치는 있으나 test/품질 gate가 미달 |
| `MODEL_FORWARD_ONLY` | 모델 forward 배선·시간만 있음. runtime/폐루프가 없음 |
| `NOT_STARTED` | 기능별 자격 데이터와 평가 증거가 없음 |
| `BLOCKED` | 명시한 선행조건이 없어 fail-closed 상태 |
| `CLOSED_LOOP_PASS` | 학습 모델이 원인인 폐루프 결과가 사전 기준을 모두 통과했을 때만 사용 |

`PASS`는 한 단계의 판정이지 실차 전체 승인과 동의어가 아니다. 특히 expert controller,
recorded-pose replay, historical VAD 실행으로 portable checkpoint의 폐루프 status를 올릴 수
없다.

## 단계와 승격 조건

| 단계 | 종료 조건 |
|---|---|
| `P0_CONTRACT` | 입력·출력·label·provenance·metric과 fail-closed validator 고정 |
| `P1_DATA` | 기능별 독립 train/val/test가 Common10과 수동 provenance 검토 통과 |
| `P2_OPEN_LOOP` | 고정 test, 3개 이상 seed, 절대 기준과 동일 조건 model A/B 통과 |
| `P3_RUNTIME` | Autoware adapter, export parity, reject/fallback, target PC 10 Hz 지연 통과 |
| `P4_CARLA_CLOSED_LOOP` | 학습 출력이 실제 ego를 제어하는 사전 정의 CARLA matrix 통과 |
| `P5_REAL_SHADOW` | actuator를 끈 실측 replay/shadow와 intervention mining 통과 |
| `P6_CLOSED_COURSE` | 보정된 실차가 폐쇄 시험장 계획 통과. 공도 승인은 별도 |

모든 하위 기능은 현재 단계와 목표 단계를 함께 가진다. 앞 단계가 미완료면 뒤 단계가 좋아
보여도 승격하지 않는다. 예를 들어 빠른 GPU forward는 `P3_RUNTIME`을, open-loop ADE는
`P4_CARLA_CLOSED_LOOP`를 대신하지 않는다.

## 공통 데이터·평가 규칙

아래 수치는 이 프로젝트의 첫 engineering screening gate v0다. 법규·형식승인 또는 공도
안전기준이 아니다. test를 열기 전에 버전과 threshold를 commit하고, 결과를 본 뒤 같은 run에
맞춰 낮추지 않는다.

### 데이터 gate

- 모든 episode가 native six-camera 10 Hz, 유효 rate `>=9.5 Hz`, p99 gap `<=150 ms`,
  absolute gap `<=250 ms`, bundle coverage `>=99%`인 Common10 planning validation을
  통과해야 한다.
- 연속 주행을 잘라 split하지 않는다. route/site/day 단위 train, val, 손대지 않은 test를
  분리하고 model/threshold 선택에는 test를 사용하지 않는다.
- feature label이 없으면 `available=false`와 valid mask로 loss 전체에서 제외한다. 0으로
  채워 “객체 없음”, “정지 불필요” 같은 가짜 정답을 만들지 않는다.
- 각 일반 기능 폐루프는 최소 20 episode, 안전 중요 기능은 최소 30 episode를 사용한다.
  적용 가능한 경우 seed와 날씨/조명을 각각 3종 이상 포함한다.

### 공통 open-loop gate

- 같은 dataset fingerprint, split, seed budget, batch, runtime/hardware와 metric 분모로
  최소 3개 seed를 비교한다.
- accepted baseline보다 안전 중요 subgroup이 5% 넘게 퇴행하면 aggregate 평균이 좋아도
  채택하지 않는다.
- 기본 trajectory 목표는 selected ADE `<=0.5 m @1 s`, `<=1.0 m @3 s`,
  `<=2.0 m @6.4 s`, FDE `<=4.0 m @6.4 s`다. 기능별 표가 더 엄격하면 그 값을 따른다.

### 공통 closed-loop gate

- portable learned trajectory가 tested planning output을 인과적으로 소유해야 한다.
- 모든 run에서 collision 0, critical traffic-rule violation 0, 10 Hz health를 요구한다.
- 실패 시도도 denominator와 함께 남긴다. model/dataset/route/config hash, 원시 로그,
  PNG/GIF와 판정 JSON이 없는 결과는 승격 근거가 아니다.

## 로컬 PC와 Pro6000 역할

| 환경 | 담당 | 하지 않는 일 |
|---|---|---|
| 로컬 CARLA·Autoware PC | map/route/scenario와 sensor truth 생성, raw 수집, Common10 변환·검증, Autoware adapter, CARLA 폐루프, 화면·로그, 추후 real replay/shadow/폐쇄시험 | Pro6000의 다른 작업을 제어하거나 open-loop 수치만으로 실차 승인 |
| Pro6000 격리 학습 환경 | immutable dataset 재검증, deterministic 학습, fixed-split 평가, seed/model A/B, export·parity vector | CARLA world/실차 제어, safety policy 결정, 시스템·Conda·타 프로젝트 환경 수정 |

Pro6000에서는 프로젝트 개인 venv만 사용하고, 실행 직전 비어 있음을 확인한 physical GPU 0만
단일 장치 allowlist로 노출한다. 다른 GPU나 다른 사용자 process를 종료·reset하지 않고 학습을
위해 reboot하지 않는다.

## 9개 상위 기능

| ID | 상위 기능 | 하위 수 | 현재 상태 | 다음 핵심 gate |
|---|---|---:|---|---|
| `MF-01` | 센서·상태 이해 | 4 | CARLA foundation 일부 PASS | static geometry label + 실제 rig 계약 |
| `MF-02` | 경로·차로 주행 | 4 | 직진/좌 데이터 일부, 우회전 open-loop 미달 | 독립 straight/left/right/stop test + A/B |
| `MF-03` | 속도·승차감 | 3 | speed open-loop 미달 | speed/curvature/comfort label과 A/B |
| `MF-04` | 교통규칙·교차로 | 4 | 신호 expert 행동만 있음 | signal/stop/right-of-way/pedestrian label |
| `MF-05` | 선행차 추종·ACC | 3 | 시작 전 | lead state와 cut-in dataset |
| `MF-06` | 충돌위험·장애물 대응 | 4 | 시작 전 | occupancy/motion/risk label + 독립 AEB |
| `MF-07` | 차선변경·합류 | 3 | 시작 전 | 좌/우 command, legal boundary, gap label |
| `MF-08` | 저속 정차·주차 | 2 | 시작 전 | near-field coverage와 terminal pose data |
| `MF-09` | 런타임 안전·실차 전환 | 3 | source/unit + 입력 없는 ROS startup smoke | parity/latency/reject/fallback |

## 30개 추적 요구사항

표의 `단계`는 `현재 → 목표`다. 숫자는 최소 통과 조건이며 machine-readable 파일에는 데이터,
open-loop, closed-loop 문장과 두 환경의 책임을 생략 없이 기록했다.

### MF-01 센서·상태 이해

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `SEN-01` | 6카메라 동기·프레임 완전성 | P1→P5 / foundation CARLA-only | native ID/stamp/hash 6종, 모든 map·실제 rig | source join/hash 100%, cadence 조작 0 | bundle>=99%, skew<=1 ms, p99 gap<=150 ms, 미보고 drop 0 | capture·transport fault / loader mutation test |
| `SEN-02` | calibration·TF·화각 | P1→P5 / foundation CARLA-only | K/D/rectification/crop/FOV/T_base와 hash | order/calibration mutation 100% reject, fixture<=0.05 m | TF 100%, reprojection median<=1 px, p95<=2 px | 실제 보정·overlay / rig hash·ablation |
| `SEN-03` | ego motion·localization | P1→P5 / foundation CARLA-only | pose/speed/accel/yaw-rate/steer/validity | stale·jump·NaN/Inf 100% reject | freshness>=99.9%, unexplained jump 0, dropout selector 100% | state truth·fault inject / history-mask·noise 분석 |
| `SEN-04` | drivable area·lane edge | P0→P4 / not started | area/edge/footprint/occlusion/map alignment | IoU>=0.70, boundary F1>=0.90, off-road<=1% | footprint off-road·solid-line crossing 0/30 | semantic truth·overlay / M2 head·subgroup metric |

### MF-02 경로·차로 주행

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `RTE-01` | 직진 차로 유지 | P1→P4 / data partial | 3개 이상 site의 독립 train/val/test straight | 공통 ADE/FDE + lane exit<=1% | success>=95%, collision/lane 0, CTE p95<=0.35 m,max<=0.75 m/20 | holdout 수집·폐루프 / 3-seed A/B |
| `RTE-02` | 좌회전 | P1→P4 / data partial | lead+arc+tail, signal/priority/lane 포함 split | 공통 ADE/FDE, wrong direction 0 | branch 100%, success>=95%, collision/lane 0, CTE p95<=0.40 m/20 | 교차로 생성·폐루프 / curvature·yaw A/B |
| `RTE-03` | 우회전 | P2→P4 / below target | Town03 외 train과 untouched test 필요 | 공통 ADE/FDE; 현재 6.4 s 6.568/16.171 m로 미달 | branch 100%, success>=95%, collision/lane 0, CTE p95<=0.40 m/20 | train/test 수집·폐루프 / error 축소 A/B |
| `RTE-04` | 경로 종점 정지 | P1→P4 / data partial | bumper-relative target+natural approach+tail | position<=0.5 m,speed MAE<=0.5 m/s,false stop<=1% | 1 m 이내 정지>=95%, <=0.1 m/s 2초, overshoot/collision 0 | stop error 계측 / stop trajectory 학습 |

### MF-03 속도·승차감

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `SPD-01` | 30 km/h 추종 | P2→P4 / below target | straight/turn speed, limit, grade, curvature, actuator | speed MAE<=1.0 m/s; 현재 1.873 m/s | steady 직선 ±3 km/h>=80%, overspeed>33 km/h 0 | actuation 보정·동일경로 / speed A/B |
| `SPD-02` | 곡률·상황 감속 | P0→P4 / not started | curvature/friction/visibility/safe-speed | envelope 위반 0, lateral accel<=2.0 m/s² | lateral accel<=2.0 m/s², lane/collision/saturation 0 | friction/curve scenario / envelope-risk head |
| `SPD-03` | accel·decel·jerk | P0→P4 / not started | IMU/actuator/grade/intervention/emergency mask | accel MAE<=0.5 m/s², jerk MAE<=1.0 m/s³ | normal accel -3~2 m/s², |jerk| p95<=2.5 m/s³ | synchronized response / comfort regularization |

### MF-04 교통규칙·교차로

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `TRF-01` | 신호 인지·준수 | P1→P4 / expert only | relevant signal/state/phase/stop-line/occlusion | signal F1>=0.95, hazardous go miss 0 | red violation 0/30, green progress>=95% | phase·crossing truth / association·behavior head |
| `TRF-02` | 정지선·표지 | P0→P4 / not started | sign/line/ego-front/dwell/occlusion | stop recall100%, point<=0.5 m,false stop<=1% | overshoot 0,<=0.1 m/s 2초 30/30 | stop scene / intent·trajectory |
| `TRF-03` | 비보호 우선권 | P0→P4 / not started | conflict zone/priority/arrival/intent/gap | unsafe-enter recall100%, go/yield F1>=0.90 | priority violation·collision 0, completion>=95%/30 | crossing traffic truth / interaction-risk head |
| `TRF-04` | 횡단보도·보행자 양보 | P0→P4 / not started | pedestrian intent/velocity/crosswalk/occlusion | conflict recall>=0.98, hazardous miss0,yield F1>=0.95 | collision/intrusion0, post-clear progress>=95%/30 | pedestrian scenario / temporal interaction |

### MF-05 선행차 추종·ACC

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `ACC-01` | 선행차 상태 | P0→P4 / not started | lane lead ID,distance,relative speed/accel,occlusion | association F1>=0.95,distance<=1 m,rel-speed<=0.5 m/s | wrong-lead0,freshness>=99.9%/30 | lead truth / dynamic head |
| `ACC-02` | time-gap ACC | P0→P4 / not started | desired gap,lead/ego,grade,delay,brake class | gap error p95<=0.5 s,predicted TTC<1.5 s 0 | collision0,TTC<1.5 s0,gap p95<=0.5 s/30 | plant·profile / follow trajectory |
| `ACC-03` | cut-in·cut-out | P0→P4 / not started | onset/lateral motion/gap/occlusion/brake/intervention | hazard recall>=0.95,unsafe path0,response<=100 ms | collision0,TTC<1 s0,stable recovery/30 | scripted actor / temporal mining |

### MF-06 충돌위험·장애물 대응

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `OBS-01` | 정적 장애물 회피 | P0→P4 / not started | footprint/occupancy/corridor/pass legality/no-path | predicted collision0,clearance>=0.5 m,no-path recall100% | collision/lane0,legal pass-or-stop>=95%/30 | obstacle placement / geometry-risk head |
| `OBS-02` | 동적 장애물 회피 | P0→P4 / not started | tracks/flow/uncertainty/conflict/intervention | actor ADE3s<=1.5 m,hazard miss0,collision proxy0 | collision0,safe avoid-or-yield>=95%/30 | moving hazards / temporal occupancy |
| `OBS-03` | risk·candidate ranking | P0→P4 / not started | candidate collision/TTC/offroad/uncertainty/counterfactual | AUPRC>=0.90,hazard FN0,ECE<=0.05 | unsafe selection0,no-safe-path handoff100% | collision oracle / risk calibration |
| `OBS-04` | 독립 AEB | P0→P6 / not started | independent TTC,stopping envelope,false positives,brake health | avoidable trigger recall100%,false trigger<=1/100 km | avoidable collision0,otherwise impact 최소화,deadline100% | 독립 AEB·실차 envelope / advisory risk만 평가 |

### MF-07 차선변경·합류

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `LCM-01` | 좌측 차선변경 | P0→P4 / not started | change_left/legal edge/target occupancy/gaps/blind spot | F1>=0.90,illegal0,collision0,rear TTC>=3 s | completion>=95%,collision/edge0,rear TTC>=3 s/30 | legal/illegal scenario / left candidate |
| `LCM-02` | 우측 차선변경 | P0→P4 / not started | change_right/legal edge/target occupancy/gaps/blind spot | F1>=0.90,illegal0,collision0,rear TTC>=3 s | completion>=95%,collision/edge0,rear TTC>=3 s/30 | legal/illegal scenario / right candidate |
| `LCM-03` | 합류·추월 | P0→P4 / not started | topology/route need/gap/legality/intent/abort/return | go-yield-abort F1>=0.90,illegal pass0,TTC gate100% | legal complete-or-abort>=95%,collision/cut-off/route loss0/30 | multi-actor topology / rare abort mining |

### MF-08 저속 정차·주차

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `LSP-01` | 갓길정차·pull-over | P0→P6 / not started | legal area/curb/rear traffic/terminal pose/no-safe-area | safe area precision100%,<=0.3 m/5°,no-safe recall100% | collision/edge0,success>=95%,<=0.5 m/7°/30 | stopping map·control / intent·no-safe reject |
| `LSP-02` | 주차·후진 | P0→P6 / not started | near-field 360°,slot/curb/reverse/steer/occupancy | collision-free100%,terminal<=0.2 m/5° | contact0,success>=95%,<=0.3 m/7°,speed<=5 km/h/30 | parking rig·sim / 별도 low-speed model |

### MF-09 런타임 안전·실차 전환

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `SAF-01` | export·지연·freshness | P2→P3 / forward only | target-PC full timing,queue,stale/drop,model hash | engine parity XY<=0.05 m,speed<=0.1 m/s full test | sensor-to-plan p99<=100 ms,deadline/drop0,stale reject100% | adapter·full timing / export·parity vector |
| `SAF-02` | uncertainty·ODD·selector | P0→P5 / not started | ID/OOD weather/rig/map,corruption,ambiguity,fallback | OOD AUROC>=0.95,invalid reject100%,false reject<=1% | rejected output to control0,no-safe fallback<=1 cycle | ODD·mux·fault / confidence calibration |
| `SAF-03` | fallback·MRM·release | P0→P6 / not started | fault taxonomy,stop path,health,intervention,signed provenance | 모든 fault의 deterministic fallback,command owner 충돌0 | CARLA MRM PASS→real shadow>=10 h→closed-course 30회 | MRM·release owner / immutable artifact·mining |

## 구현 우선순위

1. `SEN-04`, `RTE-01~04`, `SPD-01`에 필요한 독립 test episode와 lane/drivable label을
   먼저 만든다. 현재 Town03 `val`을 test로 바꾸지 않는다.
2. 동일 split과 최소 3 seed로 v0 10-epoch 기준선, untrained physical v1과 Geometry-BEV
   후보를 A/B한다. 1-epoch의 gradient clipping 관찰만 현재 10-epoch 전체 학습으로 일반화하지
   않고 learning rate, loss scale과 gradient 분포를 새 run마다 채택 gate에 포함한다.
3. `SAF-01~02` adapter·parity·freshness·selector를 구현한 뒤에만 `RTE-01~04`의
   30 km/h CARLA closed-loop를 실행한다.
4. 기본 경로·정지 gate가 통과하면 `TRF`→`ACC`→`OBS`→`LCM` 순으로 scenario와 head를
   하나씩 추가한다. AEB는 learned model과 독립된 계층으로 유지한다.
5. 30 km/h 폐루프 기준선과 real replay/shadow 전에는 60 km/h 또는 실차 actuator 연결로
   승격하지 않는다.

## 추적·변경 규칙

- 구현 PR과 evidence bundle은 반드시 하위 기능 ID를 사용한다. 예:
  `RTE-03`, `SAF-01`.
- `current_stage`, `status`, threshold를 바꾸는 commit은 근거 report 경로와 dataset/model
  hash를 함께 갱신한다.
- threshold 변경은 새 matrix schema/version 또는 명시적 revision으로 한다. 이미 본 test
  결과에 맞춰 같은 버전의 기준을 수정하지 않는다.
- `CLOSED_LOOP_PASS` 승격에는 실패 포함 전체 denominator, learned-output ownership,
  zero-collision/violation, 10 Hz health와 재현 가능한 원시 evidence가 모두 필요하다.
- 실차 승격은 `P5_REAL_SHADOW`와 `P6_CLOSED_COURSE`의 별도 승인 기록이 필요하다. 이 문서는
  공도 운행을 승인하지 않는다.
