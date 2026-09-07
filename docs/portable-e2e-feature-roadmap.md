# Portable E2E 자율주행 기능 요구사항·로드맵

> 기준일: 2026-09-07
>
> 기계 판독 원본: [`config/portable_e2e_feature_matrix.yaml`](../config/portable_e2e_feature_matrix.yaml)

## 현재 결론

<!-- HH_260906 - Refresh feature progress from completed physical-v1 training and isolated live shadow evidence. -->
목표 기능을 9개 상위 영역, 30개 하위 기능으로 분해했다. 현재 완료된 것은
`common_10hz_v1` 데이터·학습 배선, CARLA-only physical-v1 학습·open-loop 평가,
로컬 3장면의 10 Hz shadow 계측과 오늘의 네 캠페인 총 15회 학습·평가·감사다. 새 후보들은
채택 기준을 통과하지 못해 기존 shadow checkpoint를 유지한다. 학습된
checkpoint가 Autoware 또는 CARLA를 폐루프로 주행한 적은 없으며, 30개 중
`CLOSED_LOOP_PASS`인 기능은 **0개**다.

현재 판정의 근거는 다음과 같다.

- Town07 직진 309개와 CTrack 좌회전 304개를 학습하고 Town03 우회전 337개를
  validation한 총 950개 CARLA sample은 데이터 계약을 통과했다.
- **기존 shadow 운용 checkpoint** physical-v1은 train 613개로 10 epoch, 1,540 optimizer step을 완료했다. Town03 `val`
  337개에서 6.4초 selected ADE/FDE는 `4.2615/10.9172 m`, speed MAE는 `1.3933 m/s`다.
  기존 v0의 `6.567681/16.171295 m`, `1.872983 m/s`보다 개선됐지만 아래 초기 품질 gate와
  독립 test·3-seed 요구는 아직 충족하지 못했다.
- 학습 직후 보존한 physical-v1 offline geometry gate v6에서 selected PASS는 `326/337`,
  하나 이상 candidate PASS는 `326/337`, 모든 candidate PASS는 `325/337`이다. 선택은 c2에
  `337/337` 고정되어 후보 선택·품질을 추가 분석해야 한다. 이 빈도만으로 여섯 후보의 경로
  모양이 같다고 단정하지 않는다. live runtime gate는 v8이며
  이 historical v6 분모를 v8 결과로 바꾸어 읽지 않는다.
- source checkpoint
  `df2a4b75a978f35c213894c56cf3a905f547734ee8099e8142133bdc9bd3ae09`와 runtime bundle
  `bdd3daf605e269a8d90ea8e56ab1691e7e49db50e3f2100c7b66469813569d4e`를 고정했다.
- 오늘 별도로 학습한 LR A/B 6회, 데이터 확장 C 3회, selector 가중치 D 3회,
  후보 경로를 입력받는 점수 모델 E 3회는 모두
  학습·val337·gate v8 감사를 완료했지만 승격하지 않았다. A/B는 seed `20260905`, D/C는
  seed `20260903`, E/C도 seed `20260903`에서 상대 기준을 통과하지 못했고 C·D·E의
  절대 품질 판정은 모든 seed에서 FAIL이었다. C baseline을 D/E 비교에 재사용한 것은 추가
  학습 횟수로 세지 않는다. E는 연구용이며 기존 운용 checkpoint를 대체하지 않는다.
- 데이터는 Town01 우회전 train 534개와 Town04 직진 test 309개를 추가해 v3의
  train/val/test가 `1,147/337/309`개가 됐다. 기존 train/val은 보존됐고 test는 모델 평가·선택에
  사용하지 않았다. 여전히 모든 기능에 필요한 label·독립 시나리오가 갖춰진 것은 아니다.
- Town07 직진, C-track 좌회전, Town03 좌회전에서 sensor-fed Portable trajectory를
  기록했고 3/3 장면이 `EVIDENCE_VALID`, 10 Hz `ESTABLISHED_FOR_SHADOW_ONLY`였다.
  accepted 수는 각각 `547/508/497`, inference p99는 `40.408/38.482/36.862 ms`였다.
  Town03 **우회전 val**과 **좌회전 live shadow**는 다른 episode다.
- 9월 7일의 30 km/h 제어 A/B 여섯 arm에서도 Portable 10 Hz shadow는 유효했다. 차량
  제어는 Autoware VAD가 소유했고 새 제어 설정은 승격되지 않았다. Portable의 learned
  closed-loop 실적은 0회이며, `SAF-01`의 full frozen-set parity·fault/fallback·release
  요구도 모두 완료된 것은 아니다.
- Town03 expert가 신호를 기다린 기록은 있으나 신호 상태가 현재 모델 입력이나 label은
  아니다. 따라서 교통신호 기능은 `EXPERT_BEHAVIOR_ONLY`다.
- 과거 Autoware VAD 화면은 별도 stack의 역사적 증거다. 이번 portable E2E checkpoint의
  runtime 또는 closed-loop 증거로 사용하지 않는다.

자세한 현재 수치는
[2026-09-05 Common10 학습·검증 보고서](validation-2026-09-05-portable-e2e-common10-30kph.md),
[2026-09-06 duration A/B evidence](assets/validation/2026-09-06/portable_e2e_v0_duration_ab_v1/README.md),
[physical-v1 3장면 shadow evidence](assets/validation/2026-09-06/portable_e2e_physical_v1_30kph_shadow_v3/README.md),
[2026-09-07 제어 A/B](validation-2026-09-07-control-ab.md),
[오늘의 반복 학습 결과](validation-2026-09-07-portable-learning.md),
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
| `SHADOW_MEASURED_CARLA_ONLY` | 로컬 sensor-fed CARLA shadow·지연 계측 완료. 전체 release gate와 learned 제어 승인은 미완료 |
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

모든 하위 기능은 현재 작업 중인 gate와 목표 단계를 함께 가진다. `current_stage`는 그 단계의
완료 선언이 아니며 `status`와 근거를 함께 읽는다. 앞선 품질·데이터 gate가 미완료인 상태의
shadow 계측도 제어 승격 근거가 되지 않는다. 빠른 GPU forward는 `P3_RUNTIME`을,
open-loop ADE는 `P4_CARLA_CLOSED_LOOP`를 대신하지 않는다.

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
| `MF-09` | 런타임 안전·실차 전환 | 3 | 3장면 + 후속 6 arm의 10 Hz shadow 계측 | full-set parity·selector·fault/fallback·learned ownership |

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
| `RTE-01` | 직진 차로 유지 | P1→P4 / data partial | Town07 train·Town04 test 확보, 3개 이상 site와 독립 val 확장 필요 | 공통 ADE/FDE + lane exit<=1% | success>=95%, collision/lane 0, CTE p95<=0.35 m,max<=0.75 m/20 | holdout 수집·폐루프 / 3-seed A/B |
| `RTE-02` | 좌회전 | P1→P4 / data partial | lead+arc+tail, signal/priority/lane 포함 split | 공통 ADE/FDE, wrong direction 0 | branch 100%, success>=95%, collision/lane 0, CTE p95<=0.40 m/20 | 교차로 생성·폐루프 / curvature·yaw A/B |
| `RTE-03` | 우회전 | P2→P4 / below target | Town01 train534·Town03 val337, untouched 우회전 test 필요 | 공통 ADE/FDE; 기존 shadow physical-v1 6.4 s 4.262/10.917 m로 미달 | branch 100%, success>=95%, collision/lane 0, CTE p95<=0.40 m/20 | train/test 수집·폐루프 / error 축소 A/B |
| `RTE-04` | 경로 종점 정지 | P1→P4 / data partial | bumper-relative target+natural approach+tail | position<=0.5 m,speed MAE<=0.5 m/s,false stop<=1% | 1 m 이내 정지>=95%, <=0.1 m/s 2초, overshoot/collision 0 | stop error 계측 / stop trajectory 학습 |

### MF-03 속도·승차감

| ID | 기능 | 단계 / 상태 | 데이터 요구 | Open-loop 통과 | Closed-loop 통과 | Local / Pro6000 |
|---|---|---|---|---|---|---|
| `SPD-01` | 30 km/h 추종 | P2→P4 / below target | straight/turn speed, limit, grade, curvature, actuator | speed MAE<=1.0 m/s; physical-v1 1.393 m/s | steady 직선 ±3 km/h>=80%, overspeed>33 km/h 0 | actuation 보정·동일경로 / speed A/B |
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
| `SAF-01` | export·지연·freshness | P3→P3 / shadow measured CARLA-only | target-PC full timing,queue,stale/drop,model hash | engine parity XY<=0.05 m,speed<=0.1 m/s full test | sensor-to-plan p99<=100 ms,deadline/drop0,stale reject100% | shadow 계측 완료·fault 확장 / export·full-set parity |
| `SAF-02` | uncertainty·ODD·selector | P0→P5 / not started | ID/OOD weather/rig/map,corruption,ambiguity,fallback | OOD AUROC>=0.95,invalid reject100%,false reject<=1% | rejected output to control0,no-safe fallback<=1 cycle | ODD·mux·fault / confidence calibration |
| `SAF-03` | fallback·MRM·release | P0→P6 / not started | fault taxonomy,stop path,health,intervention,signed provenance | 모든 fault의 deterministic fallback,command owner 충돌0 | CARLA MRM PASS→real shadow>=10 h→closed-course 30회 | MRM·release owner / immutable artifact·mining |

## 구현 우선순위

<!-- HH_260906 - Separate completed research campaigns from the unchanged shadow deployment and outstanding feature gates. -->
2026-09-07의 반복 학습은 **네 캠페인 모두 완료, 새 모델 미채택**으로 정리했다.
마지막 E 감사 완료 시각은 `14:34:14 UTC`(`23:34:14 KST`)다.

| 실험 | 학습·평가·감사 | 판정 |
|---|---:|---|
| [LR A/B](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/01_learning_rate_ab/README.md), v2 train613 | 6회 완료 | seed 20260905 상대 비교 FAIL, 전체 절대 품질 FAIL, 미채택 |
| [데이터 확장 C](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/06_data_expansion/README.md), v3 train1147 | 3회 완료 | 모든 seed 절대 품질 FAIL, 다른 corpus와의 자동 비교·승격 없음 |
| [selector weight D/C](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/07_selector_weight_ab/README.md), 같은 v3에서 0.1→0.5 | D 3회 완료 | seed 20260903 상대 비교 FAIL, 모든 D seed 절대 품질 FAIL, 미채택 |
| [후보 경로 인지 점수 E/C](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/11_candidate_rank_ab/README.md), 같은 v3·가중치 0.1, 점수 모델 구조 변경 | E 3회 완료 | seed 20260903 상대 비교 FAIL, 모든 E seed 절대 품질 FAIL, 미채택 |

전체 자료는 [오늘의 검증 보고서](validation-2026-09-07-portable-learning.md), 실행 흐름은
[학습·검증 반복 가이드](portable-e2e-learning-loop.md)에 있다. 위 15회는 30개 기능의 완료
횟수가 아니며 새 후보가 로컬 차량을 제어했다는 의미도 아니다.

<!-- HH_260906 - Keep measured candidate-aware research results separate from the retained shadow checkpoint and CPU diagnostics. -->
[E 설계](portable-e2e-candidate-ranking.md)는 후보 경로 정보도 점수 계산에 사용하며
parameter가 C `954,590`개에서 E `1,068,249`개로 달라진다. 데이터·seed·1,540 step은
고정했지만 모델 구조와 학습 연산량까지 같은 실험은 아니다. 물리 decoder·loss·평가·gate는
유지했다. 아래는 동일 Town03 val337의 **원래 GPU 최종 평가**이며 CPU 진단값과 섞지 않는다.

| Seed | C ADE / FDE (m) | E ADE / FDE (m) | E 상대 판정 | E 절대 판정 |
|---|---:|---:|---|---|
| 20260903 | 3.874328 / 9.827706 | 4.735725 / 9.413042 | FAIL: ADE 악화 | FAIL |
| 20260904 | 6.258545 / 12.897454 | 4.539105 / 9.709312 | PASS | FAIL |
| 20260905 | 4.789547 / 11.866288 | 4.228415 / 9.348654 | PASS | FAIL |

C/E의 selected geometry는 모두 `336/337`이고 E의 속도 MAE는 세 seed 모두 C보다
낮지만, 첫 seed의 ADE 악화와 절대 품질 미달로 채택하지 않는다.
[E 3개 seed의 차량 중심 경로 PNG 18장](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/12_candidate_rank_route_analysis/README.md)은
검증 데이터 예측 그림이지 새 CARLA/Autoware 주행 촬영이 아니다. E의 live 10 Hz 계측이나
learned closed-loop 실행은 하지 않았고 기존 shadow checkpoint·bundle을 그대로 유지한다.
[코드 회귀 검증](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/13_candidate_rank_code_validation/README.md)은
`2,333 passed / 6 skipped`다. skip은 구형 로컬 Torch의 secure loader 미지원 5개와
고정 Woraksan 자산 부재 1개이며, 테스트 통과를 주행 기능 승인으로 해석하지 않는다.

이후 **첫 seed `20260903`의 selector 진단**을 완료했다. A는 c2, B는 c4를 각각
337/337 선택했지만 후보 간 평균 경로 거리는 `7.534858/7.545791 m`로 실제 경로들은
서로 달랐다. B의 selected ADE는 `4.261500 → 4.150919 m`로 개선됐지만 ADE oracle 대비
평균 regret는 `2.500854 → 2.515469 m`로 증가했다. [val337 진단과 고정 phase PNG 12개](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/04_selection_diagnostics/README.md)에
수치·정의·SHA를 보존했다. 추가한 [첫 seed C/D 진단](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/08_selector_weight_diagnostics/README.md)에서는
선택 index 종류가 늘어도 oracle 일치율이 `68.84% → 51.93%`, 평균 regret가
`2.322154 → 2.714173 m`로 악화됨을 확인했다. 이 CPU 진단은 live 10 Hz 증거가 아니며
현재 운용 모델은 자동 교체하지 않는다.

추가로 [C/D 3개 seed의 학습 목표·ADE 목표 일치 진단](assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/10_objective_alignment/README.md)을
train1147과 val337로 분리해 완료했다. 가중치를 높이면 학습 데이터의 선택 일치율은 모두
올랐지만 validation 변화는 일관되지 않았다. D seed `20260904`는 CPU/GPU 선택 분포도
달라 원래 GPU 판정과 CPU 재분석을 별도로 보존한다. near-tie 등 원인은 아직 확정하지 않았다.

반복 단위는 **실패 분석 → 필요한 데이터·label 확보 → 학습 → 고정 val 평가 → geometry·
runtime 검사 → shadow → 조건 충족 시 learned closed-loop**다. loss가 내려가는 것만 보고
epoch를 계속 늘리지 않고, 매 run의 모델·데이터·설정 hash와 채택/보류 이유를 남긴다.

완료한 LR·데이터 확장·selector 가중치·후보 경로 인지 점수 모델 실험과 목표 일치 진단을
바탕으로 다음 loss/ranking·데이터
비교를 새 고정 계획으로 진행한다. 이 val을 반복 관찰한 결과는 개발용 비교이며 독립 최종
test 성능이라고 부르지 않는다. 새 test는 route/site/day 단위로 분리하고, 열기 전에
설정·threshold를 고정한다. 후보 채택에는 최소 3개 seed 비교가 필요하다.

<!-- HH_260906 - Map all thirty requirements to concrete collection, learning, and evaluation dependencies. -->
| 반복 묶음 | 추적 기능 | 로컬 데이터·시험 준비 | Pro6000 학습·검증 | 다음 묶음으로 넘어갈 근거 |
|---|---|---|---|---|
| `ITER-01` 기본 주행·런타임 | `SEN-01~04`, `RTE-01~04`, `SPD-01~03`, `SAF-01~03` | 독립 직진·좌/우회전·종점정지, lane/drivable label, 속도·가감속·jerk 계측, selector/fallback fault | physical-v1 loss/ranking A/B, 후보별 geometry·ADE/FDE·speed·comfort, 3-seed 비교 | offline 품질·parity·shadow 통과 후 제어 소유권과 fallback을 검증한 CARLA 폐루프. `SAF` 실차 단계는 계속 별도 추적 |
| `ITER-02` 교통규칙 | `TRF-01~04` | 관련 신호·정지선/표지·우선권·보행자·가림 label과 phase/교차 시나리오 | behavior·interaction head, 위험 miss·정지/우선권 위반 평가 | 각 기능의 독립 test와 CARLA gate |
| `ITER-03` 추종·ACC | `ACC-01~03` | lead ID/거리/상대속도·gap/TTC, 정지/급감속 lead·cut-in/out | 선행차 상태·추종 trajectory, gap·반응시간·충돌 비교 | 독립 lead profile과 seed에서 ACC gate |
| `ITER-04` 장애물·위험 | `OBS-01~04` | occupancy·legal corridor·candidate collision/TTC, pass/stop·no-path·moving hazard·독립 AEB | occupancy/motion·risk ranking, 위험 false negative·clearance | 장애물 gate와 독립 AEB gate를 각각 통과 |
| `ITER-05` 차선변경·합류 | `LCM-01~03` | legal boundary·옆 차로/후방 gap·blind spot·abort/return label | maneuver별 후보와 gap 수용, illegal 선택·rear TTC·복귀 평가 | 새 topology/traffic/weather에서 left/right/merge gate |
| `ITER-06` 저속·주차 | `LSP-01~02` | 근거리 보정·정차/주차 목표·후진·접촉/종점 pose | 별도 저속/후진 모델, terminal pose·collision 평가 | CARLA 저속 gate 후 real replay/shadow와 폐쇄시험장 |

모든 묶음은 현재의 30개 요구사항을 배정한 작업 순서이며 완료 실적이 아니다. 신호·객체·차선
label이 없는 기존 corpus를 반복 학습하는 것만으로 후속 기능이 생기지는 않는다. Geometry-BEV
같은 추가 구조도 필요한 label·입력 계약과 동일 조건 평가가 준비된 뒤 별도 후보로 비교한다.
30 km/h Portable learned closed-loop 기준선, 속도별 정지거리·곡률·횡가속·fallback 검증 뒤에
속도를 확장한다. 기존 VAD의 60 km/h 계측은 Portable 모델의 해당 속도 학습 실적이 아니다.

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
