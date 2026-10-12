# Portable E2E — TRAIN 30 / 개발 VAL 10 데이터 확대 제안

<!-- HH_260906 - Report the completed non-executing cohort planner and the requested moving-data implementation; retain technical qualification, immutable failed data and explicit split boundaries. -->

## 목표와 현재 경계

<!-- HH_260906 - Record the real seed-zero pilot and qualified transfer without counting it as a preregistered cohort slot. -->
**9월 15일 실제 진행:** [Town07 이동 데이터 pilot 촬영](assets/validation/2026-09-15/portable_e2e_learning_cycle_v1/04_moving_data_pilot/README.md)을
완료했고, 후속 명시적 Common10 변환·로컬/원격 검증에서 **TRAIN 328개·영상 1,968장**이 통과했다.
실제 변환·서버 수신 증거는 [05 학습 데이터 결과](assets/validation/2026-09-15/portable_e2e_learning_cycle_v1/05_moving_train_dataset/README.md)에 있다.
원격 개인 `dataset/prepared/carla-town07-moving-pilot-20260915-v1`에 새로 보관했다.
기존 학습 데이터나 실행 모델은 교체하지 않았다. 원본 전체 실패·목표 미도달 기록을 보존했고,
선언된 이동 구간만 별도 작업 계약으로 검증했다. 이것은 seed **0** 기술 pilot이므로 아래
seed 101/102/201/202의 40개 cohort 슬롯 중 완료로 세지 않는다. 직진 한 구간이며 정지·재출발·
좌우 회전·새 DEV 확보는 남아 있다. 현재 진행 중인 20-epoch 실험에는 이 자료를 섞지 않았다.

**목표는 새 독립 episode TRAIN 30개·개발 VAL 10개다. 수집 완료 수나 자동 실행 명령이 아니다.**
현재 TRAIN 1,147표본은 Town07 직진·C-track 좌회전·Town01 우회전 **3 episode, 약 133.9초**다.
개발 VAL은 Town03 우회전 **1 episode, 약 40.15초**다. 같은 데이터를 반복 학습하는 것과
새 장면을 확보하는 것을 구분한다. [실제 분모](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/07_coverage_and_curvature/README.md)를 따른다.

이번 제안만으로 기존 manifest/split·소스·판정 기준을 바꾸지 않는다. **Town04와 Town15는
계속 잠금**이며 카탈로그·TEST 영상/정답을 읽거나 새 개발 수집 대상으로 사용하지 않는다.
Town03은 기존 suite의 `train` 표기와 관계없이 개발 전용이다. 새로 제안하는 Town02_Opt의
DEV 배정도 별도 cohort 선언·중복/과거 사용 검토가 끝나기 전에는 효력이 없다.
사용자는 이 범위의 데이터 확대 구현을 이미 요청했다. 일반적인 구현을 시작하려고 다시 포괄적인
허가를 요청하는 단계는 아니다. 실행 전 소유권·소스 고정·기술 검증과 실행 후 데이터 admission은
그와 별개로 충족해야 하며, 미충족 항목을 완료로 바꾸지 않는다.

## 1. 정확한 40개 할당

`T07`, `CT`, `T01`, `V03`, `V02`는 아래 표의 map/split에 고정한 제안 ID 접두사다.
각 셀의 `01/02`는 서로 다른 **경로 그룹과 source session**이어야 한다. 같은 경로에서 seed만
바꾼 반복을 독립 경로 2개로 세지 않는다. 경로 교차·역방향·연결 구간의 중복도 검토한다.

| 제안 split / 맵 / 접두사 | 직진 `S` | 좌 `L` | 우 `R` | 자연 감속→정지 유지 `H` | 정지 유지→재출발 `G` | 합계 |
| --- | --- | --- | --- | --- | --- | ---: |
| TRAIN / Town07 / T07 | S01·S02 | L01·L02 | R01·R02 | H01·H02 | G01·G02 | 10 |
| TRAIN / C_track_1_0_7 / CT | S01·S02 | L01·L02 | R01·R02 | H01·H02 | G01·G02 | 10 |
| TRAIN / Town01 / T01 | S01·S02 | L01·L02 | R01·R02 | H01·H02 | G01·G02 | 10 |
| DEV VAL / Town03 / V03 | S01 | L01 | R01 | H01 | G01 | 5 |
| DEV VAL / Town02_Opt / V02 | S01 | L01 | R01 | H01 | G01 | 5 |
| **합계** | **8** | **8** | **8** | **8** | **8** | **40** |

실제 ID는 `T07-S01`처럼 접두사와 셀을 연결한다. TRAIN은 행동별 6개, DEV는 행동별 2개다.
기존 3+1 episode를 이 40개에 포함하지 않는다. 어떤 run이 두 행동을 보여도 한 episode를
두 슬롯의 독립 실적으로 중복 계산하지 않는다. 재출발에는 그 전에 실측 정지 유지가 필요하다.

- TRAIN의 각 `01`: ClearNoon, traffic seed 101. 각 `02`: CloudyNoon, seed 102.
- V03: S/R/G는 ClearNoon, L/H는 CloudyNoon, seed 201.
- V02: S/R/G는 CloudyNoon, L/H는 ClearNoon, seed 202.
- 모든 슬롯은 별도 source session을 갖는다. 날씨 이름의 실제 API 지원·14개 수치 readback,
  경로 SHA·시작/끝·길이·route-group·기록 길이·예산을 실행 전 계획에 고정한다.
- 2종 날씨와 5맵은 첫 확대 묶음일 뿐이다. 모든 맵·날씨, 기능별 최소 폐루프 episode 수,
  untouched release TEST를 충족했다는 뜻이 아니다. Town02_Opt는 release TEST가 아닌 DEV 제안이다.

기존 카탈로그의 직진/좌/우 후보 수는 Town07 **8/8/6**, C-track **8/8/8**, Town01 **8/8/8**,
Town03 **8/7/8**, Town02_Opt **8/8/3**이다. 이는 후보 기하 수이지 위 40개 경로의 독립성·주행
품질 보증이 아니다. 이동 24칸의 정확한 후보 ID·SHA·길이·방향은 아래 완료된 planner 결과에
기록했다. 길이 또는 경로 독립성이 부족한 슬롯은 그대로 표시한다.
[기존 경로와 전체 맵 경계](portable-e2e-map-acceptance-2026-09-14.md)를 따른다.

Town05_Opt·Town06·Town10HD_Opt와 다른 등록 맵은 후속 확대 목록에 남긴다. Town06·Town10HD_Opt는
현재 카탈로그에 좌회전 후보가 없으며, Woraksan은 `ready` 문구와 지정 자산 상태가 다르다.
이번 40개로 전체 19맵을 완료했다고 표시하지 않는다.

## 2. 종점 미세 움직임 해결만 기다리지 않는 작업 순서

### A. 먼저 준비할 이동 행동 24개: TRAIN 18 / DEV 6

직진·좌·우를 우선 준비한다. 핵심 목표는 **이동 중 영상·경로·속도·회전 일반화 데이터**이며
정지 성공을 해당 행동의 대리 지표로 삼지 않는다. 경로를 충분히 길게 잡고, 회전 방향·진입/회전/
이탈을 모두 포함한다. 30 km/h는 상한이며 회전 중 강제 유지하지 않는다.

### 완료: 실제 경로에 결합한 비실행형 planner

[planner](../tools/e2e/plan_portable_moving_cohort.py)를 실제 non-TEST 카탈로그에 실행한
[cohort.json](../storage/experiments/training/2026-09-15/moving_cohort_v1/cohort.json)은 다음을 기록한다.

| 확인한 항목 | 실제 결과 |
| --- | ---: |
| 확인한 카탈로그 / 경로 payload | 10 / 112 |
| 서로 다른 경로 SHA·정방향/역방향 기하에 결합한 슬롯 | 24: TRAIN 18 / DEV 6 |
| 누락 카탈로그·payload | 0 |
| 공유 road-section 경고 | 9슬롯 / 6쌍 |
| 15 km/h 기하 예산 충족 | 7: TRAIN 직진 6 / Town03 DEV 직진 1 |
| 30 km/h 기하 예산 충족 | 0 |
| 새 수집 / 새 학습 승인 | 0 / 0 |

모든 좌·우 후보의 선언 방향과 직렬화 yaw 변화 부호가 일치했다. 카탈로그 길이는 XYZ arc이며,
시간·거리 예산은 별도로 재계산한 XY arc를 사용했다. 같은 road-section은 보수적 중복 경고다.
다른 파일 SHA도 통계적 독립성을 보증하지 않으며 기존 TRAIN/VAL raw 경로와의 공간 중복 검사는
아직 수행하지 않았다. Town03·Town02_Opt의 기존 `train` 표기와 제안 DEV 역할의 차이도 남겼다.

기존 Common10은 전체 raw 길이가 아니라 **첫 선택 anchor부터 마지막 선택 anchor까지 30초 이상**을
요구한다. lead-in 30m + 선택 30초 + 미래 6.4초의 단순 기하 예산은 15 km/h에서 **181.667m**,
30 km/h에서 **333.333m**다. 따라서 16개 turn 슬롯과 Town02_Opt 직진 177.148m는 현재 15 km/h
예산에서도 부족하다. 더 긴 경로나 사전에 정한 속도 계획이 필요하며 gate를 완화하지 않는다.

첫 파이프라인 pilot 후보는 `T07-S01 / town07_straight_s0000_p00`이다. 길이 210.598m,
SHA `804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848`에 결합했다.
**15 km/h, 경로 arc 30–165m**의 고정 구간은 정속 가정에서 32.4초이며, 마지막 선택 anchor의
실제 미래 6.4초를 확보하면 명목상 arc 191.667m까지 진행해 경로 약 18.931m가 남는다.
DEV 후보 `V03-S01 / town03_straight_s0000_p04`의 명목 여유는 17.683m다.
실측 선택 span·속도·미래 지원·cleanup 여유를 실행 후 따로 검증하며 **15 km/h pilot을 30 km/h
검증이라고 하지 않는다.** 165m 최초 초과로 선택 구간을 닫고 다시 진입해도 재개하지 않는
전향적 상태 규칙을 새 capture 구현에서 고정한다.

새 planner 28개와 기존 planner 28개, 합계 **56개 테스트 통과** 후 결과를 생성했다.
생성 직후 131개 소스·입력 SHA가 전후 동일함을 재확인했다. 결과 SHA는
`32765291218e35ce1ea002db68f8b2e0b694889252c82e5e38bc8cde8f271863`이다.
이 결과는 수정하지 않았다. 결과에 결합된 **이 문서의 이전 바이트**도
[provenance 사본](../storage/experiments/training/2026-09-15/moving_cohort_v1/provenance/docs/portable-e2e-data-expansion-2026-09-15.md)에
SHA `ba6a19bdc0318940a61476a0b3dfc50ad453a562fbd9ab636e89896eab6a3d18`로 보존했다.
현재 문서는 완료 결과를 추가한 후속 문서이므로 그 과거 입력 SHA와 같다고 주장하지 않는다.
같은 결과가 참조한 이전 collector 바이트는
[v9 A의 소스 아카이브](../storage/experiments/training/2026-09-15/terminal_approach_v9/01_coast_A/provenance/scripts/e2e/collect_carla_vad_expert.py)에
SHA `e1850a4e21147712768ebee8885fd5d979befe65957f58ff801e1ad96566e093`로 보존되어 있다.
후속 moving 모드 구현으로 현재 collector가 달라져도 이 과거 입력 및 결과를 덮어쓰지 않는다.

### 구현·TRAIN pilot 검증 완료 / 다음: cohort 일반화와 DEV 확보

이동 전용 opt-in 수집과 정확히 고정한 첫 사례의 변환·검증 연결은 구현하고 실제 원본으로 확인했다.
첫 변환 시도는 경로 정렬 이력 필드가 없어 거부됐다. 원본을 수정하지 않고, 사전에 고정한
owner 경로 파일·SHA로 직접 경로의 신원을 확인하는 v2 연결부를 검증한 뒤 변환했다.
기존 adapter의 일반 경로/종점 성공 조건과 과거 명시적 거부 규칙은 유지한다.
현재 qualification은 이 정확한 사례에 고정돼 있어 다른 맵에 옵션만 바꿔 일괄 실행할 수 없다.
다음 단계는 같은 불변성·선택·품질 조건을 새로 선언하는 cohort 구성으로 일반화하는 것이다.

1. 위 24개 경로 결합을 유지하며 경고가 있는 경로 그룹과 기존 raw/source-session 중복을 검토한다.
2. 경로별 직진·좌·우와 실제 회전 구간, 속도 계획, weather/seed, 6카메라 보정을 고정한다.
3. 수집할 lane/drivable·depth·signal/stop-reason의 출처·좌표·시각·availability/mask 사양을 작성한다.
   새 센서/정답 수집 구현과 검증 없이 현재 absent label을 있다고 표시하지 않는다.
4. 아래 **명시적 opt-in 이동 전용 수집·검증 경로**의 첫 TRAIN 기술 pilot 근거를 유지한다.
   cohort 배정/경로 중복 검토를 마친 TRAIN·DEV 직진을 새로 선언해 검증하고, 통과한 프로토콜로 좌/우까지 확대한다.

**보존하는 기존 동작:** 기존 `collect_carla_vad_expert.py` 모드는 종점 완료형이다. 제한 시간 만료는
실패이고, 종료 때 제동을 요청한다. `--goal-stop-profile disabled`도 종점 제동 자체를 없애는
이동 전용 모드가 아니다. 따라서 지금 있는 옵션만으로 “종점 문제를 피한 학습 데이터 수집”을
실행할 수 있다고 주장하지 않는다. 기존 wrapper나 전체 suite의 `--execute`로 우회하지 않는다.

종점 제어와 독립된 이동 데이터에는 **새로운, 사전에 정한 moving-capture 모드와 admission
검사**가 필요하다. 첫 사례의 opt-in 구현·검증은 완료했지만 아래 요구는 후속 사례에도 유지한다.
변환·전송 완료와 실제 새 데이터 학습은 별개이며, 이번 pilot을 학습한 새 모델은 아직 없다.

- 기록은 spawn/bootstrap부터 시작해 초기화·출발·이동·종료와 실패를 전부 보존한다.
- 학습용 이동 구간은 **run 전에 고정한 경로 landmark/행동 구간**으로 정의한다. 실패 곡률이나
  사후 예측 오차를 보고 시작/끝을 옮기지 않는다. 필요하면 출발부터 전부 쓰는 기존 계약과
  별도 버전임을 명시하고, 과거 실패 episode에는 소급 적용하지 않는다.
- 이동 구간 이후에도 실제 주행을 계속 기록하여 마지막 선택 anchor의 **6.4초 실제 미래**를
  확보한다. 타임스탬프 변경·pose/velocity reset·가짜 정지·누락 미래의 0 채우기를 하지 않는다.
  선택 anchor span 30초 이상도 실제 카메라 시각으로 확인하며, bootstrap·종료 실패를 지우지 않는다.
- 이동 구간의 모든 사전 선택 anchor에 **기존과 같은 수치·카메라·제어·미래 물리 기준**을 적용한다.
  한 실패 anchor를 빼서 통과시키지 않는다. collision/ownership/ACK/source/cleanup 실패는 전체 거부다.
- bootstrap/종료 구간도 원래 검사 결과를 별도로 모두 보고한다. 이것을 기존의 **전체 stop episode
  승인**이라고 부르지 않는다. 새 task-scoped admission 결과를 원본 SHA에 결합해 기록해야 한다.
- 기존 모든 recorded warmup/driving anchor를 검증하는 admission 계약을 그대로 요구한다면,
  위 이동-only 자료도 아직 편입할 수 없다. 새 스키마·선택 규칙의 검증 없이 validator를 우회하지 않는다.
  새 raw의 실제 `goal_reached=false`와 별도 `task_complete`·`PENDING_QA`를 구별하며,
  기존 adapter의 거부 규칙과 과거 개발 자료의 denied 표시는 그대로 남긴다.

즉 **밀리미터 종점 문제와 독립적으로 먼저 할 수 있는 것은 이동 경로/정답/수집 모드 준비와
그에 맞는 새 raw 수집**이다. 현재 코드가 곧바로 trainable 데이터를 만들 수 있다는 보장은 없다.
완료 기준은 새 episode의 검증·명시적 admission이며, 파일 생성 또는 테스트 수가 아니다.

### B. 정지/재출발 16개: TRAIN 12 / DEV 4는 별도 작업

현재 VAL의 정지 유지 0개를 채워야 하므로 이 슬롯을 이동 데이터로 대체하지 않는다.
실측 목표 창·≤0.1 m/s·2초 유지·6.5초 tail 및 기존 물리/미래 기준은 그대로 유지한다.
재출발은 정지 이유와 해제 조건을 원인 label로 기록해야 하며 현재 collector에 완성된
stop→release→restart 프로토콜이 있다고 가정하지 않는다.

v9 피드백은 목표에 더 가까워졌지만 곡률·횡가속도 실패가 늘었다. 이를 기본 수집기로 채택하거나
다른 맵으로 40회 복제하지 않는다. 정지/재출발용 새 controller 후보는 별도로 설계·고정하고,
같은 실패를 확인한 뒤 다음 변경 없이 반복 실행하는 것을 진척으로 세지 않는다.

## 3. 지금 쓸 수 있는 원본과 쓸 수 없는 원본

| 자료 | 허용되는 현재 용도 | 새 40개에 포함? |
| --- | --- | --- |
| 기존 v3 TRAIN 1,147 / VAL 337의 byte-exact 연구 뷰 | 기존 허가 범위의 전체 모델 학습·고정 DEV 평가; 물리/다양성 한계 표시 | 아니오 |
| Town04 TEST 309 및 Town15 | 계속 잠금; 모델 선택·새 개발 수집·payload 열람 금지 | 아니오 |
| 이전 VAD/Autoware 경로·실행 기록 | non-TEST 경로 기하·지도/제어 참고; Portable 운전·새 raw admission으로 승계 불가 | 아니오 |
| 새 stationary/coast/brake/steering/ramp/substep/v9 진단 | 불변 원본을 이용한 원인 분석·실패 증거 | 아니오 |
| 아직 미감사인 기존 raw 폴더 | 전체 출처·원래 denied flag·경로/분할/QA를 확인하기 전까지 미사용 | 아니오 |
| 앞으로 계획대로 수집하고 모든 해당 gate와 별도 admission을 통과한 새 episode | 새 버전 TRAIN 또는 DEV, 사전 배정 그대로 | 예 |

현재 adapter는 `training_data_approved=false` 또는 `development_only=true`를 만나면 변환을
거부한다. flags 삭제·상위 manifest만 새로 작성·성공 구간 잘라내기로 이 금지를 없애지 않는다.
원래 flag가 없다는 사실도 자동 승인으로 보지 않는다.

## 4. 실제 수집부터 학습까지의 인수인계

1. **cohort 사전 선언:** 40개 ID, map/route SHA, 행동, TRAIN/DEV, group/session, 날씨/seed,
   controller·sensor/label 계약, native/camera cadence, 예산과 중단 조건을 고정한다.
   새 DEV를 모델/하이퍼파라미터 선택에 쓴 사실을 기록하고 release TEST와 구분한다.
2. **수집 코드 자격:** 새 moving 모드 또는 새 stop/restart 프로토콜을 CPU/mock 검사하고,
   실제 소수 pilot의 원본 이미지·상태·ACK·모든 미래 창으로 검증한다. 필요한 구현을 마치기 전에
   40회 일괄 실행하지 않는다. 실패/차단과 미실행을 모두 분모에 남기고 자동 retry하지 않는다.
3. **소유된 로컬 실행:** frozen source·route·rig, 20 Hz 상태/제어·10 Hz 실제 6카메라, Epic,
   동일 Prius·고정 wheelbase·기록된 물리 설정. read-only readiness 후 wrapper의 최종 admission을
   따른다. 다른 CARLA를 호출/종료하거나 실행 중 map을 교체하지 않는다.
4. **새 데이터 검사:** source/hash/타임라인·보정/TF·모든 JPEG·ACK·native 운동·기존 미래 기준,
   행동/label masks, route/site/session split 중복을 검사한다. 실패 raw는 그대로 보존한다.
   명시적으로 승인된 새 episode만 새 manifest에 넣고 기존 v3와 과거 결과는 바꾸지 않는다.
5. **학습 인수인계:** 새 데이터 버전/SHA와 고정 DEV 평가를 전달한다. 소수 TRAIN 학습 가능성
   확인→GPU 0·개인 venv의 동일 조건 전체 모델 A/B→직진/좌/우/정지/지속 주행/재출발 subgroup,
   절대 경로·속도 오차와 퇴행을 확인한다. 결과가 없으면 학습 완료/개선이라고 표시하지 않는다.
6. **실행 검증:** 채택 가능한 후보의 export parity→10 Hz shadow→fallback/제어 소유권→제한된
   CARLA learned closed-loop. 이번 데이터 제안으로 0/30 기능 상태나 실차 승인을 바꾸지 않는다.

원격은 physical GPU 0·지정 개인 venv만 사용한다. 설치·Conda/시스템 변경·다른 작업 종료·SSH
Autoware 빌드·TEST 사용은 확대 계획에 포함하지 않는다. 실제 화면은 BasicAgent/VAD/shadow/learned
주체를 구별하고 원본 FOV·시간·ego 중심 경로/과거 궤적을 표시한다. 60 km/h와 real+sim 혼합은
이 첫 데이터 확대 및 안전 검증 이후 별도 단계다.

## 근거와 이번 문서의 범위

- [현재 coverage와 label 공백](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/01_data_readiness/README.md)
- [기존 비실행형 map planner](../tools/e2e/plan_portable_map_coverage.py): 19맵/95칸은 전부 계획·차단 상태.
- [기존 collector](../tools/e2e/collect_carla_vad_expert.py): 종점 완료/시간 초과/종료 제동과 개발 자료 거부 표시.
- [기존 adapter](../tools/e2e/prepare_carla_common10_dataset.py): 명시적으로 거부된 원본의 변환 금지.
- [원본 전체 미래 검사](../tools/e2e/audit_carla_raw_pre_admission.py): 기존 warmup/driving 창도 검증.
- [v9 실패와 모든 원본 분모](assets/validation/2026-09-15/portable_e2e_learning_cycle_v1/02_terminal_approach_feedback/README.md)
- [전체 모델의 최근 실제 학습](assets/validation/2026-09-14/portable_e2e_learning_cycle_v1/05_stopmix_regret_learning/README.md)

문서와 비실행형 planner 결과는 새 split·데이터 admission을 시행하지 않는다. 이 planner 실행은
CARLA·GPU·SSH·설치·TEST payload 접근을 하지 않았으며 기존 데이터도 수정하지 않았다.
후속 capture 구현·실행·admission의 실제 완료 여부는 각각의 새 증거로 보고한다.
