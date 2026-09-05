# Portable E2E Common10 30 kph 학습·검증 보고서

> 기준일: 2026-09-05
> 데이터셋: `carla-common10-30kph-town07-ctrack-town03-20260905-v2`

## 결론

CARLA BasicAgent expert로 수집한 직진 1개와 회전 2개 episode를
`common_10hz_v1`로 변환해 **3 episode, 950 sample 데이터 검증 PASS**를 확인했다.
Town07 직진 309개와 CTrack 좌회전 304개, 합계 613개를 학습에 사용하고, 학습에
포함하지 않은 Town03 우회전 337개 전체를 validation으로 사용했다. 지정한 GPU0에서만
baseline을 1 epoch, 154 optimizer step 학습했고 Town03 337개 전체의 open-loop 평가를
완료했다.

이 결과는 데이터 파이프라인과 첫 연구용 baseline이 실제로 끝까지 동작한다는 증거다.
그러나 6.4초 ADE `8.919373 m`, FDE `19.348828 m`인 **open-loop 초기 결과**이며,
CARLA closed-loop, Autoware runtime inference, 장애물 회피·차선 변경, 실차 제어를
입증하지 않았다. 체크포인트는 차량 제어용으로 승인되지 않았다.

| 항목 | 판정 | 이번 판정이 뜻하는 것 |
|---|---|---|
| CARLA expert 수집 | **PASS 3/3** | Town07 직진, CTrack 좌회전, Town03 우회전이 goal 도달·충돌 0·차선침범 0 |
| Common10 데이터 검증 | **PASS** | 950개 sample의 schema, native 10 Hz cadence, 6-camera bundle, 6.4초 future와 split 계약 통과 |
| 로컬↔학습 환경 전송 | **PASS** | 양쪽 prepared tree의 파일 수·byte·manifest SHA-256 일치 |
| CPU 1-step | **PASS** | 학습 control flow가 1 step 전진함. 모델 품질 판정은 아님 |
| GPU 학습 | **COMPLETE** | GPU0만 사용, train 613개, 1 epoch, 154 optimizer step |
| Town03 validation | **OPEN-LOOP COMPLETE** | 학습에서 제외한 우회전 337개 전체 평가 |
| 이번 모델의 Autoware runtime | **NOT RUN** | 과거 Autoware 화면과 이번 checkpoint는 연결되지 않음 |
| 전체 Town Common10 | **PENDING** | 과거 9-map Autoware 주행과 별개로, 새 10 Hz 학습 corpus는 아직 3개 장면뿐 |
| CARLA closed-loop / 실차 | **NOT APPROVED** | safety selector·runtime latency·폐쇄 시험장 검증 전에는 actuator에 연결 금지 |

정리된 PNG·GIF·수치·무결성 파일은
[2026-09-05 발행 증거 묶음](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/)에서
한 번에 확인한다.

## 환경별로 실제 수행한 일

### 로컬 개발·CARLA PC

- CARLA를 map별로 cold start하고, 20 Hz physics / native six-camera 10 Hz로 expert
  episode를 수집했다.
- route를 검증된 map 좌표에 정렬하고 goal, collision, lane invasion, cadence,
  6.4초 future label gate를 fail-closed로 검사했다.
- raw episode를 `common_10hz_v1` prepared corpus로 변환하고 dataset validator와
  read-only tree manifest를 실행했다.
- 차량을 중앙에 두고 여섯 camera와 기준 경로·주행 궤적을 함께 보여 주는 PNG/GIF를
  생성했다.
- 이번 최종 GPU 학습은 로컬 PC에서 하지 않았다.

### 격리된 원격 학습 환경

- 프로젝트 개인 Python 가상환경만 사용했고 시스템 Python, Conda base, driver 및
  다른 프로젝트 환경은 수정하지 않았다.
- 전송된 prepared corpus를 다시 검증하고 로컬과 동일한 tree manifest인지 확인했다.
- 먼저 CPU 1-step smoke를 통과시킨 뒤, 허용된 GPU0만 노출해 1 epoch를 학습했다.
- checkpoint를 학습에 사용하지 않은 Town03 `val` 337개 전체에 평가했다.
- 다른 GPU의 작업을 종료·reset하거나 재부팅하지 않았다.

### Git 저장소

Git에는 코드, 계약, 문서와 검토용 정리 증거만 둔다. 약 598 MB prepared dataset,
raw camera frame, 개인 가상환경과 checkpoint 원본은 Git 배포 대상이 아니다. 새 환경에서의
역할 분리와 명령은 [Portable E2E 학습·운용 가이드](portable-e2e-training.md)를 따른다.

## Common10 corpus

세 episode는 target profile `30 kph`, `ClearNoon`, seed `0000`, 20 Hz physics와
10 Hz six-camera라는 공통 수집 계약을 사용했다. 여기서 `30 kph`는 목표 profile의
이름이며 모든 순간에 실제 속도 30 kph를 유지했다는 뜻은 아니다. 정지 warm-up과
6.5초 stationary tail은 원본에 보존하되 학습 anchor에서는 제외했다.

| Split | Map / 장면 | 방향 | Sample | 유효 구간 | 이동거리 | 유효 camera rate |
|---|---|---|---:|---:|---:|---:|
| `train` | Town07 / straight | 직진 | 309 | 30.8 s | 208.444 m | 10.000 Hz |
| `train` | CTrack / turn | 좌회전 | 304 | 30.3 s | 204.551 m | 10.000 Hz |
| `val` | Town03 / turn | 우회전 | 337 | 33.6 s | 206.670 m | 10.000 Hz |
| 합계 | 3 episode | 직진 1 + 회전 2 | **950** | 94.7 s | 619.666 m | native 10 Hz |

각 episode는 goal reached, collision event 0, lane-invasion event 0이다. 선택된 950개
bundle은 camera coverage 100%, 최대 camera skew 0 ms, 최대 state delta 0 ms,
6.4초 future point 100%였다. `train`과 `val`은 episode 단위로 분리했으며 Town03
episode는 학습 sample에 섞지 않았다. 다만 map 하나뿐인 `val`은 최종 제품 성능을
대표하는 `test` set이 아니다.

### 회전 제어 A/B 반영

CTrack의 기존 A 설정은 goal에는 도달했지만 lane-invasion event 4건이 있어 학습 자료로
채택하지 않았다. B 설정은 BasicAgent waypoint purge lookahead를
`2.0 m + 0.2 s × speed(m/s)`로 줄였고, 횡제어 PID
`Kp=1.95, Ki=0.05, Kd=0.20`, 최대 steering `0.8`, lane offset `0.0 m`는 유지했다.
최종 CTrack 좌회전과 Town03 우회전은 이 B 설정에서 collision 0, lane invasion 0으로
통과했다. 외부 CARLA PythonAPI source는 수정하지 않았으며 실제 주입 option과 upstream
source hash는 episode provenance에 기록했다.

Town03 첫 시도는 camera 10 Hz나 PC 부하 때문에 멈춘 것이 아니다. 50초 driving budget
중 약 47.5초를 출발 직후 교통신호 대기에 사용해 제한시간을 넘었다. 이 시도는
`DIAGNOSTIC_NOT_QUALIFIED`로 제외했다. 신호를 무시하거나 PID를 바꾸지 않고 total budget만
120초로 늘린 새 시도는 driving 30.2초 뒤 goal에 도달해 최종 `val` episode가 됐다.

이 A/B는 **expert 데이터 수집 controller의 비교**다. 이번 학습 모델 자체를 A/B
비교했거나, 모델이 closed-loop로 이 경로를 주행했다는 뜻은 아니다.

## 데이터 무결성과 재현 식별자

| 식별자 | 값 |
|---|---|
| Validation | **PASS** |
| Dataset fingerprint SHA-256 | `17c248440efca864e6c322ca5a1602d08cd1a0eabfa71e10545049181a86e073` |
| Dataset manifest SHA-256 | `fdf838fcde36b7cd4259b46c9695088b77fe3dacb595c4c55c8f9915261ac3c5` |
| Common10 contract SHA-256 | `eb8f44c98a5bee6e560f4983c6ecbc67122c4fe043d4d50fe2ea3737bda9c722` |
| Prepared tree | 5,718 files, 27 directories, 597,635,140 bytes |
| Prepared tree manifest SHA-256 | `81158416448d399cfa877765de121b3e30c84fbfdc504562ed32ddc6ecfa59bb` |
| Published evidence `SHA256SUMS` SHA-256 | `ef3272e8af75eced027a943e4db46897bd46c3df5c0be64355509bae65339db2` |

로컬과 학습 환경의 tree manifest는 위 파일 수, 총 byte와 manifest SHA-256이 모두
일치했고 checksum 기반 rsync dry-run 차이도 0건이었다. 이 값은 동일 dataset을
평가했는지 확인하는 재현 식별자이지, 모델 안전성 인증값은 아니다. Validator에는
`manual_release_review_required=true`가 남아 있다. 기계 판독 원본은
[데이터 검증 요약](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/validation_summary.json)에
있다.

## 학습 결과

CPU 1-step은 dataset decode, forward, loss, backward, optimizer와 checkpoint 경로가
한 번 완주하는지를 보는 smoke다. 그 다음 지정된 GPU0에서 train 613개를 1 epoch,
154 optimizer step 학습했다. `portable_e2e.perspective_trajectory.v0` baseline은
1,053,278 parameter이고 batch 4, seed `20260903`, learning rate `1e-4`를 사용했다.
613개를 replacement 없이 한 번씩 사용했으며 버린 sample은 0개다. 이는 작은 세 장면
corpus의 첫 baseline으로, 여러 seed, architecture 또는 hyperparameter 간 우열을
판단하는 model A/B는 아직 아니다.

154 step의 clipping 전 gradient norm은 최소 `63.078`, 최대 `299.104`였고 모든
step에서 설정한 clip threshold `5.0`을 넘었다. 실제 update에는 clipping이 적용되어
학습 자체는 정상 종료했지만, 다음 A/B에서 learning rate·loss scale·gradient 분포를
반드시 점검해야 하는 신호다.

현재 모델은 여섯 camera, calibration, ego history와 route를 입력받아 6.4초짜리 후보
trajectory를 출력한다. throttle, brake, steering을 직접 출력하지 않으며 이번 실행에서는
Autoware control command로 변환하지 않았다.

## Town03 전체 open-loop 평가

학습에서 제외한 Town03 우회전 `val` 337개 모두를 평가했다. 아래 ADE/FDE는 모델이
점수로 선택한 trajectory와 정답 future trajectory의 2D 거리이며 단위는 meter다.

| 지표 | 결과 |
|---|---:|
| ADE @ 1.0 s | 1.432419 m |
| ADE @ 3.0 s | 3.778989 m |
| ADE @ 6.4 s | 8.919373 m |
| FDE @ 6.4 s | 19.348828 m |
| 평가 sample | 337 / 337 |
| model forward | 0.791326 ms/sample |

`0.791326 ms/sample`은 warm-up 뒤 GPU model forward 구간만 동기화해 나눈 값이다.
JPEG 읽기·decode, 전처리, ROS message, TF, route 구성, safety selector, 제어기,
CARLA/실차 sensor-to-actuator 지연을 포함하지 않는다. 따라서 이 숫자를 곧바로 실제
10 Hz runtime 합격이나 전체 시스템 FPS로 해석하면 안 된다. 6.4초 오차도 현재
checkpoint를 차량 제어에 사용하기에는 크므로 `vehicle_control_approved=false`가 맞다.
checkpoint 및 평가 수치의 기계 판독 요약은
[학습·평가 요약 JSON](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/reports/training_evaluation_summary.json)에
있다.

검토용으로 전체 337개를 수치 평가했고, 대표 12개 trajectory PNG를
[open-loop 화면 폴더](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/model_open_loop/town03_val/)에
발행했다. PNG는 candidate, score-selected 경로와 정답 경로의 offline plot이며 CARLA가
이 예측을 따라 주행한 영상이 아니다.

## 화면 증거와 서로 다른 의미

### 이번 CARLA expert / Common10 화면

아래 화면은 이번 10 Hz raw expert episode에서 만든 차량 중심 시각화다. 여섯 camera,
ego, 전체 기준 경로와 실제 expert 궤적을 같은 화면에 표시한다.

| 장면 | 차량 중심 PNG | 주행 GIF |
|---|---|---|
| Town07 직진 | [PNG](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/town07_straight/centered_overview.png) | [GIF](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/town07_straight/centered_drive.gif) |
| CTrack 좌회전 | [PNG](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/c_track_left_turn/centered_overview.png) | [GIF](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/c_track_left_turn/centered_drive.gif) |
| Town03 우회전 | [PNG](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/town03_right_turn/centered_overview.png) | [GIF](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/expert_data/town03_right_turn/centered_drive.gif) |

### 과거 Autoware runtime 참고 화면

같은 세 종류 장면을 이해하기 쉽도록 과거 Autoware VAD + CARLA 주행의 차량 중심 RViz,
경로·제어·조향·속도 자료도
[historical 폴더](assets/validation/2026-09-05-portable-e2e-common10-30kph-v2/autoware_runtime_historical/)에
모았다. 원본 판정은
[2026-09-02 runtime·제어 보고서](validation-2026-09-02-runtime-control-campaign.md)에 있다.

이 화면은 이번 Common10 expert 수집 또는 새 checkpoint 실행 중 캡처한 것이 아니다.
따라서 “이번 학습 모델이 Autoware에서 주행했다”는 증거로 사용하면 안 된다. 이번 모델의
Autoware 화면은 runtime adapter와 safety gate를 구현하고 동일 checkpoint를 closed-loop로
재실행한 뒤 별도 evidence로 만들어야 한다.

## 아직 남은 순서

1. 실행 가능한 모든 Town과 custom map에 대해 새 native 10 Hz Common10 직진·회전을
   같은 fail-closed 기준으로 확장한다. 과거 5 Hz Autoware 9-map/18-trial PASS 자료를
   10 Hz 학습 sample로 이름만 바꾸거나 복제하지 않는다.
2. 날씨, seed, 교통량, 정지선, 선행차, 보행자, 장애물을 늘리고 episode 단위
   train/val/test split과 map holdout을 만든다.
3. 동일 split에서 여러 baseline·seed를 비교하고 collision surrogate, 차선·정지·ACC,
   차선변경·회피용 label/head와 평가 기준을 단계적으로 추가한다.
4. Autoware inference adapter와 독립 safety selector를 구현하고 target PC에서
   sensor-to-trajectory 및 sensor-to-actuator 10 Hz latency를 측정한다.
5. CARLA closed-loop에서 직진·회전부터 시작해 정지, ACC, 회피, 차선변경 순으로
   통과시킨다. 그 뒤 실제 데이터 replay와 shadow mode, 폐쇄 시험장을 거친다.
6. 60 kph는 30 kph closed-loop 기준선을 먼저 만든 뒤 정지거리, 경로 길이, 곡률,
   횡가속도, actuator saturation과 emergency fallback 계약을 별도 캠페인으로 검증한다.

실제 차량이나 공도에서는 이 보고서의 checkpoint를 바로 실행하지 않는다. 센서 위치·화각,
calibration/TF, 시간 동기, 차량 동역학과 actuation map이 실제 장착 구성에서 다시 검증되고,
독립 안전 계층과 단계별 시험 승인이 끝나야 한다.
