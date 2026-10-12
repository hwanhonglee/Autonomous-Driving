# NuRec real-to-sim 도입 가능성 및 격리 파일럿 계획

<!-- HH_260906 - Separate neural reconstruction feasibility from the active performance-recovery experiment. -->

확인일: **2026-09-21**  
대상 브랜치/HEAD: `autoware-e2e/v1.9.0-vad-carla` / `3cb676a`  
현재 주 실행계약: **Autoware 1.9.0 + CARLA 0.9.15 + Portable E2E 10 Hz**

## 결론

아이디어 자체는 좋다. 다만 용도를 정확히 구분해야 한다.

**NuRec은 CARLA 합성 영상을 실제 사진처럼 바꾸는 synthetic-to-real 스타일 변환기가 아니다.**
공식 설명에 따르면 NuRec은 실제 카메라·LiDAR 센서 기록을 NCore 형식으로 준비하고,
그 실제 장소를 신경 3D 장면으로 재구성하여 USDZ로 내보낸 뒤 CARLA 또는 AlpaSim에서
새 카메라 위치와 시나리오로 렌더링하는 **real-to-sim** 파이프라인이다.

```text
실차 카메라·LiDAR 로그
  -> 보정·시간·궤적·메타데이터를 포함한 NCore
  -> NuRec auxiliary data
  -> 신경 장면 재구성 학습
  -> USDZ 장면
  -> NuRec gRPC 렌더러
  -> CARLA 0.9.16+의 map/actor/simulation과 결합
  -> Portable E2E용 10 Hz 이미지·상태·경로 표본
```

반대로 **기존 CARLA 합성 프레임의 외관을 실사풍으로 변환**하려는 목적에는 CARLA가 공식
연동한 [NVIDIA Cosmos Transfer](https://carla.readthedocs.io/en/latest/nvidia_cosmos_transfer/)가
더 직접적으로 맞는다. 공식 문서도 이를 CARLA 합성 데이터의 hyper-realistic variation을
만드는 도구로 설명한다. NuRec과 Cosmos Transfer는 서로 대체 관계가 아니다.

이 프로젝트에는 다음처럼 쓰는 것이 가장 안전하고 유용하다.

1. 현재 무너진 경로 선택 성능을 기존 CARLA 데이터와 고정 평가셋으로 먼저 복구한다.
2. 현재 CARLA 0.9.15·Autoware 실행환경은 그대로 보존한다.
3. 별도 CARLA 0.9.16+ NuRec 파일럿에서 공식 사전 복원 장면 **1개만** 재생해 호환성·10 Hz·
   센서 보정·데이터 계약을 검증한다.
4. 통과하면 NuRec 표본을 `실데이터`가 아닌 `real-to-sim`이라는 별도 domain으로 소량 혼합한다.
5. 마지막 판단은 반드시 독립 실차 로그와 실차 shadow/폐쇄 시험장 결과로 한다.

따라서 NuRec은 실차 데이터를 대체하는 방법이 아니라, **CARLA와 실제 환경 사이의 시각적
간격을 줄이고 실제 장소에서 재현 가능한 시뮬레이션 범위를 넓히는 보조 데이터 축**이다.

## 1. 공식 자료로 확인한 NuRec의 실제 역할

- [NVIDIA NuRec 개요](https://developer.nvidia.com/omniverse/nurec)는 실제 센서 데이터를 받아
  3D Gaussian Splatting 기반 OpenUSD 장면을 재구성하고 렌더링하는 기술로 정의한다.
- [AV 시작 가이드](https://docs.nvidia.com/nurec/av/index.html)는 입력을 NCore 형식의 카메라·
  LiDAR 기록으로 준비하고, 실제 데이터를 3D 장면으로 복원하여 USDZ를 생성한 뒤 CARLA나
  AlpaSim에서 렌더링하는 순서를 명시한다.
- [NCore 데이터 준비 문서](https://docs.nvidia.com/nurec/basics/get-data.html)에 따르면 NCore는
  sensor rig, 카메라·LiDAR·radar 구성, annotation, session metadata를 담는 표준 형식이다.
- [NuRec 장면 재구성 문서](https://docs.nvidia.com/nurec/nurec/reconstruct-av-scene.html)는
  실제 센서 데이터로 reconstruction model을 학습하고 `last.usdz`를 주요 산출물로 만든다.
  현재 문서의 산출물 설명에는 XODR drivable map, USDA scene definition, rig trajectory 등이
  포함된다.
- [CARLA 0.9.16 NuRec 가이드](https://carla.readthedocs.io/en/0.9.16/nvidia_nurec/)에서
  CARLA 서버는 map과 actor를 다루고, NuRec 컨테이너는 gRPC 요청에 대한 카메라 영상을
  렌더링한다. 즉 NuRec 자체가 CARLA 물리·교통·행동 모델 전체를 대신하지 않는다.
- [Novel-view 공식 문서](https://docs.nvidia.com/nurec/nurec/novel-view-synthesis.html)는 원본에
  없던 작은 카메라 이동·회전 관점을 만들 수 있다고 설명한다. 다만 촬영 범위에서 너무 멀리
  이동하면 품질이 저하되고, 움직이는 actor 제어는 별도의 gRPC actor-editing 경로가 필요하다.

### NuRec과 사용자가 말한 목적의 차이

| 질문 | NuRec | Cosmos Transfer |
| --- | --- | --- |
| 출발 데이터 | 실제 카메라·LiDAR 로그 | CARLA가 생성한 합성 영상 |
| 핵심 변환 | 실제 장소를 3D 신경 장면으로 복원 | 합성 영상의 시각적 외관을 실사풍으로 변환 |
| CARLA 역할 | 복원 장면의 map·actor·시뮬레이션과 렌더 요청 연결 | 합성 장면·라벨 생성 |
| 우리에게 적합한 용도 | 실제 도로 real-to-sim 재현, 센서 위치 변화, 실제 배경 domain 추가 | 기존 Town/custom map의 날씨·질감 domain augmentation |
| 실제 데이터 대체 여부 | 대체하지 못함 | 대체하지 못함 |

따라서 “CARLA 데이터들을 실제 환경으로 렌더링”이라는 표현이 **합성 프레임을 실사풍으로
바꾸자**는 뜻이라면 Cosmos Transfer가 더 정확하다. “실제 도로를 CARLA에서 다시 달리자”는
뜻이라면 NuRec이 맞다.

## 2. 현재 프로젝트와 바로 결합하지 않는 이유

현재 저장소와 실행 가이드는 CARLA **0.9.15**와 해당 Python 3.10 egg, map hash, bridge 동작을
명시적으로 고정한다. 반면 버전 고정
[CARLA NuRec 가이드](https://carla.readthedocs.io/en/0.9.16/nvidia_nurec/)의 최소 조건은
CARLA **0.9.16 이상**이다. [CARLA 0.9.16 공식 발표](https://carla.org/2025/09/16/release-0.9.16/)도
이 버전에서 NuRec 연동이 추가되었음을 명시한다.

0.9.15 runtime을 제자리 업그레이드하면 다음 기준선이 동시에 바뀐다.

- Python API와 CARLA server 계약
- 현재 사용 중인 Town/custom map cooked asset 및 OpenDRIVE hash
- sensor timestamp와 camera callback 동작
- CARLA bridge와 Autoware launch 동작
- 이미 확보한 0.9.15 학습·검증 결과와의 비교 가능성

그러므로 NuRec은 기존 `CARLA_ROOT`를 덮어쓰거나 기존 `scripts/e2e/env.sh` 기본값을 바꾸지
않는다. 별도 runtime, 별도 Python 환경, 별도 port, 별도 dataset domain, 별도 산출물 루트를
사용해야 한다.

또한 버전 결합을 임의로 섞으면 안 된다.

- CARLA 0.9.16 문서는 `nvidia-nurec-grpc:0.2.0`과 NuRec **25.07** sample scene을 안내한다.
- 최신 NVIDIA 일반 재구성 문서는 `nre-ga:26.04` 예제를 안내한다.
- 이 두 문서가 서로 다른 릴리스 계열을 가리키므로, 최신 컨테이너가 기존 CARLA gRPC 연동과
  호환될 것이라고 추정하지 않는다.

파일럿은 CARLA·NuRec scene·gRPC image·Python API의 정확한 버전과 image digest를 하나의
호환성 행으로 고정한 뒤 시작해야 한다.

## 3. PRO 6000 읽기 전용 준비도 감사

2026-09-21 19:46 KST에 `pro6000-training`에 SSH로 접속하여 명령 존재 여부, OS, GPU,
driver, disk만 읽었다. **설치, Docker pull/run, NGC/Hugging Face login, token·config 조회,
파일 생성, GPU 연산은 하지 않았다.** 두 GPU가 당시 유휴였다는 값은 순간 관측이며 예약을
뜻하지 않는다.

| 항목 | 실제 관측 | NuRec 관점 판정 |
| --- | --- | --- |
| OS / architecture | Ubuntu 24.04, x86_64, kernel 6.8.0-139 | x86_64는 적합. CARLA 0.9.16 NuRec 문서의 Ubuntu 22.04와 달라 별도 호환성 검증 필요 |
| GPU | RTX PRO 6000 Blackwell Server Edition 2장, 각 97,887 MiB | 일반 NuRec 문서의 CUDA GPU 24 GB 초과 조건을 충분히 만족 |
| NVIDIA driver | 580.173.02 | [NuRec hardware 표](https://docs.nvidia.com/nurec/basics/hardware.html)의 Blackwell R580 이상 조건 만족 |
| GPU 0 | UUID `GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5`, 0 MiB, 0% | 사용자가 허용한 GPU. 당시 유휴였을 뿐 NuRec 사용 승인은 아님 |
| GPU 1 | UUID `GPU-19562143-e5e1-f96d-21f9-ccb57d2a2fc6`, 0 MiB, 0% | 사용 금지 유지 |
| compute process | 0개 | 감사 시점 정보만 기록 |
| 개인 workspace disk | 7.3 TB 중 2.9 TB 여유 | 공식 0.9.16 sample 약 200 GB를 담을 물리 여유는 있으나 파생물·checkpoint quota를 별도 산정해야 함 |
| Docker CLI | 없음 | **BLOCKED** |
| NVIDIA Container Toolkit CLI | 없음 | **BLOCKED** |
| NGC CLI | 없음 | 자체 reconstruction을 수행하려면 **BLOCKED** |

[NVIDIA software 요구사항](https://docs.nvidia.com/nurec/basics/software.html)은 Docker 23.0.1
이상, 적합한 NVIDIA driver, NVIDIA Container Toolkit 1.13.5 이상, NGC 인증과 NGC CLI를
요구한다. Docker와 Container Toolkit은 host 수준 구성이라 `hwanhong` 개인 Python venv에
패키지를 넣는 것만으로 해결되지 않는다. 현재 사용자가 정한 “함부로 설치하지 않음” 조건에
따라 아무것도 설치하지 않았다.

**현재 준비도 판정: `NOT_READY_FOR_EXECUTION`.** GPU 성능 부족 때문이 아니라 격리 runtime과
host container 전제, 인증, 버전 호환성 승인이 아직 없기 때문이다.

## 4. 권장 격리 구조

아래 경로는 설계안이며 이번 확인에서 생성하지 않았다.

```text
/home/pro6000/personal/hwanhong/portable_e2e/
  experiments/nurec_pilot/
    runtime_manifest/
    official_sample/
    rendered_raw/
    prepared_nurec_real2sim_v1/
    reports/
```

격리 원칙은 다음과 같다.

- 기존 `venvs/py312`, 현재 Portable 학습 source worktree, checkpoint를 수정하지 않는다.
- CARLA 0.9.16 NuRec client는 공식 조건에 맞는 **별도 Python 3.10 환경**으로 둔다.
- 기존 로컬 CARLA 0.9.15와 별도의 `CARLA_ROOT_NUREC` 개념을 사용한다.
- 기존 0.9.15 server port, map cache, ROS domain, dataset path를 공유하지 않는다.
- GPU 지정은 index `0`이 아니라 승인된 GPU 0 UUID를 사용하고 GPU 1은 노출하지 않는다.
- container tag뿐 아니라 image digest, USDZ hash, CARLA package hash, config hash를 기록한다.
- token은 manifest·로그·Git에 기록하지 않는다.
- installer script가 package 설치와 대용량 download를 함께 수행하므로, 내용을 먼저 검토하고
  승인된 격리환경 외에서는 실행하지 않는다.

## 5. 성능 복구 이후의 단계별 파일럿

### Phase 0 — 현재 모델 성능 기준선 복구

NuRec 도입 전에 기존 CARLA 데이터와 다중 경로 추가 데이터를 비교하는 전체 모델 재학습으로
직진·우회전·combined 성능 복구를 검증한다. selector 재보정만으로 복구되었다고 보지 않는다.
새 시각 domain을 동시에 넣으면 데이터 효과와 기존 모델 오류를 분리하기 어렵기 때문이다.

완료 조건:

- 고정된 Town03 DEV에서 직진·우회전·combined 절대 gate 통과
- 후보 생성 성능과 후보 선택 성능을 별도 보고
- geometry coverage 유지
- TEST 미열람, 새로운 기능 head 추가 없음

이 단계가 실패하면 NuRec으로 가리지 않고 기존 data/sampler/selector 원인을 먼저 해결한다.

### Phase 1 — 공식 사전 복원 장면 1개 replay

실차 로그를 바로 변환하거나 전체 sample collection을 받지 않는다. CARLA 0.9.16 문서가
권장하는 25.07 사전 복원 USDZ 장면 1개로 다음만 확인한다.

- 고정한 CARLA·gRPC image·scene 조합이 실제로 시작되는가
- front/side camera frame과 CARLA frame ID·simulation timestamp가 대응하는가
- 목표 10 Hz에서 누락·중복·역순 frame이 없는가
- p50/p95/p99 render latency와 장시간 queue growth가 허용되는가
- CARLA actor pose와 렌더된 actor/background가 일치하는가
- 종료 후 container, port, GPU process, output ownership이 정상 정리되는가

이 단계는 **환경 smoke test**이며 Portable 모델 학습이나 성능 향상 증거가 아니다.

### Phase 2 — Portable E2E 데이터 계약 adapter

NuRec 출력은 기존 schema에 억지로 넣지 않고 `domain=nurec_real2sim`을 명시한다.

반드시 보존할 항목:

- 원본 real recording ID, NuRec scene ID, USDZ SHA-256
- CARLA/NuRec/gRPC container version과 digest
- 각 camera의 원본·렌더 intrinsics, distortion, extrinsics, shutter model
- CARLA frame, simulation time, render request/response time, source sensor time
- ego state, route, future label의 생성 근거와 좌표계
- scene/site/day/weather/route group 및 split fingerprint
- image bytes SHA-256와 완전한 실패 분모

경로·속도·future trajectory 정답은 이미지에서 추측하지 않는다. CARLA replay와 XODR/actor
state로 인과적으로 생성하고, 렌더 frame과 동일 simulation tick임을 검증한다.

### Phase 3 — 작은 domain-mix A/B 학습

기본 주행 성능이 복구된 checkpoint를 고정 기준선으로 두고, 한 번에 하나의 변수만 바꾼다.

| 실험 | TRAIN 구성 | 목적 |
| --- | --- | --- |
| A | 기존 품질 통과 CARLA 100% | 복구된 기준선 재현 |
| B1 | CARLA 80% + NuRec real-to-sim 20% | 작은 실제 외관 domain 추가 효과 확인 |
| B2 | CARLA 60% + NuRec real-to-sim 40% | B1 통과 시에만 비율 확대 |
| C | B 계열 + 승인된 실제 로그 | 최종 sim/real 혼합학습 후보. 실제 데이터가 생긴 뒤에만 수행 |

비율은 sample 총량이 아니라 **batch/domain sampler가 실제로 본 exposure**로 계산한다. 같은
scene을 novel view로 수백 번 늘려 다른 실제 장소 수가 늘었다고 세지 않는다. map·scene·route·
site·day 단위 split을 먼저 고정하고, 동일 real recording에서 파생된 원본·NuRec·novel view는
서로 다른 split에 배치하지 않는다.

실제 데이터가 충분해져도 CARLA와 NuRec을 모두 버릴 필요는 없다. 권장 구성은 다음 세 축이다.

1. **CARLA native:** 완전한 제어 가능한 geometry·traffic·행동 다양성
2. **NuRec real-to-sim:** 실제 장소·질감·조명에서의 재현과 작은 viewpoint perturbation
3. **raw real:** 실제 센서 noise, lens, motion blur, exposure, vehicle dynamics를 보존한 최종 근거

NuRec 프레임을 `raw real`로 표시해서는 안 된다.

### Phase 4 — domain별 평가

전체 평균 하나로 합치지 않고 최소 다음을 분리한다.

| 평가층 | 평가 데이터 | 핵심 지표 |
| --- | --- | --- |
| reconstruction | 원본 real view와 held-out view | PSNR 등 공식 출력 지표, 시간적 flicker, edge/actor artifact, failed frame 비율 |
| CARLA native open-loop | 기존 고정 DEV | ADE, FDE, speed error, candidate regret, 직진/좌·우회전/정지 subgroup |
| NuRec open-loop | TRAIN과 다른 scene/route | 같은 trajectory 지표 + frame loss/latency + calibration subgroup |
| raw-real log replay | 학습에 쓰지 않은 site/day/drive | trajectory·speed·stop error, OOD confidence, fallback 비율 |
| Autoware shadow | 10 Hz 비제어 topic | end-to-end p50/p95/p99, stale input, timeout, fallback, 좌표·시간 계약 |
| 제한적 closed-loop | 격리 CARLA/NuRec 장면 | route completion, collision, lane invasion, off-route, red-light, comfort |
| 실차 | 폐쇄 시험장 shadow부터 | 수동 개입, 최소거리, 횡오차, 정지오차, latency, safety-driver 판단 |

NuRec image metric이 좋아도 자율주행 성능이 좋아졌다고 판정하지 않는다. 반대로 aggregate
ADE가 좋아도 raw-real held-out 성능, 10 Hz deadline, fallback이 나빠지면 채택하지 않는다.

## 6. 센서 보정은 TF만 맞춰서는 부족하다

NuRec 도입에서는 센서 위치와 화각이 핵심이다. TF는 3D 위치·자세를 표현하지만 다음을 모두
대신하지 못한다.

- focal length, principal point, resolution과 실제 horizontal/vertical FOV
- radial/tangential 또는 F-Theta distortion
- exposure, gain, white balance, motion blur
- rolling-shutter readout와 camera별 시간 offset
- camera-to-camera 및 camera-to-LiDAR 시간 동기
- 차량 body/IMU/odometry 축과 handedness
- crop, resize, padding 뒤의 유효 intrinsic

CARLA 0.9.16 NuRec 가이드는 custom F-Theta, precise intrinsic, distortion polynomial,
camera transform matrix, rolling shutter 설정을 지원한다고 명시한다. 이것은 보정값을 대충
바꿔도 된다는 뜻이 아니라, **실차 rig 값을 정확히 기록하고 재현할 수 있는 인터페이스가
있다**는 뜻이다.

현재 Portable E2E의 6카메라 순서와 10 Hz 계약을 유지하려면 각 camera에 대해 다음 gate를
먼저 통과해야 한다.

- K/D/extrinsic matrix와 단위·축 정의가 원본 calibration과 일치
- 원본 실차 rig, NuRec render rig, 모델 전처리 후 effective K의 변환 이력 보존
- camera별 timestamp skew와 frame age 계측
- ego mask/차체 노출 영역 일치
- 작은 translation/rotation sweep에서 품질이 단조롭게 무너지지 않는지 확인

공식 novel-view 문서도 큰 camera shift가 훈련 데이터 coverage 밖으로 나가 품질을 떨어뜨린다고
경고한다. 센서 위치가 다르면 TF만 이동하고 끝내지 말고 해당 위치가 복원 가능한 범위인지
실측해야 한다.

## 7. 한계와 실패 가능성

<!-- HH_260906 - Require semantic and multi-camera consistency before training on generated appearance variants. -->

Cosmos Transfer를 별도로 시험할 경우에도 실사풍 영상이라는 이유로 기존 정답을 그대로
승인하지 않는다. 차선 경계·신호등 상태·장애물 위치가 변하지 않았는지, 시간축 flicker와
6카메라 사이의 객체 일관성이 유지되는지 확인해야 한다. CARLA 공식 연동의 RGB·depth·
segmentation 등 제어 입력 지원이 이 정합을 자동 보장하지는 않는다. 원본과 변환본은 같은
split으로 묶고, 생성 영상은 오프라인 증강 데이터로만 사용한다. 생성 속도와 최종 Portable
모델의 10 Hz 추론 속도는 별개로 측정한다.

- **실차가 아니다.** 실제 장면에서 시작하지만 최종 프레임은 신경 렌더링 결과다.
- **차량 dynamics를 학습하지 않는다.** CARLA physics와 control/actuation calibration 문제는
  별도로 남는다.
- **촬영하지 않은 영역은 근거가 약하다.** 큰 lateral shift, 가려졌던 물체 뒤, 급격히 다른
  시점은 artifact가 생길 수 있다.
- **dynamic actor는 자동 만능이 아니다.** static novel view와 편집 가능한 actor 경로를
  구분하고, 렌더 영상·CARLA collision geometry·label의 일치를 검증해야 한다.
- **데이터 누출 위험이 크다.** 동일 recording의 원본 frame과 파생 novel view를 TRAIN/DEV로
  나누면 사실상 같은 장소를 양쪽에 넣는 leakage가 된다.
- **센서 realism이 완성되는 것은 아니다.** lens flare, dirt, vibration, dropped frame,
  compression, exposure transition, IMU/GNSS drift 등은 별도 실측이 필요하다.
- **semantic·geometry label을 신뢰하기 전에 검증이 필요하다.** USDZ와 CARLA map/actor state가
  보인다는 이유만으로 모든 frame의 정답 정합이 자동 보장되지는 않는다.
- **runtime 비용이 추가된다.** CARLA와 별도 NuRec gRPC renderer가 함께 동작하므로 현재의
  화면 끊김·10 Hz 문제를 자동 해결하지 않으며, 오히려 latency 측정 대상이 하나 늘어난다.
- **버전 drift가 있다.** CARLA 0.9.16의 권장 25.07 scene/gRPC 계열과 최신 NuRec 26.04
  reconstruction 문서를 임의 혼합하지 않는다.
- **법·개인정보·라이선스가 남는다.** 직접 촬영한 번호판·얼굴·사유지, 공개 dataset과 생성
  artifact의 사용권을 학습 전 확인한다.

## 8. 시작·중단 gate

다음 조건을 모두 만족하기 전에는 NuRec 실행을 시작하지 않는다.

- 현재 Portable E2E 기본 주행 성능 복구 실험이 완료되고 고정 baseline이 생김
- CARLA 0.9.15 운영경로를 건드리지 않는 별도 0.9.16 runtime 위치 승인
- host 관리자가 Docker·NVIDIA Container Toolkit 설치와 Ubuntu 24.04 호환 방식을 승인
- NGC/Hugging Face 계정·약관·token 보관 방식 승인
- GPU 0 UUID 단독 노출, GPU 1 비노출, 기존 작업과의 lease 규칙 확정
- CARLA/NuRec/gRPC/scene 버전 matrix 및 digest 고정
- sample 1개 download 크기와 파생물 disk budget 승인
- 실제 센서 calibration과 데이터 권리 검토
- 기존 TEST split 계속 봉인

다음 중 하나면 파일럿을 중단하고 원인을 기록한다.

- 공식 sample 1개가 고정 matrix에서 재생되지 않음
- 10 Hz에서 frame 누락·중복·역순 또는 지속적인 queue 증가
- camera/actor/route 좌표 정합을 증명할 수 없음
- NuRec 혼합이 CARLA native DEV 또는 raw-real held-out 성능을 악화
- 재현에 token, unpinned image, 덮어쓴 output이 필요
- GPU 1 사용이나 기존 PRO6000 작업 중단이 필요

## 9. 권장 의사결정

현재는 **NuRec 설치·다운로드·학습을 시작하지 않는다.** 성능 복구와 NuRec 파일럿을 동시에
진행하면 원인이 다시 섞이고, 현 서버도 container 실행 전제가 준비되지 않았다.

성능 복구 뒤의 우선순위는 다음과 같다.

1. 승인된 격리환경에서 CARLA 0.9.16 + NuRec 25.07 공식 sample 1개 replay
2. 10 Hz·보정·시간·좌표·실패분모 gate
3. Portable schema용 `nurec_real2sim` adapter와 별도 split
4. CARLA 80% + NuRec 20%의 단일 A/B
5. 독립 raw-real 로그 평가
6. 효과가 있을 때만 자체 실차 로그의 NCore 변환·NuRec reconstruction
7. synthetic-to-real appearance가 별도 필요하면 Cosmos Transfer를 독립 A/B로 평가

이 순서라면 NuRec의 장점을 활용하면서도 현재 성능 문제를 외관 변환으로 가리지 않고,
실차 일반화 여부를 실제 데이터로 끝까지 확인할 수 있다.

## 공식 참고자료

- [NVIDIA Omniverse NuRec 개요](https://developer.nvidia.com/omniverse/nurec)
- [NVIDIA NuRec AV 시작 가이드](https://docs.nvidia.com/nurec/av/index.html)
- [NVIDIA NCore 데이터 준비](https://docs.nvidia.com/nurec/basics/get-data.html)
- [NVIDIA NuRec hardware 요구사항](https://docs.nvidia.com/nurec/basics/hardware.html)
- [NVIDIA NuRec software 요구사항](https://docs.nvidia.com/nurec/basics/software.html)
- [NVIDIA NuRec auxiliary data](https://docs.nvidia.com/nurec/nurec/nurec-aux-data.html)
- [NVIDIA AV scene reconstruction](https://docs.nvidia.com/nurec/nurec/reconstruct-av-scene.html)
- [NVIDIA NuRec novel-view synthesis](https://docs.nvidia.com/nurec/nurec/novel-view-synthesis.html)
- [NVIDIA NuRec gRPC API server](https://docs.nvidia.com/nurec/api/grpc_api_guide.html)
- [CARLA 0.9.16 NuRec integration](https://carla.readthedocs.io/en/0.9.16/nvidia_nurec/)
- [CARLA 0.9.16 release](https://carla.org/2025/09/16/release-0.9.16/)
- [CARLA Cosmos Transfer integration](https://carla.readthedocs.io/en/latest/nvidia_cosmos_transfer/)
