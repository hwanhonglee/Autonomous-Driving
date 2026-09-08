# 자체 Portable 모델과 VAD·VisionPilot의 역할 및 비교 계획

<!-- HH_260906 - Distinguish the independent learned planner, the existing VAD reference, and an unexecuted external VisionPilot comparison. -->

확인일: **2026-09-08**. 이 문서는 공식 소스 조사와 로컬 환경의 읽기 전용 확인 결과다.
**VisionPilot 모델 다운로드·추론·학습·빌드·주행 테스트는 아직 하지 않았다.**

## 1. 우리는 새 모델을 만들고 있는 것이 맞다

현재 만드는 **Portable E2E는 자체 설계·학습 코드가 있는 새 모델**이다. VAD를 계속
재학습하는 프로젝트가 아니다. 외부 모델도 비교해 더 나은 구성요소나 전체 방식을
채택할 수 있으며, 그 판단을 위해 기존 결과를 버릴 필요는 없다.

| 이름 | 쉽게 말하면 | 현재 저장소에서의 역할 |
|---|---|---|
| 기존 VAD + Autoware | 이미 연결해 둔 비교 기준 | TensorRT VAD의 궤적을 경로 어댑터·Autoware 제어와 연결한 기존 CARLA 실행 환경 |
| 자체 Portable E2E | 우리가 설계하고 학습하는 새 모델 | 6카메라·보정값·차량 이력·경로를 받아 미래 궤적 후보를 출력하는 PyTorch 연구 모델. 학습·평가 이력은 있지만 학습 모델의 폐루프 차량 제어는 아직 승인되지 않음 |
| VisionPilot | 공식 외부 비교 후보 | 전방 단안 기반 L2 운전자 보조 스택. 아직 이 PC/서버에서 실행하거나 우리 모델과 성능 비교하지 않음 |
| CARLA BasicAgent 원본 수집 | 선생님 역할의 주행 기록 | 최근 출발·정지·회전 품질 검사에 사용한 규칙 기반 expert. 이 촬영을 Portable 모델이 운전한 결과라고 부르면 안 됨 |

자체 모델의 실제 코드는 [model.py](../portable_e2e/model.py), [train.py](../portable_e2e/train.py),
[evaluate.py](../portable_e2e/evaluate.py)에 있다. 기존 VAD 연결은
[carla_vad.launch.xml](../autoware_e2e_vad_launch/launch/carla_vad.launch.xml),
Portable의 비제어 연결은 [shadow node](../autoware_e2e_vad_launch/scripts/portable_e2e_shadow_node.py)다.
최근 학습 실행 이력과 현재 실행 중인지 여부는 [학습 루프 안내](portable-e2e-learning-loop.md)를
따른다. **코드나 과거 checkpoint가 있다는 사실은 지금 GPU에서 학습 중이라는 뜻이 아니다.**

<!-- HH_260906 - Describe the current research baseline without implying scientific novelty or a complete autonomous-driving feature set. -->
현재 모델은 PyTorch로 구현한 작은 영상 인코더와 보정값·차량 이력·경로 결합 구조에서
미래 궤적 후보를 예측하는 **초기 연구 기준선**이다. 기본 설정은 6개 카메라, 10개 과거
차량 상태, 128개 경로점, 100 ms 간격의 미래 64점과 후보 6개다. 가속·제동·조향 명령과
독립 안전 검사는 모델 바깥에 있다. “새 모델”은 자체 코드와 학습 실험이 있다는 뜻이지,
논문 수준의 독창성이나 모든 주행 기능·실차 안전성이 입증됐다는 뜻은 아니다.

## 2. VisionPilot에 실제로 있는 것과 아직 보장하지 않는 것

조사한 VisionPilot 커밋은
[`f9fb99799e779ac602fde06175f2ec3ea3df3f6f`](https://github.com/autowarefoundation/vision_pilot/tree/f9fb99799e779ac602fde06175f2ec3ea3df3f6f)
이다. 이전 `autoware.privately-owned-vehicles` 주소는 현재 `vision_pilot`으로 연결된다.

| 구성 | 공식 구현·학습 자료 | 범위를 과장하면 안 되는 부분 |
|---|---|---|
| AutoSpeed | 주행 경로 안·진입/이탈·밖의 물체 검출. OpenLane 준비·학습·ONNX 변환 코드 | 이 검출기 하나가 모든 장애물 회피 행동을 학습한 것은 아님 |
| AutoSteer 2.0 | 기존 차로를 따라갈 영상 좌표계 경로 예측. OpenLane·TuSimple·CurveLanes 준비 및 학습 코드 | 목적지 지시에 따른 교차로 좌우회전이나 차선변경 계획기와 다름 |
| AutoDrive | 연속 전방 두 프레임으로 앞차 거리·존재 확률·도로 곡률 예측. ZOD 라벨 준비·학습 코드 | 주변 전체 차량의 상호작용과 도심 모든 행동을 출력하는 모델은 아님 |
| VisionPilot 전체 | 위 모델들을 융합하고 종방향 제어·횡방향 MPC와 연결. ROS2·영상·카메라 입력 경로 | 하나의 신경망이 모든 기능을 직접 제어하는 구조는 아님 |

공식 근거와 조사한 모델 저장소 버전:

- [AutoSpeed 설명·가중치 안내](https://github.com/autowarefoundation/auto_speed/tree/b62e55b93ba728787cb1152347463030443634c4), [학습 진입점](https://github.com/autowarefoundation/auto_speed/blob/b62e55b93ba728787cb1152347463030443634c4/Models/training/README.md).
- [AutoSteer 설명·가중치 안내](https://github.com/autowarefoundation/auto_steer/tree/727c8ae62b3e729913bd28596af768697a99c874), [실제 학습 코드](https://github.com/autowarefoundation/auto_steer/blob/727c8ae62b3e729913bd28596af768697a99c874/Models/training/auto_steer_trainer.py).
- [AutoDrive 설명·가중치 안내](https://github.com/autowarefoundation/auto_drive/tree/83cf0fa1a8e8ee760dd854e60ecee8a03ab9072d), [ZOD 준비 코드 설명](https://github.com/autowarefoundation/auto_drive/blob/83cf0fa1a8e8ee760dd854e60ecee8a03ab9072d/Models/data_parsing/zod/README.md), [실제 학습 코드](https://github.com/autowarefoundation/auto_drive/blob/83cf0fa1a8e8ee760dd854e60ecee8a03ab9072d/Models/training/train_auto_drive.py).
- [VisionPilot 모델 융합 코드](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/VisionPilot/modules/models/src/inference.cpp), [실제 planner](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/VisionPilot/modules/safety_guardian/planning/src/planning.cpp).

저장소는 Apache-2.0이며 공식 가중치 안내에는 PyTorch·ONNX FP32/INT8 경로가 있다.
이는 이번에 가중치 파일을 받아 무결성·추론을 검증했다는 뜻이 아니다. 도입 시 해당
가중치와 원본 데이터셋의 이용 조건도 각각 확인한다.

공식 중심 범위는 **차로 내 L2 운전자 보조**다. 우리가 정리한
[9개 상위·30개 하위 기능](portable-e2e-feature-roadmap.md)이 모두 완성되어 있다는 뜻은 아니다.
안전 문서도 **DRAFT**이며 차량 통합·생산 검증·최종 안전 사례 등을 시스템 통합자의
책임으로 구분한다. 이름에 Autoware가 들어간다고 실차 무인주행을 바로 승인할 수 없다.
[공식 안전 범위](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/Functional_Safety/SAFETY_ELEMENT_OUT_OF_CONTEXT.md)

## 3. 섞어서 쓰려면 먼저 맞춰야 하는 부분

- **센서:** VisionPilot 문서의 기준은 전방 1대, 약 50–55° 수평 화각, 1–2MP, 10 Hz다.
  현재 [Portable 센서 설정](../autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml)의
  전방 카메라는 640×360·70°다. crop/resize/TF 변경만으로 원래 없던 원거리 화상 정보를
  복구할 수 없다. 적합한 센서·전처리·보정 조건을 먼저 선언해야 한다.
  [공식 센서·보정 문서](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/github-io/hardware.md)
- **출력:** Portable의 시간별 XY·속도 후보, AutoSteer의 영상 경로, AutoDrive의 거리·곡률은
  서로 다른 의미다. 영상 경로를 지상 좌표로 바꾸고 속도·시간을 붙이는 어댑터의 동작을
  별도로 검증해야 한다. 배열 모양만 맞춰 checkpoint나 ROS topic을 바꾸면 안 된다.
- **ROS2:** VisionPilot 차량 인터페이스는 현재 `Float64` 속도 입력과 조향·가속도 출력이다.
  Autoware trajectory ABI와 같지 않다. 예제 CARLA 브리지의 watchdog 생성은 주석 처리되어
  있으므로 기존의 입력 신선도·출력 권한·비상정지 검증이 자동으로 따라온다고 가정하지 않는다.
  [차량 인터페이스](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/VisionPilot/modules/middleware_interfaces/ros2_interface/vehicle_ros2_interface/src/vehicle_ros2_interface.cpp),
  [예제 CARLA 브리지](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/Simulation/CARLA/ROS2/src/carla_control_publisher/carla_control_publisher/carla_control_publisher_node.py)
- **시뮬레이터:** 공식 안내는 CARLA 0.9.16, 현재 저장소는 0.9.15 기준이다. 버전·브리지 차이는
  검증 대상이지 자동 업그레이드 사유가 아니다.
- **데이터:** 현재 Common10 궤적 라벨을 AutoSpeed의 검출/CIPO 라벨로 바로 사용할 수는 없다.
  실제·가상 데이터를 함께 쓰려면 학습 과제별 라벨·센서 의미·분할을 맞춰야 한다.
  명시적으로 학습 거부된 새 진단 원본을 외부 trainer로 우회 투입하지 않는다.

## 4. 공정하게 비교한 뒤 채택하는 순서

아래는 단계별 진행 상태다. **설치 없는 로컬 사전 검사까지 실행**했으며,
VisionPilot 모델 추론·학습·주행 단계는 아직 **NOT RUN**이다.

| 단계 | 할 일 | 완료 판단 | 현재 상태 |
|---|---|---|---|
| 0. 공식 소스 조사 | 모델·학습 코드·라이선스·입출력·안전 범위 확인 | 조사 커밋과 근거 구분 | 읽기 전용 조사 완료, 실행 검증 아님 |
| 1. 실행 가능성 검토 | 센서 차이·환경 의존성·adapter 계약·승인 범위 확인 | 설치 없이 필요한 항목과 별도 승인 대상을 명시 | [사전 검사 완료 / NOT_READY](assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/26_visionpilot_preflight/README.md) — 선언한 센서/출력과 실제 라이브러리 발견을 구분; adapter는 미구현 |
| 2. 공식 사전학습 모델 오프라인 평가 | 센서 조건이 맞는 영상에 추론, 전방 경로·앞차 예측·실측 지연 확인 | 샘플/가중치/전처리 hash, 전체 실패 분모, p50/p95/p99·10 Hz deadline 기록 | NOT RUN |
| 3. 비제어 shadow 비교 | 별도 topic에 예측·진단만 출력 | 차량 제어 연결 없음, 좌표·단위·시간·입력 신선도 검사 통과 | NOT RUN |
| 4. 공통 ODD 폐루프 비교 | 적합성이 확인된 차로 유지·완만한 곡선·앞차 추종부터 비교 | 동일 시나리오·속도·날씨·seed·실패 기준, 독립 비상정지, 모든 시도 보존 | NOT RUN |
| 5. 채택 결정 | 전체 스택 채택·일부 모델/구조 활용·Portable 유지 중 선택 | 사전 선언한 성능·안전·지연·복잡도 기준으로 결정 | NOT RUN |

ODD는 **테스트할 도로·날씨·속도·교통 상황의 범위**다. 목적지 선택이나 차선변경이 없는
모델을 그 기능을 지원하는 모델과 한 숫자로 줄 세우지 않는다. 우선 모두 수행할 수 있는
과제에서 경로 오차·차로 이탈·충돌·차간거리·정지 성능·지연을 비교한다.

같은 장면이어도 1카메라와 6카메라, 경로 입력 유무가 다르면 **전체 시스템 비교**이며
신경망 구조 하나만 비교한 실험은 아니다. 입력 정보량 차이를 보고서에 명시한다.
오프라인 가중치 평가, 학습 실험, 폐루프 제어 성능도 별도로 기록한다.

최종 선택은 미리 정하지 않는다. 예를 들어 AutoSpeed만 보조 인지로 쓰거나, AutoSteer의
표현을 자체 설계와 비교하거나, 단안 L2 목적에는 VisionPilot 전체를 채택할 수 있다.
새로운 다중 카메라·경로 조건부 행동 모델이 필요하면 Portable 연구를 계속할 수 있다.
어느 경우에도 **공식 모델이라는 이유만으로 승인하거나 자체 모델이라는 이유로 고집하지 않는다.**

## 5. 환경 확인 범위와 설치 제한

<!-- HH_260906 - Link reproducible prerequisite inspection without treating discovery as inference readiness. -->
18:23 KST에는 [사전 검사기](../scripts/e2e/visionpilot_preflight.py)를 실제 실행해
위 단계 1의 초기 결과를 JSON·해시·명령으로 남겼다. 검사 완료와 모델 미실행 상태를 구분한다.
아래 기존 발견과 일치하며, 설정 YAML은 원본 해시만 자동 연결하고 센서 수치는 CLI 선언으로 받는다.
실시간 센서·추론·10 Hz 처리량을 검사했다고 표시하지 않는다.

2026-09-08 로컬 `/usr/bin/python3` 3.10.12에서 모듈 위치 검색과 배포 메타데이터만 읽었다.

| 확인 항목 | 관찰 | 의미 |
|---|---|---|
| `onnxruntime` Python 모듈 | 발견되지 않음 | 해당 interpreter 검색 경로 기준. PC 전체에 없다고 단정하지 않음 |
| `cv2` Python 모듈 | 발견됨 | import·영상 추론은 실행하지 않음 |
| `onnx` 배포 메타데이터 | 1.18.0 | ONNX 파일 형식 도구이며 ONNX Runtime 추론 엔진과 다름 |
| 동적 라이브러리 캐시 | OpenCV core 4.5 계열 발견, ONNX Runtime 항목 미발견 | private SDK·다른 venv·C++ 헤더·ABI 호환성까지 검사한 결과는 아님 |

확인은 `importlib.util.find_spec`, `importlib.metadata.version`, `ldconfig -p`로 수행했다.
CUDA·모델을 import하지 않았고 서버에는 접근하지 않았다. 이 결과만으로 빌드나 10 Hz
추론 가능 여부를 판정할 수 없다.

VisionPilot은 C++/ONNX Runtime 스택이며 CPU/GPU 및 선택적 ROS2 빌드 경로가 있다.
**네이티브 라이브러리 설치는 Python venv 설치와 다르다.** 사용자의 설치 제한을 유지하며
공식 README의 `apt`, `.deb`, Conda base 설치나 CUDA 변경을 그대로 실행하지 않는다.
서버 사용은 기존대로 **GPU 0만**, 패키지 설치는 **hwanhong 개인 폴더의 venv만** 허용된
범위다. 새 의존성·native build·가중치 획득이 필요한 실행 단계는 별도 범위를 확인한다.
[공식 빌드 구성](https://github.com/autowarefoundation/vision_pilot/blob/f9fb99799e779ac602fde06175f2ec3ea3df3f6f/VisionPilot/CMakeLists.txt)
