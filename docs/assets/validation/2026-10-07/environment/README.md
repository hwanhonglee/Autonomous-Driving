# 실행 환경: 무엇을 확인했고 무엇을 건드리지 않았는가

<!-- HH_261007 - Explain the actual canonical CPU environment and separate a frozen diagnostic from a reusable launch instruction. -->

이번 환경 검사는 로컬 PC에서 완료했습니다. 설치된 Python과 ROS를 그대로 사용했고, 패키지를 설치하거나 시스템 설정을 바꾸지 않았습니다. **환경 확인, 실제 기록 추론, ROS 송수신 성공은 각각 다른 검사입니다.**

## 확인 완료

일반 터미널에는 다른 Autoware 작업공간의 환경이 섞여 있었습니다. 새 검사에서는 그 환경을 상속하지 않고, 이 프로젝트의 `autonomy/install/local_setup.bash`와 ROS Humble 환경만 사용했습니다.

| 항목 | 실제 확인 |
| --- | --- |
| Python | `/usr/bin/python3`, 3.10.12 |
| Torch / NumPy | 기존 1.8.0a0 / 1.21.5 |
| Autoware 생성 메시지 | 이 프로젝트의 `autonomy/install`에서 읽음. 정상 빌드 링크의 실제 대상은 `autonomy/build` |
| 기본 메시지 검사 | Trajectory·SteeringReport·카메라·위치·가속·TF·시계 등 10종의 직렬화와 복원 일치 |
| 이 환경 검사에서 모델 계산 / 학습 / ROS 노드 / GPU | 모두 0 |

[실제 검사 기록](../../../../../storage/experiments/diagnostics/2026-10-07/nontraining_cpu_ros_environment_v1/result.json)과 [검사 코드](../../../../../tools/e2e/check_nontraining_cpu_ros_environment_20261007_v1.py)를 함께 보존합니다. 이것은 실제 주행 입력의 성공이나 ROS 메시지 지연 검사까지 끝냈다는 뜻은 아닙니다.

<!-- HH_261007 - Separate the no-model environment probe from later genuine CPU inference and privately compiled DDS compatibility code. -->

이후 기록 재생에서는 같은 격리 환경에서 기존 GOOD 가중치를 읽고 실제 모델을 계산합니다. 직접 계산과 ROS 경유 모두 CPU 4스레드·interop 1스레드이며 CUDA를 사용하지 않습니다. 이는 가중치를 업데이트하는 학습이나 현재 CARLA·차량을 움직이는 시험이 아닙니다. 최신 실행 결과와 실패 범위는 [주행·ROS 안내](../driving_ros/README.md)에서 별도로 확인합니다.

설치된 ROS Python 실행기에서 빠진 실제 발신자 정보를 읽기 위해 프로젝트 전용 native 호환 모듈을 만들었습니다. 공개 API로 실제 수신 handle의 발신자 근거를 확인한 [모델 없는 실제 6건 송수신 결과](../../../../../storage/experiments/diagnostics/2026-10-07/owned_dds_info_bridge_v5/result.json)를 보존합니다. 기존 ROS·라이브러리 설치본을 덮어쓰거나 새로운 패키지를 설치한 것은 아닙니다. 작은 6건 성공이 전체 기록의 100ms·10Hz·주행 기능 합격을 뜻하지도 않습니다.

## 실행할 때 주의할 점

다른 작업공간을 `source`한 터미널에서 실행하면 생성 메시지와 라이브러리가 섞일 수 있습니다. 직접 계산과 ROS 경유 비교는 같은 격리 환경·같은 입력·같은 가중치를 사용해야 합니다. 현재 설치된 버전을 과거 시험 당시의 버전이라고 가정하지 않습니다.

새 실행기는 사용자 패키지 경로와 CUDA 환경을 상속하지 않는 깨끗한 환경에서 ROS Humble과 현재 `autonomy/install/local_setup.bash`만 읽습니다. ROS 로그도 해당 실행의 결과 폴더 안에 고정해 출력 예산에 포함합니다. 실행 소유자의 프로세스 세대와 잠금, 허용 노드·topic·실제 메시지 발신자 검사를 유지하며 실제 차량용 actuator topic은 허용하지 않습니다.

이번 날짜별 실행기는 **동일 결과 폴더 덮어쓰기를 거절**합니다. 이미 완료한 실행 명령을 그대로 다시 호출하는 것은 재현이 아닙니다. 새 실행은 새 리비전·새 결과 위치·입력/소스 고정·검토된 실행 선언으로 진행합니다. 이전 실패와 결과를 지워서 재시도하지 않습니다.

실제 차량 명령, 모델 승격, 새 서버 학습은 이번 범위에 없습니다. 준비 검사나 환경 일치만으로 메인9개·하위30개 기능을 합격 처리하지 않습니다. 정식 완료는 **0/9·0/30**이며, 전체 상태는 [오늘의 안내](../README.md)를 보세요.
