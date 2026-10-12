# 공간·서버·공유 자원

<!-- HH_261007 - Explain exact resource blockers without converting an inaccessible host or unreadable descriptors into a safety proof. -->

<!-- HH_261007 - Use only the latest observed free bytes and authenticated-status receipt, without inferred timestamps or automatic cleanup. -->

**이번 작업에서는 새 학습·GPU 추론·CARLA 주행을 시작하지 않았습니다.** 기존 기록을 사용하는 로컬 CPU/ROS 검사는 별도입니다. 원격 PRO6000의 다른 작업이 실행 중인지, 서버가 켜져 있는지는 접속 실패만으로 판단할 수 없습니다.

## 실행을 막는 것과 막지 않는 것

| 작업 | 현재 조건 |
| --- | --- |
| 소스 검토·작은 검사·기존 결과 분석 | 로컬에서 진행 가능하며 실제 진행 중 |
| 기존 기록의 CPU/ROS 비교 | 별도 CPU 규약: 최소 여유 2GiB + 공유 여유 64MiB + 전체 출력 상한 128MiB. 출력 안에 종료 기록용 16MiB를 포함. 소유 잠금·같은 환경 적용 |
| 기존 계획의 GPU 가속 준비 | 요구 예약 5,972,688,896byte에 미달. 21:47 KST 읽기 확인의 쓰기 가능 공간은 약4.08GB(4,077,441,024byte). 다음 실행 전 다시 확인해야 함 |
| CARLA 새 폐루프 | GPU 준비·공간·독립 시뮬레이터 소유 조건 확인 전 미실행 |
| PRO6000 접속 | 최신 [21:51 KST 읽기 전용 확인](../../../../../storage/experiments/diagnostics/2026-10-07/ssh_readonly_recheck_v4.json)도 인증 전 `No route to host`. 원격 명령에 도달하지 못해 GPU·학습 상태 미확인 |

`df`의 디스크 전체 빈 공간과 일반 사용자가 실제로 쓸 수 있는 공간은 다를 수 있습니다. 판단에는 `Available`과 `statvfs.f_bavail`을 사용합니다. 큰 원본을 같은 디스크의 다른 폴더로 옮기는 것은 공간 확보가 아닙니다.

<!-- HH_261007 - Explain the observed free-versus-writable discrepancy without changing filesystem reservations. -->

[21:47 KST 실제 공간 확인](../../../../../storage/experiments/diagnostics/2026-10-07/local_space_readonly_v3.json)에서는 전체 빈 공간이 **38.38GB**로 보였지만 일반 사용자가 쓸 수 있는 공간은 **4.08GB**였습니다. 차이 **34.30GB**는 이 파일시스템에서 일반 사용자에게 제공되지 않는 빈 공간입니다. 그래서 화면의 약40GB와 실행기에 필요한 쓰기 가능 공간은 서로 다른 수치입니다. [이전 4.14GB 관측](../../../../../storage/experiments/diagnostics/2026-10-07/local_space_readonly_v2.json)도 보존했으며, 예약 설정을 바꾸거나 root 권한으로 우회해 실행하지 않았습니다.

CPU 기록 검사를 별도 작은 예산으로 진행할 수 있다는 뜻이지, 기존 GPU/CARLA 실행의 공간 기준을 낮추거나 공간 정리를 완료했다는 뜻은 아닙니다.

## 정리 상태

Documents·Downloads의 분류 이동과 홈의 [논문 자료 안내](/home/a/thesis_4pc/README.md)는 완료했습니다. CARLA·Unreal·Autoware·RoadRunner·지도·모델·venv와 실패 기록은 보존했습니다.

기존 승인된 `/var/tmp/ccache` 정리는 서버 백업·전량 검증 조건이 충족돼야 제거합니다. 현재 접속이 막혀 해당 백업과 삭제는 하지 않았습니다. 기존 승인은 유지되며 같은 승인을 다시 요구하는 상황이 아닙니다.

추가 후보인 `/home/a/.ros/log/component_container_mt_4904_1744181413078.log` 한 개는 6,502,342,656byte입니다. 압축 보존·전량 복원 해시 검증 도구는 **준비만** 했습니다. 이 파일에 대한 사용자 답변이 아직 없고 최종 비사용도 확인되지 않아 보존·정리는 실행하지 않았습니다. 일반 권한으로 읽을 수 없는 다른 프로세스의 파일 사용 여부는 확인된 것으로 처리하지 않습니다. 휴지통도 수정 소스와 실패 자료가 있어 일괄 비우지 않습니다.

## 다른 테스트와 겹치지 않는 방법

ROS 기록 검사는 프로젝트의 runtime 잠금을 확보하고 localhost 전용 Domain 93과 허용된 topic·노드만 사용합니다. 다른 테스트의 CARLA에 붙거나 맵을 바꾸거나 프로세스를 종료하지 않습니다.

V10에서는 수신 묶음의 provenance, 실제 모델 실행 직전, 출력·ACK 발행 때 전체 그래프를 검사합니다. 메시지마다 소유 프로세스 세대·실제 발신자 GID·메시지형·내용 검증은 유지합니다. **매 메시지마다 전체 그래프를 조사하거나 검사 사이에도 외부 노드가 없었다고 증명하는 것은 아닙니다.** 발견된 비허용·중복 노드나 topic은 모델 실행 또는 발행 전에 거절하며, 이를 DDS 암호 인증이나 실제 차량 구동의 승인으로 표현하지 않습니다.

서버 학습을 다시 허용받아 진행할 때도 `/home/pro6000/personal/hwanhong` 아래 개인 venv와 물리 GPU 0만 사용합니다. GPU 1·시스템 설치·재부팅·원격 Autoware 빌드는 범위 밖입니다.
