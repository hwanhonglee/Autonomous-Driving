# PC1 ros2_ws v1.1 scope

## 확정 배포 단위

```text
working directory: /home/a/ros2_ws
branch: h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1
tag: IONIQ_EV_308_PC1_r
```

공개 저장소에는 PC1 307 `ros2_ws/v1.0` branch가 없다. PC2/PC3 ros2_ws history도 서로 독립이므로 PC1 `_r`은 Autoware commit을 parent로 쓰지 않는 독립 history로 준비한다.

준비 전 `/home/a/ros2_ws`는 존재하지 않았다. 별도 clone은 만들지 않으며, `/home/a/autoware`에서 다음 명령을 한 번 실행하면 directory, source, local commit/tag와 push helper가 함께 준비된다.

```bash
cd /home/a/autoware
bash src/migration_work/releases/PC1_v1.1/PREPARE_IN_PLACE.sh
```

## 포함 구조

```text
/home/a/ros2_ws/
├── src/ros2_socketcan/          # PC1 SocketCAN driver/package tree
├── scripts/start.sh             # can0/can1 500 kbit/s 초기화
├── config/cyclonedds_pc1.xml    # PC1 vehicle-network DDS binding
├── config/pc1_ros_env.sh        # Domain/RMW/DDS/alias environment
├── docs/v1.1/                   # PC1 release audit 문서
└── push_v1_1.sh                 # branch/tag push helper
```

source와 payload 원본:

| 원본 | ros2_ws 위치 | 역할 |
|---|---|---|
| `/home/a/autoware/src/sensor_component/ros2_socketcan` | `src/ros2_socketcan` | CAN receiver/sender/converter package tree |
| `/home/a/autoware/src/migration_work/releases/PC1_v1.1/payload/ros2_ws/scripts/start.sh` | `scripts/start.sh` | repeatable CAN setup |
| `payload/ros2_ws/config/cyclonedds_pc1.xml` | `config/cyclonedds_pc1.xml` | PC1 NIC 고정 |
| `payload/ros2_ws/config/pc1_ros_env.sh` | `config/pc1_ros_env.sh` | portable ROS environment |
| `migration_work/releases/PC1_v1.1/` | `docs/v1.1/` | 변경·문제·검증·rollback 기록 |

`_a` full snapshot에도 같은 ros2_socketcan source가 있다. 두 branch는 원격 보관/배포 단위를 나누기 위한 것이며 실제 PC에서 같은 package 두 사본을 동시에 overlay source하지 않는다.

## flatten/exclusion 정책

- source 내부 nested `.git`/`.gitmodules`는 포함하지 않는다.
- ros2_ws의 generated `build/`, `install/`, `log/`는 포함하지 않는다.
- copied ros2_socketcan tree에 존재하는 backup/pycache는 사용자의 현재-source 보존 정책에 따라 그대로 포함한다.
- 개인 `~/.bashrc` 전체, conda init block, NetworkManager UUID/MAC와 credential은 넣지 않는다. 필요한 export/alias만 portable `config/pc1_ros_env.sh`로 제공한다.
- secret 후보 또는 100 MB 초과 파일이 발견되면 준비/push를 중단한다.

## start.sh 개선

v1.1 payload는 다음 순서를 사용한다.

1. can0/can1 down 시도
2. 양쪽 bitrate 500000 설정
3. can0/can1 up
4. 존재하는 legacy tty/video/docker permission만 처리
5. 성공 메시지 출력

script mode는 `0755`다. 기존 `chmod 777` 정책은 그대로 남은 보안 부채이며 개선으로 주장하지 않는다.

## 환경 export

```bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///absolute/path/to/config/cyclonedds_pc1.xml
```

`pc1_ros_env.sh`는 `BASH_SOURCE` 기준으로 DDS XML을 찾아 `/home/a` 고정경로 의존을 줄인다. 기존 alias 이름 `run_autoware`, `run_planning_universe`, `run_bridge`는 유지한다. `run_bridge`는 물리 CAN TX를 포함하므로 receive-only 명령으로 사용하면 안 된다.

## 검증과 push

```bash
cd /home/a/ros2_ws
git status -sb
git branch --show-current
git tag --points-at HEAD
bash -n scripts/start.sh config/pc1_ros_env.sh push_v1_1.sh

./push_v1_1.sh
```

예상 branch는 `h2_i/IONIQ_EV_308/PC1_rocket/ros2_ws/v1.1`, HEAD tag는 `IONIQ_EV_308_PC1_r`이다. push helper는 force를 사용하지 않으며 HTTPS 인증 prompt에서만 PAT를 받는다.

## 남은 작업

- `chmod 777`을 필요한 device group/udev rule로 대체
- fresh shell에서 alias/env 검증
- clean ros2_ws build/test
- 재부팅 후 NetworkManager NIC binding과 Chrony source 확인
- actual can0/can1, cruise, control command recorded evidence 보존
