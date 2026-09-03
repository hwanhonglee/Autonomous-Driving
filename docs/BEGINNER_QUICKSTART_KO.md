# Autoware E2E 초보자 설치·실행 가이드

이 문서는 터미널과 ROS를 처음 쓰는 사람도
`git clone -> 환경 진단 -> source 준비 -> CARLA/모델/맵 준비 -> build -> 직진/회전 주행`
순서로 진행할 수 있도록 작성한 입문 문서다.

가장 먼저 알아야 할 결론은 다음과 같다.

- **clone 직후 바로 실행할 수 있는 것은 읽기 전용 환경 진단과 Git에 발행된 문서다.**
- Autoware source, CARLA 실행 파일, VAD ONNX, TensorRT engine, Lanelet2/PCD 맵,
  build 결과와 raw 실험 artifact는 Git에 들어 있지 않다.
- 따라서 “어떤 PC에서든 clone 한 번으로 즉시 자율주행”은 현재 구조에서는 불가능하다.
- 이 branch에서 검증한 full stack 기준 환경은 **native Ubuntu 22.04 x86_64,
  ROS 2 Humble, NVIDIA GPU/driver, CARLA 0.9.15**다.
- Windows와 macOS에서는 문서를 볼 수 있고 POSIX 호환 Bash 환경이 있을 때만
  preflight를 실행할 수 있다. WSL, ARM, 일반 Docker image를 포함해 어느 쪽도
  동일한 CARLA/VAD 폐루프 실행 환경으로 검증되지 않았다.

이 문서의 모든 설치 명령은 사용자가 직접 읽고 실행하는 명령이다.
clone 직후 실행하는 `bootstrap_preflight.sh`는 `sudo`, 설치, 다운로드, 빌드,
파일 생성, 프로세스 종료를 전혀 수행하지 않는다.

## 1. 용어부터 이해하기

| 용어 | 이 프로젝트에서 뜻하는 것 |
|---|---|
| repository 또는 repo | Git으로 관리하는 이 프로젝트 폴더 |
| workspace | repo와 그 아래 `src/`, `build/`, `install/`, `data/`를 합친 작업 공간 |
| terminal | 명령을 입력하는 창. 긴 실행은 Terminal 1/2처럼 별도 창으로 나눈다. |
| ROS 2 | Autoware의 node와 topic을 연결하는 middleware 환경 |
| CARLA server | 가상 세계, 차량 물리와 카메라를 계산하는 simulator process |
| Autoware stack | 센서 bridge, VAD, route manager, 제어기와 RViz를 실행한 ROS process 묶음 |
| route JSON | 시작점, 목표점, 경로점과 회전 command가 들어 있는 episode 입력 |
| full-map bundle | Lanelet2 OSM, PCD, projector metadata와 provenance manifest의 묶음 |
| artifact | 한 시험의 JSON, rosbag, PNG, GIF, log와 설정 사본 |
| PASS/BLOCK/HOLD | 각각 계약 통과, prerequisite 부재, 개선 후보 보류를 뜻한다. |

명령 예시의 `$` 문자는 shell prompt를 뜻한다. 복사할 때 `$`는 입력하지 않는다.
`/absolute/path/...`는 예시이므로 반드시 자신의 실제 절대 경로로 바꾼다.

## 2. 지원 환경과 현실적인 하드웨어 범위

### 2.1 검증된 기준

| 항목 | 기준 |
|---|---|
| OS | native Ubuntu 22.04 LTS |
| CPU architecture | x86_64/amd64 |
| ROS | ROS 2 Humble, 기본 경로 `/opt/ros/humble` |
| simulator | packaged CARLA 0.9.15 Linux build |
| Python API | `carla-0.9.15-py3.10-linux-x86_64.egg` |
| GPU | NVIDIA. RTX 3060 12 GB에서 느린 역사 실행, RTX 3090 Ti 24 GB에서 현재 campaign 검증 |
| project CUDA | workspace-local CUDA 12.8 toolchain |
| project TensorRT | workspace-local TensorRT 10.8 |
| DDS | CycloneDDS, 기본 ROS domain 42 |
| display | native X11 desktop에서 RViz owned-window capture 검증. CARLA off-screen도 정상 NVIDIA Vulkan/OpenGL이 필요 |

운영 여유로 logical CPU 8개 이상, RAM 32 GiB 이상, **설치 전 빈 디스크 100 GiB
이상**을 권장한다. 현재 한 검증 환경에서는 CARLA가 약 38 GiB, project `data/`가
약 15 GiB, `build/install/log`가 수 GiB였다. 맵과 장기 rosbag을 더 모으면 훨씬
커질 수 있다. 이는 보장 사양이 아니라 실제 설치 규모를 바탕으로 한 시작점이다.

공식 Autoware source 설치 문서도 Ubuntu 22.04와 ROS 2 Humble을 prerequisite로
안내한다.

- [Autoware 공식 source installation](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/installation/autoware/source-installation.md)
- [Autoware 1.9.0 release](https://github.com/autowarefoundation/autoware/releases/tag/1.9.0)
- [CARLA 0.9.15 download](https://carla.readthedocs.io/en/0.9.15/download/)
- [CARLA package quick start](https://carla.readthedocs.io/en/latest/start_quickstart/)

### 2.2 현재 지원하지 않는 환경

- **Windows/macOS:** CARLA 자체 지원 여부와 별개로 이 repo의 Bash wrapper,
  ROS Humble 경로, Linux CARLA egg와 NVIDIA local bundle 계약을 그대로 실행할 수 없다.
- **WSL2:** GPU 전달과 GUI를 구성할 수 있더라도 이 campaign의 CARLA/Vulkan,
  process-group cleanup과 RViz 캡처 계약으로 검증하지 않았다.
- **ARM/aarch64:** `arm64.env`가 있어도 pinned spconv와 CARLA Python egg가 amd64
  전용이므로 현재 full VAD 경로는 지원하지 않는다.
- **Docker/container:** 이 branch에는 project 전용 Dockerfile, compose,
  NVIDIA runtime/GUI/CARLA asset mount를 함께 검증한 구성이 없다. 임의 container에서
  실행됐다는 이유로 native 결과와 동일하다고 간주하지 않는다.
- **CPU-only:** 문서와 일부 분석 script는 사용할 수 있지만 TensorRT VAD와 CARLA
  six-camera 주행은 지원 대상이 아니다.

다른 환경을 지원하려면 단순 경로 수정이 아니라 CARLA binary/API, NVIDIA runtime,
ROS DDS, GUI, shared memory, port, cleanup과 전체 폐루프 회귀를 새로 검증해야 한다.

## 3. Git clone 직후 첫 명령

Ubuntu Terminal을 열고 Git과 Git LFS가 없다면 먼저 설치한다. 이 명령은 system
package를 변경하므로 내용을 확인한 뒤 직접 실행한다.

```bash
sudo apt-get update
sudo apt-get install git git-lfs
```

LFS object가 600개 이상이고 과거 MCAP에는 수백 MB짜리 파일도 있으므로, 처음에는
LFS 자동 다운로드를 생략하고 project branch만 clone하는 경량 경로를 권장한다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone \
  --branch autoware-e2e/v1.9.0-vad-carla \
  --single-branch \
  https://github.com/hwanhonglee/Autonomous-Driving.git
cd Autonomous-Driving
```

발행 PNG/GIF와 일부 MCAP은 Git LFS object다. 우선 최신 runtime-control campaign의
화면 자료만 선택해서 받는다(현재 약 129 MB).

```bash
git lfs install
git lfs pull \
  --include='docs/assets/validation/2026-09-02-runtime-control-campaign-v1/**' \
  --exclude=''
git lfs status
```

용량을 확인했고 모든 LFS object가 꼭 필요할 때만 include 없이 `git lfs pull`을
사용한다. validation 화면 전체(현재 약 1.3 GB)가 필요하되 과거 대형 MCAP은 제외하려면
다음처럼 범위를 넓힌다.

```bash
git lfs pull --include='docs/assets/validation/**' --exclude=''
```

이미 일반 `git clone`을 했지만 LFS client가 없어서 pointer만 남은 경우에도 Git LFS를
설치한 뒤 위 선택 pull 중 하나를 그대로 실행하면 복구된다. 일반 clone도 동작하지만
Git LFS가 미리 활성화돼 있으면 clone 도중 큰 object를 전부 받을 수 있다.

이제 어떤 dependency도 설치하지 않는 첫 진단을 실행한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh
```

출력 의미는 다음과 같다.

- `[PASS]`: 해당 prerequisite가 발견됐다.
- `[WARN]`: 실행을 반드시 막지는 않지만 재현성 또는 성능 문제가 있다.
- `[BLOCK]`: full stack을 지금 실행하면 안 된다.
- `status: READY_FOR_DOCTOR`: 허용된 map 상태와 기본 runtime 파일/metadata가
  발견됐다. ABI, GPU engine load, ROS graph나 실제 주행 PASS를 뜻하지는 않는다.
- `mutation: NONE`: 진단이 system과 workspace를 변경하지 않았다는 뜻이다.

기본 모드는 blocker가 있어도 전체 결과와 다음 명령을 끝까지 보여주고 종료 코드 0을
반환한다. CI나 최종 확인에서는 엄격 모드를 사용한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh --strict
echo $?
```

`echo $?`가 `0`이면 blocker가 없고, `1`이면 하나 이상 남아 있다. 잘못된 option은
종료 코드 `2`다. 특정 외부 경로를 검사하려면 다음처럼 지정한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh \
  --carla-root /absolute/path/to/CARLA_0.9.15 \
  --map /absolute/path/to/Town01_full
```

## 4. clone에 포함되는 것과 포함되지 않는 것

| 항목 | clone 포함 | 준비 방법 |
|---|---:|---|
| project launch/config/scripts/tests/patches | 예 | Git checkout |
| 발행된 validation PNG/GIF | LFS | `git lfs pull` |
| Autoware repository manifest | 예 | `autoware.repos` |
| Autoware source `src/` | 아니오 | `vcs import` |
| ROS/CMake/system dependency | 아니오 | 호스트 준비 + `rosdep` |
| CARLA 0.9.15 server와 Python egg | 아니오 | 별도 package/runtime 확보 |
| 공식 VAD v0.1 ONNX | 아니오 | project download script |
| GPU별 TensorRT `.engine` cache | 아니오 | 첫 실행 시 생성 |
| CUDA/TensorRT/spconv/acados local prefix | 아니오 | build wrapper가 준비 |
| Lanelet2/PCD full-map source와 bundle | 아니오 | 별도 source 확보 후 setup |
| custom cooked map(C-track/월악산) | 아니오 | 별도 배포/권한/해시 계약 필요 |
| `build/`, `install/`, `log/` | 아니오 | local build |
| raw `artifacts/`와 rosbag | 아니오 | local trial에서 새로 생성 |

`src/`, `data/`, `artifacts/`, `build/`, `install/`, `log/`는 `.gitignore` 대상이다.
다른 PC의 이 폴더를 Git에 강제로 넣거나, 절대경로 symlink를 그대로 복사해서
재현했다고 판단하지 않는다.

preflight의 `.engine` 확인은 0바이트 파일을 cache로 인정하지 않는다. non-empty 파일도
다른 GPU/driver에서 생성됐거나 중단된 cache일 수 있으므로 provenance와 실제 load
호환성은 `doctor.sh`와 VAD runtime gate에서 다시 확인한다.

## 5. 호스트 준비

### 5.1 이 repo의 `setup-dev-env.sh`를 바로 실행하지 않는 이유

현재 `ansible-galaxy-requirements.yaml`은 `./ansible` collection을 참조하지만 그
directory는 이 branch의 clone에 포함돼 있지 않다. 따라서 현재 repo의
`setup-dev-env.sh`는 신규 PC 설치 진입점이 아니다. 현재 guard는 이 부재를 발견하면
apt/pip/prompt 전에 아무 system 변경 없이 종료 코드 2로 즉시 거부하고 pinned upstream
절차를 안내한다. 실제 provisioning script는 system 설정을 바꿀 수 있으므로
preflight가 자동 실행하지 않는다.

### 5.2 권장 host provisioning 경로

Autoware Foundation의 **별도 upstream 1.9.0 exact checkout**에서 공식 Ansible
entrypoint를 사용한다. 이 project가 감사한 upstream commit은
`10718787ba6e28f038a0cb29ff99cc627b5abfd2`다. tag를 clone한 뒤 exact commit인지
검사하고, 다르면 설치를 시작하지 않는다. 아래 작업은 `sudo`, package 설치와 license
확인을 포함할 수 있으므로 공식 문서와 prompt를 먼저 읽는다.

```bash
cd ..
git clone --branch 1.9.0 --depth 1 \
  https://github.com/autowarefoundation/autoware.git \
  autoware-upstream-1.9.0
cd autoware-upstream-1.9.0

AUTOWARE_UPSTREAM_EXPECTED=10718787ba6e28f038a0cb29ff99cc627b5abfd2
AUTOWARE_UPSTREAM_ACTUAL="$(git rev-parse HEAD)"
if [[ "$AUTOWARE_UPSTREAM_ACTUAL" != "$AUTOWARE_UPSTREAM_EXPECTED" ]]; then
  printf 'ERROR: upstream commit mismatch; 설치를 중단합니다.\n' >&2
else
  bash ansible/scripts/install-ansible.sh &&
    export PATH="$HOME/.local/bin:$PATH" &&
    ansible --version &&
    ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml &&
    ansible-playbook autoware.dev_env.install_dev_env \
      -e cuda_install_drivers=false
fi
```

이 기본 경로는 공식 NVIDIA user-space/toolkit task와 apt repository는 준비하되 kernel
driver package 설치만 막는다. 이 repository가 있어야 뒤의 `build_full.sh`가 고정
CUDA 12.8/TensorRT 10.8 deb를 내려받아 workspace-local prefix로 풀 수 있다. NVIDIA
kernel driver는 해당 PC 관리 정책에 따라 별도로 정상 설치하고 재부팅한 뒤 확인한다.
특정 driver version을 이 문서에서 무조건 설치하지 않는다.

두 NVIDIA 선택은 의미가 다르다.

- `-e cuda_install_drivers=false`: 공식 playbook의 NVIDIA user-space/toolkit task는
  실행하되 현재 kernel driver package 교체만 막는다. 새 host의 기본값이다.
- `--skip-tags nvidia`: CUDA, TensorRT, spconv와 driver 관련 Ansible task를 모두
  건너뛴다. NVIDIA CUDA/TensorRT apt repository까지 이미 별도로 구성하고 검증한
  관리자만 사용할 수 있으며, 초보자/fresh-host 절차에는 사용하지 않는다.

둘은 같은 의미가 아니다. `--skip-tags nvidia`를 선택해 `apt-cache show
cuda-nvcc-12-8` 또는 pinned `libnvinfer10` 검사가 실패하면 local vendor build도
진행할 수 없다. 이 상태는 bootstrap preflight가 `vendor.*_source` BLOCK으로 보고한다.

upstream exact checkout에도 `setup-dev-env.sh --no-cuda-drivers`와 `--no-nvidia`가
남아 있지만 deprecated entrypoint다. 새 절차에서는 위
`ansible/scripts/install-ansible.sh`와 `install_dev_env` playbook을 사용한다.

```bash
timeout 10s nvidia-smi
```

GPU 이름, driver와 VRAM이 오류 없이 보여야 한다. `Driver/library version mismatch`가
나오면 먼저 재부팅한다. 임시 compatibility library는 정상 driver 정합을 대체하지
않으며 상세 조건은 `E2E_SETUP.md`의 NVIDIA 절을 따른다.

ROS도 확인한다.

```bash
test -f /opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

마지막 출력은 `humble`이어야 한다.

맵 검사·좌표 변환과 PNG/GIF 캡처에 쓰는 Ubuntu 도구도 준비한다. 다음 명령은 system
package를 설치하므로 목록을 확인한 뒤 직접 실행한다.

```bash
sudo apt-get update
sudo apt-get install \
  python3-numpy python3-scipy python3-open3d pcl-tools \
  ffmpeg x11-utils
```

`pcl-tools`는 packaged Town PCD의 좌표 변환에, NumPy/SciPy/Open3D는 맵 구조·정합
검사에 필요하다. `ffmpeg`와 `x11-utils`는 차량 중심 owned-RViz 화면을 녹화하고
정확한 창 소유권·크기를 검사할 때 필요하다.

## 6. pinned Autoware source 가져오기

프로젝트 repo로 돌아온다. 폴더 이름을 다르게 clone했다면 그 이름을 사용한다.

```bash
cd ../Autonomous-Driving
mkdir -p src
vcs import src < autoware.repos
```

`autoware.repos`는 이 branch가 사용하는 Autoware 1.9.0 계열 source 32개를 선언한다.
중간에 network 오류가 났다면 같은 명령을 다시 실행한 뒤 preflight의
`source.layout`이 현재 manifest의 동적 count인 `32/32`인지, `source.revision`이
32개 HEAD와 각 manifest tag/commit의 일치를 보고하는지 확인한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh
```

기본 full VAD build에는 `autoware.repos`를 사용한다. `tools.repos`의 `main`과
`simulator.repos`의 개발 branch는 mutable/optional 입력이므로 초보자 baseline에
섞지 않는다.

source가 준비된 뒤 이 workspace의 ROS system dependency를 해석한다. 다음 명령은
system package를 설치할 수 있다.

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init
rosdep update
rosdep install \
  --from-paths src autoware_e2e_vad_launch \
  --ignore-src \
  --rosdistro humble \
  -y
```

`sudo rosdep init`에서 “already initialized”가 나오면 기존 초기화를 지우지 말고
`rosdep update`부터 진행한다.

## 7. CARLA 0.9.15 준비

CARLA는 이 Git repo에서 다운로드하지 않는다. 공식 packaged CARLA 0.9.15 또는
프로젝트 소유자가 제공한 동일 계약의 runtime을 별도로 확보하고 압축을 푼다.
기본 탐색 위치는 repo의 형제 directory다.

```text
<workspace-parent>/carla-autoware-universe/CARLA_0.9.15
```

다른 위치라면 **CARLA를 사용하는 모든 새 terminal에서** 절대경로를 지정한다.

```bash
export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15
```

이 branch는 단순히 `pip install carla`만 한 환경이 아니라 다음 두 파일을 요구한다.

```bash
test -x "$CARLA_ROOT/CarlaUE4.sh"
test -f "$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
```

두 명령 모두 출력 없이 종료 코드 0이어야 한다. 공식 배포본에 Python 3.7 artifact만
있거나 pip의 다른 CARLA version이 설치돼 있어도 이 branch의 `env.sh` 계약을
통과하지 않는다. 임의 이름 변경이나 다른 version 파일 복사로 우회하지 않는다.

Town06, Town07, Town10 계열은 CARLA additional map asset이 필요할 수 있다.
C-track과 월악산은 stock CARLA map이 아니므로 별도의 cooked `.umap`, OpenDRIVE와
그 provenance가 필요하다. 배포 권한과 source 위치가 제공되지 않은 새 PC에서는
이 custom-map 결과를 재실행할 수 없다.

CARLA 위치만 검사한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh --carla-root "$CARLA_ROOT"
```

## 8. VAD model 받기

공식 `carla_tiny` v0.1 ONNX/JSON 네 파일은 다음 script가 HTTPS로 받고 고정 SHA-256을
검사한다. 약 400 MB 이상의 여유와 network가 필요하다.

```bash
scripts/e2e/download_vad_models.sh
```

기본 저장 위치는 다음과 같다.

```text
data/ml_models/vad/v0.1/
```

재실행하면 hash가 일치하는 파일은 다시 받지 않는다. `.engine` 세 개는 Git이나
download 대상이 아니다. 첫 VAD 실행 때 현재 GPU에 맞게 생성되며 몇 분 걸릴 수 있다.
다른 GPU에서 만든 engine을 복사해 재사용하지 않는다.

## 9. Full Autoware map 준비

### 9.1 왜 CARLA map만으로 부족한가

CARLA는 도로와 물리 world를 제공하지만 Full Autoware map stack은 별도로 다음 파일을
요구한다.

```text
Town01_full/
  lanelet2_map.osm
  pointcloud_map.pcd
  map_projector_info.yaml
  map_bundle.json
```

이 OSM/PCD 원본은 크기와 라이선스 때문에 Git에 포함되지 않는다. CARLA world 이름과
Autoware bundle 이름도 같은 개념이 아니다.

### 9.2 Town01 첫 bundle과 모든 packaged Town

`scripts/e2e/packaged_town_full_maps.yaml`에는 각 packaged Town의 OSM, PCD, CARLA
level/OpenDRIVE hash가 고정돼 있다. 그러나 manifest가 원본 파일을 배포하지는 않는다.
원본 Lanelet2/HDMaps와 정확한 CARLA package를 확보한 뒤에만 다음 inventory/setup
도구를 사용한다.

```bash
export AUTOWARE_E2E_TOWN_LANELET_ROOT=/absolute/path/to/autoware-contents/maps
export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15

python3 scripts/e2e/prepare_packaged_town_full_maps.py --map town01
```

첫 명령에는 `--prepare`가 없으므로 파일을 만들지 않는 inventory다. JSON의 Town01
`status`가 `READY_TO_PREPARE` 또는 이미 구성된 `FULL_MAP_READY`인지 확인한다.
`FAIL`, `BLOCKED_*`, hash/size/alignment 오류가 있으면 진행하지 않는다. source contract가
모두 맞을 때만 다음 명령으로 Town01을 준비한다.

```bash
python3 scripts/e2e/prepare_packaged_town_full_maps.py \
  --map town01 \
  --prepare
```

이 canonical 도구는 CARLA PCD를 ROS handedness로 변환하고 결과 hash를 검사한 뒤,
`data/maps/Town01_full`에 OSM/PCD symlink, projector와 provenance를 담은
`map_bundle.json`까지 만든다. 원본 또는 `data/generated/packaged_town_full_maps`를
이동하면 symlink가 깨지므로 보존한다. 완료 뒤 반드시 다음 엄격 검사를 통과시킨다.

```bash
bash scripts/e2e/bootstrap_preflight.sh \
  --map data/maps/Town01_full \
  --strict
```

나머지 packaged Town도 inventory에서 map id만 바꿔 같은 순서로 준비한다. 아무 `--map`도
주지 않으면 manifest의 전체 Town을 inventory하며, 여러 map을 한 번에 준비하기 전에는
각 source 결과와 필요한 디스크 용량을 먼저 확인한다.

두 환경변수를 지정하지 않으면 manifest의 과거 workstation 절대경로만 탐색하므로
새 PC에 원본 파일이 있어도 찾지 못할 수 있다. C-track/월악산은
`scripts/e2e/custom_map_bundles.yaml`과 `setup_custom_full_map.py`의 별도 계약을 따른다.
맵 소유자가 다운로드 위치와 권한을 배포하지 않은 환경에서는 preflight의 map BLOCK이
정상이며, 임의 PCD/OSM으로 통과시켜서는 안 된다.

## 10. Build

CARLA 경로와 ROS를 현재 terminal에 준비한다.

```bash
export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15
source /opt/ros/humble/setup.bash
```

Full Autoware, VAD, map/system/control/API/RViz를 한 번에 빌드한다.

```bash
scripts/e2e/build_full.sh
```

이 wrapper는 단순 compiler 실행만 하지 않는다. pinned source 상태를 확인하고 project
patch를 적용하며 workspace-local CUDA 12.8, TensorRT 10.8, spconv, acados와
`tl-expected`를 준비한다. 다운로드와 수 GB의 파일 생성이 발생한다. Full build는
기본적으로 package를 순차 처리하고 내부 병렬도를 2로 제한한다.

메모리가 부족하면 병렬도를 낮춘다.

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 COLCON_WORKERS=1 scripts/e2e/build_full.sh
```

중단 뒤 완료 package를 건너뛰며 이어갈 때만 resume를 사용한다.

```bash
AUTOWARE_E2E_FULL_BUILD_RESUME=1 scripts/e2e/build_full.sh
```

source drift나 patch hash mismatch가 발생했을 때 무조건 파일을 덮어쓰거나
`git reset --hard`하지 않는다. 실패 메시지와 `git status --short`를 먼저 기록한다.

## 11. Build 후 정적 검증

preflight strict부터 다시 실행한다.

```bash
bash scripts/e2e/bootstrap_preflight.sh --strict
scripts/e2e/doctor.sh
```

`doctor.sh`는 GPU, CARLA, model, spconv, ROS package와 TensorRT link를 검사한다.
두 진단의 PASS는 “실행 prerequisite가 준비됨”을 뜻하며 주행 성공을 뜻하지 않는다.

Python/launcher 회귀 테스트는 다음과 같다.

```bash
source scripts/e2e/env.sh
python3 -m pytest -q tests
python3 -m pytest -q autoware_e2e_vad_launch/test

colcon test --packages-select autoware_e2e_vad_launch
colcon test-result --verbose
```

두 pytest directory에는 이름이 같은 test module이 있으므로 인자 없이 한 번에 합쳐
수집하지 않는다. 위처럼 두 process로 분리하는 것이 이 repository에서 검증한 명령이다.

`env.sh`는 실행 파일이 아니다. 반드시 `source scripts/e2e/env.sh` 형태로 현재 shell에
적용한다. 다른 Autoware workspace를 먼저 source한 terminal도 정리하지만, 처음에는
새 terminal을 사용하는 편이 이해하기 쉽다.

## 12. 첫 30 km/h 직진과 회전 시험

다음은 **CARLA simulation screening**이다. 실차 실행 절차가 아니며
`real_vehicle_ready=false`다. 30 km/h는 nominal target이고 회전에서는 곡률 안전
제한 때문에 실제 속도가 낮아지는 것이 정상일 수 있다.

한 GPU에서 CARLA server를 두 개 동시에 실행하지 않는다. 기존 server와 Autoware가
없음을 먼저 확인한다.

```bash
pgrep -af 'CarlaUE4|ros2 launch|rviz2' || true
ss -ltnp | grep ':2100' || true
```

### 12.1 Terminal 1 — CARLA 한 개 시작

새 terminal을 열고 다음을 실행한다.

PNG/GIF를 저장하는 `--capture-desktop` 경로는 X11 `DISPLAY`와 `ffmpeg`, `ffprobe`,
`xdpyinfo`, `xprop`, `xwininfo`를 요구한다. preflight의 visual 절이 PASS인지 먼저 본다.
Wayland session, SSH의 빈 `DISPLAY`나 headless container에서는 UI 없이 계측은 가능해도
동일한 owned-window 화면 증거는 만들 수 없다.

preflight의 `visual.screen_geometry`는 X11 screen 사전조건만 확인한다. RViz client 창은
desktop panel과 window decoration만큼 작으므로 실제 owned-window 크기는 trial의
캡처 gate가 다시 검사하며, screen PASS만으로 캡처 성공을 보장하지 않는다.

최대화된 RViz 소유 창은 **1280×720 이상, 1920×1080 이하**여야 한다. 입력 창을
늘이거나 줄이지 않고 1920×1080 canvas 중앙에 검은 여백으로 맞추는 계약이므로,
2560×1440 desktop에서 최대화해 상한을 넘거나 작은 가상 display에서 하한에 못 미치면
캡처가 fail-closed로 중단된다. 이 경우 별도의 1920×1080 X11 display/session을 쓰고
preflight의 `visual.screen_geometry`를 다시 확인한다.

```bash
cd /absolute/path/to/Autonomous-Driving
export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15

scripts/e2e/run_carla_map.sh Town01 \
  --port 2100 \
  --quality Epic \
  --startup-timeout-sec 180 \
  -- -RenderOffScreen -nosound
```

이 command는 CARLA가 끝날 때까지 terminal을 점유한다. 그대로 둔다.
`-RenderOffScreen`은 창만 숨기고 RGB camera를 렌더링한다. `-nullrhi` 또는 CARLA
no-rendering mode는 VAD camera 입력을 없애므로 사용하지 않는다.

### 12.2 Terminal 2 — 직진/회전 route 두 개 생성

두 번째 terminal을 연다. CARLA bridge/Autoware를 시작하기 전에 route를 만든다.

```bash
cd /absolute/path/to/Autonomous-Driving
export CARLA_ROOT=/absolute/path/to/CARLA_0.9.15
export CARLA_PORT=2100
source scripts/e2e/env.sh
mkdir -p data/routes

scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario straight \
  --min-distance 170 \
  --max-distance 260 \
  --preferred-distance 210 \
  --max-traces 20000 \
  --output data/routes/beginner_town01_straight.json

scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario left \
  --min-distance 20 \
  --max-distance 120 \
  --preferred-distance 60 \
  --max-traces 20000 \
  --output data/routes/beginner_town01_left.json
```

자동 선택 route는 새 환경에서 기능을 배우기 위한 것이다. 반복 A/B 비교에서는 생성된
JSON과 spawn index, SHA-256을 고정해야 한다.

### 12.3 Terminal 2 — 직진 실행·계측·화면 저장

출력 directory는 존재하지 않는 새 경로를 사용한다.
첫 실행은 GPU별 TensorRT engine을 생성할 수 있어 아래 두 trial 모두 600초 readiness
timeout을 사용한다. engine cache가 정상 생성된 뒤에만 동일 PC의 후속 시험에서 이 값을
줄인다.

```bash
scripts/e2e/run_recorded_route_trial.sh \
  --recommended \
  --speed-30kph \
  --camera-source-5hz \
  --visualize \
  --capture-desktop \
  --ready-timeout 600 \
  artifacts/runs/beginner_town01_straight_30kph \
  data/routes/beginner_town01_straight.json \
  carla_port:=2100
```

이 helper가 자신이 시작한 Autoware, recorder와 RViz process group을 종료한다.
CARLA server는 다음 episode를 위해 남긴다. 종료 후 결과 directory에서 우선 다음을
확인한다.

```text
result.json
diagnosis.json
route_result.png
path_vs_control.png
steering_tracking.png
autoware_rviz_fullscreen.png
autoware_rviz_drive.gif
```

### 12.4 Terminal 2 — 회전은 별도 실행

직진과 같은 output directory를 재사용하거나 덮어쓰지 않는다.

```bash
scripts/e2e/run_recorded_route_trial.sh \
  --recommended \
  --speed-30kph \
  --camera-source-5hz \
  --visualize \
  --capture-desktop \
  --ready-timeout 600 \
  artifacts/runs/beginner_town01_left_30kph \
  data/routes/beginner_town01_left.json \
  carla_port:=2100
```

회전 PASS는 VAD가 전역 경로를 단독 생성했다는 뜻이 아니다. 현재 architecture는
VAD local candidate에 CARLA global route, command 선택, route corridor와 Autoware
MPC/PID를 결합한 `vad_route_manager_hybrid`다. 결과를 볼 때 goal, CTE, actual-to-final
tracking뿐 아니라 raw-to-final trajectory correction도 함께 본다.

### 12.5 종료

두 trial이 끝나면 Terminal 1에서 `Ctrl-C`를 한 번 누른다. project wrapper가 소유한
CARLA만 정상 종료하도록 기다린 뒤 확인한다.

```bash
pgrep -af 'CarlaUE4|ros2 launch|rviz2' || true
ss -ltnp | grep ':2100' || true
```

다른 사용자의 process를 이름만 보고 `pkill`하거나 강제로 종료하지 않는다.

## 13. 모든 준비된 Town을 돌릴 때

한 route를 이해하고 결과를 확인한 뒤에만 matrix runner를 사용한다. 이 runner는
각 map/trial마다 owned CARLA를 cold-start하고 straight와 turn을 분리하며, 준비되지
않은 map은 실패 실행으로 속이지 않고 BLOCKED로 기록한다.

```bash
scripts/e2e/run_autoware_vad_town_matrix.sh \
  artifacts/validation/my_30kph_matrix \
  --runtime-profile speed_30kph_camera_source_5hz_best_effort_image \
  --port 2100
```

모든 map의 정확한 CARLA level과 full-map bundle이 먼저 준비돼 있어야 한다. 긴 실행을
중단했다가 같은 계약으로 이어갈 때만 `--resume`을 사용한다. 새 parameter나 map을
비교하면서 이전 output을 덮어쓰지 않는다.

## 14. 60 km/h는 입문 실행으로 사용하지 않는다

60 km/h profile은 현재 simulation-only straight exploratory pilot이다. 목표점에는
도착했지만 60 km/h speed exposure gate를 통과하지 못했고 trajectory curvature,
속도 cap과 actuation-map 범위 문제가 남아 있다. 따라서 다음을 지킨다.
endpoint-tapered spatial-C1 offline 후보도 일부 곡률은 개선했지만 25/399 snapshot을
reject해 `HOLD`이며 live option으로 연결되지 않았다.

- 실차에 적용하지 않는다.
- turn이나 모든 Town으로 확대하지 않는다.
- `max_throttle`만 올려 해결됐다고 판단하지 않는다.
- production-equivalent geometry preflight, 60 km/h 범위 actuation calibration,
  controller/gate A/B와 반복 폐루프를 순서대로 통과하기 전에는 PASS로 표시하지 않는다.

현재 판정과 측정값은 `docs/validation-2026-09-02-runtime-control-campaign.md`를 본다.

## 15. 자주 막히는 문제

### `source.layout 0/32` 또는 `source.revision` mismatch

원인: `src/`는 Git에 들어 있지 않다.

```bash
mkdir -p src
vcs import src < autoware.repos
```

### `setup-dev-env.sh`가 `./ansible`을 찾지 못함

원인: 현재 project clone에는 그 local collection이 없다. 이 문서의 “호스트 준비”처럼
별도 upstream Autoware 1.9.0 checkout의 공식 Ansible entrypoint를 사용한다.

### PNG/GIF를 열었는데 LFS pointer text가 보임

```bash
git lfs install
git lfs pull \
  --include='docs/assets/validation/2026-09-02-runtime-control-campaign-v1/**' \
  --exclude=''
```

### `CARLA Python API not found`

`CARLA_ROOT`가 틀렸거나 exact Python 3.10 egg가 없는 package다.

```bash
printf '%s\n' "$CARLA_ROOT"
find "$CARLA_ROOT/PythonAPI/carla/dist" -maxdepth 1 -type f -print
```

다른 version의 pip module로 조용히 우회하지 않는다.

### map file이 있는데 preflight가 broken symlink 또는 3/4라고 함

canonical packaged map 도구는 검증한 source/generated PCD를 symlink한다. 원본을
이동했는지 확인한다. `map.full_bundle 3/4`는 legacy `setup_town01_full_map.sh`가 만든
3-file directory일 수 있으며 runtime admission 대상이 아니다. 그 directory에 빈 JSON을
추가하지 말고 9.2의 manifest inventory와 `--prepare`부터 다시 실행한다.

```bash
find data/maps/Town01_full -maxdepth 1 -type l -ls
```

### `nvidia-smi` driver/library mismatch

먼저 모든 작업을 저장하고 재부팅한다. 재부팅 뒤에도 mismatch면 kernel module과
user-space package를 시스템 관리자와 맞춘다. 과거 PC의 compatibility 경로를 그대로
복사하지 않는다.

### 첫 VAD 시작이 몇 분 동안 느림

ONNX에서 현재 GPU용 TensorRT engine 세 개를 만드는 과정일 수 있다. log에 engine
build가 진행 중이고 GPU/driver 오류가 없다면 기다린다. 생성 중 강제 종료한 `.engine`
을 정상 cache로 간주하지 않는다.

### 화면이 끊겨 보임

카메라 topic Hz만 보지 말고 CARLA real-time factor, wall-time bundle rate, complete
six-camera coverage와 GPU/CPU load를 함께 본다. 현재 권장 30 km/h campaign은
5 sim-Hz source와 localhost-only BestEffort depth-1 camera 경로를 사용한다.
5 Hz는 60 km/h에서 frame당 약 3.33 m이므로 runtime이 정상이어도 영상은 계단식으로
느껴질 수 있다.

### build가 메모리 부족으로 종료됨

```bash
CMAKE_BUILD_PARALLEL_LEVEL=1 COLCON_WORKERS=1 \
  AUTOWARE_E2E_FULL_BUILD_RESUME=1 \
  scripts/e2e/build_full.sh
```

disk와 kernel log도 함께 확인한다.

## 16. 도움을 요청할 때 붙일 정보

비밀번호, token이나 개인 데이터는 제외하고 다음 결과를 text로 첨부한다.

```bash
git branch --show-current
git rev-parse HEAD
git status --short
bash scripts/e2e/bootstrap_preflight.sh
timeout 10s nvidia-smi
df -h .
free -h
```

실행 실패라면 사용한 exact command, route JSON 경로, output directory와 마지막 오류
50줄을 함께 제공한다. “안 돼요”만 전달하면 route 문제, GPU 문제, stale process,
map mismatch를 구분할 수 없다.

## 17. 재현 가능하다고 말할 수 있는 경계

다른 PC에서 다음이 모두 같아야 동일 조건 재현에 가까워진다.

- project commit과 branch
- `autoware.repos` import revision
- CARLA 0.9.15 binary/API와 map level hash
- Lanelet2/PCD/projector bundle hash
- VAD ONNX/JSON hash와 deployment parameter
- route JSON hash, weather, spawn과 target
- controller/gate/actuation map parameter hash
- runtime camera/DDS profile
- evaluator gate와 cleanup 결과

clone과 build 성공만으로 과거 주행 결과가 재현됐다고 말하지 않는다. 또 CARLA PASS는
실차 안전성, arbitrary map 일반화, pure end-to-end planning 또는 60 km/h 준비 완료를
의미하지 않는다. 상세 architecture, 각 campaign 수치와 제한은 `E2E_SETUP.md`와
날짜별 `docs/validation-*.md`를 함께 확인한다.
