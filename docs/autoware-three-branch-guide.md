# Autoware 세 브랜치의 용도와 재현 방법

<!-- HH_261012 - Treat fixmoving as a future platform name rather than a vehicle model while preserving separate research and architecture branches. -->

이 저장소는 `hwanhonglee/Autonomous-Driving`입니다. 현재 작업공간의 Autoware 소스와 수정 패치를 공통 기준선으로 보존하고, 아래 세 목적을 별도 브랜치로 관리합니다. 브랜치를 만드는 것은 차량 구동 승인이나 자체 E2E 모델의 성능 승인을 뜻하지 않습니다.

| 브랜치 | 용도 | 현재 제한 |
| --- | --- | --- |
| `autoware-platform/v1.9.0-fixmoving` | 앞으로 개발할 fixmoving 플랫폼에 Autoware를 연계·확장하는 기준선 | fixmoving은 차량 모델명이 아님. 플랫폼 구성·외부 인터페이스는 후속 설계에서 정의 |
| `autoware-e2e/v1.9.0-vad-carla` | CARLA 수집·Autoware/VAD 실험·자체 주행 및 지도 모델 연구 | 학습 모델과 teacher를 구분하고 원래 평가 기준과 실패를 보존. 실차 보정값으로 사용하지 않음 |
| `autoware-architecture/v1.9.0-scaffold` | 후속 코드 아키텍처와 인터페이스 스캐폴드 변경 | 기준선에서 시작하는 개발 브랜치. 아직 별도 아키텍처 변환이나 실차 기능을 구현한 것은 아님 |

각 브랜치의 루트 `BRANCH_PURPOSE.md`를 먼저 읽으세요. 공통 코드가 같더라도 해당 브랜치의 목적과 검증 범위는 다릅니다. 기존 PC4/VILS 브랜치와 별도 작업공간의 소스는 이번 E2E Autoware 게시 범위에 포함하지 않습니다.

`fixmoving`은 사용자가 앞으로 진행할 플랫폼의 이름입니다. 특정 차량 모델·제조사나 CAN/DBW 구동 방식을 뜻하지 않으며, 플랫폼 브랜치의 용도를 정하기 위해 차량 모델명을 요구하지 않습니다.

## 실제 사용하는 소스와 설치 환경

`source tools/e2e/env.sh`가 선택하는 작업공간은 `autonomy/`입니다. 가져온 소스는 `autonomy/src`, 설치된 패키지는 `autonomy/install`에 있습니다. 로컬 설치 환경은 Git에 넣지 않으며, 현재 282개 설치 패키지가 있다고 다른 PC의 빌드까지 완료된 것은 아닙니다.

현재 구성은 `autoware_core` 1.9.0, `autoware_universe` 및 `autoware_launch` 0.52.0, `autoware_lanelet2_extension` 1.2.0을 포함합니다. 브랜치 이름의 1.9.0만으로 모든 의존 저장소의 버전을 추측하지 마세요. 실제 가져온 32개 저장소의 정확한 커밋은 [현재 소스 snapshot](../autonomy/source_snapshots/2026-10-12/active_v1/)의 exact manifest가 기준입니다.

중첩된 `autonomy/src`는 root Git의 게시 대상이 아닙니다. 대신 원본 HEAD와 변경 패치를 snapshot에 보존합니다. 수정된 4개 저장소의 tracked 변경뿐 아니라 새 파일도 포함해야 재현할 수 있습니다. upstream Autoware 저장소에 사용자 변경을 직접 push하지 않습니다.

## 주석과 원본 보존

새 설명은 파일 형식에 맞는 `HH_261012 - English explanation` 주석을 사용합니다. 이미 있는 `HH_YYMMDD` 주석의 날짜를 오늘 날짜로 일괄 바꾸지 않습니다. 이번 주석 작업은 기존 사용자 수정의 의미를 설명하며, 차량 제어 로직을 변경하거나 설치 바이너리를 다시 빌드한 것은 아닙니다.

직접 영어 주석을 추가한 의존 소스는 62개, 자체 launch·호환 소스는 48개입니다. 봉인 설정·일반 JSON 23개는 바이트를 바꾸지 않고 [별도 영어 설명](../autonomy/annotations/2026-10-12/autoware_comments.json)에 등록했습니다. [의존 소스 내용 보존 검산](../autonomy/provenance/2026-10-12/nested-source-comments-v1.json)도 함께 보존합니다. 환경 진입점의 설명은 `tools/e2e/env.sh`에 별도로 추가했습니다.

동결된 과거 실험 소스·선언·체크섬·패치는 그대로 유지합니다. snapshot은 새로운 파일로 추가하며 기존 봉인 파일에 주석을 덧붙이지 않습니다.

## 처음 저장소를 받는 방법

로컬 PC의 원하는 상위 폴더에서 다음과 같이 받습니다. `--branch`에는 위 표에서 사용할 브랜치 하나를 선택하세요.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone \
  --branch autoware-e2e/v1.9.0-vad-carla \
  https://github.com/hwanhonglee/Autonomous-Driving.git
cd Autonomous-Driving
bash tools/e2e/bootstrap_preflight.sh
```

사전진단은 설치·다운로드·차량 구동을 하지 않습니다. Ubuntu/ROS·NVIDIA·CARLA·모델·맵 의존성이 준비되지 않았다면 표시된 조건부터 해결해야 합니다. 어떤 운영체제에서든 `git clone`만으로 바로 주행할 수 있다는 의미는 아닙니다. [초보자 환경 준비](BEGINNER_QUICKSTART_KO.md)를 따르세요.

PNG/GIF 등의 대용량 기록은 Git LFS 대상입니다. 첫 clone에서는 생략할 수 있으며, 필요한 날짜만 선택해서 받습니다.

```bash
git lfs pull --include='docs/assets/validation/2026-10-12/**'
```

snapshot을 적용할 때는 exact manifest의 커밋으로 새 소스를 준비하고, 각 저장소에 대응하는 패치를 적용해야 합니다. 이미 수정된 소스에 기존 패치와 snapshot 패치를 겹쳐 적용하지 마세요. snapshot의 manifest와 검증 결과를 먼저 확인합니다. 다운로드된 소스를 얻었다고 해서 Autoware 전체 빌드·런타임·실차 검증이 완료된 것은 아닙니다.

의존 소스가 아직 없는 새 clone에서는 `vcs`가 준비된 환경에서 다음을 사용합니다. 게시된 snapshot의 저장 바이트와 깨끗한 exact HEAD에 대한 적용 가능성을 먼저 검증합니다. `--check-only`는 실제 소스를 변경하지 않습니다.

```bash
mkdir -p autonomy/src
vcs import autonomy/src < autonomy/source_snapshots/2026-10-12/active_v1/exact.repos
python3 tools/maintenance/export_active_autoware_snapshot_20261012.py \
  --verify --output-dir autonomy/source_snapshots/2026-10-12/active_v1
python3 tools/maintenance/export_active_autoware_snapshot_20261012.py \
  --check-only --output-dir autonomy/source_snapshots/2026-10-12/active_v1
```

검증에 성공한 새 소스에만 아래 네 패치를 적용합니다. 이미 수정된 현재 작업공간에서 실행하지 않습니다.

```bash
git -C autonomy/src/core/autoware_core apply ../../../source_snapshots/2026-10-12/active_v1/patches/03_core__autoware_core.patch
git -C autonomy/src/core/autoware_lanelet2_extension apply ../../../source_snapshots/2026-10-12/active_v1/patches/05_core__autoware_lanelet2_extension.patch
git -C autonomy/src/launcher/autoware_launch apply ../../../source_snapshots/2026-10-12/active_v1/patches/12_launcher__autoware_launch.patch
git -C autonomy/src/universe/autoware_universe apply ../../../source_snapshots/2026-10-12/active_v1/patches/18_universe__autoware_universe.patch
```

이는 패치 적용 단계이며 빌드·설치·주행을 실행하지 않습니다. 기존 개별 패치 자동 적용 도구는 일부 겹치는 hunk가 있어, snapshot 적용 뒤 전체 재빌드를 바로 실행하기 전 패치 진입점의 중복 적용 여부를 확인해야 합니다.

## fixmoving 플랫폼에서 먼저 정할 내용

먼저 fixmoving에서 Autoware가 맡을 역할과 필요한 입력·출력, 다른 구성 요소와의 연결 범위, 검증 기준을 정합니다. 구체적인 하드웨어와 기존 vehicle interface의 존재를 이름만으로 가정하지 않습니다. 현재 브랜치는 이 후속 개발의 기준선이며 플랫폼 전체 구현이 완료된 것은 아닙니다.

향후 실제 차량과 연결할 경우에는 그때 차량 인터페이스·CAN/DBW 사양·센서 보정·차량 치수·actuator 특성을 별도로 확인하고, 비구동 메시지·통신 중단·명령 차단·비상정지부터 검증합니다. 실차 구동은 별도의 명시적 승인과 안전 조건이 필요하며 CARLA 보정값이나 연구 모델을 그대로 실차에 적용하지 않습니다.

## 학습과 모델 검증 상태

로컬은 수집·추론·폐루프 검증을 담당하고 PRO6000 개인 `hwanhong` 환경은 학습·평가를 담당합니다. 서버 물리 GPU 0만 사용합니다. 10월 12일 17:36 KST 재확인에서는 SSH 인증은 성공했지만 NVIDIA 모듈 580.173.02와 설치된 라이브러리 580.178.04의 불일치로 GPU 0 조회가 종료값 18로 실패했습니다. 새 학습은 시작하지 않았습니다.

지도 crop-only 비교는 같은 모델과 초기 상태에서 입력 crop만 바꾸는 후속 학습입니다. CPU 입력 연결 이후에도 전체 학습기·기록·실패 처리·새 수치 검사와 GPU 소유권 확인이 필요합니다. 주행 후속 학습은 기본 차로 유지·속도·회전·목표 정지의 실제 실패를 기준으로 별도로 진행합니다. 새 head, optimizer 갱신 또는 코드 검사 통과만으로 기능 완료를 표시하지 않습니다.

정식 완료는 현재 **메인 0/9·하위 0/30**입니다. [현재 결과와 다음 학습 조건](assets/validation/2026-10-12/README.md), [기능 정의와 평가 기준](portable-e2e-feature-roadmap.md)을 유지합니다.
