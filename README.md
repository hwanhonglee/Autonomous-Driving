# Autonomous-Driving: Autoware 아키텍처 스캐폴드 개발

<!-- HH_261012 - Distinguish the shared Autoware snapshot from the vehicle, simulation and architecture branch purposes. -->
[이 브랜치의 목적](BRANCH_PURPOSE.md) · [실차 플랫폼과 CARLA와 아키텍처 세 브랜치 안내](docs/autoware-three-branch-guide.md).
실차 플랫폼 브랜치는 통합 준비용이며 차량 구동 승인이 아닙니다. 현재 사용하는 중첩 Autoware 소스는 정확한 원본 커밋과 새 변경 패치로 별도 보존합니다.

<!-- HH_260906 - Use the canonical layout after verified removal of every root compatibility alias. -->
**새 실제 구조:** `autonomy/` · `learning/` · `tools/` · `storage/` · `docs/`.
[구조 안내](WORKSPACE.md) · [VS Code 작업공간](autoware_e2e.code-workspace) ·
[레거시 제거 원장](docs/workspace-layout-migration.md).
기존 루트 호환 링크 **16개 모두 제거, 남은 0개**입니다.
[최종 정리·전체 회귀 및 설치 검증](docs/workflows/05_maintenance/link_cleanup_20260925.md).
Python import 호환 처리는 명시적 실행 진입점에 남아 있으며 파일시스템 링크는 아닙니다.
이번 정리에서 데이터 보강·새 학습은 실행하지 않았습니다.

<!-- HH_260906 - Put current categorized navigation ahead of historical experiment links. -->
**작업공간 시작점:** [폴더·실행 파일 안내](WORKSPACE.md) · [주행](docs/workflows/01_driving/README.md) · [자동 지도](docs/workflows/02_mapping/README.md) · [최신 결과](docs/workflows/04_results/README.md)

<!-- HH_261008 - Surface the actual current model scope and bounded status command; do not present September fixed logs as live training. -->
**현재 자체 모델:** [최신 상태·메인9개/하위30개](docs/assets/validation/2026-10-02/README.md) · [10월8일 비학습 작업](docs/assets/validation/2026-10-08/README.md) · [두 모델의 실제 구조·학습 범위](docs/assets/validation/2026-10-08/model_scope/README.md) · [실제 종료 상태 확인 방법](docs/assets/validation/2026-10-08/workspace_status/README.md).
정식 승인 기능은 메인0/9·하위0/30입니다. 과거 `python3 tools/workspace.py status --remote`는 고정된9월 결과·로그 조회이며 현재 학습 중인지 판단하는 명령이 아닙니다.

<!-- HH_261001 - Surface the current learned-model evidence without replacing historical VAD reports. -->
**이전 자체 모델 작업 — 10월1일:** [당시 진행 현황과 9개 영역](docs/assets/validation/2026-10-01/README.md) ·
[주행 재출발 학습의 개선과 퇴행](docs/assets/validation/2026-10-01/warmup_supervision_ab/directional_gradient_comparison_v1/RESULTS_KO.md) ·
[새 지도 경로 학습·4장면 전량 결과](docs/assets/validation/2026-10-01/map_geometry_endpoints/RESULTS_KO.md) ·
[전체480프레임 지도 누적·8개 비교 화면](docs/assets/validation/2026-10-01/map_geometry_full_projection/README.md) ·
[주행 모델 200m 전진·미완주 원인·전체 GIF](docs/assets/validation/2026-10-01/warmup_supervision_ab/README.md).
지도 모델 개선은 확인했지만, 자체 주행 모델은 아직 기본 주행 복구 단계이며 실차·전체 기능 승인이 아닙니다.

<!-- HH_261012 - Identify prospective scaffold work without claiming a completed architecture transformation. -->
이 브랜치는 Autoware·자체 주행 모델·지도 모델의 인터페이스와 구조 개편을 준비합니다.
공통 소스 기준선을 분리한 것이며 아직 새 구조 변환이나 주행 기능을 구현한 것은 아닙니다.
이하 CARLA·모델 연구 자료와 canonical 경로를 유지하고 레거시 별칭은 복원하지 않습니다.
아래 clone 명령은 이 스캐폴드 개발 브랜치를 선택하며 사전진단은 구동하지 않습니다.

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone \
  --branch autoware-architecture/v1.9.0-scaffold \
  --single-branch \
  https://github.com/hwanhonglee/Autonomous-Driving.git
cd Autonomous-Driving
bash tools/e2e/bootstrap_preflight.sh
```

전체 CARLA 주행은 현재 검증된 **Ubuntu 22.04 x86_64 + NVIDIA GPU** 환경과 별도
CARLA·모델·맵 데이터가 필요하다. 이 대용량/라이선스 의존 파일은 Git에 넣지 않는다.
대신 사전진단 자체는 깨끗한 클론에서도 다운로드나 시스템 변경 없이 실행되며,
현재 단계와 누락 항목을 `PASS`, `WARN`, `BLOCK`으로 설명한다.

<!-- HH_260906 - Link completed research separately from existing Autoware driving and shadow evidence. -->

- 처음 설치하는 사람: [한국어 초보자 Quick Start](docs/BEGINNER_QUICKSTART_KO.md)
- 모든 옵션과 설계 경계: [E2E 상세 가이드](E2E_SETUP.md)
- 새 Portable 모델 반복 학습: [15회 학습·평가 결과](docs/validation-2026-09-07-portable-learning.md) · [두 PC의 역할과 실행 상태 조회](docs/portable-e2e-learning-loop.md)
- 이전 9월 9일 학습: [GPU 0 전체 모델 6회 완료·실제 학습 곡선·예측 PNG 72장](docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/README.md) — 가속도 입력 제외 후보는 품질 기준 미달로 미채택
- 모델 역할과 비교 방향: [자체 Portable 개발과 VAD·VisionPilot 활용](docs/portable-e2e-visionpilot-comparison.md) — VisionPilot은 조사 단계이며 아직 실행·교체하지 않았습니다.
- 학습 곡선·차량 중심 예측 경로·정답 수집 영상: [분류된 학습 증거 폴더](docs/assets/validation/2026-09-07/portable_e2e_learning_cycle_v1/README.md) — 새 후보 모델은 미채택이며 learned 차량 제어 증거가 아님
- 최신 30 km/h 제어 A/B와 Town06 strict 10 Hz 60 km/h `NO-GO` 분석: [2026-09-07 검증 보고서](docs/validation-2026-09-07-control-ab.md)
- Autoware + Portable E2E 10 Hz shadow 기준선: [2026-09-06 검증 보고서](docs/validation-2026-09-06.md)
- 최신 차량 중심 전체 화면·경로·제어 A/B·60 km/h live 분석: [2026-09-07 통합 발행 증거](docs/assets/validation/2026-09-07/control_ab_30kph_and_60kph_readiness_v1/)
- 이전 학습·open-loop 단계: [2026-09-05 Common10 검증 보고서](docs/validation-2026-09-05-portable-e2e-common10-30kph.md)
- Autoware runtime과 30/60 kph 역사적 결과: [runtime·control 검증 보고서](docs/validation-2026-09-02-runtime-control-campaign.md)

> `GIT_LFS_SKIP_SMUDGE=1`은 2 GB가 넘는 과거 화면·MCAP 자료를 첫 클론에서
> 생략하기 위한 선택 사항이다. 실행 코드는 그대로 받아지며, 필요한 화면만 받는
> 명령은 Quick Start에 설명되어 있다.

---

# Autoware - the world's leading open-source software project for autonomous driving

![Autoware_RViz](https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png)

<!--- Contributors -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware.universe/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware.universe?style=flat&label=Autoware%20Universe%20Contributors"
            alt="Autoware Universe Contributors" /></a>
    <a href="https://github.com/autowarefoundation/autoware/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/autowarefoundation/autoware?style=flat&label=Autoware%20Contributors"
            alt="Autoware Contributors" /></a>
</p>

<!--- Commit Activity -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware.universe/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware.universe?style=flat&label=Autoware%20Universe%20Commit%20Activity"
            alt="Autoware Universe Activity" /></a>
    <a href="https://github.com/autowarefoundation/autoware/pulse">
        <img src="https://img.shields.io/github/commit-activity/m/autowarefoundation/autoware?style=flat&label=Autoware%20Commit%20Activity"
            alt="Autoware Activity" /></a>
</p>

<!--- License -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware/blob/main/LICENSE">
        <img src="https://img.shields.io/github/license/autowarefoundation/autoware?style=flat&label=License"
            alt="License" /></a>
</p>

<!--- CI Reports -->
<p align="center">
    <a href="https://github.com/autowarefoundation/autoware/actions/workflows/health-check.yaml?query=branch%3Amain">
        <img src="https://img.shields.io/github/actions/workflow/status/autowarefoundation/autoware/health-check.yaml?style=flat&label=health-check"
            alt="health-check CI" /></a>
    <a href="https://app.codecov.io/gh/autowarefoundation/autoware.universe">
        <img src="https://img.shields.io/codecov/c/gh/autowarefoundation/autoware.universe?style=flat&label=Coverage&logo=codecov&logoColor=white"
            alt="Code Coverage" /></a>
</p>

<!--- Social Media -->
<p align="center">
    <a href="https://discord.gg/Q94UsPvReQ">
        <img src="https://img.shields.io/discord/953808765935816715?logo=discord&logoColor=white&style=flat&label=Autoware%20Discord"
            alt="Autoware Discord"></a>
    <a href="https://twitter.com/intent/follow?screen_name=AutowareFdn">
        <img src="https://img.shields.io/twitter/follow/AutowareFdn?logo=x&logoColor=white&style=flat"
            alt="Autoware Twitter / X"></a>
    <a href="https://www.linkedin.com/company/the-autoware-foundation/">
        <img src="https://img.shields.io/badge/Linkedin-Autoware%20Foundation-0a66c2?logo=linkedin&logoColor=white&style=flat"
            alt="Autoware Linkedin"></a>
</p>

Autoware is an open-source software stack for self-driving vehicles, built on the [Robot Operating System (ROS)](https://www.ros.org/). It includes all of the necessary functions to drive an autonomous vehicles from localization and object detection to route planning and control, and was created with the aim of enabling as many individuals and organizations as possible to contribute to open innovations in autonomous driving technology.

![Autoware architecture](https://static.wixstatic.com/media/984e93_552e338be28543c7949717053cc3f11f~mv2.png/v1/crop/x_0,y_1,w_1500,h_879/fill/w_863,h_506,al_c,usm_0.66_1.00_0.01,enc_auto/Autoware-GFX_edited.png)

## Documentation

To learn more about using or developing Autoware, refer to the [Autoware documentation site](https://autowarefoundation.github.io/autoware-documentation/main/). You can find the source for the documentation in [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation).

## Repository overview

- [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware)
  - Meta-repository containing `.repos` files to construct an Autoware workspace.
  - It is anticipated that this repository will be frequently forked by users, and so it contains minimal information to avoid unnecessary differences.
- [autowarefoundation/autoware_common](https://github.com/autowarefoundation/autoware_common)
  - Library/utility type repository containing commonly referenced ROS packages.
  - These packages were moved to a separate repository in order to reduce CI execution time
- [autowarefoundation/autoware.core](https://github.com/autowarefoundation/autoware.core)
  - Main repository for high-quality, stable ROS packages for Autonomous Driving.
  - Based on [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto) and [Autoware.Universe](https://github.com/autowarefoundation/autoware.universe).
- [autowarefoundation/autoware.universe](https://github.com/autowarefoundation/autoware.universe)
  - Repository for experimental, cutting-edge ROS packages for Autonomous Driving.
  - Autoware Universe was created to make it easier for researchers and developers to extend the functionality of Autoware Core
- [autowarefoundation/autoware_launch](https://github.com/autowarefoundation/autoware_launch)
  - Launch configuration repository containing node configurations and their parameters.
- [autowarefoundation/autoware-github-actions](https://github.com/autowarefoundation/autoware-github-actions)
  - Contains [reusable GitHub Actions workflows](https://docs.github.com/ja/actions/learn-github-actions/reusing-workflows) used by multiple repositories for CI.
  - Utilizes the [DRY](https://en.wikipedia.org/wiki/Don%27t_repeat_yourself) concept.
- [autowarefoundation/autoware-documentation](https://github.com/autowarefoundation/autoware-documentation)
  - Documentation repository for Autoware users and developers.
  - Since Autoware Core/Universe has multiple repositories, a central documentation repository is important to make information accessible from a single place.

## Using Autoware.AI

If you wish to use Autoware.AI, the previous version of Autoware based on ROS 1, switch to [autoware-ai](https://github.com/autowarefoundation/autoware_ai) repository. However, be aware that Autoware.AI has reached the end-of-life as of 2022, and we strongly recommend transitioning to Autoware Core/Universe for future use.

## Contributing

- [There is no formal process to become a contributor](https://github.com/autowarefoundation/autoware-projects/wiki#contributors) - you can comment on any [existing issues](https://github.com/autowarefoundation/autoware.universe/issues) or make a pull request on any Autoware repository!
  - Make sure to follow the [Contribution Guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
  - Take a look at Autoware's [various working groups](https://github.com/autowarefoundation/autoware-projects/wiki#working-group-list) to gain an understanding of any work in progress and to see how projects are managed.
- If you have any technical questions, you can start a discussion in the [Q&A category](https://github.com/autowarefoundation/autoware/discussions/categories/q-a) to request help and confirm if a potential issue is a bug or not.

## Useful resources

- [Autoware Foundation homepage](https://www.autoware.org/)
- [Support guidelines](https://autowarefoundation.github.io/autoware-documentation/main/support/support-guidelines/)
- [CI metrics](https://autowarefoundation.github.io/autoware-ci-metrics/)
