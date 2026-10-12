# Town13 소스 맵 카탈로그 소유 실행기

<!-- HH_260906 - Record the static Town13 catalog owner contract before any UE4Editor launch. -->

**2026-09-27 판정: v2 Town13 카탈로그 소유 실행 `PASS`; 카메라 DEV 자격은 미검증.** [Town13 전용 실행기](../../../../../tools/e2e/run_owned_town13_route_catalog.py)의 기본 호출은 읽기 전용 사전 점검이다. Python 3.10에서 소스 CARLA egg와 navigation agents가 올바른 경로에서 import되는지, suite의 Town13 validation 바인딩과 UE4Editor·프로젝트·egg·`.umap`·`.xodr` 다섯 SHA256이 맞는지 확인했다. [첫 v1 owner 결과](../../../../../storage/experiments/validation/2026-09-27/map_town13_route_catalog_v1/owner_result.json)는 `FAIL`: UE4Editor가 Town13으로 응답했지만 `world.get_map().name`의 `/Game/` 접두어 차이로 실행기가 카탈로그 시작 전에 거부했다. v1은 `stopped`·포트 해제·소스/자산 해시가 통과했고 보존했다. 정확한 두 전체 경로 표기만 허용한 [새 v2 owner 결과](../../../../../storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2/owner_result.json)는 `PASS`다. Town13 12,478개 spawn에서 `lane_follow`·직진·좌회전·우회전 각 8개, 총 **32개** 경로를 만들었다. 카탈로그 SHA256은 `ddd1832dc8456ccd47e1c47206d31e2af84a380fd91bdfb3be2665ae08686c2d`이며 소스·자산 불변, `ready/after_catalog/stopped`와 포트 해제를 확인했다. **RGB·semantic·depth 카메라, 표지판 객체 가시성, 독립 DEV 승인, 지도 모델 학습은 여전히 0이다.**

```bash
python3 tools/workspace_python.py tools/e2e/run_owned_town13_route_catalog.py
# 실제 단회 소유 카탈로그가 필요할 때만 명시적으로 실행:
python3 tools/workspace_python.py tools/e2e/run_owned_town13_route_catalog.py --execute
```

소스 에디터 시작 명령은 `UE4Editor CarlaUE4.uproject /Game/Carla/Maps/Town13/Town13 -game -vulkan -prefernvidia -quality-level=Epic -carla-port=2100 -RenderOffScreen -nosound -unattended`로 고정했다. 실행 URL `/Game/Carla/Maps/Town13/Town13`와 관측된 RPC 표기 `Carla/Maps/Town13/Town13` **두 전체 자산 경로만** 허용했다. 같은 `Town13` basename의 다른 패키지는 거부한다. 완료된 경로 목록은 [v2 카탈로그](../../../../../storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2/route_catalog.json)에 있다. 소스 client/server 버전 문자열은 둘 다 `0.9.15-dev-dirty`이며 이를 공식 0.9.15 릴리스로 바꾸어 적지 않는다.

실행 계약은 다음과 같다.

- workspace 공용 `flock`을 획득하고 CARLA/UE4Editor 기존 프로세스와 포트 2100–2102 사용 여부를 확인한다. 점유 중이면 시작하지 않는다. 소유한 새 프로세스 그룹만 종료한다.
- 실패 영수증이 있는 v1과 성공한 v2는 그대로 보존한다. 실행기는 현재 고정 v2 출력이 이미 있으므로 같은 명령의 재실행을 거부한다. 다음 실험은 새 계획·출력 이름이 필요하다.
- 카탈로그는 `town13`, `source_editor_0915_4ws`, `lane_follow,straight,left,right`, seed `1,2`, seed당 4개, 60–140 m, 선호 100 m, 최대 12,000 trace/시나리오, 시작 420초·검색 1,200초의 고정 인자를 사용한다. `--allow-map-load`는 전달하지 않는다.
- 코드 원본 바이트를 `provenance/`에 보관하고 외부 소스 자산 SHA256을 `owner_plan.json`에 고정한다. `ready`, `after_catalog`, `stopped` 영수증과 `owner_result.json`을 남긴다. 실제 맵, 버전, spawn 수, 검색 조건, 경로 존재, 소스 바이트, 세 포트 해제를 검사한다.
- 카메라 촬영·차량 주행·지도 모델 학습·데이터셋 편입은 수행하지 않는다. 카탈로그가 성공해도 표지판이 카메라에서 보이는 DEV 경로라는 뜻이 아니다. 이후 경로를 **사전에 고정**하고 별도 Town13 수집 scope 및 화면 QA가 필요하다.

검증: `python3 tools/workspace_python.py -m pytest -- -q learning/tests/test_run_owned_town13_route_catalog.py learning/tests/test_audit_town13_source_editor_preflight.py`에서 **22개 통과**했다. 기본 호출의 무실행, 다른 프로필·변조 SHA·점유 포트·기존 출력의 시작 전 거부, 두 정확한 전체 맵 경로 허용 및 다른 경로 거부, 소유 PID 프로세스 그룹 정리를 검사한다. v2의 완료는 단위 테스트가 아니라 별도 소유 실행 결과로 확인했다.
