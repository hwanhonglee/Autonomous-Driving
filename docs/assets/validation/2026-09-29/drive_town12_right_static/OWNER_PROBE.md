# Town12 source editor: 별도 소유 실행 경로

<!-- HH_260929 - Record the static preflight and the failed owned Town12 cold start. -->

**현재 판정: 정적 사전 점검 PASS, 소유 UE4Editor 콜드 스타트 FAIL.** 기존 [expert 소유 실행기](../../../../../tools/e2e/run_owned_carla_expert_trial.sh)는 packaged CARLA를 시작한다. Town12는 suite에서 `source_editor_0915_4ws`, `/Game/Carla/Maps/Town12/Town12`로 지정되어 있어 그 실행기를 그대로 사용하면 정확한 맵 소유를 보장하지 못한다. [Town12 전용 소유 프로브](../../../../../tools/e2e/run_owned_town12_source_editor_probe.py)를 별도로 추가했다. 이 프로브는 주행·카메라 수집 없이 원본 맵의 시작과 식별을 시도하는 소유 경로다. [실시간 실패 진단](LIVE_PROBE_FAILURE.md)에 종료 코드 `-11`과 렌더 스레드 스택을 구분해 기록했다.

먼저 실행한 정적 사전 점검 명령은 다음과 같다.

```bash
cd /home/a/autoware_e2e
python3 tools/workspace_python.py -m scripts.e2e.run_owned_town12_source_editor_probe
```

이 명령은 UE4Editor·프로젝트·source Python API·Town12 level·OpenDRIVE 파일의 고정 SHA256이 모두 일치하며 Python 3.10 source API import가 가능한지 확인했다. 출력 상태는 `STATIC_READY_NOT_LIVE_TESTED`, `simulator_started=false`, `capture_enabled=false`, `output_absent=true`였다. 사전 점검만으로 Town12 화면, 타일, 신호등, live OpenDRIVE 일치를 증명하지 않는다.

그 뒤 소유한 빈 포트에서 다음 명령을 **한 번 실행**했다. `owner_result.json`은 `FAIL`, UE4Editor 종료 코드 `-11`, `capture_performed=false`, `actor_spawned=false`를 기록했다. 시작 RPC가 준비되지 않아 live 맵 이름과 OpenDRIVE 해시는 읽지 못했다. 소유 프로세스와 포트는 정리되었고 고정 소스 바이트는 그대로였다. 기존 실패 폴더는 재사용하지 않았다.

```bash
python3 tools/workspace_python.py -m scripts.e2e.run_owned_town12_source_editor_probe -- --execute
```

실행 시 비차단 workspace 런타임 잠금과 CARLA TCP/UDP 2100–2102 포트, 다른 CARLA/UE4Editor 프로세스 부재, 새 출력 폴더를 먼저 검사한다. 위 고정 소스를 보관하고 Town12 URL을 가진 UE4Editor를 독립 process group으로 시작한다. `probe_carla_server.py`가 PID/PGID와 맵의 **전체 경로**를 확인하고, source CARLA API가 `carla.Map.to_opendrive()`를 읽어 고정 XODR SHA256 `26fa53d4902ac3397d35f948969f0c77c1bf023755ede2c67edf00bef817151f`와 비교한다. 어떤 실패에서도 소유 process group만 종료하고 포트 해제 영수증과 `owner_result.json`을 기록한다. 기존 결과 폴더는 덮어쓰지 않는다.

이 경로는 actor 생성이나 world tick 명령을 내리지 않는다. UE4Editor 자체의 비동기 시뮬레이션 시간은 흐를 수 있다. 이번 실행은 서버 식별 단계에도 도달하지 못했다. [정적 3742 경로](junction3742_derivative.json)를 expert 수집용 route JSON으로 변환하거나 live planner의 connector 3745 재현, 신호 actor와 stop-line 대응, 10Hz/20Hz 주행 계측, 화면 품질과 학습 데이터 QA는 구현·검증하지 않았다. [신호 수집 점검](SIGNAL_AWARE_CAPTURE_GAP.md)의 나머지 관문이 완료될 때까지 `capture_authorized=false`, `training_data_admitted=false`, learned closed-loop 검증 0이다.

고정 사전 점검과 [9개 단위 시험](../../../../../learning/tests/test_run_owned_town12_source_editor_probe.py)은 통과했다. 시험은 잘못된 맵·프로필·해시, 점유 포트와 기존 출력을 시작 전에 막고, 잘못된 전체 맵 경로와 live XODR 불일치를 성공으로 보고하지 않는지를 다룬다. 실제 UE4Editor 콜드 스타트는 실패했고 종료·포트 해제 영수증만 확인했다. Town12 카메라와 동적 신호는 시험하지 않았다.
