# 수집기 실제 시간 계측 — 선택 옵션 구현, 실제 계측 전

<!-- HH_260906 - Integrate only after the frozen v4 captures stopped; do not reinterpret historical captures as measured wall performance. -->

기존 v4 실행 두 건이 모두 종료된 것을 확인한 후, `scripts/e2e/carla_wall_timing.py`와 수집기의 명시적 `--wall-timing` 옵션을 연결했습니다. 기본값은 False이며, 물리 설정·센서 주기·제어기·품질 기준은 바꾸지 않습니다. 단위 테스트와 가짜 환경 통합 테스트를 진행했지만 **새 실제 계측 결과는 아직 없습니다**. 실행은 별도 소스 고정·사전 계획 검토 후 결정합니다.

계측 연결 전 기준 수집기 SHA256: `53d213bdef56895a27969cc6dd4eac638ba4fa6697badeaaa3367bd5741f01d0`. 기존 v4 원본은 이 실행 당시 archive로 검증하며, 현재 변경된 파일과 비교해 과거 증거를 무효화하지 않습니다. 같은 코드가 들어 있다는 것과 실행 바이너리의 동일성은 다릅니다.

## 구분하려는 것

센서의 시뮬레이션 시간 간격이 0.1초라고 해서 실제 컴퓨터에서 초당 10장이 처리되는 것은 아닙니다. `perf_counter_ns()`로 실제 처리 시간을 재고, 원본 `frame`·시뮬레이션 `timestamp`는 변경 없이 옆에 붙입니다.

| 계측값 | 의미 | 이것만으로 알 수 없는 것 |
|---|---|---|
| 시뮬레이션 frame·timestamp 간격 | 기록상 물리·카메라 주기 유지 여부 | 실제 시간 처리량 |
| world.tick + get_snapshot | 서버 틱 요청부터 동일 프레임 스냅샷 확보까지 | 순수 GPU 렌더 시간·CPU 원인 |
| observation_control | 좌표 변환·관측·경로 투영·BasicAgent·상태 append | 학습 모델 추론 시간 |
| camera_queue_wait | 기존 순서의 6개 카메라 큐에서 정확한 프레임을 확보하는 잔여 대기 | 각 센서가 이미 생성된 전체 시간·노출 지연 |
| jpeg_encode_write | 6개 이미지 검사·인코딩·파일 저장 및 카메라 기록 append | 카메라 화면 표시 FPS |
| control_rpc | 기존 명령 전달·수신 확인 및 receipt 연결 | 물리 액추에이터 적용 시각 |
| native/camera wall throughput | 관측 지점 또는 카메라 저장 완료 지점 사이 실제 시간 처리량 | RViz·CARLA 창·GIF 재생 FPS |

큐는 순차적으로 읽습니다. 첫 카메라를 기다리는 동안 다른 카메라가 준비될 수 있어, 두 번째 이후 대기 시간이 짧다고 센서가 더 빠른 것은 아닙니다. 카메라별 대기를 병렬 처리하거나 기존 공통 timeout을 카메라별 timeout으로 바꾸면 안 됩니다.

## 최소 연결 위치

아래는 연결의 논리 구조입니다. 실제 수집기는 기존 본문 순서를 유지하면서 `begin_stage/end_stage`, `begin_camera/end_camera`로 감싸고, 틱 wrapper가 예외 시 닫히지 않은 범위를 실패로 마감합니다. 기본 비활성 경로에서는 helper를 import하거나 시계를 추가로 읽지 않습니다.

```python
# HH_260906 - Proposed optional timing only; keep all existing collection and exception semantics.
with timing.tick(phase, tick_index, camera_expected=tick_index % interval == 0) as tick_time:
    with tick_time.stage("world_tick_snapshot"):
        frame = int(world.tick(args.timeout))
        snapshot = world.get_snapshot()
        # Keep the existing exact frame equality check here.
        timestamp = float(snapshot.timestamp.elapsed_seconds)
        tick_time.bind_observation(frame, timestamp)

    with tick_time.stage("observation_control"):
        # Keep snapshot state, measured fields, projection, governor,
        # BasicAgent, current/next control and state_records.append in their existing order.
        tick_time.mark_state_recorded()  # Only after the actual state append and annotations.

    if tick_index % interval == 0:
        with tick_time.stage("camera_queue_wait"):
            bundle = exact_camera_bundle(queues, frame, args.sensor_timeout_sec, timing=tick_time)
        with tick_time.stage("jpeg_encode_write"):
            for name in MODEL_CAMERA_ORDER:
                with tick_time.camera(name):
                    # Keep geometry validation and _save_jpeg exactly as they are.
                    pass
            # Keep camera_records.append exactly where it is.
            tick_time.mark_camera_recorded()  # Only after that append.

    with tick_time.stage("control_rpc"):
        # Keep send_control(...) and its existing finally: receipt link unchanged.
        pass
    # Keep tick_index increment, alignment/governor failure checks and return inside this tick scope.
```

`exact_camera_bundle`에는 선택 인자 `timing=None`을 추가했습니다. 기존의 단일 `deadline = time.monotonic() + timeout_sec`와 카메라 순서를 유지하고, 각 카메라의 **전체 while 루프**에 하위 범위를 붙입니다. 과거 프레임 폐기·다음 프레임 오류·동기 타임스탬프 검사는 그대로입니다. 새로 큐를 읽거나 센서 이미지를 다른 프레임으로 바꾸지 않습니다. 비활성 경로에는 새로운 파일·manifest 필드를 만들지 않습니다.

모듈의 `CAMERAS`와 수집기의 `MODEL_CAMERA_ORDER`가 정확히 같은지 opt-in 사전 검사합니다. `_save_jpeg` 본문은 변경하지 않고 바깥에서 한 번 측정합니다. 여기서 인코딩과 디스크 저장은 합산값이며 별도 분리 측정이라고 주장하지 않습니다.

## 파일·실패·종료 처리

- CLI는 명시적 `--wall-timing`이며 기본값 False입니다. 기존 기본 manifest·상태·카메라 JSON·수신 기록을 바꾸지 않습니다.
- 활성화 시 `partial/wall_timing.jsonl`을 배타적으로 새로 만들고, 완료 또는 예외가 난 **각 틱**의 레코드를 append/flush하는 콜백을 넘깁니다. 단계 중에는 로그 파일을 쓰지 않으며, 기존 다음 제어 명령 전달 후 틱 범위를 벗어날 때 씁니다. 추가 `world.tick`, 대기, 모델 로드 또는 하드웨어 조회는 없습니다.
- append 비용은 별도 측정합니다. 해당 레코드 자신의 `total_ns`에는 포함되지 않지만 다음 틱과의 간격 및 관측 간 실제 처리량에는 포함됩니다. 계측 자체의 시간 비용이 있으므로 향후 Low/Epic 비교에는 같은 계측 옵션을 적용해야 합니다.
- 카메라가 없는 물리 틱은 삭제하지 않고 `NOT_SCHEDULED`로 구분합니다. 예외로 실행되지 못한 단계는 `NOT_REACHED`, 실패한 단계는 `FAILED`입니다. 센서 timeout·JPEG 오류·제어 RPC 실패·정리 전 오류의 마지막 틱도 남깁니다.
- journal 저장 실패는 오류 **종류만** 공개 가능한 메모리 기록에 남기고, 일반 I/O 예외로 기존 제어·actor 정리를 막지 않습니다. 전체 timing rows는 메모리에 보관하며, 최종 finally에서 journal과 정확히 비교합니다. 불일치하면 기존 journal을 보존하고 별도 `wall_timing_recovery.jsonl`을 만들어 진단 FAIL로 기록합니다. 복구가 실패해도 `result.wall_timing`을 FAIL로 남기며 원본 캡처를 성공으로 재분류하지 않습니다.
- `KeyboardInterrupt`·`SystemExit` 같은 종료 요청은 삼키지 않습니다. 기존 수집기 바깥 `finally`의 actor 삭제·world/signal 복원을 보존합니다. 계측 요약·해시 실패 역시 이 정리를 건너뛰게 배치하지 않습니다.
- 수집 전체의 성공/실패가 확정된 **바깥 run finally**에서 요약합니다. `expected_state_frames`, `expected_camera_frames`, `expected_phase_counts`는 필터하지 않은 실제 `state_records`·`camera_records`에서 가져와 대조합니다. 특정 단계·실패를 빼면 완전한 계측으로 판정하지 않습니다.

```python
# HH_260906 - Proposed finalization must run only after owned cleanup has already been attempted.
report = timing.summarize(
    expected_state_frames=[row["frame"] for row in state_records],
    expected_camera_frames=[row["frame"] for row in camera_records],
    expected_phase_counts={phase: sum(row["capture_phase"] == phase for row in state_records)
                           for phase in ("stationary_warmup", "driving", "stationary_tail")},
    capture_succeeded=error is None,
)
```

프리캡처 bootstrap, 초기화·cleanup, 틱 바깥 initial-drive/driving-end 명령은 이 **native tick 단계 합계에 포함되지 않습니다**. 초기/종료 명령의 시간이 틱 사이에 들어가면 inter-tick gap에 보이지만 `control_rpc`로 귀속시키지는 않습니다. 전체 프로그램·GUI 지연까지 측정했다는 표현을 쓰지 않습니다. 필요하면 후속 버전에서 별도 session/setup/boundary span을 사전 정의해야 합니다.

## 출력과 검증 조건

각 시도 틱의 ns 구간·단계·카메라별 하위 구간·원본 frame/timestamp·실제 append 여부를 기록합니다. 요약에는 모든 단계의 평균·nearest-rank p50/p95/p99/max, 50/100/500/1,000ms 초과 횟수, warmup/driving/tail 전체 수, 카메라 없는 틱, 부분 실패, 로그 저장 오류, 실제 시간 처리량과 sim/wall 비율을 남깁니다. UTC 첫/마지막 시각은 표시·대조용입니다. NTP 조정으로 UTC가 역행할 수 있어 성능 계산에는 사용하지 않습니다.

새 모듈은 앞으로의 실행에만 사용합니다. 기존 ACK/v4 기록에는 wall timing이 없으므로 과거 결과를 새로 계측한 것처럼 채우지 않습니다. 소유 실행 wrapper는 expert의 명시적 `--wall-timing`에만 helper를 추가해 **정확한 11개 소스**를 보관합니다. 기존 비활성 expert는 10개 목록과 metadata를 유지합니다. 다른 worker에 이 옵션을 넘기면 시작 전에 거부합니다. 명시적 옵션·helper SHA·계측 스키마·원본 timing journal SHA를 owner/manifest에 묶습니다. 별도 사전 계획과 독립 timing 보고서 검증 없이 기존 10-source 감사기를 느슨하게 변경해 통과시키지 않습니다.

현재는 helper 단위 테스트와 전체 fake-world의 기본/활성 경로, 센서 timeout·인코딩 실패·RPC 실패·journal 실패·정리, wrapper의 10/11-source 보관 및 helper 변경 거부를 테스트했습니다. 실제 Low/Epic 주행이나 화면 끊김 해결·10 Hz 실시간 성능 달성을 주장하지 않습니다.
