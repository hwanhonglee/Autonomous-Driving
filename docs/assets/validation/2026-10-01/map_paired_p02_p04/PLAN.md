# Town01 p02/p04 FALSE–TRUE 동시 수집·QA 계획

<!-- HH_261001 - Prepare a bounded same-frame capture after full240 map diagnosis; no simulator or fit executed. -->

상태: **사전 계획만 작성**. 새 CARLA 수집, 원본 변경, 학습, 지도 승격은 아직 없다. [전체 240프레임 지도 결과](../map_geometry_full_projection/README.md)에서 새 geometry 모델의 도로 셀은 개선됐으나 차선 정답 셀 대부분에 채택된 차선 증거가 없고 도로·pole 오검출이 늘었다. 기존 p02/p04의 480장은 FALSE 외관뿐이다. 다음 한정 자료 보강은 같은 경로의 **새 물리 주행 한 번당 RGB FALSE/TRUE를 같은 CARLA 틱에 동시 수집**해 외관 차이를 관찰하는 것이다. 수집 결과가 차선 문제를 해결한다는 가정이나 CTrack 점수 기반 프레임 선택은 하지 않는다.

## 고정 범위와 비교 단위

| 새 수집 | 경로 원본 | SHA256 | 기대 프레임 |
| --- | --- | --- | ---: |
| Town01 우회전 p02 | `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/turn/routes/town01/right/town01_right_s0000_p02.json` | `788665af9e3b5449634f1c5392e94b2a6258ae3262d26e69f3cf81b96791489a` | 240 |
| Town01 좌회전 p04 | `storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/turn/routes/town01/left/town01_left_s0000_p04.json` | `f4d474eee01892e16b8b78a69de5a3b32ccc8e4f227e30a9a89dc023dbfc25a7` | 240 |

두 JSON은 `Town01`, `base_link`, `ClearNoon`, 각각 `right`/`left`로 고정한다. 이전 FALSE p02/p04 수집의 owner·lifecycle·원본 재생은 통과했고 주행 거리는 각각 79.492m/83.989m였다. 새 물리 주행은 같은 **경로 정의**를 사용해도 BasicAgent의 프레임별 차량 위치가 이전 수집과 정확히 같다는 보장은 없다. 유효한 직접 쌍은 **새 수집 안의 같은 틱 FALSE/TRUE**이다. 기존 FALSE 프레임과 새 TRUE 프레임을 같은 위치의 쌍으로 간주하지 않는다. p02/p04는 상대 경로에서 겹치는 위치가 많으므로 독립 두 사이트로 세지 않는다.

각 경로는 240개 연속 관측을 전량 보관한다. p04의 과거 차선 없는 32프레임, 초기 저속/종료 정지 구간도 누락·중복 제거 없이 유지한다. 원본 p02/p04의 과거 작은 표지판 실패와 p04 guardrail tag28→`other`, 일부 받침 tag2→`sidewalk` 해석 한계를 그대로 기록한다. **새 paired 원본은 `PENDING_QA`, `dataset_admitted=false`, `train_eligible=false`**로 시작하고 전수 감사 후에만 `QA_CONDITIONAL`을 받을 수 있다. 기존 FALSE480의 한정 TRAIN 결정이 새 캡처를 자동 편입하지 않는다. Town01 DEV, Town02 p02 holdout, CTrack EVAL은 이번 수집·학습 입력이 아니다.

## 수집 구현 계약

[기존 paired v2 수집기](../../../../../tools/e2e/collect_map_paired_rgb_20261001_v2.py)와 [독립 원본 감사기](../../../../../tools/e2e/audit_map_paired_rgb_20261001_v2.py)의 센서 ABI·검증을 새 범위의 출발점으로 사용한다. **동결된 v2 파일과 과거 영수증은 수정하지 않고**, p02/p04만 허용하는 새 이름의 collector/owner/auditor와 계약 검사를 만든다. 현재 v2의 `ROUTES`는 p01/p00만 허용하므로 그대로 실행하면 위 경로는 사전 검증에서 거부된다. 새 코드가 준비되기 전에는 실행 명령이 없다.

- 고정 CARLA 0.9.15 packaged Town01 map/배포 바이너리/설정/OpenDRIVE 해시를 검증하고, 실제 world 이름과 `to_opendrive()` 해시를 owner 프로세스·RPC 소켓에 결속한다. Epic, ClearNoon, 빈 world, Prius, BasicAgent 15km/h, 명시적 traffic-light override, 물리 주행, 0.1초 동기 틱을 사용한다. 기존 30틱 준비 후 240개 관측 틱을 기록한다. 다른 CARLA owner가 있으면 포트가 달라도 시작하지 않는다.
- 같은 차량에 같은 `(x=1.5,y=0,z=2.0,pitch=-12°)`로 네 카메라를 붙인다: RGB `enable_postprocess_effects=false`, RGB `true`, raw depth, raw semantic. 모두 native 640×360 BGRA uint8, FOV90°, `sensor_tick=0`, `lens_k=lens_kcube=0`이다. depth/semantic에 postprocess 속성이 있으면 `false`로 설정하고, 없으면 설정하지 않은 상태와 실제 blueprint/actor 속성을 영수증에 남긴다. RGB 두 actor 속성은 postprocess 값 외에 같아야 한다.
- 매 관측마다 네 센서의 `frame`과 timestamp가 같고 네 `world_from_camera`가 허용 오차 내에 같은지 확인한다. **한 틱의 depth·semantic·pose 하나를 FALSE/TRUE RGB 둘 다의 공통 정답/기하**로 사용한다. PNG 두 장과 `rgb_false`, `rgb_true`, `depth`, `semantic`, `world_from_camera`를 포함한 압축 NPZ 한 개, 4개 raw BGRA 해시·artifact 해시·vehicle pose/control/속도·road/lane 픽셀 수를 저장한다. 두 경로 총 960 PNG와 480 NPZ가 기대 산출물이다.
- semantic은 설치된 CARLA enum의 road/lane/sidewalk/fixture/sign/pole 태그와 `other`를 사용한다. GuardRail=28은 `other`이며 새 정답 수정이 아니다. depth·semantic·pose는 학습 모델 입력으로 쓰지 않고 QA/투영 증거로 보존한다. 수집 중 oracle map을 누적하고, 저장한 NPZ만으로 독립 재생하여 snapshot 완전 일치를 요구한다.

## 소유권·공간·실패 경계

새 owner는 전역 workspace runtime lock을 획득하고 실행 중인 UE4Editor/CarlaUE4-Linux-Shipping, 선택한 RPC 포트와 그 뒤 두 포트의 점유를 사전 검사한다. 현재 Town07 주행 owner가 포트 2440–2442를 사용하는 동안 **어느 포트에서도 지도 수집을 시작하지 않는다**. 그 작업 종료·owned cleanup 확인 후, 새 collector를 단일 owner로 p02→QA→p04→QA 순서로만 실행한다. 기본값은 no-launch preflight, 실제 실행은 명시 `--execute`; owner는 자기 프로세스 그룹의 서버·수집기만 종료한다. 완료 후 `ready`, `after_capture`, `stopped` lifecycle 세 영수증과 프로세스 그룹/소켓 바인딩, worker·외부 BasicAgent·route byte archive를 보존한다.

2026-10-01 점검 당시 가용 공간은 **4,278,611,968 bytes (약 4.0GiB)**로 기존 paired 수집기의 5GiB 최소 보존 기준보다 작다. 이전 paired v2 디렉터리는 약 320MiB/333MiB였다. 새 구현은 처음부터 **6GiB 이상 가용**일 때만 시작하고, 두 새 owner/capture 디렉터리를 각각 400MiB, QA·로그 등 나머지 새 산출물을 합쳐 전체 1GiB로 제한한다. 각 쓰기 전후 5GiB 가용 공간을 유지하고, 한계 접근 시 새 프레임 쓰기를 중단해 부분 원본과 실패 영수증을 남긴다. 결과를 얻기 위해 기존 원본·동결된 결과를 지우거나 cap을 올리지 않는다. 이미 있는 출력 경로는 거부하고, 실패한 디렉터리에 append/resume하지 않는다. 원본 RGB/NPZ는 QA용으로 복사하지 않는다.

## 독립 QA와 종료 판정

새 감사기는 owner 종료 코드0, 세 lifecycle PASS, cleanup 오류0, 충돌0, source/package/route 해시 불변, 원본 240쌍의 누락·초과 없음부터 확인한다. 모든 프레임의 연속 tick·0.1초 cadence·네 센서 frame/timestamp/pose/raw 해시, NPZ ABI와 PNG↔raw RGB 픽셀 동일성을 전수 검사한다. 차량 pose↔카메라 mount, 경로 station/횡오차, 순간 이동, 실제 이동 및 과거 0.25m 기준 보존 위치≥120도 검사하되 240프레임은 전량 남긴다. 도로·차선 semantic 지지량과 유효 depth 범위는 사전 계약으로 검사하고, **p04의 32개 차선 부재 프레임을 이유로 제외하지 않는다**. 저장 raw depth·semantic·pose에서 240개 oracle 관측을 독립 재생해 live snapshot과 정확히 비교한다.

각 경로 고정 인덱스 0/119/239에서 FALSE와 TRUE 원본 화각, 공통 semantic overlay를 나란히 수동 확인한다. 얇은 차선·그림자·수목·하늘·표지판/신호등 구조물, p04 장벽 tag를 기록하되 점수나 시각 결과로 프레임을 골라내지 않는다. QA 통과 결과도 `QA_CONDITIONAL`이며 실제 모델 적합도, 독립 DEV, 10Hz, 카메라 전용 지도, HD-map 승인 또는 주행 기능 통과는 아니다. 이후 matched fit이 필요하면 별도 고정 schedule·same parent·same exposure 비교로 계획한다.

## 현재 준비 상태

<!-- HH_261001 - Record the completed static owner/collector/QA implementation without implying a CARLA capture. -->

경로 bytes/해시와 선행 full240 근거, 기존 paired ABI·owner·auditor를 확인했다. p02/p04 전용 [collector](../../../../../tools/e2e/collect_map_paired_town01_p02_p04_20261001.py), [owner](../../../../../tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh), [independent QA](../../../../../tools/e2e/audit_map_paired_town01_p02_p04_20261001.py), [계약 테스트](../../../../../learning/tests/test_map_paired_town01_p02_p04_20261001.py)를 별도 파일로 추가했다. 새 코드는 고정 경로·해시, 원본 소스 아카이브, 첫 p02 QA 뒤 p04 순서, 전역 lock, 실행 중 CARLA/포트 거부, 6GiB 시작·5GiB 실행 여유, 각 owner 400MiB·캠페인 1GiB 제한을 검사한다. 기존 v2 센서 엔진은 원본을 수정하지 않고 재사용한다. 새 원본은 계속 `PENDING_QA`, `dataset_admitted=false`, `train_eligible=false`다.

정적 검증: `python3 -B tools/workspace_python.py -m pytest -- -q learning/tests/test_map_paired_town01_p02_p04_20261001.py learning/tests/test_map_paired_rgb_20261001_v2.py`에서 **16 passed**; 새 shell 구문 검사 및 Python 구문 검사도 통과했다. 특히 다른 CARLA 패키지·CARLA 시작 스크립트 중복 소유자 검사와 프로세스 그룹의 PID 세대 재사용 거부를 테스트했다. **CARLA 수집·QA·학습·지도 승격은 아직 실행하지 않았다.** 2026-10-01 정적 검증 때 가용 공간은 6GiB 미만이라 새 owner의 **각 실행 시작 시 6GiB** 조건에 미달했다. 공간이 확보되고 다른 CARLA owner가 끝난 뒤에만 아래 순서로 진행한다. p02를 마친 뒤 p04도 다시 6GiB 시작 여유를 검사한다.

```bash
# HH_261001 - Run the two new physical captures serially only after the space and ownership gates pass.
base=storage/experiments/validation/2026-10-01/map_paired_p02_p04_v1
routes=storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town01/catalog/turn/routes/town01
bash tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh "$base/p02" "$routes/right/town01_right_s0000_p02.json"
bash tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh "$base/p02" "$routes/right/town01_right_s0000_p02.json" --execute
python3 -B tools/workspace_python.py -m scripts.e2e.audit_map_paired_town01_p02_p04_20261001 -- --source "$base/p02/mapping" --output "$base/qa/p02.json"
bash tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh "$base/p04" "$routes/left/town01_left_s0000_p04.json"
bash tools/e2e/run_owned_map_paired_town01_p02_p04_20261001.sh "$base/p04" "$routes/left/town01_left_s0000_p04.json" --execute
python3 -B tools/workspace_python.py -m scripts.e2e.audit_map_paired_town01_p02_p04_20261001 -- --source "$base/p04/mapping" --output "$base/qa/p04.json"
```
