# Autoware 1.9.0 + TensorRT VAD + CARLA 0.9.15 단일 가이드

이 문서는 이 워크스페이스에서 CARLA 시작점부터 목표점까지 VAD 기반으로
주행하고, 경로와 날씨를 바꿔 반복 평가하는 방법을 설명한다. 기존에 사용하던
Autoware와 섞이지 않도록 기본 ROS domain은 42이며, 새 CARLA는 다른 포트로
띄울 수 있다.

이 프로젝트의 설치, 실행, 맵 변경, 평가, 시각화, 성능 최적화는 **이 파일 하나를
기준 문서로 사용한다.** 별도 quick-start 문서를 만들지 않는다. 처음 실행할 때는
바로 아래 절을 사용하고, 구조와 안전 경계가 필요할 때 뒤의 상세 절을 읽는다.

## 30 kph all-Town VAD 검증 상태와 끊김 진단 (2026-09-01)

현재 `speed_30kph`는 실제 차량용 속도 설정이 아니라 CARLA에서만 쓰는 명시적
simulation screening profile이다. 목표 nominal speed는 `8.333333 m/s`이지만
`real_vehicle_ready=false`이며, VAD의 geometry는 평가해도 VAD가 낸 cruise velocity는
평가하지 않는다. 종방향 속도는 `explicit_simulation_nominal` overlay가 제공한다.
또한 planning architecture는 순수 end-to-end가 아니라
`vad_route_manager_hybrid`이다. v16 18건의 개별 진단 분포는 `path_dominant` 14건,
`within_thresholds` 3건, `mixed_path_and_control` 1건이다. 전체 claim
boundary는 이 개별 진단 label이 아니라 **hybrid simulation screening**이다.
VAD local candidate에 JSON route command, corridor/goal 처리와 Autoware MPC/PID가
개입하므로 `speed_30kph` profile PASS를 VAD 단독 속도 계획, 모든 trial의
실측 30 kph 도달, 또는 실차 30 kph 준비 완료로 해석하지 않는다.

### 현재 matrix 판정 경계

2026-09-01의 최종 v16은 19개 canonical map을 inventory하고, 그중 full-map과 CARLA
runtime이 승인된 9개 map에 straight/turn 두 trial씩 실행했다. 결과는 runnable map
`9/9`, trial `18/18` PASS다. 각 catalog와 trial은 CARLA를 별도 owned process group으로
cold-start하고 실제 map, completion, cleanup과 port 반환을 검사했다. 나머지 10개
`BLOCKED` row는 실행 실패가 아니라 검증된 Lanelet2+PCD bundle 또는 runtime asset이
없어 폐루프 claim을 보류한 항목이다.

속도 gate는 straight와 turn이 다르다. straight 9건은 `>=7.5 m/s`(`27 kph`)를
`>=1.0 s` 유지해야 했고 실측 최고속도는 `27.69-27.94 kph`였다. turn
9건은 곡률·횡가속도 제한을 우선하므로 minimum sustained-speed gate가 `0`이고,
실측 최고속도는 `17.62-24.02 kph`였다. 따라서 `18/18 PASS`는 각 trial의
정의된 기능·안전 gate 통과이지 모두가 정확히 30 kph를 냈다는 뜻이 아니다.

- runnable 9개: `town01`, `town02_opt`, `town03`, `town04`, `town05_opt`, `town06`,
  `town07`, `town10hd_opt`, `c_track_1_0_7`
- blocked 10개: `town02`, `town05`, `town08`, `town09`, `town10hd`, `town11`,
  `town12`, `town13`, `town15`, `woraksan_1_0_3`
- 최종 v16 matrix:
  `artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/`
- 발행 보고서: [2026-09-01 validation](docs/validation-2026-09-01.md)
- 발행 자산: [docs/assets/validation/2026-09-01](docs/assets/validation/2026-09-01/)

v14와 v15는 최종 판정이 아니라 아래 safety 사건의 재현 이력으로 보존한다.

- v14 전체 matrix:
  `artifacts/validation/2026-09-01/autoware_vad_town_matrix_30kph_v14_final/`
- v15 Town10HD_Opt exact-route 제한 재실행:
  `artifacts/validation/2026-09-01/autoware_vad_town10hd_repro_30kph_v15/`

v14의 유일한 실패는 `town10hd_opt/turn`에서 한 frame 동안 나온 VAD vehicle object가
standard AEB와 MRM을 작동시킨 사건이다. object confidence는 약 `0.512`였고 당시 전방
화면에는 대응 차량이 확인되지 않아 transient false positive 가능성이 높다. 다만 그
message를 입력으로 받은 AEB/MRM이 정지한 것은 안전 로직의 정상 반응이다. 동일 source
route hash로 다시 실행한 v15에서는 Town10HD_Opt straight와 turn이 모두 PASS했다.
v15 aggregate의 `INCOMPLETE 1/9`는 Town10HD_Opt만 의도적으로 선택했기 때문이며 전체
matrix PASS를 뜻하지 않는다. 최종 v16도 같은 source-route SHA로 straight와 turn이
PASS했다. v16 `turn`에서는 주행 중 dynamic forward CAR, RSS, AEB
brake/virtual wall, MRM 개입이 모두 0이었다. 단일 frame velocity spike는
v16 `straight`의 AEB corridor 밖 off-corridor 관측이었고 AEB/MRM을 작동시키지
않았다. v16에서도 정적·off-corridor object는 있었으므로 두 번의 clean
safety repeat가 v14의 확률적 transient를 지우거나 재발 가능성을 0으로
만들지는 않는다.

따라서 object confidence threshold를 전역으로 올려 안전 반응을 약화시키지 않는다.
후속 개선은 false positive 유형을 더 수집한 뒤 multi-frame/temporal confirmation을
별도 safety A/B로 검증한다. v16의 Town10HD_Opt 우회전은 기능 기준은 PASS지만
최대 correction `13.286/15 m`, 최대 CTE `0.735/1.0 m`이고 진단은
`mixed_path_and_control`이다. C-track 좌회전도 correction `12.679/15 m`인
`path_dominant`라서 두 경로는 고도화 우선순위다.

18개 화면은 모두 RViz 소유 창을 직접 녹화했다. `root_capture=false`, target
`base_link`, TopDownOrtho scale `10`, 입력 `1850x1016`을 무배율 대칭 padding해
`1920x1080`으로 만들었다. 대표 PNG는 route 평가 시간의 정확한 중간 frame이며, 차량은
전체 모니터의 물리 중심이 아니라 **우측 RViz 지도 viewport의 중앙**에 유지된다. 경로,
VAD 후보, 최종 궤적, 전방 camera, Routing/Localization/Autonomous 상태가 함께 보이고
GNOME bar, dock, terminal, tooltip과 알림은 캡처에서 제외된다. 전체 18개 MKV/GIF는
전 frame decode와 변화 검사를 통과했다.

### 화면이 끊겨 보이는 직접 원인

v16의 CARLA physics는 synchronous `fixed_delta_seconds=0.05`, 즉 simulation
기준 20 Hz다. 화면은 `Epic`, `-RenderOffScreen` 조건의 `640x360` RGB camera 6대다.
현재 `--recommended`가 실제로 고르는
`sensor_mapping_vad_fast_reliable_imu.yaml`은 각 camera의 `sensor_tick=0.0`이므로
여섯 camera를 모든 physics frame에 렌더한다. 이는 simulation 시간 기준 camera
image 120장/s다. bridge는 그중 같은 CARLA frame stamp의 완전한 6-camera bundle만
5 Hz, DDS `reliable`로 발행한다.

| 관측 항목 | v16 현행 값 | 해석 |
|---|---:|---|
| CARLA fixed step | `0.05 s` | simulation physics 20 Hz |
| 전체 RTF | `0.244-0.251` | simulation이 wall time의 약 1/4 속도로 진행 |
| camera simulation stamp rate | `5 Hz` | bridge가 의도한 bundle cadence는 유지 |
| wall-time effective bundle rate | 약 `1.22-1.24 Hz` | 화면에서는 약 `0.8 s`마다 새 simulation frame처럼 보임 |
| VAD inference | 대개 `34-39 ms`; 별도 spot sample `30-36 ms` | 약 0.8 s 끊김의 직접 병목이 아님 |
| candidate acceptance / coalesced drop | 약 `100%` / `0` | VAD assembler가 frame을 대량 폐기한 현상은 아님 |

즉 publish 설정 자체가 5 Hz라 시각적으로 부드러운 camera는 아니지만, 더 큰 계단감은
RTF 약 `0.25` 때문에 wall-time 유효 camera rate가 약 `1.2 Hz`까지 내려간 결과다.
5 fps로 만든 evidence GIF도 이 계단감을 추가로 강조한다. 여기까지가
artifact로 확인한 **화면 끊김의 직접 메커니즘**이다. 반면 낮은 RTF 자체의
원인을 CARLA synchronous six-camera rendering, bridge의 reliable exact-bundle 경로,
또는 다른 stack의 국소 직렬 대기 중 어디까지로 분리하는 A/B는 아직 하지
않았다. six-camera/exact-bundle은 현재 설정에 기반한 **원인 가설**이지 인과가
확정된 결론이 아니다.

v16 실행 중 12회 보조 표본의 CPU idle 최소/중앙/최대는 `72/84/90%`, iowait와 swap은
모두 `0`이었다. RTX 3090 Ti utilization 최소/중앙/최대는 `9/10/49%`, VRAM은 약
`4.9/24.6 GB`, 온도 `56-58 C`, 전력 `131-144 W`, throttle `0`이었다. 24 logical CPU
중 CARLA는 약 `1.45` core, route manager는 약 `2.33` core, VAD node는 약 `0.05` core를
썼다. 다만 이 12회 표본은 raw log·수집 시각·route stage가 trial의 immutable
artifact로 보존되지 않은 **보조 관찰**이다. 표본 시점에 지속적인 host 전체
포화가 보이지 않았다는 뜻이지, CARLA render thread나 exact-bundle 경로의 국소
직렬 대기, 순간 spike 또는 다른 process 간섭을 배제하거나 낮은 RTF의 원인을
증명하는 근거로 쓰지 않는다.

bag에서 odometry의 simulation-time 최대 간격은 `0.1 s`, control command 간격은
`0.05 s`였다. 따라서 현재 관측은 controller가 약 0.8초씩 명령을 멈춘 현상보다는,
simulation 5 Hz camera가 RTF 약 0.25로 wall time에서는 약 1.2 Hz가 되고 이를 5 fps
화면에 반복 표시하는 **시각적 계단감**에 가깝다.

### 다음 camera cadence A/B

첫 A/B는 같은 route, weather, `Epic`, UI와 physics 조건을 유지하고 camera source의
`sensor_tick`만 `0.0 -> 0.2`로 바꾼다. 기존 `sensor_mapping_vad_fast.yaml`은
best-effort camera이고 IMU가 없어 단일-변수 대조군이 아니다. 따라서 현재
`sensor_mapping_vad_fast_reliable_imu.yaml`의 5 Hz `reliable` exact-stamp camera, IMU,
GNSS와 나머지 모든 값을 동일하게 복제한 새 immutable mapping을 만들고 camera
`sensor_tick`만 `0.2`로 바꾸어야 한다. 그 mapping과 SHA를 각 trial에 보존한다.
이렇게 하면 source rendering도 5 Hz로 줄지만 ROS publish는 기존 5 Hz
`reliable` exact-stamp bundle을 그대로 유지한다. 채택 gate는 route PASS,
six-camera bundle coverage `>=99%`, candidate acceptance `100%`, coalesced drop `0`이며
RTF와 wall-time bundle rate도 함께 비교한다.

camera ROS publish Hz만 먼저 올리는 것은 첫 비교 변수로 삼지 않는다. source가 모든
physics frame을 이미 렌더하는 상태에서 DDS와 VAD 호출 부하를 늘려 RTF를 더
낮출 가능성이 있기 때문이다. 이 인과도 아직 A/B로 확정된 것은 아니다.
위 5 Hz source A/B가 기능·안전 gate를 통과한 뒤에만 10 Hz profile을 별도 campaign으로
검토한다.

## 역사 자료: C-track Driving Map Set Virtual 검증 (2026-08-29)

> 이 절은 2026-08-29의 2.5 m/s 계열 custom-map 검증 snapshot이다. 당시 입력 자산의
> `/home/hong/...` 경로는 provenance로만 보존하며 현재 실행 경로가 아니다. 현재
> workspace와 30 kph C-track 판정은 문서 맨 앞의 2026-09-01 v16 절을 따른다.

당시 검증은 사용자가 지정한
`/home/hong/Downloads/Driving_Map_Set/Driving Map Set`의 **Virtual PCD**를 실제
Full Autoware map bundle에 사용했다. 비교 그림의 공통 도로 구간에서는 같은 폴더의
v1.0.8 **Lanelet-only geometry가 Virtual PCD 차선과 더 가깝게 겹친다.** XODR 파생
Lanelet2를 현재 실행 bundle에 넣은 이유는 PCD 정합이 더 좋아서가 아니라, packaged
CARLA v1.0.7의 전체 road topology와 선택 route coverage를 유지하기 위해서다.

제공 Lanelet2 두 개는 현재 v1.0.7 OpenDRIVE의 서쪽 도로 영역을 포함하지 않으므로
현재의 전체-route CARLA bundle에는 넣지 않고 PCD 정합 reference로 유지한다. 제공된
`LocalCartesianUTM` projector도 그대로 적용하지 않는다. 해당 geodetic origin을
적용하면 기준 `local_x/local_y` 및 PCD frame에서 약 `13.2 m`가 이동한다.

따라서 실행 bundle `C_track_1_0_7_xodr_full`은 다음 조합으로 고정했다.

| 항목 | 채택한 기준 |
|---|---|
| CARLA 도로 | packaged `C_track_1_0_7.xodr`, RoadRunner/Unreal 원본과 SHA-256 동일 |
| Lanelet2 | 현재 전체-route 실행은 **정확한 v1.0.7 XODR** 파생본 사용; 제공 Lanelet-only는 PCD 정합 reference로 보존 |
| PCD | Driving Map Set Virtual의 `pointcloud_map (CARLA_C_track_LIOSAM_transform_RoadRunner_C_track_v1_0_7).pcd` |
| projector | `Local`; 제공된 `LocalCartesianUTM` 설정은 사용하지 않음 |
| CARLA -> map | `x=0`, `y=0`, `yaw=0`, `z=-15 m`; 실행 시 `map_bundle.json`이 route/pose에 적용 |

생성 Lanelet2는 node `89,579`, way `1,785`, road lanelet `395`이고 Virtual PCD는
`49,337,228` points다. 전체 CARLA waypoint와 Lanelet의 평면 오차는
`p50=0.089 m`, `p95=0.237 m`, `99.7% < 0.5 m`였다. 고정 직진 경로의 정적 오차는
`p50=0.077 m`, `max=0.250 m`이고, 이전 좌회전 경로의 정적 오차도
`p50=0.052 m`, `max=0.292 m`였다. 이 수치는 XODR 파생 Lanelet과 CARLA waypoint의
상호 일관성을 뜻하며, Virtual PCD에 가장 잘 맞는다는 뜻은 아니다. 비교 그림의
공통 coverage에서는 Lanelet-only가 시각적으로 더 가깝고, 기존 큰 평면 offset은
MPC가 아니라 **서로 다른 map generation과 projector를 섞은 것**에서 발생했다.

[PCD/XODR/Lanelet 좌표 비교](docs/assets/c_track_virtual/map_alignment.png)

2026-08-29 기준으로 두 구성을 다음처럼 구분한다.

- `c_track_virtual_lanelet_only_reference`: 제공 `_52SCF0.osm`과 Virtual PCD를 직접
  묶은 공통 coverage의 시각적/static XY 정합 reference다. XODR보다 서쪽 범위가 약
  `91.4 m` 짧고, `z=0`은 미검증 placeholder이므로 폐루프나 LiDAR localization
  결과로 해석하지 않는다. `_52SCF60.osm`은 local geometry가 동일한 provenance
  reference이며 bundle 입력은 아니다.
- `c_track_simulation_xodr_current`: XODR-derived Lanelet과 Virtual PCD를 묶어
  packaged CARLA v1.0.7 전체 topology 및 현재 검증 route를 실행한 profile이다.
  PCD overlap 우위나 LiDAR localization 완료를 의미하지 않는다.

### 폐루프 결과와 해석

아래 PASS/HYBRID PASS는 모두 `c_track_simulation_xodr_current` 결과이며
`c_track_virtual_lanelet_only_reference`에는 적용되지 않는다.

| 경로 | 판정 | 핵심 결과 |
|---|---|---|
| 직진 `394 -> 290` | `PASS` | `59.73 m` 중 `58.88 m` 주행, 잔여 `0.850 m`, max route CTE `0.394 m`, max corridor correction `0.649 m`, raw/final path p95 `0.538/0.429 m` |
| 좌회전 `369 -> 425` | **`HYBRID PASS`** | `70.29 m` 주행, 잔여 `0.861 m`, max route CTE `0.542 m`, max correction `11.483 m`, raw VAD/final path p95 `7.785/0.557 m`, MPC actual-to-final p95 `0.108 m` |
| Full UI 좌회전 재실행 | `HYBRID PASS` | RViz, Virtual PCD와 표준 Autoware node를 함께 실행해 잔여 `0.825 m`, max CTE `0.543 m`, max correction `11.173 m` 확인 |

좌회전의 `PASS`는 **순수 VAD E2E 성공을 뜻하지 않는다.** 현재 흐름은
`6 cameras -> TensorRT VAD의 command별 local candidate -> route command lookahead와
corridor 보정 -> final trajectory -> Autoware MPC -> CARLA`다. JSON/global route
manager가 command와 goal을 제공하고 corridor가 raw VAD 궤적을 크게 바꾼다. 실제로
좌회전 raw VAD의 p95 route 오차가 `7.785 m`, 최대 보정량이 `11.483 m`이므로 현재
결과 분류도 `path_dominant`다. MPC는 보정된 final trajectory를 p95 `0.108 m`로
추종했으므로 이 경로의 주된 잔여 문제는 제어기보다 VAD planning output이다.

좌회전 성공에는 `15 m` maneuver lookahead와, 중간 `STRAIGHT` 표식이 뒤의
`LEFT/RIGHT` command를 가리지 않도록 방향 command를 우선하는 현재 route logic이
필요하다. 이것은 VAD weight 개선이 아니며, corridor 없이 같은 품질이 나온다는
근거도 아니다. 순수 VAD를 목표로 하면 C-track camera/ego-state/future-trajectory
데이터로 planning head를 추가 학습하고 corridor를 단계적으로 줄여 다시 검증해야 한다.

다음 경로는 채택하지 않는다.

- `111 -> 55`: 처음 약 `12.46 m`가 약 90도 우회전인데 `LANE_FOLLOW`로 표시되고,
  이후 별도 좌회전이 이어지는 잘못된 단일-left 시험이다. 초기 correction
  `17.324 m`로 즉시 실패했다.
- `481 -> 383`: 정적 map 및 회전 전 raw VAD는 양호했지만 약 `14.5 m`,
  `(-250.55, 38.04)`에서 Unreal road collision/mesh에 걸려 물리적으로 정체됐다.

시각 결과는 Git에 포함되는 다음 한 폴더만 본다.

- [직진 결과 PNG](docs/assets/c_track_virtual/straight_route_result.png),
  [직진 GIF](docs/assets/c_track_virtual/straight_drive.gif)
- [좌회전 수치 결과 PNG](docs/assets/c_track_virtual/left_route_result.png),
  [좌회전 궤적 GIF](docs/assets/c_track_virtual/left_turn.gif)
- [Full Autoware 좌회전 PNG](docs/assets/c_track_virtual/full_autoware_left.png),
  [Full Autoware 좌회전 GIF](docs/assets/c_track_virtual/full_autoware_left.gif)

### Bundle 생성과 재현 실행

이미 `data/generated/xodr_lanelet/`이 있으면 먼저 inspect/setup만 실행한다.
XODR에서 다시 생성해야 할 때만 첫 번째 명령을 실행한다. `.venv-map`은 wrapper가
`requirements-map.txt`의 고정 버전으로 생성한다.

```bash
cd /path/to/autoware_e2e
source scripts/e2e/env.sh

# 원본 Virtual PCD의 현재 호스트 경로를 명시한다. manifest의 hash/size 검증은 유지된다.
C_TRACK_PCD=/absolute/path/to/c_track_virtual_pointcloud_map.pcd

# 필요할 때만: packaged runtime XODR -> Autoware Local Lanelet2 재생성
scripts/e2e/build_xodr_lanelet_map.sh --install --force \
  --translation-z-m -15 \
  --json-report data/generated/xodr_lanelet/c_track_finalize.json \
  "$CARLA_ROOT/CarlaUE4/Content/Carla/Maps/OpenDrive/C_track_1_0_7.xodr" \
  data/generated/xodr_lanelet/c_track_commonroad.osm \
  data/generated/xodr_lanelet/c_track_autoware.osm

# Virtual PCD hash/구조와 모든 pinned source를 확인하고 bundle 생성
python3 scripts/e2e/setup_custom_full_map.py \
  inspect c_track_simulation_xodr_current \
  --source "pointcloud_map=$C_TRACK_PCD" --skip-reference-assets
python3 scripts/e2e/setup_custom_full_map.py \
  setup c_track_simulation_xodr_current \
  --source "pointcloud_map=$C_TRACK_PCD" --skip-reference-assets
```

Lanelet-only와 Virtual PCD의 pinned source를 재확인하려면 다음 명령을 사용한다.
`setup --dry-run`까지는 파일 검증과 생성 예정 작업만 확인하며, 실제 `setup`도
symlink와 metadata를 만드는 정적 bundle 작업일 뿐 주행 검증은 아니다.

```bash
python3 scripts/e2e/setup_custom_full_map.py \
  inspect c_track_virtual_lanelet_only_reference --json
python3 scripts/e2e/setup_custom_full_map.py \
  setup c_track_virtual_lanelet_only_reference --dry-run
```

이 reference의 서쪽 coverage와 Z를 보완하고 별도 route preflight/폐루프 검증을
마치기 전에는 `C_track_virtual_lanelet_only_reference`를
`AUTOWARE_E2E_FULL_MAP_PATH`로 지정하거나 아래 `394 -> 290`, `369 -> 425` 결과를
재사용하지 않는다.

Terminal A에서 CARLA를 시작한다. C-track은 여섯 camera 사용 시 `Low` renderer
crash가 재현됐으므로 반드시 `Epic`을 사용한다.

```bash
cd /path/to/autoware_e2e
scripts/e2e/run_carla_map.sh C_track_1_0_7 \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
```

Terminal B에서 Full Autoware, VAD, MPC와 시각화를 시작한다. 처음 `89,579`-node
OSM과 592 MB PCD를 읽는 cold load는 이 PC에서 약 `100 s` 걸렸고 이후 cache된
실행은 훨씬 짧았다. 첫 로드 중 route가 잠시 없다고 바로 실패로 판단하지 않는다.

```bash
cd /path/to/autoware_e2e
source scripts/e2e/env.sh
export CARLA_PORT=2100
export AUTOWARE_E2E_FULL_MAP_PATH="$PWD/data/maps/C_track_1_0_7_xodr_full"

# 직진 394 -> 290
scripts/e2e/run_route_vad_fast.sh --full --visualize --recommended \
  autoware_e2e_vad_launch/config/routes/c_track_straight_394_290.json
```

검증된 좌회전은 기본 `--recommended`의 `3 m`가 아니라 `15 m` lookahead를 쓴
명시적 profile이다. 다음 명령은 당시 채택한 VAD mapping, MPC와 제한값을 그대로
드러내므로 임의의 숨은 기본값에 의존하지 않는다.

```bash
scripts/e2e/run_route_vad_fast.sh --full --visualize \
  --model-override autoware_e2e_vad_launch/config/vad_carla_tiny_recommended.param.yaml \
  --sensor-mapping autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml \
  autoware_e2e_vad_launch/config/routes/c_track_left_369_425.json \
  use_lateral_controller_param_override:=true \
  lateral_controller_param_path:="$PWD/autoware_e2e_vad_launch/config/mpc_carla_recommended.param.yaml" \
  use_longitudinal_controller_param_override:=true \
  controller_stop_offset_m:=0.60 comfortable_deceleration_mps2:=0.60 \
  maneuver_lookahead_m:=15.0 turn_inward_corridor_half_width_m:=0.20 \
  trajectory_geometry_smoothing_strength:=10.0 maximum_speed_mps:=2.5
```

폐루프 검증이 완료된 맵도 실행 중 hot-swap하지 않는다. Autoware를 `Ctrl-C`로
종료하고 CARLA도 종료한 뒤, 실행 가능한 profile을 `setup`, 해당 CARLA level을
`run_carla_map.sh`로 cold-start하고, 새 bundle을 `AUTOWARE_E2E_FULL_MAP_PATH`에
지정한다. Route JSON의 `town`도 새 CARLA map과 같아야 preflight를 통과한다.
`reference_only` 상태인 profile은 이 일반 실행 절차의 대상이 아니다.

### 남은 3D 제한

Virtual PCD와 packaged XODR는 평면에서는 identity로 정렬되지만 높이 차이는
공간적으로 일정하지 않다. 현재 표본에서 route가 PCD road surface보다 직진 구간은
약 `0.68 m`, 좌회전 구간은 약 `2.55 m` 낮아 단일 `z` offset으로 두 구간을 모두
맞출 수 없다. 현재 simulation은 truth localization과 2D corridor/control을 사용해
폐루프가 가능하지만, 이 상태를 실제 LiDAR localization 준비 완료로 해석하면 안 된다.
실차/PCD localization 전에는 여러 3D anchor 또는 lane graph를 따른 공간 가변 height
fit을 적용하고, PCD-Lanelet-CARLA Z residual을 다시 측정해야 한다.
`c_track_virtual_lanelet_only_reference`의 `z=0`은 이 residual을 해결한 값이 아니라
CLI/metadata 스키마를 위한 명시적 미검증 placeholder다.

## 역사 자료: Custom map Full Autoware + VAD 상태 (2026-08-28)

> 아래 C-track·월악산 수치는 2026-08-28 고정 route 결과다. 특히 월악산 `3/3 PASS`를
> 2026-09-01 v16 matrix에 합치지 않는다. v16에서는 승인된 packaged runtime/full-map
> 조합이 없어 `woraksan_1_0_3`을 `BLOCKED`로 유지한다.

C-track과 월악산은 CARLA level만 확인한 상태가 아니다. 이 PC의 RoadRunner/Unreal,
OpenDRIVE, Lanelet2, PCD와 좌표 변환을 묶어 **Full Autoware shell + TensorRT VAD +
Autoware MPC/PID** 폐루프 주행까지 실행했다. 결과는 순수 goal-conditioned E2E가
아니라 route command/corridor를 사용하는 `vad_route_manager_hybrid`임을 전제로 한다.
아래 C-track 행은 위 2026-08-29 Virtual PCD 결과가 아니라 64.29M-point PCD를 사용한
`c_track_simulation` legacy baseline이다.

| 맵/고정 경로 | 실행 profile | 반복 판정 | 최대 route CTE | 목표 잔여 | wall time |
|---|---|---:|---:|---:|---:|
| C-track `394 -> 290` | `c_track_simulation` + `--recommended` | `1/1 PASS` | `0.382 m` | `0.858 m` | `30.27 s` |
| 월악산 safe `2 -> 8` | `--recommended --tight-corridor` | `3/3 PASS` | `0.247 / 0.226 / 0.216 m` | `0.809 / 0.734 / 0.813 m` | `19.04 / 23.44 / 23.24 s` |

요약 시각 자료는 한 폴더에 모았다.

- [월악산 Autoware 주행 화면](docs/assets/custom_maps/woraksan_autoware_driving.png)
- [C-track 결과 PNG](docs/assets/custom_maps/c_track_route_result.png), [GIF](docs/assets/custom_maps/c_track_drive.gif)
- [월악산 결과 PNG](docs/assets/custom_maps/woraksan_route_result.png), [GIF](docs/assets/custom_maps/woraksan_drive.gif)
- [구조화된 결과](docs/assets/custom_maps/results.json)

월악산 초기 실행은 `comfortable_deceleration_mps2=1.2`에서 목표 `1.941 m` 전에
물리적으로 정체됐다. PID는 당시 `DRIVE` 상태로 양의 가속을 명령하고 있었으므로
goal tolerance나 MPC 횡제어 문제가 아니었다. Launch 인자 배선을 추가한 뒤 종단
감속을 실제 `0.6 m/s^2`로 낮췄고, runtime parameter dump와 bag trajectory의
최소 가속도 `-0.600000 m/s^2`를 확인한 유효 반복이 `3/3 PASS`했다. 당시
`--recommended`는 이 값을 고정한다. 월악산의 `--tight-corridor`는 별도 맵용
옵션이며 모든 맵의 전역 기본값은 아니다.

## 역사 자료: expert 데이터 수집 범위와 VAD 학습 경계 (2026-08-28)

> 이 절의 smoke 표와 local asset 경로는 2026-08-28 수집 snapshot이다. 현재 v16
> Full VAD 폐루프의 runnable/blocked 범위나 30 kph 결과로 재해석하지 않는다.

당시 구현된 것은 `CARLA BasicAgent expert 주행 -> 6-camera/차량 상태 수집 ->
0.5초 간격 3초 future trajectory label export -> PNG/GIF 검수` 파이프라인이다.
이는 VAD weight를 실행 중 갱신하는 self-training이 아니다. BasicAgent가 차량을
제어하고 정답 궤적을 만드는 **offline supervised imitation data collection**이며,
당시 collector 안에서는 VAD shadow inference도 실행하지 않았다. 따라서 이 절의
`PASS`는 **BasicAgent expert 수집, export 계약과 6-camera visual QA가 통과했다**는
뜻이다. VAD inference, VAD weight 학습, VAD 폐루프 주행 또는 다른 route와 맵으로의
일반화가 검증됐다는 뜻이 아니다.

약 4 TB는 로컬에 존재하는 파일이 아니라 공개 Bench2Drive Full을 받을 때의
배포 규모다. **이번 단계에서는 Full과 Base bulk dataset 다운로드를 모두 제외**하고,
당시 저장공간 gate는 약 20 GiB smoke collection만 확인했다. 외장 저장장치가 준비되기
전에는 대규모 수집이나 dataset/checkpoint 다운로드를 시작하지 않는다.

```bash
cd /home/a/autoware_e2e
scripts/e2e/setup_b2d_vad_training.sh --check
```

이 명령은 read-only 점검이다. dataset/checkpoint를 받지 않고 환경도 설치하지 않는다.
현재 RTX 3060 12 GB에서는 공개 VAD 전체 모델 재학습보다 backbone/BEV feature를
freeze/cache한 planning-head-only 실험이 현실적인 범위다. 다만 실제 freeze/cache
training config, dataset loader, PyTorch fine-tune 및 ONNX/TensorRT parity는 아직
구현되지 않았다. `--install --accept-noncommercial-license`도 dataset/checkpoint를
받지 않으며, license flag가 upstream 저장소 간 라이선스 차이나 수정 checkpoint의
상업적 배포 권한을 해결해 주는 것은 아니다.

### 2026-08-28 당시 expert smoke 결과

| 맵/조건 | raw camera anchors -> validated samples | collision / lane invasion | max route CTE | 판정 |
|---|---:|---:|---:|---|
| Town01 / ClearNoon / right / seed 0 | `153 -> 138` | `0 / 0` | `1.559 m` | collector/export/visual smoke PASS |
| Town02_Opt / CloudyNoon / right / seed 11 | `249 -> 234` | `0 / 0` | `1.897 m` | collector/export/visual smoke PASS |
| Town03 / Epic / ClearNoon / lane_follow / seed 0 | `149 -> 134` | `0 / 0` | `0.496030 m` | packaged strict smoke PASS |
| Town04 / ClearNoon / lane_follow / seed 0 | `123 -> 108` | `0 / 0` | `0.003157 m` | packaged strict smoke PASS |
| Town05_Opt / ClearNoon / lane_follow / seed 0 | `111 -> 96` | `0 / 0` | `0.010790 m` | packaged strict smoke PASS |
| Town06 / ClearNoon / lane_follow / seed 0 | `122 -> 107` | `0 / 0` | `0.083438 m` | packaged strict smoke PASS |
| Town07 / ClearNoon / lane_follow / seed 0 | `122 -> 107` | `0 / 0` | `0.003447 m` | packaged strict smoke PASS |
| Town10HD_Opt / ClearNoon / lane_follow / seed 1011 | `64 -> 49` | `0 / 0` | `0.001 m` | collector/export/visual smoke PASS |
| Town10HD_Opt / ClearNoon / lane_follow / seed 0 | `94 -> 79` | `0 / 0` | `0.000815 m` | formal suite 1/1 PASS |
| C-track / Epic / ClearNoon / lane_follow / seed 0 | `121 -> 106` | `0 / 0` | `0.035801 m` | packaged strict smoke PASS |
| 월악산 / Epic / ClearNoon / straight / seed 103 | `81 -> 66` | `0 / 0` | `0.346838 m` | packaged strict smoke PASS |

각 strict smoke는 한 맵의 한 route만 검증한다. 표준 Town05는 여섯 SceneCapture2D가
동작할 때 `FSkeletalMeshSceneProxy::GetMeshElementsConditionallySelectable`에서 renderer
SIGSEGV가 발생해 제외했고 Town05_Opt로 대체했다. C-track도 `Low`에서는 같은 skeletal
mesh rendering crash가 재현됐지만 `Epic` cold-start에서는 동일한 여섯 camera 수집을
완주했다. C-track의 skeletal show flag를 강제로 끄는 두 진단도 crash를 해결하지 못해
채택하지 않았다. 실패 실행은 `data/rejected/suites/`에 격리했다.

경로 catalog 생성기도 함께 보강했다. GlobalRoutePlanner가 spawn 앞뒤의 다른 lane으로
snap하거나 goal 뒤까지 overshoot한 구간은 시작점과 목표점에 가장 가까운 경로 구간으로
자르고, endpoint offset은 높이 `z`가 아닌 평면 `XY`로 측정한다. 기본
`--max-endpoint-offset 2`를 넘는 route는 거부한다. Collector는 BasicAgent의 `done()`만
성공으로 믿지 않고, 직렬화된 catalog goal까지의 남은 route와 base_link의 평면 거리가
모두 `--goal-tolerance-m` 안에 들어올 때 제동하고 성공으로 기록한다. BasicAgent가 그
전에 끝나면 실패다. 이 gate를 통해 Town05_Opt의 잘못 snap된 catalog, 월악산의 goal
overshoot와 Town03의 너무 엄격한 초기 `2.0 m` 판정을 격리한 뒤 위 결과만 채택했다.

- Town01: [`overview.png`](artifacts/training/2026-08-27/town01_right_pilot/overview.png), [`drive.gif`](artifacts/training/2026-08-27/town01_right_pilot/drive.gif)
- Town02_Opt: [`overview.png`](artifacts/training/2026-08-27/town02_opt_right_pilot/overview.png), [`drive.gif`](artifacts/training/2026-08-27/town02_opt_right_pilot/drive.gif)
- Town03 packaged: [`collection_plan.json`](data/training/suites/town03_packaged_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town03_packaged_lane_follow_smoke/town03/town03_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town03_packaged_lane_follow_smoke/town03/town03_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- Town04: [`collection_plan.json`](data/training/suites/town04_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town04_lane_follow_smoke/town04/town04_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town04_lane_follow_smoke/town04/town04_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- Town05_Opt: [`collection_plan.json`](data/training/suites/town05_opt_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town05_opt_lane_follow_smoke/town05_opt/town05_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town05_opt_lane_follow_smoke/town05_opt/town05_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- Town06: [`collection_plan.json`](data/training/suites/town06_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town06_lane_follow_smoke/town06/town06_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town06_lane_follow_smoke/town06/town06_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- Town07: [`collection_plan.json`](data/training/suites/town07_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town07_lane_follow_smoke/town07/town07_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town07_lane_follow_smoke/town07/town07_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- Town10HD_Opt: [`overview.png`](artifacts/training/2026-08-27/town10hd_opt_lane_follow_pilot/overview.png), [`drive.gif`](artifacts/training/2026-08-27/town10hd_opt_lane_follow_pilot/drive.gif)
- Town10HD_Opt formal suite: [`collection_plan.json`](data/training/suites/town10hd_opt_lane_follow_smoke/collection_plan.json), [`overview.png`](data/training/suites/town10hd_opt_lane_follow_smoke/town10hd_opt/town10hd_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/town10hd_opt_lane_follow_smoke/town10hd_opt/town10hd_opt_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- C-track packaged Epic: [`collection_plan.json`](data/training/suites/c_track_packaged_epic_smoke/collection_plan.json), [`overview.png`](data/training/suites/c_track_packaged_epic_smoke/c_track_1_0_7/c_track_1_0_7_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/overview.png), [`drive.gif`](data/training/suites/c_track_packaged_epic_smoke/c_track_1_0_7/c_track_1_0_7_lane_follow_s0000_p00/ClearNoon/seed_0000/preview/drive.gif)
- 월악산 packaged Epic: [`collection_plan.json`](data/training/suites/woraksan_packaged_straight_smoke/collection_plan.json), [`overview.png`](data/training/suites/woraksan_packaged_straight_smoke/woraksan_1_0_3/woraksan_1_0_3_straight_s0000_p00/ClearNoon/seed_0103/preview/overview.png), [`drive.gif`](data/training/suites/woraksan_packaged_straight_smoke/woraksan_1_0_3/woraksan_1_0_3_straight_s0000_p00/ClearNoon/seed_0103/preview/drive.gif)
- 격리 근거: [Town05 renderer crash](data/rejected/suites/town05_standard_renderer_crash/town05/town05_lane_follow_s0000_p00/ClearNoon/seed_0000/logs/collector.log), [C-track Low renderer crash](data/rejected/suites/c_track_packaged_renderer_crash_seed0/c_track_1_0_7/c_track_1_0_7_lane_follow_s0000_p00/ClearNoon/seed_0000/logs/collector.log), [Town05_Opt bad endpoint catalog](data/rejected/suites/town05_opt_bad_endpoint_catalog/route_catalog.json), [월악산 overshoot](data/rejected/suites/woraksan_packaged_agent_overshoot/collection_plan.json), [Town03 2 m goal reject](data/rejected/suites/town03_goal_tolerance_2m_reject/collection_plan.json)

### 맵 범위와 실행 순서

packaged strict smoke가 끝난 범위는 Town01, Town02_Opt, Town03, Town04, Town05_Opt,
Town06, Town07, Town10HD_Opt, C-track과 월악산이다. 표준 Town02와 Town05는 이 설치의
renderer crash 때문에 각각 optimized variant로 대체한다. 표준 Town10HD는 아직 검증하지
않았으므로 Town10HD_Opt를 사용한다. Town08/09는 공개 배포되지 않은 Leaderboard unseen
map이고 로컬 level/OpenDRIVE asset도 없어 현재 수집할 수 없다. Town11은 undecorated
proof-of-concept이며 Town12/13/15는 source-editor profile과 별도 visual QA가 남아 있다.

이전 source-editor 실행에서 Town03, C-track과 월악산에 검은 하늘과 editor-only
`SkyAtmosphere` 경고가 보였지만, cooked packaged map을 `Epic`으로 cold-start한 이번
여섯 camera 영상에는 그 경고나 검은 하늘이 없다. 따라서 **현재 세 맵에는 사용자가
sky 설정, Blueprint/material 수정 또는 재패키징을 할 필요가 없다.** 아래의 generated
`Engine.ini`는 sky를 고치는 설정이 아니라 Shipping build가 시작할 map을 선택하는
bootstrap 설정이다. 이후 map asset을 바꾸면 같은 PNG/GIF visual gate를 다시 통과해야
하며, C-track은 `Low` renderer crash 때문에 계속 `Epic`을 사용한다.

### 로컬 custom map asset과 Cook 판단

재현 기준은 `scripts/e2e/custom_map_bundles.yaml`이다. 발견된 편집 원본과 현재
실행 bundle은 다음과 같다.

| 맵 | RoadRunner / OpenDRIVE | Unreal source level | Full Autoware source / bundle |
|---|---|---|---|
| C-track | `/home/hong/Documents/RoadRunner/Scenes/C_track_1_0_7.rrscene`, `Exports/C_track_1_0_7.xodr` | `/home/hong/carla/Unreal/CarlaUE4/Content/map_package/Maps/C_track_1_0_7/C_track_1_0_7.umap` | `/home/hong/camrod_ws/src/lanelet2_maps_(c_track_test).osm` + `/home/hong/carla-autoware-universe/op_carla/op_agent/autoware-contents/maps/C_track_1_0_7/pointcloud_map.pcd` -> `data/maps/C_track_1_0_7_full` |
| 월악산 | `/home/hong/Documents/RoadRunner/Woraksan/Scenes/Woraksan_v1_0_3_parking_lot_hegiht_fit.rrscene`, 같은 project의 export XODR | `/home/hong/carla/Unreal/CarlaUE4/Content/map_package/Maps/Woraksan_v1_0_3_parking_lot_hegiht_fit/Woraksan_v1_0_3_parking_lot_hegiht_fit.umap` | `/home/hong/camrod_ws/src/lanelet2_maps.osm` + `/home/hong/camrod_ws/src/util/flexcloud/georef_glim_worak_v2_binary.pcd` -> `data/maps/Woraksan_v1_0_3_parking_lot_hegiht_fit_full` |

현재 packaged CARLA 0.9.15에는 두 level과 동일 OpenDRIVE가 이미 Cook되어 있고,
Epic cold-start/6-camera/full VAD 주행이 통과했다. 따라서 **현재 asset을 그대로
시험할 때 Unreal sky 수정, import 또는 재-Cook은 필요 없다.** `.rrscene`, FBX,
XODR 또는 `.umap`을 실제로 수정한 경우에만 CARLA import/Cook/package 등록과 모든
visual/route gate를 다시 수행한다. 향후 월악산을 새 package로 Cook할 때는 source의
`map_package.Package.json`과 `DefaultGame.ini` 등록도 명시적으로 확인한다.

월악산 runtime OSM은 원본을 덮어쓰지 않는다. `setup_custom_full_map.py`가 relation
`4584`의 공유 boundary 방향만 project-owned 파생본으로 고치며, 원본 SHA-256과
파생본 SHA-256을 `map_bundle.json`에 함께 기록한다. RoadRunner의 오래된 v8
Lanelet 파일은 현재 확장 도로와 route lanelet `4574 -> 4584`가 없어 Full 주행용으로
교체하지 않는다.

### Packaged map cold-start

CARLA Shipping build는 positional map override를 무시하고 일부 맵에서
`client.load_world()`가 crash한다. `run_carla_map.sh`는 packaged `Content`에서 정확한
`.umap` 하나를 찾아 workspace-local `data/generated/carla_engine_ini/<MAP>.Engine.ini`를
만들고 `-EngineINI`로 cold-start한다. packaged shared `DefaultEngine.ini`는 수정하지
않으며 Python API가 실제 world 이름과 spawn 수를 읽을 때까지 기다린다.

```bash
cd /home/a/autoware_e2e

# 일반 packaged Town
scripts/e2e/run_carla_map.sh Town04 \
  --port 2100 --quality Low --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound

# source-editor sky 수정 없이 검증된 Epic packaged maps
scripts/e2e/run_carla_map.sh Town03 \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
scripts/e2e/run_carla_map.sh C_track_1_0_7 \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
scripts/e2e/run_carla_map.sh Woraksan_v1_0_3_parking_lot_hegiht_fit \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
```

위 명령은 CARLA가 종료될 때까지 foreground에서 기다린다. 맵을 바꿀 때는 먼저
`Ctrl-C`로 현재 packaged CARLA를 완전히 종료하고 다음 map 명령을 실행한다. Wrapper는
동시에 실행 중인 같은 packaged server가 있으면 시작을 거부한다. `-RenderOffScreen`은
RGB sensor를 렌더링하지만 `-nullrhi`는 그렇지 않으므로 expert/VAD 수집에 사용하지 않는다.

정적 inventory는 CARLA를 실행하지 않고 현재 계약과 asset을 검사한다.

```bash
python3 scripts/e2e/inventory_carla_training_maps.py \
  --output artifacts/inventory/carla_training_maps_static.json
```

경로 catalog는 **해당 맵으로 이미 시작했고 actor가 없는** CARLA server 한 개에 대해
map profile별로 따로 만든다. 아래는 packaged 기본 맵 Town10HD_Opt 예시다. 기본 동작은 현재 server map이
manifest의 Town10HD_Opt와 일치하는지 확인하고 10초 동안 renderer가 안정되기를 기다릴 뿐,
`load_world`를 호출하지 않는다.

```bash
source scripts/e2e/env.sh

python3 scripts/e2e/prepare_carla_expert_route_catalog.py \
  --map-id town10hd_opt \
  --output-root data/training/catalogs/town10hd_opt \
  --active-server-profile packaged_0915 \
  --max-endpoint-offset 2 \
  --host 127.0.0.1 --port 2100
```

suite는 기본이 dry-run이며 CARLA에 접속하거나 수집하지 않고 job 수와 예상 용량만
`collection_plan.json`에 쓴다. 아래 명령은 전체 matrix가 아니라 확인된
`lane_follow/ClearNoon/seed 0` 한 건만 선택한다. plan을 먼저 확인한 뒤 같은 profile을
명시하고 `--execute`를 추가해야 collector, exporter, renderer가 순서대로 실행된다.
exporter는 기본적으로 차선 침범을 한 건도 허용하지 않으며, 품질 기준을 의도적으로
완화하는 실험에서만 `--maximum-lane-invasions N`을 exporter에 직접 지정한다.

```bash
python3 scripts/e2e/run_carla_expert_collection_suite.py \
  --catalog data/training/catalogs/town10hd_opt/route_catalog.json \
  --output-root data/training/suites/town10hd_opt_lane_follow_smoke \
  --active-server-profile packaged_0915 \
  --host 127.0.0.1 --port 2100 \
  --scenarios lane_follow --weathers ClearNoon --seeds 0 \
  --fail-fast

python3 scripts/e2e/run_carla_expert_collection_suite.py \
  --catalog data/training/catalogs/town10hd_opt/route_catalog.json \
  --output-root data/training/suites/town10hd_opt_lane_follow_smoke \
  --active-server-profile packaged_0915 \
  --host 127.0.0.1 --port 2100 \
  --scenarios lane_follow --weathers ClearNoon --seeds 0 \
  --execute --fail-fast
```

2026-08-27 실제 실행은 `1/1 COMPLETE`, 예상 `0.306 GiB`, 실제 `88 MB`였고 목표 도달,
`374` state, `94` camera anchor, `79` validated sample을 기록했다. 충돌과 차선 침범은
모두 0건이며 exporter의 최대 route CTE는 `0.000815 m`였다. 같은 output root로 다시
실행한 검증도 `SKIP_RESUME_VALIDATED`, 추가 예상량 `0 GiB`로 끝나 기존 데이터를
덮어쓰지 않았다.

서로 다른 server profile catalog를 한 실행에 섞으면 runner가 차단한다. 현재 검증한
Town03, C-track과 월악산은 모두 `packaged_0915`다. Runner는 외부 CARLA server를
시작하거나 종료하지 않고 기본적으로 map도 바꾸지 않는다. 현재 맵이 다르면 actor를
만들기 전에 실패한다. `--allow-map-load`는 renderer SIGSEGV가 재현된 위험한 진단
옵션이므로 일반 수집에서는 사용하지 않는다. 맵마다 server를 종료한 뒤 위 cold-start
wrapper로 정확한 level과 quality를 시작한다. Town11/12/13/15 같은 source-only 후보를
시험할 때만 `source_editor_0915_4ws`와 별도 source visual gate를 사용한다.

월악산 catalog는 확인된 불량 spawn 4를 양쪽 후보에서 제외한다.

```bash
python3 scripts/e2e/prepare_carla_expert_route_catalog.py \
  --map-id woraksan_1_0_3 \
  --output-root data/training/catalogs/woraksan_packaged_safe_smoke \
  --active-server-profile packaged_0915 \
  --seeds 0 --pairs-per-seed 1 \
  --max-endpoint-offset 2 \
  --exclude-spawn-indices 4 \
  --host 127.0.0.1 --port 2100
```

외장 저장장치가 준비되면 처음부터 안정된 mount 경로를 정하고 `--output-root`만 바꾼다.
예를 들어 mount가 `/media/hong/E2E_DATA`라면 suite root는
`/media/hong/E2E_DATA/autoware_e2e_training/suites/town10hd_opt`처럼 둔다. Raw episode와
export manifest는 서로 연결되므로 bulk 수집 뒤 한쪽만 따로 이동하지 않는다. B2D
점검 명령의 `--dataset-root`는 이미 받은 upstream depth layout을 검사하는 옵션이지
다운로드 위치를 정하는 옵션이 아니다.

현재 pilot schema는 640x360 JPEG, 5 Hz anchor, planning future label만 포함한다.
Bench2Drive Full의 10 Hz info pickle, depth/object/vector-map annotation과 같지 않으므로
upstream loader에 그대로 넣을 수 없다. 공개 B2D camera order adapter와 Autoware Tiny
camera order는 exporter에 기록돼 있지만, 학습 전에는 별도 loader와 5 Hz/10 Hz 정책을
구현해야 한다.

## 역사 자료: VAD raw 경로 진단 (2026-08-27)

우회전 bag을 `/planning/vad_route/selected_raw_trajectory`, 최종
`/planning/trajectory`, MPC predicted trajectory, 실제 odometry 순서로 다시
분해했다. 결론은 **제어기가 정상 경로를 크게 놓치는 문제가 아니라 VAD planning
head가 처음부터 S자와 2.5-3.0초 horizon 편향을 내는 문제**다.

- 선택된 raw trajectory 176/176개가 route command와 해당 candidate branch에
  정확히 일치했다. 우회전 핵심 구간에서는 `RIGHT` branch가 여섯 candidate 중
  가장 나았으므로 command index를 바꾸는 해법은 아니다.
- 출력 축, 부호, delta 누적, rear-axle `base_link`, camera mirror, causal state sync를
  각각 검증했다. 잘못된 x/y 해석이나 double-cumsum 문제는 아니었다.
- 핵심 우회전 raw snapshot의 91-100%가 한 horizon 안에 양/음 곡률을 동시에
  포함했다. 이 때문에 앞에서는 한쪽으로 갔다가 뒤에서 반대로 꺾이는 S자가 생긴다.
- legacy 두 반복의 raw route-offset p95는 `3.11/2.48 m`, raw-to-final correction
  p95는 둘 다 `4.13 m`, 최대 corridor correction은 `8.75/9.65 m`였다.
- 반면 MPC actual-to-final p95는 `0.11/0.16 m`, 실제 route CTE max는
  `0.35/0.31 m`였다. 차량이 완주한 것은 MPC가 나쁜 raw 경로를 잘 따라가서가
  아니라 route manager가 먼저 raw 경로를 corridor 안으로 크게 고쳤기 때문이다.
- VAD auxiliary `Center` map head는 OpenDRIVE centerline 대비 median 약 `0.74 m`,
  frame-to-frame Chamfer median 약 `0.26-0.29 m`였다. 전체 BEV perception이
  붕괴한 상태보다는 planning head의 far-horizon/domain 문제가 더 크다.

따라서 현재 성공 주행은 `pure VAD`가 아니라
`VAD + route-manager safety correction + Autoware MPC` hybrid다. MPC의
`input_delay/tau`를 더 바꾸어도 raw S자는 없어지지 않는다.

### 런타임 수정 A/B 판정

동일한 Town01 우회전, camera, 속도, MPC, corridor 조건에서 다음을 실제 폐루프로
실행했다. 모든 유효 run은 `goal reached`였지만 모든 진단은 `path_dominant`였다.

| 변경 | raw pooled p95 | raw snapshot p95 | raw-to-final p95 | 최대 correction | 판정 |
|---|---:|---:|---:|---:|---|
| temporal + legacy, 2회 | `3.11/2.48 m` | `4.33/4.51 m` | `4.13/4.13 m` | `8.75/9.65 m` | 기준 유지 |
| no-temporal | `2.65 m` | `5.08 m` | `4.74 m` | `13.55 m` | 기각 |
| `ros_corrected` BEV shift | `2.38 m` | `4.43 m` | `4.15 m` | `9.45 m` | 기각 |
| zero BEV shift, 1회 | `2.30 m` | `3.69 m` | `3.41 m` | `9.43 m` | 재현성/정지 jitter 부족, 미채택 |
| Rotate center `[100,53]`, 2회 | `2.55/2.54 m` | `4.31/3.33 m` | `3.94/2.98 m` | `9.43/8.19 m` | 장기 horizon/S자 잔존, 미채택 |
| Rotate center + zero shift | `2.51 m` | `3.84 m` | `3.64 m` | `9.79 m` | 기각 |

`no-temporal`은 0.5-1.5초만 좋아지고 3초 median CTE가 `5.05 m`로 악화됐으며,
auxiliary Center map Chamfer median도 legacy `0.28-0.29 m`에서 `1.08 m`로
무너졌다. Rotate center 2-vs-2 평균은 일부 pooled 지표가 9-16% 낮아졌지만 3초
core p95는 `7.21 -> 7.42 m`, S자 발생률은 `100% -> 95%`로 제거되지 않았다.
따라서 현재 채택값은 다음과 같다.

```text
use_temporal_head=true
vad_bev_shift_mode=legacy
official temporal head 유지
MPC input_delay=0.12 s, steer_tau=0.15 s
```

RotatePlugin 검증용 ONNX는 공식 파일을 덮어쓰지 않는다. 다음 생성기는 공식 SHA,
BEV `[1,256,106,200]`, initializer 한 개의 `[100,100] -> [100,53]` raw-data 변경만
허용하고 manifest를 남긴다. 폐루프 결과가 채택 기준을 통과하지 못했으므로 일반
실행 profile에는 연결하지 않았다.

```bash
scripts/e2e/generate_vad_rotate_center_variant.py
scripts/e2e/generate_vad_rotate_center_variant.py --validate-only
```

비교 이미지는 다음 세 파일에서 바로 확인한다.

- [`temporal_ab.png`](artifacts/summary/2026-08-27/vad_path_contract/temporal_ab.png)
- [`bev_shift_ab.png`](artifacts/summary/2026-08-27/vad_path_contract/bev_shift_ab.png)
- [`rotate_center_ab.png`](artifacts/summary/2026-08-27/vad_path_contract/rotate_center_ab.png)

각 run의 `route_result.png`, `path_vs_control.png`, `turn_path_control.gif`는
`artifacts/runs/2026-08-27/vad_path_contract/` 아래에 있다. 출발하지 못한 dry-steer
실행과 byte-minimal 계약 전 Rotate variant는 삭제하지 않고
`artifacts/diagnostics/2026-08-27_invalid_trials/vad_path_contract/`로 분리했다.

### raw 경로를 실제로 고치는 순서

공식 Autoware CARLA Tiny 배포물은 PyTorch checkpoint가 아니라 backbone/head
ONNX와 TensorRT deployment metadata만 공개한다. ONNX에는 optimizer, gradient,
training state가 없고 private Tiny 구조는 공개 Bench2Drive VAD-Base와 BEV 크기,
decoder 수, camera 순서가 달라 **현재 ONNX를 그대로 fine-tune할 수 없다**.
[공식 model card](https://huggingface.co/AutowareFoundation/tensorrt_vad/blob/v0.1/README.md)도
이 모델을 CARLA simulation용으로 한정하고 실제 차량용으로 보지 않는다.

`pure VAD`를 목표로 할 때의 주 경로는 공개
[Bench2DriveZoo VAD](https://github.com/Thinklab-SJTU/Bench2DriveZoo/tree/uniad/vad)를
별도 PyTorch 모델로 가져와 planning head를 먼저 fine-tune하고, Autoware용 camera,
CAN, coordinate, temporal/no-prev ONNX export와 TensorRT adapter를 새로 만드는 것이다.
공개 Base 모델은 현재 Tiny ONNX의 drop-in replacement가 아니다.

가장 빠른 공학적 대안은 기존 VAD를 freeze하고 raw 6-point trajectory, command,
ego state, local route를 입력으로 받는 작은 learned residual corrector를 VAD와 MPC
사이에 두는 것이다. 이 방법은 full perception label 없이 우회전 오차를 직접
학습할 수 있지만 route-dependent hybrid이므로 pure VAD라고 부르면 안 된다.

어느 학습 경로든 현재 VAD 폐루프 odometry를 정답으로 다시 쓰면 안 된다. 잘못된
VAD 경로가 label로 복제된다. CARLA BasicAgent/conventional planner/검증된 수동
운전이 ego를 제어해야 한다. VAD shadow inference는 추가할 수 있지만 현재 expert
collector에는 아직 연결하지 않았다. 각 6-camera anchor에 대해 0.5초 간격 3초 future
expert pose, command, camera calibration/source timestamp, controller source,
collision/lane-invasion, episode boundary를 저장한다. 현재 CARLA direct JPEG/JSONL
collector, sample exporter와 PNG/GIF renderer까지 구현됐고, 실차 쪽은
`record_vad_training_data.sh`의 raw-bag contract가 있다. Dataset loader, PyTorch
fine-tune, ONNX parity 단계는 아직 없다.

구현 우선순위는 다음과 같다.

1. 현재 collector/exporter로 맵, 날씨, straight/left/right expert episode를 늘리고 visual QA를 통과시킨다.
2. 공개 B2D 학습 code가 읽는 planning-only dataset loader와 5 Hz/10 Hz sampling 정책을 만든다.
3. Town/weather/scenario 단위로 split해 open-loop ADE/FDE/CTE/곡률을 평가한다.
4. pure VAD면 공개 B2D planning head를 fine-tune하고, 빠른 hybrid면 residual corrector를 학습한다.
5. ONNX/PyTorch/TensorRT parity 뒤 held-out CARLA 폐루프를 통과한 모델만 실제차 shadow mode로 옮긴다.

## 역사 자료: 2026-08-27 권장 quick-start

> 이 절은 2026-08-27 환경의 재현용 snapshot이다. 절 안의 `/home/hong/autoware_e2e`
> 경로와 `--recommended` 단독 `maximum_speed_mps=2.5` 구성은 당시 값으로 보존한다.
> 현재 workspace는 `/home/a/autoware_e2e`이며, 2026-09-01 v16 30 kph screening은
> `--recommended --speed-30kph`를 함께 사용한 별도 profile이다. 현재 결과와 실행
> 경계는 문서 맨 앞의 2026-09-01 절을 사용하고, 아래 명령을 현재 진입점으로
> 그대로 복사하지 않는다.

당시 운용 기준은 `run_route_vad_fast.sh --recommended`였다. 이 option은 minimal
launcher가 아니라 **Full Autoware shell, 표준 control/API/RViz와 TensorRT VAD를
합친 profile**을 선택한다. `--visualize`를 함께 쓰면 Autoware RViz와 전방 camera
화면이 같이 열린다. 아래 예시는 이전 코드판에서 완주한 Town01 좌회전 route이며, 직진과 우회전은
각각 `town01_fast_lane_follow_clear_noon.json`,
`town01_fast_right_clear_noon.json`으로 바꾸면 된다.

> **당시 코드판 검증 상태:** geometry smoothing fixed-point 수정 뒤
> `--recommended` 우회전 3/3과 좌회전 non-regression 1/1이 `goal reached`로
> 통과했다. 우회전 outlier filter는 별도 3/3 완주에도 곡률과 조향 p95가 악화되어
> `HOLD`다. 일반 실행에는 `--trajectory-stability`를 붙이지 않는다.

2026-08-27 재부팅 후 loaded NVIDIA kernel module과 system user-space library가 모두
`580.178.04`로 맞았고 native `nvidia-smi`, CARLA, TensorRT VAD 주행을 통과했다.
과거 shell에 임시 우회가 남지 않도록 **새 terminal마다 compatibility 변수를 unset**하고
Terminal 1에서 CARLA 2100을 먼저 시작한다.

```bash
cd /home/hong/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT
export CARLA_PORT=2100

scripts/e2e/run_carla.sh \
  -RenderOffScreen \
  -nosound \
  -carla-port=2100
```

Terminal 2에서 Full Autoware + VAD + UI를 시작한다. `--recommended`가 `--full`을
자동 적용하므로 `--full`을 다시 적을 필요는 없다.

```bash
cd /home/hong/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT

scripts/e2e/run_route_vad_fast.sh --recommended --visualize \
  data/routes/town01_fast_left_clear_noon.json \
  carla_port:=2100
```

이 시점에는 stack과 화면이 준비된 것이며 차량은 아직 자동 engage하지 않는다.
실제 주행과 PASS/FAIL 저장은 Terminal 3에서 evaluator를 실행한다.

```bash
cd /home/hong/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT

scripts/e2e/route_test.sh \
  --full-stack \
  --route-file data/routes/town01_fast_left_clear_noon.json \
  --result artifacts/runs/$(date +%F)/my_recommended_left/result.json
```

UI 없이 주행, rosbag, evaluator, 분석 PNG/GIF와 cleanup을 한 번에 재현할 때는
Terminal 1의 CARLA를 실행해 둔 상태에서 다른 terminal에 다음 한 명령만 실행한다.
`OUTPUT_DIR`은 새 경로여야 한다. 이 helper는 자신이 시작한 Autoware stack과 recorder만
종료하며 CARLA server는 다음 episode를 위해 남긴다.

```bash
cd /home/hong/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT
export CARLA_PORT=2100

scripts/e2e/run_recorded_route_trial.sh --recommended \
  artifacts/runs/$(date +%F)/my_recommended_left \
  data/routes/town01_fast_left_clear_noon.json \
  carla_port:=2100 carla_timeout:=60
```

결과 directory에는 `result.json`, compact rosbag, `diagnosis.json`, latency JSON/PNG,
`route_result.png`, `path_vs_control.png`, `steering_tracking.png`,
`turn_path_control.gif`, model override/sensor/MPC/actuation 설정과 SHA-256이 함께
저장된다. ONNX/TensorRT weight 자체는 용량 때문에 복사하지 않으며, 실행 전
`scripts/e2e/doctor.sh`의 공식 model hash 검사로 무결성을 확인한다.

`--recommended`가 고정하는 기준 구성은 다음과 같다. 이 값을 command line에서
개별 override하면 wrapper가 실행을 거부하므로, 변경 시험은 별도 profile과 artifact로
분리한다.

| 계층 | 2026-08-27 당시 권장 구성 |
|---|---|
| Autoware | Full vehicle/system/map/sensing/control/API shell + VAD planning/object + RViz |
| camera | `640x360`, 5 Hz, raw 6-camera, DDS `reliable`; CARLA frame ID로 capture timestamp를 복원한 완전한 6장 bundle 발행 |
| VAD sync | 같은 timestamp의 6장을 우선하는 exact-first assembler, 허용 오차 안의 bounded approximate fallback, frame/image queue `32/32` |
| VAD precision | `carla_tiny` backbone FP16 + temporal/no-prev head FP32 |
| 경로 후처리 | command lookahead `3.0 m`, turn inward corridor `0.20 m`, endpoint-fixed smoothing strength `10.0` |
| 속도/정지 | trajectory maximum speed `2.5 m/s`, terminal comfortable deceleration `0.60 m/s^2`, controller stop offset `0.60 m` |
| 제어 | 표준 Autoware lateral MPC `input_delay=0.12 s`, `vehicle_model_steer_tau=0.15 s` + longitudinal PID. CARLA `--recommended`만 정지 중 조향 수렴 대기를 해제하며 실제차 launch 기본값은 유지 |
| actuation | Autoware CARLA interface의 **stock** accel/brake/steer map |

`turn_inward_corridor_half_width_m=0.20`은 회전 안쪽 경계를 제한하는 route-manager
값이지 VAD network weight를 바꾸는 값이 아니다. Lookahead는 여섯 command candidate
중 어느 branch를 선택할지 바꾸며, smoothing은 선택 뒤 final trajectory geometry에만
적용된다. 따라서 이 세 값은 재학습 없이 경로 출력을 바꾸지만 raw VAD model 자체를
개선한 것으로 해석하면 안 된다.

### 2026-08-27 당시 반복 폐루프 결과

재부팅 후 native NVIDIA `580.178.04`에서 위 권장 구성을 실행한 당시 결과다. 좌회전은
기존 bridge timestamp 경로, 직진과 우회전은 CARLA 0.9.15 GPU callback의 한-tick
header race를 frame ID로 보정한 최종 경로다. 세 경로 모두 Full evaluator의
`goal reached`로 PASS했다.

| 2026-08-27 native run | PASS | evaluator CTE max (m) | route CTE max / p95 (m) | actual→final max / p95 (m) | final steer peak (rad) | 목표 잔여 (m) | sim / wall (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 직진, frame-stamp fix | 1/1 | 0.162 | 0.162 / 0.158 | 0.0021 / 0.0010 | 0.0060 | 0.782 | 12.90 / 12.92 |
| 좌회전 | 1/1 | 0.715 | 0.723 / 0.588 | 0.520 / 0.390 | 0.601 | 0.820 | 21.10 / 21.16 |
| 우회전, frame-stamp fix | 1/1 | 0.953 | 0.955 / 0.710 | 0.747 / 0.513 | 0.595 | 0.840 | 31.60 / 31.67 |

직진의 controller 전달 horizon은 XY corridor/smoothing correction 최대
`0.000716 m`(<1 mm), raw→final p95/max `0.000594/0.000886 m`로
`within_thresholds`였다. 즉 XY geometry는 `vad_unassisted`였지만 route manager가 goal
밖의 VAD horizon을 자르고 속도를 shaping하므로 전체 planning 구조는 계속 hybrid다.
최종 우회전 bag은 6-camera bundle coverage `100%`, stamp span 최대
`0 ms`, bridge bundle reject `0`이었다. TensorRT inference 중앙값은 `87.633 ms`였다.
한 번의 반복은
카메라가 아니라 VAD raw lane-follow horizon의 trajectory correction이 `15.116 m`로
안전 한계 `15.0 m`를 넘어서 FAIL했고 `invalid/`에 보존했다. 따라서 당시 PASS도
VAD 출력 변동성이 사라졌다는 뜻은 아니다.

2026-08-25 재부팅 전에는 matching compatibility library를 사용한 CARLA server와
권장 구성을 고정하고
`Town01/ClearNoon` 직진, 좌회전, 우회전을 각각 독립 2회 실행했다. 여섯 실행 모두
Full evaluator의 `goal reached`로 **2/2 PASS**했다. `/planning/trajectory` 추종 오차와
reference route 오차는 서로 다른 지표이므로 함께 표시한다.

| Case / screen | PASS | route CTE max (m) | route CTE p95 (m) | actual→final max (m) | actual→final p95 (m) | steer command peak (rad) | 목표 잔여 (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 직진 25 / 26 | 2/2 | 0.166 / 0.166 | 0.162 / 0.162 | 0.0009 / 0.0019 | 0.0005 / 0.0008 | 0.0071 / 0.0065 | 0.807 / 0.811 |
| 좌회전 32 / 33 | 2/2 | 0.737 / 0.726 | 0.669 / 0.662 | 0.536 / 0.521 | 0.466 / 0.460 | 0.563 / 0.554 | 0.798 / 0.787 |
| 우회전 30 / 31 | 2/2 | 0.997 / 0.990 | 0.788 / 0.685 | 0.793 / 0.783 | 0.592 / 0.491 | 0.516 / 0.581 | 0.834 / 0.858 |

새 install space를 만든 뒤 wrapper 자체도 다음과 같은 실제 진입점으로 별도 통합
검증했다.

```bash
scripts/e2e/run_route_vad_fast.sh --recommended \
  data/routes/town01_fast_left_clear_noon.json \
  carla_port:=2100 carla_timeout:=60
```

screen 35는 `LEFT`와 `LANE_FOLLOW` command를 모두 사용하고 stable STOP까지
`goal reached`로 PASS했다. 최대 route CTE `0.752916 m`, 목표 잔여
`0.811363 m`, simulation/wall `21.300/21.341 s`였다. 즉 위 32/33 수동 조합과 같은
값을 설치된 `--recommended` wrapper가 실제로 전달하는 것까지 확인했다.

실제 차량 속도 peak는 직진 `2.867/2.869 m/s`, 좌회전
`2.907/2.906 m/s`, 우회전 `2.906/2.907 m/s`였다. `maximum_speed_mps=2.5`는
trajectory target 상한이며 실제 vehicle response peak와 같은 값이 아니다. 우회전은
완주 재현성은 확보했지만 route CTE가 약 `1.0 m`이므로 추가 개선 대상이다.
좌/우회전에는 raw VAD trajectory를 route corridor로 보정한 구간이 있으므로 판정은
여전히 `vad_route_manager_hybrid`다. `2/2 PASS`를 순수 VAD geometry 또는 임의의
새 환경에 대한 일반화 증명으로 해석하지 않는다.

당시 비교 화면과 animation은 다음 위치에 있다.

- 재부팅 후 native 좌회전: `artifacts/runs/2026-08-27/post_reboot/town01_left_recommended_native_580178/`
- 재부팅 후 frame-stamp 직진: `artifacts/runs/2026-08-27/post_reboot/town01_straight_recommended_native_580178/`
- 재부팅 후 frame-stamp 우회전: `artifacts/runs/2026-08-27/post_reboot/town01_right_recommended_native_580178/`
- timestamp 원인 분리 실패군: `artifacts/runs/2026-08-27/post_reboot/invalid/`
- `sensor_tick=0` 대조 성공군: `artifacts/runs/2026-08-27/post_reboot/experiments/town01_right_recommended_sensor_tick0_native_580178/`
- 전체 비교 PNG: `artifacts/runs/2026-08-25/generalization/generalization_comparison.png`
- 직진 PNG: `artifacts/runs/2026-08-25/generalization/town01_lane_follow_recommended_stock_repeat02_screen26/route_result.png`
- 좌회전 GIF/PNG directory: `artifacts/runs/2026-08-25/generalization/town01_left_recommended_inward020_stock_repeat02_screen33/`의 `turn_path_control.gif`, `route_result.png`, `path_vs_control.png`
- 설치 wrapper 통합 PNG: `artifacts/runs/2026-08-25/generalization/town01_left_recommended_wrapper_integration_screen35/route_result.png`
- 우회전 GIF/PNG directory: `artifacts/runs/2026-08-25/generalization/town01_right_recommended_inward020_stock_repeat02_screen31/`의 `turn_path_control.gif`, `route_result.png`, `path_vs_control.png`
- 전체 수치 원본: `artifacts/runs/2026-08-25/generalization/generalization_summary.json`

### 2026-08-27 회전 경로 추종 재분해와 최종 A/B 판정

재부팅 후 native 좌/우회전 bag을 같은 signed Frenet 좌표로 다시 분해했다. 현재 bridge는
`/vehicle/status/steering_status`에 이미 virtual tire angle을 발행하므로 새 기록 helper는
분석기에 `--steering-report-mode virtual`을 자동 전달한다. 과거 기본값
`legacy_fl`로 분석한 조향 fit은 이 각도를 한 번 더 변환한 값이므로 CTE 판단에는 영향이
없지만 조향 지연/이득 판단에는 사용하지 않는다.

| 항목 | 좌회전 | 우회전 |
|---|---:|---:|
| peak actual→route | `+0.723 m` | `-0.955 m` |
| 같은 station의 final→route | `+0.201 m` | `-0.204 m` |
| actual→final tracking residual | `+0.522 m` | `-0.750 m` |
| raw 기준 residual | `+0.086 m` | `-0.060 m` |
| final offset, current→5 m | `+0.201→-0.465 m` | `-0.204→+0.327 m` |

우회전 peak의 final path 자체는 route에서 약 `0.20 m` 안쪽이고 실제 차량은 약
`0.95 m` 안쪽이다. 따라서 이 시점 오차의 약 `21%`는 final path 형상이고 약 `79%`는
actual→final tracking residual이다. **경로를 전혀 다른 선으로 잘못 추종하는 상태가
아니라, 조금 안쪽인 갱신 경로에 조향 phase 오차가 더해진 상태**다. 진입에서는 실측
조향이 command보다 `0.029~0.080 rad` 덜 꺾이고, apex/탈출에서는 반대로
`0.040~0.076 rad` 더 꺾인 채 남았다. 즉 늦게 들어가고 늦게 풀려 안쪽으로 말린다.

Raw VAD는 current에서 차량과 가까웠지만 미래 5 m 안에서 route 반대편으로 넘어가며,
연속 final trajectory update의 공통 horizon 최대 jump도 좌 `0.675 m`, 우
`0.700 m`였다. 고정된 동일 final trajectory를 사용한 MPC replay 오차는 live 오차의
약 1/4~1/5이므로 controller gain 하나만 바꾸기 전에 trajectory update 안정성도 함께
검증해야 한다.

Raw point0와 callback 직전 odometry 차이는 turn 구간 p95가 좌 `0.144 m`, 우 약
`0.27 m`였다. 따라서 전체 궤적을 current ego로 rigid SE(2) 이동하는 보정은 채택하지
않았다. 이 보정은 hard corridor가 point0를 다시 떼어 놓아 peak tracking error와 5 m
부호 교차를 없애지 못했고 좌회전 curvature/jump는 오히려 증가했다.

Route manager에는 다음 실험 장치를 추가했지만 기본 동작은 그대로다.

- `left/right_turn_outward_corridor_half_width_m`: 좌/우 바깥쪽 한계를 독립 설정한다.
  `0.0`은 공통값 상속이며 기존 동작과 같다.
- `route_corridor_entry_distance_m`: raw point0부터 hard corridor까지 quintic ramp를 만든다.
  기본 `0.0 m`는 기존 즉시 clamp와 정확히 같다. Offline에서 `5 m` ramp는 first gap을
  줄였지만 0→5 m offset 변화를 좌 `0.65→1.13 m`, 우 `0.45→0.93 m`로 키워 1순위
  폐루프 후보에서는 끈다.
- Geometry smoothing의 fixed point는 두 번째 corridor pass에서 다시 이동시키지 않고
  같은 envelope 안인지 검증한다. 기존에는 `0.1~3.4 mm` 재이동 뒤 fixed-point guard가
  smoothing 전체를 버리는 경우가 있었다.
- `left/right_turn_trajectory_lateral_filter_gain`과
  `trajectory_lateral_filter_activation_threshold_m`은 특정 방향에서 update jump가
  threshold를 넘을 때만 이전 route-offset profile과 혼합한다. 기본 gain `1.0`과
  threshold `0.0`은 기존 trajectory를 바꾸지 않는다.

최종 `--trajectory-stability` 실험은 `--recommended`에 다음 두 값만 추가했다.

```text
right_turn_trajectory_lateral_filter_gain=0.75
trajectory_lateral_filter_activation_threshold_m=0.20
```

Offline 우회전 3-bag replay에서는 update jump p95/max가
`0.694/0.700 → 0.604/0.617 m`, curvature p95/max가
`0.339/1.161 → 0.332/0.993 1/m`로 줄어 이 값을 폐루프로 올렸다. 같은 CARLA server에서
baseline과 후보를 번갈아 각각 3회 기록한 최종 결과는 다음과 같다.

| 우회전 3회 평균 | `--recommended` | outlier gate | 변화 |
|---|---:|---:|---:|
| goal reached | 3/3 | 3/3 | 동일 |
| route CTE max | `0.9705 m` | `0.9492 m` | `-2.2%` |
| route CTE p95 | `0.6773 m` | `0.6647 m` | `-1.9%` |
| actual→final CTE p95 | `0.4714 m` | `0.4604 m` | `-2.3%` |
| final-path curvature p95 | `0.3415 1/m` | `0.3612 1/m` | `+5.8%` 악화 |
| steer command p95 | `0.3229 rad` | `0.3447 rad` | `+6.8%` 악화 |
| actual→final yaw p95 | `0.2089 rad` | `0.1933 rad` | `-7.5%` |

CTE와 yaw는 조금 좋아졌지만 path curvature와 조향 p95 비악화 gate를 통과하지 못했다.
따라서 outlier gate는 **`HOLD`**, 일반 실행은 계속 **`--recommended`**다. outlier gate의
curvature 회귀는 MPC로 고칠 수 없고, frozen replay의 steering cap `0.65`도 command
peak를 `2.29%` 줄이는 대신 CTE RMS `6.16%`, yaw p95 `2.20%`를 악화시켜 기각했다.

다른 단일 변경도 채택 기준을 통과하지 못했다.

- 우회전 outward corridor `0.30 m`: 반복 CTE 개선이 재현되지 않았고 좌회전은 악화됐다.
- MPC `input_delay/tau=0.03/0.15`: frozen replay 1위였지만 실제 CARLA에서 route p95
  `0.6970 m`, steer p95 `0.3533 rad`로 권장 반복 평균보다 나빠졌다.
- curvature speed cap `0.8 m/s²`: route/final p95가 `0.8901/0.6878 m`로 악화됐다.
- 단순 nearest route-station 재투영: station bunching과 curvature/jump가 늘었다.

0→5 m lateral offset의 부호가 바뀌는 것은 진입/탈출의
`outside → inside → outside` corner-cut일 수 있다. 이를 무조건 금지하면 VAD geometry를
route-center planner로 바꾸므로 crossing 횟수 자체는 결함 판정이나 채택 gate로 쓰지
않는다. 대신 update jump, curvature, 요구 조향각과 실제 CTE를 함께 본다.

일반 재현은 다음 명령을 사용한다.

```bash
export CARLA_HOST=localhost CARLA_PORT=2100 ROS_DOMAIN_ID=42

scripts/e2e/run_recorded_route_trial.sh --recommended \
  artifacts/runs/$(date +%F)/my_right_run \
  data/routes/town01_fast_right_clear_noon.json
```

`--trajectory-stability`는 실패를 재현하거나 후속 알고리즘을 비교할 때만 사용하며 실행
artifact의 `CLOSED_LOOP_VALIDATION_STATE=right_turn_repeat_screened_hold`로 기록된다.

당시 시각 자료:

- `artifacts/reports/2026-08-27_path_tracking/decision.png`: 3회 평균과 독립 run, 채택 gate
- `artifacts/reports/2026-08-27_path_tracking/comparison.png`: 7개 run별 수치 표
- `artifacts/reports/2026-08-27_path_tracking/baseline_vs_outlier_gate.gif`: 대표 우회전 비교
- `artifacts/reports/2026-08-27_path_tracking/baseline_vs_outlier_gate_keyframe.png`:
  같은 비교의 중간 회전 frame
- `artifacts/reports/2026-08-27_path_tracking/decision.json`: 집계 원본과 gate 판정
- 각 symlink directory의 `path_vs_control.png`, `steering_tracking.png`,
  `turn_path_control.gif`: 경로/제어 분해와 animation

두 CARLA를 동시에 GPU에서 실행한 두 trial은 camera가 1~2 frame 뒤 멈춰 성능 판정에서
제외하고 `artifacts/diagnostics/2026-08-27_invalid_trials/`로 옮겼다. 이 경우 OS 재부팅이
필수인 것은 아니다. 중복 CARLA를 종료하고 한 server만 다시 시작한 뒤 정상 주행했다.

### 채택하지 않은 후보와 무효 실행

| 후보 | 실제 결과 | 현재 판정 |
|---|---|---|
| full calibrated actuation, screen 21/22 | 2/2 goal reached지만 실제 속도 peak `2.944/2.940 m/s`, route p95 `0.782/0.792 m`, final p95 `0.486/0.496 m` | `REJECT`, stock map 유지 |
| 40% accel blend, screen 34 | 유효 1회 PASS지만 좌회전 stock 32/33보다 route/final/steer가 모두 소폭 악화, 속도 peak `2.915 m/s` | `HOLD/REJECT`, 반복 확대 전 기본값 사용 금지 |
| turn speed target `2.2 m/s`, screen 27/28 | 2/2 PASS와 조향 감소, 그러나 우회전 route p95 `0.755/0.836 m`와 final p95 `0.458/0.536 m`로 종합 개선 없음 | `REJECT`, 저속 안전 실험만 가능 |
| 표준 MPC `0.09/0.15`, 기타 timing/weight/rate/cap sweep | 일부 과거 route에서 trade-off가 있었지만 당시 frozen replay와 반복군을 함께 이기지 못함 | 과거 opt-in 결론 `SUPERSEDED` |
| Smart MPC nominal iLQR | 계산은 동작했지만 CARLA에서 route CTE `1.528 m`, goal `3.27 m` 전 stall | `REJECT/HOLD`, 연구용 |

screen 29의 40% blend 실행은 여섯 camera 모두 약 `4.0 s` 발행 gap이 생겨 perception
timeout과 MRM이 동작했다. Actuation 비교로 사용할 수 없으므로 유효 결과에서 제외하고
`artifacts/diagnostics/2026-08-25_invalid_trials/lookahead3_inward030_blend040_camera_gap_mrm_screen29/`
로 옮겼다. **재부팅 전 mismatch 상태에서는** CARLA를 compatibility library 없이 재시작하면 약 60초 뒤
`GameThread timed out waiting for RenderThread`와 exit 139가 재현됐다. 위
당시 kernel과 맞는 `580.173.02` compatibility root로 CARLA를 다시 시작한 뒤에는 60초 경계를 넘겼고,
당시 최종 반복 주행에서는 camera 최대 gap이 `0.40~0.60 s` 범위로 유지됐다. 따라서
screen 29는 model/control 실패가 아니라 GPU/render/sensor 입력이 깨진 **invalid trial**로
분류한다. 현재 native `580.178.04` 환경에는 이 과거 root를 적용하지 않는다.

다음 실행 경로 표도 **2026-08-27 당시 snapshot**이다. 표의 Town01/C-track/
Woraksan 검증 범위와 `그 밖의 route는 미검증` 문구는 현재 v16 상태가
아니다. 서로 다른 시점·구성의 검증 결과를 같은 결과로 해석하면 안 된다.

| 실행 경로 | 진입점 | 실제 구성 | 2026-08-27 당시 상태 |
|---|---|---|---|
| minimal E2E | `carla_vad.launch.xml`, `run_route_vad.sh` | CARLA bridge, truth state, VAD, route manager, 최소 MPC/PID와 command gate | 아래 4개 주행 결과까지 검증됨 |
| full Autoware shell | `carla_vad_full.launch.xml`, `run_route_vad_full.sh` | 표준 vehicle/system/map/sensing/control/API/RViz + VAD planning/object 출력 | Town01 baseline과 fast+Full C-track 고정 route 1/1, 월악산 고정 route 3/3 검증 완료 |
| fast VAD | `run_route_vad_fast.sh` | 640x360 raw 6-camera, 5 Hz; minimal/full/`--recommended` 선택 | Town01 직진/좌/우와 C-track/월악산의 위 고정 route PASS. 그 밖의 route/날씨/traffic은 미검증 |

기존 `carla_vad.launch.xml`은 `autoware_carla_interface`의
`use_e2e_planning=true` 경로를 사용한다. Upstream 구현 자체가 이 경로를
trajectory follower만 포함한 minimal control로 구성하므로, 전체
`autoware.launch.xml`을 실행한 결과가 아니다. 기존 Autoware에서 보던 Map,
Sensing, Localization, Perception, Planning, Control, System 패널과 module이
적게 보인 이유가 이것이다.

2026-08-23에 공식 릴리스를 기준으로 확인한 버전은 다음과 같다.

- Autoware stable 1.9.0, Universe/Launch 0.52.0
- `autoware_tensorrt_vad` ROS package 0.52.0
- AutowareFoundation TensorRT VAD `carla_tiny` model artifact tag v0.1
- CARLA 0.9.15
- ROS 2 Humble, Ubuntu 22.04

공식 자료:

- [Autoware 1.9.0 release](https://github.com/autowarefoundation/autoware/releases/tag/1.9.0)
- [TensorRT VAD 0.52.0 package](https://github.com/autowarefoundation/autoware_universe/tree/0.52.0/e2e/autoware_tensorrt_vad)
- [AutowareFoundation TensorRT VAD model v0.1](https://huggingface.co/AutowareFoundation/tensorrt_vad)
- [CARLA interface supported environment](https://github.com/autowarefoundation/autoware_universe/blob/0.52.0/simulator/autoware_carla_interface/README.md#supported-environment)

## 실험용 최소 부하 fast 실행

이 절의 기본 fast/minimal 명령은 GPU 부하 측정과 A/B 실험용이다. **현재 주행 권장은
문서 맨 앞의 `--recommended` Full profile**이며, 아래 2026-08-24 결과와 option은
그 이전 baseline 기록이다.

Route JSON과 CARLA 시작 방법은 아래 Full quick-start와 같다. 기존 2000번 CARLA를
종료한 뒤 2100번 CARLA 하나만 실행하고, route를 준비한 다음 기본 fast wrapper를
사용한다.

```bash
cd /home/a/autoware_e2e
export CARLA_PORT=2100

# 최소 Autoware control shell, RViz 없음
scripts/e2e/run_route_vad_fast.sh \
  data/routes/town01_lane_follow_clear_noon.json \
  carla_port:=2100
```

화면이 필요하면 별도 terminal에서 다음 명령을 먼저 띄운다. Fast profile은 합성기
자체를 끄므로 6-camera mosaic 대신 전방 raw camera를 표시한다.

```bash
scripts/e2e/run_visualization.sh --fast
```

표준 Autoware UI까지 한 번에 확인할 때만 다음 명령을 사용한다. 이 모드는 fast
camera/model 설정을 쓰지만 Full Autoware module과 RViz를 다시 켜므로 최대 경량
측정용은 아니다.

```bash
scripts/e2e/run_route_vad_fast.sh --full --visualize \
  data/routes/town01_lane_follow_clear_noon.json
```

Temporal head FP16은 별도 engine cache를 만드는 실험 옵션이다. 첫 실행은 engine
생성 때문에 느릴 수 있으며, baseline과 동일 route에서 trajectory/object 수치와
주행 결과를 비교하기 전에는 기본값으로 사용하지 않는다.

```bash
scripts/e2e/run_route_vad_fast.sh --fp16-heads \
  data/routes/town01_lane_follow_clear_noon.json \
  carla_port:=2100
```

주행 평가는 minimal fast에 기존 `route_test.sh`, full fast에 `--full-stack` evaluator를
사용할 수 있다. 다만 fast mapping은 사용하지 않는 LiDAR/IMU와 compressed camera
helper를 의도적으로 제거했으므로 기존 `validate_full_stack.py`의 baseline 56개 graph
검증과 같은 결과로 보고하면 안 된다.

### 2026-08-24 fast 직진 폐루프 실제 결과

`Town01`, `ClearNoon`, spawn 143→145의 12.08 m `LANE_FOLLOW` route에서 기본 fast
profile로 실제 CARLA-VAD-Autoware 폐루프를 실행했다. 실험용 `--fp16-heads`는 사용하지
않았으며 evaluator 결과는 `PASS (goal reached)`다.

| 항목 | 측정값 |
|---|---:|
| 시뮬레이션 / wall 시간 | 11.6 s / 45.4 s |
| 전체 real-time factor | 0.256 |
| 이동 / 목표 잔여 거리 | 11.04 m / 0.99 m |
| 최대 절대 CTE | 0.13 m |
| 최대 trajectory correction | 0.00 m |
| 녹화 | 640x360, 58 frame, simulation-time 5 fps |
| 실행 중 GPU 표본 | 64–66 C, 약 5.2–5.5 GB VRAM, thermal slowdown 비활성 |

결과 파일은 `artifacts/runs/2026-08-24/fast_lane_follow/`에 있다.

- `fast_vad_drive.gif`: 전방 camera의 실제 주행 전 구간
- `route_result.png`: route, 실제 궤적, PASS 지표
- `rviz_driving.png`: 주행 중 RViz와 VAD candidate trajectory
- `six_camera_inputs.png`: 같은 실행의 VAD 6-camera 입력
- `result.json`: evaluator 원본 수치와 84개 actual-path sample
- `frames/manifest.json`: GIF frame timestamp와 녹화 조건

### 2026-08-24 fast 좌/우회전 폐루프 실제 결과

같은 기본 fast profile과 `Town01/ClearNoon` 조건에서 교차로 좌회전과 우회전을 각각
실행했다. 두 case 모두 evaluator가 목표 정지까지 확인했지만 XY corridor correction이
개입했으므로 순수 VAD geometry `PASS`가 아니라 **`HYBRID PASS`**다.

| Case | Route | 판정 | 최대 CTE | 최대 보정 | 최종 잔여 | simulation / wall | RTF |
|---|---:|---|---:|---:|---:|---:|---:|
| LEFT 229→88 | 34.77 m | `HYBRID PASS` | 1.09 m | 10.94 m | 0.92 m | 19.85 / 78.65 s | 0.252 |
| RIGHT 235→58 | 59.47 m | `HYBRID PASS` | 1.24 m | 8.24 m | 0.99 m | 31.05 / 122.58 s | 0.253 |

좌회전 결과에는 `LEFT, LANE_FOLLOW`, 우회전 결과에는 `RIGHT, LANE_FOLLOW` command가
실제로 기록됐다. 실행 종료 전 GPU 표본은 56 C, 5,748 MiB, thermal slowdown
비활성이었다. 실험용 `--fp16-heads`는 사용하지 않았다.

결과 파일은 `artifacts/runs/2026-08-24/fast_turns/`에 있다.

- `turn_results.png`: 좌/우 reference와 actual path, 정량 판정을 합친 PNG
- `left/turn.gif`, `right/turn.gif`: 각 회전 핵심 구간 640x360 GIF
- `left/drive.gif`, `right/drive.gif`: 접근부터 목표 정지까지 전체 전방 camera GIF
- `left/rviz_turn.png`, `right/rviz_turn.png`: 회전 중 VAD raw/candidate/corrected trajectory와 camera
- `turn_keyframes.png`: 좌/우 접근, 회전, 이탈 camera keyframe
- `left/result.json`, `right/result.json`: evaluator 원본 판정과 actual path
- `left/frames/manifest.json`, `right/frames/manifest.json`: 녹화 simulation timestamp

`maximum_trajectory_correction_m`는 차량이 reference에서 그 거리만큼 벗어났다는 뜻이
아니라 VAD raw trajectory point를 corridor 안으로 이동한 최대 XY 거리다. 실제 차량의
reference 대비 오차는 최대 CTE로 구분한다. 이번 결과는 **Town01/ClearNoon의 지정된
교차로 두 건에서 hybrid 구조가 완주 가능함**을 증명한다. 다른 Town과 날씨, traffic,
반복 재현성 및 baseline 대비 정확도/속도 A/B는 아직 통과했다고 해석하면 안 된다.

이 결과를 만들 당시에는 NVIDIA kernel/userspace mismatch 우회가 필요했다. 2026-08-27
재부팅으로 양쪽이 `580.178.04`로 맞은 뒤에는 native 실행이 기본이며
`AUTOWARE_E2E_NVIDIA_COMPAT_ROOT`를 unset한다.

### 2026-08-24 우회전 경로/제어 분리 진단

같은 `Town01/ClearNoon` 우회전을 rosbag으로 다시 실행해 다음 신호를 같은 시간축으로
비교했다.

```text
VAD selected raw -> route-manager final -> MPC predicted -> actual odometry
                                              |
                                              +-> steer command -> CARLA steering report
```

결론은 **`mixed_path_and_control`**, 즉 둘 다 문제다. `path_vs_control.png`의 주황
점선과 파란 선 차이는 VAD/route-manager 경로 문제이고, 아래 CTE 그래프의 파란 선은
최종 경로를 받은 MPC/차량의 추종 문제다. 화면에서 보인 우회전 안쪽 corner cut은 다음처럼
같은 route 법선에서 정확히 분해된다.

```text
actual -> route CTE = final -> route offset + actual -> final tracking error
```

기존 bridge는 CARLA의 front-left 개별 wheel angle을 Autoware가 요구하는 virtual bicycle
tire angle처럼 publish했고, yaw rate도 CARLA `deg/s, clockwise-positive`를 ROS
`rad/s, counter-clockwise-positive`로 변환하지 않았다. lateral velocity 부호도 반대였다.
이 값들은 MPC feedback뿐 아니라 VAD can-bus 입력에도 들어간다. 다음 수정은 이미 적용됐다.

- FL Ackermann angle -> virtual tire angle 변환
- angular velocity의 world -> vehicle frame 회전과 degree/sign 변환
- lateral velocity의 CARLA -> ROS 부호 변환
- `wheel_base=2.79 m`, `wheel_tread=1.64 m`를 vehicle-info YAML에서 bridge로 전달
- `scripts/e2e/build.sh`와 `build_full.sh`에서 재현 가능한 patch 자동 적용

같은 우회전을 기본 MPC `input_delay=0.24`, `vehicle_model_steer_tau=0.27`로 실행한
결과다. 수정 후에는 독립적으로 두 번 완주했다.

| 항목 | 수정 전 | 수정 후 #1 | 수정 후 #2 |
|---|---:|---:|---:|
| evaluator 최대 route CTE | 1.160 m | 0.932 m | 1.131 m |
| 재계산 최대 route CTE | 1.166 m | 0.932 m | 1.134 m |
| 피크 final -> route 안쪽 offset | 0.500 m | 0.494 m | 0.496 m |
| 피크 actual -> final 제어 성분 | 0.666 m | 0.437 m | 0.638 m |
| actual -> final 전체 최대 | 0.666 m | 0.536 m | 0.637 m |
| raw -> final common-progress p95 | 3.974 m | 3.803 m | 1.404 m |
| yaw-rate twist/pose gain | -57.490 | 1.007 | 1.001 |
| 조향 fit delay / tau | 0.10 / 0.198 s | 0.10 / 0.198 s | 0.10 / 0.198 s |

두 수정 후 실행의 피크 제어 성분은 `0.44~0.64 m`로 변했다. 첫 실행은 수정 전보다
34% 감소했지만 두 번째는 4% 감소에 그쳤으므로, bridge 수정 하나로 overshoot가 해결됐다고
주장할 수는 없다. 반면 yaw-rate gain은 두 번 모두 약 `1.0`으로 정상화돼 bridge 메시지
계약 오류가 제거됐음은 직접 확인했다. final 경로가 반복해서 route 안쪽 약 `0.50 m`에
붙고, 차량이 거기서 다시 안쪽으로 최대 `0.44~0.64 m` 벗어나므로 경로와 제어 양쪽을
모두 수정해야 한다.

VAD 쪽의 큰 raw 보정에는 command scheduling도 섞여 있다. 이 진단 당시
`maneuver_lookahead_m=2.0`인데 VAD trajectory horizon은 약 20 m이므로, 아직
`LANE_FOLLOW`를 선택한 상태에서도 raw 미래 점은 교차로를 직진하고 route manager는
그 미래 점을 우회전 corridor로 강하게 되돌린다. 따라서 `maximum_trajectory_correction`
하나만 보고 현재 ego가 그만큼 이탈했다고 해석하면 안 된다.

실측값에 맞춘 `input_delay=0.06`, `vehicle_model_steer_tau=0.20`도 bridge 수정 전에
시험했지만 실행마다 VAD 경로가 달라 공정한 제어 A/B가 되지 않았고 추종 p95도 개선되지
않았다. 이 두 값은 **채택하지 않았고 기본 `0.24/0.27`로 복원했다.** 실측 plant fit은
반복해서 약 `delay=0.10 s`, `tau=0.20 s`가 나오지만, 같은 final trajectory를 replay하는
통제 시험 전에는 MPC 파라미터로 확정하지 않는다.

이 문단은 2026-08-24 진단 단계의 결정 기록이다. 이후 exact-first camera sync와
반복 replay/폐루프 시험을 거쳐 현재 `--recommended`는 `0.12/0.15`, lookahead
`3.0 m`, turn inward corridor `0.20 m`, smoothing `10.0`을 사용한다. 따라서 위
`0.24/0.27` 복원 결론을 현재 권장값으로 읽으면 안 된다.

진단 산출물은 다음 세 디렉터리에 있다.

- 수정 전: `artifacts/diagnostics/2026-08-24_right_turn/baseline/`
- 수정 후 #1: `artifacts/diagnostics/2026-08-24_right_turn/bridge_fixed_run01/`
- 수정 후 #2: `artifacts/diagnostics/2026-08-24_right_turn/bridge_fixed_run02/`

- `path_vs_control.png`: raw/final/MPC/actual과 시간별 CTE
- `steering_tracking.png`: 조향 명령, FL/virtual 측정, yaw rate와 추종 오차
- `diagnosis.json`: 전체 정량 지표와 peak corner-cut snapshot
- `result.json`: 목표 도달 evaluator 결과
- `bag/`: 분석에 사용한 rosbag2 원본

기존 bag이나 같은 토픽으로 새로 기록한 bag은 다음 명령으로 다시 분석한다.

```bash
source scripts/e2e/env.sh
python3 scripts/e2e/analyze_turn_dynamics.py \
  --bag artifacts/diagnostics/2026-08-24_right_turn/baseline/bag \
  --route-file data/routes/town01_fast_right_clear_noon.json \
  --result-dir artifacts/diagnostics/2026-08-24_right_turn/baseline \
  --steering-report-mode legacy_fl \
  --mpc-input-delay-sec 0.24 \
  --mpc-steer-tau-sec 0.27
```

bridge 수정 후 bag은 `--steering-report-mode virtual`을 사용한다. 수정 전 bag에
`virtual`을 쓰거나 수정 후 bag에 `legacy_fl`을 쓰면 조향각을 잘못 해석한다.

```bash
python3 scripts/e2e/analyze_turn_dynamics.py \
  --bag artifacts/diagnostics/2026-08-24_right_turn/bridge_fixed_run02/bag \
  --route-file data/routes/town01_fast_right_clear_noon.json \
  --result-dir artifacts/diagnostics/2026-08-24_right_turn/bridge_fixed_run02 \
  --steering-report-mode virtual \
  --mpc-input-delay-sec 0.24 \
  --mpc-steer-tau-sec 0.27
```

새 우회전 bag은 stack과 evaluator를 실행한 상태에서 별도 terminal에 다음 recorder를
하나만 띄워 기록한다. 같은 ROS domain에 recorder가 이미 있으면 중복 node로 MRM이
동작할 수 있으므로 이 helper가 실행을 거부한다.

```bash
source scripts/e2e/env.sh
scripts/e2e/record_turn_dynamics.sh \
  artifacts/runs/$(date +%F)/right_turn_tuning/my_run/bag
```

### VAD lookahead/corridor가 바꾸는 것

두 값은 모두 `/planning/trajectory`를 바꿀 수 있지만 적용 단계가 다르다.

| 값 | 적용 단계 | 값을 줄였을 때 | 재학습 |
|---|---|---|---|
| `maneuver_lookahead_m` | 현재 route progress 앞의 command를 보고 여섯 VAD candidate 중 `LEFT/RIGHT/LANE_FOLLOW` branch 선택 | 회전 candidate로 전환하는 시점이 늦어진다. 선택되는 VAD raw trajectory 자체가 바뀐다. | 불필요 |
| `maneuver_exit_lookahead_m` | 현재 maneuver에서 `LANE_FOLLOW`로 복귀할 command 선택 | 값을 줄이면 maneuver branch를 더 오래 유지한다. | 불필요 |
| `route_corridor_half_width_m` | 선택한 raw trajectory를 resample한 뒤 reference route 중심선 주변으로 제한 | final trajectory 허용 폭이 좁아진다. 모델 출력이 좋아지는 것이 아니라 route manager의 XY 보정이 강해진다. | 불필요 |
| `turn_outward_corridor_half_width_m` | LEFT/RIGHT command에서 회전 바깥쪽 lateral bound만 제한 | 줄이면 5 m horizon에서 반대편으로 크게 넘어가는 S형 진폭을 제한한다. | 불필요 |
| `left/right_turn_outward_corridor_half_width_m` | 방향별 outward corridor override, `0.0`이면 공통값 상속 | 한 방향만 바꿀 수 있지만 경계 부착과 곡률 회귀를 따로 검증해야 한다. | 불필요 |
| `route_corridor_entry_distance_m` | raw point0에서 기존 hard corridor까지 quintic envelope 적용 | 양수이면 point0 detachment는 줄지만 이미 벗어난 ego 쪽 경로를 허용하므로 폐루프 검증이 필수다. | 불필요 |
| `route_corridor_mode` | corridor 투영을 hard clamp 또는 smooth saturation으로 적용 | `soft`는 경계 곡률을 줄일 수 있지만 corridor 안쪽 점도 중심으로 당긴다. | 불필요 |
| `trajectory_lateral_filter_gain` | 연속 VAD update의 route-relative lateral offset을 시간축으로 혼합 | `1.0`보다 작으면 trajectory jump를 줄이지만 오래된 경로 영향이 늘어난다. | 불필요 |
| `left/right_turn_trajectory_lateral_filter_gain` | 방향별 temporal gain override, `0.0`이면 공통값 상속 | 한 방향의 update만 필터링할 수 있다. | 불필요 |
| `trajectory_lateral_filter_activation_threshold_m` | 겹치는 horizon의 최대 offset 변화가 threshold를 넘을 때만 temporal filter 적용 | 키우면 작은 정상 update는 그대로 두고 큰 jump만 혼합한다. | 불필요 |
| `maximum_lateral_acceleration_mps2` | final trajectory 곡률을 보고 속도 상한 적용 | 양수이면 급곡률 구간을 미리 감속한다. 경로 XY는 바꾸지 않는다. | 불필요 |

따라서 corridor `0.5 -> 0.3 m`는 final trajectory의 route offset 상한을 기계적으로
줄일 수 있다. 그러나 raw VAD 출력의 오차를 숨기거나 경계에 붙는 곡률을 만들 수 있고,
final trajectory를 받은 controller의 추종 오차는 고치지 못한다. 이 값만 줄여서 최대
route CTE가 반드시 작아진다고 볼 수 없다.

### 역사 자료: 2026-08-25 초기 우회전 튜닝

`Town01/ClearNoon`, 동일 우회전 route에서 기본 MPC
`input_delay=0.24`, `vehicle_model_steer_tau=0.27`을 고정하고 VAD 쪽만 바꿨다.
MRM이 발생한 중복 recorder 실행과 sensor mapping 오류 실행은
`artifacts/diagnostics/2026-08-25_invalid_trials/`로 분리했으며 아래 표에서 제외했다.

| 실행 | shell | lookahead / corridor | 최대 route CTE | final→route 최대 | actual→final 최대 | raw→final p95 | turn freshness | 판정 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| bridge-fixed 기준 #2 | minimal | 2.0 / 0.5 m | 1.134 m | 0.500 m | 0.637 m | 1.404 m | 구 recorder, 직접 비교 제한 | 기준값 |
| lookahead only | full | 1.0 / 0.5 m | 1.003 m | 0.500 m | 비교 제외 | 5.912 m | 67.2% | freshness 부족 |
| combined #1 | full | 1.0 / 0.3 m | 1.020 m | 0.300 m | 0.718 m | 1.550 m | 90.2% | `REJECT` |
| combined #2 | full | 1.0 / 0.3 m | 1.063 m | 0.300 m | 0.756 m | 2.421 m | 84.4% | `REJECT` |

Combined 두 실행에서 corridor 제한과 command 전환 위치는 의도대로 작동했다. 그러나
`actual -> final` 최대 오차는 기준 `0.637 m`보다 두 번 모두 커졌고, turn trajectory
freshness도 요구값 `99%`를 두 번 모두 통과하지 못했다. 조향 command peak도 두 번
모두 `0.5 rad`를 넘었으며 command total variation 개선은 두 번째 실행에서 재현되지
않았다. 따라서 이 단계에서는 `lookahead=1.0/corridor=0.3`을 **`REJECT/HOLD`**하고
source 기본값 `2.0/0.5`를 유지했다. 이 결론은 이후 reliable exact-first 입력과
lookahead `3.0`, turn inward corridor `0.20`, smoothing `10.0`의 2회 반복 검증으로
**`SUPERSEDED`**됐다. 아래 수치는 실패 원인과 탐색 이력을 보존하기 위한 것이며 현재
실행값이 아니다.

관련 시각 자료는 다음 위치에 있다.

- `artifacts/runs/2026-08-25/right_turn_tuning/comparison_summary.png`: 유효 실행 핵심 수치 비교
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030/turn_path_control.gif`: raw/final/MPC/actual 우회전 animation
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030/turn_path_control_keyframe.png`: 최대 안쪽 이탈 부근 경로 분해 PNG
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030/front_camera_drive.gif`: 같은 실행의 전방 camera
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030/front_camera_turn_keyframe.png`: 실제 우회전 중 전방 camera PNG
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030/path_vs_control.png`: 경로와 제어 오차 분리
- `artifacts/runs/2026-08-25/right_turn_tuning/lookahead_1_corridor_030_repeat03/path_vs_control.png`: 독립 반복 결과
- `artifacts/runs/2026-08-25/right_turn_tuning/experiment_summary.json`: 입력값, 품질 gate와 수치 원본

### 역사 자료: 2026-08-25 초기 MPC/VAD/Smart 최적화

이 절의 `0.09/0.15` 채택 판단은 당시 camera freshness와 corridor 조건에서 얻은 중간
결론이다. 이후 frozen replay, exact-first/reliable VAD 입력과 좌/우/직진 폐루프 반복을
합친 현재 결론은 문서 맨 앞의 `0.12/0.15` 권장 profile이며, 이 절의 opt-in 판단은
**`SUPERSEDED`**됐다.

bridge 수정 뒤 같은 `Town01/ClearNoon` 우회전과 fast VAD 입력을 사용했다. 기본과 표준
MPC 후보는 각각 독립 3회, VAD 후처리는 단계별 screen, Smart MPC는 nominal iLQR로
시험했다. 카메라 PNG 저장으로 CARLA real-time factor가 낮아진 실행은 영상 전용
`visual01`로 분리하고 통계에서 제외했다.

| 반복군 | strict 완주 | actual→route 최대 CTE 중앙값 | actual→final 최대 중앙값 | steer command p95 중앙값 | 판정 |
|---|---:|---:|---:|---:|---|
| stock `0.24/0.27` | 3/3 | 1.141 m | 0.640 m | 0.236 rad | 기준 |
| 표준 MPC `0.09/0.15` | 3/3 | 1.115 m | 0.617 m | 0.290 rad | 당시 후보, 현재 `SUPERSEDED` |
| 표준 MPC `0.12/0.15` | 2/3 | 1.036 m | 0.533 m | 0.276 rad | 당시 `HOLD`, 이후 권장 profile에 채택 |

`0.09/0.15`는 기본 대비 중앙값 기준 안쪽 route overshoot를 `2.3%`, controller tracking
오차를 `3.5%` 줄였고 세 번 모두 목표 정지했다. 대신 조향 p95가 커졌으므로 실차 기본값이나
전 환경 최적값으로 확정한 것이 아니다. `0.12/0.15`는 오차가 더 작았지만 한 번은 direct
goal tolerance `1.0 m`보다 `1.8 mm` 바깥에서 표준 Autoware가 STOP으로 전환해 strict
실패했다. 따라서 이 단계에서는 source 기본 `0.24/0.27`을 그대로 두고 `0.09/0.15`를
해당 simulation route의 opt-in 후보로 두었다. **현재는 이 결론을 적용하지 않으며**
`--recommended`가 설치된 canonical `0.12/0.15` YAML을 시작 시점에 주입한다.

VAD 후처리 screen 결과는 다음과 같다. 숫자는 단일 screen이므로 반복 MPC 표보다
증거 수준이 낮으며, 어느 것도 기본 반복 중앙값을 안정적으로 이기지 못했다.

| VAD screen | actual→route 최대 | actual→final 최대 | 판정 |
|---|---:|---:|---|
| soft corridor | 1.228 m | 0.746 m | `REJECT` |
| soft + temporal gain `0.75` | 1.191 m | 0.709 m | `REJECT` |
| soft + temporal + lateral accel `1.2` | 1.143 m | 0.655 m | `HOLD/REJECT` |
| 기본 hard + lateral accel `1.2` | 1.274 m | 0.774 m | `REJECT` |

Smart MPC nominal iLQR는 합성 입력에서 첫 JIT `10.38 s`, 이후 평균 `29.69 ms`, 제어
p95 `7.55 ms`로 계산에는 성공했다. 그러나 full CARLA에서는 actual→route `1.528 m`,
actual→final `1.031 m`였고 goal `3.27 m` 전에 정지해 stall 실패했다. 현재 VAD의 미래
zero-speed profile과 Smart 종제어를 다시 맞추기 전에는 표준 controller보다 낫지 않다.

결과 원본과 화면은 다음에 있다.

- `artifacts/runs/2026-08-25/optimization/optimization_summary.json`
- `artifacts/runs/2026-08-25/optimization/optimization_comparison.png`
- `artifacts/runs/2026-08-25/optimization/mpc_delay009_tau015_visual01/turn_path_control.gif`
- `artifacts/runs/2026-08-25/optimization/mpc_delay009_tau015_visual01/front_camera_drive.gif`
- `artifacts/runs/2026-08-25/optimization/mpc_delay009_tau015_visual01/front_camera_keyframes.png`
- `artifacts/runs/2026-08-25/controller_replay/standard_mpc_repeat3/sweep_summary.json`

아래 명령은 과거 `0.09/0.15` artifact를 재현하는 연구용 기록이다. **현재 권장 실행에
사용하지 않는다.** MPC 파일은 stock YAML 전체를 복제한 뒤 두 timing 값만 바꾸고 base
SHA256과 effective delay metadata를 함께 남긴다. 실행 중 `ros2 param set`은 steer
model을 완전히 재생성하지 않으므로 시작 시점 주입 방식을 사용했다.

```bash
mkdir -p data/generated/control
python3 scripts/e2e/generate_mpc_config.py \
  --input-delay 0.09 \
  --steer-tau 0.15 \
  --output data/generated/control/mpc_delay009_tau015.param.yaml

mpc_file=$(realpath data/generated/control/mpc_delay009_tau015.param.yaml)
scripts/e2e/run_route_vad_fast.sh --full --visualize \
  data/routes/town01_fast_right_clear_noon.json \
  use_lateral_controller_param_override:=true \
  lateral_controller_param_path:="$mpc_file"
```

### 기본 MPC와 Smart MPC

현재 full/minimal 실행의 controller는 Autoware Universe 표준 조합이다.

- 횡방향: `autoware_mpc_lateral_controller`, linear MPC + OSQP
- 종방향: `autoware_pid_longitudinal_controller`
- upstream/source sample 값: `input_delay=0.24 s`, `vehicle_model_steer_tau=0.27 s`
- 현재 `--recommended` canonical 값: `0.12/0.15 s`
- 과거 `0.09/0.15 s` opt-in 결론: `SUPERSEDED`, 사용하지 않음

`0.24/0.27`은 이 CARLA Prius에서 새로 검증한 최적값이 아니라 workspace가 사용하던
Autoware sample/vehicle 설정이다. 공식 문서도 MPC 기본 파라미터가 Lexus RX450h의
40 km/h 이하 조건에 맞춰졌으며 실제 steering delay와 time constant를 다시 확인하라고
명시한다. 이 프로젝트는 실제 Autoware MPC/OSQP와 고정 trajectory bicycle plant replay로
후보를 좁힌 뒤 CARLA 반복으로 검증했다. 일반 full/minimal launch를 직접 실행하면 source
값이 남을 수 있지만, 문서 맨 앞의 `--recommended`는 설치된
`mpc_carla_recommended.param.yaml`을 강제로 주입하므로 `0.12/0.15`로 실행된다.

- [공식 MPC lateral controller 문서](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/)

Smart MPC 소스 `autoware_smart_mpc_trajectory_follower`는 이 workspace의 Universe
`0.52.0`에서 opt-in build/install과 nominal preset까지 준비돼 있다. 공식 control launch에서
`trajectory_follower_mode:=smart_mpc_trajectory_follower` 선택을 지원하지만
`under development`로 표시한다. Smart MPC는 lateral MPC만 교체하는 drop-in이 아니라
하나의 node가 acceleration과 steering을 함께 출력하므로 기존 lateral MPC와 longitudinal
PID를 동시에 대체한다.

기본 `ilqr` nominal mode는 학습 없이 `use_trained_model=false`로 먼저 실행할 수 있다.
현재 local nominal 값은 steering `delay=0.27 s`, `time_constant=0.24 s`다. 표준 MPC의
숫자와 순서가 우연히 반대이므로 숫자 위치가 아니라 **delay끼리, time constant끼리**
비교해야 한다. 학습 모델을 켜려면 CARLA 차량에서 control/vehicle state bag을 수집해
별도 학습과 회귀 검증을 해야 한다. MPPI는 공식 문서도 기본 sample 수로 성능이 제한되고
GPU 지원을 개발 중이라고 설명하므로 첫 비교는 nominal iLQR이 적절하다.

- [공식 Smart MPC 문서](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_smart_mpc_trajectory_follower/)
- [공식 control launch의 Smart MPC selector](https://github.com/autowarefoundation/autoware_launch/blob/main/tier4_universe_launch/tier4_control_launch/launch/control.launch.xml)

Smart MPC 실행은 다음과 같다. wrapper는 비교 조건을 맞추기 위해 full shell과 당시
fast VAD 입력을 함께 사용한다. 현재 결과는 위와 같이 `FAIL`이므로 연구용이며
`--recommended`를 대체하지 않는다.

```bash
scripts/e2e/build_smart_mpc.sh
scripts/e2e/run_route_vad_smart_mpc.sh \
  data/routes/town01_fast_right_clear_noon.json
```

잘못된 final path는 어떤 controller도 복구할 수 없고, Smart MPC는 종/횡 결합과 stop
profile이라는 변수가 추가된다. 다음 Smart 단계는 CARLA control/state data로 학습하기
전에 VAD zero-speed horizon과 조기 정지를 고치고 같은 strict test를 통과시키는 것이다.

## 역사 자료: 기존 Full baseline 재현 (2026-08-23)

아래 세 terminal 절은 2026-08-23의 non-fast Full baseline을 재현하는 역사 자료다.
현재 일반 주행과 UI 확인은 문서 맨 앞의 `--recommended --visualize` 절을 사용한다.

이 baseline에서 실제 폐루프 검증이 끝난 조합은 `Town01`, `ClearNoon`,
spawn 143→145다.
기존 Woraksan CARLA가 2000번 포트를 사용하므로 아래 project 시험은 2100번을
사용한다. 재현 명령은 현재 workspace인 `/home/a/autoware_e2e`에서 실행한다.

2100번 port는 통신 충돌 방지용일 뿐 GPU 격리가 아니다. 같은 RTX 3060에서 시험할
때는 기존 2000번 CARLA를 먼저 정상 종료한 뒤 Terminal 1을 시작한다. 기존 CARLA를
계속 사용해야 하면 새 시험은 다른 시간, 두 번째 GPU 또는 다른 PC에서 실행한다.
이 프로젝트의 script는 기존 2000번 process를 자동으로 종료하지 않는다.

현재 이 workspace는 model, dependency, build와 Town01 full map이 준비돼 있다. 새로
clone했거나 ignored `data/`와 `install/`을 지운 경우에는 먼저 다음 초기 준비를 한다.
Town01 map source 위치와 세부 조건은 뒤의 `준비와 빌드` 절을 따른다.

```bash
cd /home/a/autoware_e2e
scripts/e2e/download_vad_models.sh
scripts/e2e/setup_town01_full_map.sh --dry-run
scripts/e2e/setup_town01_full_map.sh
scripts/e2e/build_full.sh
```

준비된 workspace에서는 환경 진단부터 시작한다.

```bash
cd /home/a/autoware_e2e
scripts/e2e/doctor.sh
```

Driver/library mismatch가 나오면 먼저 재부팅한 뒤 이 문서의 NVIDIA compatibility
절을 확인한다. 현재 PC는 native `580.178.04`가 정상이므로 아래 Terminal 1, 2, 3에서
과거 우회 변수를 각각 unset한다.

### Terminal 1: CARLA 2100

```bash
cd /home/a/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT
export CARLA_PORT=2100

scripts/e2e/run_carla.sh \
  -RenderOffScreen \
  -nosound \
  -carla-port=2100
```

`run_carla.sh`는 기본적으로 CARLA Low quality를 사용한다. VAD는 실제 RGB sensor
영상이 필요하므로 `-nullrhi`나 no-rendering mode는 사용하지 않는다.

### Terminal 2: route 준비 후 Full stack

Route 준비는 CARLA server만 실행되고 Autoware bridge는 아직 없는 상태에서 한다.

```bash
cd /home/a/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT
export CARLA_PORT=2100

scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario lane_follow \
  --start-index 143 \
  --goal-index 145 \
  --output data/routes/town01_lane_follow_clear_noon.json

scripts/e2e/run_route_vad_full.sh \
  data/routes/town01_lane_follow_clear_noon.json
```

두 번째 명령이 CARLA bridge, Full Autoware vehicle/system/map/sensing/control/API,
TensorRT VAD와 RViz를 함께 실행한다. `run_visualization.sh`를 추가로 실행하지 않는다.

### Terminal 3: 자동 주행과 결과 저장

```bash
cd /home/a/autoware_e2e
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT

scripts/e2e/route_test.sh \
  --full-stack \
  --route-file data/routes/town01_lane_follow_clear_noon.json \
  --result artifacts/runs/2026-08-23/full_stack_lane_follow/town01_lane_follow_full_stack.json \
  --ready-timeout 180 \
  --service-timeout 30 \
  --engage-timeout 60 \
  --sim-timeout 180 \
  --wall-timeout 900 \
  --stall-timeout 20 \
  --data-stale-timeout 60
```

Evaluator는 준비 조건, autonomous 전이, 실제 이동, 정확한 JSON goal 거리와 정지
속도를 확인하고 마지막에 stable `STOP`으로 전환한다.

### 종료와 다음 episode

Evaluator가 끝난 뒤 Terminal 2의 Full stack을 `Ctrl-C`로 완전히 종료한다. 맵,
날씨, 시작점 또는 목표점을 바꿀 때는 bridge가 없는 상태에서 route를 다시 준비한
뒤 Full stack을 재시작한다. Full launch가 자식으로 시작한 RViz도 함께 종료됐는지
다음 episode 전에 확인한다.

```text
evaluator 완료 및 stable STOP
  -> Full stack Ctrl-C 및 process 종료 확인
  -> 다음 route JSON 준비
  -> Full stack 재시작
  -> 다음 evaluator 실행
```

## 맵, 날씨, 시작점과 목표점 변경

### 같은 Town에서 변경

`Town01` 안에서 날씨, 시작점, 목표점 또는 maneuver만 바꿀 때는 Autoware map
bundle을 바꾸지 않는다. Full stack을 종료하고 route JSON만 새로 만든다.

```bash
scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather HardRainNoon \
  --scenario left \
  --min-distance 20 \
  --max-distance 45 \
  --preferred-distance 30 \
  --output data/routes/town01_left_hard_rain.json

scripts/e2e/run_route_vad_full.sh \
  data/routes/town01_left_hard_rain.json
```

고정 회귀에는 `--start-index`와 `--goal-index`, 새 route 탐색에는 거리 조건을
사용한다. Route가 생성됐다는 사실은 VAD가 그 route를 성공한다는 뜻이 아니다.

### C-track / 역사 자료인 월악산 custom map

> C-track은 2026-09-01 v16 runnable 범위에 포함된다. 반면 아래 월악산 setup과 고정
> route 명령은 2026-08-28 재현 자료이며 현재 v16 실행 진입점이 아니다. 월악산은
> packaged runtime asset과 승인된 full-map bundle을 다시 확보한 뒤 새 catalog와
> straight/turn 폐루프를 통과해야 `BLOCKED`를 해제한다.

처음 한 번에는 manifest를 검사하고 두 Full map bundle을 만든다. Source asset이 바뀌면
먼저 `scripts/e2e/custom_map_bundles.yaml`의 고정 path/SHA-256/topology를 검토해 갱신해야
한다. 변경하지 않은 target에 대한 `setup`은 `keep`으로 끝나는 idempotent 작업이다.
의도한 source 변경 때문에 generated metadata나 파생 OSM이 달라진 경우에는 diff를 검토한
뒤에만 `setup <profile> --refresh-generated`를 사용한다. Regular OSM/PCD 파일을 이 옵션이
임의로 덮어쓰지는 않는다.

```bash
cd /home/a/autoware_e2e
python3 scripts/e2e/setup_custom_full_map.py list
python3 scripts/e2e/setup_custom_full_map.py inspect c_track_simulation
python3 scripts/e2e/setup_custom_full_map.py setup c_track_simulation
python3 scripts/e2e/setup_custom_full_map.py inspect woraksan_simulation_current
python3 scripts/e2e/setup_custom_full_map.py setup woraksan_simulation_current
```

맵 변경 순서는 `Full stack 종료 -> CARLA Ctrl-C -> 다음 CARLA cold-start -> 다음
stack`이다. 실행 중 `client.load_world()`로 두 custom map을 바꾸지 않는다. C-track은
반드시 `Epic`으로 시작한다.

```bash
# Terminal 1: 둘 중 현재 시험할 맵 하나만 실행한다.
scripts/e2e/run_carla_map.sh C_track_1_0_7 \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound

# 또는, 앞 server를 Ctrl-C한 뒤 월악산
scripts/e2e/run_carla_map.sh Woraksan_v1_0_3_parking_lot_hegiht_fit \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
```

검증에 사용한 고정 route는 이미 `data/routes/custom_maps/`에 있다. 아래 명령은 canonical
파일을 보존하도록 `_regenerated.json`에 새 candidate를 만든다. 월악산 canonical safe
route의 정확한 원본은 검증된 catalog JSON이며, raw `2 -> 8` goal을 그대로 사용하지 않고
Lanelet2 centerline, 차량 footprint와 boundary clearance를 검사한다.

```bash
# C-track server가 실행 중일 때
scripts/e2e/prepare_carla_route.sh --port 2100 \
  --town C_track_1_0_7 --weather ClearNoon --scenario lane_follow \
  --start-index 394 --goal-index 290 \
  --output data/routes/custom_maps/c_track_lane_follow_394_290_regenerated.json

# 월악산 canonical safe route를 같은 검증 catalog에서 다시 파생
python3 scripts/e2e/trim_route_goal.py \
  data/training/catalogs/woraksan_packaged_safe_smoke/routes/woraksan_1_0_3/straight/woraksan_1_0_3_straight_s0000_p00.json \
  --map-bundle data/maps/Woraksan_v1_0_3_parking_lot_hegiht_fit_full \
  --output data/routes/custom_maps/woraksan_straight_lanelet_safe_regenerated.json --json
```

새 월악산 spawn `2 -> 8` candidate를 CARLA에서 다시 찾으려면 위 catalog 파일 대신
`prepare_carla_route.sh`의 `--start-index 2 --goal-index 8` 출력을 trim 입력으로 쓴다.
그 결과는 기존 canonical과 route/footprint preflight 및 폐루프 반복을 비교한 뒤에만
채택한다.

UI 실행과 결과 기록은 서로 다른 진입점이다. UI를 별도 terminal로 실행할 때는 source
route에 map transform을 한 번 적용한 stable JSON을 먼저 만든다. **Full stack과
`route_test.sh`가 반드시 같은 aligned JSON을 사용해야 한다.** Source route를 stack에만
주고 evaluator에 그대로 주면 월악산 goal 좌표가 서로 달라진다.

```bash
# 한 번 준비: source CARLA route -> Autoware map frame route
mkdir -p data/generated/aligned_routes
python3 scripts/e2e/align_carla_route_to_map.py \
  data/routes/custom_maps/c_track_lane_follow_394_290.json \
  data/maps/C_track_1_0_7_full \
  --output data/generated/aligned_routes/c_track_lane_follow_394_290.map.json --json
python3 scripts/e2e/align_carla_route_to_map.py \
  data/routes/custom_maps/woraksan_straight_lanelet_safe.json \
  data/maps/Woraksan_v1_0_3_parking_lot_hegiht_fit_full \
  --output data/generated/aligned_routes/woraksan_straight_lanelet_safe.map.json --json

# C-track Terminal 2: Full stack + RViz
scripts/e2e/run_route_vad_fast.sh --recommended --visualize \
  data/generated/aligned_routes/c_track_lane_follow_394_290.map.json

# C-track Terminal 3: engage/evaluate. Terminal 2와 같은 aligned route를 사용한다.
scripts/e2e/route_test.sh --full-stack \
  --route-file data/generated/aligned_routes/c_track_lane_follow_394_290.map.json \
  --result artifacts/runs/custom_maps/c_track_ui_result.json --ready-timeout 300

# 한 명령으로 정량 기록할 때는 raw canonical route를 준다. 이 runner는 자동 정합한다.
CARLA_PORT=2100 scripts/e2e/run_recorded_route_trial.sh \
  --recommended --ready-timeout 300 \
  artifacts/runs/custom_maps/c_track_my_run \
  data/routes/custom_maps/c_track_lane_follow_394_290.json

# 월악산 Terminal 2: +/-0.20 m corridor는 이 맵에서 3회 반복 검증했다.
scripts/e2e/run_route_vad_fast.sh --recommended --tight-corridor --visualize \
  data/generated/aligned_routes/woraksan_straight_lanelet_safe.map.json

# 월악산 Terminal 3
scripts/e2e/route_test.sh --full-stack \
  --route-file data/generated/aligned_routes/woraksan_straight_lanelet_safe.map.json \
  --result artifacts/runs/custom_maps/woraksan_ui_result.json --ready-timeout 300

# 월악산 자동 기록
CARLA_PORT=2100 scripts/e2e/run_recorded_route_trial.sh \
  --recommended --tight-corridor --ready-timeout 300 \
  artifacts/runs/custom_maps/woraksan_my_run \
  data/routes/custom_maps/woraksan_straight_lanelet_safe.json
```

Stack wrapper는 `map_bundle.json`의 CARLA-to-map 3D transform을 route와 odometry/TF에
적용하고 launch 전 route/Lanelet/PCD preflight를 수행한다. Recorded runner는 같은 aligned
route를 evaluator와 renderer에도 자동 전달한다. 월악산 transform은 단일 anchor에서 구한
empirical 값이므로 다른 구간의 정밀 정합을 보장하지 않는다. `--tight-corridor`는 raw VAD
weight를 개선하는 옵션이 아니라 선택 trajectory를 route 가까이 제한하는 더 강한 hybrid
intervention이다.

### 다른 CARLA Town으로 변경

다른 Town에서는 다음 세 값이 같은 세계와 좌표계를 가리켜야 한다.

```text
route JSON의 town: TownXX
CARLA world:         TownXX
Autoware map:        data/maps/TownXX_full/
```

Full map directory에는 다음 세 파일이 필요하다.

```text
data/maps/TownXX_full/
  lanelet2_map.osm
  pointcloud_map.pcd
  map_projector_info.yaml
```

`run_route_vad_full.sh`는 route JSON의 `town`을 읽어 기본적으로
`data/maps/${town}_full`을 찾지만 CARLA server를 시작하거나 world를 바꾸지는 않는다.
다른 packaged map으로 바꿀 때는 bridge와 기존 CARLA를 완전히 종료한 뒤 Terminal 1에서
먼저 정확한 map을 cold-start한다. `prepare_carla_route.py`는 현재 world가 다르면
`client.load_world()`를 호출하므로, renderer crash를 피하려면 준비 명령 전에 두 이름이
이미 같아야 한다.

```bash
# 예: Town03 world만 먼저 시작한다. Full 주행에는 Town03_full도 별도로 필요하다.
scripts/e2e/run_carla_map.sh Town03 \
  --port 2100 --quality Epic --startup-timeout-sec 180 -- \
  -RenderOffScreen -nosound
```

다른 절대 Autoware map 경로를 쓸 때는 Full stack 실행 terminal에서
`AUTOWARE_E2E_FULL_MAP_PATH`를 지정한다.

```bash
export AUTOWARE_E2E_FULL_MAP_PATH=/absolute/path/to/TownXX_full
scripts/e2e/run_route_vad_full.sh data/routes/townxx_route.json
```

2026-09-01 v16에서 Full VAD 폐루프까지 승인·검증한 executable variant는
`town01`, `town02_opt`, `town03`, `town04`, `town05_opt`, `town06`, `town07`,
`town10hd_opt`, `c_track_1_0_7`의 9개다. 각 map의 straight/turn이 모두 통과해
trial `18/18 PASS`다. standard `town02`, `town05`, `town10hd`는 각 optimized variant와
다른 map identity이므로 여전히 `BLOCKED`다. Woraksan도 현재 packaged runtime asset과
승인된 full-map bundle이 없어 `BLOCKED`이며, 과거 고정 route 검증을 v16 전체 matrix
결과로 해석하지 않는다. `setup_town01_full_map.sh`는 Town01 전용이며 custom
map에는 `setup_custom_full_map.py`를 사용한다.

이 Full Autoware map 조건은 VAD 폐루프와 RViz 표준 map 표시에 필요한 것이다. 문서
앞부분의 BasicAgent expert collection만 수행할 때는 CARLA level과 OpenDRIVE가 있으면
되며 Autoware PCD/Lanelet2 bundle은 요구하지 않는다.

공식 VAD는 RViz `2D Goal Pose`의 임의 goal까지 전역 경로를 만드는 모델이 아니다.
현재 goal source는 route JSON이므로 route 변경 후 Full stack을 재시작한다.

## 먼저 알아야 할 구조

아래 데이터 흐름과 이 문서의 기존 주행 결과는 먼저 완성한 **minimal E2E
경로**를 설명한다.

공식 VAD `carla_tiny`는 목적지 좌표를 입력받아 전역 경로를 만드는
goal-conditioned 모델이 아니다. 카메라 영상과 차량 상태를 받아 아래 여섯 개
고수준 command에 해당하는 짧은 로컬 궤적 후보를 동시에 출력한다.

| 값 | VAD command |
|---:|---|
| 0 | `LEFT` |
| 1 | `RIGHT` |
| 2 | `STRAIGHT` |
| 3 | `LANE_FOLLOW` |
| 4 | `CHANGE_LANE_LEFT` |
| 5 | `CHANGE_LANE_RIGHT` |

이 프로젝트는 CARLA의 `GlobalRoutePlanner`와 `vad_route_manager`를 추가하여
목표 주행을 구성한다.

```text
CARLA 시작/목표 spawn index
  -> CARLA GlobalRoutePlanner가 reference route JSON 생성
  -> CARLA 6개 RGB 카메라 + odometry/acceleration
  -> 공식 TensorRT VAD가 6개 command candidate 생성
  -> vad_route_manager가 현재 route 위치와 전방 maneuver를 계산
  -> command에 대응하는 candidate 1개 선택
  -> 선택 궤적을 CARLA route corridor로 제한
     (baseline 반폭 0.5 m, recommended 회전 안쪽 한계 0.20 m 추가)
  -> 목표 정지를 위한 속도/감속 프로파일과 정지점을 적용
  -> /planning/trajectory
  -> Autoware MPC 횡제어 + PID 종제어
  -> vehicle command gate/raw command converter
  -> CARLA ego vehicle
```

따라서 현재 구성은 다음 두 부분이 결합된 형태다.

- E2E 부분: 여섯 카메라를 사용한 VAD의 인지 및 로컬 궤적 후보 생성
- 기존 방식 부분: CARLA 전역 route, command 선택, route corridor 보정,
  Autoware MPC/PID 추종 및 목표 정지 판정

즉, 현재 결과는 **순수 goal-conditioned E2E**가 아니라 VAD와 전역 route
adapter를 결합한 시뮬레이션용 하이브리드다. Route manager가 실제 command로
선택한 원본은 `/planning/vad_route/selected_raw_trajectory`, controller 입력은
`/planning/trajectory`로 분리한다. 매 frame의 최대 보정 거리는
`/planning/vad_route/trajectory_correction`으로 공개해 개입 정도를 숨기지 않는다.
Upstream `/planning/vad/raw_trajectory`는 모델의 고정 `default_command` 출력이므로
교차로에서 manager-selected raw와 동일하다고 가정하면 안 된다.

RViz에서 임의의 `2D Goal Pose`를 클릭하면 공식 VAD가 그 목표까지 알아서
주행하는 구조는 아니다. 현재는 실행 전에 CARLA spawn index로 시작점과
목표점을 정하고 route JSON을 생성해야 한다.

## Full Autoware shell의 경계

`carla_vad_full.launch.xml`은 익숙한 Autoware 구조와 UI를 복원하기 위한 별도
통합 런치다. CARLA interface는 이 경로에서 `use_e2e_planning=false`로 실행되어
sensor와 raw actuation 변환만 담당한다. 그 위에 표준 `autoware.launch.xml`의
다음 부분을 실행한다.

- vehicle와 sensor TF/description
- system, map, sensing processing
- 표준 control stack과 vehicle command 처리
- Autoware API와 `autoware_vad_carla.rviz`
- RViz의 DateTime, AutowareState, ControlMode 패널과 System, Map, Sensing,
  Localization, Perception, Planning, Control, Debug display group
- VAD camera, candidate trajectory, selected raw/final trajectory, object와 BEV
  `MarkerArray`를 추가한 VAD display group

기본 `sensor_mapping_full.yaml`은 VAD용 RGB camera 여섯 개 외에 top LiDAR, IMU와
GNSS를 생성한다. Mapping의 publish cap은 camera/LiDAR 11 Hz, IMU 50 Hz, GNSS
20 Hz지만, Full launch의 기본 CARLA tick은 0.05초다. 현재 throttle 구현에서 실효
출력은 camera/LiDAR 약 10 Hz, IMU/GNSS 최대 20 Hz다. GNSS pose는
`/sensing/gnss/pose_with_covariance`로 publish되어 기본 truth localization bridge의
pose 입력이 된다.

RViz의 Planning group은 표준 mission planner의
`/planning/mission_planning/route_marker`와 JSON reference/actual path를 함께
표시한다. VAD BEV map 출력 `/perception/vad/map_points`의 실제 message type은
`visualization_msgs/MarkerArray`이며 RViz도 `MarkerArray` display로 표시한다.
`PointCloud2` display는 실제 point cloud인 Map group의 `/map/pointcloud_map`과
Sensing group의 `/sensing/lidar/top/pointcloud_before_sync`에만 사용한다.

기본값에서는 CARLA truth가 localization을, VAD가 object와 final trajectory를
소유한다. 즉 모든 standard algorithm을 동시에 돌리는 구성이 아니라, 표준
Autoware의 외곽 구조와 제어/UI는 유지하고 localization/perception/planning의
입력 소유권을 명시적으로 바꾼 구성이다.

| 기본 topic | 기본 publisher |
|---|---|
| `/localization/kinematic_state` | CARLA truth state bridge |
| `/localization/acceleration` | `twist2accel` |
| `/perception/object_recognition/objects` | VAD |
| `/planning/vad/raw_trajectory` | VAD |
| `/planning/trajectory` | `vad_route_manager` |
| `/control/command/control_cmd` | 표준 Autoware control stack |
| `/control/command/actuation_cmd` | CARLA raw command converter |

표준 AEB는 이 구성에서도 끄지 않는다. Conventional perception이 꺼진 기본값에서는
`vad_aeb_configurator`가 AEB parameter service를 통해
`use_pointcloud_data=false`, `use_predicted_object_data=true`를 atomic하게 적용한다.
이는 분할하지 않은 CARLA ground point cloud 대신 canonical topic의 VAD
`PredictedObjects`를 표준 AEB에 입력하기 위한 simulation 설정이다. 성공한 뒤에만
latched `/system/vad/aeb_configured=true`가 publish되며, full evaluator는 이를
engage 전 필수 준비 조건으로 검사한다.

VAD는 항상 raw trajectory를 private topic에 publish한다. 기본값에서는 route
manager만 final trajectory를 publish하며, conventional planning을 켜면 VAD는
shadow 출력만 남기고 standard planner가 final trajectory를 소유한다. 같은
topic에 두 planner가 동시에 publish하도록 구성하면 안 된다.

### Route JSON과 표준 mission route

`run_route_vad_full.sh`는 하나의 route JSON에서 CARLA town, spawn transform,
truth initial pose를 읽고 full launch에 전달한다. 기본 map directory는
`data/maps/<Town>_full`이며 다른 위치는 `AUTOWARE_E2E_FULL_MAP_PATH`로 지정한다.

Full 경로에서도 VAD의 maneuver 선택과 goal stop의 기준은 route JSON이다.
`vad_standard_route_adapter`는 localization이 올라온 뒤 같은 JSON의 goal을 표준
mission planner에 보내 RViz/API용 lanelet route를 만든다. 생성된 standard route가
JSON과 정렬됐다는 판정은 다음 조건을 모두 만족해야 한다.

- 시작점 거리 2.5 m 이하
- 목표점 거리 1.5 m 이하
- 목표 yaw 오차 0.35 rad 이하
- lanelet route segment가 하나 이상 존재
- 표준 route state가 `SET` 또는 `ARRIVED`

결과는 route 또는 route-state callback 때 다음 latched topic에 publish되고,
이후 1초 주기의 heartbeat로 반복된다.

```text
/planning/vad_route/standard_route_aligned
```

기본 `launch_standard_mission_planner=true`에서는 `vad_route_manager`의
`require_standard_route_alignment`도 `true`다. Manager는 heartbeat 수신 시각을
검사하며 2.5초 동안 새 message가 없으면 정렬이 stale한 것으로 처리한다. 정렬
결과가 아직 없으면 `waiting_for_standard_route`, false이거나 stale이면
`standard_route_mismatch` 상태로 두고 주행을 허용하지 않는다.

이 gate는 VAD candidate가 처음 오기 전에도 동작한다. Odometry가 준비되면 manager는
현재 ego pose부터 전방 0.5 m 간격으로 구성한 명시적인 3-point trajectory를 만들고,
세 point의 속도와 가속도를 0으로 설정해 정지 출력을 계속 publish한다. 이전
candidate가 있으면 그 header/앞 세 point를 복사하되 pose는 같은 ego anchor로
다시 설정한다.

이 정렬은 화면과 API를 같은 목표에 맞추는 안전 gate일 뿐이다. 실제 VAD command
선택, corridor와 goal stop의 conditioning source는 계속 route JSON이며, 표준
lanelet route는 VAD network나 route manager의 입력을 대체하지 않는다. 목표를
바꾸려면 bridge를 종료하고 route JSON을 다시 생성해야 한다.

또한 현재 alignment는 **endpoint 중심 검사**다. Start/goal 위치, goal yaw와
segment 존재 여부는 확인하지만, 중간 lanelet ID 순서, 전체 polyline geometry,
교차로 maneuver가 JSON route와 같은지는 비교하지 않는다. 같은 endpoint를 가진
다른 중간 경로도 통과할 수 있으므로 이 signal을 전체 route 동등성 증명으로
해석하면 안 된다.

## Minimal baseline 경로의 동작 범위

이 절은 `run_route_vad.sh`의 baseline 기본값을 설명한다. 문서 맨 앞의
`--recommended` Full profile 값과 혼합하지 않는다.

- 입력: 1600x900 surround RGB 6개, odometry, acceleration
- 모델 입력 변환: 각 카메라를 640x384로 resize
- 출력: 선택 궤적, 여섯 후보 궤적, 예측 객체, VAD BEV map marker
- 전역 안내: CARLA route의 `RoadOption`을 VAD command로 변환
- 경로 호환: 선택 VAD 궤적을 reference route 중심 0.5 m corridor 안으로 제한
- 제어: Autoware MPC lateral controller와 PID longitudinal controller
- 목표 판정: route 잔여 거리, 목표 직선 거리, 정지 속도를 함께 검사
- 평가: CTE, 궤적 보정량, 이동 거리, 잔여 거리, stall, timeout, topic freshness 기록
- 시각화: RViz/rqt 실시간 화면과 route/result PNG

기본 route manager 설정은 목표 허용 오차 1.0 m, 정지 속도 0.15 m/s 이하,
1.0 simulation second 유지다. VAD 원본 후보는 0.5 m 간격으로 보간되며 목표에
가까워지면 최대 1.2 m/s^2 범위에서 감속 속도가 제한된다. 세부 값은
`autoware_e2e_vad_launch/config/vad_route_manager.param.yaml`에 있다.

Minimal baseline의 maneuver command는 교차로 시작 2.0 m 전부터 선택하고, 교차로 종료
2.5 m 전부터 `LANE_FOLLOW`로 복귀한다. corridor 반폭도 0.5 m다. 이 값들은
CARLA `carla_tiny` baseline용이며 다른 town이나 모델에서는 suite 결과를 근거로
다시 조정해야 한다. 현재 `--recommended`는 이 source 기본값 대신 lookahead
`3.0 m`, 회전 안쪽 한계 `0.20 m`, 종단 감속 `0.60 m/s^2`를 명시적으로 전달한다.

## Minimal 경로의 검증 결과

2026-08-23 KST에 최종 설정으로 실제 CARLA engage부터 목표 정지, disengage,
PNG 생성까지 확인한 결과다. **이 표는 `run_route_vad.sh`로 실행한 minimal E2E
경로의 결과이며, `run_route_vad_full.sh`의 검증 결과가 아니다.**

| Case | 판정 | 최대 CTE | 최대 보정 | 최종 잔여 |
|---|---|---:|---:|---:|
| ClearNoon lane follow | `PASS / XY 보정 없음` | 0.086 m | 0.00 m | 0.993 m |
| HardRainNoon lane follow | `PASS / XY 보정 없음` | 0.075 m | 0.00 m | 0.978 m |
| ClearNoon left | `HYBRID PASS` | 1.140 m | 12.917 m | 0.964 m |
| ClearNoon right | `HYBRID PASS` | 1.069 m | 9.074 m | 0.961 m |

결과와 입력 이미지는 다음 위치에 있다.

- `artifacts/regressions/route_suites/final_lane_regression/`
- `artifacts/regressions/route_suites/final_hard_rain_regression/`
- `artifacts/regressions/route_suites/town01_left_corridor_lookahead2/`
- `artifacts/regressions/route_suites/town01_right_corridor/`

Turn case는 목표까지 주행했다는 뜻이지 순수 VAD geometry 성공이 아니다.
보정량이 큰 이유와 원본/보정 궤적 차이는 반드시 함께 보고해야 한다. 이 PC의
VAD real-time factor가 낮아 32 simulation seconds가 약 20 wall minutes 걸렸다.
보정 metric은 XY 위치 이동량만 뜻한다. XY가 수정된 trajectory는 경로 접선에
맞춰 yaw도 다시 계산한다. `vad_unassisted`는 XY corridor가 개입하지 않았다는
분류일 뿐이며 command 선택, 목표 감속, goal truncate까지 없는 순수 E2E라는
뜻은 아니다.

## 디렉터리

```text
autoware_e2e/
  ansible/, docker/             # upstream Autoware 개발/컨테이너 도구, 이동 금지
  repositories/                # pinned repository manifest
  src/                         # Autoware 1.9.0 pinned repositories
  autoware_e2e_vad_launch/     # 프로젝트 ROS launch, route manager, RViz 설정
  patches/                     # upstream 수정의 재현 가능한 patch
  scripts/e2e/                 # build, run, evaluate, analysis entry points
  tests/                       # 프로젝트 단위/회귀 테스트
  data/maps/Town01_full/       # full shell용 Lanelet2/PCD/projector bundle
  data/maps/C_track_1_0_7_full/
  data/maps/Woraksan_v1_0_3_parking_lot_hegiht_fit_full/
  data/ml_models/vad/v0.1/     # ignored model/engine artifacts
  data/routes/custom_maps/     # C-track/월악산 고정 및 map-aware goal route
  data/routes/                 # 수동으로 생성한 나머지 route JSON
  data/vendor/cuda-12.8/       # ignored project-local CUDA 12.8 toolchain
  data/vendor/spconv-cu128/    # ignored local C++ dependency root
  data/vendor/acados-v0.5.3/   # ignored local acados source/install and Python venv
  artifacts/
    demos/                     # 최초 full UI와 camera 시각 증거
    diagnostics/               # 경로/제어 원인 분석과 무효 실험
    regressions/               # 여러 route/weather suite 결과
    runs/                      # 비교 조건이 유효한 PASS/FAIL episode와 후보 실행
    summary/2026-08-28/custom_maps/  # custom map 대표 PNG/GIF/결과 JSON
  build/, install/             # 현재 실행에 필요한 colcon 생성물, 임의 이동 금지
  log/                         # colcon 로그; latest target만 남기고 과거 로그 정리 가능
```

Python `venv`만으로는 ROS, CUDA, TensorRT, CARLA Python API를 격리할 수 없다.
대신 모든 wrapper가 `scripts/e2e/env.sh`를 source하여 이전 Autoware prefix를
제거하고 ROS 2 Humble, 이 workspace, CARLA 0.9.15 Python API만 설정한다.

`runs/`에 있다는 것은 비교 조건이 유효했다는 뜻이며 PASS나 parameter 채택을 뜻하지
않는다. 채택 여부는 tuning 묶음의 `optimization_summary.json` 또는
`experiment_summary.json`과 이 문서의 판정을 따른다.
node 중복, 잘못된 sensor mapping, stale stack처럼 비교 조건이 깨진 실행은
`diagnostics/*_invalid_trials/`에만 둔다.

## 준비와 빌드

이 PC의 기본 CARLA 경로는 다음과 같다.

```text
/home/a/carla-autoware-universe/CARLA_0.9.15
```

다른 위치를 쓸 때는 실행 전에 `CARLA_ROOT`를 지정한다. 모델 또는 local
spconv 디렉터리를 다시 만들 때만 아래 명령이 필요하다.

```bash
scripts/e2e/download_vad_models.sh
scripts/e2e/setup_spconv.sh
```

### CUDA 12.8

Autoware 1.9.0 source는 CUDA 12.8 API를 사용한다. 이 PC에 기본 연결된 구버전
CUDA header로 빌드하면 `cudaStreamGetDevice`를 찾지 못해 `cuda_blackboard`에서
실패한다. 다음 스크립트는 system CUDA나 NVIDIA driver를 변경하지 않고 필요한
CUDA 12.8 compiler/header/runtime package를 다운로드해
`data/vendor/cuda-12.8/root/usr/local/cuda-12.8`에 추출한다.

```bash
scripts/e2e/setup_cuda_12_8.sh
source scripts/e2e/env.sh
"${CUDACXX}" --version
```

`build.sh`도 이 setup을 먼저 호출하며, `env.sh`는 준비된 local 12.8의 `nvcc`와
library를 우선한다. TensorRT root는 별도 설정이다. 기본값은
`/usr/local/cuda`이고 다른 설치를 쓸 때는 다음처럼 지정한다.

```bash
export AUTOWARE_E2E_TENSORRT_ROOT=/path/to/tensorrt/root
```

이 local CUDA setup은 kernel driver를 교체하거나 system package를 install하지
않는다. NVIDIA apt repository 접근과 다운로드 네트워크는 필요하다.

### acados v0.5.3

Full Autoware의 path/trajectory optimizer는 acados CMake package와 Python code
generator를 요구한다. `build_full.sh`는 Autoware 1.9.0의 Ansible role과 같은
버전 및 옵션으로 다음 setup을 먼저 실행한다.

```bash
scripts/e2e/setup_acados.sh
source scripts/e2e/env.sh
```

이 스크립트는 acados `v0.5.3` source와 submodule을
`data/vendor/acados-v0.5.3`에 shallow clone하고, 그 source tree 안에
`ACADOS_WITH_QPOASES=ON` 및 position-independent code로 install한다. 공식
`tera_renderer v0.2.0` binary는 SHA-256을 확인해 `bin/t_renderer`에 놓고,
전용 `.venv`에는 `casadi`, `sympy`, editable `acados_template`를 설치한다.
System `/opt/acados`와 `.bashrc`는 변경하지 않는다.

`env.sh`는 local install을 `ACADOS_SOURCE_DIR`, `CMAKE_PREFIX_PATH`,
`LD_LIBRARY_PATH`에 연결한다. 다른 project-local checkout을 쓸 때만 build와
실행 전에 `AUTOWARE_E2E_ACADOS_ROOT`를 지정한다.

### NVIDIA user-space compatibility 우회

`doctor.sh` 또는 `nvidia-smi`가 loaded kernel module과 user-space NVIDIA
library의 version mismatch를 보고하면 우선 재부팅으로 맞추는 것이 정상적인
해결이다. 당장 system을 변경할 수 없는 개발 환경에서만 다음 스크립트를 쓴다.

2026-08-27 재부팅 후 이 PC에서 직접 확인한 kernel module, NVML/OpenGL user-space,
`nvidia-smi` 값은 모두 `580.178.04`다. 현재 정상 절차는 다음과 같다.

```bash
unset AUTOWARE_E2E_NVIDIA_COMPAT_ROOT
source scripts/e2e/env.sh
nvidia-smi
```

재부팅 후에도 실제 mismatch가 확인될 때만 `scripts/e2e/setup_nvidia_compat.sh`를
실행하고, 스크립트가 출력한 경로를 그 shell에 설정한다.

```bash
scripts/e2e/setup_nvidia_compat.sh
export AUTOWARE_E2E_NVIDIA_COMPAT_ROOT=/path/printed/by/setup_nvidia_compat
source scripts/e2e/env.sh
```

스크립트는 현재 loaded kernel module과 같은 version의 NVIDIA library를 project
아래에 추출하고, `env.sh`가 그 경로를 `PATH`와 `LD_LIBRARY_PATH` 앞에 둔다.
출력된 export는 shell-local이다. 우회가 정말 필요한 동안에는 새 terminal마다 같은
값을 설정해야 한다.
여기에는 CARLA의 NVIDIA Vulkan/OpenGL 초기화에 필요한 matching
`libnvidia-gpucomp`도 포함된다. 이 library가 빠진 mismatch 환경에서는 CARLA가
`llvmpipe`로 떨어지거나 시작 중 멈출 수 있다. 정상 driver 환경에서는 이 변수를
설정하지 않는다. 이는 project-local 임시 개발용 우회이며 재부팅이나 정상적인
driver/package 정합성 복구를 대신하지 않는다.

재부팅 전 mismatch 상태에서는 compatibility root 없이 CARLA를 재시작했을 때 약
60초 뒤 render thread timeout과 exit 139가 재현됐고, matching root 적용 뒤에는 그
경계를 통과했다. 이는 과거 진단 기록이다. Camera gap/MRM이 함께 발생한 screen 29는
문서 맨 앞의 invalid-trial 절처럼 control 비교에서 제외하며, 현재 native 실행에
과거 `580.173.02` root를 다시 주입하지 않는다.

### Town01 full map

Minimal launch는 CARLA town 이름만 필요하지만, full Autoware map stack은
Lanelet2, point cloud, `map_projector_info.yaml`이 들어 있는 directory가
필요하다. 기본 Town01 bundle은 다음 순서로 준비한다.

```bash
scripts/e2e/setup_town01_full_map.sh --dry-run
scripts/e2e/setup_town01_full_map.sh
```

기본 source는 다음 두 파일이다.

```text
~/Downloads/sample-planning-map/vector_maps/lanelet2/CARLA Town/Town01_light.osm
~/Downloads/sample-planning-map/point_cloud_maps/CARLA Town/Town01.pcd
```

스크립트는 source hash, Lanelet2 구조와 PCD 좌표 범위를 검사하고 source를
복사하거나 수정하지 않는다. Target의 OSM/PCD는 source를 가리키는 symlink이며,
Local projector metadata만 새로 만든다. 결과는
`data/maps/Town01_full`이다. 다른 source와 target은 `--osm-source`,
`--pcd-source`, `--target-dir` 또는 대응 환경변수로 지정한다. Custom map은
이 script에 넣지 않는다. C-track/월악산은 asset hash, transform과 파생 OSM 규칙을
고정한 `scripts/e2e/setup_custom_full_map.py`를 사용한다.

`run_route_vad_full.sh`는 CARLA world 이름 `Town01`과 Autoware map directory
`Town01_full`을 별도 launch argument로 전달한다. Directory 이름을 CARLA town
이름으로 해석하지 않는다.

### 빌드와 진단

Minimal VAD 경로만 빌드할 때는 다음 순서로 실행한다.

```bash
scripts/e2e/build.sh
scripts/e2e/doctor.sh
```

표준 map/sensing/system/control/API/RViz까지 포함한 full 경로는 별도 명령 하나로
전체 의존성을 빌드한다. `build.sh`를 먼저 실행할 필요는 없다.

```bash
scripts/e2e/build_full.sh
scripts/e2e/doctor.sh
```

Full 빌드는 패키지를 순차 처리하고 각 패키지 내부 병렬 작업을 기본 2개로
제한한다. 중단된 빌드를 완료된 패키지 다음부터 이어갈 때는 다음처럼 실행한다.

```bash
AUTOWARE_E2E_FULL_BUILD_RESUME=1 scripts/e2e/build_full.sh
```

패키지 내부 병렬 수는 `CMAKE_BUILD_PARALLEL_LEVEL`로 조정할 수 있다.

`doctor.sh`의 core package 진단 통과는 build/runtime prerequisite 확인이다.
그 자체가 full Autoware launch가 정상 기동했거나 주행했다는 검증은 아니다.

두 build script는 재현 가능한 core project patch를 먼저 적용하고, 이미
적용된 경우 건너뛴다.

- `patches/autoware_tensorrt_vad_candidate_uuid.patch`: candidate와 generator UUID를
  같게 만들어 route manager가 여섯 command를 안전하게 연결한다.
- `patches/autoware_tensorrt_vad_fp16_layer_norm.patch`: head FP16 A/B를 위한 선택적
  LayerNorm FP32 경계를 추가한다. 권장 profile은 head FP32를 유지한다.
- `patches/autoware_tensorrt_vad_frame_assembly.patch`: exact-first 6-camera frame
  assembler, bounded approximate fallback, queue 크기와 image QoS parameter를 추가한다.
- `patches/autoware_mission_planner_lane_only_no_area.patch`: `allow_area=false`이거나
  Lanelet2 `areaLayer`가 비어 있으면 mission planner가 lane-only shortest-path API를
  사용한다. Town05_Opt 실패 topology의 ID `[22775, 28467, 1850]`를 재현한 합성
  lanelet 연결과 빈 area layer, 그리고 `allow_area=true` lane-area-lane 동작을 C++
  회귀 fixture로 고정한다. 실제 Town05 맵에서의 증명은 별도 matrix E2E 결과다.
- `patches/autoware_carla_interface_camera_fast_options.patch`: mapping의 `sensor_tick`과
  postprocess option을 전달한다. CARLA 0.9.15 GPU callback이 같은 capture frame에
  서로 다른 늦은 header timestamp를 붙이는 경우 frame ID와 fixed step으로 capture
  stamp를 복원하고, 완전한 6-camera bundle을 직렬 발행한다.
- `patches/autoware_carla_interface_vehicle_status_contract.patch`: Ackermann virtual tire
  angle, ROS yaw-rate/velocity sign와 단위를 Autoware message contract에 맞춘다.

Mission planner patch 적용기는
`patches/autoware_mission_planner_lane_only_no_area.manifest.json`에 고정된 Autoware
Universe `0.52.0` commit과 패치 전/후 소스 SHA를 먼저 검사한다. 두 파일이 정확한
BASE 또는 PATCHED 한 상태가 아니거나 부분 적용·추가 drift가 있으면 빌드를 중단한다.
Minimal `build.sh`도 `autoware_mission_planner_universe`를 명시적으로 빌드하고,
실제로 로드되는 lanelet2 plugin library의 marker/SHA와 exact source/patch SHA를
`build/autoware_mission_planner_universe/e2e_lane_only_build_provenance.json`에 기록한다.
Town matrix runner는 campaign plan을 만들기 전에 이 provenance를 새로 검증한다.

Route logic 회귀 테스트는 `colcon test`에도 등록되어 있다.

```bash
source scripts/e2e/env.sh
colcon test --packages-select autoware_e2e_vad_launch
colcon test-result --verbose
```

첫 VAD 실행에서는 ONNX로부터 GPU 전용 TensorRT engine 세 개를 만들 수 있어
몇 분이 걸릴 수 있다. Driver/library 오류는 먼저 재부팅으로 정상 정합성을 복구하고,
실제 mismatch가 남은 경우에만 앞의 NVIDIA compatibility 절을 따른다.

## 포트와 프로세스 원칙

두 실행 경로의 **기본 포트가 다르다.** `run_vad.sh`와
`carla_vad.launch.xml`의 minimal 기본값은 기존 호환을 위해 2000이고,
`run_route_vad_full.sh`와 `carla_vad_full.launch.xml`의 full 기본값은 별도 시험용
2100이다. `CARLA_PORT` 환경변수나 `carla_port:=...` launch argument로 둘 다
명시적으로 바꿀 수 있다.

기존 CARLA가 2000번 포트에서 실행 중이면 project script로 건드리지 않는다. 병렬
연결 smoke test가 꼭 필요할 때만 새 CARLA에 2100을 사용하고, timing/full 주행은
기존 CARLA 종료 후 또는 별도 GPU/PC에서 수행한다. 아래 예제는 port 충돌을 피하도록
minimal 실행도 모두 2100을 명시한다.

포트와 ROS domain 분리는 network와 ROS graph 충돌만 막는다. 두 CARLA와 TensorRT
VAD를 한 GPU에서 동시에 실행하면 GPU 사용률과 온도가 포화되고 simulation
real-time factor가 크게 낮아지며, standard diagnostic에서 일시적인 stale가 발생할
수 있다. 병렬 실행 가능성 확인과 성능 측정은 구분해야 한다. 반복 회귀와 timing
측정에는 기존 CARLA를 종료할 수 있을 때 project CARLA를 단독으로 실행하는 방식을
권장하며, 병렬 실행에는 충분히 큰 wall timeout을 사용한다.

가장 중요한 규칙은 CARLA synchronous world의 tick owner가 하나뿐이어야
한다는 것이다.

- `run_route_vad.sh` 또는 `run_route_vad_full.sh`가 실행 중일 때
  `autoware_carla_interface`가 유일한 tick owner다.
- 그 상태에서 다른 클라이언트가 `world.tick()`, `load_world()`,
  `apply_settings()`를 호출하면 안 된다.
- `prepare_carla_route.sh`는 town load, weather 변경, async 전환을 수행하므로
  bridge가 완전히 종료된 상태에서만 실행한다.
- RViz와 rqt는 ROS topic만 구독하므로 계속 켜 두어도 된다.
- 같은 map과 quality에서 route/weather만 바꾸는 episode 사이에는 CARLA server를
  재시작할 필요가 없다. Map 또는 quality를 바꿀 때는 현재 server를 종료하고
  `run_carla_map.sh`로 cold-start한다.

Suite는 자신이 시작한 bridge/VAD를 별도 process group으로 관리하고 완전히
종료한 다음 다음 route를 준비한다. 같은 ROS domain에서 기존
`autoware_carla_interface`, `vad_carla_tiny`, `vad_route_manager`가 보이면
안전상 실행을 거부한다. `--skip-stack-preflight`는 이 검사를 끄므로 일반
테스트에서는 사용하지 않는다. 다른 ROS domain의 외부 bridge까지 자동으로
검출하지는 못하므로 같은 CARLA 포트에 별도 tick owner를 연결하지 않아야 한다.

## Minimal 단일 경로 수동 테스트

### 1. CARLA 서버 시작

Terminal 1:

```bash
export CARLA_PORT=2100
scripts/e2e/run_carla.sh -RenderOffScreen -nosound -carla-port=2100
```

CARLA 화면 자체가 필요하면 `-RenderOffScreen`을 빼도 된다. VAD는 실제 camera
sensor image가 필요하므로 CARLA의 `-RenderOffScreen`은 가능하지만
`-nullrhi`처럼 sensor rendering까지 없애는 옵션은 사용하면 안 된다.
기본 world가 아닌 packaged map을 쓸 때는 `run_carla.sh` 대신 문서 앞부분의
`run_carla_map.sh MAP --port 2100 --quality Low|Epic -- ...` 형식으로 시작한다.

### 2. Autoware 시각화 시작

Terminal 2:

```bash
scripts/e2e/run_visualization.sh
```

RViz와 `rqt_image_view`는 아직 topic이 없어도 대기하다가 VAD stack이 시작되면
연결된다. RViz에는 다음 항목이 표시된다.

- 녹색 reference route
- 파란색 actual odometry path
- 선택된 `/planning/trajectory`와 여섯 VAD candidate
- 시작점, 목표점, remaining distance, CTE, trajectory correction, command, goal 상태
- 예측 객체와 VAD BEV map marker

`rqt_image_view`는 학습 순서에 맞춘 surround camera 6개 합성 영상을 표시한다.
주행 중 동일한 화면을 파일로 남길 때는 다음 명령을 사용할 수 있다.

```bash
scripts/e2e/capture_ros_image.sh artifacts/vad_surround_input.png
```

### 3. Route JSON 준비

VAD/bridge가 실행 중이지 않은 상태에서 Terminal 3에서 실행한다.

```bash
export CARLA_PORT=2100

scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario lane_follow \
  --start-index 143 \
  --goal-index 145 \
  --output data/routes/town01_lane_follow_clear_noon.json
```

생성 파일에는 town, weather, CARLA spawn transform, ROS 좌표계의 시작/목표,
route 길이, waypoint, 누적 거리, `RoadOption`, VAD command가 들어간다.
`run_route_vad.sh`는 이 JSON에서 town과 ego spawn point를 자동으로 읽는다.

Spawn index를 직접 정하지 않고 조건에 맞는 route를 찾게 할 수도 있다.

```bash
scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario left \
  --min-distance 20 \
  --max-distance 45 \
  --preferred-distance 30 \
  --output data/routes/town01_auto_left.json
```

`scenario`는 `lane_follow`, `straight`, `left`, `right`, `any` 중 하나다.
자동 선택은 CARLA 도로망에서 조건에 맞는 route를 찾는 기능일 뿐, 그 route를
VAD가 반드시 성공한다는 의미는 아니다. 회귀 테스트에는 고정 spawn index를
사용해야 결과를 비교하기 쉽다.

### 4. VAD, bridge, Autoware controller 시작

Route 준비가 끝난 뒤 같은 Terminal 3에서 실행한다.

```bash
scripts/e2e/run_route_vad.sh \
  data/routes/town01_lane_follow_clear_noon.json \
  carla_port:=2100
```

이 명령은 route manager mode를 켠다. Upstream 기본 선택 출력은
`/planning/vad/raw_trajectory`, manager가 이번 route command로 선택한 원본은
`/planning/vad_route/selected_raw_trajectory`로 분리된다. 실제 controller가
구독하는 `/planning/trajectory`는 `vad_route_manager` 하나만 publish한다.

### 5. 자동 engage와 평가

Terminal 4:

```bash
scripts/e2e/route_test.sh \
  --route-file data/routes/town01_lane_follow_clear_noon.json \
  --result artifacts/runs/2026-08-23/minimal_lane_follow/town01_lane_follow_clear_noon.json \
  --sim-timeout 180 \
  --wall-timeout 900 \
  --stall-timeout 20 \
  --max-cte 2.0 \
  --path-sample-distance 0.1 \
  --report-interval 10
```

Evaluator는 필요한 topic과 engage service가 준비될 때까지 기다리고, 먼저
disengage 상태를 확인한 다음 engage한다. 주행 중 다음 조건을 검사한다.

- `goal_reached` Bool과 route status가 모두 완료 상태인지
- JSON의 실제 `goal_ros_pose`까지 직선 거리가 제한 이내인지
- 정지 속도가 제한 이하이며 완료 판정 뒤 새 odometry에서도 유지되는지
- CTE 절댓값이 제한을 넘지 않는지
- 선택 raw 궤적의 corridor 보정량이 기본 15.0 m 제한을 넘지 않는지
- remaining distance가 일정 시간 이상 줄지 않는지
- odometry/trajectory/route metric이 stale하지 않은지
- simulation time과 wall-clock timeout을 넘지 않는지
- Autoware가 목표 전에 임의로 disengage되지 않는지

성공, 실패, `Ctrl-C`, `SIGTERM` 어느 경우든 evaluator는 마지막에 control을
disengage하고 JSON을 atomic write한다. CARLA의 real-time factor가 매우 낮을 수
있으므로 simulation timeout과 wall timeout은 별도로 둔다.

### 6. 결과 PNG 생성

Evaluator가 끝난 뒤 CARLA나 ROS 없이 생성할 수 있다.

```bash
scripts/e2e/render_route_result.sh \
  data/routes/town01_lane_follow_clear_noon.json \
  artifacts/runs/2026-08-23/minimal_lane_follow/town01_lane_follow_clear_noon.json \
  --output artifacts/runs/2026-08-23/minimal_lane_follow/town01_lane_follow_clear_noon.png
```

PNG의 녹색 선은 reference route다. 실제 경로는 성공일 때 파란색, 실패일 때
빨간색이며 시작점, 목표점, 최종 위치와 주요 metric이 함께 표시된다. Corridor가
한 번이라도 개입한 성공은 단순 `PASS`가 아니라 `HYBRID PASS`로 표시한다. 최신
evaluator JSON은 `actual_path`를 포함한다. 이전 JSON에 이 배열이 없으면
renderer는 initial/final 위치만 연결한 fallback을 명시한다.

### 7. 다음 경로 또는 종료

Evaluator가 disengage한 것을 확인한 뒤 Terminal 3의 VAD launch를 `Ctrl-C`로
완전히 종료한다. 그 다음에만 새로운 `prepare_carla_route.sh`를 실행한다.

여러 경로를 수동 반복할 때의 순서는 항상 다음과 같다.

```text
evaluator 완료/disengage
  -> VAD/bridge launch Ctrl-C 및 완전 종료
  -> 다음 town/weather/route 준비
  -> VAD/bridge 새로 시작
  -> 다음 evaluator 시작
```

RViz/rqt와 CARLA 서버는 이 과정에서 계속 실행해도 된다. 모든 테스트가 끝난
뒤 visualization을 먼저 종료하고 CARLA 서버를 종료한다.

## Full Autoware UI bring-up

이 절은 표준 Autoware module/UI를 포함한 새 경로를 기동하는 방법이다. 위
minimal PASS를 재현하는 절차와 구분한다.

CARLA server를 2100번 port에 시작하고, bridge가 아직 없는 상태에서 위와 같은
방법으로 route JSON을 준비한다. Town01 full map이 없다면 한 번 준비한다.

```bash
scripts/e2e/setup_town01_full_map.sh

export CARLA_PORT=2100
scripts/e2e/prepare_carla_route.sh \
  --port 2100 \
  --town Town01 \
  --weather ClearNoon \
  --scenario lane_follow \
  --start-index 143 \
  --goal-index 145 \
  --output data/routes/town01_lane_follow_clear_noon.json
```

그 다음 full wrapper를 실행한다.

```bash
export CARLA_PORT=2100
scripts/e2e/run_route_vad_full.sh \
  data/routes/town01_lane_follow_clear_noon.json
```

Wrapper가 JSON의 town/spawn/truth initial pose와 full map path를 계산한다. Full
launch는 기본으로 RViz를 함께 시작하므로 `run_visualization.sh`를 별도로 실행할
필요가 없다. 이 RViz 설정은 standard Autoware display와 VAD 전용 display를 한
창에서 보여준다.

제어를 engage하기 전에 다른 terminal에서 publisher 소유권과 route 정렬을
확인한다.

```bash
source scripts/e2e/env.sh
ros2 topic echo /planning/vad_route/standard_route_aligned --once
ros2 topic echo /system/vad/aeb_configured --once
ros2 topic echo /planning/route_state --once
ros2 topic info -v /localization/kinematic_state
ros2 topic info -v /perception/object_recognition/objects
ros2 topic info -v /planning/trajectory
```

`standard_route_aligned.data`와 `aeb_configured.data`가 모두 `true`이고 canonical
state/object/trajectory마다 의도한 publisher가 하나인지 확인해야 한다. Route
alignment가 false이면 manager가 stop gate를 유지한다. AEB configured가 false이면
표준 AEB가 VAD object 입력으로 전환되지 않은 상태이므로 full evaluator도
engage하지 않는다.

다음 read-only validator는 node/topic 존재, publisher 단일 소유권, 여섯 camera의
bridge-to-VAD 연결, route/AEB readiness와 live message freshness를 한 번에 검사한다.

```bash
source scripts/e2e/env.sh
scripts/e2e/validate_full_stack.py \
  --verbose \
  --json artifacts/runs/2026-08-23/full_stack_lane_follow/full_stack_validation.json
```

이 validator는 full launch의 **기본 hybrid profile**만 대상으로 한다. 즉 CARLA
truth localization, canonical VAD objects, VAD route manager, standard mission
planner/adapter, standard control과 VAD용 AEB 설정을 기대한다. Conventional module
switch를 켠 구성이나 `launch_standard_mission_planner=false` JSON-only mode는 graph
소유권과 필수 topic이 달라 이 validator에서 의도적으로 통과하지 않는다. Validator
PASS는 startup과 data-flow 확인이지 engage, 차량 이동 또는 목표 정지 성공이
아니다.

Full stack의 표준 Autoware 상태 전이까지 포함해 주행을 평가할 때는 다른
terminal에서 `--full-stack`을 명시한다.

```bash
source scripts/e2e/env.sh
scripts/e2e/route_test.sh \
  --full-stack \
  --route-file data/routes/town01_lane_follow_clear_noon.json \
  --result artifacts/runs/2026-08-23/full_stack_lane_follow/town01_lane_follow_full_stack.json \
  --ready-timeout 180 \
  --service-timeout 30 \
  --engage-timeout 60 \
  --sim-timeout 180 \
  --wall-timeout 900 \
  --stall-timeout 20 \
  --data-stale-timeout 60 \
  --max-cte 2.0 \
  --report-interval 10
```

이 모드에서는 legacy engage service를 직접 호출하지 않는다. Evaluator는
localization state가 `INITIALIZED`이고 standard mission route가 JSON goal과
정렬됐으며 `/system/vad/aeb_configured=true`이고 autonomous operation mode가
available일 때까지 기다린다. 그 다음
`/api/operation_mode/change_to_stop`으로 초기 상태를 정리하고
`/api/operation_mode/change_to_autonomous`로 전환한다. CARLA control mode가 이미
활성 상태가 아닌 구성에서만 `enable_autoware_control`도 호출한다. 성공, 실패,
signal 종료와 관계없이 마지막에는 `change_to_stop`을 호출하고 stable `STOP`
상태를 확인한 뒤 JSON 결과를 기록한다. Operation-mode 요청은 제한 시간 안에서
service unavailable과 명시된 transient 응답만 재시도하며, 진단 gate를 우회하지
않는다. 각 요청과 최종 상태도 artifact에 남긴다. 완료 시에는 route manager의
Bool/status만 믿지 않고 JSON의 정확한 goal까지 거리, 정지 속도, 완료 이후 새
odometry를 함께 검사한다. `--full-stack`을 생략하면 기존 minimal 경로의
engage/disengage 동작이 그대로 사용된다. 현재 `--full-stack` evaluator도 standard
route alignment를 필수로 검사하므로 JSON-only reduced-safety mode용 평가기라고
해석하면 안 된다.

### Conventional module switch

Full launch의 switch는 동일한 canonical topic의 publisher 중복을 피하기 위한
소유권 경계다.

| Launch argument | 기본값과 동작 | 켰을 때 주의점 |
|---|---|---|
| `launch_conventional_localization` | `false`: CARLA truth state 사용 | `true`: truth state node가 억제되고 standard localization이 state를 소유한다. NDT/EKF 초기화와 sensor/map 정합성이 별도로 준비되어야 한다. |
| `launch_carla_truth_state_bridge` | `true`: conventional localization이 꺼졌을 때만 truth state publish | `false`: 다른 localization publisher가 반드시 있어야 한다. |
| `launch_conventional_perception` | `false`: VAD object가 canonical object topic을 소유 | `true`: standard perception이 canonical object를 소유하고 VAD object는 `/perception/vad/objects` shadow topic으로 이동한다. |
| `configure_aeb_for_vad_objects` | `true`: standard AEB를 끄지 않고 point cloud 입력을 끈 뒤 VAD `PredictedObjects` 입력을 켠다. | `false`: `/system/vad/aeb_configured` 준비 조건이 충족되지 않아 기본 `--full-stack` evaluator는 engage하지 않는다. 검증된 다른 AEB 입력 구성이 있을 때만 사용한다. |
| `launch_conventional_planning` | `false`: VAD route manager가 final trajectory를 소유 | `true`: standard planning이 final trajectory와 자체 mission planner를 소유한다. VAD route manager와 standalone mission planner/adapter는 억제되고 VAD raw trajectory만 shadow로 남는다. |
| `launch_standard_mission_planner` | `true`: VAD planning mode에서 standard route UI/API를 띄우고 1초 alignment heartbeat를 필수 gate로 사용 | `false`: planner/adapter와 alignment gate를 함께 끄고 JSON만 따라가는 reduced-safety mode가 된다. 표준 route UI/API 정합성 보호가 없으며 기본 full validator/evaluator profile의 대상이 아니다. Conventional planning이 켜지면 이 standalone planner 설정은 무시된다. |
| `use_route_manager` | `true`: JSON command 선택, corridor, goal stop 적용 | `false`: VAD raw trajectory를 final topic에 relay하므로 goal route 주행/정지 평가용이 아니다. |

이 switch는 module을 실행하거나 억제하는 장치일 뿐, 모든 조합의 sensor model,
parameter, diagnostic, initialization을 자동으로 완성하지 않는다. 특히 세
conventional module을 켜면 일반 Autoware의 map/calibration/model prerequisite가
그대로 필요하며 아직 이 프로젝트의 CARLA full 회귀 테스트로 검증되지 않았다.

### 기존 Full baseline의 검증 상태

2026-08-23 KST에 기본 hybrid profile로 Town01 ClearNoon의 spawn 143→145
lane-follow episode 하나를 끝까지 검증했다. Standard mission route와 JSON route가
정렬된 상태임을 evaluator와 validator artifact에 기록했다. AEB는 VAD object
입력으로 구성됐으며, 종료 후 stable `STOP`도 확인했다.

| 항목 | 결과 |
|---|---:|
| graph validator | `PASS`, 111 nodes / 342 topics / 56 of 56 checks |
| route 길이 | 12.082 m |
| 실제 이동 거리 | 11.083 m |
| 최종 goal 직선 거리 | 0.953 m |
| 최종 속도 | 0.0000004 m/s |
| 최대 절대 CTE | 0.091 m |
| 최대 XY corridor 보정 | 0.000 m |
| simulation / wall 시간 | 14.3 s / 561.2 s |
| 최종 판정 | `PASS / vad_unassisted geometry` |

재현과 확인에 사용하는 artifact는 다음과 같다.

- `artifacts/runs/2026-08-23/full_stack_lane_follow/town01_lane_follow_full_stack.json`: evaluator 원본 결과
- `artifacts/runs/2026-08-23/full_stack_lane_follow/town01_lane_follow_full_stack.png`: reference/actual path와 metric
- `artifacts/runs/2026-08-23/full_stack_lane_follow/full_stack_validation.json`: 56개 graph/data-flow 검사
- `artifacts/runs/2026-08-23/full_stack_lane_follow/full_stack_rviz_driving.png`: 별도 반복 실행에서 캡처한 주행 중 full Autoware RViz
- `artifacts/runs/2026-08-23/full_stack_lane_follow/full_stack_rviz_stopped_final.png`: 목표 정지 후 full Autoware RViz
- `artifacts/runs/2026-08-23/full_stack_lane_follow/full_stack_vad_input_final.png`: 실제 여섯 camera 합성 입력

이 결과는 **한 개의 짧은 직선 lane-follow episode**에 한정된다. Minimal 경로에서
성공한 turn/weather 결과가 full 경로에도 자동 이전되는 것은 아니며, turn, 다른
날씨/town, traffic scenario와 conventional module switch는 아직 full 폐루프로
검증하지 않았다. 기본 profile은 CARLA truth localization과 VAD canonical object를
쓰므로 conventional localization/perception 검증 결과도 아니다. 동일 GPU에서 기존
CARLA와 병렬 실행한 이번 측정은 resource 포화 상태였으므로 timing benchmark로
사용하지 않는다.

## VAD 계산 속도 줄이기

### 2026-08-23 RTX 3060 병렬 실행 역사 기록

> 이 절의 `0.0255 RTF`, `92 C`, thermal slowdown 수치는 2026-08-23에 RTX 3060에서
> CARLA 두 개를 병렬로 띄운 과거 표본이다. 2026-09-01 RTX 3090 Ti 단일 campaign의
> 현재 끊김 원인은 문서 상단 진단을 사용한다. 두 환경의 수치를 합치거나 이 역사
> 표본으로 현재 PC 포화를 주장하지 않는다.

당시 확인된 느린 값은 **VAD TensorRT node 하나의 inference latency가 아니라** CARLA
두 개, Full Autoware, TensorRT VAD와 RViz를 한 RTX 3060에서 함께 실행한 전체 시스템
시간이다. 위 Full episode의 real-time factor는 다음과 같다.

```text
14.3 simulation s / 561.2 wall s = 0.0255 RTF
즉 1 simulation second에 약 39.2 wall seconds
```

2026-08-23에 기존 Woraksan CARLA만 실행된 상태를 다시 표본 측정했을 때도 GPU
utilization과 해당 UE4Editor의 SM 사용률이 각각 약 96%, 온도 92 C, 전력
158/170 W였다. 이때 NVIDIA의 software thermal slowdown도 `Active`였고 graphics
clock은 1695 MHz였다. 즉 병렬 부하뿐 아니라 열에 의한 clock 제한도 실제로 발생하고
있다. 2000번과 2100번 port, 서로 다른 ROS domain은 통신 충돌만 막으며 GPU 자원은
분리하지 않는다. 따라서 지금 결과만으로 "VAD 한 frame이 39초 걸린다"고 결론 내릴
수 없다.

공식 통합 PR은 RTX 3080에서 약 20 ms, 약 3 GB, 안정적인 50 Hz를 보고했다. 다만
그 수치에 image decode/preprocess가 전부 포함됐는지 명시되지 않았고 CARLA와 Full
Autoware까지 포함한 end-to-end benchmark도 아니다. 현재 PC와 직접 비교할 때는 같은
조건의 node 단독 측정이 필요하다.

- [공식 TensorRT VAD 통합 PR의 성능 기록](https://github.com/autowarefoundation/autoware_universe/pull/11578)
- [CARLA 0.9.15 rendering options](https://carla.readthedocs.io/en/0.9.15/adv_rendering_options/)

### 구현된 fast profile

`run_route_vad_fast.sh`는 검증된 baseline을 덮어쓰지 않고 default fast A/B 또는 현재
권장 deployment를 별도로 선택한다.

아래 `--recommended` 열은 camera·VAD·Full-shell deployment base를 뜻한다.
`--recommended` 단독 속도 상한은 `2.5 m/s`이고, v16 30 kph matrix는 여기에
`--speed-30kph`를 추가해 nominal `8.333333 m/s` simulation overlay를 선택했다. 두
구성의 속도 claim을 섞지 않는다.

| 항목 | baseline | default fast A/B | `--recommended` |
|---|---|---|---|
| CARLA camera source | 6 x 1600x900, renderer 최대 20 Hz | 6 x 640x360, `sensor_tick=0.2`로 source 5 Hz | 6 x 640x360, **`sensor_tick=0.0`**, 20 Hz physics frame마다 6대 렌더 |
| ROS camera 발행 | 실효 약 10 Hz, reliable | 5 Hz, best effort | 5 Hz, **reliable**, 같은 CARLA frame의 완전한 6장 bundle |
| VAD transport | JPEG compressed | raw | raw |
| VAD frame sync | exact-first + bounded approximate, queue 8 | exact-first + bounded approximate, queue 8 | exact-first + bounded approximate, queue **32** |
| TensorRT 입력 | 6 x 640x384 | 동일 | 동일 |
| camera helper | JPEG republisher 7개, traffic relay, 6-camera combiner | 모두 끔 | 모두 끔 |
| 다른 CARLA sensor | LiDAR, IMU, GNSS | truth state용 GNSS만 유지 | truth state용 GNSS + VAD acceleration용 IMU 유지 |
| TensorRT precision | backbone FP16, head 2개 FP32 | 기본 동일; `--fp16-heads`만 실험값 | backbone FP16, head FP32 고정 |
| Autoware shell/control | Full baseline | minimal이 기본, `--full` 선택 가능 | Full 고정, MPC `0.12/0.15`, 종단 감속 `0.60 m/s^2`, stock actuation |

Default fast의 source pixel rendering은 이론상 `172.8 MPix/s`에서
`6.912 MPix/s`로 약 96% 줄지만, 현재 `--recommended` source는 모든 20 Hz physics
frame을 렌더하므로 `27.648 MPix/s`, baseline 대비 약 84% 감소다. 두 fast profile의
ROS camera 발행은 5 Hz라 output pixel rate는 `6.912 MPix/s`이고 baseline 실효
10 Hz 대비 약 92% 줄어든다. `--recommended`의 raw BGRA publish payload는 약
27.6 MB/s이므로 같은 PC용 설정이다. CARLA와 Autoware를 다른 PC로 분리하면 network
대역폭 때문에 compressed transport를 다시 비교해야 한다.

Source를 `640x384`가 아니라 `640x360`으로 만든 이유는 기존 `1600x900`과 같은
16:9/FOV를 유지하기 위해서다. VAD CUDA preprocess가 이를 model 고정 크기
`640x384`로 resize할 때 최종 scaled intrinsic은 baseline과 같다. Source 자체를
`640x384`로 바꾸면 세로 FOV와 입력 분포가 달라진다.

Default fast와 `--recommended` 모두 network 한 번의 FLOPs와 ONNX shape는 바꾸지
않는다. 대신 inference
호출을 약 10 Hz에서 5 Hz로 줄이고 CARLA rendering, JPEG encode/decode와 mosaic를
제거한다. `fixed_delta_seconds`는 vehicle control과 temporal history 간격까지 바꾸므로
성능 knob로 사용하지 않는다. `-nullrhi`와 CARLA no-rendering mode도 RGB 입력을
비우므로 사용할 수 없다.

### 2026-08-23 역사 구성에서 확인한 시스템 부하

아래 순서는 RTX 3060 병렬 CARLA 표본에 대한 당시 조치다. 2026-09-01 all-Town
campaign은 map renderer 안정성과 동일 조건 비교를 위해 `Epic`을 쓰므로, 아래 `Low`
권고를 현재 matrix에 그대로 적용하지 않는다.

| 순서 | 변경 | 이유 |
|---:|---|---|
| 1 | CARLA를 한 개만 실행 | 2000/2100 port와 ROS domain은 GPU를 격리하지 않는다. 시험 시간을 분리하거나 두 번째 GPU/PC를 사용한다. |
| 2 | thermal slowdown 해결 | 측정 당시 기존 CARLA 하나만으로도 92 C와 software thermal slowdown이 확인됐다. 냉각, fan, 흡배기 상태를 먼저 정상화한다. |
| 3 | `Low`, `-nosound`, headless RViz | `run_carla.sh`의 Low 품질을 유지하고 정량 시험에서는 fast 기본 minimal/no-RViz를 사용한다. |
| 4 | TensorRT engine cache 유지 | ONNX나 precision을 바꾸지 않았다면 GPU별 `.engine`을 매 episode 삭제하지 않는다. FP16 head는 별도 이름을 쓴다. |
| 5 | minimal 회귀 후 full UI | route/model 변경은 minimal fast로 먼저 거르고 표준 UI 확인 때만 `--full --visualize`를 사용한다. |

### 현재 artifact에서 고정된 것

| 항목 | 현재 값 | 의미 |
|---|---:|---|
| camera 수와 순서 | 6, FRONT/BACK/FL/BL/FR/BR | camera를 빼거나 순서를 바꿀 수 없다. |
| TensorRT model 입력 | 각 camera 640x384 | 더 작은 network 입력은 새 학습/ONNX/export가 필요하다. |
| backbone precision | FP16 | 이미 절반 정밀도다. |
| temporal/no-prev head | FP32 | FP16은 별도 engine으로만 시험한다. |

- [공식 carla_tiny 0.52.0 설정](https://github.com/autowarefoundation/autoware_universe/blob/0.52.0/e2e/autoware_tensorrt_vad/config/vad_carla_tiny.param.yaml)
- [공식 VAD model artifact](https://huggingface.co/AutowareFoundation/tensorrt_vad)

### A/B 측정 방법

각 실험은 같은 route, weather, traffic, CARLA quality와 동일한 다른 process 조건에서
한 변수만 바꾼다. GPU 확인 전에 NVIDIA library mismatch가 있으면 재부팅을 우선하고,
불일치가 남은 경우에만 앞의 compatibility 절을 적용한다.

```bash
source scripts/e2e/env.sh
nvidia-smi dmon -s pucvmt
nvidia-smi \
  --query-gpu='temperature.gpu,clocks.current.graphics,clocks_event_reasons.sw_thermal_slowdown' \
  --format=csv

# 별도 terminal에서 입력과 VAD 출력의 실제 publish rate 확인
ros2 topic hz /sensing/camera/CAM_FRONT/image_raw             # fast
ros2 topic hz /sensing/camera/CAM_FRONT/image_raw/compressed  # baseline
ros2 topic hz /planning/vad/candidate_trajectories

# evaluator 결과의 전체 real-time factor
jq '.metrics.sim_elapsed_sec / .metrics.wall_elapsed_sec' \
  artifacts/runs/2026-08-23/full_stack_lane_follow/town01_lane_follow_full_stack.json
```

최소 비교 항목은 VAD output Hz, 전체 RTF, GPU utilization/temperature/memory, PASS/FAIL,
최대 CTE, goal 거리, 정지 속도, trajectory correction, 객체와 AEB topic freshness다.
10초짜리 warm-up 뒤 같은 episode를 3회 실행해 중앙값을 쓰면 engine 초기화와 일시적인
부하 영향을 줄일 수 있다.

다음 변경은 성능 개선으로 사용하지 않는다.

- confidence threshold는 postprocess 결과를 바꾸며 network 계산량은 거의 줄이지 않는다.
- timeout이나 controller gain은 inference 속도를 개선하지 않는다.
- JSON의 BEV 크기, query 수, decoder layer를 ONNX와 독립적으로 바꾸지 않는다.
- temporal history를 끄는 동작은 현재 공식 node가 지원하는 fast mode가 아니다.
- TensorRT workspace 크기는 주로 engine build tactic 선택 범위이며 일반적인 runtime 속도 knob가 아니다.
- INT8과 multi-stream은 공식 설계 문서에서도 아직 TODO이므로 calibration/Q-DQ model 없이 precision 문자열만 바꾸지 않는다.

[공식 설계 문서의 INT8/multi-stream TODO](https://github.com/autowarefoundation/autoware_universe/blob/0.52.0/e2e/autoware_tensorrt_vad/docs/design_vad_model.md#L312-L315)를
먼저 해결하지 않은 INT8 전환은 현재 권장 경로가 아니다.

## VAD 모델 손보기

가능하지만 무엇을 바꾸는지에 따라 작업 수준이 완전히 다르다.

| 수준 | 가능한 변경 | 재학습/C++ 변경 |
|---|---|---|
| deployment tuning | confidence threshold, detection range, camera sync, engine path/precision | 재학습 없음 |
| compatible model 교체 | 기존과 입출력 이름, shape, 의미가 같은 backbone/head ONNX 3개 | 새 학습/export 필요, C++는 보통 유지 |
| architecture 변경 | camera 수, BEV 크기, decoder/query 수, 출력 의미, 새 navigation 입력 | 새 학습/export와 VAD C++ adapter 변경 필요 |

### 바로 조정할 수 있는 설정

편집용 overlay는
`autoware_e2e_vad_launch/config/vad_user_override.param.yaml`이다. Official YAML이나
다운로드한 JSON/ONNX를 직접 덮어쓰지 않고 다음처럼 마지막 parameter layer로 넣는다.

```bash
scripts/e2e/run_route_vad_fast.sh \
  --model-override autoware_e2e_vad_launch/config/vad_user_override.param.yaml \
  data/routes/town01_lane_follow_clear_noon.json \
  carla_port:=2100
```

Threshold를 높이면 false positive가 줄 수 있지만 miss가 늘고, 낮추면 반대다. 이는
postprocess 결과만 바꾸므로 inference FLOPs를 거의 줄이지 않는다. Detection range도
network가 계산한 결과를 나중에 거르는 값이다. `default_command`는 여섯 branch 중
upstream 기본 출력 선택만 바꾸며, 이 프로젝트의 목표 주행은 route manager가 각
candidate를 선택하므로 별도 재학습 없이 command lookahead와 goal stop을 조정할 수 있다.

### ONNX를 교체할 때 지켜야 할 계약

현재 `carla_tiny v0.1` deployment의 핵심 계약은 다음과 같다.

- backbone: `img [6,3,384,640]` -> `out.0 [6,1,256,12,20]`
- temporal head: feature, `shift [1,2]`, `lidar2img [6,4,4]`,
  `can_bus [1,18]`, `prev_bev [21200,1,256]`
- 주요 출력: `out.ego_fut_preds [1,6,6,2]`,
  `out.bev_embed [21200,1,256]`와 object/map decoder 출력
- custom TensorRT plugin: rotate, multi-scale deformable attention, select-and-pad

이 이름, shape와 좌표 의미가 같으면 새 ONNX와 고유한 `.engine` 경로를 overlay에
지정해 교체할 수 있다. 기존 engine 이름을 재사용하면 이전 cache가 섞일 수 있다.
Decoder layer 수 등 shape를 바꾸면 현재 decoder axis를 전제로 한 C++도 함께
고쳐야 한다. PyTorch/ONNX/TensorRT 출력 parity를 먼저 확인한 뒤 같은 고정 route,
날씨와 metric으로 폐루프 회귀한다.

### 현재 공식 artifact의 재학습 한계

AutowareFoundation이 배포한
[`carla_tiny v0.1`](https://huggingface.co/AutowareFoundation/tensorrt_vad)은 model
card부터 **Simulation only**로 명시돼 있다. Artifact에는 실행용 ONNX, JSON과
TensorRT 설정은 있지만 그 ONNX를 정확히 재현하는 PyTorch checkpoint, 학습 config와
export script는 포함돼 있지 않다. 정확한 checkpoint/export 공개 요청도
[공식 discussion](https://github.com/orgs/autowarefoundation/discussions/7183)에서 해결되지
않았다. 따라서 **현재 ONNX 파일에 optimizer를 붙여 그 가중치부터 직접 fine-tuning하는
지원·재현 가능한 경로는 없다.** Deployment parameter를 바꾸는 일과 학습 가능한 원본
모델을 확보하는 일은 구분해야 한다.

실차 domain adaptation을 시작할 때 검토할 공개 경로는 다음과 같다.

| 출발점 | 공개된 학습 자산 | 이 프로젝트에서의 판단 |
|---|---|---|
| AWF `tensorrt_vad carla_tiny` | CARLA용 ONNX/JSON, TensorRT runtime | 현재 simulation replay 기준이다. 실차 fine-tuning 원본으로 사용하지 않는다. |
| [`hustvl/VAD`](https://github.com/hustvl/VAD) | 실제 nuScenes 6-camera용 PyTorch config/checkpoint와 [학습 절차](https://github.com/hustvl/VAD/blob/main/docs/train_eval.md) | **실차 학습의 우선 출발점**이다. 이 프로젝트의 camera/command/좌표/출력 계약으로 dataset converter와 export adapter를 만들어야 한다. |
| [`Bench2DriveZoo VAD`](https://github.com/Thinklab-SJTU/Bench2DriveZoo/tree/uniad/vad) | CARLA Bench2Drive 학습 code/config | CARLA 학습 참고용이다. 공개 base는 BEV `200x200`, decoder 6 layer이고 `carla_tiny`는 `106x200`, 3 layer라 그대로 교체할 수 없다. Repository의 CC BY-NC-ND 4.0 조건도 수정·배포·상업 사용 전에 별도로 검토한다. |
| [`AutowareFoundation/auto_e2e`](https://github.com/autowarefoundation/auto_e2e) | 별도의 최신 E2E 학습/runtime architecture | 7 cameras, rendered map tile, egomotion/visual history를 입력하고 10 Hz에서 6.4초 acceleration/curvature를 출력한다. 현재 6-camera VAD trajectory node의 drop-in model이 아니다. |

따라서 첫 실차 실험은 `hustvl/VAD`의 nuScenes checkpoint/config를 독립된
container/conda 학습 환경에서 재현하고, 아래 수집 계약을 nuScenes/VAD sample로
변환해 fine-tuning하는 방향이 가장 명확하다. 학습 환경의 Python/CUDA/GCC는 현재
Autoware CUDA 12/TensorRT 10 runtime과 섞지 않는다. Export 단계에서는
[NVIDIA VAD TensorRT adapter](https://github.com/NVIDIA/DL4AGX/tree/master/AV-Solutions/vad-trt#export-to-onnx)를
참고하되, 앞 절의 입력 이름/shape/좌표 의미를 맞춘 ONNX와 PyTorch/ONNX/TensorRT
parity test가 필요하다. Dataset, pretrained checkpoint, code 각각의 license는 실제
사용 목적에 맞는지 시작 전에 확인한다.

## Episode마다 VAD를 재시작하는 이유

VAD는 이전 frame의 BEV feature와 ego motion을 다음 inference에 사용하는
temporal 모델이다. 단순히 ego를 다른 spawn point로 순간 이동하거나 map과
simulation clock만 바꾸면 이전 episode의 내부 상태가 새 장면에 섞일 수 있다.

또한 bridge가 ego vehicle과 여섯 sensor를 route의 시작점에 다시 생성해야 한다.
따라서 route, weather 또는 town이 바뀔 때 VAD node만 parameter로 재사용하지
않고 전체 VAD/bridge launch를 재시작한다. RViz 자체는 temporal state를 갖지 않는다.
Minimal 경로에서 별도 실행한 RViz/rqt는 유지할 수 있지만, Full wrapper가 자식으로
시작한 RViz는 Full launch와 함께 종료·재시작한다.

## Minimal 여러 환경 자동 Suite

현재 `run_route_suite.sh`는 내부에서 `run_route_vad.sh`를 호출하므로 minimal
경로 전용이다. 이 suite의 PASS/FAIL과 PNG를 full Autoware 결과로 간주하지
않는다.

CARLA 서버와 visualization만 실행한 상태에서 수동 VAD stack이 없는지 확인한다.
먼저 실제 실행 없이 설정과 생성될 명령을 확인할 수 있다.

```bash
scripts/e2e/run_route_suite.sh --port 2100 --dry-run
```

짧은 baseline 한 개만 실행:

```bash
scripts/e2e/run_route_suite.sh \
  --port 2100 \
  --case town01_lane_follow_clear_noon
```

기본 6개 case 전체 실행:

```bash
scripts/e2e/run_route_suite.sh --port 2100
```

기본 matrix는 `scripts/e2e/route_suite.default.json`에 있다.

| Case | Route | Weather | Start -> Goal |
|---|---|---|---:|
| `town01_lane_follow_clear_noon` | lane follow | `ClearNoon` | 143 -> 145 |
| `town01_lane_follow_wet_cloudy_noon` | lane follow | `WetCloudyNoon` | 143 -> 145 |
| `town01_lane_follow_hard_rain_noon` | lane follow | `HardRainNoon` | 143 -> 145 |
| `town01_straight_clear_noon` | straight | `ClearNoon` | 229 -> 103 |
| `town01_left_clear_noon` | left | `ClearNoon` | 229 -> 242 |
| `town01_right_clear_noon` | right | `ClearNoon` | 235 -> 58 |

날씨 case 세 개는 같은 경로에서 clear, wet/cloudy, hard rain의 영향을 비교한다.
Turn case 세 개는 맑은 날의 intersection command 선택과 추종을 확인한다.

원하는 case를 여러 번 지정할 수도 있다.

```bash
scripts/e2e/run_route_suite.sh \
  --port 2100 \
  --case town01_lane_follow_clear_noon \
  --case town01_lane_follow_wet_cloudy_noon \
  --case town01_lane_follow_hard_rain_noon
```

Suite는 기본적으로 실패한 case가 있어도 나머지를 계속 실행한다. 첫 실패에서
멈추려면 `--fail-fast`를 사용한다. 별도 matrix는 기본 JSON을 복제해 수정한 뒤
다음처럼 지정한다.

```bash
scripts/e2e/run_route_suite.sh \
  --port 2100 \
  --suite path/to/my_route_suite.json \
  --output-dir artifacts/regressions/route_suites/my_run
```

각 episode에서 suite가 수행하는 작업은 다음과 같다.

1. 같은 ROS domain에 기존 bridge/VAD가 없는지 확인한다.
2. bridge가 없는 상태에서 town, weather, route를 준비한다.
3. VAD/bridge를 새 process group으로 시작해 temporal state를 초기화한다.
4. Evaluator가 engage, 주행 평가, disengage를 수행한다.
5. Suite가 자신이 시작한 VAD/bridge process group만 종료한다.
6. JSON을 읽을 수 있으면 PASS/FAIL과 무관하게 PNG를 생성한다.
7. 다음 episode로 이동한다.

Suite는 외부 CARLA 서버와 RViz/rqt를 시작하거나 종료하지 않는다.

## Minimal Suite 결과 해석

기본 출력 위치는 UTC timestamp가 붙은 디렉터리다.

```text
artifacts/regressions/route_suites/<UTC timestamp>/
  summary.json
  suite.resolved.json
  routes/<case>.json
  results/<case>.json
  images/<case>.png
  logs/<case>.prepare.log
  logs/<case>.stack.log
  logs/<case>.evaluator.log
  logs/<case>.renderer.log
```

`summary.json`에서 먼저 볼 필드는 다음과 같다.

- `success`: 선택한 모든 주행 case가 통과했는지
- `counts.passed`, `counts.failed`, `counts.not_run`: 전체 진행 상태
- `episodes[].reason`: `goal reached`, CTE 초과, stall, timeout 등 최종 이유
- `episodes[].evaluator_exit_code`: evaluator 종료 코드
- `episodes[].stack_exit_code`: episode stack 종료 코드
- `episodes[].visualization_success`: PNG 생성 성공 여부
- `episodes[].route`, `result`, `image`, `logs`: 상세 artifact 경로

`success`는 주행 evaluator 기준이다. Renderer 실패는
`visualization_success=false`로 별도 기록되며 주행 성공을 실패로 바꾸지는 않는다.

`suite.resolved.json`은 실행 당시 default와 episode override가 합쳐진 완전한
설정이다. 나중에 같은 timeout, CTE limit, route와 launch argument로 재현할 때
사용한다.

각 `results/<case>.json`의 주요 항목은 다음과 같다.

- `success`, `reason`: 개별 주행 판정
- `limits`: 이 실행에 적용한 timeout, CTE, progress 기준
- `initial`, `final`: 위치, simulation time, remaining distance, CTE, command,
  route status, goal/engage 상태
- `metrics`: simulation/wall 시간, 실제 이동 거리, 최소 remaining distance,
  최대 CTE, 최대 trajectory correction, 관측한 command
- `assessment.trajectory_geometry`: `vad_unassisted` 또는
  `hybrid_route_assisted`; route 완료와 순수 VAD geometry를 구분
- `actual_path`: PNG에 사용하는 odometry 표본
- `engage.disengage_confirmed`: minimal은 control disengage, full은 stable `STOP` 전환이
  확인됐는지. Full에서는 호환성을 위해 같은 field명을 사용하며
  `is_autoware_control_enabled=false`까지 의미하지 않는다.
- `message_counts`: 필수 topic이 실제로 수신됐는지

실패 분석 순서는 `summary.json -> images/<case>.png -> evaluator.log ->
stack.log -> prepare.log`가 효율적이다. 예를 들어 PNG에서 경로를 크게 벗어났다면
CTE와 command를 먼저 보고, 차량이 움직이지 않았다면 route status와 trajectory,
engage service, candidate timeout 로그를 확인한다.

## Minimal 빠른 입출력 Smoke Test

목표 route가 아니라 모델 입출력과 고정 command만 빠르게 확인할 때는 기존
launch를 사용할 수 있다. 먼저 한 터미널에서 stack을 시작한다.

```bash
export CARLA_PORT=2100
scripts/e2e/run_vad.sh carla_port:=2100
```

다른 터미널에서 topic 검사와 제한 거리 주행을 실행한다.

```bash
scripts/e2e/smoke_test.sh
scripts/e2e/demo_drive.sh 10
```

이 경로는 `default_command` 기반의 짧은 minimal 확인용이다. Minimal 시작점부터
목표점까지의 평가에는 `prepare_carla_route.sh`, `run_route_vad.sh`,
`route_test.sh` 또는 suite를 사용한다. Full 경로의 완료 판정에는 이 smoke
결과를 사용하지 않는다.

수동 engage는 진단 상황에서만 사용한다.

```bash
scripts/e2e/engage.sh true
scripts/e2e/engage.sh false
```

## 한계와 성공의 의미

CARLA planner가 route JSON을 만들었다고 해서 어느 시작점과 목표점에서나
주행이 보장되는 것은 아니다.

- 공식 `carla_tiny`는 전역 목적지, 전체 route, 남은 거리 자체를 입력받지 않는다.
- 실제 VAD maneuver conditioning source는 route JSON이다. Full RViz/API에 보이는
  표준 lanelet route는 start/goal/yaw/segment 정렬을 확인하는 UI 및 stop gate용이며
  VAD 입력을 대체하지 않는다.
- Route manager는 학습된 여섯 로컬 candidate 중 하나를 선택할 뿐, 새로운
  형상의 궤적을 신경망으로 생성하거나 실패 후 전역 replanning하지 않는다.
- Simulation adapter의 baseline corridor 반폭은 0.5 m이고, `--recommended`는
  회전 안쪽에 `0.20 m` 한계를 추가한다. 선택된 궤적이 이 경계를 넘으면 route
  manager가 보정한다. 따라서 corridor가 개입한 episode를
  순수 VAD 경로 일반화 성공이라고 해석하면 안 되며 `maximum_trajectory_correction_m`
  값을 함께 보고해야 한다.
- 월악산의 `--tight-corridor`는 모든 command의 반폭을 `0.20 m`로 제한해 해당 고정
  route의 최대 CTE를 줄인 맵용 옵션이다. C-track이나 Town 회전 route에 자동 적용하지
  않으며, VAD raw geometry보다 route adapter의 비중이 더 커진다.
- 교차로에서 필요한 command를 미리 선택해도 candidate가 도로 곡률과 장면을
  충분히 표현하지 못하면 CTE 초과 또는 stall이 발생할 수 있다.
- 현재 goal stop은 모델 출력이 아니라 route manager가 속도와 정지점을
  후처리한 결과다.
- Route 생성기와 loader는 마지막 route point가 JSON `goal_ros_pose`와 1 mm 이내로
  일치하도록 강제한다. Evaluator는 새 odometry를 받은 뒤 실제 goal 직선 거리
  1.5 m 이하와 속도 0.15 m/s 이하까지 확인한다. 이는 해당 episode의 완료 증거이지
  일반적인 주행 안전성 증거는 아니다.
- VAD가 객체를 예측하더라도 이 테스트 통과는 충돌 회피와 functional safety가
  검증됐다는 의미가 아니다. 표준 AEB는 켜 두고 VAD `PredictedObjects` 입력으로
  전환하지만, 이 연결 자체도 safety 검증을 의미하지 않는다.
- 현재 CARLA interface는 실제 제어 인계 상태와 무관하게
  `/vehicle/status/control_mode`를 항상 `AUTONOMOUS`로 report한다. 따라서 RViz의
  ControlMode 표시만으로 operation mode 전환 성공을 판단하지 않으며 full
  evaluator는 표준 operation-mode API 상태와 service 전이를 별도로 검사한다.
- 현재 CARLA interface는 `/vehicle/status/gear_status`를 항상 `DRIVE`로 report하고
  reverse gear command를 차량에 적용하지 않는다. 이 프로젝트의 route와 평가도
  전진 주행만 지원하며 후진 및 주차 maneuver는 지원하거나 검증하지 않는다.
- 기본 suite는 정적 ego route와 세 날씨 및 turn을 검사한다. Bench2Drive 전체
  traffic scenario나 모든 town을 통과한다는 의미가 아니다.
- C-track과 월악산 Full 결과도 각각 한 시작점/목표점 route만 검증했다. 특히 월악산
  CARLA-to-Lanelet transform은 single-anchor empirical 정합이므로 전체 지도 임의
  route의 XY/Z 정합을 보장하지 않는다.
- 학습 분포 밖의 town, camera calibration, 날씨, 차량 dynamics에서는 성능을
  다시 측정하고 필요하면 데이터 수집과 재학습이 필요하다.

즉, `goal reached`는 지정한 한 episode에서 route 허용 오차, CTE, stall,
timeout 조건을 만족하며 정지했다는 뜻이다. 일반적인 자율주행 가능 또는 임의
환경 일반화의 증명은 아니다. 새로운 환경은 suite에 case를 추가하고 동일한
metric으로 반복 검증해야 한다.

## 실차 학습 데이터 수집과 단계적 전환

현재 AWF `carla_tiny`는 simulation-only이므로 실제 카메라를 바로 연결해 주행시키지
않는다. 이 워크스페이스에는 **실차 VAD 학습용 raw bag 수집, 정적 계약 검사, bag
검증과 calibration preview**까지 준비돼 있다. 실제 vehicle/CAN interface, 조향·제동
actuation, 차량별 actuator map, 독립 emergency stop을 연결한 실차 launch는 아직
구현하거나 검증하지 않았다. 아래 수집 명령은 차량을 제어하지 않지만, 같은 ROS domain의
다른 node가 제어 명령을 내리지 않도록 수집 때도 actuation 경로를 물리적으로 차단한다.

### 먼저 학습 범위를 선택한다

같은 bag이라도 어떤 label이 있느냐에 따라 할 수 있는 학습이 다르다. 현재 validator는
네 gate를 서로 독립적으로 보고한다.

| Gate | 필요한 내용 | 이 gate가 보장하지 않는 것 |
|---|---|---|
| `raw_capture` | calibration/rectification/commissioning provenance, 6-camera와 ego state, device/PTP 상태, 시간·TF·해상도·encoding 계약 | 학습 label 의미와 VAD 출력 품질 |
| `runtime_replay` | `raw_capture` + 1 ms camera bundle + reliable image QoS + transient-local `/tf_static` | 실시간 TensorRT 처리 속도와 폐루프 주행 |
| `planning_finetune_labels_present` | `raw_capture` + `/planning/vad_route/command` + odometry에서 만들 수 있는 3.0초 future ego window + 실제 이동 | command/전문가 trajectory의 품질과 dataset leakage |
| `full_vad_multitask_labels_present` | planning gate + `/training/ground_truth/objects` + `/training/ground_truth/vector_map` | object track/map annotation의 정확성과 upstream VAD 형식 변환 |

**Planning-only adaptation**은 여섯 영상, ego motion, maneuver command와 미래 ego
trajectory로 planning head를 먼저 맞추는 범위다. 현재 profile의 future label은 별도
topic이 아니라 `/localization/kinematic_state`를 기준으로 0.5초 간격, 3.0초 horizon을
만들 수 있는지 검사한다. 적은 annotation 비용으로 시작할 수 있지만, perception/map
head까지 실제 domain으로 바뀌었다는 뜻은 아니다.

**Full VAD multitask training**은 planning label에 시간적으로 일관된 3D object track과
vector map/lane topology annotation을 더해야 한다. 현재 ROS topic은 label 존재만
검사하므로, `TrackedObjects`와 `LaneletMapBin`을 선택한 upstream VAD 학습 형식으로
변환하고 좌표계, class, visibility, track ID와 timestamp를 별도로 검증해야 한다.
Validator PASS는 자동으로 semantic quality를 증명하지 않는다.

### 1. 실제 sensor rig 계약을 작성한다

Rig별 파일은 tracked template를 직접 덮어쓰지 않고 `data/` 아래에 둔다. `data/`는
로컬 대용량 자산용이며 Git에 포함되지 않는다.

```bash
cd /home/a/autoware_e2e

RIG=data/real_vehicle/vad_rig01
mkdir -p "${RIG}"
cp autoware_e2e_vad_launch/config/vad_real_data_collection.yaml \
  "${RIG}/collection.yaml"
cp autoware_e2e_vad_launch/config/vad_real_calibration.template.yaml \
  "${RIG}/calibration.yaml"
cp autoware_e2e_vad_launch/config/vad_real_commissioning.template.yaml \
  "${RIG}/commissioning.yaml"
cp autoware_e2e_vad_launch/config/vad_real_reprojection_report.template.yaml \
  "${RIG}/reprojection_report.yaml"
cp autoware_e2e_vad_launch/config/vad_real_device_inventory.template.yaml \
  "${RIG}/device_inventory.yaml"
```

실제 camera driver의 rectification 설정도 같은 directory에
`rectification_config.yaml`로 보존한다. `collection.yaml`의 모든
`REPLACE_WITH_*` 값을 채우고 provenance file은 profile directory 기준의 상대 경로로
지정한다.

```yaml
provenance:
  calibration_file: calibration.yaml
  rectification_config_file: rectification_config.yaml
  commissioning_file: commissioning.yaml
```

`calibration.yaml`에는 고정된 model 순서 `FRONT`, `BACK`, `FRONT_LEFT`,
`BACK_LEFT`, `FRONT_RIGHT`, `BACK_RIGHT`의 서로 다른 physical serial, 640x360
rectified `K/D`, `base_link`에서 각 optical frame까지의 측정 pose를 기록한다.
`timestamp_source`는 `hardware_exposure`, `trigger_mode`는 `hardware` 또는 `ptp`여야
한다. `D=0`은 topic의 pixel이 이미 rectification된 뒤라는 선언이지 증거가 아니므로,
왜곡 전 raw image를 이름만 `image_rect`로 발행하면 안 된다.

### 2. 수집용 ROS topic을 발행한다

Sensor/localization stack은 recorder보다 먼저 같은 `ROS_DOMAIN_ID`에서 다음 22개
필수 topic을 발행해야 한다. 기본 domain은 `42`다.

| 수량 | Topic/type | 계약 |
|---:|---|---|
| 6 | `/sensing/camera/<NAME>/image_rect`, `sensor_msgs/msg/Image` | 640x360 `bgr8`, optical frame, hardware exposure timestamp, reliable QoS |
| 6 | `/sensing/camera/<NAME>/camera_info`, `sensor_msgs/msg/CameraInfo` | image와 같은 frame/calibration, completed manifest의 `K`, rectified `D=0` |
| 6 | `/sensing/camera/<NAME>/device_info`, `diagnostic_msgs/msg/DiagnosticStatus` | `OK`, physical serial과 camera name/topic/frame, exposure timestamp source와 trigger mode 증거 |
| 1 | `/localization/kinematic_state`, `nav_msgs/msg/Odometry` | header `map`, child `base_link`, normalized pose와 vehicle velocity |
| 1 | `/localization/acceleration`, `geometry_msgs/msg/AccelWithCovarianceStamped` | header `base_link`, image와 시간 정렬 가능 |
| 1 | `/tf_static`, `tf2_msgs/msg/TFMessage` | `base_link`에서 6 optical frame까지 유일한 tree, transient-local QoS |
| 1 | `/sensing/time_sync/status`, `diagnostic_msgs/msg/DiagnosticStatus` | PTP lock와 clock source, `offset_ns`를 지속 보고 |

Camera `device_info`는 `level=OK`, `hardware_id=<physical serial>`로 발행하고 key/value에
`camera_name`, `serial`, `image_topic`, `optical_frame`,
`timestamp_source=hardware_exposure`, `trigger_mode`, `firmware`, `exposure_mode`를 넣는다.
각 image마다 SDK exposure counter와 원본 hardware stamp를 `exposure_counter`,
`exposure_stamp_ns`로 넣고 `phc_clock_source`도 기록한다. Validator는 counter가 단조
증가하고 exposure stamp 전체가 실제 image header stamp와 일치하는지, header와 bag 수신
순서가 1:1로 같은지 검사한다. 대응하는 device status와 image bag stamp 차이는 기본
50 ms, image header와 bag 수신 시각 차이는 기본 250 ms 이내여야 한다. 뒤의
firmware/exposure 값도
`collection.yaml` provenance와 정확히 같아야 한다. PTP status도
`level=OK`이고 key/value의 `locked=true`,
`clock_source`, `offset_ns`가 실제 clock 상태를 나타내야 한다. 기본 profile은 PTP
절대 offset `100000 ns` 이하, status age `1500 ms` 이하를 요구한다. 단순히 문자열만
고정 발행하는 adapter는 hardware 동기화의 증거가 아니다.

여섯 image timestamp의 dataset 수집 허용치는 20 ms지만 현재 runtime replay 허용치는
1 ms다. 실차 VAD에 그대로 재생할 데이터라면 hardware trigger/PTP로 처음부터 1 ms
bundle을 목표로 한다. Camera raw stamp를 수신 시각으로 다시 찍으면 camera 간 동기화와
ego motion alignment를 모두 잃는다.

Context, control, intervention topic은 `collection.yaml`에 이미 정의돼 있으며 존재하면
함께 기록한다. Planning adaptation에는 command `Int8` 값 `0..5`를 model branch 의미와
동일하게 발행해야 한다. Full multitask에는 object/map ground truth publisher가 추가로
필요하다.

### 3. rectification commissioning을 먼저 승인한다

Checkerboard/calibration용 bag 또는 원본 dataset으로 여섯 camera의 실제 pixel
rectification과 extrinsic을 검사한다. 다음 renderer는 첫 surround frame에
`base_link` ground grid를 투영한 3x2 PNG를 만든다.

```bash
cd /home/a/autoware_e2e
RIG=data/real_vehicle/vad_rig01

python3 scripts/e2e/render_vad_calibration_preview.py \
  /absolute/path/to/commissioning_bag \
  --profile "${RIG}/collection.yaml" \
  --calibration "${RIG}/calibration.yaml" \
  --output "${RIG}/commissioning_projection.png"

sha256sum \
  "${RIG}/calibration.yaml" \
  "${RIG}/rectification_config.yaml" \
  "${RIG}/checkerboard_evidence.tar.zst" \
  "${RIG}/reprojection_report.yaml" \
  "${RIG}/commissioning_projection.png" \
  "${RIG}/device_inventory.yaml" \
  "${RIG}/device_status_adapter.py" \
  "${RIG}/device_status_adapter.yaml"
```

PNG에서 model index와 실제 방향, 직선의 왜곡, horizon/optical axis, ground-grid 정렬을
사람이 확인한다. Checkerboard 전체 dataset에서 mean/max reprojection error를 계산하고
Camera SDK/udev 출력으로 `device_inventory.yaml`을 만들고 model 순서별 serial, firmware,
SDK device ID와 device node를 기록한다. `device_info`를 만드는 adapter 코드와 설정도
동일 directory에 보존한다. 위 여섯 evidence filename과 SHA256, calibration/rectification
SHA256, 측정 오차가 `commissioning.yaml`과 모두 일치하고 최대 1.0 pixel 이하일 때만
`approved_by`, UTC 시각, `approved=true`, `projection_preview_approved=true`를 기록한다.
Evidence의 `*_file` 값은 `commissioning.yaml` 옆에 있는 단순 filename이어야 하며 절대
경로나 상위 directory 경로는 허용하지 않는다. Recorder는 이 파일들을 session에
reflink/copy하고 다시 hash한다. `valid_until_utc`도 정해 두며 만료된 rig 승인은 새
capture를 시작하거나 session을 `validated`로 확정할 수 없다. Firmware, exposure mode,
camera 교체 또는 calibration 변경 시 만료일과 관계없이 commissioning을 다시 수행한다.
승인 유효기간은 최대 366일이고 미래 시각으로 승인할 수 없다.
자동 grid segment가 보인다는 사실만으로 승인하지 않는다. Calibration이나 rectification
설정이 바뀌면 SHA가 달라지므로 commissioning을 다시 수행해야 한다.

정적 profile, calibration, commissioning과 현재 model hash가 모두 준비됐는지 검사한다.
Placeholder가 하나라도 남거나 SHA/오차 한계가 맞지 않으면 capture-ready가 실패한다.

```bash
PROFILE=/home/a/autoware_e2e/data/real_vehicle/vad_rig01/collection.yaml

python3 scripts/e2e/vad_training_data_contract.py \
  check --capture-ready "${PROFILE}"
```

이 명령은 파일 계약 검사다. 실제 publisher의 type, 활동과 QoS는 sensor stack을 켠 뒤
다음 live preflight로 확인한다. Recorder도 기록 직전에 같은 검사를 반복한다.

```bash
export ROS_DOMAIN_ID=42
python3 scripts/e2e/vad_training_data_contract.py \
  preflight "${PROFILE}" --timeout 20
```

### 4. bag을 수집하고 gate를 확인한다

첫 수집은 actuation을 끈 상태에서 60초 정도로 pipeline만 확인한다. 출력 directory는
새 경로여야 한다. 실제 학습 수집에서는 `--duration`을 늘리거나 생략하고 Ctrl-C를 한
번 눌러 rosbag이 flush되도록 기다린다. `--skip-preflight`는 실제 dataset 수집에
사용하지 않는다.

```bash
cd /home/a/autoware_e2e
export ROS_DOMAIN_ID=42
PROFILE=/home/a/autoware_e2e/data/real_vehicle/vad_rig01/collection.yaml
SESSION="artifacts/datasets/real_vehicle/rig01_$(date +%Y%m%d_%H%M%S)"

scripts/e2e/record_vad_training_data.sh \
  --profile "${PROFILE}" \
  --duration 60 \
  "${SESSION}"
```

Recorder는 sqlite3 bag을 4 GiB 단위로 split하고 file-level zstd compression을 적용한다.
정상 종료 뒤 다음 결과가 한 session directory에 남는다.

- `bag/`: finalized rosbag과 `metadata.yaml`
- `collection_profile.yaml`, `calibration.yaml`, `rectification_config.yaml`: 수집 시점 snapshot
- `commissioning_evidence/`: 승인 manifest와 checkerboard/report/PNG/inventory/adapter snapshot
- `session_manifest.json`: software/model/calibration/bag hash와 lifecycle 상태
- `training_data_report.json`: 네 gate와 camera/state/TF/label 세부 수치
- `calibration_preview.png`: 첫 surround frame의 ground-grid 확인 화면

Manifest 상태는 `recording -> recorded_unvalidated -> validated` 순서로 진행한다.
Contract 또는 preview가 실패하면 각각 `validation_failed`, `preview_failed`로 남으므로
해당 session을 정상 학습 데이터로 채택하지 않는다.

Gate 결과는 한 번에 확인할 수 있다.

```bash
jq '.gates | map_values(.pass)' "${SESSION}/training_data_report.json"
```

학습 dataset으로 채택하기 전에는 목표 gate를 명시해 snapshot 기준으로 다시 검사한다.

```bash
python3 scripts/e2e/validate_vad_training_bag.py "${SESSION}/bag" \
  --profile "${SESSION}/collection_profile.yaml" \
  --calibration "${SESSION}/calibration.yaml" \
  --commissioning "${SESSION}/commissioning_evidence/commissioning.yaml" \
  --rectification-config "${SESSION}/rectification_config.yaml" \
  --require-gate planning_finetune_labels_present
```

Planning-only 수집이면 planning gate PASS가 필요하고, joint VAD 학습이면 multitask gate를
요구한다. Runtime bag replay에 쓸 session은 별도로 `runtime_replay`도 PASS해야 한다.
`raw_capture`만 PASS한 bag을 label이 준비된 학습 dataset으로 해석하지 않는다. 생성된
`calibration_preview.png`도 매 session 수동 확인하며, 방향/노출/가림/투영 이상이 있는
session은 gate가 PASS해도 제외한다.

640x360 `bgr8`, 6 cameras, 10 Hz의 image payload만 계산해도
`640 * 360 * 3 * 6 * 10 = 41,472,000 B/s`, 약 **39.6 MiB/s**, **139 GiB/hour**다.
CameraInfo, state, label, sqlite index overhead가 추가되고 zstd 절감률은 장면에 따라
달라진다. 장시간 수집 전 전용 NVMe 여유 공간, write throughput, thermal 상태를 실제
5~10분 bag으로 측정하고 원본과 학습 변환 dataset의 보존 정책을 정한다.

### 5. 학습부터 실제 저속 시험까지

Tesla처럼 실제 domain 성능을 반복 개선하는 것은 model 하나를 즉시 연결하는 일이
아니라 `수집 -> 품질 gate -> label/자동 label 검수 -> scenario 단위 split -> 학습 ->
open-loop 평가 -> export parity -> shadow 결과 mining -> 재수집`의 지속적인 data
pipeline이다. Train/validation/test를 인접 frame 단위로 섞지 않고 route, 날짜, 장소,
날씨 단위로 분리해 temporal leakage를 막는다.

실제 적용 순서는 다음을 고정한다.

1. 위 계약으로 정지·직진·좌/우회전, 밝기, 날씨, 속도, traffic을 포함한 실차 bag을
   **수동 운전으로만** 수집한다.
2. `hustvl/VAD` checkpoint에서 planning-only를 먼저 fine-tune하고, 필요할 때 검증된
   object/map label을 추가해 multitask training으로 확장한다.
3. PyTorch open-loop metric과 PyTorch/ONNX/TensorRT parity를 통과한 model만 기존 CARLA
   직진/좌/우회전 회귀에 넣는다. Simulation-only 결과를 실차 성능으로 간주하지 않는다.
4. 실제 bag replay와 vehicle에서 **shadow mode**로 VAD trajectory를 기록하되 control
   mux, CAN, steering/brake에는 연결하지 않는다. Expert trajectory, intervention,
   off-route, TTC와 latency를 비교해 실패 장면을 다시 수집한다.
5. 실차 전용 sensor/vehicle launch, calibrated actuator map, localization, MPC/PID,
   collision monitor, minimum-risk stop, 물리 emergency stop과 수동 override가 독립 safety
   review를 통과한 뒤에만 저속 폐쇄코스에서 actuation을 연결한다.
6. 저속 직진부터 회전, 정지, 제한된 obstacle 순으로 ODD를 한 단계씩 넓히고, 각 단계는
   독립 driver와 즉시 해제 가능한 제어권을 둔다.

CARLA launch와 실차 launch는 분리한다. 공통으로 재사용할 수 있는 것은 검증된
VAD tensor/interface와 route-command 의미, dataset/evaluation 계약이다. Camera driver,
TF, localization, vehicle/CAN interface, actuator map과 safety layer는 실제 차량 전용이다.
CARLA에서 채택한 MPC `input_delay/tau`, accel/brake/steer map과 정지 거리를 실차에
복사하지 말고 실제 차량 step-response와 delay 측정부터 다시 식별한다.

## 종료 시 알려진 메시지

Ubuntu 22.04/ROS 2 Humble에서 upstream `image_transport republish`가 종료 중
exit code `-11`을 출력할 수 있다. 독립 republisher에서도 재현된 teardown 시점
문제로, runtime node가 이미 정지했고 evaluator 결과와 disengage가 정상이라면
완료된 주행 결과 자체를 무효화하지 않는다. 다음 episode 전에 bridge/VAD
process가 실제로 모두 종료됐는지는 반드시 확인해야 한다.

유용한 topic 점검 명령:

```bash
source scripts/e2e/env.sh
ros2 topic hz /sensing/camera/CAM_FRONT/image_raw/compressed
ros2 topic hz /localization/kinematic_state
ros2 topic hz /planning/vad/candidate_trajectories
ros2 topic hz /planning/vad_route/selected_raw_trajectory
ros2 topic hz /planning/trajectory
ros2 topic echo /planning/vad_route/status --once
ros2 topic echo /planning/vad_route/remaining_distance --once
ros2 topic echo /planning/vad_route/standard_route_aligned --once  # full only
```

VAD는 학습 순서대로 여섯 카메라를 모두 요구한다. Upstream의 한 카메라짜리
lightweight mapping으로 바꾸면 안 되며, fast wrapper가 지정하는 project의
6-camera mapping을 사용해야 한다. Minimal launch의 `map_path`는
CARLA town을 선택하며 PCD/lanelet map stack을 시작하지 않는다. Full launch는
CARLA world용 `carla_map`과 완전한 Autoware map directory용 `map_path`를 별도로
사용한다.
