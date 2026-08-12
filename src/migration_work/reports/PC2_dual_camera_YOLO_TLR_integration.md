# PC2 듀얼 Lucid 카메라·YOLOX·TLR `run_autoware` 통합 기록

기준일: 2026-08-12 KST
대상: PC2 — sensing(camera), perception, GPU processing
운용 환경: ROS 2 Humble, `ROS_DOMAIN_ID=10`, `rmw_cyclonedds_cpp`,
`ROS_LOCALHOST_ONLY=0`

## 1. 결론

이번 작업으로 PC2의 두 Lucid 카메라를 서로 다른 물리 NIC와 serial에 고정하고, 두 드라이버를
`run_autoware` 한 번으로 함께 시작하도록 통합했다.

현재 결과는 다음처럼 구분해야 한다.

| 검증 항목 | 결과 | 의미 |
|---|---|---|
| 두 카메라의 IP/NIC 경로 | **PASS** | 각 카메라가 전용 NIC의 올바른 source IP로 통신한다. |
| 두 Lucid 장치 동시 open/stream | **PASS** | 서로 다른 두 component process에서 두 장치가 동시에 스트리밍했다. |
| Windshield `camera0` raw image | **PASS** | 2880x1860 BGR8 영상과 CameraInfo가 발행되었다. |
| Windshield generic YOLOX 2D ROI | **PASS** | 10개 메시지, 14.725 Hz, 메시지당 검출 1개, 잘못된 ROI 0개를 확인했다. |
| Loop Top `camera1` TLR 입력 영상 | **PASS** | 1920x1200 JPEG → TLR decompressor → raw 경로가 연결되었다. |
| TLR 노드/엔진 기동 | **PASS** | camera1의 fine detector/classifier 계열 컴포넌트와 TensorRT 엔진이 기동했다. |
| 최종 traffic-signal 의미 payload | **BLOCKED** | 해당 실행에서 PC3 map/TF/LiDAR 기반 입력이 없어서 최종 신호 의미 메시지는 나오지 않았다. |
| Windshield camera-LiDAR 3D fusion | **의도적으로 OFF** | serial 222의 intrinsic/extrinsic이 아직 검증되지 않아 잘못된 3D 결과 생성을 막았다. |
| RViz의 `run_autoware` 종속 실행 | **PASS** | 별도 수동 실행이 아니라 동일 launch의 child process로 기동/종료했다. |
| PC2 no-actuation 경계 | **PASS(로컬 launch 범위)** | vehicle/planning/control/API/vehicle-interface를 모두 끈 상태로 시험했다. |
| 정상 종료와 잔류 프로세스 | **PASS** | SIGINT 후 두 카메라가 StopStream/DestroyDevice/CloseSystem을 수행했고 launch group이 정리됐다. |

따라서 현재 PC2는 **두 카메라 전송과 Windshield 2D YOLOX까지 동작**한다. 그러나 이를
“TLR 의미 인식 완료” 또는 “camera-LiDAR 3D fusion 완료”라고 기록하면 안 된다. 두 기능의
최종 판정에는 PC3 map/localization/TF/LiDAR와 실제 카메라 보정 검증이 추가로 필요하다.

## 2. 확정한 물리 NIC와 카메라 매핑

### 2.1 PC2 포트 매핑

| 역할 | PC2 인터페이스 | PC2 주소 | 상대 장치 | 상대 주소 | 비고 |
|---|---|---|---|---|---|
| PC1/PC3 DDS + 현재 유선 인터넷 | `enp0s31f6` | DHCP `192.168.9.11/24` | PC1 / PC3 | `.2` / `.7` | CycloneDDS를 이 NIC에 고정 |
| 저장된 차량망 전용 대안 | `enp0s31f6` | 고정 `192.168.9.110/24` | PC1 / PC3 | `.2` / `.7` | `ROS2` profile, 현재 비활성 |
| Windshield Lucid | `enp1s0f0` | `192.168.41.1/24` | serial `222301529` | `192.168.41.2/24` | TRI054S-C, MTU 9000 |
| Loop Top Lucid | `enp1s0f3` | `169.254.0.1/24` | serial `214000332` | `169.254.0.11` | TRI023S-C, MTU 9000 |

카메라 MAC까지 포함한 발견 결과는 다음과 같다.

| 위치 | serial | model | camera MAC | camera IP | PC2 NIC |
|---|---:|---|---|---|---|
| Windshield | `222301529` | `TRI054S-C` | `1C:0F:AF:03:49:78` | `192.168.41.2` | `enp1s0f0` |
| Loop Top | `214000332` | `TRI023S-C` | `1C:0F:AF:01:0C:A5` | `169.254.0.11` | `enp1s0f3` |

`run_autoware` 도우미는 현재 DHCP 주소 `.11`만 강제하지 않는다. 활성 프로필이 `Ethernet`
또는 `ROS2` 중 하나이고, `enp0s31f6`의 실제 source 주소가 `192.168.9.0/24`에 있으며 PC1/PC3
경로도 같은 NIC를 사용할 때만 시작한다. 그러므로 유선 인터넷 DHCP 모드와 Wi-Fi 인터넷 +
고정 차량망 모드를 모두 지원하되 DDS NIC 자체는 바꾸지 않는다.

인터넷/Wi-Fi/차량망/카메라망 전환 절차와 롤백 명령은
[PC2_network_and_port_setup.md](../runbooks/PC2_network_and_port_setup.md)에 별도로 기록했다.

### 2.2 Windshield NetworkManager 변경

Windshield 카메라는 `192.168.41.2/24`인데 기존 PC2 profile은
`169.254.121.1/24`였다. 동일 subnet이 아니어서 장치 discovery만으로 실제 stream 경로를
보장할 수 없었다. 카메라의 persistent IP는 건드리지 않고 PC2의 해당 profile만 다음처럼
수정했다.

| 항목 | 변경 전 | 변경 후 |
|---|---|---|
| host address | `169.254.121.1/24` | `192.168.41.1/24` |
| camera address | `192.168.41.2/24` | 변경 없음 |
| gateway / DNS | 없음 | 없음 |
| `ipv4.never-default` | `no` | `yes` |
| MTU | 9000 | 9000 |
| profile UUID | `9100063a-de5a-4a02-adbc-ef458e474c13` | 동일 |

이 변경으로 `192.168.41.2` 경로가 `dev enp1s0f0 src 192.168.41.1`로 결정된다. 카메라
전용 NIC가 인터넷 기본 경로를 가져가지 않도록 gateway/DNS를 비우고 `never-default=yes`를
적용했다.

Loop Top은 기존 전용 profile을 유지한다. 실행 도우미는 serial 214 장치에 대해
`169.254.0.11/32` runtime route를 정확히 추가하고, 종료할 때 그 route만 제거한다. 저장된
NetworkManager route는 변경하지 않는다.

## 3. 왜 카메라 component를 두 프로세스로 나눴는가

기존 `ArenaCameraNode`는 component가 생성될 때마다
`arena_cameras_handler.cpp`에서 `Arena::OpenSystem()`을 호출한다. Arena SDK 예제도 한 번에
하나의 system만 열 수 있다고 명시한다.

두 카메라 component를 하나의 `component_container`에 넣으면 같은 프로세스 안에서
`OpenSystem()`이 두 번 호출되는 구조가 된다. 이를 피하기 위해 다음 구조를 선택했다.

```text
run_autoware
  └─ dual_camera.launch.py
      ├─ process 1: windshield_camera_container
      │    └─ arena_camera_node_windshield (serial 222301529)
      └─ process 2: loop_top_camera_container
           └─ arena_camera_node_loop_top (serial 214000332)
```

즉, container 이름만 두 개로 만든 것이 아니라 **OS process 자체를 둘로 분리**했다. 각
프로세스가 자체 Arena system과 장치 하나를 소유하므로 현재 driver 구조를 크게 재작성하지
않고 두 장치를 독립적으로 open, stream, shutdown할 수 있다.

관련 근거:

- `/home/a/ros2_ws/src/lucid_vision_driver/src/arena_cameras_handler.cpp`
- `/home/a/ArenaSDK/ArenaSDK_Linux_x64/Examples/ArenaC/C_Enumeration/C_Enumeration.c`
- `/home/a/ros2_ws/src/lucid_vision_driver/launch/dual_camera.launch.py`

## 4. 최종 토픽 아키텍처

### 4.1 Windshield — camera0, generic 2D YOLOX

```text
Lucid serial 222301529 / TRI054S-C / 192.168.41.2
  frame: camera0/camera_optical_link
  native raw:
    /lucid_vision/windshield/image
        │ topic_tools relay, best_effort
        ▼
    /sensing/camera/camera0/image_raw
        │
        ▼
    /tensorrt_yolox
        │ generic 8-class DetectedObjectsWithFeature
        ▼
    /perception/object_recognition/detection/rois0
        │
        └─ /topic_state_monitor_windshield_yolox_rois

  native CameraInfo:
    /lucid_vision/windshield/camera_info
        │ topic_tools relay, best_effort
        ▼
    /sensing/camera/camera0/camera_info
```

Windshield driver에서는 `enable_compressing=false`로 두었다. YOLOX가 raw image를 직접
사용하므로 사용하지 않는 2880x1860 JPEG를 매 프레임 생성할 이유가 없기 때문이다.

YOLOX는 lazy subscription 동작을 하므로 출력 subscriber가 없으면 image 처리를 멈출 수
있다. 그래서 `topic_state_monitor`를 `rois0`의 영구 subscriber로 넣었다. 이 monitor는 ROI
rate/timeout을 진단하지만 2D ROI를 3D object로 변환하지 않는다.

현재 generic YOLOX label은 기존 8개 클래스
(`UNKNOWN`, `CAR`, `TRUCK`, `BUS`, `BICYCLE`, `MOTORBIKE`, `PEDESTRIAN`, `ANIMAL`)를
그대로 사용한다. Windshield의 목적은 전방 사람/차량 계열 2D 검출이며, label 파일을 임의로
사람 하나만 남기도록 축소하지 않았다.

### 4.2 Loop Top — camera1, traffic-light recognition

```text
Lucid serial 214000332 / TRI023S-C / 169.254.0.11
  frame: traffic_light_camera/camera_optical_link
  native compressed:
    /lucid_vision/camera/image_compressed
        │ topic_tools relay, best_effort
        ▼
    /sensing/camera/camera1/traffic_light/image_raw/compressed
        │ Autoware traffic_light_image_decompressor
        ▼
    /sensing/camera/camera1/traffic_light/image_raw
        │
        ├─ traffic-light fine detector
        ├─ car/pedestrian traffic-light classifiers
        └─ ROI visualizer / arbiter chain

  native CameraInfo:
    /lucid_vision/camera/camera_info
        │ topic_tools relay, best_effort
        ▼
    /sensing/camera/camera1/traffic_light/camera_info
```

Loop Top은 기존 1920x1200 JPEG 전송 경로를 유지한다. normalized `image_raw` publisher는
driver raw relay가 아니라 TLR decompressor 하나뿐이므로 중복 raw publisher를 만들지 않는다.

### 4.3 두 카메라 역할 분리

- `camera0` / Windshield: generic 2D YOLOX 전용.
- `camera1` / Loop Top: traffic-light recognition 전용.
- `image_number=1`, fusion slot 0의 예약 토픽은 camera0에 맞췄다.
- 그러나 실제 `perception_mode=lidar`로 실행하여 camera-LiDAR fusion component는 0개다.
- Loop Top image를 generic YOLOX에 다시 넣지 않으므로 같은 영상의 중복 GPU inference를
  피했다.

## 5. frame과 TF 수정

이미지 좌표는 일반 `camera_link`가 아니라 REP-103 optical axis를 따르는
`camera_optical_link`를 사용해야 한다. 두 driver header frame을 다음처럼 수정했다.

| 카메라 | 이전/문제 상태 | 현재 image/CameraInfo frame |
|---|---|---|
| Windshield | camera0 link가 launch/URDF에서 비활성 또는 불명확 | `camera0/camera_optical_link` |
| Loop Top | `traffic_light_camera/camera_link` | `traffic_light_camera/camera_optical_link` |

`sample_sensor_kit_description/urdf/sensor_kit.xacro`에서 camera0 macro도 활성화하여 다음 TF
chain을 기술했다.

```text
sensor_kit_base_link
  ├─ traffic_light_camera/camera_link
  │    └─ traffic_light_camera/camera_optical_link
  └─ camera0/camera_link
       └─ camera0/camera_optical_link
```

여기서 주의할 점은 **TF 이름과 optical rotation을 연결한 것**과 **실제 차량 장착 위치를
정밀 보정한 것**은 다르다는 것이다. camera0의 실제 장착 extrinsic은 아직 계측/검증하지
않았다. PC3가 최종 robot description/TF owner이므로 다음 통합 시험에서 PC3가 위 chain을
단일 broadcaster로 발행하는지 확인해야 한다.

## 6. 보정값의 현재 상태

### 6.1 Windshield serial 222301529

`windshield_222301529_transport_only.yaml`은 live stream에서 확인한 2880x1860 크기에 맞춘
**임시 transport/2D 시험용 CameraInfo**다. 파일명과 주석에서도 이를 명시했다.

이 값에 대해 다음을 주장하지 않는다.

- serial 222 렌즈의 정식 checkerboard intrinsic calibration 완료;
- 왜곡 보정 정확도 검증 완료;
- 차량 장착 extrinsic calibration 완료;
- LiDAR-camera projection 또는 metric evaluation 사용 가능.

YOLOX 2D detector는 image 자체로 검출할 수 있으므로 이번 범위에서는 transport와 2D ROI를
시험했다. 실제 camera-LiDAR fusion은 intrinsic/extrinsic 보정 전까지 꺼 둔다.

### 6.2 Loop Top serial 214000332

기존 `loop_top_214000332_1920x1200.yaml`을 사용했다. 이 파일은 2025-04-23에 생성된 로컬
결과이고 이전 stationary rectification 시험 이력은 있으나, 카메라를 재장착했거나 렌즈가
변했다면 다시 보정해야 한다. 이번 듀얼 카메라 시험은 이를 새 정식 calibration으로
재인증한 시험이 아니다.

## 7. 변경 파일과 변경 목적

### 7.1 `/home/a/ros2_ws/src/lucid_vision_driver`

| 파일 | 변경 내용 |
|---|---|
| `launch/dual_camera.launch.py` | 두 camera parameter를 읽고, 물리 카메라당 별도 component process 하나를 생성 |
| `param/windshield_cam.yaml` | camera name `windshield`, serial 222, optical frame, 15 Hz, raw-only, auto exposure/gain 지정 |
| `param/loop_top_cam.yaml` | serial 214를 유지하고 frame을 traffic-light optical frame으로 변경 |
| `config/windshield_222301529_transport_only.yaml` | live 2880x1860 transport 크기와 provisional CameraInfo 명시 |
| `src/arena_camera_node.cpp` | raw image publish에서 message를 move한 뒤 그 header를 다시 읽지 않도록 stable local header를 CameraInfo에 사용 |
| `src/arena_cameras_handler.cpp` | exposure/gain auto mode일 때 무효인 manual value setter를 호출하지 않도록 guard |

`arena_camera_node.cpp` 수정은 CameraInfo header가 image와 같은 stamp/frame을 안정적으로 갖게
한다. 이전 순서는 `img_msg`가 publisher로 move된 뒤 그 header를 참조할 여지가 있었다.

auto exposure/gain을 사용하는 Windshield에서는 manual target을 다시 적용하려는 경고가 실제
동작 실패처럼 보였다. auto mode에서는 manual setter가 효력이 없으므로 해당 mode가 꺼진
경우에만 setter를 호출한다. Loop Top의 manual exposure/gain 경로는 유지된다.

### 7.2 `/home/a/autoware/src`

| 파일 | 변경 내용 |
|---|---|
| `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/camera.launch.xml` | dual driver include, camera0 raw/info relay, camera1 compressed/info relay 구성 |
| `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml` | PC2 소유 두 카메라 launch를 sensing module에 포함 |
| `sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/urdf/sensor_kit.xacro` | camera0 link/optical-link macro 활성화 |
| `launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml` | fusion slot 0의 image/info/rois contract를 camera0으로 정렬 |
| `launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | sensing/perception/YOLOX/TLR/RViz를 한 `run_autoware` launch에 연결하고 fusion은 off |
| `migration_work/scripts/run_pc2_autoware.sh` | 동적 DDS source 검증, 두 카메라 NIC/route/ping 검증, 중복 publisher guard, 정확한 cleanup 추가 |
| `migration_work/scripts/probe_yolox_contract.py` | camera0 2880x1860 2D ROI의 stamp/frame/rate/ROI 범위/endpoint contract 검증 |
| `migration_work/scripts/validate_pc2_cycles.sh` | 두 카메라·YOLOX·TLR endpoint, payload, 안전 경계, 종료 잔류를 cycle 단위로 검증 |
| `migration_work/runbooks/PC2_network_and_port_setup.md` | 유선 인터넷/Wi-Fi/차량 DDS/두 카메라 NIC 전환과 복구 절차 기록 |

## 8. 빌드와 정적 검증

Lucid package는 다음 로그의 `colcon build`로 다시 빌드했다.

- `/home/a/ros2_ws/log/build_2026-08-12_14-41-38/`
- `/home/a/ros2_ws/log/build_2026-08-12_14-41-38/lucid_vision_driver/stdout_stderr.log`

빌드 결과는 성공이며 `install/lucid_vision_driver`가 갱신됐다. Python launch compile,
launch argument expansion, XML parse와 sample sensor-kit URDF 생성도 확인했다. 검사 산출물은
다음과 같다.

- `/tmp/dualcam_show_args.txt`
- `/tmp/sample_sensor_kit_dual.urdf`

`/tmp` 파일은 일시 검증 산출물이므로 commit 대상이 아니다.

## 9. standalone 듀얼 카메라 동시 스트림 시험

### 9.1 실행 근거

두 카메라만 격리하여 `dual_camera.launch.py`를 실행했다.

ROS launch 원본 로그:

- `/home/a/.ros/log/2026-08-12-14-31-36-826415-a-10818/launch.log`

로그에는 다음이 함께 기록되어 있다.

- `windshield_camera_container`와 `loop_top_camera_container` 두 process 시작;
- 두 process 모두 serial 222와 214를 discovery;
- 각 process가 지정 serial에 대해 `CreateDevice done`;
- 두 process 모두 `StartStream done`;
- Windshield frame `camera0/camera_optical_link`;
- Loop Top frame `traffic_light_camera/camera_optical_link`;
- Ctrl+C 후 양쪽 모두 StopStream → DeregisterImageCallback → DestroyDevice → CloseSystem;
- 두 container 모두 `process has finished cleanly`.

### 9.2 bounded payload 측정

CLI graph를 반복 조회하지 않고 지속 subscriber로 측정한 standalone 결과가 실제 카메라
전송률의 기준이다.

| 카메라 | image 크기 | image rate | CameraInfo rate | stamp/frame |
|---|---:|---:|---:|---|
| Windshield serial 222 | 2880x1860 | **14.891 Hz** | 약 **14.98 Hz** | image/info 공통 stamp와 optical frame 확인 |
| Loop Top serial 214 | 1920x1200 | **14.885 Hz** | 약 **14.98 Hz** | image/info 공통 stamp와 optical frame 확인 |

두 장치를 동시에 15 Hz 설정으로 운용해도 어느 한쪽이 device-open 단계에서 탈락하지 않았고,
종료 후 카메라 ownership도 정상 해제됐다.

## 10. 전체 `run_autoware` 통합 시험

### 10.1 실행 근거

전체 PC2 launch 원본 로그:

- `/home/a/.ros/log/2026-08-12-14-33-21-804273-a-11952/launch.log`

이 실행에서는 `ros2 launch autoware_launch autoware.launch.xml`의 같은 process group 아래에
다음이 함께 시작됐다.

- Windshield/Loop Top camera container 2개;
- camera0/camera1 relay;
- PC2 perception pointcloud container;
- traffic-light recognition camera1 component container와 arbiter;
- generic `autoware_tensorrt_yolox_node_exe`;
- YOLOX ROI `topic_state_monitor`;
- `rviz2`.

RViz는 별도 터미널에서 수동 실행한 것이 아니다. launch log의 `rviz2-27`이 같은 launch의
child로 시작됐고, SIGINT 후 `process has finished cleanly`로 종료됐다.

### 10.2 endpoint cardinality

bounded full-run probe에서 확인한 endpoint 수는 다음과 같다. `publisher/subscription`은 그
시점의 전체 Domain 10 graph cardinality다.

| 경로 | publisher/subscription | 판정 |
|---|---:|---|
| Wind native `/lucid_vision/windshield/image` | `1/1` | driver → camera0 relay 단일 경로 |
| Wind native `/lucid_vision/windshield/camera_info` | `1/1` | driver → camera0 info relay 단일 경로 |
| Wind normalized `/sensing/camera/camera0/image_raw` | `1/1` | relay → YOLOX 단일 경로 |
| Wind normalized `/sensing/camera/camera0/camera_info` | `1/0` | 발행 확인, 3D fusion은 off |
| YOLOX `/perception/object_recognition/detection/rois0` | `1/1` | YOLOX → permanent topic monitor |
| Loop native `/lucid_vision/camera/image_compressed` | `1/1` | driver → TL compressed relay |
| Loop normalized `/sensing/camera/camera1/traffic_light/image_raw` | `1/2` | decompressor 단일 publisher, TLR consumers 2 |
| Loop normalized `/sensing/camera/camera1/traffic_light/camera_info` | `1/4` | info relay 단일 publisher, TLR consumers 4 |
| final `/perception/traffic_light_recognition/traffic_signals` | `2/2` | endpoint 존재, 이 시험에서 payload 0 |

카메라 native/normalized topic마다 publisher가 하나이므로 동일 serial을 manual launch와 sensor
launch가 중복 open한 흔적은 없다.

반면 최종 `traffic_signals`의 publisher가 2개였으므로 이 토픽은 아직 single-owner PASS가
아니다. Autoware 내부 relay/arbiter 경로와 원격 PC endpoint를 node FQN/GID로 귀속한 뒤 의도된
publisher 하나만 남는지 확인해야 한다. payload가 0이라는 사실도 publisher 중복 위험을
없애지는 않는다.

### 10.3 Windshield camera/YOLOX payload

전체 launch 중 camera payload probe는 다음을 확인했다.

| 경로 | image/info sample | 공통 stamp | 측정 rate |
|---|---:|---:|---:|
| Windshield camera0 | image 10 / info 26 | 9 | image 5.175 Hz / info 13.047 Hz |
| Loop Top camera1 | image 10 / info 15 | 10 | image 9.797 Hz / info 14.506 Hz |

이 짧은 측정의 image rate는 고해상도 image를 ROS CLI/검사 프로세스가 deserialize하는 부하와
동시에 전체 perception/TensorRT가 기동하는 영향을 포함한다. 카메라 자체 전송률의 권위 있는
기준은 앞 절의 덜 침습적인 standalone 지속 subscriber 결과인 약 14.9 Hz다. 전체 실행의
수치는 “payload가 실제로 도착했다”는 contract 증거로 사용하고 카메라 FPS 성능 한계로
해석하지 않는다.

YOLOX 전용 probe 결과:

```text
result=PASS
samples=10
rate_hz=14.725
frames=['camera0/camera_optical_link']
total_objects=10
objects_per_message=1..1
invalid_rois=0
graph=PASS
```

즉, 이 장면과 측정 구간에서는 매 ROI 메시지에 검출 하나가 있었고 모든 ROI가 2880x1860
image 범위 안에 있었다. 이것은 **관측된 샘플의 2D contract PASS**이며, 모든 조명/거리에서
사람을 정확히 분류한다는 정확도 평가나 ground-truth mAP 평가가 아니다.

### 10.4 TLR의 현재 경계

camera1의 compressed image, decompressed raw image, CameraInfo, TLR 노드 및 TensorRT 엔진
기동까지는 확인했다. 그러나 이 실행에서 PC3가 제공해야 하는 다음 기반 데이터가 Domain 10
graph에 없었다.

- `/map/vector_map`과 traffic-light regulatory element;
- 유효한 차량 localization;
- `map`에서 sensor/camera까지 이어지는 TF;
- LiDAR 및 필요한 동기화 입력.

그 결과 map-based traffic-light ROI를 유효하게 생성할 수 없었고, 최종
`/perception/traffic_light_recognition/traffic_signals`는 publisher/subscriber endpoint만
있고 payload는 0이었다. 따라서 상태는 다음과 같다.

```text
Loop camera transport        PASS
TLR process/model loading    PASS
TLR final semantic payload   BLOCKED — PC3 map/localization/TF prerequisite absent
```

신호등이 화면에 보인다는 사실만으로 이 blocker가 해제되지는 않는다. 다음 세 PC 통합 시험에서
map ROI, optical TF, image stamp, classifier output, final traffic signal을 같은 시간 창에서
연속 측정해야 한다.

### 10.5 안정화 후 최종 guarded cycle

스크립트 수정이 모두 끝난 뒤 다음 명령으로 별도 1회를 다시 실행했다.

```bash
cd /home/a/autoware/src
REQUIRE_PC3_INPUTS=0 ./migration_work/scripts/validate_pc2_cycles.sh 1
```

최종 결과는 `cycle=1 result=PASS`였다. 주요 실측값은 다음과 같다.

| 항목 | 최종 결과 |
|---|---|
| DDS 물리 경로 | `enp0s31f6`, source `192.168.9.11/24`, PC1 `.2`/PC3 `.7` route PASS |
| launch 구성 | process 28, PC2 node 53, 중복 PC2 node 없음 |
| Lucid 구성 | camera component 2/2, container 2/2, 각 native/normalized publisher 1 |
| Loop Top contract | 1920x1200 BGR8, optical frame, image 13.471 Hz, CameraInfo 14.968 Hz, PASS |
| Windshield contract | 2880x1860 BGR8, optical frame, payload/공통 stamp PASS |
| Windshield YOLOX | 10 messages, 11.794 Hz, 1 object/message, invalid ROI 0, graph PASS |
| TLR 초기화 | fine/car/pedestrian/ROI visualizer 4/4, PASS |
| camera-LiDAR fusion | component 0/0, 의도한 비활성 상태 |
| PC2 안전 경계 | control/vehicle process 0, 신규 safety publisher 0, `control_cmd` publisher 0 |
| 종료 | process group/files/graph/Loop route cleanup 모두 PASS, 1.563 s |
| 종료 오류 | TensorRT teardown/process-died/camera-memory/relay-shutdown 오류 모두 0 |

Windshield raw의 짧은 probe 수신률은 5.516 Hz였지만 CameraInfo는 12.937 Hz이고 같은 실행의
YOLO 출력이 11.794 Hz였다. 15 Hz에 가까운 실제 장치 전송은 9.2절의 standalone 지속 측정으로
확인했다. 따라서 이 짧은 raw probe 값은 2880x1860 메시지를 별도 Python subscriber가
deserialize하는 부하로 해석하며 카메라 FPS 판정값으로 사용하지 않는다.

첫 안정화 시도는 readiness 제한 90초 직후 마지막 TLR ROI visualizer가 로드되어
`traffic_components=3/4`로 TIMEOUT 처리됐다. 로그상 모든 네 컴포넌트는 정상 로드됐고
카메라/YOLO/종료 계약도 통과했으므로, cold TensorRT 순차 초기화 실측에 맞춰 readiness의
상한만 150초로 늘렸다. 두 번째 실행은 동일한 기능 계약으로 4/4 readiness와 최종 cleanup까지
PASS했다. 이는 기능을 우회한 것이 아니라 성공한 cold start를 90초에서 잘못 TIMEOUT으로
분류하던 검증기 문제를 수정한 것이다.

최종 로그:

```text
/home/a/autoware/src/migration_work/test_logs/live_camera_cycles/20260812_145512/cycle_1.log
```

이 최종 cycle에서도 PC3 입력 publisher는 0이었다. `REQUIRE_PC3_INPUTS=0`은 PC2 로컬
카메라·YOLO·TLR 기동과 종료를 검증하기 위한 모드이며, TLR semantic/3-PC PASS를 뜻하지 않는다.

## 11. 3D fusion을 꺼 둔 이유

`tier4_perception_component.launch.xml`의 slot 0 토픽은 향후 camera0 fusion을 위해 정렬해
두었지만, 현재 top launch의 `perception_mode`는 `lidar`다. full run에서 camera fusion
component 수가 0인 것이 기대값이다.

다음 조건 없이 Windshield ROI를 공식 3D object 경로에 연결하면 영상 위치가 LiDAR 공간의
잘못된 위치로 투영되어 PC1 planning 입력을 오염시킬 수 있다.

1. serial 222의 정식 2880x1860 intrinsic/distortion calibration;
2. 실제 차량 장착 상태의 camera0-to-sensor-kit extrinsic;
3. PC3가 발행하는 유일하고 연속적인 TF chain;
4. 카메라/LiDAR clock 및 stamp 동기 검증;
5. 실제 target으로 projection overlay와 3D association 평가.

이 조건을 통과한 뒤에만 `perception_mode:=camera_lidar_fusion`을 명시적으로 활성화한다. 현재
2D YOLOX 출력은 진단/시각화/데이터 수집용이며 `/perception/object_recognition/objects`에
camera 3D 결과로 합쳐지지 않는다.

## 12. 안전 경계

이번 PC2 top launch의 기본값과 실제 실행 인수는 다음 모듈을 끈다.

- `launch_vehicle=false`;
- `launch_system=false`;
- `launch_map=false`;
- `launch_localization=false`;
- `launch_planning=false`;
- `launch_control=false`;
- `launch_api=false`;
- `launch_vehicle_interface=false`;
- `enable_all_modules_auto_mode=false`.

PC2에는 camera/sensing과 perception만 남기고 planning/control/vehicle command ownership을
두지 않았다. 이 결과는 이번 PC2 launch group의 로컬 no-actuation 경계이며, PC1/PC3/PC4의
전체 CAN writer 부재를 대신 증명하지는 않는다.

## 13. 종료와 잔류 상태

standalone 및 full `run_autoware` 모두 Ctrl+C/SIGINT 정상 종료를 사용했다. 두 Lucid
process에서 다음 순서를 확인했다.

```text
rclcpp pre-shutdown callback
  -> callback gate
  -> StopStream
  -> DeregisterImageCallback
  -> DestroyDevice
  -> CloseSystem
  -> component_container finished cleanly
```

full-run 로그에서도 RViz, YOLOX, TLR container, relay, camera container가 모두
`process has finished cleanly`로 끝났다. 종료 직후 점검에서 다음 잔류가 없었다.

- `autoware.launch.xml` process group;
- Lucid component container 2개;
- YOLOX/TLR/RViz process;
- PC2-owned ROS application node/topic publisher;
- `/tmp/autoware_pc2_run.pid`와 `/tmp/autoware_pc2_run.lock`;
- helper가 소유했던 Loop Top `/32` runtime route.

다른 PC가 살아 있을 때 Domain 10에 남는 원격 node는 PC2 잔류 process로 세지 않아야 한다.
cleanup 판정은 process group, PC2 node FQN, PC2-owned publisher, guard file과 owned route를
함께 사용한다.

## 14. 알려진 제한과 다음 acceptance gate

### Gate A — 재부팅/네트워크 모드 검증

- 재부팅 후 `enp0s31f6`가 DHCP `Ethernet`인지 고정 `.110`의 `ROS2` profile인지 기록한다.
- Wi-Fi 인터넷 모드에서도 PC1/PC3 DDS 유선은 `enp0s31f6`에 유지한다.
- 세 PC 모두 Domain 10, CycloneDDS, `ROS_LOCALHOST_ONLY=0`인지 비교한다.
- PC1 `.2`, PC3 `.7`에 대해 route/source, ping, DDS multicast, 지속 topic graph를 확인한다.

### Gate B — PC3 sensor foundation

- GNSS/IMU/LiDAR payload의 rate, stamp, frame, field, QoS를 확인한다.
- map pointcloud/vector map payload를 확인한다.
- `map -> base_link -> sensor_kit_base_link -> 각 camera optical frame`이 연결되는지 확인한다.
- PC3가 camera TF를 단일 소유하고 PC2가 중복 TF를 발행하지 않는지 확인한다.

### Gate C — TLR semantic PASS

- 실제 C_track map의 traffic-light regulatory element와 현장 신호등 위치를 대조한다.
- camera1 map ROI가 image 안에 유효하게 투영되는지 RViz/debug image로 확인한다.
- fine detector ROI, car/pedestrian classifier, arbiter 입력/출력을 같은 stamp로 측정한다.
- 최종 `traffic_signals`가 0이 아닌 rate로 발행되고 실제 신호 색/형태와 일치하는지 기록한다.

### Gate D — Windshield calibration/fusion

- serial 222 전용 2880x1860 checkerboard intrinsic calibration을 새로 수행한다.
- 장착 상태에서 camera0-LiDAR extrinsic을 구한다.
- projection overlay와 timestamp alignment를 검증한다.
- 통과 전에는 `perception_mode=lidar`를 유지한다.
- 통과 후에만 camera-LiDAR fusion을 켜고 공식 object topic의 source ownership/정확도를 다시
  검증한다.

### Gate E — 3-PC/4-PC end-to-end

- PC2 `rois0`, TLR 최종 payload와 `/perception/object_recognition/objects`를 PC1의 실제
  subscriber callback에서 측정한다.
- PC1 planning/control 결과가 있어도 stationary 단계에서는 물리 actuation 경로가 완전히
  차단되어 있는지 별도 증명한다.
- PC4 bridge/source가 PC2와 같은 공식 perception topic을 중복 publish하지 않도록 한다.
- 논문 데이터 수집 전 세/네 PC clock offset, rosbag topic contract, dropped frame, GPU/CPU
  load를 함께 기록한다.

## 15. 재현용 최소 확인 명령

네트워크 profile을 임의로 바꾸지 않은 상태에서 새 터미널로 확인한다.

```bash
nmcli connection show --active
ip -br -4 address
ip -4 route get 192.168.9.2
ip -4 route get 192.168.9.7
ip -4 route get 192.168.41.2
ip -4 route get 169.254.0.11

printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
run_autoware
```

실행 후 별도 터미널에서는 단순 `topic list`만 보지 말고 다음 contract를 확인한다.

```bash
ros2 topic info /sensing/camera/camera0/image_raw --verbose
ros2 topic info /perception/object_recognition/detection/rois0 --verbose
ros2 topic info /sensing/camera/camera1/traffic_light/image_raw --verbose
ros2 topic info /perception/traffic_light_recognition/traffic_signals --verbose
```

정량 PC2 로컬 cycle 검증은 다음 스크립트를 사용한다.

```bash
cd /home/a/autoware/src
REQUIRE_PC3_INPUTS=0 ./migration_work/scripts/validate_pc2_cycles.sh 1
```

세 PC가 모두 올라온 통합 검증에서는 다음처럼 원격 입력 gate까지 활성화한다.

```bash
REQUIRE_PC3_INPUTS=1 ./migration_work/scripts/validate_pc2_cycles.sh 1
```

로컬 모드도 PC1/PC3 필수 endpoint가 없다는 사실을 로그에 남긴다. 카메라/YOLO local PASS를
TLR/3-PC semantic PASS로 승격하면 안 된다.

## 16. 증거 경로 요약

- 네트워크 운용/복구 가이드:
  `migration_work/runbooks/PC2_network_and_port_setup.md`
- standalone 두 카메라 launch:
  `/home/a/.ros/log/2026-08-12-14-31-36-826415-a-10818/launch.log`
- 전체 `run_autoware` launch:
  `/home/a/.ros/log/2026-08-12-14-33-21-804273-a-11952/launch.log`
- 안정화 후 최종 guarded cycle의 child launch/cleanup:
  `migration_work/test_logs/live_camera_cycles/20260812_145512/cycle_1.log`
- cold-start 90초 제한이 짧음을 확인한 중간 cycle:
  `migration_work/test_logs/live_camera_cycles/20260812_145029/cycle_1.log`
- Lucid rebuild:
  `/home/a/ros2_ws/log/build_2026-08-12_14-41-38/`
- cycle launcher log 위치:
  `migration_work/test_logs/live_camera_cycles/`
- 카메라 contract probe:
  `migration_work/scripts/probe_camera_contract.py`
- YOLOX contract probe:
  `migration_work/scripts/probe_yolox_contract.py`
- guarded top-level launcher:
  `migration_work/scripts/run_pc2_autoware.sh`

Git commit에는 source, parameter, launch, script, runbook과 이 보고서를 포함하되 `/tmp`,
`build`, `install`, `log`, generated TensorRT engine와 대용량 runtime log는 source tree에
복사하지 않는다. 원본 실행 로그의 절대 경로와 측정 수치는 이 문서에 보존한다.
