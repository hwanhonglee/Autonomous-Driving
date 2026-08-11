# PC1 experiments and reverted changes

이 문서는 최종 v1.1 코드가 아니라, 원인 분석 과정에서 적용했다가 되돌린 변경과 그 부작용을 기록한다. 아래 항목을 final configuration으로 복사하면 안 된다.

## 1. fail-closed top launch

### 목적

정차 통신 시험에서 Planning/API만 먼저 실행하고 Control, RViz, 차량 interface 및 비-PC1 모듈을 끄려 했다.

### 변경

- `autoware.launch.xml` 여러 default를 false로 변경
- control, parking, RViz adaptor 일부를 끔
- guarded `run_autoware` helper를 alias 뒤에 배치

### 발생 문제

- 사용자가 요구한 기존 PC1 전체 기능과 달라졌다.
- Control과 RViz가 없어 실제 3-PC 경로/제어 시험을 할 수 없었다.
- helper 파일과 alias가 불일치한 순간 `run_autoware_safe.sh: No such file or directory`가 발생했다.
- 이미 열린 shell에 옛 alias가 cache되어 현재 파일과 보이는 동작이 달라졌다.

### 최종 처리

`autoware.launch.xml`과 direct alias를 v1.0으로 복원했다. 현재 active top launch SHA는 v1.0과 같다.

## 2. vcan bridge

### 목적

물리 actuator와 분리하여 raw ROS↔CAN 전달 및 `/vehicle/status/*` decoder를 먼저 확인하려 했다.

### 변경

- vcan0 생성
- PC별 namespaced raw CAN topic 시도
- physical sender와 control adapter 차단

### 발생 문제

- 사용자가 요구한 실제 차량 cruise display와 longitudinal/lateral hold를 검증할 수 없었다.
- 현재 historical launch는 전역 `/to_can_bus`를 전제로 하므로 namespaced topic 실험과 호환되지 않았다.
- 실제 운영 순서가 불필요하게 복잡해졌다.

### 최종 처리

vcan, namespace remap, safe wrapper를 제거하고 v1.0 full physical bridge로 복원했다.

## 3. physical receive-only bridge

### 목적

실차 CAN status는 받되 PC1이 actuator CAN을 쓰지 않게 하려 했다.

### 변경

- receiver ON
- sender OFF
- `twistController2VCU2EPS2ACC_node` OFF
- lock과 process guard 적용

### 직접 부작용

기존 차량은 cruise 버튼과 함께 control adapter가 CAN ID `0x630`을 약 33 Hz로 보내야 display와 종·횡 제어 hold가 잡히는 운영 흐름이었다. receive-only 구성은 0x630 write를 없앴기 때문에 다음 현상이 발생했다.

- cruise display가 나타나지 않음
- steering/longitudinal hold가 잡히지 않음
- `run_bridge` lock이 남거나 다른 instance가 있으면 `another run_bridge instance holds ...`로 실행 차단

### 최종 처리

sender와 converter를 포함하는 v1.0 full bridge를 복원했다. cruise decoder를 새로 고쳐서 해결한 것이 아니다.

## 4. cruise debounce experiment

cruise button debounce를 잠시 시험하고 rebuild했으나 현재 source는 사전 snapshot과 byte-identical이다. 변경 내용은 최종 코드에 남아 있지 않으며 v1.1 개선으로 기록하지 않는다.

## 5. isolated planning simulator experiment

### 목적

Domain 110/localhost 환경에서 map, route, trajectory, internal control을 물리 CAN 없이 시험하려 했다.

### 변경

- `launch_map`과 `launch_control` opt-in
- `initial_engage_state=false`
- RViz default false
- local map include

### 발생 문제

- PC1 top launch는 실제 map include가 없어서 별도 map group이 필요했다.
- simulator occupancy grid가 존재하지 않는 pointcloud container에 component를 load하려 했다.
- 실제 3-PC `run_autoware`와 다른 경로여서 실차 통합의 정본으로 사용할 수 없었다.

### 최종 처리

2026-08-11 release 감사에서 `planning_simulator.launch.xml`을 v1.0 SHA로 복원했다. 이 실험은 문서 이력만 남긴다.

## 6. 47-node five-cycle test

fail-closed Planning/API profile에서 5회 start/stop을 수행하여 각 cycle 47 nodes, duplicate FQN 0, clean Ctrl+C를 기록했다. 이는 현재 full Planning/Control/API/RViz configuration의 회귀 시험이 아니므로 v1.1 acceptance 결과로 재사용하지 않는다.

## 교훈

- 안전 profile은 운영 profile과 별도 launch/branch로 분리해야 한다.
- 기존 alias 이름을 유지하더라도 shell cache와 helper 존재 여부를 함께 검증해야 한다.
- receive-only와 full control bridge는 같은 `run_bridge` 이름으로 조용히 바꾸면 안 된다.
- runtime 상태, 중간 실험, final release 상태를 같은 changelog의 “active” 표에 섞지 않는다.
