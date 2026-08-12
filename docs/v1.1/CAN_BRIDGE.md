# PC1 CAN bridge architecture

## 시작 경로

```text
cd ~/scripts && ./start.sh
  -> can0/can1 DOWN
  -> bitrate 500000
  -> can0/can1 UP

run_bridge
  -> ros2 launch ros2_socketcan can_brdige.launch.xml
     -> socket_can_bridge.launch.xml
        -> socket_can_receiver_node_exe (can0)
        -> socket_can_sender_node_exe (can0)
     -> twistController2VCU2EPS2ACC_node
```

현재 `run_bridge`는 full RX/TX bridge다. 차량 상태 수신만 하는 명령이 아니다.

## 수신 경로

```text
Vehicle CAN
  -> Kvaser USBcan
  -> Linux SocketCAN can0
  -> /socket_can_receiver_can0
  -> /from_can_bus
  -> custom IONIQ EV decoder
  -> /vehicle/status/* and /current/*
```

| CAN ID | Decimal | 주기/조건 | 주요 decode 출력 | 비고 |
|---|---:|---|---|---|
| `0x371` | 881 | 약 10 ms | `/current/brake_press_status` | brake press scaling |
| `0x372` | 882 | 차량 상태 | `/vehicle/status/gear_status` | dual raw mapping은 실차 trace 검증 필요 |
| `0x381` | 897 | 약 10 ms | steering status와 RViz steering | steering wheel angle을 tire angle로 환산 |
| `0x386` | 902 | wheel speed | velocity 계산의 longitudinal 부분 | 0x394와 함께 combined publish |
| `0x394` | 916 | accel/rate | `/vehicle/status/velocity_status` | 코드 내 DBC mapping 불일치 주석 존재 |
| `0x4F1` | 1265 | cruise button | `/current/cruise_mode_status`, `/vehicle/status/control_mode` | exact-byte 0x28/0x08 비교 및 edge toggle |

publisher endpoint는 존재하지만 hazard/turn decode switch case가 주석 처리되어 있어 해당 status가 실제로 갱신된다고 가정하면 안 된다.

## 송신 경로

```text
/control/command/control_cmd
  -> /twistController2VCU2EPS2ACC_node
  -> /to_can_bus (can_msgs/msg/Frame)
  -> /socket_can_sender_can0
  -> Linux SocketCAN can0
  -> Vehicle CAN
```

converter는 약 30 ms timer, 즉 약 33.3 Hz로 CAN ID `0x630`을 발행한다.

| 입력/상태 | 타입/초깃값 | 처리 |
|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | acceleration과 steering command 수신 |
| `/current/cruise_mode_status` | `std_msgs/msg/Bool` | data byte에 control request 반영 |
| acceleration | 초기 `-1.0 m/s²` | 범위 `[-5.0, 1.5]`로 clamp |
| steering | 초기 `0 rad` | tire rad × RAD2DEG × 16.16667, 약 ±438° 제한 |
| CAN ID | `0x630` | standard frame, `/to_can_bus`로 전송 |

control_cmd가 한 번도 오지 않아도 converter timer가 초기값 프레임을 발행한다. 따라서 `run_bridge` 실행 자체가 물리 CAN write를 시작하며, 단순 subscriber 준비 상태로 해석하면 안 된다.

## cruise/display 문제가 발생했던 이유

중간 안전 실험에서 sender와 converter를 끄고 receiver만 실행했다. 이 구성에서는 0x4F1 버튼 프레임을 읽을 수 있어도 차량 display/제어 요청에 필요한 0x630 write가 없었다. 결과적으로 cruise 표시와 steering/longitudinal hold가 잡히지 않았다.

full bridge를 복원한 뒤 동작이 돌아왔다. 이번 작업에서 cruise decoder를 새 알고리즘으로 변경하여 해결한 것이 아니다.

## 정상 운영 순서

실차에서 다음 순서는 actuator-capable 절차다. 운전자, 브레이크, 주변 안전과 차량 시험 권한이 확보된 경우에만 수행한다.

1. 차량 고정/정차와 운전자 제동을 확인한다.
2. `cd ~/scripts && ./start.sh`로 can0/can1을 준비한다.
3. `run_bridge` 한 인스턴스만 실행한다.
4. receiver/sender/converter 세 노드가 각 하나인지 확인한다.
5. `/vehicle/status/gear_status`, steering, velocity, control_mode의 실제 freshness를 확인한다.
6. cruise 버튼과 0x4F1/0x630 흐름을 확인한다.
7. PC3 foundation, PC2 perception, PC1 Autoware를 올리고 데이터 readiness를 확인한다.
8. map/localization/TF/perception/MRM/start_planner가 모두 정상일 때만 Auto/engage 시험으로 진행한다.

## v1.1 미해결 위험

### 전역 TX topic

sender는 Domain 10의 전역 `/to_can_bus`를 구독한다. 다른 PC가 같은 topic을 publish하면 PC1 sender가 물리 can0에 쓸 수 있다. PC별 namespace와 explicit allowlist가 없다.

### command freshness watchdog 부재

converter는 마지막 acceleration/steering 값을 보관하고 timer로 반복 전송한다. upstream control_cmd가 끊겼을 때 일정 시간 후 안전값으로 전환하는 명시적 watchdog이 없다.

2026-08-12 실측에서는 마지막 control command가 약 1시간 이상 오래된 상태에서도 CAN ID `0x630`이 약 33.33 Hz로 계속 송신됐다. payload의 acceleration은 cached `-2.4 m/s²`, steering은 0이었고 cruise=false라 control request byte는 0이었다. 그러나 cruise가 true가 되면 cached 값은 그대로인 채 request byte가 1이 될 수 있으므로, “현재 정차 중” 또는 “upstream topic이 끊김”을 안전 근거로 사용할 수 없다.

### interface argument typo

`socket_can_bridge.launch.xml`은 child에 `interface1`을 넘기지만 child launch가 선언한 이름은 `interface`다. 현재 can0 동작은 child default에 의존한다. v1.1 기능 patch로 수정하려면 launch parse와 can0 regression을 별도 commit으로 검증해야 한다.

### cruise state edge logic

0x4F1 비교가 exact byte에 의존하고 toggle state를 사용한다. 다른 status bit가 함께 설정되거나 frame drop이 발생하면 실제 vehicle mode와 ROS state가 어긋날 수 있다.

### ACC state 변수

현재 코드에는 ACC branch에서 `previous_acc_pressed`가 아닌 `previous_cancel_pressed`를 갱신하는 것으로 보이는 기존 결함이 있다. 실차 입력 trace를 기반으로 수정하고 unit test를 추가해야 한다.

## 릴리스 검증 요구

- `/to_can_bus` publisher는 의도한 converter 하나만 존재
- sender subscriber는 PC1 하나만 존재
- CAN 0x630 rate와 payload를 candump로 기록
- control_cmd 중단 시 실제 송신 동작 기록
- cruise press/cancel 각각 20회 edge test
- P/R/N/D 및 drive-mode별 0x372 raw byte 기록
- Ctrl+C 종료 후 sender/receiver/converter orphan 0
- CAN error counter, bus-off, dropped frame 확인
