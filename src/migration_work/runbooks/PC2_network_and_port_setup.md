# PC2 네트워크·물리 포트 현장 설정 가이드

기준 시각: 2026-08-12 KST
대상 장비: PC2
범위: 인터넷, PC1/PC3 ROS 2 DDS, Lucid 카메라 2대의 NIC 분리와 복구

> 이 문서는 현재 PC2에서 직접 확인한 설정을 기준으로 한다. Wi-Fi 비밀번호는 기록하지
> 않으며, 아래 절차도 비밀번호를 명령행 인수로 받지 않는다. 네트워크 프로필을 바꾸면 실행
> 중인 ROS 2 통신이 끊길 수 있으므로 `run_autoware`를 내린 상태에서 모드를 전환한다.

## 1. 먼저 답: 유선 인터넷 선을 빼고 Wi-Fi를 켜면 무엇이 바뀌나

IP 주소는 PC 전체에 하나가 붙는 것이 아니라 **각 네트워크 인터페이스(NIC)에 따로** 붙는다.
따라서 유선 케이블을 빼고 Wi-Fi를 연결하면 다음처럼 된다.

- `wlx200db04a992d`는 Wi-Fi 공유기에서 별도의 DHCP 주소, 기본 게이트웨이, DNS를 받는다.
- `enp0s31f6`는 케이블 링크를 잃는다. 이 인터페이스의 `192.168.9.11` 또는
  `192.168.9.110` 주소와 `192.168.9.0/24` 직접 경로도 사용할 수 없게 된다.
- 인터넷은 Wi-Fi로 계속 사용할 수 있다.
- PC1(`192.168.9.2`)·PC3(`192.168.9.7`)가 그 유선 차량망에 있다면 두 PC와의 ROS 2
  통신은 끊긴다.
- 현재 CycloneDDS가 `enp0s31f6`에 고정되어 있으므로 인터넷이 Wi-Fi에서 잘 되더라도 ROS 2가
  자동으로 Wi-Fi 포트로 옮겨 가지 않는다. 이것이 의도된 동작이다.
- `enp1s0f0`과 `enp1s0f3`의 카메라 케이블을 그대로 두면 카메라 전용 링크는 영향을 받지
  않는다.

즉, **Wi-Fi 인터넷을 쓰면서 PC1/PC3와 통신하려면 인터넷용 유선 케이블을 그냥 빼는 것이
아니라, Wi-Fi를 인터넷 기본 경로로 만들고 `enp0s31f6` 케이블은 차량 ROS 2 전용으로 계속
연결해야 한다.**

## 2. 여기서 말하는 “포트”의 구분

현장에서 혼동하기 쉬운 세 가지를 구분한다.

1. 물리/NIC 포트: `enp0s31f6`, `enp1s0f0`, `enp1s0f3`, `wlx200db04a992d` 같은 Linux
   인터페이스 이름이다.
2. IP 주소: 활성화한 NetworkManager 프로필이 해당 NIC에 붙이는 주소이다.
3. UDP/TCP 포트 번호: DDS와 GigE Vision이 통신에 사용하는 전송 계층 번호이다. ROS 2 DDS는
   단일 고정 포트 하나만 쓰는 구조가 아니며, 현재 구성에서는 `ROS_DOMAIN_ID=10`, 멀티캐스트,
   NIC 선택이 더 중요한 구분 기준이다.

프로필은 저장된 설정이고, NIC는 실제 장치이다. 한 NIC에는 한 시점에 하나의 NetworkManager
프로필만 활성화된다. 예를 들어 `enp0s31f6`에서 `Ethernet`을 쓰다가 `ROS2`를 활성화하면
동일 NIC의 활성 프로필과 주소가 교체된다.

## 3. 현재 확인된 물리 구성

```text
                 모드 A의 인터넷/차량망 라우터 또는 스위치
                              192.168.9.0/24
                       ┌────────────┴────────────┐
                 PC1 192.168.9.2          PC3 192.168.9.7
                              │
              enp0s31f6: 192.168.9.11/24 (현재 DHCP)
                              │
                            PC2
             ┌────────────────┼──────────────────┐
             │                │                  │
      enp1s0f0          enp1s0f3       wlx200db04a992d
      192.168.41.1      169.254.0.1     Wi-Fi 인터넷(현재 꺼짐)
             │                │
      Windshield CAM    Loop Top CAM
      192.168.41.2      169.254.0.11
```

| 역할 | 인터페이스 | MAC | 현재 프로필/주소 | 게이트웨이 정책 |
|---|---|---|---|---|
| PC1/PC3 DDS + 현재 유선 인터넷 | `enp0s31f6` | `78:D0:04:2F:17:C5` | `Ethernet`, DHCP `192.168.9.11/24` | 현재 `192.168.9.1`, 기본 경로 있음 |
| 저장된 차량 ROS 2 전용 설정 | `enp0s31f6` | 동일 | `ROS2`, 고정 `192.168.9.110/24`, 현재 비활성 | 게이트웨이/DNS 없음, `never-default=yes` |
| Windshield Lucid | `enp1s0f0` | `C4:00:AD:C8:57:98` | `192.168.41.1/24` | 게이트웨이/DNS 없음, `never-default=yes`, MTU 9000 |
| Loop Top Lucid | `enp1s0f3` | `C4:00:AD:C8:57:9B` | `169.254.0.1/24` | 게이트웨이/DNS 없음, `never-default=yes`, MTU 9000 |
| Wi-Fi 인터넷 | `wlx200db04a992d` | `20:0D:B0:4A:99:2D` | 저장 프로필 여러 개, 현재 DOWN | Wi-Fi 연결 시 DHCP 기본 경로 사용 |

현재 추가 상태:

- Wi-Fi 하드웨어는 보이지만 `WIFI=disabled`, `rfkill Soft blocked=yes`, `Hard blocked=no`이다.
- `/home/a/.bashrc`와 PC2 실행 도우미는 `ROS_DOMAIN_ID=10`,
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_LOCALHOST_ONLY=0`을 사용한다.
- CycloneDDS는 이름이 `enp0s31f6`인 NIC에 고정되어 있다.
- 현재 기본 IPv4 경로는 `default via 192.168.9.1 dev enp0s31f6`이다.
- `192.168.9.11`은 DHCP 임대 주소이므로 케이블 재연결이나 재부팅 후 반드시 같다고 보장할 수
  없다. 고정 ROS 2 프로필의 주소는 `192.168.9.110`이다.

## 4. 지원하는 두 가지 운용 모드

### 모드 A — 현재 상태: 유선 하나로 인터넷과 ROS 2를 함께 사용

`enp0s31f6`에서 `Ethernet` DHCP 프로필을 쓴다.

- 현재 주소: `192.168.9.11/24`
- 인터넷 기본 경로: `192.168.9.1`
- PC1/PC3 경로: 같은 `192.168.9.0/24`의 직접 연결
- 장점: 단순하고 현재 PC1·PC3 핑이 검증되었다.
- 주의: DHCP 주소가 바뀔 수 있다. PC2 주소를 `.11`로 가정한 방화벽, 허용 목록, 기록은 다시
  확인해야 한다.

### 모드 B — 권장 분리: Wi-Fi는 인터넷, 유선은 ROS 2 전용

```text
Internet ─ Wi-Fi AP ─ wlx200db04a992d (DHCP + default route)

PC1/PC3 vehicle LAN ─ enp0s31f6 (ROS2 profile, 192.168.9.110/24, no gateway)

Wind camera ─ enp1s0f0 (192.168.41.1/24)
Loop camera ─ enp1s0f3 (169.254.0.1/24)
```

이 모드에서는 `enp0s31f6` 케이블을 빼면 안 된다. 인터넷만 Wi-Fi로 옮기고, PC1/PC3 DDS는
계속 유선으로 보낸다.

중요한 선행 조건은 **Wi-Fi가 차량 유선망과 다른 서브넷을 받아야 한다**는 것이다. Wi-Fi가
`192.168.9.x/24`를 받으면 Wi-Fi와 `enp0s31f6`에 동일한 `192.168.9.0/24` 경로가 두 개
생긴다. 이 경우 PC1/PC3 패킷이 잘못된 NIC로 나갈 수 있으므로 모드 B로 진행하지 않는다.
다른 핫스팟/공유기 서브넷을 사용하거나 모드 A를 유지한다. Wi-Fi 서브넷은 다음 네 네트워크와
겹치지 않는 것이 안전하다.

- 차량망: `192.168.9.0/24`
- Windshield 카메라망: `192.168.41.0/24`
- Loop Top 카메라망: `169.254.0.0/24`
- 다른 현장 장비가 이미 사용하는 고정망

## 5. 모드 A에서 모드 B로 안전하게 전환

### 5.1 실행 중인 노드와 현재 상태 확인

네트워크 전환 전에 `run_autoware`를 정상 종료한다. 그다음 비밀 정보가 포함되지 않는 상태를
기록한다.

```bash
date
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
nmcli -f NAME,UUID,TYPE,DEVICE,AUTOCONNECT connection show
ip -br -4 address
ip -4 route show table main
rfkill list
```

### 5.2 Wi-Fi 활성화

현재는 소프트 블록이므로 다음 순서로 켠다.

```bash
nmcli radio wifi on
rfkill list
```

여전히 `Soft blocked: yes`이면 그때만 다음을 실행한다.

```bash
sudo rfkill unblock wifi
nmcli radio wifi on
```

`Hard blocked: yes`이면 명령으로 우회하지 않는다. USB Wi-Fi 동글, 물리 스위치, BIOS 설정을
확인한다.

활성화 가능한 AP를 확인한다.

```bash
nmcli device wifi rescan ifname wlx200db04a992d
nmcli device wifi list ifname wlx200db04a992d
```

이미 저장된 프로필은 다음처럼 연결한다. 비밀번호가 필요하면 `--ask`가 터미널에서 숨김
입력을 요청한다.

```bash
nmcli --ask connection up id "<저장된_WIFI_프로필명>" ifname wlx200db04a992d
```

새 AP를 등록해야 할 때도 비밀번호를 명령행에 직접 쓰지 않는다.

```bash
nmcli --ask device wifi connect "<SSID>" ifname wlx200db04a992d
```

또는 Ubuntu 데스크톱 Wi-Fi 메뉴/`nm-connection-editor`에서 암호를 입력한다. 다음과 같은
형태는 셸 기록과 프로세스 목록에 암호가 남을 수 있으므로 사용하지 않는다.

```text
# 사용 금지 예시
nmcli device wifi connect "<SSID>" password "<PASSWORD>"
```

Wi-Fi 라디오를 켜면 도달 가능한 자동연결 프로필 중 하나가 먼저 붙을 수 있다. 반드시 활성
프로필과 실제 DHCP 주소를 확인한다.

```bash
nmcli connection show --active
ip -br -4 address show dev wlx200db04a992d
ip -4 route show dev wlx200db04a992d
```

여기서 Wi-Fi 주소가 `192.168.9.x/24`, `192.168.41.x/24`, 또는
`169.254.0.x/24`이면 중단하고 다른 AP를 선택한다.

### 5.3 유선을 바꾸기 전에 Wi-Fi 인터넷 자체 검증

현재 유선 기본 경로가 더 우선일 수 있으므로 일반 `ping`만 보고 Wi-Fi가 검증됐다고 판단하지
않는다. 인터페이스를 지정한다.

```bash
ping -I wlx200db04a992d -c 3 -W 1 1.1.1.1
resolvectl status wlx200db04a992d
curl --interface wlx200db04a992d --max-time 8 -I https://example.com/
```

ICMP가 차단된 AP에서는 `ping`이 실패해도 HTTPS가 성공할 수 있다. DHCP 주소, Wi-Fi 기본
경로, DNS, HTTPS를 함께 판단한다.

### 5.4 같은 유선 NIC를 고정 ROS 2 프로필로 교체

Wi-Fi 인터넷과 비중복 서브넷을 확인한 뒤에만 실행한다. 이 명령은 `enp0s31f6`의 활성
`Ethernet` 프로필을 `ROS2`로 교체하므로 유선 주소가 `.11`에서 `.110`으로 바뀐다.

```bash
nmcli connection up uuid ed3c9c36-5ad4-47a6-8a13-6ba48e0e8e05 ifname enp0s31f6
```

예상 결과:

- `enp0s31f6`: `192.168.9.110/24`
- `enp0s31f6`에는 기본 게이트웨이와 DNS가 없음
- 인터넷 기본 경로는 `wlx200db04a992d`
- PC1/PC3의 `192.168.9.0/24`는 `enp0s31f6`의 직접 경로

### 5.5 모드 B 검증

```bash
nmcli connection show --active
ip -br -4 address
ip -4 route show table main
ip -4 route get 192.168.9.2
ip -4 route get 192.168.9.7
ip -4 route get 1.1.1.1
ping -I enp0s31f6 -c 3 -W 1 192.168.9.2
ping -I enp0s31f6 -c 3 -W 1 192.168.9.7
```

정상적인 핵심 출력은 다음 의미를 가져야 한다.

```text
192.168.9.2 ... dev enp0s31f6 src 192.168.9.110
192.168.9.7 ... dev enp0s31f6 src 192.168.9.110
1.1.1.1 ... dev wlx200db04a992d
```

`1.1.1.1`의 실제 Wi-Fi 게이트웨이와 소스 주소는 연결한 AP에 따라 달라진다.

### 5.6 ROS 2 환경과 발견 검증

새 터미널에서 다음을 확인한다.

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
```

현재 기대값은 다음과 같다.

```text
ROS_DOMAIN_ID=10
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI 안의 NetworkInterface name="enp0s31f6"
```

IP/프로필 전환 전에 생성된 ROS 2 CLI 데몬은 이전 네트워크 상태를 보유할 수 있다. 환경을
확인한 새 터미널에서 데몬을 다시 만든다.

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

DDS 발견에는 몇 초가 걸릴 수 있다. PC1/PC3에서도 Domain 10과 동일 RMW를 확인한다. 핑은
통과하지만 토픽이 안 보이면 아래 문제표의 DDS 항목을 따른다.

## 6. 모드 B에서 모드 A로 복귀

유선 케이블이 `enp0s31f6`에 연결되어 있고 링크가 올라온 것을 먼저 확인한다.

```bash
ip -br link show dev enp0s31f6
sudo ethtool enp0s31f6 | grep -E 'Speed:|Duplex:|Link detected:'
```

그다음 DHCP `Ethernet` 프로필을 활성화한다.

```bash
nmcli connection up uuid 4b840dfd-3599-49c2-a59a-4a0b6dddd5ee ifname enp0s31f6
```

DHCP가 끝난 뒤 주소와 경로를 검증한다. 과거 임대가 `.11`이었다고 해서 다시 `.11`임을
가정하지 않는다.

```bash
ip -br -4 address show dev enp0s31f6
ip -4 route get 192.168.9.2
ip -4 route get 192.168.9.7
ip -4 route get 1.1.1.1
ping -I enp0s31f6 -c 3 -W 1 192.168.9.2
ping -I enp0s31f6 -c 3 -W 1 192.168.9.7
curl --interface enp0s31f6 --max-time 8 -I https://example.com/
```

유선 인터넷과 PC1/PC3 통신이 모두 확인된 뒤에만 필요 시 Wi-Fi를 끈다.

```bash
nmcli radio wifi off
```

## 7. 카메라 전용 NIC 설정

카메라 NIC에는 인터넷 기본 게이트웨이와 DNS를 절대 넣지 않는다. 카메라 트래픽이 인터넷이나
PC1/PC3 트래픽과 경쟁하지 않도록 별도 서브넷, `never-default=yes`, MTU 9000을 유지한다.

### 7.1 현재 Windshield 변경 내용

Windshield 카메라(serial `222301529`)는 `192.168.41.2/24`로 발견되었다. 호스트 프로필의
이전 `169.254.121.1/24`는 이 카메라와 같은 서브넷이 아니었다. 현재는 다음처럼 맞췄다.

| 항목 | 이전 기록 | 현재 운용값 |
|---|---|---|
| NIC | `enp1s0f0` | `enp1s0f0` |
| 호스트 주소 | `169.254.121.1/24` | `192.168.41.1/24` |
| 카메라 주소 | `192.168.41.2/24` | `192.168.41.2/24` — 카메라 자체 주소는 변경하지 않음 |
| 기본 경로 차단 | 이전 `never-default=no` 기록 | `never-default=yes` |
| MTU | 9000 | 9000 |

현재 알려진 정상 설정을 재적용해야 할 때만 다음 복구 명령을 사용한다. 실행하면 해당 카메라
링크가 잠시 끊기므로 카메라/Autoware를 먼저 종료한다.

```bash
nmcli connection modify uuid 9100063a-de5a-4a02-adbc-ef458e474c13 \
  connection.interface-name enp1s0f0 \
  ipv4.method manual \
  ipv4.addresses 192.168.41.1/24 \
  ipv4.gateway "" \
  ipv4.routes "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  802-3-ethernet.mtu 9000
nmcli connection up uuid 9100063a-de5a-4a02-adbc-ef458e474c13 ifname enp1s0f0
```

감사 목적의 정확한 이전 상태 롤백은 아래와 같다. **현재 카메라 `.2`와 통신이 끊기므로 현
운용 토폴로지에서는 실행하지 않는다.** 비교 실험을 승인받고 즉시 다시 현재 값으로 복구할
때만 사용한다.

```bash
nmcli connection modify uuid 9100063a-de5a-4a02-adbc-ef458e474c13 \
  ipv4.method manual \
  ipv4.addresses 169.254.121.1/24 \
  ipv4.gateway "" \
  ipv4.routes "" \
  ipv4.dns "" \
  ipv4.never-default no \
  802-3-ethernet.mtu 9000
nmcli connection up uuid 9100063a-de5a-4a02-adbc-ef458e474c13 ifname enp1s0f0
```

### 7.2 Loop Top 현재 설정

- 호스트: `enp1s0f3`, `169.254.0.1/24`
- 카메라(serial `214000332`): `169.254.0.11`
- 프로필: `Lucid Camera Loop Top`
- 게이트웨이/DNS 없음, `never-default=yes`, MTU 9000

### 7.3 두 카메라 링크 검증

```bash
nmcli connection show --active
ip -br -4 address show dev enp1s0f0
ip -br -4 address show dev enp1s0f3
ip -4 route get 192.168.41.2
ip -4 route get 169.254.0.11
ping -I enp1s0f0 -c 3 -W 1 192.168.41.2
ping -I enp1s0f3 -c 3 -W 1 169.254.0.11
sudo ethtool enp1s0f0 | grep -E 'Speed:|Duplex:|Link detected:'
sudo ethtool enp1s0f3 | grep -E 'Speed:|Duplex:|Link detected:'
```

예상 경로는 Windshield `.2`가 `enp1s0f0 src 192.168.41.1`, Loop Top `.11`이
`enp1s0f3 src 169.254.0.1`이다. `run_autoware` 도우미가 실행 중에 Loop Top 장치에 대한
정확한 `/32` 경로를 임시로 추가했다가 종료 시 제거할 수 있다. 저장 프로필의 정적 라우트가
생겼다는 뜻은 아니므로 런타임과 종료 후 상태를 구분한다.

## 8. 프로필, 인터페이스 이름, MAC 바인딩

NetworkManager가 프로필을 잘못된 물리 포트에 붙이지 않도록 다음 세 정보를 함께 본다.

- `connection.interface-name`: 프로필을 특정 Linux 인터페이스 이름에 제한한다.
- `802-3-ethernet.mac-address` 또는 `802-11-wireless.mac-address`: 특정 하드웨어 MAC에
  제한한다.
- `GENERAL.HWADDR`: 현재 실제 NIC의 MAC이다.

현재 `ROS2`, 두 카메라 프로필, 저장된 Wi-Fi 프로필은 인터페이스 이름에 묶여 있다.
`Ethernet` DHCP 프로필은 저장 설정상 인터페이스/MAC 제한이 없지만 현재
`enp0s31f6`에서 활성화되어 있다. 유선 프로필의 MAC 제한은 현재 설정되어 있지 않다.

읽기 전용 확인 명령:

```bash
nmcli -f connection.id,connection.uuid,connection.interface-name,connection.autoconnect,\
802-3-ethernet.mac-address,802-3-ethernet.mtu,ipv4.method,ipv4.addresses,ipv4.gateway,\
ipv4.routes,ipv4.dns,ipv4.never-default connection show "ROS2"

nmcli -f GENERAL.DEVICE,GENERAL.HWADDR,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY \
  device show enp0s31f6
```

MAC까지 고정하는 작업은 장치 교체 시 프로필이 올라오지 않게 하는 장점이 있지만, NIC 교체나
USB Wi-Fi 동글 교체 후에는 반드시 새 MAC으로 갱신해야 한다. 현장 테스트 직전에 임의로
바꾸지 말고, 아래 매핑을 실제 `GENERAL.HWADDR`과 대조한 뒤 별도 유지보수 시간에 적용한다.

```text
enp0s31f6        78:D0:04:2F:17:C5
enp1s0f0         C4:00:AD:C8:57:98
enp1s0f3         C4:00:AD:C8:57:9B
wlx200db04a992d  20:0D:B0:4A:99:2D
```

인터페이스 이름이 달라졌다면 이름을 추측하지 말고 MAC으로 찾는다.

```bash
ip -br link
nmcli -f GENERAL.DEVICE,GENERAL.HWADDR,GENERAL.TYPE device show
```

## 9. 재부팅 후 지속성

NetworkManager의 저장 프로필은 재부팅 후에도 남지만, 어떤 프로필이 자동으로 올라오는지는
`connection.autoconnect`와 무선 라디오 상태에 달려 있다.

현재 확인값:

- `Ethernet`: `autoconnect=no`
- `ROS2`: `autoconnect=yes`
- 두 Lucid 카메라 프로필: `autoconnect=yes`
- 저장된 Wi-Fi 프로필들: `autoconnect=yes`
- Wi-Fi 라디오: 현재 disabled/soft-blocked

따라서 지금 재부팅하면 현재 수동 활성화된 `Ethernet` DHCP가 다시 선택된다고 가정하면 안
된다. 케이블이 연결되어 있으면 자동연결 `ROS2` 프로필 `.110`이 선택될 수 있다. Wi-Fi
라디오의 켜짐/꺼짐 상태도 재부팅 후 다시 확인한다.

재부팅 직후에는 `run_autoware`보다 먼저 다음 체크를 수행한다.

```bash
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status
nmcli connection show --active
ip -br -4 address
ip -4 route show table main
rfkill list
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
```

그 결과를 보고 모드 A 또는 모드 B 중 하나를 명시적으로 완성한다. 두 모드의 중간 상태에서
Autoware를 실행하지 않는다.

## 10. 장애 증상별 빠른 판단표

| 증상 | 가능성이 큰 원인 | 먼저 확인/조치 |
|---|---|---|
| Wi-Fi 인터넷은 되지만 PC1/PC3 토픽이 안 보임 | `enp0s31f6` 케이블/프로필 DOWN 또는 CycloneDDS가 유선에 고정 | `ip -br link`, `ip route get 192.168.9.2`, 유선 핑, Domain 10 확인 |
| PC1/PC3 핑은 되지만 ROS 토픽이 안 보임 | Domain/RMW/`ROS_LOCALHOST_ONLY` 불일치, CLI 데몬이 이전 환경 사용, 멀티캐스트 차단 | 세 PC 환경 비교, `ros2 daemon stop/start`, DDS 멀티캐스트 확인 |
| 모드 B에서 ROS는 되지만 인터넷이 안 됨 | Wi-Fi DHCP/default route/DNS 미완료 | `ip route get 1.1.1.1`, `resolvectl status`, Wi-Fi 지정 HTTPS 확인 |
| Wi-Fi와 유선을 함께 켠 뒤 PC1/PC3 경로가 Wi-Fi로 감 | Wi-Fi도 `192.168.9.0/24`를 받아 서브넷 중복 | 모드 B 중단, 다른 AP/핫스팟 사용 또는 모드 A 복귀 |
| 재부팅 후 `.11`이 아니라 `.110`이 됨 | `Ethernet autoconnect=no`, `ROS2 autoconnect=yes` | 오류가 아니라 저장 정책 결과. 원하는 모드를 명시적으로 활성화 |
| 재연결 후 DHCP 주소가 `.11`이 아님 | DHCP 임대 변경 | 실제 주소를 기준으로 테스트하고 고정 주소 가정을 제거 |
| 카메라는 discovery되지만 스트림이 안 됨 | 호스트 주소/카메라 주소 불일치, 잘못된 NIC, 링크/MTU 문제 | `ip route get`, 인터페이스 지정 핑, 프로필/MTU/serial 확인 |
| 카메라 NIC가 인터넷 기본 경로로 표시됨 | 카메라 프로필에 gateway가 있거나 `never-default`가 꺼짐 | 카메라 노드 종료 후 알려진 정상 프로필 값 재적용 |
| 프로필 전환 직후 토픽 수가 0 또는 들쭉날쭉함 | DDS 재발견/CLI 데몬 캐시 | 새 터미널에서 환경 확인, 데몬 재시작 후 수 초 지속 관찰 |
| `ROS2` 프로필이 올라오지 않음 | 케이블 no-carrier, 잘못된 ifname, `.110` 주소 충돌 | `ethtool`, 프로필 바인딩, 현장 주소 할당표 확인 |

PC1/PC3에서 멀티캐스트를 양방향으로 확인할 수 있을 때는 한 PC에서 수신을 먼저 실행하고
다른 PC에서 송신한다.

```bash
# 수신측
ros2 multicast receive

# 송신측
ros2 multicast send
```

세 PC 모두 `ROS_DOMAIN_ID=10`, `ROS_LOCALHOST_ONLY=0`, 동일하게 의도한 RMW인지 함께
기록한다. `ping` 성공은 IP 계층만 검증하며 DDS 발견 성공을 대신하지 않는다.

## 11. 복구 원칙

1. 인터넷이 끊겼다면 카메라 프로필을 만지지 말고 기본 경로와 Wi-Fi/`enp0s31f6`만 본다.
2. PC1/PC3만 끊겼다면 `enp0s31f6` 링크, `192.168.9.0/24` 경로, DDS 환경만 본다.
3. 한 카메라만 끊겼다면 해당 카메라 NIC 하나만 진단한다.
4. 프로필을 바꾸기 전에 현재 활성 프로필, 주소, 라우트를 기록한다.
5. 활성 프로필 변경은 그 NIC의 연결을 순간적으로 끊는다. 원격 접속에 사용 중인 경로를 먼저
   대체하지 않은 채 바꾸지 않는다.
6. 임시 `ip addr`/`ip route` 명령보다 NetworkManager 프로필을 사용한다. 임시 명령은
   재부팅 후 사라져 상태를 재현하기 어렵다.

## 12. 하지 말아야 할 작업

- Wi-Fi 비밀번호를 명령행, 문서, Git, 채팅 로그에 평문으로 쓰지 않는다.
- `nmcli connection delete`, `ip addr flush`, `ip route flush`처럼 범위가 큰 삭제 명령을
  진단 목적으로 사용하지 않는다.
- 카메라 NIC와 `ROS2` 고정 프로필에 기본 게이트웨이/DNS를 넣지 않는다.
- 같은 IPv4 서브넷을 서로 다른 두 NIC에 동시에 붙이지 않는다.
- 인터넷이 Wi-Fi로 된다는 이유로 CycloneDDS NIC를 Wi-Fi로 바꾸지 않는다. PC1/PC3의 실제
  차량망은 계속 `enp0s31f6`이다.
- PC1/PC3와 카메라의 현재 IP를 추측해 카메라 persistent IP나 PC 프로필을 일괄 변경하지
  않는다.
- `run_autoware`가 실행 중인 상태에서 프로필, 주소, MTU, 케이블을 바꾸지 않는다.
- `ros2 topic list` 한 번의 짧은 결과만으로 통신 단절을 확정하지 않는다. 라우트, 핑, DDS
  환경, 지속 관찰을 함께 사용한다.

## 13. 현장 출발 전 최소 체크리스트

- [ ] 운용 모드를 A 또는 B 중 하나로 결정했다.
- [ ] 활성 프로필과 모든 NIC 주소를 기록했다.
- [ ] 인터넷 기본 경로가 의도한 NIC로 간다.
- [ ] PC1 `.2`, PC3 `.7` 경로가 `enp0s31f6`로 간다.
- [ ] PC1/PC3 핑과 Domain 10 ROS 2 발견이 지속적으로 통과한다.
- [ ] Windshield `.2` 경로가 `enp1s0f0`로 간다.
- [ ] Loop Top `.11` 경로가 `enp1s0f3`로 간다.
- [ ] 카메라 NIC와 고정 ROS 2 프로필에 기본 게이트웨이가 없다.
- [ ] CycloneDDS가 `enp0s31f6`에 고정되어 있다.
- [ ] 프로필 전환 후 ROS 2 CLI 데몬을 새 환경에서 다시 만들었다.
- [ ] 위 조건을 만족한 뒤에만 `run_autoware`를 실행한다.
