# PC1 DDS and network environment

## 실행 환경

| 항목 | PC1 값 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-94-generic |
| ROS 2 | Humble |
| Domain ID | 10 |
| RMW | `rmw_cyclonedds_cpp` |
| CycloneDDS config | `migration_work/config/cyclonedds_pc1.xml` |
| 차량 PC망 | `enp0s31f6`, `192.168.9.2/24` |
| 카메라/NPU망 | `enp8s0`, `192.168.1.11/24` |
| PC3/NTP 후보 | `192.168.9.7` |
| PC2 | runtime에서 `192.168.9.110`으로 식별 |
| CAN | Kvaser USBcan, can0/can1 500 kbit/s |

## 하드웨어

- Neousys Nuvo-8108GC
- Intel Xeon E-2176G, 6 cores / 12 threads
- RAM 31 GiB
- NVIDIA GeForce RTX 3070 8 GiB
- NVIDIA driver 570.211.01
- CUDA toolkit 12.3
- TensorRT 8.6.1.6

## 문제 증상

PC1에는 192.168.1.0/24와 192.168.9.0/24 두 유선 NIC가 있다. CycloneDDS URI가 없을 때 middleware가 `enp8s0`/192.168.1.11을 임의로 선택했다. 이 상태에서 Domain ID 10이 맞아도 차량 PC망의 PC2/PC3 node와 topic이 0개로 보였다.

## 원인 판단

- 192.168.9.7과 192.168.9.110의 route는 차량망 NIC를 사용해야 한다.
- 원격 PC는 살아 있었지만 default DDS graph에서는 보이지 않았다.
- 임시로 `enp0s31f6`를 강제한 shell에서 즉시 PC3 map/sensing/system node가 발견됐다.
- `/map/vector_map`, `/map/pointcloud_map`의 transient-local publisher와 실제 message를 PC1에서 수신했다.

따라서 Domain/RMW mismatch가 아니라 다중 NIC 자동 선택이 직접 원인이었다.

## 수정

`cyclonedds_pc1.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enp0s31f6" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

shell environment:

```bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/a/autoware/src/migration_work/config/cyclonedds_pc1.xml
```

NetworkManager에서도 차량망 profile을 `enp0s31f6`, 카메라/NPU profile을 `enp8s0`에 고정했다. rollback용 profile은 삭제하지 않고 autoconnect를 끈 상태로 보존했다. 공개 문서에는 profile UUID와 MAC 주소를 싣지 않는다.

## 시간 동기화

2026-08-11 감사 시 Chrony selected source는 `192.168.9.7`이고 reach 377이었다. 이전 snapshot에서는 public NTP가 선택된 적이 있으므로 논문 데이터 수집 직전에 다음을 다시 기록해야 한다.

```bash
chronyc -n tracking
chronyc -n sources -v
```

세 PC가 같은 source를 사용한다는 사실, offset과 uncertainty가 실험 기준 이내라는 사실을 bag metadata와 함께 보존해야 한다.

## 재부팅 후 확인

```bash
ip -brief address
ip route get 192.168.9.7
ip route get 192.168.9.110
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI
chronyc -n sources -v
ros2 node list --no-daemon
```

acceptance:

- `enp0s31f6` = `192.168.9.2/24`
- `enp8s0` = `192.168.1.11/24`
- PC2/PC3 route source = 192.168.9.2
- Domain 10과 CycloneDDS가 모든 차량 PC에서 일치
- map/localization/perception publisher를 PC1에서 실제 sample로 확인
- graph endpoint 존재만으로 통신 PASS 처리하지 않음

## 배포 주의

NIC name은 PC1 하드웨어에 종속된다. 기존 307호 또는 새 308호의 다른 PC에 이 XML을 그대로 복사하기 전에 `ip -brief address`와 `udevadm`으로 실제 vehicle NIC를 확인해야 한다. 잘못된 NIC를 고정하면 다시 graph 0개 상태가 된다.
