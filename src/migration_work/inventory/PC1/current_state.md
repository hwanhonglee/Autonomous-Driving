# PC1 current state

Evidence: mixed (`runtime-verified-local` and `runtime-unverified-4pc`)

- Host: Neousys Nuvo-8108GC; Ubuntu 22.04.5; kernel 6.8.0-94.
- CPU/RAM: Xeon E-2176G, 6 cores/12 threads; 31 GiB RAM; no swap.
- Storage: approximately 468 GiB NVMe root filesystem; 24% used at audit time.
- GPU: RTX 3070 8 GiB; driver 570.211.01; driver CUDA API 12.8; installed toolkit 12.3; TensorRT 8.6.1.6.
- ROS: Humble; `ROS_DOMAIN_ID=10`; `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; `ROS_LOCALHOST_ONLY=0`.
- NICs: `enp8s0=192.168.1.11/24`, `enp0s31f6=192.168.9.2/24`; both 1 Gb/s full duplex and MTU 1500 at audit time.
- CycloneDDS selected `enp8s0` in the observed daemon session; the intended 4-PC interface is not yet approved.
- CAN: Kvaser USBcan Light 4xHS attached; `can0` through `can3` existed and were DOWN at guard validation.
- Sensors: two generic UVC cameras observed; no local Lucid/Arena package or serial GNSS/LiDAR device observed.
- Workspace: `/home/a/autoware` with 379 source/build/install packages. Installed Autoware launch is a symlink to the active source file.
- Provenance: useful Git metadata is absent/empty, so branch, commit, and dirty state are unprovable.
- Runtime: no Autoware, SocketCAN writer, custom control-to-CAN adapter, or component container was active at the validation snapshot.
- 4-PC graph, remote hardware, bridge configuration, QoS compatibility, TF freshness, and publisher ownership remain runtime-unverified.
