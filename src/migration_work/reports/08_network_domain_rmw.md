# 08 — Network, Domain ID, and RMW

Evidence: `runtime-verified-PC1` plus `runtime-observed-PC2-PC3-discovery`

PC1 uses ROS Humble, Domain ID 10, CycloneDDS, and non-localhost-only discovery. The active
`CYCLONEDDS_URI` points to `../config/cyclonedds_pc1.xml`, which pins CycloneDDS to
`enp0s31f6` at `192.168.9.2`. PC2/PC3 nodes and data were observed on that Domain 10 interface.
The other PC1 interface is `enp8s0` at `192.168.1.11/24`.

The explicit interface policy resolves the earlier wrong-interface selection observed during the
audit. UFW configuration reported disabled, but complete PC2/PC3/PC4 route, firewall, multicast,
and clock evidence has not been collected locally.

PARTIAL PASS: PC1 discovered and exchanged data with PC2 and PC3 on Domain 10. BLOCKED for PC4 and
for a per-host Domain/RMW/NIC/firewall/clock audit. Do not change sysctl, NIC, firewall, or kernel
settings without a measured baseline and rollback.
