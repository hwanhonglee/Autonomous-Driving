# PC1 metrics and Chrony audit

## Coverage

The metrics logger began at about 18:27:05 KST, well after the rosbag recorder. Valid system rows cover 18:27:05.559–18:29:42.217 (156.658 s); GPU rows continue to 18:29:59.158. Power loss truncated the last SYSTEM, PING, CHRONY, and PROCESS blocks. Only complete rows/blocks are included below.

The CSV timestamps use a comma as the fractional separator without quoting, so generic CSV readers see one extra column. Reconstruct the timestamp by joining the first two fields before numerical analysis.

## Chrony

- 23/23 tracking blocks reported reference ID `C0A80907` (`192.168.9.7`, PC3), stratum 4, Leap Normal.
- In 22 complete source blocks, PC3 was always `^-`, never the selected `^*` source.
- PC3 reference time remained frozen at 09:13:21 UTC and was already stale by roughly 14–16 minutes during capture.
- Last offset: -0.131732 ms; RMS offset: 0.299243 ms.
- Root dispersion: 18.330–18.623 ms and increasing.
- PC3 source estimate: mean about -3.660 ms with stated uncertainty about ±23 ms.
- No PC2, PC3, or PC4 local `chronyc` evidence was captured.

Conclusion: the sidecar proves a stale last-known PC3 reference, not live four-PC common time. Header-to-bag age is useful for local ordering and coarse diagnostics but must not be presented as one-way latency or sub-millisecond cross-host synchronization.

## Reachability RTT

All complete ping attempts returned successfully:

| Peer | Samples | Mean | Median | p95 | Max |
|---|---:|---:|---:|---:|---:|
| PC3 `192.168.9.7` | 24 | 1.293 ms | 0.563 ms | 4.957 ms | 6.680 ms |
| PC2 `192.168.9.110` | 24 | 4.415 ms | 4.075 ms | 8.554 ms | 9.210 ms |
| PC4 `192.168.9.12` | 23 | 0.250 ms | 0.211 ms | 0.599 ms | 0.690 ms |

These are PC1 round-trip reachability samples, not message PDR or one-way latency.
Peer labels follow the acquisition configuration; the PC1 bag does not cryptographically authenticate remote-host identity.

## Host and interface metrics

- 12 logical CPUs; load1 mean 16.25, range 11.77–19.34.
- available memory: 25.276–25.482 GiB.
- recorder directory growth during the valid system window: about 119.40 MiB (0.762 MiB/s).
- vehicle NIC `enp0s31f6`: 4,993,579 received packets, 387,484 transmitted packets, zero RX/TX errors; RX drop counter increased by 13,600 (about 0.272% of delivered plus dropped packets).
- CAN0: 278,193 received packets and 5,221 transmitted packets in the valid window; 1,775.8 RX/s and 33.327 TX/s; no RX/TX error, drop, restart, or bus-off increase. The absolute pre-existing CAN RX-drop counter is not an acquisition-window loss.

## GPU and processes

- RTX 3070 utilization: mean 46.32%, range 40–55%.
- GPU memory: mean 486.8 MiB, maximum 500 MiB of 8,192 MiB.
- temperature: 53–54°C; power: mean 80.03 W, range 78.60–82.24 W.
- average observed CPU: rosbag recorder 112%, RViz 68.0%, CAN receiver 10.09%, bridge 4.36%, sender 3.79%, converter 4.49%, Autoware launch parent 3.36%.

The filtered process log proves the selected processes were present in 21 complete snapshots; it does not prove every Autoware child remained healthy. Apparent omissions in the last snapshot are caused by tail truncation.
