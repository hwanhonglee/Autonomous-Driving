# 21 — PC1 vehicle-LAN recovery and live input recheck

Evidence: `runtime-verified-2026-08-11`; status: `PC1_READY_LOCALIZATION_LIVE_STABILITY_BLOCKED`

> HH_260811 - Public-release copy: physical NIC MAC addresses and NetworkManager connection UUIDs
> are replaced with descriptive placeholders. Private ROS/vehicle-network IP addresses are retained
> because they are required to reproduce the PC1 network topology.

This report records the live PC1 recovery and recheck on ROS Domain 10. PC1 used the real
`run_autoware` alias and the physical-CAN receive-only `run_bridge`; the planning simulator was
not used.

## Vehicle-LAN recovery

At the start of the run, NetworkManager had selected the two unbound Ethernet profiles on the
wrong interfaces:

- `Profile 1` (`192.168.1.11/24`) was active on `enp0s31f6`;
- `RLRC` was attempting DHCP on `enp8s0` and had no IPv4 address;
- the intended Domain 10 interface therefore had no `192.168.9.2` address.

Before changing the profiles, non-autoconnecting backups were created:

| Backup connection | UUID |
|---|---|
| `RLRC HH_260811 copy_org` | `<REDACTED_NETWORKMANAGER_UUID_RLRC_BACKUP>` |
| `Profile 1 HH_260811 copy_org` | `<REDACTED_NETWORKMANAGER_UUID_PROFILE1_BACKUP>` |

The active originals were then bound by interface name and MAC address. After explicit
activation, the verified mapping was:

| Network | Profile | Interface | MAC | Address |
|---|---|---|---|---|
| vehicle-PC ROS network | `RLRC` | `enp0s31f6` | `<REDACTED_PC1_VEHICLE_LAN_MAC>` | `192.168.9.2/24` from DHCP |
| camera/NPU network | `Profile 1` | `enp8s0` | `<REDACTED_PC1_CAMERA_NPU_LAN_MAC>` | `192.168.1.11/24` |

The persistent CycloneDDS configuration already selects `enp0s31f6`, so no DDS file change was
required. Processes started before the live interface reassignment retained the obsolete DDS
address; they were stopped with Ctrl+C and restarted after the network mapping was corrected.

## Current simultaneous state

- PC1 `run_autoware` parent: PID `12372`, started 12:26:59 KST.
- PC1 receive-only `run_bridge` parent: PID `12791`; receiver PID `12854`.
- `can0` and `can1`: UP at 500 kbit/s; `can2` and `can3`: DOWN.
- `can0`: `ERROR-ACTIVE`, zero bus errors, zero transmitted packets.
- No SocketCAN sender, control-to-CAN adapter, vehicle interface, or CAN utility process ran.
- Domain 10 recheck: 179 fully-qualified nodes and zero duplicate FQNs while PC1, PC2, and PC3
  roles were simultaneously visible.
- Reachable peer addresses: `192.168.9.7` and `192.168.9.110`. The previously assumed
  `192.168.9.11` was not the live second peer address.

## Verified live inputs on PC1

| Input | Runtime result |
|---|---|
| `/map/vector_map` | one publisher; PC1 mission planner advanced from `waiting lanelet map` to `waiting odometry` |
| `/map/pointcloud_map` | one publisher in the simultaneous graph |
| `/sensing/gnss/selected/fix` | fresh valid-fix sample, `status=2`, frame `gnss_link` |
| `/sensing/gnss/novatel/oem7/fix` | fresh valid-fix sample |
| `/sensing/gnss/novatel/oem7/imu/data` | fresh IMU sample, frame `imu_link` |
| `/sensing/imu/imu_data` | fresh corrected IMU sample, frame `base_link` |
| `/sensing/lidar/top/pointcloud_before_sync` | fresh Hesai pointcloud sample, frame `hesai_top` |
| `/sensing/lidar/top/aw_points` | fresh processed pointcloud sample, frame `hesai_top` |
| `/perception/object_recognition/objects` | one official PC2 publisher; fresh `map`-frame sample with an empty object array |
| `/pc1/can/from_can_bus` | approximately 1.9 kHz during the measured window |
| `/vehicle/status/gear_status` | approximately 100 Hz |
| `/vehicle/status/velocity_status` | fresh zero-velocity sample, frame `base_link` |

An empty object array proves transport and publisher ownership only; it is not a perception-quality
acceptance result.

## Localization state at the first recheck

The following checks failed in the same window:

- `/localization/kinematic_state`: publisher count 0 and no bounded sample;
- `/localization/acceleration`: publisher count 0 and no bounded sample;
- `map -> base_link`: two disconnected TF trees;
- `/sensing/gnss/pose`: endpoint present but no sample in eight seconds;
- no functional localization node was present; only the localization topic-state monitor appeared;
- diagnostics reported NovAtel orientation invalid: `ins_status=12`, not
  `SOLUTION_GOOD(3)`, `valid_orientation=false`, azimuth RMSE `180 deg`;
- `/localization/pose_twist_fusion_filter/pose` and the map-to-base-link monitor reported
  `NotReceived`/Error.

Consequently route state, final trajectory, internal controller command, and gated command emitted
no samples in bounded checks. No goal or engage/autonomous request was sent.

## Localization late-start update

PC3 localization subsequently appeared in the same Domain 10 session. The live graph contained
NDT scan matcher, EKF localizer, gyro odometer, pose initializer, twist-to-acceleration, stop
filter, pose-instability detector, and localization error monitor nodes. The following contracts
then passed:

- `/localization/kinematic_state`: one publisher and fresh data at approximately 50 Hz;
- `/localization/acceleration`: one publisher and fresh data at approximately 50 Hz;
- `/localization/pose_twist_fusion_filter/pose`: one publisher and fresh data;
- `/sensing/gnss/pose`: fresh `map`-frame data;
- continuous `map -> base_link` TF;
- `/localization/initialization_state`: state `3`;
- `/api/routing/state`: state `1` (`UNSET`).

Localization availability therefore passed. Stability did not yet pass: during a 9.982-second
stationary observation, 500 odometry messages moved from `(3.262, -142.832, 11.691)` to
`(2.493, -141.549, 11.329)`, a 3D displacement of `1.539 m`, while both odometry and decoded CAN
longitudinal velocity remained exactly `0.0 m/s`. Treat this as startup convergence or pose drift
until a longer stable window and NDT diagnostics prove otherwise. Do not set the route yet.

## Next action

PC1 should remain in this receive-only integration state. PC3 localization is now enabled, so the
next gate is stability rather than availability. Continue only after all of the following remain
simultaneously fresh and the stationary pose stays within the accepted tolerance:

1. `/sensing/gnss/pose`;
2. `/localization/kinematic_state`;
3. `/localization/acceleration`;
4. continuous `map -> base_link` TF.

Also recheck NovAtel orientation/INS diagnostics and NDT convergence. After those gates pass,
recheck the route state and select a goal inside the current real map.
Do not use the old sample-map coordinates and do not engage or switch to autonomous mode during
the stationary communication test.
