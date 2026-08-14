# PC1 VILS acquisition metadata

- Recorder start: 2026-08-14 18:15:55 KST
- Host role: PC1 planning, control, vehicle-status and CAN evidence
- Output: `/home/a/vils_data/pc1_20260814_181555`
- Storage: ROS 2 sqlite3, split every 60 seconds, no compression
- Recorder process at start: PID 15242
- ROS domain: 10
- RMW: `rmw_cyclonedds_cpp`
- Vehicle DDS NIC: `enp0s31f6`, `192.168.9.2/24`
- Wi-Fi: SSID redacted for publication, `172.20.10.9/28`
- Autoware branch: `agent/vils-pc1-integration`
- Autoware commit: `e495c0c39a5a1dc349204693e584488e8575f535`
- CycloneDDS config SHA-256: `0e4f30b76e868f5477dc8252736d66502466efbd5fd6bb7d1cde8cddbd49f880`
- Last verified PC1 clock state before acquisition: Chrony synchronized to public NTP, not an accepted common four-PC time source. Cross-PC one-way latency claims therefore remain unapproved.

## Recorded topic groups

- Raw and commanded CAN: `/from_can_bus`, `/to_can_bus`
- Cruise/brake/local velocity state: `/current/*` selected topics
- Vehicle reports: velocity, steering, gear, control mode, indicators, hazards, battery
- Control commands: control, emergency, gear, indicators and hazards
- Planning: mission route and final trajectory
- Localization: kinematic state, acceleration, `/tf`, `/tf_static`
- Perception: tracked/final object topics attributed by the configured architecture to PC2, and the expected PC4-namespaced virtual tracked-object topic; the PC1 SQLite bag does not authenticate remote-host ownership
- Safety/API: hazard status, MRM state, localization/operation/routing states
- Expected VILS diagnostics/candidate/status topics are listed in the recorder command and will be included if they appear during the session.

High-bandwidth LiDAR/pointcloud data is intentionally not pulled across the network into this PC1 bag; it must be acquired locally on PC3.

## Live event alignment

- 18:14:49-18:14:52 KST: control-request activation occurred before recorder start and is not in this bag.
- 18:14:53 KST: second sustained activation began before recorder start.
- 18:15:55 KST: recorder started while Cruise=true, vehicle control mode AUTONOMOUS, Autoware control enabled, and ROS/physical 0x630 request byte=1; vehicle speed was 0.0 m/s.
- 18:17:18 KST: Cruise, vehicle mode, Autoware control, ROS request and physical 0x630 request all returned safe together. This bag contains roughly the final 83 seconds of the sustained activation and its recovery transition.
- Topic endpoints attributed by the configured architecture to PC2 and PC3 had repeated ROS/DDS visibility/process flaps before acquisition. At recorder start those core endpoints were visible; PC1 did not independently authenticate their host ownership.
- PC4 VILS object/gateway endpoints were not visible at recorder start.
- At 18:18:36 KST two `/rosbag2_recorder` node names were visible across Domain 10. The second recorder may be a distributed peer; its host ownership was not proven from PC1.
- 18:20:53 KST: PC2 tracker and canonical PredictedObjects publishers dropped from one to zero while recording. PC3 endpoints remained present and PC4 provided no fallback. Treat the interval from this transition until a later verified recovery as missing canonical perception.
- 18:21:31 KST: PC2 tracker and canonical publishers recovered after about 35 seconds with new DDS GIDs, consistent with another participant/process restart. The canonical-perception gap above ends at this timestamp, subject to payload-freshness validation.
- 18:21:31-18:21:32 KST: a new sustained activation began: Cruise=true, vehicle AUTONOMOUS, ROS and physical 0x630 request byte=1, and Autoware control enabled. Vehicle longitudinal speed was near zero and PC4 VILS input was still absent. Record the eventual recovery timestamp separately.
- 18:22:21 KST: PC4 shadow `TrackedObjects` endpoint appeared on Domain 10. A bounded 18:23:38-18:23:53 live sample observed 148 messages (9.933 Hz), frame `map`, fresh stamps, but every message in that early probe contained zero objects. PC4 adapter diagnostics and PC2 VILS candidate/accepted paths remained absent. PC2 tracked/final streams were about 10.18 Hz with 8-14 objects during the same probe.
- Offline bag analysis later found PC4 non-empty messages from 18:24:23.331 through 18:28:14.337 KST, including a stable CAR object for about 218.7 seconds in the conservative DB0-DB12 window. The early live empty-scene probe must not be generalized to the rest of the recording.
- 18:23:44-18:26:55 KST and continuing at the last check: the control-request activation remained sustained. Vehicle speed was 7.54 m/s at 18:25:44 and 8.25 m/s at 18:26:21 while Cruise/request/control were active. Because no PC2 accepted-status/candidate evidence or canonical PC4 match was recorded, this remains observational data rather than proof of virtual-obstacle response.
- 18:25:15 KST: four exact-FQN `/rosbag2_recorder` nodes were visible on Domain 10, consistent with possible distributed acquisition, but remote host ownership was not provable from PC1.
- 18:27:05 KST onward: PC1 Chrony tracking retained reference ID `192.168.9.7` (PC3), local stratum 4, last offset -0.131732 ms, RMS offset 0.299243 ms, and root dispersion beginning at 18.330470 ms. The captured source blocks marked PC3 `^-`, never `^*`, and its reference time was stale, so this is not proof that PC3 was the live selected source. Full tracking/sources/sourcestats blocks are in `PC1_CHRONY.log`.
- Remote `chronyc -h` queries to PC2, PC3 and PC4 were unavailable from PC1; each remote host must retain its own Chrony evidence for four-host clock claims.
- 18:29:42.159 KST: last recoverable ROS message. Vehicle/PC power loss interrupted rosbag2 and the metrics logger; the final approximately 14.7 seconds never reached persistent SQLite pages.

## Interpretation limits

- A publisher endpoint alone is not proof of fresh or semantically valid data.
- PC4 absence and prior PC2/PC3 flaps prevent declaring a valid four-PC VILS acceptance window until all required streams are simultaneously fresh and stable.
- MANUAL/Cruise/request state must be read from the recorded vehicle/CAN signals for every claimed interval.
