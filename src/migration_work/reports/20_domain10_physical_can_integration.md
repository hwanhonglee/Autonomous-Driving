# 20 — Domain 10 physical-CAN receive integration

Evidence: `runtime-verified-simultaneous-PC1-PC2-PC3`; status: `PARTIAL_PASS_BLOCKED_ON_PC3_LOCALIZATION`

This is the latest accepted runtime path. PC1 used the real `run_autoware` alias, not the planning
simulator. The observation window simultaneously contained PC1 planning/control/API, the PC1
physical-CAN receive-only bridge, PC2 perception, and PC3 foundation nodes on ROS Domain 10.
PC4 was not present, so this is not a completed four-PC test.

## PC1 launch and physical CAN

- Active `autoware.launch.xml`: planning, control, and Autoware API ON; vehicle interface, map,
  localization, sensing, perception, system, system monitor, and RViz OFF.
- `/home/a/scripts/start.sh` configured physical `can0` and `can1` UP at 500 kbit/s. `can2` and
  `can3` remained DOWN, and the temporary `vcan0` interface was deleted.
- `run_bridge` used physical `can0` with receiver ON, SocketCAN sender OFF, and the
  control-to-CAN adapter OFF.
- Both active physical links were `ERROR-ACTIVE`. During the measured window their receive
  counters rose into the hundreds of thousands with zero receive errors/drops and zero transmit
  packets. Raw receive traffic was approximately 1.7 kHz.

| Decoded topic | Observed value | Measured rate |
|---|---:|---:|
| `/vehicle/status/gear_status` | report `22` | approximately 100 Hz |
| `/vehicle/status/steering_status` | steering `0` | approximately 50 Hz |
| `/vehicle/status/velocity_status` | longitudinal velocity `0` | approximately 50 Hz |
| `/vehicle/status/control_mode` | mode `4` | approximately 50 Hz |

PC3 diagnostics received the velocity and steering status streams at approximately 50 Hz, and
the vehicle-status converter produced its twist output at approximately 50 Hz. This proves the
measured PC1 CAN RX → ROS Domain 10 → PC3 consumer direction. It does not prove a CAN TX path.

## Cross-PC data observed on PC1

| Source | Observation | Result |
|---|---|---|
| PC3 map | `/map/vector_map` and `/map/pointcloud_map` each had one publisher and delivered actual samples | PASS for these two inputs |
| PC2 perception | `/perception/object_recognition/objects` came from `map_based_prediction` at approximately 9.09 Hz; observed messages used frame `map` and contained zero objects | PASS for transport; scene content was empty |
| PC3 localization | `/localization/kinematic_state` and `/localization/acceleration` each had publisher count 0 | BLOCK |
| PC3 TF | `map → base_link` did not resolve | BLOCK |
| PC3 diagnostics | pose fusion and obstacle segmentation reported `NotReceived` | BLOCK |
| PC3 GNSS/LiDAR | endpoints were discoverable, but bounded sample checks returned no data | BLOCK until fresh samples are proven |

The simultaneous graph contained 174 fully qualified nodes and zero duplicate FQNs. This is a
same-window PC1+PC2+PC3 measurement, not proof of functional-ownership uniqueness under renamed
nodes and not a four-PC count.

## Actual `run_autoware` goal/control result

Before the goal request, the queried trajectory and control endpoints each exposed one publisher,
but bounded eight-second observations received no samples. The goal request through the actual
Autoware routing API failed with `status=false`, code `1`, and message `route already set`; the
routing state remained `UNKNOWN`. Source inspection explains that the API reports every state
other than `UNSET` through that message path. With PC3 odometry absent and `map → base_link`
unresolved, the request was not accepted. Repeating the goal did not produce trajectory or control
samples.

This is an upstream-readiness failure, not evidence that a goal must be sent to make a healthy
control pipeline work. Fix PC3 localization, acceleration, and TF first; then clear/reset routing
state through the supported API and retry without engage or autonomous-mode calls.

## Write-path assessment for this window

The only local SocketCAN process was the receiver. No SocketCAN sender, control adapter, or
vehicle interface ran; physical CAN TX stayed at zero. Therefore
`PC1_PHYSICAL_CAN_RECEIVE_ONLY: PASS` for the measured window. Final end-to-end no-actuation remains
blocked until remote bridge directions, physical actuator endpoints, cabling, and engage history
are audited in the same window.

## Excluded diagnostic run

A planning simulator was mistakenly started briefly in isolated Domain 110 and was stopped. With
a test occupancy grid it reached route state `SET`, emitted a 167-point trajectory, and produced
an internal control sample with velocity `0 m/s` and acceleration `-3.4 m/s²`. This is useful only
as a diagnostic showing the local planning/control chain can compute with synthetic prerequisites.
It is explicitly `NON_ACCEPTANCE_EVIDENCE` and must not be substituted for the actual
`run_autoware` result above.
