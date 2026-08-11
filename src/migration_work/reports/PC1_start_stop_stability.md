# PC1 start/stop stability

Evidence: `runtime-verified-local-five-cycles`

Scope: isolated PC1 fail-closed planning/API profile. These cycles validate local process
lifecycle and the absence of the disabled PC1 control/CAN path. They do not validate PC2, PC3,
PC4, cross-PC QoS/TF/data flow, or an end-to-end no-actuation path.

| Cycle | Launch PID/PGID | Nodes before/during/after | Duplicate FQNs | Forbidden nodes | RSS during | Physical CAN | Ctrl+C | Post-stop process/node/lock | Verdict | ROS log directory |
|---:|---:|---|---:|---:|---:|---|---|---|---|---|
| 1 | 28528 | 0 / 47 / 0 | 0 | 0 | 570932 KiB | can0-can3 DOWN | clean, approximately 0.45 s | 0 / 0 / 0 | PASS_LOCAL | `/home/a/.ros/log/2026-08-10-20-45-19-979204-a-28528` |
| 2 | 28999 | 0 / 47 / 0 | 0 | 0 | 570468 KiB | can0-can3 DOWN | clean, approximately 0.45 s | 0 / 0 / 0 | PASS_LOCAL | `/home/a/.ros/log/2026-08-10-20-46-15-140036-a-28999` |
| 3 | 29482 | 0 / 47 / 0 | 0 | 0 | 569676 KiB | can0-can3 DOWN | clean, approximately 0.45 s | 0 / 0 / 0 | PASS_LOCAL | `/home/a/.ros/log/2026-08-10-20-48-09-550517-a-29482` |
| 4 | 29941 | 0 / 47 / 0 | 0 | 0 | 574268 KiB | can0-can3 DOWN | clean, approximately 0.45 s | 0 / 0 / 0 | PASS_LOCAL | `/home/a/.ros/log/2026-08-10-20-48-57-574017-a-29941` |
| 5 | 30420 | 0 / 47 / 0 | 0 | 0 | 571208 KiB | can0-can3 DOWN | clean, approximately 0.45 s | 0 / 0 / 0 | PASS_LOCAL | `/home/a/.ros/log/2026-08-10-20-49-48-311189-a-30420` |

`Forbidden nodes` covers the control namespace, SocketCAN nodes, the custom control-to-CAN
adapter, the parking container, and the disabled RViz initial-pose/routing adaptors. The stable
cycles also kept `/control/command/control_cmd`, `/to_can_bus`, and `/from_can_bus` absent.

## Debugging runs excluded from the five-cycle result

| Run | ROS log directory | Observation | Correction before stable cycles |
|---|---|---|---|
| D1 | `/home/a/.ros/log/2026-08-10-20-39-30-303320-a-25514` | The AD API web server inherited Anaconda Python 3.13 and could not load the ROS Humble Python 3.10 `rclpy` extension. Shutdown also exposed unused RViz-adaptor and parking-load races. | The safe launcher now prioritizes Ubuntu `/usr/bin` Python. |
| D2 | `/home/a/.ros/log/2026-08-10-20-41-34-390531-a-26921` | The web server started successfully on localhost:8888; the unused initial-pose/routing adaptors and the parking container still raced during shutdown. | Disabled the RViz API adaptors in the lightweight PC1 profile. |
| D3 | `/home/a/.ros/log/2026-08-10-20-43-47-096785-a-27860` | RViz-adaptor abort was gone; the parking container still had a pending component-load shutdown race while PC3 pose/map inputs were absent. | Disabled the parking module in the stationary PC1 profile. |

Local lifecycle result: `PASS_PC1_ISOLATED`. Integration result remains
`BLOCKED_PENDING_PC2_PC3_PC4`.
