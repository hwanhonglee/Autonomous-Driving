# PC1 no-actuation assessment

Evidence: mixed

## Verdict

- `LOCAL_ISOLATED_RUNTIME_MITIGATION: PASS`
- `END_TO_END_NO_ACTUATION: BLOCKED_PENDING_4PC_RUNTIME`

Local isolated-runtime mitigation passes because all five PC1 planning/API cycles suppressed the
whole control stack, physical SocketCAN, and the control-to-CAN adapter. The control command and
CAN topics were absent, forbidden-node count was zero, and can0 through can3 remained DOWN. Every
cycle also ended with zero remaining process, ROS node, or lock.

This is not final no-actuation proof. PASS requires all of the following in the same test window:

- a human confirms the vehicle is stationary and secured;
- engage and autonomous-mode transition calls are never made;
- no physical actuator interface subscribes to any command topic;
- no local or remote bridge forwards a command to an actuator-capable endpoint;
- no SocketCAN/CAN writer process exists and all physical CAN links remain DOWN;
- the PC2/PC3/PC4 publisher/subscriber graph and bridge allowlist are captured;
- current physical cabling and endpoints are identified.

Known manual bypasses include explicit `launch_control:=true`, explicit
`enable_physical_can_bridge:=true`, direct child launch/run commands, CAN utilities, legacy launch
copies, and the executable `start (copy_org).sh` rollback artifact. These are forbidden by the
operator runbook but are not a hardware interlock.
