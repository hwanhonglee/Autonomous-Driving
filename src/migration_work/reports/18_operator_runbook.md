# 18 — Stationary operator runbook

Evidence: procedure; not yet executed

## Global safety gate

1. A human secures the stationary vehicle and records actuator power/cabling state.
2. Select exactly one mode: `real_pc2` or `virtual_pc4`.
3. Never call engage, autonomous mode, `/api/operation_mode/change_to_autonomous`, or any control-enable service.
4. Verify every PC's Domain ID, RMW, NIC, multicast/firewall policy, and accepted Chrony source.
5. On PC1 run `run_autoware --check-only`; independently record `can0` through `can3` DOWN and no SocketCAN, custom CAN, bridge, planning-simulator, or stale component process.
6. Audit the live command subscriber/bridge graph. Any physical or remote actuator-capable endpoint is an immediate BLOCK.

## Real-object mode

1. Start PC3; require one vector map, fresh localization and acceleration, fresh `map → base_link`, nonfatal diagnostics, and no writer.
2. Start PC2; require its inputs, one perception stack, and exactly one predicted-object publisher with matching type/QoS.
3. Start PC1 with `run_autoware`; expect planning/API only and zero control/CAN nodes. Capture nodes, components, topics, QoS, rates, TF, processes, and resource use.
4. Start the one PC4 bridge/logger only after its allowlist and forbidden directions are reviewed; verify selected PC1→PC4 communication.

## Virtual-object mode

1. Start and pass PC3.
2. Start PC2 with its official predicted-object publisher suppressed.
3. Start PC4's single bridge, TX/RX loggers, and single virtual source readiness path. For replay, start logger/bridge before replay and coordinate actual data publication with PC1 readiness.
4. Only after the official object-input gate is satisfied, start PC1 and capture the same evidence.
5. Start remaining PC4 metrics after PC1 is visible.

## Stop and stability

Stop the exact launch terminal with Ctrl+C. Wait for its children, verify nodes and containers
disappear, and never use broad `pkill`, `killall`, or `/dev/shm` deletion. After one clean pass,
perform five recorded cycles and then a ten-minute stationary run.

Forbidden during this workflow: `run_bridge`, `/home/a/scripts/start.sh`,
`run_planning_universe`, direct CAN sender/converter utilities, explicit control/CAN opt-ins, and
execution of any `(copy_org)` or legacy launch.
