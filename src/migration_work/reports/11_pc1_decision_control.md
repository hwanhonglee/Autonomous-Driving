# 11 — PC1 decision, planning, control, and API

Evidence: `source-derived`, `runtime-verified-local-five-cycles`, and `runtime-verified-simultaneous-PC1-PC2-PC3`

Planning, internal control computation, and Autoware API are now enabled by default in the actual
`run_autoware` path. Vehicle interface, CAN sender, control-to-CAN adapter, RViz, unused RViz API
adaptors, and the parking module remain OFF. Internal control is therefore present without a local
physical write path.

The isolated profile produced 47 nodes in each of five runs with zero duplicate FQNs and no
control, SocketCAN, adapter, parking, or RViz-adaptor nodes. `/adapi/web_server` ran on
localhost:8888. Ctrl+C was clean in approximately 0.45 seconds per cycle with no remaining
process, ROS node, or lock.

The latest Domain 10 graph was partly input-ready. PC3 vector and pointcloud maps delivered
samples, and PC2 objects arrived from `map_based_prediction` at approximately 9.09 Hz. PC3
kinematic state and acceleration still had zero publishers, and `map → base_link` was unresolved.
Consequently the actual goal request failed with `route already set` while route state remained
`UNKNOWN`; trajectory and control endpoints had one publisher each but no samples in bounded
eight-second checks.

The receive-only `run_bridge` path was run beside Autoware only after `run_autoware` started. It
launched one physical CAN receiver and no sender or adapter; CAN TX remained zero. Do not enable
the sender, adapter, or vehicle interface. Vehicle geometry for `sample_vehicle` also remains
unverified.
