# 17 — Unresolved blockers

Evidence: mixed

The following block cross-PC bring-up and planning-data readiness:

1. PC2/PC3/PC4 aliases, active launch paths, processes, hardware, Domain IDs, RMW, NIC policy, and clocks are unverified.
2. The complete publisher/subscriber graph for `/control/command/control_cmd` and every bridge direction is unverified.
3. Current CAN cabling and the Kvaser physical endpoints are unknown.
4. `vehicle_model:=sample_vehicle` geometry is not approved for the current vehicle.
5. PC3 map/localization/TF freshness and PC2/PC4 official-object readiness are unavailable.
6. Real PC2 versus virtual PC4 source mode has not been selected; the XOR publisher rule is not measured.
7. Live topic types, QoS compatibility, measured rates, publisher counts, and transform broadcasters are unverified.
8. PC4 bridge configuration and remote actuator-capable endpoints are unknown.
9. Chrony selects a public source on PC1; the four-PC time topology is not accepted.
10. Git branch/commit/dirty provenance is unprovable in the active workspace.
11. Control OFF removes PC1's operation-mode-state publisher; the isolated graph measured publisher 0/subscriber 6, and the trajectory endpoint produced no samples.
12. Launchable/executable backup and legacy files remain manual bypasses and must not be run.

The isolated fail-closed PC1 profile completed five clean runtime cycles. Because the end-to-end
no-actuation and required-input blockers remain unresolved, coordinated PC2/PC3/PC4 integration,
control enablement, and the ten-minute stationary integration test remain blocked.
