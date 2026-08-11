# 00 — Source audit

Evidence: `source-derived` unless marked otherwise

The historical `hwanhonglee/Autonomous-Driving` audit supports PC1
`autoware_universe/PC1_rocket`, PC2 `PC2_nebula`, and PC3 `PC3_spectra`. Matching `ros2_ws`
branches were found for PC2 and PC3 but not PC1; no dedicated PC4 branch was identified.

Historical ownership agrees with the target split:

- PC1: planning, control, Autoware API, optional RViz.
- PC2: perception and a possible physically attached Lucid traffic-light camera.
- PC3: map, system, sensing, localization, LiDAR/GNSS, TF, and possible read-only vehicle state.
- PC4: no audited implementation; bridge and virtual-source configuration must come from the live host.

Local PC1 inspection supersedes historical assumptions for its active path. It found an
actuator-capable, separately launched SocketCAN bridge; a planning-simulator duplicate path; and
an unverified `sample_vehicle` geometry. Historical IPs, serials, and old sensor settings were not
applied.

Source evidence does not prove current PC2/PC3/PC4 hardware, aliases, Domain IDs, RMW, bridge
directions, Chrony topology, CAN cabling, QoS, publisher counts, or physical actuator endpoints.
