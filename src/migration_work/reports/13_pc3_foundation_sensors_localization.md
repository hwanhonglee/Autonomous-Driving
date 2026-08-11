# 13 — PC3 foundation, sensors, and localization

Evidence: `historical-source-derived` plus `runtime-observed-from-PC1-on-domain10`

Historical PC3 owns system, map, sensing, localization, and vehicle components, with separate
Hesai, NovAtel, and u-blox workspaces. Old Hesai IP `192.168.2.101`, UDP 2368, and old serial/IP
settings must not be applied without current hardware verification.

Target PC3 exclusively owns map, localization, `map → base_link`, common sensing, and physically
attached LiDAR/GNSS. A vehicle-status read path is optional only if it is separable from every
write path. Any actuator-capable CAN writer blocks the stationary test.

In the latest simultaneous Domain 10 window, PC3 published and delivered actual vector-map and
pointcloud-map samples. PC3 diagnostics also consumed PC1's decoded steering and velocity status
at approximately 50 Hz, and its converter emitted twist at approximately 50 Hz.

PC3 was not localization-ready: `/localization/kinematic_state` and
`/localization/acceleration` each had publisher count zero, `map → base_link` did not resolve,
pose fusion reported `NotReceived`, and obstacle segmentation reported `NotReceived`. GNSS and
LiDAR endpoints were discoverable but delivered no samples in bounded checks. No PC3 file was
edited from PC1, and PC3 alias/hardware settings still require local inspection.
