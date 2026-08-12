# PC2 Autoware Universe v2.0.0 migration bundle

Release baseline: 2026-08-12 KST.

This directory contains the source, operating procedures, and reproducible validation tools for
the PC2 sensing/perception role. Vendored ROS source is flattened into ordinary Git files. Generated
build/install/log trees, raw runtime logs, backups, and nested Git metadata are intentionally not
part of the snapshot.

Current authoritative documents:

- `reports/PC2_V2.0.0_RELEASE_NOTES.md` — release scope, evidence, and known limits.
- `reports/PC2_dual_camera_YOLO_TLR_integration.md` — detailed camera/YOLOX/TLR design and tests.
- `runbooks/PC2_network_and_port_setup.md` — wired, Wi-Fi, DDS, and camera NIC procedures.
- `scripts/run_pc2_autoware.sh` — guarded PC2 launch entry point.
- `scripts/validate_pc2_cycles.sh` — bounded lifecycle, payload, safety, and cleanup validator.
- `scripts/build_topic_tools_overlay.sh` — source-only bootstrap for the shutdown-safe relay overlay.

Validated locally:

- two Lucid cameras start from `run_autoware` in separate component processes;
- Windshield camera0 feeds generic YOLOX;
- Loop Top camera1 feeds the traffic-light-recognition components;
- RViz starts as part of the same launch;
- no PC2 control, vehicle-interface, or CAN-writer process starts;
- guarded shutdown removes PC2 processes, publishers, guard files, and the owned camera route.

Do not claim final TLR semantics or camera-LiDAR fusion until PC3 map/dynamic TF/LiDAR inputs and
camera-specific intrinsic/extrinsic calibration pass the acceptance gates in the release notes.
