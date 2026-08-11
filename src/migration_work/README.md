# IONIQ EV 308 PC2 v1.1 migration record

This directory is the authoritative PC2 migration record for the deployment tagged
`IONIQ_EV_308_PC2_a`. It was derived from the historical IONIQ EV 307 PC2 Autoware v1.0
branch and validated on the current PC2 hardware on 2026-08-10 and 2026-08-11 KST.

Start here:

- [`reports/PC2_V1.1_RELEASE_NOTES.md`](reports/PC2_V1.1_RELEASE_NOTES.md): scope, causes,
  changes, architecture, limitations, and operator procedure.
- [`reports/PC2_V1.1_TEST_REPORT.md`](reports/PC2_V1.1_TEST_REPORT.md): build and runtime
  evidence with precise PASS boundaries.
- [`inventory/PC2_V1.1_ENVIRONMENT.md`](inventory/PC2_V1.1_ENVIRONMENT.md): machine, GPU,
  ROS, camera, and network inventory.
- [`reports/PC2_V1.1_FILE_MANIFEST.md`](reports/PC2_V1.1_FILE_MANIFEST.md): committed file
  purpose and SHA-256 manifest.
- [`reports/PC2_V1.1_COMPLETE_SOURCE_SNAPSHOT.md`](reports/PC2_V1.1_COMPLETE_SOURCE_SNAPSHOT.md):
  flattened Git/submodule audit, complete package accounting, and source exclusions.
- [`inventory/PC2_V1.1_PACKAGE_PATHS.txt`](inventory/PC2_V1.1_PACKAGE_PATHS.txt): every package
  manifest path included in the Autoware release tree.
- [`scripts/run_pc2_autoware.sh`](scripts/run_pc2_autoware.sh): the guarded operator entry
  point behind the `run_autoware` alias.
- [`scripts/validate_pc2_cycles.sh`](scripts/validate_pc2_cycles.sh): bounded PC2 lifecycle
  and data-contract validator.
- [`scripts/build_topic_tools_overlay.sh`](scripts/build_topic_tools_overlay.sh): reproducible
  user-space relay overlay build.

All editable source packages are included as ordinary files, including the flattened topic_tools
overlay source. Historical reports and generated test artifacts from the working machine were not
copied wholesale. Several early reports describe a camera-blocked, RViz-off, LiDAR-only state and
are superseded by the v1.1 documents above. Build trees, extracted binary-package copies, JPEGs,
rosbag files, bulk ROS logs, backups, and Python caches remain excluded from source control.
