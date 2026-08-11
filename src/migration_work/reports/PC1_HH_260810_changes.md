# PC1 HH_260810 changes

Evidence: `source-derived` plus local static and five-cycle runtime validation

- Verified four non-overwriting `copy_org` backups before editing.
- Changed non-PC1 module, vehicle interface, system monitor, control, RViz, and RViz respawn defaults to false.
- Kept planning and Autoware API defaults true.
- Added an explicit default-false physical CAN bridge gate to both the SocketCAN include and custom adapter.
- Disabled `can0/can1` bitrate and UP commands in `/home/a/scripts/start.sh`.
- Preserved the `run_autoware` alias name and active filename while inserting an exact lock/process/CAN-state guard.
- Created a separate non-overwriting `run_autoware_safe (copy_org).sh` backup before correcting the launcher environment.
- Prioritized Ubuntu Python 3.10 in the safe launcher so ROS Humble Python nodes do not inherit the active Anaconda Python 3.13 interpreter.
- Disabled unused RViz API adaptors and the parking module in the stationary PC1 profile after their missing-input shutdown races were reproduced.
- Completed five isolated PC1 start/stop cycles: 47 nodes each, duplicate FQNs 0, forbidden nodes 0, and post-stop process/node/lock 0.
- Added inventory, ownership, topic, readiness, no-actuation, validation, operator, and rollback artifacts.

Source diff/hashes and rollback details are in `../HH_260810_CHANGELOG.md`; the five-cycle runtime
evidence and debugging-run separation are in `PC1_start_stop_stability.md`.
