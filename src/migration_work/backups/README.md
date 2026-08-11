# Verified backups

Evidence: `runtime-verified-local`

The mandatory non-overwriting backups were created beside their active files and verified by
SHA-256, owner, mode, size, and timestamp before editing:

- `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch (copy_org).xml`
- `/home/a/autoware/src/sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch (copy_org).xml`
- `/home/a/scripts/start (copy_org).sh`
- `/home/a/.bashrc (copy_org)`

Hashes are in `../inventory/PC1/baseline_sha256.txt`. These backups must never be overwritten.

Safety warning: the two XML backups are inside package `launch` directories and may become
launchable after a rebuild. The `start (copy_org).sh` backup retains mode 777 and contains the
old CAN-UP commands. They are rollback artifacts only and must not be launched during a
stationary test.
