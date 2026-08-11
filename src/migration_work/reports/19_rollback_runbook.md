# 19 — Rollback runbook

Evidence: procedure; not executed

Rollback is per file, not all-or-nothing. First stop only the exact PC1 launch process group,
verify no relevant process remains, and save any post-change evidence. Confirm each absolute
source/backup target and compare backup SHA-256 with `../inventory/PC1/baseline_sha256.txt`.

Restore only the selected file using the verified backup:

```bash
cp --preserve=all \
  '/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch (copy_org).xml' \
  '/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml'

cp --preserve=all \
  '/home/a/autoware/src/sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch (copy_org).xml' \
  '/home/a/autoware/src/sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch.xml'

cp --preserve=all '/home/a/scripts/start (copy_org).sh' '/home/a/scripts/start.sh'
cp --preserve=all '/home/a/.bashrc (copy_org)' '/home/a/.bashrc'
```

After each restore, verify the active hash equals the recorded original, rerun XML/shell parsing,
and open a new shell after `.bashrc` restoration. Never modify the backup itself.

Warning: restoring the old CAN launch or `start.sh` reintroduces unconditional physical CAN behavior.
Do not launch the restored system for a stationary test until a new no-actuation review passes.
Git rollback is unavailable because the active workspace lacks usable Git metadata.
