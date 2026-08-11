# Change comparison

Evidence: `source-derived`

Canonical comparisons use the verified same-directory backup as the left side:

```bash
diff -u 'launcher/autoware_launch/autoware_launch/launch/autoware.launch (copy_org).xml' \
  launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml

diff -u 'sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch (copy_org).xml' \
  sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch.xml

diff -u '/home/a/scripts/start (copy_org).sh' /home/a/scripts/start.sh
diff -u '/home/a/.bashrc (copy_org)' /home/a/.bashrc
```

The exact behavioral changes and rollback hashes are recorded in
`../HH_260810_CHANGELOG.md`. Git diff cannot be authoritative because usable Git metadata is
absent from the active workspace.
