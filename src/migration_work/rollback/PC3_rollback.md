# PC3 HH_260810 rollback

This is an operator runbook; no rollback was executed during the audit.

## Important safety warning

The immediate pre-change PC3 launch defaults can start sensor hardware, the actuator-capable vehicle interface, MRM command publishers, a second pointcloud container, localization, and RViz. A full source rollback therefore restores known unsafe or duplicate-prone behavior. Do not run the unchanged `run_autoware` alias after a full rollback until the vehicle is independently immobilized and an approved safety override is in place.

The old Nebula wrapper also has a proven joinable-thread shutdown defect. Rolling it back intentionally restores the prior SIGABRT behavior.

## Authoritative restore source

Use the dated immediate pre-change snapshot:

`/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810`

Do not substitute an older historical `(copy_org)` when its checksum differs from the immediate pre-change active file. Existing `(copy_org)` files must remain untouched.

Before restoration:

1. Stop the exact launch with one scoped `Ctrl+C`.
2. Verify there is no remaining `ros2 launch`, component container, Hesai, or OEM7 process.
3. Verify the snapshot: from its directory run `sha256sum -c --quiet SHA256SUMS`; require exit code 0.
4. Record the current final manifest result from `/home/a/autoware/src`: `sha256sum -c --quiet migration_work/backups/PC3/post_HH_260810/SHA256SUMS`.

## Full source/config restore

The following commands overwrite only the nine explicitly named active targets. They do not alter any `(copy_org)` file:

```bash
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml' '/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml' '/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/components/tier4_system_component.launch.xml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml' '/home/a/autoware/src/universe/autoware.universe/launch/tier4_system_launch/launch/system.launch.xml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml' '/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/lidar.launch.xml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml' '/home/a/autoware/src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml' '/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp' '/home/a/autoware/src/universe/autoware.universe/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp' '/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/include/nebula_ros/hesai/hesai_ros_wrapper.hpp'
cp --preserve=mode,timestamps '/home/a/autoware/src/migration_work/backups/PC3/pre_HH_260810/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp' '/home/a/autoware/src/sensor_component/external/nebula/nebula_ros/src/hesai/hesai_ros_wrapper.cpp'
```

## Rebuild after restore

```bash
cd /home/a/autoware
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select autoware_launch tier4_system_launch sample_sensor_kit_launch autoware_gnss_poser nebula_ros
```

Require a successful build and XML/YAML parse. Do not use a live driver start as a rollback test.

## OEM7 restoration rollback

The OEM7 source restoration is additive and remains hardware-gated, so the safe default is to leave the byte-identical restored source and packages in place. Disabling sensing hardware prevents OEM7 startup.

If an exact Hesai-only workspace is later required, move—not delete—the following explicit source/build/install directories into a dated quarantine after a separate operator approval:

- `/home/a/ros2_ws/src/novatel_oem7_driver`
- `/home/a/ros2_ws/build/novatel_oem7_driver`
- `/home/a/ros2_ws/build/novatel_oem7_msgs`
- `/home/a/ros2_ws/install/novatel_oem7_driver`
- `/home/a/ros2_ws/install/novatel_oem7_msgs`

Keep `/home/a/ros2_ws_original.zip` and its audited SHA256. Never test the rollback by opening the NovAtel serial device; OEM7 startup changes the receiver's active LOG configuration.

## Post-rollback verification

- Confirm the active hashes match the corresponding pre-change manifest entries.
- Confirm the `run_autoware` alias text and active filename are unchanged.
- Perform launch parsing first.
- Treat a runtime smoke test as blocked until safe explicit overrides account for the restored hardware, vehicle-interface, MRM, and old LiDAR gate behavior.
- Record which subset was rolled back and retain both pre- and post-change manifests.
