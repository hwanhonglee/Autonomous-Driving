# 02 — Active launch paths

Evidence: `runtime-verified-local`

| Purpose | Active path | Verified backup | Active post-change SHA-256 |
|---|---|---|---|
| PC1 Autoware | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | same directory `autoware.launch (copy_org).xml` | `d7ecc96f...` |
| PC1 SocketCAN convenience launch | `/home/a/autoware/src/sensor_component/ros2_socketcan/ros2_socketcan/launch/can_brdige.launch.xml` | same directory `can_brdige.launch (copy_org).xml` | `a9f431ba...` |
| CAN setup helper | `/home/a/scripts/start.sh` | `/home/a/scripts/start (copy_org).sh` | `189a73e9...` |
| shell aliases | `/home/a/.bashrc` | `/home/a/.bashrc (copy_org)` | `155b83b2...` |

`/home/a/autoware/install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml`
resolves to the edited source file. Thus the launch change is active without a rebuild. The
backup launch XML files may also be installed by a later rebuild and are forbidden execution paths.
