# PC2 Autoware v1.1 file manifest

Hashes below identify the source/config inputs prepared from the tested PC2 working tree. Generated
install/build artifacts are not included.

## Launch and source changes

| Path under repository root | SHA-256 | Purpose |
|---|---|---|
| `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml` | `e953f171f31cf9819de20422ce4c298da98172981e50771329d99fe54b3d7da0` | PC2 ownership, container, fusion slot0, YOLOX and RViz |
| `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml` | `20939eab01156f79832b97a605e46d345a41755d614ca90a894dfe7a45273b8d` | expose and forward single-camera fusion contract |
| `src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/camera.launch.xml` | `79034d0c4b66fcb31cf3f3a07717e239cf247e628265380173b40b312ce23409` | Lucid include and normalized relays |
| `src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/sensing.launch.xml` | `3aeb90bb0ec9889592c427e79bb7d36b7fa0bcb26e9e99c146a56cc370fd60e7` | camera-only PC2 sensing include |
| `src/universe/autoware.universe/launch/tier4_perception_launch/launch/object_recognition/detection/detection.launch.xml` | `ae0201b59d6adc4c51852addc2a68e6f497a831052edaeae1590429c38a62850` | remove LiDAR merger argument typo |
| `src/universe/autoware.universe/perception/autoware_lidar_centerpoint/lib/network/tensorrt_wrapper.cpp` | `9e355d4ea43aab57785ef9a3835c85b00ffe4458fe01be303ba347f8375fa9a4` | correct TensorRT destruction order |
| `src/universe/autoware.universe/perception/autoware_tensorrt_yolox/src/tensorrt_yolox.cpp` | `d47347d003dd39cd994625e5d37e2268da2b7aee3e8ef2f2cf06152d2247b3f6` | clamp both ROI corners |

## Operator and validation inputs

| Path | SHA-256 |
|---|---|
| `src/migration_work/scripts/run_pc2_autoware.sh` | `bf3ee023c841502b0c125e203eb270c1c24df14696578a9f788dac0dd71d14e1` |
| `src/migration_work/scripts/validate_pc2_cycles.sh` | `35cdb268c2c79da05cfad89fa2cf383f3e6147759e272f612b21874d7e847405` |
| `src/migration_work/scripts/probe_camera_contract.py` | `c5c92080f6165e5729621969df0e785a01909588cf69324c50d96a106c968154` |
| `src/migration_work/scripts/probe_yolox_contract.py` | `543f4f4255345da884d16259dfb1793f6fe9bdfc261db6b2f23f155c4c92d7c1` |
| `src/migration_work/patches/topic_tools-1.1.2-relay-shutdown.patch` | `3981a0f4cbb7aef6d43f2bbe1054c208c3547004f2f5ec03e7675d9fbd029dae` |

`build_topic_tools_overlay.sh` and the documentation are new release-engineering files; their
hashes should be taken from the final tagged commit rather than this pre-commit manifest.

## Complete-source additions

The later complete-source snapshot adds every current editable package path, the full flattened
topic_tools 1.1.2 source, the Eagleye source marker, current PC2 package configuration/reference
files, and the resolved CalibrationTools PNG/PDF assets. The exhaustive package-path inventory and
scope rules are in `PC2_V1.1_COMPLETE_SOURCE_SNAPSHOT.md` and
`inventory/PC2_V1.1_PACKAGE_PATHS.txt`; those two files supersede this focused active-file table for
repository-completeness checks.
