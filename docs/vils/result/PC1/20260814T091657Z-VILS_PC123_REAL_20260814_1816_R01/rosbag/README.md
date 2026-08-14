# Rosbag payload

This directory contains one rosbag2 SQLite run:

- `metadata.yaml`
- `pc1_20260814_181555_0.db3.zst` through `_13.db3.zst`
- `BAG_INFO_EXTENDED_RECOVERY.txt`
- `DB_INTEGRITY.txt`
- compressed and raw SHA-256 manifests

The 14 raw SQLite files total 737,767,424 bytes. They are published as individually compressed Zstandard files totaling 142,131,008 bytes; the largest file is under 13 MB. The compressed files are stored through Git LFS under the repository result-data policy.

Verify and restore without deleting the compressed files:

```bash
git lfs pull --include='docs/vils/result/PC1/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/rosbag/*.db3.zst'
cd docs/vils/result/PC1/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/rosbag
sha256sum -c SHA256SUMS_COMPRESSED_DB3_ZST.txt
zstd -t -- *.db3.zst
zstd -d -k -- *.db3.zst
sha256sum -c SHA256SUMS_RAW_DB3.txt
```

Inspection:

```bash
ros2 bag info .
```

Do not replay this bag on a vehicle-connected network. If replay is required for analysis, use an isolated ROS domain, disable all DDS/domain bridges, keep physical CAN interfaces down, and ensure no SocketCAN sender or vehicle command adapter exists.
