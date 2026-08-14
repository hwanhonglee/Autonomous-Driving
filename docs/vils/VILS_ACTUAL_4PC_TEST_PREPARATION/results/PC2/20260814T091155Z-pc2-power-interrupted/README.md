# PC2 partial field evidence: power interruption

Run ID: `20260814T091155Z-pc2-power-interrupted`

Status: **PARTIAL — NOT AN ACCEPTANCE RUN**

This directory contains only bounded PC2-side observations captured before vehicle power was lost.
It is not a completed four-PC, synchronized, PG-VILS, Stage 2, Stage 3, or Stage 4 result.

What was observed:

- PC1 vehicle status, trajectory, and control-command payloads reached PC2.
- PC3 localization reached PC2, while the legacy LiDAR relay was degraded.
- PC2 produced canonical predicted-object messages and bounded Windshield YOLOX output.
- TLR judged/final transport produced fresh empty arrays; semantic signal recognition was not shown.
- PC4 was ICMP reachable in a preceding check, but both expected VILS publishers were absent.

What is missing:

- no common four-host run manifest or same-window four-host Chrony set;
- no coordinated rosbag, pcap, or PC4 object/diagnostic payload;
- no accepted map/transform/source contract;
- no clean shutdown marker for the final launch;
- no moving-vehicle or closed-loop PG-VILS acceptance.

Two raw ROS launch logs remain outside Git. Their paths, sizes, and SHA-256 values are recorded in
`manifest/hashes.sha256`.
