# 05 — Node ownership matrix

Evidence: `source-derived`; runtime enforcement incomplete

| Function | PC1 | PC2 | PC3 | PC4 |
|---|---|---|---|---|
| Planning | owner | forbidden | forbidden | forbidden |
| Control computation | deferred owner | forbidden | forbidden | forbidden |
| Autoware API | owner | forbidden | forbidden | forbidden |
| Perception | forbidden | owner | forbidden | virtual source only |
| Map/localization/TF/system | forbidden | forbidden | owner | forbidden |
| Common sensing and physical LiDAR/GNSS | forbidden | conditional camera only | owner if attached | forbidden |
| Physical vehicle/CAN writer | forbidden | forbidden | forbidden | forbidden |
| Domain bridge | forbidden | forbidden | forbidden | single owner |
| RViz | optional and default OFF | forbidden | forbidden | forbidden |

Machine-readable policy: `../config/node_ownership.yaml`.
