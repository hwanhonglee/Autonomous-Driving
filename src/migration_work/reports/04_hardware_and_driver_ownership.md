# 04 — Hardware and driver ownership

Evidence: mixed; live PC1 only

| Resource | Target owner | Live evidence | Decision |
|---|---|---|---|
| PC1 Kvaser USBcan Light 4xHS | none for write in stationary mode | attached; can0..can3 DOWN at snapshot | all writer paths OFF |
| PC1 generic UVC cameras | not assigned | two devices observed | do not start as Autoware sensor drivers without need/ownership proof |
| Lucid traffic-light camera | PC2 if physically attached | historical PC2 source only | inspect PC2 before enabling or removing |
| Hesai LiDAR and GNSS | PC3 if physically attached | historical PC3 source only | verify new hardware/IP/serial; do not reuse old settings |
| map/localization/TF | PC3 | source policy only | runtime proof required |
| virtual source and bridge | PC4 | no audited local implementation | blocked pending PC4 inspection |

PC1 must not own raw sensor, vehicle-interface, or CAN-writer processes. A read-only vehicle status
source may remain on PC3 only when read/write separation is independently verified.
