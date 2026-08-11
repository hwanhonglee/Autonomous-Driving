# 16 — Four-PC integration

Evidence: `runtime-verified-simultaneous-PC1-PC2-PC3`; status `PARTIAL_PASS`, four-PC completion `BLOCKED`

A simultaneous Domain 10 PC1+PC2+PC3 window was completed with the actual `run_autoware` path,
PC1 physical-CAN receive-only bridge, PC2 perception, and PC3 foundation nodes. It contained 174
node FQNs with duplicate FQNs zero. PC3 vector and pointcloud maps delivered samples; PC2 predicted
objects arrived at approximately 9.09 Hz with an empty object list; and PC1 decoded physical CAN
status was consumed on PC3 at approximately 50 Hz.

This was not a four-PC completion: PC4 was absent. PC3 also supplied no kinematic-state or
acceleration publisher, `map → base_link` did not resolve, and the actual `run_autoware` goal
request failed before trajectory/control data became available. Full evidence is in
[20_domain10_physical_can_integration.md](20_domain10_physical_can_integration.md).

Remaining exit criteria are:

- readiness order appropriate to real or virtual object mode;
- exclusive ownership violations zero;
- duplicate critical publishers and static TF conflicts zero;
- one approved inter-PC bridge instance and one official object source;
- accepted Chrony topology;
- PC4 TX and PC1 RX logging with matched type/QoS/rate;
- no command path to a physical actuator; a receive-only PC1 CAN window with TX zero has passed,
  but remote/physical endpoint audit remains;
- one clean coordinated run, five coordinated cycles, then a ten-minute stationary run.

Results must be captured from the same time window; combining stale per-PC snapshots is not sufficient.
