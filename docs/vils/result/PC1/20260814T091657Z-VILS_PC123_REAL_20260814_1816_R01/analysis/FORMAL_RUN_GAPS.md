# Formal-run gap record

This result is a retrospective, power-loss-recovered communication and behavior
dataset. It is not the completed formal VILS scenario defined by the preparation
runbook.

The following required formal-run evidence was absent, incomplete, or not frozen
before acquisition:

- a pre-run PC1 manifest containing the shared run ID, scenario, condition, random
  seed, boot identity, Git dirty state, installed executable/configuration hashes,
  and operator approvals;
- the approved route, actor schedule, numeric acceptance thresholds, abort record,
  and one continuous graph/QoS/GID ownership manifest;
- a four-host clock record proving the same selected source and bounded offset,
  jitter, dispersion, holdover, and absence of clock steps;
- PC2 validator/candidate/accepted-source status and PC4 adapter diagnostic records;
- a complete CAN-FD/socket ownership inventory and service/action call audit for the
  recorded interval;
- a graceful recorder/metrics shutdown; vehicle power loss truncated the final
  SQLite database and logger tails; and
- the contract-preferred MCAP storage format. PC1 recorded split rosbag2 SQLite3 at
  `/home/a/vils_data/pc1_20260814_181555` and recovered DB13 into a separately
  validated publication copy.

The shared ID is justified retrospectively by serialized-payload linkage to PC3,
documented in [`../RUN_LINKAGE.md`](../RUN_LINKAGE.md). This linkage identifies a
common acquisition run; it does not repair the missing formal gates or establish
cross-host one-way latency.
