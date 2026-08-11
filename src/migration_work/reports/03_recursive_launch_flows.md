# 03 — Recursive launch flows

Evidence: `source-derived` plus `runtime-verified-PC1-local-five-cycles`

See [PC1_launch_flow.md](PC1_launch_flow.md) for the active tree and
[PC1_command_path_graph.md](PC1_command_path_graph.md) for the deferred control path.

The top-level active groups are global parameters, planning, control, Autoware API, and RViz.
After the change, default evaluation keeps global parameters, planning, and API; control, RViz,
parking, and the RViz API adaptors are false. No vehicle/map/sensing/perception/localization/system
include exists in this PC1 top file even though legacy arguments remain for interface
compatibility. The local evaluated tree produced 47 nodes with duplicate FQNs 0 in each of five
cycles.

The separate planning simulator recursively includes the same Autoware launch and adds dummy
simulation modules. It must never run beside `run_autoware`.
