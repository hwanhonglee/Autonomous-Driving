# PC1 launch include tree

Evidence: `source-derived` plus `runtime-verified-local-five-cycles`. See
[PC1_launch_flow.md](PC1_launch_flow.md). The active top-level includes global parameters,
planning, deferred control, Autoware API, and optional/default-off RViz. The stationary profile
also disables parking and the RViz API adaptors. Non-PC1 role arguments do not correspond to
active includes in this top file. The evaluated tree stabilized at 47 nodes in all five runs.
