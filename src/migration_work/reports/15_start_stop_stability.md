# 15 — Start/stop stability

Evidence: `runtime-verified-local-five-cycles`

The isolated PC1 fail-closed planning/API profile completed five start/clean-Ctrl+C cycles. Every
cycle produced 47 nodes, zero duplicate FQNs, zero forbidden control/CAN/parking/RViz-adaptor
nodes, and zero process/node/lock remnants after shutdown. Physical can0 through can3 remained
DOWN. Detailed measurements and log directories are in
[PC1_start_stop_stability.md](PC1_start_stop_stability.md).

Verdict: `PASS_PC1_ISOLATED`; `BLOCKED_PENDING_PC2_PC3_PC4` for coordinated multi-PC cycles and
the ten-minute stationary integration run.
