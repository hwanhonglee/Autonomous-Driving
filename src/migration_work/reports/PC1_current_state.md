# PC1 current state

Evidence: mixed. See `../inventory/PC1/current_state.md` for the host inventory,
`PC1_runtime_graph.md` for the local runtime graph, and `PC1_no_actuation.md` for the safety
verdict.

The fail-closed PC1 planning/API profile is locally runtime-verified across five clean start/stop
cycles. It is not yet input-ready: required PC3 map/localization/TF, a selected PC2-or-PC4 object
source, and operation-mode state are unavailable. The final trajectory endpoint exists but
produced no samples. Current `sample_vehicle` geometry is unverified.

Local status: `PASS_PC1_ISOLATED`. End-to-end status:
`BLOCKED_PENDING_PC2_PC3_PC4`.
