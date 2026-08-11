# 01 — Alias traces

Evidence: `runtime-verified-local` for PC1; other PCs unverified

PC1 details are in [PC1_alias_trace.md](PC1_alias_trace.md).

- `run_autoware`: preserved name; now targets `migration_work/scripts/run_autoware_safe.sh`.
- `run_planning_universe`: alias name preserved but blocked during the stationary test; the underlying direct simulator path remains a manual bypass.
- `run_bridge`: alias name preserved but blocked; the underlying SocketCAN launch is also fail-closed by default.

No equivalent live alias traces have been collected from PC2, PC3, or PC4. They remain blocking
readiness evidence.
