# Rollback entry point

Use `../reports/19_rollback_runbook.md`. Verify every source and destination by absolute path
and compare the backup SHA-256 against `../inventory/PC1/baseline_sha256.txt` before copying.

Restoring the old SocketCAN launch or old `start.sh` reintroduces actuator-capable behavior.
Do not execute restored files as part of the stationary test.
