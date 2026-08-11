# PC1 local validation — HH_260810

Evidence: `runtime-verified-local` for the listed checks only

| Check | Result |
|---|---|
| Four original-to-`copy_org` SHA-256 comparisons before editing | PASS |
| Backup owner/mode/size/timestamp comparisons | PASS |
| `xmllint --noout` for both edited XML launch files | PASS |
| `bash -n` for `start.sh`, `.bashrc`, and `run_autoware_safe.sh` | PASS |
| `run_autoware_safe.sh --check-only` | PASS; no launch started |
| Guard while the exact run lock is already held | PASS; returned BLOCKED |
| Guard with a harmless process using `cansend` as argv[0] | PASS; returned BLOCKED; test process stopped by exact PID |
| `run_planning_universe` and `run_bridge` stationary aliases | PASS; both return BLOCKED without launching |
| `ros2 launch autoware_launch autoware.launch.xml --show-args` | PASS |
| Autoware default args: planning/API true; control/non-PC1/RViz false | PASS |
| `ros2 launch ros2_socketcan can_brdige.launch.xml --show-args` | PASS |
| `enable_physical_can_bridge` default false | PASS |
| Installed active Autoware launch resolves to edited source symlink | PASS |
| Isolated PC1 planning/API runtime graph | PASS; 47 nodes, duplicate FQNs 0, forbidden nodes 0 |
| AD API web server | PASS after Ubuntu Python 3.10 path correction; `/adapi/web_server` listening on localhost:8888 |
| Planning trajectory endpoint | PARTIAL; one publisher from `/planning/planning_validator`, but no samples without required upstream inputs |
| Required PC3/PC2-or-PC4 inputs | BLOCKED; `/map/vector_map` pub 0/sub 8, objects pub 0/sub 6, operation-mode state pub 0/sub 6 |
| Fail-closed command topics during PC1 cycles | PASS; control command and CAN topics absent |
| One real Autoware start and clean Ctrl+C | PASS_LOCAL |
| Five start/stop cycles | PASS_LOCAL; 47 nodes each, clean shutdown, post process/node/lock 0 |
| Four-PC integration and ten-minute stationary run | NOT RUN — readiness blocked |

`--show-args` parses included launch descriptions and does not prove that disabled actions are
safe when explicitly re-enabled. The local runtime PASS does not prove remote bridge direction,
physical endpoints, QoS compatibility, data freshness, TF validity, or current vehicle geometry.
