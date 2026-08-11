# PC1 duplicate analysis

Evidence: `source-derived` plus `runtime-verified-local-five-cycles`; cross-PC counts unverified

| Risk | Mitigation | Current status |
|---|---|---|
| A second `run_autoware` | exact lock plus process scan | PASS locally; five cycles had duplicate FQNs 0 and post-stop lock 0 |
| `run_planning_universe` beside `run_autoware` | stationary alias is blocked; helper also detects an existing simulator | locally mitigated; direct launch bypass remains |
| `run_bridge` or direct SocketCAN launch | stationary alias blocked; bridge default OFF; helper process/CAN checks | locally mitigated; direct/remote path unknown |
| RViz on multiple PCs | PC1 default OFF; at most one manual PC1 instance | cross-PC unverified |
| PC2 and PC4 both publishing predicted objects | XOR contract | blocked pending source-mode selection |
| PC2 and PC3 both owning common sensing | node ownership contract | runtime-unverified |
| Duplicate static TF | PC3 owns each transform | runtime-unverified |
| Duplicate bridge | PC4 single owner | runtime-unverified |
| PC3 wrongly assumed to publish operation mode | contract records PC1 control as source owner | control intentionally absent |

Node renaming is not accepted as a duplicate fix. PC1's isolated count does not clear the
cross-PC risks. The integration run must record node FQNs, component containers, publisher
counts, QoS, and transform broadcasters on all four PCs in the same time window.
