# PC1 runtime graph

Evidence: `runtime-verified-local-five-cycles`

The isolated fail-closed PC1 profile stabilized at 47 nodes in each of five runs. Duplicate fully
qualified node names were zero. Control, SocketCAN, the custom control-to-CAN adapter, parking,
and the unused RViz API adaptors were absent.

| Endpoint or node | Runtime observation | Interpretation |
|---|---|---|
| `/planning/scenario_planning/trajectory` | type `autoware_planning_msgs/msg/Trajectory`; publisher count 1 from `/planning/planning_validator`; no sample received | Output endpoint exists, but planning is not data-ready without PC3 and the selected PC2/PC4 source. |
| `/map/vector_map` | publisher count 0; subscriber count 8 | Required PC3 map input missing in this isolated run. |
| `/perception/object_recognition/objects` | publisher count 0; subscriber count 6 | Required PC2-or-PC4 object source missing in this isolated run. |
| `/system/operation_mode/state` | publisher count 0; subscriber count 6 | Expected while the PC1 control/operation-mode owner is disabled. |
| `/control/command/control_cmd` | topic absent | Fail-closed local control output remained absent. |
| `/to_can_bus`, `/from_can_bus` | topics absent | No local CAN bridge was part of these Autoware cycles. |
| `/adapi/web_server` | node present; web service listening on localhost:8888 | API web server works with Ubuntu Python 3.10 selected ahead of Anaconda Python 3.13. |

Five-cycle invariants:

- node count: 47;
- duplicate FQNs: 0;
- forbidden nodes: 0;
- can0 through can3: DOWN;
- post-Ctrl+C process/node/lock counts: 0/0/0.

The graph proves local launch composition and lifecycle only. It does not prove cross-PC topic
types/QoS/rates, TF freshness, exclusive remote ownership, bridge direction, or physical
no-actuation. Integration status: `BLOCKED_PENDING_PC2_PC3_PC4`.
