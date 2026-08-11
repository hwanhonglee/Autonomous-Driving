# No-actuation path

Evidence: `source-derived` plus five isolated PC1 runtime cycles

The source-audited hazard path is:

```text
/control/command/control_cmd
  → twistController2VCU2EPS2ACC_node
  → /to_can_bus (CAN ID 0x630 every 30 ms)
  → socket_can_sender_can0
  → can0
  → Kvaser USBcan
  → unknown physical endpoint
```

The adapter can publish its initialized frame without first receiving a control command. In the
five isolated PC1 Autoware cycles, the adapter, SocketCAN, control command topic, and CAN topics
were absent, while can0 through can3 stayed DOWN. Remote DDS/bridge subscribers and physical
endpoints are still not verified. Local verdict: `PASS_PC1_ISOLATED`; final verdict:
`BLOCKED_PENDING_4PC_RUNTIME`.
