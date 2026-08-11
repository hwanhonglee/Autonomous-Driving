# PC1 command-path graph

Evidence: `source-derived` plus `runtime-verified-PC1-local-five-cycles`; physical and remote
endpoint evidence is incomplete

```text
/planning/scenario_planning/trajectory
  └─ /control/trajectory_follower/controller_node_exe       [control currently OFF]
     └─ /control/trajectory_follower/control_cmd             Control; ~33.3 Hz configured
        └─ /control/vehicle_cmd_gate                         [control currently OFF]
           └─ /control/command/control_cmd                   Control; 10 Hz configured
              ├─ remote DDS/bridge subscribers               UNKNOWN
              └─ /twistController2VCU2EPS2ACC_node           [bridge default OFF]
                 └─ /to_can_bus                              can_msgs/Frame ID 0x630; 30 ms
                    └─ /socket_can_sender_can0               [bridge default OFF]
                       └─ Linux SocketCAN can0                [DOWN at validation snapshot]
                          └─ Kvaser USBcan
                             └─ physical endpoint             UNKNOWN
```

The custom adapter initializes acceleration to `-1.0` and steering to `0.0`, then publishes CAN
ID `0x630` on its 30 ms timer even before a control command is received. Therefore merely being
disengaged is not a safety proof: the adapter and sender must not run.

In all five isolated PC1 Autoware cycles, the control command and CAN topics were absent, the
forbidden-node count was zero, and can0 through can3 remained DOWN. Current defenses are control
default OFF, bridge include and adapter default OFF, CAN-UP commands disabled in `start.sh`, and
the local alias guard. Direct/manual launch bypasses and all remote subscribers still require an
operator and simultaneous four-PC runtime audit.
