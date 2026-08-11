# 07 — Four-PC topic contract

Evidence: `source-derived`; measured rates and live endpoints are `TBD_RUNTIME`

The canonical table is `../4pc_topic_contract.csv`; PC1's focused table is
`PC1_input_output_contract.csv`.

Critical rules:

- PC3 exclusively owns vector map, localization, and `map → base_link` TF.
- PC2 real perception XOR PC4 virtual source owns exactly one
  `/perception/object_recognition/objects` publisher per run.
- PC1 owns `/planning/scenario_planning/trajectory`; `/planning/trajectory` is not the current output.
- PC1 control owns operation-mode and internal control topics only after no-actuation approval.
- `/control/command/control_cmd` must have no actuator-capable subscriber or bridge in stationary mode.
- Candidate PC1→PC4 API/route/status topics are not approved until the PC4 bridge configuration and live graph agree.

The two virtual-traffic-light names in current source must be reconciled before that feature is enabled.
