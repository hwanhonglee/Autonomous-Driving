# 14 — PC4 simulation and bridge

Evidence: `runtime-unverified`

No dedicated PC4 source branch or live bridge configuration was identified in the audited set.
PC4 must own exactly one virtual-domain source, one bridge instance, and the TX logger. Its bridge
must use an explicit allowlist and must not forward any control command to an actuator-capable
vehicle endpoint.

Virtual-object mode requires PC2's official object publisher to be suppressed and PC4's single
publisher ready before PC1's object-input gate can pass. Recorded replay requires the bridge and
logger before replay; live CARLA order must be documented from the actual implementation.

PC4 Domain IDs, RMW, bridge directions, topic remaps, QoS, rates, logger path, and Chrony state are
blocking unknowns.
