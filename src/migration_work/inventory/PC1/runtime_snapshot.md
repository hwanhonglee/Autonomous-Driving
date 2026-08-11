# PC1 runtime snapshot

Evidence: `runtime-verified-local-snapshot`

The read-only Phase A snapshot and post-edit local guard check found no Autoware application
process, no SocketCAN sender/receiver, no custom control-to-CAN process, and no component
container. `can0`, `can1`, `can2`, and `can3` were all `DOWN` when
`run_autoware_safe.sh --check-only` passed.

The ROS daemon observed temporarily during the audit was an audit side effect and was stopped.
No PC2, PC3, or PC4 runtime state was captured from this host. This snapshot is not an
end-to-end no-actuation proof and expires as soon as processes, links, cabling, or the remote
graph change.
