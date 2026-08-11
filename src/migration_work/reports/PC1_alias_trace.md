# PC1 alias trace

Evidence: `runtime-verified-local` for the local shell definition

## Before

`run_autoware` was a direct alias with no guard:

```bash
alias run_autoware='ros2 launch autoware_launch autoware.launch.xml map_path:=$HOME/Downloads/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit'
```

The shell sources ROS Humble and `/home/a/autoware/install/setup.bash`. The installed active
launch is a symlink to:

`/home/a/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`

## After

The alias name is unchanged:

```bash
alias run_autoware='/home/a/autoware/src/migration_work/scripts/run_autoware_safe.sh'
```

The helper rejects extra launch arguments, holds an exact `flock`, scans `/proc` for duplicate
Autoware/planning-simulator/CAN/bridge processes, requires `can0` through `can3` to be DOWN, and
then ends with the same launch target and parameters using `exec`:

```bash
exec ros2 launch autoware_launch autoware.launch.xml \
  map_path:="$HOME/Downloads/sample-map-planning" \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

`run_autoware --check-only` runs the local checks without launching anything. A new terminal or
`source ~/.bashrc` is required for an already-open shell to pick up the changed alias.

Separate convenience alias names remain but now fail immediately in the stationary workflow:

- `run_planning_universe`: blocked because it duplicates planning/control/API and historically sets simulation engage true.
- `run_bridge`: blocked because its historical target owns an actuator-capable CAN sender and adapter.

The underlying CAN launch also remains default OFF as a second defense. Direct child launch/run
commands remain bypasses. These helpers are local defense layers, not the PC2/PC3/PC4 readiness
or end-to-end no-actuation proof.
