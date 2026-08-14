# PC2 VILS object integration

<!-- HH_260810 - State the prototype boundary explicitly so this package is not mistaken for a deployed safety feature. -->

This package is the PC2 fail-closed acceptance boundary for PC4 virtual
`TrackedObjects`. It validates source identity, metadata, timing, map/transform
identity, and payload bounds before a physical-main fusion candidate can be
published. It is a Stage 3 prototype: the installed configuration contains no
approved deployment thresholds, and no distributed or vehicle-driving PASS is
claimed by this package.

## Source modes and gates

<!-- HH_260810 - Separate observation, canonical selection, and required-operation authority. -->

| Mode | VILS node | Output role | Required launch authority |
|---|---:|---|---|
| `real_only` | No | Existing physical pipeline only | None |
| `shadow` | Yes | Non-canonical candidate/debug only | None; an unapproved contract accepts nothing |
| `hybrid_optional` | Blocked | Dormant canonical physical-main seam only | Stage 3 acceptance + READY/MRM supervision not implemented |
| `hybrid_required` | Blocked | Not launchable in this prototype | PC1/PC3 MRM availability path is not implemented |
| `virtual_only_required` | Blocked | Not launchable in this prototype | PC1/PC3 MRM availability path is not implemented |

`real_only` does not construct the integration process. This prototype rejects
every canonical mode before process creation because the required Stage 3
evidence, READY supervision, and MRM availability path do not exist. The
contract and canonical-selection gates are retained as dormant Stage 4 wiring;
they do not override the prohibition or approve values in a parameter file.

## Data ownership

<!-- HH_260810 - Preserve the documented single-writer and single-predictor topology. -->

PC3 remains the owner of the legacy LiDAR relay. PC2 remains the perception
owner. PC4 publishes raw virtual objects on
`/perception/pc4/virtual_obstacles/tracked_objects` plus matching diagnostic
metadata on `/diagnostics/pc4/object_adapter`. In shadow mode, PC2 publishes
only `/perception/pc2/vils/candidate_tracked_objects`. The existing physical
tracker and the existing single map-based predictor remain unchanged.

For a later canonical mode, the physical tracker output must be moved to a
private PC2 physical-object topic and this node must be the only canonical
tracked-object writer. That promotion is outside the installed default and
must not occur before the documented Stage 3 evidence gates pass.

## Fail-closed contract

<!-- HH_260810 - List the values that require owner review rather than supplying plausible-looking guesses. -->

The default file at `config/vils_object_integration.param.yaml` has
`contract_approved: false`, `__UNAPPROVED__` identity fields, and zero timing,
content, covariance, and association bounds. Shadow mode may start with this
file for graph/evidence inspection, but it rejects all PC4 snapshots.

Before approval, owners must record at least:

- the PC4 object and diagnostic publisher FQNs and writer GIDs;
- exact hashing contract and diagnostic key schema; the current prototype hashes
  its bounded receiver-side ROS reserialization, not proven original wire CDR bytes;
- PC3/PC4 map and transform digests;
- measured TTL, join timeout, source-age, clock-skew, fusion-skew, and payload bounds;
- association threshold and classification rule;
- whether the named scenario permits a fresh, explicit zero-actor snapshot;
- append-only PC2 provenance-log destination;
- four-host time synchronization and the manual/PARK/stationary arming procedure.
- measured linear-velocity and yaw-rate standstill thresholds.

Changing the source GID, session, map, transform, mode, or reconnect state
clears accepted virtual state and requires a new manual arm request. Missing,
stale, replayed, future-dated, malformed, or ambiguous input is never treated
as a valid empty snapshot.

## Launch examples

<!-- HH_260810 - Show the safe default and bounded shadow invocation without fabricating approval values. -->

Keep the physical pipeline unchanged:

```bash
ros2 launch autoware_vils_object_integration vils_object_integration.launch.py \
  mode:=real_only
```

Start a fail-closed shadow observer (it will not accept PC4 objects with the
installed parameter file):

```bash
ros2 launch autoware_vils_object_integration vils_object_integration.launch.py \
  mode:=shadow \
  contract_approved:=false \
  output_objects_topic:=/perception/pc2/vils/candidate_tracked_objects
```

An approved test profile must use a reviewed YAML file and an explicit
append-only evidence path. Canonical and required invocations should be issued
only by the upper PC2 `run_autoware` wiring after its tracker-output ownership
checks succeed.

## Verification scope

<!-- HH_260810 - Distinguish local contract tests from the blocked four-PC acceptance campaign. -->

Build and run the pure contract tests with:

```bash
colcon build --symlink-install \
  --packages-up-to autoware_vils_object_integration
colcon test --packages-select autoware_vils_object_integration
colcon test-result --verbose
```

The unit tests cover exact mode parsing, fail-closed limits, malformed payloads,
physical-priority overlap suppression, and unmatched-virtual append behavior.
They do not replace Stage 1.5/2/3 four-PC tests, PC4 real-ego replay evidence,
map/transform approval, clock qualification, PC1 RX-only status review, MRM
review, or real-vehicle safety validation.
