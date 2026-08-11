# PC3 ROS 2 workspace v1.1 changelog

<!-- HH_260811 - Record the source-audited PC3 ROS 2 v1.1 release scope and rollback path. -->

Date: 2026-08-11
Maintainer: Hwanhong Lee `<hwanhong57@gmail.com>`

## Release identity and historical base

This change set is the PC3 `ros2_ws` half of the source-audited vehicle update.
It is based on the immutable historical branch and commit below:

```text
Repository: hwanhonglee/Autonomous-Driving
Base branch: h2_i/IONIQ_EV_307/PC3_spectra/ros2_ws/v1.0
Base commit: 39872d589c5f3d056d2fba6950e0bbca3bde3f32
Target branch: h2_i/IONIQ_EV_307/PC3_spectra/ros2_ws/v1.1
Target tag: IONIQ_EV_308_PC3_r
```

The independent GitHub branch archive used for comparison is:

```text
/home/a/Downloads/Autonomous-Driving-IONIQ_EV_307_PC3_r.zip
SHA-256: ea3f533e16e25c2675181bf2364c451e627739be62ddf0e8b0fe16e98c1e845f
```

The old `v1.0` branch and the existing `IONIQ_EV_307_PC3_r` tag must remain
unchanged. Version `v1.1` is a new sibling branch, not a rename or force update.
The new `IONIQ_EV_308_PC3_r` tag identifies the reviewed v1.1 ROS 2 workspace
commit and must not be moved after publication.

## Restored NovAtel source provenance

The historical `novatel_oem7_driver` source was recovered before making the v1.1
changes. The primary recovery archive is:

```text
/home/a/ros2_ws_original.zip
SHA-256: fb9ff84cf4f89f25c4790bfd3133c3ba48b901d357fb4d86778b256e90b3e923
```

Before the v1.1 edits, its normalized 137-file NovAtel subtree matched the
independent PC3 `v1.0` archive exactly. Both normalized trees had SHA-256:

```text
8b01b0d2997176e61fc52d0227a1c5d31619266499d79b92528b9292314bca64
```

The restored packages are:

- `novatel_oem7_msgs`, package version `20.6.0`;
- `novatel_oem7_driver`, package version `20.6.0`.

The vehicle-specific `std_msg_topics.yaml` mappings already present in the
historical PC3 branch were retained. In particular, `GPSFix`, `NavSatFix`, and
`INSSTDEV` use `gnss_link`. Files whose names contain `copy_org` were preserved
and were not used as active runtime configuration.

## New PC3 NTRIP correction package

Added `src/pc3_ntrip_client`, a ROS 2 Humble Python package that provides the
following behavior:

- loads caster, serial, and ROS settings from one external private YAML file;
- supports NTRIP v1 and v2 success responses, including legacy `ICY 200 OK` and
  HTTP chunked transfer coding;
- authenticates to the configured caster without exposing credentials as ROS
  parameters, diagnostics, exception text, or normal log output;
- reconnects with bounded exponential backoff after network or processing errors;
- incrementally frames RTCM3 data and rejects packets with invalid CRC-24Q;
- publishes validated frames as `rtcm_msgs/msg/Message` on
  `/sensing/gnss/novatel/rtcm`;
- publishes the same validated frames as `rtcm_msgs/msg/Message` on the absolute
  `/rtcm` topic expected by the preserved historical PC3 `ublox_gps` fallback;
- writes validated correction bytes to the dedicated NovAtel port0 stable device
  path while the OEM7 navigation driver continues to use port1;
- sends `INTERFACEMODE THISPORT AUTO NONE OFF` when enabled so that the correction
  port detects RTCM input in receiver RAM;
- never sends `SAVECONFIG`, so the receiver configuration is not persisted;
- publishes sanitized stream, CRC, reconnect, last-message-type, serial setup,
  and serial byte-count diagnostics;
- closes the active socket, joins the worker, and closes the serial descriptor on
  node shutdown.

The tracked `config/ntrip_config.yaml` is a publishable template with explicit
`CHANGE_ME` values. Bring-up notes record `www.gnssdata.or.kr:2101` and candidate
mountpoint `CNJU-RTCM32`, but operators must confirm these values with the service
provider and enter them only in the external private file. The current client supports
RTCM-only streams and does not send an NMEA GGA request.

### u-blox RTCM injection contract

HH_260811 - Restore the correction message contract after auditing the historical
PC3 source. Its `ublox_gps` node subscribes to absolute `/rtcm` as
`rtcm_msgs/msg/Message` and forwards the message bytes to `gps_->sendRtcm()`. The
apt Humble `UInt8MultiArray` raw-data-stream path is logging-oriented and is not an
equivalent receiver correction-injection path. The NTRIP client therefore publishes
`rtcm_msgs/msg/Message` on both configured topics; fallback acceptance must use the
restored historical u-blox source rather than assuming the apt binary implements it.

### Credential handling contract

HH_260811 - Separate release documentation from operational secrets. The repository
and installed package contain this placeholder-only configuration:

```text
src/pc3_ntrip_client/config/ntrip_config.yaml
```

The runtime default and the currently preserved PC3 credential file are external:

```text
/home/a/.config/pc3_ntrip/ntrip_config.yaml
```

Controls applied to this split are:

- tracked template values are explicit `CHANGE_ME` placeholders and contain no live
  account or vehicle serial data;
- tests require the tracked username/password placeholders to remain present and
  reject an unchanged copied template at runtime;
- `setup.py` installs only the tracked placeholder template for operator reference;
- the launch file, direct node default, and companion Autoware GNSS launch default to
  the external private path rather than the source or install tree;
- private directory mode is `0700` and private file mode is `0600`;
- runtime rejection of group/other permission bits, non-regular files, oversized
  files, unchanged placeholders, and symbolic-link traversal where `O_NOFOLLOW` is
  available;
- sanitized configuration and transport errors that omit paths, headers, and
  secret values.

Before committing, the tracked template must be inspected for placeholder integrity,
while `git ls-files --error-unmatch` must fail for the external live path. The live
file must not be copied into the source tree, Git index, artifact, log bundle, or
installed share path. Accidental `*.local.yaml` and `*.private.yaml` source-tree
variants remain ignored, but the canonical tracked template is intentionally visible
to Git so credential replacement there is detectable as a source modification.

The current caster connection uses NTRIP Basic authentication over the caster's
configured TCP endpoint; this package does not add TLS. Network access therefore
must remain limited to the trusted vehicle/test network appropriate for that
caster service.

## NovAtel OEM7 runtime corrections

### Single-antenna kinematic alignment threshold

`config/std_init_commands.yaml` now sends:

```text
SETALIGNMENTVEL 0.833333
```

This is 3 km/h in metres per second. It permits kinematic INS heading alignment
at the requested low test speed. It does not create a stationary absolute heading
and does not turn a single-antenna receiver into a dual-antenna system.

### Strict initialization response handling

`src/oem7_message_node.cpp` no longer accepts an arbitrary condition-variable
wakeup as successful command initialization. Binary receiver output can arrive
while the driver is waiting for an ASCII command response. A response now completes
the attempt only when it is exactly `OK`; any other response is logged without its
payload and retried under the existing retry limit.

### Duplicate INS message registration removal

`src/ins_handler.cpp` previously registered both `RAWIMUSX` and `INSPVAS` twice in
the same handler table. The duplicate entries were removed so that one receiver
message is handled once and cannot cause duplicate IMU/INS publication through
the duplicated registration path.

## Legacy file-replay test compatibility

The archived OEM7 `.gps` inputs and reference rosbag data retain their historical
`gps` frame IDs. The active PC3 output configuration uses `gnss_link`. The test
framework was updated without rewriting either historical artifact:

- each derived replay test explicitly lists only the topics whose active frame is
  expected to be `gnss_link`;
- the comparison first asserts the unit-under-test frame ID;
- only after that assertion is the legacy reference message normalized for field
  comparison;
- all other fields and all unlisted topics retain the original strict comparison.

This prevents compatibility normalization from hiding a runtime frame regression.
The explicit mappings cover BESTPOS `/fix`, BESTPOS `/gps`, and INS1 `/insstdev`.

## Package metadata

The new `pc3_ntrip_client` package records:

```text
Maintainer: Hwanhong Lee
Email: hwanhong57@gmail.com
License: Apache-2.0
Package version: 0.1.0
```

## Validation completed

The low-parallel PC3 workspace build completed for all four selected packages:

- `hesai_ros_driver`;
- `novatel_oem7_msgs`;
- `novatel_oem7_driver`;
- `pc3_ntrip_client`.

Build log:

```text
/home/a/ros2_ws/log/build_2026-08-11_18-30-46
```

That test result reported zero errors, failures, or skipped tests. At that stage the
NTRIP package had 11 passing pytest cases, including a local
fake-caster-to-pseudo-TTY end-to-end correction test. All six OEM7 file-replay
launch-test scenarios passed, including BESTPOS and INS1 frame migration checks.

Final OEM7 test logs:

```text
/home/a/ros2_ws/log/test_2026-08-11_18-42-37
```

After the maintainer metadata and release documentation audit, the NTRIP package
was rebuilt and its 11 tests were rerun successfully. At that audit stage, the live
configuration remained mode `0600` and was absent from the installed package.

```text
/home/a/ros2_ws/log/build_2026-08-11_19-03-30
/home/a/ros2_ws/log/test_2026-08-11_19-03-33
```

HH_260811 - After moving live credentials to the external user config, installing
the tracked placeholder, rejecting unchanged templates, and restoring the historical
`rtcm_msgs/msg/Message` fallback contract, the package was rebuilt again. All 13
tests passed with zero errors, failures, or skipped tests. The private file parsed
successfully without exposing its contents, retained mode `0600` under a mode `0700`
directory, and the installed share directory contained only the tracked placeholder.

```text
/home/a/ros2_ws/log/build_2026-08-11_19-27-47
/home/a/ros2_ws/log/test_2026-08-11_19-28-03
```

The tests are local and hardware-safe: they do not open the physical NovAtel serial
device, contact the production caster, or start the Hesai sensor. Receiving RTCM
bytes is not proof of RTK FIX. Live acceptance must separately confirm the native
OEM7 solution type/status, correction age, INS alignment, heading behavior, IMU
rate, exclusive serial ownership, and LiDAR data.

## Deliberate exclusions and unresolved live gates

- The historical `src/ublox` directory remains absent from this recreated live
  workspace but is preserved in the v1.1 Git tree. A fresh checkout must build and
  source that overlay for correction-assisted fallback: it owns the
  `rtcm_msgs/msg/Message` `/rtcm` subscriber that calls `Gps::sendRtcm()`. The apt
  Humble binary can provide ordinary position fallback but its `UInt8MultiArray`
  raw-data path is not an NTRIP correction-injection substitute. The preserved
  three-package overlay built successfully in an isolated checkout without opening
  hardware; live acceptance must also confirm `ros2 pkg prefix ublox_gps` resolves
  under `/home/a/ros2_ws/install`.
- Historical map paths and missing map data were not restored or modified.
- Hesai source and hardware settings were not changed by this ROS 2 v1.1 change set.
- No old NovAtel serial path, LiDAR IP address, antenna offset, or calibration value
  is claimed valid merely because it exists in an archived configuration.
- NTRIP account validity, mountpoint availability, RTK FIX, heading convergence,
  and correction injection still require a controlled outdoor hardware test.
- Single-antenna heading remains motion/INS-derived. A future dual-antenna receiver
  requires its own verified heading topic and extrinsic configuration.
- Generated `build`, `install`, `log`, `.pytest_cache`, and `__pycache__` content is
  not part of the source commit.
- The external `/home/a/.config/pc3_ntrip/ntrip_config.yaml` live credential file is
  operational state and is never part of the Git release or installed package.

### In-place worktree note

The v1.1 commit retains the historical map files and `src/ublox` from the v1.0
Git tree even though those paths are absent from the live recreated workspace.
After the commit, those absent paths and the runtime-only tracked `log.log`
difference are marked `skip-worktree` in the local index so that routine status
checks show only publishable source changes. This flag is local metadata and is
not pushed. Operators can audit it with `git ls-files -v | grep '^S'` and must
clear it with `git update-index --no-skip-worktree <path>` before deliberately
materializing or modifying one of those preserved historical paths.

## Rollback

The complete rollback is to stop PC3, switch back to the immutable
`h2_i/IONIQ_EV_307/PC3_spectra/ros2_ws/v1.0` branch, rebuild the selected packages,
and restart the vehicle stack in MANUAL/PARK.

For a correction-only rollback:

1. disable or stop `pc3_ntrip_client`;
2. verify that no process holds the NovAtel port0 correction device;
3. restart the OEM7 data driver only if its port1 ownership is clean;
4. use the companion Autoware launch argument to keep NTRIP disabled.

Because the NTRIP package does not send `SAVECONFIG`, its `INTERFACEMODE` setup is
RAM-only and is cleared by a receiver power cycle. Removing
`SETALIGNMENTVEL 0.833333` and rebuilding restores the historical alignment command
set. Do not roll back by copying `copy_org` files over active files; switch branches
or revert the named v1.1 commit so that provenance remains auditable.
