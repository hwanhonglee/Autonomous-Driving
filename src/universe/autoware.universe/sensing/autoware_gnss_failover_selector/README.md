# Autoware GNSS failover selector

HH_260811 - Added fail-closed NovAtel-primary selection, u-blox fallback, and INS orientation validation.

HH_260811 - Added diagnostic-only HEADING2 readiness for a future receiver-aligned dual antenna.

This package adds two fail-closed nodes without changing either receiver driver:

- `gnss_failover_selector` selects a valid, fresh NovAtel `NavSatFix` first and
  changes to u-blox only after the configured failure and activation holds. It
  returns to NovAtel only after a continuous recovery hold.
- `novatel_inspvax_orientation` publishes OEM7 `INSPVAX` as Autoware ENU
  `GnssInsOrientationStamped` only when the complete INS attitude is fresh,
  `INS_SOLUTION_GOOD`, finite, and within the configured RMSE bounds. It also
  validates `HEADING2` for future dual-antenna readiness diagnostics, but never
  converts that log directly into vehicle attitude.

The single-antenna `WAITING_AZIMUTH` state remains fail-closed for yaw. The
adapter never invents zero roll, substitutes antenna-baseline pitch for body
pitch, assumes the base-to-rover vector is vehicle forward, or accepts the
current 180-degree azimuth uncertainty. When a calibrated dual antenna is
installed, OEM7 ALIGN must feed SPAN so that `INSPVAX` itself reaches
`INS_SOLUTION_GOOD`; that existing output path then activates automatically.

The selector output is `selected/fix`, the latched position source state is
`gnss_failover_selector/selected_source`, and the orientation source/valid
states are `orientation/selected_source` and `orientation/valid`. Both nodes
report on `/diagnostics`. `orientation/heading2_valid` separately reports a
valid fixed HEADING2 observation; it does not mean vehicle yaw is calibrated.
The INS diagnostic also decodes `INSPVAX.ext_sol_status` alignment indication
and the live ALIGN-heading-update bit, so a future dual-antenna alignment can be
distinguished from static or kinematic alignment without changing the output
gate. Invalid and stale messages are never forwarded.

## Integration

HH_260812 - Document the best-available position path and source-bound INS attitude policy.

Launch both receiver drivers in separate topic namespaces, include
`gnss_redundancy.launch.xml` inside the existing `sensing/gnss` namespace, and
publish the selector result on `selected/fix`. The active PC3 sample launch feeds
that result to `autoware_gnss_poser`. Fresh validated NovAtel INS attitude is
used only when the selected fix frame is `gnss_link`; a u-blox `gps`-frame fix
can never be paired with stale or unrelated NovAtel attitude.

When validated primary attitude is unavailable, GNSS poser still publishes a
position seed. It starts with identity yaw (or holds the last validated yaw),
updates course heading only after 1 m of displacement, and reports 10 rad^2 yaw
variance. This gives NDT a best-effort initialization path without presenting
fallback yaw as measured attitude. NDT reliability remains the final acceptance
gate. The u-blox antenna transform is still unmeasured, so its position fallback
is degraded and must not be described as centimeter-level vehicle pose.

The active topic contract inside the `sensing/gnss` namespace is:

```text
novatel/oem7/fix       -> selector primary
ublox/nav_sat_fix      -> selector fallback
selected/fix           -> autoware_gnss_poser best-available position
novatel/oem7/inspvax   -> INSPVAX orientation adapter
novatel/oem7/heading2  -> diagnostic-only dual readiness monitor
/autoware_orientation  -> autoware_gnss_poser orientation
orientation/selected_source -> none or inspvax
orientation/valid      -> latched complete-orientation validity
orientation/heading2_valid -> latched HEADING2 solution validity only
```

OEM7 defines `HEADING2.heading` in degrees clockwise from true north along the
base-to-rover antenna vector, and the log has no roll field. Therefore, even a
valid HEADING2 is not treated as a calibrated vehicle attitude. The dual
antenna baseline direction, ALIGN configuration, receiver heading offset, and
vehicle/IMU rotations must be measured and reviewed during installation. This
package does not send `HEADINGOFFSET`, `SAVECONFIG`, or any other receiver
command. Its readiness gate accepts only `SOL_COMPUTED` (0) with `NARROW_INT`
(50), bounded heading/pitch standard deviations, and fresh stamps. See the
official [OEM7 HEADING2 log
definition](https://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm) and
[SPAN real-time status definitions](https://docs.novatel.com/OEM7/Content/SPAN_Operation/Real_Time_Operation.htm).

Do not rewrite the u-blox `frame_id` to `gnss_link` unless the u-blox antenna
lever arm has been measured and that frame is physically correct. The selected
fix preserves the source header, so the robot model needs a static transform
for each possible receiver frame.

For the first fallback test, apply `ublox_fallback_no_persist.param.yaml` to the
u-blox node. It uses the verified stable USB identity, disables startup
configuration, and explicitly sets all load/save masks to zero. It therefore
does not write receiver settings to non-volatile memory. This passive mode only
works if the receiver is already emitting the needed UBX messages. Enabling
`config_on_startup` later still requires a separately reviewed configuration;
never copy the stock `c94_m8p_rover.yaml` `save.mask: 3103` into a test launch.
The companion `ublox_fallback_driver.launch.xml` is disabled by default and
uses that passive configuration when explicitly enabled.

HH_260811 - Record the required PC3 u-blox overlay and its RTCM injection
contract so a fresh v1.1 checkout does not silently fall back to the
incompatible binary package.

PC3 must build and source the preserved `ros2_ws/src/ublox` overlay. That
driver owns an absolute `/rtcm` subscription of type `rtcm_msgs/msg/Message`
and forwards each message to `Gps::sendRtcm`; the NTRIP client publishes the
same contract. The stock Humble 2.3.0 binary package is not an equivalent RTK
fallback: it has no `rtcm_msgs` dependency, and its
`std_msgs/msg/UInt8MultiArray` raw-data-stream subscriber is a file-logging
path rather than a correction-injection path. After building, verify that
`ros2 pkg prefix ublox_gps` resolves under the PC3 `ros2_ws/install` overlay,
not `/opt/ros/humble`, before testing RTK.

The preserved driver publishes NavSatFix on its private `~/fix` topic, while
the Ubuntu Humble binary publishes the relative `fix` topic. The fallback
launch remaps both names to `ublox/nav_sat_fix`, so either installed driver
reaches the selector without changing its public output contract.

`NavSatFix.status == STATUS_GBAS_FIX` does not prove OEM7 RTK integer-fixed:
the restored OEM7 driver maps several differential, RTK-float, and RTK-fixed
solution types to that same status. Confirm RTK fixed from OEM7 `BESTPOS.pos_type`
and u-blox carrier-phase messages during acceptance testing.
