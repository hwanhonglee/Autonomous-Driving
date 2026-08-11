# PC3 NTRIP client

HH_260811 - Add private configuration loading, CRC-validated RTCM3 fanout, and
dependency-free NovAtel port0 injection.

This ROS 2 Humble package obtains RTCM3 corrections from an NTRIP v1/v2 caster,
validates every frame with CRC-24Q, publishes each validated frame as
`rtcm_msgs/msg/Message`, and optionally writes the same bytes to the dedicated
NovAtel OEM7 correction port. The canonical `/rtcm` fallback topic is also emitted
as `rtcm_msgs/msg/Message`, matching the preserved historical PC3 `ublox_gps`
subscriber that forwards the bytes to `gps_->sendRtcm()`.

No username or password is a ROS parameter. The node exposes only the path parameter
`config_file`, then reads all settings from an external mode-0600 YAML file. It never
logs configuration values, HTTP headers, or credentials.

HH_260811 - Separate the publishable configuration contract from live credentials.
The tracked `config/ntrip_config.yaml` contains only explicit `CHANGE_ME` placeholders
and is installed as documentation. The runtime default is the external private file
`~/.config/pc3_ntrip/ntrip_config.yaml`; the installed template is never used as the
runtime default.

## Configuration

Create a private copy from the tracked template, then edit only the private copy:

```bash
cd /home/a/ros2_ws/src/pc3_ntrip_client
install -d -m 700 /home/a/.config/pc3_ntrip
install -m 600 config/ntrip_config.yaml \
  /home/a/.config/pc3_ntrip/ntrip_config.yaml
${EDITOR:-vi} /home/a/.config/pc3_ntrip/ntrip_config.yaml
```

Replace every `CHANGE_ME` value and confirm the file remains mode `0600`. The schema
has three top-level mappings: `ntrip`, `serial`, and `ros`. Values that must be checked
for each deployment are:

- caster host, TCP port, mountpoint, username, and password supplied by the service;
- whether the selected mountpoint requires an NMEA GGA request (this client currently
  supports RTCM-only streams and does not send GGA);
- the stable `/dev/serial/by-id/...-port0` correction path and its exclusive ownership;
- baud rate, ROS topics, and `frame_id` for the installed receiver and vehicle.

During PC3 bring-up, `www.gnssdata.or.kr:2101` and candidate mountpoint
`CNJU-RTCM32` were recorded. They are operational inputs, not guaranteed defaults;
verify them with the NTRIP provider before entering them in the private file. The OEM7
data driver remains on NovAtel USB port1 while correction bytes go to the separate
stable port0 path. Valid frames publish to the NovAtel diagnostic/raw topic as
`rtcm_msgs/msg/Message` and absolute `/rtcm` as `rtcm_msgs/msg/Message` for the
historical PC3 `ublox_gps` correction-injection fallback.

Do not replace `/rtcm` with the apt Humble package's `UInt8MultiArray` raw-data path.
That path is logging-oriented and does not provide the historical `sendRtcm()`
receiver-injection behavior required by this fallback design.

The loader fails closed when the file is missing, not a regular file, larger than
64 KiB, has group/other permission bits, or still contains tracked placeholders.
Never put real credentials into `src/pc3_ntrip_client/config/ntrip_config.yaml`, a
Git commit, a shell command line, ROS parameters, or a log. The currently preserved
PC3 private file is `/home/a/.config/pc3_ntrip/ntrip_config.yaml` with mode `0600`.

When `serial.configure_novatel_rtcm_input` is true, opening port0 first sends
`INTERFACEMODE THISPORT AUTO NONE OFF` and then starts the RTCM stream. This is a
receiver-RAM-only setup; the package never sends a persistent receiver-save command.
Setup attempts/successes are reported separately, and setup bytes are excluded from
the correction-byte counter.

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/a/ros2_ws/install/setup.bash
ros2 launch pc3_ntrip_client ntrip_client.launch.py
```

The launch default already resolves to `~/.config/pc3_ntrip/ntrip_config.yaml`. An
alternate external private file is the only supported launch override:

```bash
ros2 launch pc3_ntrip_client ntrip_client.launch.py \
  config_file:=/absolute/private/ntrip_config.yaml
```

The equivalent direct-node override is:

```bash
ros2 run pc3_ntrip_client ntrip_client_node --ros-args \
  -p config_file:=/absolute/private/ntrip_config.yaml
```

RTK FIX is confirmed from the NovAtel solution status after the receiver consumes
corrections; receipt of RTCM alone does not prove an RTK-fixed navigation solution.
