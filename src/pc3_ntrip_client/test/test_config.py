import os
from pathlib import Path

# HH_260811 - Verify private-file enforcement and RTCM topic fanout configuration.

from pc3_ntrip_client.config import ConfigError, DEFAULT_CONFIG_FILE, load_config
import pytest


CONFIG = """\
ntrip:
  host: 127.0.0.1
  port: 2101
  mountpoint: TEST
  username: test-user
  password: test-password
  protocol_version: "2.0"
  connect_timeout_sec: 1.0
  read_timeout_sec: 1.0
  reconnect_initial_sec: 0.1
  reconnect_max_sec: 1.0
serial:
  enabled: true
  configure_novatel_rtcm_input: true
  port: /dev/null
  baud: 115200
  reopen_interval_sec: 1.0
ros:
  rtcm_topics: [/sensing/gnss/novatel/rtcm, /rtcm]
  diagnostics_topic: /diagnostics
  frame_id: gnss_link
  diagnostic_period_sec: 1.0
"""


def test_load_private_config(tmp_path):
    path = tmp_path / "ntrip_config.yaml"
    path.write_text(CONFIG)
    os.chmod(path, 0o600)
    loaded = load_config(str(path))
    assert loaded.ntrip.mountpoint == "TEST"
    assert loaded.serial.configure_novatel_rtcm_input is True
    assert loaded.ros.rtcm_topics == ("/sensing/gnss/novatel/rtcm", "/rtcm")


def test_reject_readable_secret_file(tmp_path):
    path = tmp_path / "ntrip_config.yaml"
    path.write_text(CONFIG)
    os.chmod(path, 0o644)
    with pytest.raises(ConfigError, match="0600"):
        load_config(str(path))


def test_accept_legacy_single_topic_key(tmp_path):
    path = tmp_path / "ntrip_config.yaml"
    path.write_text(CONFIG.replace(
        "rtcm_topics: [/sensing/gnss/novatel/rtcm, /rtcm]",
        "rtcm_topic: /rtcm",
    ))
    os.chmod(path, 0o600)
    assert load_config(str(path)).ros.rtcm_topics == ("/rtcm",)


def test_default_config_is_external_to_the_package():
    # HH_260811 - Keep the runtime default in the user's private config directory.
    expected = Path.home() / ".config" / "pc3_ntrip" / "ntrip_config.yaml"
    assert Path(DEFAULT_CONFIG_FILE) == expected


def test_tracked_template_is_publishable_and_rejected_until_customized(tmp_path):
    # HH_260811 - Guard against committing credentials or running unchanged placeholders.
    template = Path(__file__).parents[1] / "config" / "ntrip_config.yaml"
    text = template.read_text(encoding="utf-8")
    assert "CHANGE_ME_NTRIP_USERNAME" in text
    assert "CHANGE_ME_NTRIP_PASSWORD" in text
    assert "@" not in text

    private_copy = tmp_path / "ntrip_config.yaml"
    private_copy.write_text(text, encoding="utf-8")
    os.chmod(private_copy, 0o600)
    with pytest.raises(ConfigError, match="placeholder"):
        load_config(str(private_copy))
