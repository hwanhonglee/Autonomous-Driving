"""Load the external private, non-ROS NTRIP configuration file."""

# HH_260811 - Isolate caster credentials in one mode-0600 file under the user's config home.

from dataclasses import dataclass
import os
import stat
from typing import Any, Dict, Tuple

import yaml


DEFAULT_CONFIG_FILE = os.path.join(
    os.path.expanduser("~"), ".config", "pc3_ntrip", "ntrip_config.yaml"
)


class ConfigError(RuntimeError):
    """Configuration is absent or unsafe. Its text never includes secret values."""


@dataclass(frozen=True)
class NtripConfig:
    host: str
    port: int
    mountpoint: str
    username: str
    password: str
    protocol_version: str
    connect_timeout_sec: float
    read_timeout_sec: float
    reconnect_initial_sec: float
    reconnect_max_sec: float


@dataclass(frozen=True)
class SerialConfig:
    enabled: bool
    configure_novatel_rtcm_input: bool
    port: str
    baud: int
    reopen_interval_sec: float


@dataclass(frozen=True)
class RosConfig:
    rtcm_topics: Tuple[str, ...]
    diagnostics_topic: str
    frame_id: str
    diagnostic_period_sec: float


@dataclass(frozen=True)
class AppConfig:
    ntrip: NtripConfig
    serial: SerialConfig
    ros: RosConfig


def _mapping(value: Any, section: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"configuration section '{section}' must be a mapping")
    return value


def _required_text(data: Dict[str, Any], key: str, section: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"configuration field '{section}.{key}' must be non-empty text")
    if "\r" in value or "\n" in value:
        raise ConfigError(f"configuration field '{section}.{key}' contains invalid characters")
    return value.strip()


def _reject_placeholder(value: str, field: str) -> None:
    """Fail closed when a copied public template has not been customized."""
    normalized = value.strip().casefold()
    if normalized.startswith(("change_me", "replace_with_", "<")):
        raise ConfigError(f"configuration field '{field}' still contains a placeholder")


def _number(data: Dict[str, Any], key: str, section: str, minimum: float) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < minimum:
        raise ConfigError(f"configuration field '{section}.{key}' is out of range")
    return float(value)


def _integer(data: Dict[str, Any], key: str, section: str, minimum: int, maximum: int) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ConfigError(f"configuration field '{section}.{key}' is out of range")
    return value


def _read_private_yaml(path: str) -> Dict[str, Any]:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise ConfigError("private NTRIP configuration cannot be opened") from exc

    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode):
            raise ConfigError("private NTRIP configuration is not a regular file")
        if info.st_mode & 0o077:
            raise ConfigError("private NTRIP configuration permissions must be 0600")
        if info.st_size > 65536:
            raise ConfigError("private NTRIP configuration is unexpectedly large")
        with os.fdopen(fd, "r", encoding="utf-8") as stream:
            fd = -1
            loaded = yaml.safe_load(stream)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ConfigError("private NTRIP configuration cannot be parsed") from exc
    finally:
        if fd >= 0:
            os.close(fd)

    if not isinstance(loaded, dict):
        raise ConfigError("private NTRIP configuration must be a mapping")
    return loaded


def load_config(path: str) -> AppConfig:
    data = _read_private_yaml(path)
    ntrip = _mapping(data.get("ntrip"), "ntrip")
    serial = _mapping(data.get("serial"), "serial")
    ros = _mapping(data.get("ros"), "ros")

    protocol_version = _required_text(ntrip, "protocol_version", "ntrip")
    if protocol_version not in ("1.0", "2.0"):
        raise ConfigError("configuration field 'ntrip.protocol_version' must be 1.0 or 2.0")
    host = _required_text(ntrip, "host", "ntrip")
    mountpoint = _required_text(ntrip, "mountpoint", "ntrip").lstrip("/")
    username = _required_text(ntrip, "username", "ntrip")
    password = _required_text(ntrip, "password", "ntrip")
    if not mountpoint or any(char.isspace() for char in mountpoint):
        raise ConfigError("configuration field 'ntrip.mountpoint' is invalid")
    for value, field in (
        (host, "ntrip.host"),
        (mountpoint, "ntrip.mountpoint"),
        (username, "ntrip.username"),
        (password, "ntrip.password"),
    ):
        _reject_placeholder(value, field)

    enabled = serial.get("enabled")
    if not isinstance(enabled, bool):
        raise ConfigError("configuration field 'serial.enabled' must be true or false")
    configure_novatel = serial.get("configure_novatel_rtcm_input", False)
    if not isinstance(configure_novatel, bool):
        raise ConfigError(
            "configuration field 'serial.configure_novatel_rtcm_input' must be true or false"
        )
    serial_port = _required_text(serial, "port", "serial")
    if enabled:
        _reject_placeholder(serial_port, "serial.port")

    topic_value = ros.get("rtcm_topics", ros.get("rtcm_topic"))
    if isinstance(topic_value, str):
        topic_values = [topic_value]
    elif isinstance(topic_value, list):
        topic_values = topic_value
    else:
        raise ConfigError("configuration field 'ros.rtcm_topics' must be a list")
    if not topic_values or not all(
        isinstance(item, str) and item.strip() for item in topic_values
    ):
        raise ConfigError("configuration field 'ros.rtcm_topics' contains an invalid topic")
    normalized_topics = tuple(dict.fromkeys(item.strip() for item in topic_values))

    result = AppConfig(
        ntrip=NtripConfig(
            host=host,
            port=_integer(ntrip, "port", "ntrip", 1, 65535),
            mountpoint=mountpoint,
            username=username,
            password=password,
            protocol_version=protocol_version,
            connect_timeout_sec=_number(ntrip, "connect_timeout_sec", "ntrip", 0.1),
            read_timeout_sec=_number(ntrip, "read_timeout_sec", "ntrip", 0.1),
            reconnect_initial_sec=_number(ntrip, "reconnect_initial_sec", "ntrip", 0.1),
            reconnect_max_sec=_number(ntrip, "reconnect_max_sec", "ntrip", 0.1),
        ),
        serial=SerialConfig(
            enabled=enabled,
            configure_novatel_rtcm_input=configure_novatel,
            port=serial_port,
            baud=_integer(serial, "baud", "serial", 1200, 4000000),
            reopen_interval_sec=_number(serial, "reopen_interval_sec", "serial", 0.1),
        ),
        ros=RosConfig(
            rtcm_topics=normalized_topics,
            diagnostics_topic=_required_text(ros, "diagnostics_topic", "ros"),
            frame_id=_required_text(ros, "frame_id", "ros"),
            diagnostic_period_sec=_number(ros, "diagnostic_period_sec", "ros", 0.1),
        ),
    )
    if result.ntrip.reconnect_max_sec < result.ntrip.reconnect_initial_sec:
        raise ConfigError("ntrip reconnect maximum must not be less than its initial delay")
    return result
