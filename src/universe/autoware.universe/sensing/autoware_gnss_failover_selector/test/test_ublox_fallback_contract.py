# HH_260811 - Lock the private fix remap and non-persistent u-blox safety contract.

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_ublox_private_fix_topic_is_remapped_to_fallback_output() -> None:
    root = ET.parse(PACKAGE_ROOT / "launch/ublox_fallback_driver.launch.xml").getroot()
    remaps = {(element.attrib["from"], element.attrib["to"]) for element in root.iter("remap")}

    assert ("~/fix", "$(var output_fix_topic)") in remaps
    assert not any(source == "fix" for source, _ in remaps)


def test_ublox_passive_config_never_requests_persistent_writes() -> None:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/ublox_fallback_no_persist.param.yaml").read_text()
    )["/**"]["ros__parameters"]

    assert config["config_on_startup"] is False
    assert config["load.mask"] == 0
    assert config["load.device"] == 0
    assert config["save.mask"] == 0
    assert config["save.device"] == 0
    assert config["clear_bbr"] is False
    assert config["save_on_shutdown"] is False
