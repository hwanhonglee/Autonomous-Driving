"""HH_261001 - V2 requires RGB switch but faithfully handles absent raw-target flag."""
import ast
import re
from scripts.e2e import collect_map_paired_rgb_20261001_v2 as capture


class Blueprint:
    def __init__(self,supported):
        self.values = dict(capture.ATTRIBUTES)
        if supported:
            self.values["enable_postprocess_effects"] = "true"
    def has_attribute(self,name):
        return name in self.values
    def set_attribute(self,name,value):
        assert name in self.values
        self.values[name] = value


def test_rgb_requires_and_sets_both_explicit_conditions():
    import pytest
    for value in ("false","true"):
        bp = Blueprint(True)
        contract = capture.configure_camera(bp,"rgb",value)
        assert contract["supported"] and contract["applied_value"] == value
        assert bp.values["enable_postprocess_effects"] == value
    with pytest.raises(ValueError,match="RGB must expose"):
        capture.configure_camera(Blueprint(False),"rgb","false")


def test_raw_targets_record_absence_without_pretending_false():
    for kind in ("depth","semantic_segmentation"):
        bp = Blueprint(False)
        contract = capture.configure_camera(bp,kind,"false")
        assert not contract["supported"] and contract["applied_value"] is None
        assert "enable_postprocess_effects" not in bp.values
        assert bp.values == capture.ATTRIBUTES


def test_raw_supported_flag_matches_historical_conditional_false():
    bp = Blueprint(True)
    contract = capture.configure_camera(bp,"depth","false")
    assert contract["supported"] and contract["applied_value"] == "false"
    assert bp.values["enable_postprocess_effects"] == "false"


def test_v2_owner_keeps_no_launch_default_and_parses_all_python():
    text = (capture.ROOT/"tools/e2e/run_owned_map_paired_rgb_20261001_v2.sh").read_text()
    assert "execute=0" in text and "--execute) execute=1" in text
    assert "worker_module=scripts.e2e.collect_map_paired_rgb_20261001_v2" in text
    for block in re.findall(r"<<'PY'\n(.*?)\nPY",text,re.DOTALL):
        ast.parse(block)


def test_v2_preserves_exact_route_and_sensor_count():
    from scripts.e2e import collect_map_paired_rgb_20261001 as old
    assert capture.ROUTES == old.ROUTES and capture.SENSOR_SPECS == old.SENSOR_SPECS
    assert capture.ATTRIBUTES == old.ATTRIBUTES
