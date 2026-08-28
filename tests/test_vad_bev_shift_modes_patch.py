from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
VAD = ROOT / "src/universe/autoware_universe/e2e/autoware_tensorrt_vad"


def test_builds_apply_bev_modes_after_frame_state_contracts() -> None:
    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        frame = source.index("apply_vad_frame_assembly_patch.sh")
        causal = source.index("apply_vad_causal_state_sync_patch.sh")
        bev = source.index("apply_vad_bev_shift_modes_patch.sh")
        carla = source.index("apply_carla_fast_sensor_patch.sh")
        assert frame < causal < bev < carla


def test_patch_and_source_keep_legacy_default_with_three_modes() -> None:
    patch = (
        ROOT / "patches/autoware_tensorrt_vad_bev_shift_modes.patch"
    ).read_text(encoding="utf-8")
    source = (VAD / "lib/input_converter/bev_shift_converter.cpp").read_text(
        encoding="utf-8"
    )
    config = (VAD / "config/vad_carla_tiny.param.yaml").read_text(
        encoding="utf-8"
    )

    for marker in (
        'mode == "legacy"',
        'mode == "ros_corrected"',
        'mode == "zero"',
        "BEVShiftMode::RosCorrected",
        "return {forward / real_w_, left / real_h_};",
        "return {-left / real_w_, forward / real_h_};",
    ):
        assert marker in source
        assert marker in patch
    assert "bev_shift_mode: legacy" in config
    assert "bev_shift_mode: legacy" in patch


def test_cpp_contract_covers_three_headings_and_both_local_axes() -> None:
    test = (VAD / "test/test_bev_shift_contract.cpp").read_text(encoding="utf-8")
    assert "YawZeroPlusAndMinusNinety" in test
    assert "testing::Values(0.0f, kPi / 2.0f, -kPi / 2.0f)" in test
    assert "RosCorrectedMapsForwardAndLeftToOnnxXY" in test
    assert "LegacyPreservesRightForwardAxisOrder" in test
    assert "ZeroModeDisablesTranslationAtAnyHeading" in test


def test_fast_launch_chain_exposes_recordable_bev_mode_argument() -> None:
    launch_dir = ROOT / "autoware_e2e_vad_launch/launch"
    fast = ET.parse(launch_dir / "vad_carla_tiny_fast.launch.xml").getroot()
    fast_args = {node.get("name"): node for node in fast.findall("./arg")}
    assert fast_args["bev_shift_mode"].get("default") == "legacy"

    param = next(
        node
        for node in fast.iter("param")
        if node.get("name") == "interface_params.bev_shift_mode"
    )
    assert param.get("value") == "$(var bev_shift_mode)"

    for name in ("carla_vad.launch.xml", "carla_vad_full.launch.xml"):
        root = ET.parse(launch_dir / name).getroot()
        args = {node.get("name"): node for node in root.findall("./arg")}
        assert args["vad_bev_shift_mode"].get("default") == "legacy"
        include_arg = next(
            node
            for node in root.iter("arg")
            if node.get("name") == "bev_shift_mode"
        )
        assert include_arg.get("value") == "$(var vad_bev_shift_mode)"
