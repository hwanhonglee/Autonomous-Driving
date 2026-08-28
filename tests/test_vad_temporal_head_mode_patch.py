from __future__ import annotations

import json
from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]
VAD = ROOT / "src/universe/autoware_universe/e2e/autoware_tensorrt_vad"
PATCH = ROOT / "patches/autoware_tensorrt_vad_temporal_head_mode.patch"
APPLY_SCRIPT = ROOT / "scripts/e2e/apply_vad_temporal_head_mode_patch.sh"
STATELESS_OVERLAY = (
    ROOT
    / "autoware_e2e_vad_launch/config/vad_carla_tiny_no_temporal_head.param.yaml"
)
RECOMMENDED_OVERLAY = (
    ROOT / "autoware_e2e_vad_launch/config/vad_carla_tiny_recommended.param.yaml"
)


def ros_parameters(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]


def test_default_is_temporal_and_stateless_overlay_preserves_recommended_sync() -> None:
    official = ros_parameters(VAD / "config/vad_carla_tiny.param.yaml")
    overlay = ros_parameters(STATELESS_OVERLAY)
    recommended = ros_parameters(RECOMMENDED_OVERLAY)

    assert official["model_params"]["use_temporal_head"] is True
    assert overlay["sync_params"] == recommended["sync_params"]
    assert overlay["model_params"] == {"use_temporal_head": False}

    schema = json.loads(
        (VAD / "schema/tensorrt_vad.schema.json").read_text(encoding="utf-8")
    )
    property_schema = schema["definitions"]["tensorrt_vad_node"]["properties"][
        "model_params"
    ]["properties"]["use_temporal_head"]
    assert property_schema["type"] == "boolean"
    assert property_schema["default"] is True


def test_stateless_model_path_never_touches_temporal_bev_state() -> None:
    config = (VAD / "src/vad_config.hpp").read_text(encoding="utf-8")
    node = (VAD / "src/vad_node.cpp").read_text(encoding="utf-8")
    model = (VAD / "src/vad_model.hpp").read_text(encoding="utf-8")
    policy = (VAD / "src/temporal_head_policy.hpp").read_text(encoding="utf-8")

    assert "bool use_temporal_head{true};" in config
    assert 'declare_parameter<bool>("model_params.use_temporal_head", true)' in node
    assert 'return {"head_no_prev", PrevBevAction::NONE};' in policy
    assert "if (!vad_config.use_temporal_head && engine.name == \"head\")" in model
    assert "std::lock_guard<std::mutex> lock(inference_mutex_);" in model
    assert "execution_plan.prev_bev_action == PrevBevAction::INITIALIZE" in model
    assert "execution_plan.prev_bev_action == PrevBevAction::ADVANCE" in model


def test_cpp_policy_covers_default_stateless_and_failure_semantics() -> None:
    test = (VAD / "test/test_temporal_head_policy.cpp").read_text(encoding="utf-8")

    assert "DefaultTemporalModeSeedsThenAdvancesPrevBev" in test
    assert "StatelessModeNeverInitializesOrAdvancesPrevBev" in test
    assert "FailedInferenceDoesNotConsumeFirstFrameSemantics" in test
    assert "PrevBevAction::NONE" in test


def test_patch_is_idempotent_and_ordered_before_carla_patches() -> None:
    assert PATCH.is_file()
    completed = subprocess.run(
        [str(APPLY_SCRIPT)], check=False, capture_output=True, text=True
    )
    assert completed.returncode == 0, completed.stderr
    assert "already applied" in completed.stdout

    for name in ("build.sh", "build_full.sh"):
        source = (ROOT / "scripts/e2e" / name).read_text(encoding="utf-8")
        bev = source.index("apply_vad_bev_shift_modes_patch.sh")
        temporal = source.index("apply_vad_temporal_head_mode_patch.sh")
        carla = source.index("apply_carla_fast_sensor_patch.sh")
        assert bev < temporal < carla

    syntax = subprocess.run(
        ["bash", "-n", str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr
