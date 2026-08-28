from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
UNIVERSE = ROOT / "src" / "universe" / "autoware_universe"
PACKAGE = UNIVERSE / "e2e" / "autoware_tensorrt_vad"
PATCH = ROOT / "patches" / "autoware_tensorrt_vad_fp16_layer_norm.patch"
APPLY_SCRIPT = ROOT / "scripts" / "e2e" / "apply_vad_fp16_layer_norm_patch.sh"
OVERLAY = (
    ROOT
    / "autoware_e2e_vad_launch"
    / "config"
    / "vad_carla_tiny_fast_fp16_heads.param.yaml"
)


def test_reproducible_patch_apply_script_is_idempotent() -> None:
    assert PATCH.is_file()
    completed = subprocess.run(
        [str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "already applied" in completed.stdout


def test_precision_policy_is_strict_and_scoped_before_setup() -> None:
    source = (PACKAGE / "lib" / "networks" / "net.cpp").read_text(encoding="utf-8")
    header = (PACKAGE / "src" / "networks" / "net.hpp").read_text(encoding="utf-8")

    policy_start = source.index("if (should_force_layer_norm_fp32")
    setup_start = source.index("trt_common->setup", policy_start)
    policy_block = source[policy_start:setup_start]

    assert 'precision == "fp16"' in source
    assert "NetworkType::HEAD || network_type == NetworkType::HEAD_NO_PREV" in source
    assert "NetworkType network_type" in header
    assert 'layer_name.find("/norm/")' in source
    assert 'layer_name.find("/norms.")' in source
    assert "LayerType::kNORMALIZATION" in source
    assert "setPrecision(nvinfer1::DataType::kFLOAT)" in source
    assert "setOutputType(output_index, nvinfer1::DataType::kFLOAT)" in source
    assert "clearFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS)" in policy_block
    assert "setFlag(nvinfer1::BuilderFlag::kOBEY_PRECISION_CONSTRAINTS)" in policy_block
    assert "forced_layers == 0" in policy_block


def test_apply_script_is_in_both_build_flows() -> None:
    expected = "scripts/e2e/apply_vad_fp16_layer_norm_patch.sh"
    for build_script in ("build.sh", "build_full.sh"):
        contents = (ROOT / "scripts" / "e2e" / build_script).read_text(encoding="utf-8")
        assert expected in contents

    completed = subprocess.run(
        ["bash", "-n", str(APPLY_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr


def test_fp16_overlay_uses_new_cache_paths_without_reusing_old_engines() -> None:
    contents = OVERLAY.read_text(encoding="utf-8")

    assert "vad-carla-tiny_head_fp16_layernorm_fp32.engine" in contents
    assert "vad-carla-tiny_head_no_prev_fp16_layernorm_fp32.engine" in contents
    assert contents.count('precision: "fp16"') == 2
    assert 'vad-carla-tiny_head_fp16.engine"' not in contents
    assert 'vad-carla-tiny_head_no_prev_fp16.engine"' not in contents
